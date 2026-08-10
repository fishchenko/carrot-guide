"""The runner closed around a fake vehicle: guidance, transport and logging together.

This is the closest thing to an integration test that needs neither a socket nor a
simulator. The fake vehicle integrates whatever velocity it is commanded, so the test
covers the wiring — frame conversion, command dispatch, sample recording — and fails
loudly if the runner ever feeds a law the wrong frame.
"""

import pytest

from carrot_guide.guidance import HoldPoint, Limits
from carrot_guide.recording import MemorySink
from carrot_guide.runner import FixedRateLoop, GuidanceRunner, StaleTelemetry
from carrot_guide.state import NED, GlobalPosition, from_local_ned
from carrot_guide.telemetry import TelemetryTracker

ORIGIN = GlobalPosition(50.4501, 30.5234, 0.0)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(0.0, seconds)


class FakeVehicle:
    """Stands in for `MavlinkLink`, integrating commands into the tracker's state."""

    def __init__(self, tracker: TelemetryTracker, dt: float, start: NED) -> None:
        self.tracker = tracker
        self.dt = dt
        self.position = start
        self.commands: list[tuple[NED, float | None]] = []
        self._publish(NED(0.0, 0.0, 0.0))

    silent = False  # set by the failsafe test to simulate a dead position stream

    def _publish(self, velocity: NED) -> None:
        if self.silent:
            return
        self.tracker.position = from_local_ned(self.position, ORIGIN)
        self.tracker.velocity = velocity
        self.tracker.armed = True
        self.tracker.mode = "GUIDED"
        self.tracker.position_updates += 1

    def drain(self, tracker: TelemetryTracker, budget_s: float = 0.0) -> int:
        return 1

    def send_velocity(self, velocity: NED, yaw_deg: float | None = None) -> None:
        self.commands.append((velocity, yaw_deg))
        self.position = self.position + velocity.scaled(self.dt)
        self._publish(velocity)


def build(start: NED, hz: float = 10.0) -> tuple[GuidanceRunner, FakeVehicle, TelemetryTracker]:
    clock = FakeClock()
    tracker = TelemetryTracker()
    tracker.set_origin(ORIGIN)
    vehicle = FakeVehicle(tracker, dt=1.0 / hz, start=start)
    runner = GuidanceRunner(
        link=vehicle,
        tracker=tracker,
        rate_hz=hz,
        loop_factory=lambda rate: FixedRateLoop(
            rate, monotonic=clock.monotonic, sleep=clock.sleep, spin_slack_s=0.0
        ),
    )
    return runner, vehicle, tracker


def test_the_runner_flies_the_law_to_the_target():
    runner, _, _ = build(start=NED(0.0, 0.0, -20.0))
    law = HoldPoint(target=NED(30.0, 10.0, -20.0), limits=Limits(max_horizontal_speed=5.0))
    report = runner.fly(law, duration_s=60.0, label="hold")
    assert report.samples[-1].error_m < 0.1


def test_every_cycle_is_recorded_once():
    runner, vehicle, _ = build(start=NED(0.0, 0.0, -20.0))
    sink = MemorySink()
    report = runner.fly(HoldPoint(target=NED(5.0, 0.0, -20.0)), 3.0, sink=sink)
    # 3 s at 10 Hz, plus the tick at t = 0; one command per cycle, plus the stop command.
    assert len(report.samples) == 31
    assert len(sink.samples) == 31
    assert len(vehicle.commands) == 32


def test_streaming_mode_keeps_nothing_in_memory():
    runner, _, _ = build(start=NED(0.0, 0.0, -20.0))
    runner.retain_samples = False
    sink = MemorySink()
    report = runner.fly(HoldPoint(target=NED(5.0, 0.0, -20.0)), 3.0, sink=sink)
    assert report.samples == []
    assert len(sink.samples) == 31
    assert report.loop.ticks == 30  # the statistics are unaffected


def test_the_run_ends_by_commanding_a_stop():
    runner, vehicle, _ = build(start=NED(0.0, 0.0, -20.0))
    runner.fly(HoldPoint(target=NED(50.0, 0.0, -20.0)), 1.0)
    last_velocity, _ = vehicle.commands[-1]
    assert last_velocity == NED(0.0, 0.0, 0.0)


def test_samples_carry_the_local_frame_and_the_commands():
    runner, _, _ = build(start=NED(0.0, 0.0, -20.0))
    sink = MemorySink()
    runner.fly(HoldPoint(target=NED(20.0, 0.0, -20.0)), 1.0, sink=sink, label="hold")
    first = sink.samples[0]
    assert first.label == "hold"
    assert first.north_m == pytest.approx(0.0, abs=1e-6)
    assert first.down_m == pytest.approx(-20.0, abs=1e-6)
    assert first.cmd_vn > 0.0
    assert first.error_m == pytest.approx(20.0, abs=1e-6)
    assert first.mode == "GUIDED"
    assert first.armed is True


def test_the_local_frame_survives_the_round_trip_through_wgs84():
    runner, _, _ = build(start=NED(-40.0, 75.0, -20.0))
    sink = MemorySink()
    runner.fly(HoldPoint(target=NED(-40.0, 75.0, -20.0)), 1.0, sink=sink)
    first = sink.samples[0]
    assert first.north_m == pytest.approx(-40.0, abs=1e-6)
    assert first.east_m == pytest.approx(75.0, abs=1e-6)
    assert first.error_m == pytest.approx(0.0, abs=1e-6)


def test_flying_without_an_anchored_frame_is_refused():
    clock = FakeClock()
    tracker = TelemetryTracker()
    vehicle = FakeVehicle(tracker, dt=0.1, start=NED(0.0, 0.0, 0.0))
    runner = GuidanceRunner(link=vehicle, tracker=tracker, rate_hz=10.0)
    with pytest.raises(RuntimeError):
        runner.fly(HoldPoint(target=NED(0.0, 0.0)), 1.0)
    assert clock.now == 0.0


def test_a_dead_position_stream_stops_the_loop_rather_than_flying_blind():
    runner, vehicle, _ = build(start=NED(0.0, 0.0, -20.0))
    law = HoldPoint(target=NED(50.0, 0.0, -20.0))

    # After five cycles the vehicle stops reporting where it is.
    original = vehicle.send_velocity

    def send_and_maybe_go_quiet(velocity: NED, yaw_deg: float | None = None) -> None:
        original(velocity, yaw_deg)
        if len(vehicle.commands) == 5:
            vehicle.silent = True

    vehicle.send_velocity = send_and_maybe_go_quiet  # type: ignore[method-assign]

    with pytest.raises(StaleTelemetry):
        runner.fly(law, duration_s=30.0)

    # And it braked on the way out instead of leaving the last command standing.
    assert vehicle.commands[-1][0] == NED(0.0, 0.0, 0.0)
    assert len(vehicle.commands) < 30  # nowhere near the 300 cycles it was asked for


def test_loop_statistics_come_back_with_the_run():
    runner, _, _ = build(start=NED(0.0, 0.0, -20.0), hz=20.0)
    report = runner.fly(HoldPoint(target=NED(5.0, 0.0, -20.0)), 2.0)
    assert report.loop.target_hz == 20.0
    assert report.loop.ticks == 40
    assert report.loop.mean_period_s == pytest.approx(0.05)
