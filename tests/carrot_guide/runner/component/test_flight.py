"""The runner closed around a fake vehicle: guidance, transport and logging together.

This is the closest thing to an integration test that needs neither a socket nor a
simulator. The fake vehicle integrates whatever velocity it is commanded, so the test
covers the wiring — frame conversion, command dispatch, sample recording — and fails
loudly if the runner ever feeds a law the wrong frame.
"""

import pytest

from carrot_guide.guidance import HoldPoint, Limits, Pursuit, Target
from carrot_guide.recording import MemorySink
from carrot_guide.runner import FixedRateLoop, GuidanceRunner, StaleTelemetry
from carrot_guide.state import NED
from carrot_guide.telemetry import TelemetryTracker

from tests.doubles.clocks import FakeClock
from tests.doubles.vehicles import ORIGIN, FakeVehicle


def _runner_on_a_fake_clock(
    vehicle: "FakeVehicle", tracker: TelemetryTracker, hz: float
) -> tuple[GuidanceRunner, FakeClock]:
    """A runner whose loop is driven by a clock the caller can inspect.

    Handing the clock back is the point: a test that asserts on a clock the runner does
    not actually use asserts nothing, and one here did exactly that.
    """
    clock = FakeClock()
    runner = GuidanceRunner(
        link=vehicle,
        tracker=tracker,
        rate_hz=hz,
        loop_factory=lambda rate: FixedRateLoop(
            rate, monotonic=clock.monotonic, sleep=clock.sleep, spin_slack_s=0.0
        ),
    )
    return runner, clock


def build(
    start: NED, hz: float = 10.0, goes_silent_after: int | None = None
) -> tuple[GuidanceRunner, FakeVehicle]:
    """A runner over a fake vehicle. Tests that need the clock use the helper directly."""
    tracker = TelemetryTracker()
    tracker.set_origin(ORIGIN)
    vehicle = FakeVehicle(tracker, dt=1.0 / hz, start=start, goes_silent_after=goes_silent_after)
    runner, _ = _runner_on_a_fake_clock(vehicle, tracker, hz)
    return runner, vehicle


@pytest.mark.component
def test_the_runner_flies_the_law_to_the_target():
    runner, _ = build(start=NED(0.0, 0.0, -20.0))
    law = HoldPoint(target=NED(30.0, 10.0, -20.0), limits=Limits(max_horizontal_speed=5.0))
    report = runner.fly(law, duration_s=60.0, label="hold")
    assert report.samples[-1].error_m < 0.1


@pytest.mark.component
def test_every_cycle_is_recorded_once():
    runner, vehicle = build(start=NED(0.0, 0.0, -20.0))
    sink = MemorySink()
    report = runner.fly(HoldPoint(target=NED(5.0, 0.0, -20.0)), 3.0, sink=sink)
    # 3 s at 10 Hz, plus the tick at t = 0; one command per cycle, plus the stop command.
    assert len(report.samples) == 31
    assert len(sink.samples) == 31
    assert len(vehicle.commands) == 32


@pytest.mark.component
def test_streaming_mode_keeps_nothing_in_memory():
    runner, _ = build(start=NED(0.0, 0.0, -20.0))
    runner.retain_samples = False
    sink = MemorySink()
    report = runner.fly(HoldPoint(target=NED(5.0, 0.0, -20.0)), 3.0, sink=sink)
    assert report.samples == []
    assert len(sink.samples) == 31
    # The tick count is the row count: a summary and its log have to reconcile.
    assert report.loop.ticks == 31


@pytest.mark.component
def test_the_run_ends_by_commanding_a_stop():
    runner, vehicle = build(start=NED(0.0, 0.0, -20.0))
    runner.fly(HoldPoint(target=NED(50.0, 0.0, -20.0)), 1.0)
    last_velocity, _ = vehicle.commands[-1]
    assert last_velocity == NED(0.0, 0.0, 0.0)


@pytest.mark.component
def test_samples_carry_the_local_frame_and_the_commands():
    runner, _ = build(start=NED(0.0, 0.0, -20.0))
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


@pytest.mark.component
def test_the_local_frame_survives_the_round_trip_through_wgs84():
    runner, _ = build(start=NED(-40.0, 75.0, -20.0))
    sink = MemorySink()
    runner.fly(HoldPoint(target=NED(-40.0, 75.0, -20.0)), 1.0, sink=sink)
    first = sink.samples[0]
    assert first.north_m == pytest.approx(-40.0, abs=1e-6)
    assert first.east_m == pytest.approx(75.0, abs=1e-6)
    assert first.error_m == pytest.approx(0.0, abs=1e-6)


@pytest.mark.component
def test_flying_without_an_anchored_frame_is_refused():
    tracker = TelemetryTracker()
    vehicle = FakeVehicle(tracker, dt=0.1, start=NED(0.0, 0.0, 0.0))
    runner, clock = _runner_on_a_fake_clock(vehicle, tracker, hz=10.0)
    started_at = clock.now

    with pytest.raises(RuntimeError):
        runner.fly(HoldPoint(target=NED(0.0, 0.0)), 1.0)

    # Refused before the loop was built: no tick was scheduled and nothing was commanded.
    # The clock has to be the one the runner uses, or this asserts nothing at all.
    assert clock.now == started_at
    assert vehicle.commands == []


@pytest.mark.component
def test_a_dead_position_stream_stops_the_loop_rather_than_flying_blind():
    # After five cycles the vehicle stops reporting where it is.
    runner, vehicle = build(start=NED(0.0, 0.0, -20.0), goes_silent_after=5)
    law = HoldPoint(target=NED(50.0, 0.0, -20.0))

    with pytest.raises(StaleTelemetry):
        runner.fly(law, duration_s=30.0)

    # And it braked on the way out instead of leaving the last command standing.
    assert vehicle.commands[-1][0] == NED(0.0, 0.0, 0.0)
    assert len(vehicle.commands) < 30  # nowhere near the 300 cycles it was asked for


@pytest.mark.component
def test_loop_statistics_come_back_with_the_run():
    runner, _ = build(start=NED(0.0, 0.0, -20.0), hz=20.0)
    report = runner.fly(HoldPoint(target=NED(5.0, 0.0, -20.0)), 2.0)
    assert report.loop.target_hz == 20.0
    assert report.loop.ticks == 41  # 2 s at 20 Hz, plus the tick at t = 0
    assert report.loop.mean_period_s == pytest.approx(0.05)


@pytest.mark.component
def test_a_law_aimed_at_a_moving_target_is_handed_the_run_clock():
    """The loop owns the only clock, so a moving target is wrong without this wiring."""
    runner, _ = build(start=NED(0.0, 0.0, -20.0))
    target = Target(start=NED(40.0, 0.0, -20.0), velocity=NED(0.0, 4.0))
    sink = MemorySink()
    runner.fly(Pursuit(target=target, speed_mps=5.0), duration_s=2.0, sink=sink)

    # The target is due north at t = 0, so nothing of the first command points east.
    assert sink.samples[0].cmd_ve == pytest.approx(0.0, abs=1e-9)
    # Two seconds on it is 8 m east of where it started and the vehicle is going after
    # it. A law handed t = 0 on every tick would still be flying due north.
    assert sink.samples[-1].cmd_ve > 1.0
