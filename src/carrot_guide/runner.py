"""The control loop: a fixed-rate scheduler and the guidance runner built on it.

The scheduler takes its clock and its sleep as parameters, so the timing behaviour —
drift, catch-up, jitter accounting — is tested with a fake clock in microseconds
instead of by watching a real loop for a minute.
"""

from __future__ import annotations

import statistics
import time
from dataclasses import dataclass, field
from typing import Callable, Iterator, Protocol

from carrot_guide.guidance import VelocityCommand
from carrot_guide.recording import Sample, SampleSink
from carrot_guide.state import NED, VehicleState, to_local_ned
from carrot_guide.telemetry import TelemetryTracker
from carrot_guide.utils import percentile


class GuidanceLaw(Protocol):
    """What the runner needs from a law; see `guidance` for the implementations."""

    def command(self, position: NED, velocity: NED) -> VelocityCommand: ...

    def tracking_error(self, position: NED) -> float: ...


class VehicleLink(Protocol):
    """What the runner needs from the transport; see `link.MavlinkLink` for the one.

    Stated as a Protocol for the same reason `GuidanceLaw` is: the loop is closed over
    a fake vehicle in the tests, and naming the concrete `MavlinkLink` here would both
    force those tests to lie about the type and drag pymavlink into every import of
    the control loop.
    """

    def drain(self, tracker: TelemetryTracker, budget_s: float = 0.0) -> int: ...

    def send_velocity(self, velocity: NED, yaw_deg: float | None = None) -> None: ...


@dataclass(frozen=True)
class Tick:
    """One iteration of the loop."""

    index: int
    elapsed_s: float
    lateness_s: float


@dataclass
class LoopStats:
    """How well the loop actually held its period."""

    target_hz: float
    ticks: int
    mean_period_s: float
    jitter_stdev_s: float
    max_lateness_s: float
    p95_lateness_s: float
    spin_slack_s: float = 0.0
    resyncs: int = 0
    skipped_cycles: int = 0

    def as_dict(self) -> dict[str, float]:
        return {
            "target_hz": self.target_hz,
            "ticks": float(self.ticks),
            "mean_hz": round(1.0 / self.mean_period_s, 4) if self.mean_period_s else 0.0,
            "jitter_stdev_ms": round(self.jitter_stdev_s * 1000.0, 3),
            "max_lateness_ms": round(self.max_lateness_s * 1000.0, 3),
            "p95_lateness_ms": round(self.p95_lateness_s * 1000.0, 3),
            "spin_slack_ms": round(self.spin_slack_s * 1000.0, 3),
            # Lateness alone cannot describe a stall: see `ticks()` for why a resync
            # reports as zero lateness, and why these two have to be quoted with it.
            "resyncs": float(self.resyncs),
            "skipped_cycles": float(self.skipped_cycles),
        }


def measure_sleep_overshoot(
    request_s: float,
    samples: int = 20,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> float:
    """Worst overshoot of a `sleep(request_s)` on this host, in seconds.

    `request_s` has no default on purpose: the answer depends on it strongly enough
    that any default would be a wrong answer waiting to be used (see below).

    A desktop OS wakes a sleeping thread on its own timer grid, and on Darwin the
    slack it allows itself grows with the requested interval — a 1 ms sleep is accurate
    to a fraction of a millisecond while a 90 ms one lands ~9 ms late. So the
    calibration has to sleep for roughly as long as the loop will, not for some token
    interval. Measuring this instead of assuming it is what lets the same code hold a
    clean period on a laptop and on a small ARM board.
    """
    worst = 0.0
    for _ in range(samples):
        before = monotonic()
        sleep(request_s)
        worst = max(worst, monotonic() - before - request_s)
    return worst


@dataclass
class FixedRateLoop:
    """Deadline-driven scheduler: deadlines come from the tick index, so it does not drift.

    A naive `sleep(period)` loop loses the time the body takes on every iteration and
    ends up running measurably slow; scheduling against absolute deadlines keeps the
    average rate honest and turns the body's cost into lateness, which is measured.
    """

    hz: float
    monotonic: Callable[[], float] = time.monotonic
    sleep: Callable[[float], None] = time.sleep
    # `sleep` on a general-purpose OS overshoots by milliseconds, which at 10 Hz shows
    # up as a steady lateness of a whole percent of the period. Sleeping to just short
    # of the deadline and spinning the rest cuts that down; set to zero to sleep the
    # whole way (and in tests, where a fake clock would never advance while spinning).
    spin_slack_s: float = 0.002
    lateness: list[float] = field(default_factory=list)
    periods: list[float] = field(default_factory=list)
    # Ticks actually served. Counted rather than taken from `len(periods)`, which holds the
    # intervals *between* ticks and is one short: reported as a tick count it put 5 999 in the
    # README for a ten-minute run at 10 Hz, next to the 6 000 rows the same run wrote.
    served: int = 0
    # How often the schedule had to be abandoned and re-based, and how many cycles that
    # cost. Kept because `lateness` structurally cannot see these events.
    resyncs: int = 0
    skipped_cycles: int = 0

    # Never give up more than this share of the period to spinning: a loop that burns
    # a fifth of its budget waiting is worse than one that admits its jitter.
    MAX_SPIN_SHARE = 0.2

    @classmethod
    def calibrated(cls, hz: float, margin: float = 1.2, samples: int = 5) -> "FixedRateLoop":
        """Build a loop whose spin slack is sized to this host's sleep granularity.

        Costs `samples` periods up front — half a second at 10 Hz — which is cheap
        against a run measured in minutes, and means the reported jitter describes the
        guidance loop rather than the host's timer.
        """
        period = 1.0 / hz
        overshoot = measure_sleep_overshoot(samples=samples, request_s=period * 0.9)
        slack = min(period * cls.MAX_SPIN_SHARE, overshoot * margin + 0.0005)
        return cls(hz=hz, spin_slack_s=slack)

    @property
    def period_s(self) -> float:
        return 1.0 / self.hz

    def ticks(self, duration_s: float) -> Iterator[Tick]:
        period = self.period_s
        start = self.monotonic()
        # Deadlines are computed from the tick index rather than accumulated one period
        # at a time: adding 0.1 a thousand times is visibly not the same number as
        # multiplying it by a thousand, and the difference lands on the schedule.
        origin = start
        previous = start
        index = 0
        while True:
            deadline = origin + index * period
            now = self.monotonic()
            if now < deadline:
                sleep_until = deadline - self.spin_slack_s
                if now < sleep_until:
                    self.sleep(sleep_until - now)
                # A genuine spin: `sleep(0)` here would be worse than useless, because
                # it parks the thread until the host's next timer slice — the very
                # granularity this is meant to sidestep.
                while self.monotonic() < deadline:
                    pass
                now = self.monotonic()
            elapsed = now - start
            if elapsed > duration_s:
                return
            late = now - deadline
            if index:
                self.lateness.append(late)
                self.periods.append(now - previous)
            self.served += 1
            yield Tick(index=index, elapsed_s=elapsed, lateness_s=late)
            previous = now
            index += 1
            # A body that overruns badly (a blocked socket, a stalled simulator) would
            # otherwise leave a backlog of instantly-due deadlines. Give up on the lost
            # cycles and shift the schedule instead of firing a burst back to back.
            #
            # Re-basing the schedule also moves the deadline the *next* tick measures
            # its lateness against, so the stall that caused it reports as exactly zero
            # lateness — the metric is blind to precisely the excursion it looks like it
            # is watching for. Only overruns small enough not to trigger a resync ever
            # show up there. Hence the counters: a run may quote a sub-millisecond
            # worst-case lateness and still have dropped cycles, and both have to be on
            # the record for either to mean anything.
            after_body = self.monotonic()
            next_deadline = origin + index * period
            if after_body > next_deadline + period:
                self.resyncs += 1
                # Whole deadlines passed over and never served. The tick about to run
                # is served late rather than skipped, so it is not counted here; the
                # epsilon keeps a stall that lands exactly on a period boundary from
                # rounding down to one fewer.
                self.skipped_cycles += int((after_body - next_deadline) / period + 1e-9)
                origin = after_body - index * period

    def stats(self) -> LoopStats:
        return LoopStats(
            target_hz=self.hz,
            ticks=self.served,
            mean_period_s=statistics.fmean(self.periods) if self.periods else 0.0,
            jitter_stdev_s=statistics.pstdev(self.periods) if len(self.periods) > 1 else 0.0,
            max_lateness_s=max(self.lateness, default=0.0),
            p95_lateness_s=percentile(self.lateness, 0.95),
            spin_slack_s=self.spin_slack_s,
            resyncs=self.resyncs,
            skipped_cycles=self.skipped_cycles,
        )


class StaleTelemetry(RuntimeError):
    """The position stream stopped while the loop was still commanding."""


@dataclass
class RunReport:
    """Everything one guidance run produced."""

    samples: list[Sample]
    loop: LoopStats


@dataclass
class GuidanceRunner:
    """Drives a guidance law against a live vehicle at a fixed rate."""

    link: VehicleLink
    tracker: TelemetryTracker
    rate_hz: float = 10.0
    # How long the loop may keep commanding on a picture of the world that has stopped
    # being refreshed. Without this it would happily fly a dead reckoning of its own
    # invention for the whole requested duration.
    stale_after_s: float = 1.0
    # Keeping every cycle in memory is what the summary is computed from, and it costs
    # ~700 B per cycle — visible as a few megabytes over a ten-minute run, and unbounded
    # over an indefinite one. Turn it off and the CSV sink is the only record.
    retain_samples: bool = True
    # Injected so the loop can be driven by a fake clock in tests; in flight it is the
    # calibrated one, which is the only place the host's timer behaviour is dealt with.
    loop_factory: Callable[[float], FixedRateLoop] = FixedRateLoop.calibrated

    def fly(
        self,
        law: GuidanceLaw,
        duration_s: float,
        sink: SampleSink | None = None,
        label: str = "",
    ) -> RunReport:
        if self.tracker.origin is None:
            raise RuntimeError("local frame is not anchored; call tracker.set_origin() first")

        origin = self.tracker.origin
        loop = self.loop_factory(self.rate_hz)
        samples: list[Sample] = []

        seen_updates = self.tracker.position_updates
        fresh_at = 0.0

        for tick in loop.ticks(duration_s):
            self.link.drain(self.tracker)

            if self.tracker.position_updates != seen_updates:
                seen_updates = self.tracker.position_updates
                fresh_at = tick.elapsed_s
            elif tick.elapsed_s - fresh_at > self.stale_after_s:
                self.link.send_velocity(NED(0.0, 0.0, 0.0))
                raise StaleTelemetry(
                    f"no position report for {tick.elapsed_s - fresh_at:.2f} s; stopped commanding"
                )

            state: VehicleState = self.tracker.snapshot()
            position = to_local_ned(state.position, origin)
            command = law.command(position, state.velocity)
            self.link.send_velocity(command.velocity, command.yaw_deg)

            sample = Sample(
                t_s=tick.elapsed_s,
                label=label,
                lat_deg=state.position.lat_deg,
                lon_deg=state.position.lon_deg,
                north_m=position.north,
                east_m=position.east,
                down_m=position.down,
                vn=state.velocity.north,
                ve=state.velocity.east,
                vd=state.velocity.down,
                cmd_vn=command.velocity.north,
                cmd_ve=command.velocity.east,
                cmd_vd=command.velocity.down,
                error_m=law.tracking_error(position),
                lateness_ms=tick.lateness_s * 1000.0,
                mode=state.mode,
                armed=state.armed,
            )
            if self.retain_samples:
                samples.append(sample)
            if sink is not None:
                sink.write(sample)

        # Stop commanding: ArduPilot brakes when velocity targets stop arriving.
        self.link.send_velocity(NED(0.0, 0.0, 0.0))
        return RunReport(samples=samples, loop=loop.stats())
