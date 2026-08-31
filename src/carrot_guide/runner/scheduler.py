from __future__ import annotations

import statistics
import time
from dataclasses import dataclass, field
from typing import Callable, Iterator

from carrot_guide.runner.stats import LoopStats, Tick
from carrot_guide.utils import percentile


def measure_sleep_overshoot(
    request_s: float,
    samples: int = 20,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> float:
    """Worst overshoot of a `sleep(request_s)` here, in seconds. Darwin's slack grows with the
    interval — under 1 ms at 1 ms, ~9 ms at 90 ms — so request close to the loop's real period."""
    worst = 0.0
    for _ in range(samples):
        before = monotonic()
        sleep(request_s)
        worst = max(worst, monotonic() - before - request_s)
    return worst


@dataclass
class FixedRateLoop:
    hz: float
    monotonic: Callable[[], float] = time.monotonic
    sleep: Callable[[float], None] = time.sleep
    # Sleep to this much short of the deadline and spin the rest, since sleep overshoots by
    # milliseconds; zero sleeps the whole way, which is what a fake clock needs.
    spin_slack_s: float = 0.002
    lateness: list[float] = field(default_factory=list)
    periods: list[float] = field(default_factory=list)
    # Not `len(periods)`, which holds the intervals *between* ticks and is one short.
    served: int = 0
    resyncs: int = 0
    skipped_cycles: int = 0

    # A loop that spins away a fifth of its period is worse than one that admits its jitter.
    MAX_SPIN_SHARE = 0.2

    @classmethod
    def calibrated(cls, hz: float, margin: float = 1.2, samples: int = 5) -> "FixedRateLoop":
        """Spin slack sized to this host's sleep granularity; costs `samples` periods up front."""
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
        # Deadlines come from the index, never accumulated: adding 0.1 a thousand times drifts.
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
                # A genuine spin: `sleep(0)` parks the thread until the host's next timer slice.
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
            # An overrun past two periods re-bases the schedule instead of firing the backlog of
            # instantly-due deadlines. Re-basing moves the deadline the next tick measures against,
            # so the stall itself reports as zero lateness — quote the counters alongside it.
            after_body = self.monotonic()
            next_deadline = origin + index * period
            if after_body > next_deadline + period:
                self.resyncs += 1
                # The tick about to run is served late, not skipped; the epsilon keeps a stall
                # landing exactly on a period boundary from rounding down to one fewer.
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
