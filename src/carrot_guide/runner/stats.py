from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Tick:
    index: int
    elapsed_s: float
    lateness_s: float


@dataclass
class LoopStats:
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
            "resyncs": float(self.resyncs),
            "skipped_cycles": float(self.skipped_cycles),
        }
