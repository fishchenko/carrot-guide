from __future__ import annotations

import statistics
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class LatencySummary:
    trials: int
    median_ms: float
    min_ms: float
    max_ms: float

    def as_dict(self) -> dict[str, float]:
        return {
            "trials": float(self.trials),
            "median_ms": round(self.median_ms, 1),
            "min_ms": round(self.min_ms, 1),
            "max_ms": round(self.max_ms, 1),
        }


def summarise_latency(latencies_s: Sequence[float]) -> LatencySummary:
    if not latencies_s:
        raise ValueError("no latency trials recorded")
    return LatencySummary(
        trials=len(latencies_s),
        median_ms=statistics.median(latencies_s) * 1000.0,
        min_ms=min(latencies_s) * 1000.0,
        max_ms=max(latencies_s) * 1000.0,
    )
