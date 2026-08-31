from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from typing import Sequence

from carrot_guide.metrics.events import (
    DEFAULT_HOLD_LEAD_IN_S,
    DEFAULT_REACH_THRESHOLD_M,
    DEFAULT_SETTLE_THRESHOLD_M,
    hold_start,
    reach_time,
    settle_time,
)
from carrot_guide.recording import Sample
from carrot_guide.utils import percentile


WINDOW_HOLD = "hold"
WINDOW_POST_SETTLE = "post-settle"
WINDOW_WHOLE_RUN = "whole run"


@dataclass(frozen=True)
class RunSummary:
    label: str
    duration_s: float
    settle_time_s: float | None
    hold_start_s: float | None
    window: str
    measured_samples: int
    mean_error_m: float
    rms_error_m: float
    max_error_m: float
    p95_error_m: float
    # `min_error_t_s` is the deepest approach, not the time to intercept: a law that stays with
    # its target after the pass keeps making new ones. `reach_time_s` is that answer.
    min_error_m: float
    min_error_t_s: float
    reach_time_s: float | None
    max_speed_mps: float

    def as_dict(self) -> dict[str, float | str | None]:
        return {
            "label": self.label,
            "duration_s": round(self.duration_s, 2),
            "settle_time_s": None if self.settle_time_s is None else round(self.settle_time_s, 2),
            "hold_start_s": None if self.hold_start_s is None else round(self.hold_start_s, 2),
            "window": self.window,
            "measured_samples": self.measured_samples,
            "mean_error_m": round(self.mean_error_m, 3),
            "rms_error_m": round(self.rms_error_m, 3),
            "max_error_m": round(self.max_error_m, 3),
            "p95_error_m": round(self.p95_error_m, 3),
            "min_error_m": round(self.min_error_m, 3),
            "min_error_t_s": round(self.min_error_t_s, 2),
            "reach_time_s": None if self.reach_time_s is None else round(self.reach_time_s, 2),
            "max_speed_mps": round(self.max_speed_mps, 2),
        }


def summarise(
    samples: Sequence[Sample],
    label: str = "",
    settle_threshold_m: float = DEFAULT_SETTLE_THRESHOLD_M,
    reach_threshold_m: float = DEFAULT_REACH_THRESHOLD_M,
    hold_lead_in_s: float = DEFAULT_HOLD_LEAD_IN_S,
) -> RunSummary:
    if not samples:
        raise ValueError("cannot summarise an empty run")

    settled_at = settle_time(samples, settle_threshold_m)
    started_at = hold_start(samples, settle_threshold_m, hold_lead_in_s)

    # A run too short to settle still gets a summary; `window` says which one it got.
    if started_at is not None:
        window, measured = WINDOW_HOLD, [s for s in samples if s.t_s >= started_at]
    elif settled_at is not None:
        window = WINDOW_POST_SETTLE
        measured = [s for s in samples if s.t_s >= settled_at]
    else:
        window, measured = WINDOW_WHOLE_RUN, list(samples)

    errors = [s.error_m for s in measured]
    closest = min(measured, key=lambda s: s.error_m)
    # The whole run, not `measured`: peak speed belongs to the approach the window excludes, so
    # this is the one field `window` does not describe.
    speeds = [math.hypot(s.vn, s.ve) for s in samples]

    return RunSummary(
        label=label or samples[0].label,
        duration_s=samples[-1].t_s - samples[0].t_s,
        settle_time_s=settled_at,
        hold_start_s=started_at,
        window=window,
        measured_samples=len(measured),
        mean_error_m=statistics.fmean(errors),
        rms_error_m=math.sqrt(statistics.fmean(e * e for e in errors)),
        max_error_m=max(errors),
        p95_error_m=percentile(errors, 0.95),
        min_error_m=closest.error_m,
        min_error_t_s=closest.t_s,
        reach_time_s=reach_time(samples, reach_threshold_m),
        max_speed_mps=max(speeds, default=0.0),
    )
