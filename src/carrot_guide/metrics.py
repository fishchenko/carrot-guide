"""Turning a flight log into the numbers the README claims.

Errors are reported over the *hold* — the stretch where the vehicle is keeping station —
and not over the approach that precedes it. The approach is a property of the speed
limit, not of the law's tracking quality, and averaging it in would let a slow approach
hide a bad hold.

Reaching the target and holding it are two different events, and this module keeps them
apart. Crossing the settle threshold only means the vehicle has arrived; the outer loop
is still an exponential away from its steady state at that moment, and on these runs the
steady error is two orders of magnitude smaller than the threshold. Averaging from the
crossing therefore measures mostly the tail of the approach: on `logs/hold-calm.csv` it
put the mean at 0.053 m where the vehicle was actually holding to 0.010 m, and it made
the published mean depend on the length of the run rather than on the law — the 60 s and
600 s runs of the same law differed fourfold for no reason but the averaging window.

So the statistics start a lead-in after the crossing, and the summary says which window
it used. See `DEFAULT_HOLD_LEAD_IN_S` for where the lead-in comes from.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from typing import Sequence

from carrot_guide.recording import Sample
from carrot_guide.utils import percentile

# How long after reaching the target the statistics start.
#
# The outer loop is a first-order lag with a time constant of 1/kp — 1.25 s at the
# default kp = 0.8 — so eight seconds is about six and a half time constants, leaving
# under 5 mm of the 2 m threshold still decaying. That is an order of magnitude below
# the steady error these runs hold to, which is why the measured statistics stop moving
# here: sweeping the lead-in across the repo's logs, the mean error falls steeply to
# ~8 s and is then flat to within a few tenths of a millimetre out to 20 s. Measured,
# not picked — the same discipline the rest of the project's constants follow.
DEFAULT_HOLD_LEAD_IN_S = 8.0

# Error is judged against this distance: under it the vehicle counts as arrived. Named
# because the CLI offers it on two subcommands and all three used to spell it `2.0`.
DEFAULT_SETTLE_THRESHOLD_M = 2.0

# How close counts as having reached a moving target. Far below the tens of metres an
# engagement opens at, and far above the 0.4 m one 10 Hz tick covers at closing speed,
# so the answer does not depend on which tick happened to be sampled.
DEFAULT_REACH_THRESHOLD_M = 3.0

# Which part of the run the error statistics describe. A run that is too short to have a
# settled stretch, or that never settles at all, still gets a summary — but it says so,
# rather than reporting a different quantity under the same field names.
WINDOW_HOLD = "hold"
WINDOW_POST_SETTLE = "post-settle"
WINDOW_WHOLE_RUN = "whole run"


@dataclass(frozen=True)
class RunSummary:
    """Tracking quality of one guidance run."""

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
    # The closest the vehicle ever came to what it was aiming at, and when. A footnote
    # on a station-keeping run; on an intercept the minimum of the range *is* the miss
    # distance. Here on the one summary rather than in a second one beside it, so that
    # `carrot-guide report` hands the number back from a recorded log without having to
    # be told which experiment wrote it.
    #
    # `min_error_t_s` is not the time to intercept, however tempting it looks: it is the
    # time of the *deepest* approach, and a law that stays with its target after the
    # pass keeps making new ones. A pursuit run that had been within 3 m since 24.3 s
    # reported 43.6 s this way, purely because a later wobble dipped a millimetre lower.
    # `reach_time_s` is the honest answer and the two differ by design.
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


def settle_time(samples: Sequence[Sample], threshold_m: float) -> float | None:
    """When the error drops under `threshold_m` for good — i.e. when the vehicle arrived.

    Found by scanning backwards for the last violation rather than forwards for the
    first success, so a brief excursion late in the run cannot be mistaken for a
    settled one.
    """
    last_violation = -1
    for index, sample in enumerate(samples):
        if sample.error_m > threshold_m:
            last_violation = index
    if last_violation < 0:
        return samples[0].t_s if samples else None
    if last_violation + 1 >= len(samples):
        return None
    return samples[last_violation + 1].t_s


def reach_time(samples: Sequence[Sample], threshold_m: float) -> float | None:
    """First time the error drops under `threshold_m` — i.e. when the vehicle got there.

    The mirror of `settle_time`, and scanning the other way for the same reason. A hold
    is about staying, so settling looks for the last violation; an intercept is about
    arriving, and nothing the vehicle does after the pass makes the pass itself earlier
    or later. None when it never got that close.
    """
    for sample in samples:
        if sample.error_m < threshold_m:
            return sample.t_s
    return None


def hold_start(
    samples: Sequence[Sample],
    threshold_m: float,
    lead_in_s: float = DEFAULT_HOLD_LEAD_IN_S,
) -> float | None:
    """When the vehicle is holding rather than still arriving, or None if it never is."""
    settled_at = settle_time(samples, threshold_m)
    if settled_at is None:
        return None
    start = settled_at + lead_in_s
    return start if any(sample.t_s >= start for sample in samples) else None


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

    # Three windows, always named in the output. The fallbacks exist because a summary
    # is more useful than a refusal, but a run that never settled and a run that held
    # perfectly must not report the same-looking numbers under the same field names.
    if started_at is not None:
        window, measured = WINDOW_HOLD, [s for s in samples if s.t_s >= started_at]
    elif settled_at is not None:
        window = WINDOW_POST_SETTLE
        measured = [s for s in samples if s.t_s >= settled_at]
    else:
        window, measured = WINDOW_WHOLE_RUN, list(samples)

    errors = [s.error_m for s in measured]
    # Taken from `measured` like the rest, so `window` describes it too: on an intercept
    # that window is the whole run, which is what makes the minimum a miss distance
    # rather than the smallest range inside some stretch chosen after the fact.
    closest = min(measured, key=lambda s: s.error_m)
    # Deliberately the whole run, not `measured`: the fastest the vehicle ever went is a
    # property of the approach and of the speed limit, and it is the approach the window
    # exists to exclude. It is the one field here the `window` does not describe.
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


@dataclass(frozen=True)
class LatencySummary:
    """Command-to-reaction latency, measured over several step commands."""

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
