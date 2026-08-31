from __future__ import annotations

from typing import Sequence

from carrot_guide.recording import Sample


# ~6.5 time constants of the outer loop's 1/kp = 1.25 s at the default kp = 0.8, leaving under
# 5 mm of the 2 m threshold decaying. Sweeping the repo's logs, the mean error falls steeply to
# ~8 s and is flat within a few tenths of a millimetre out to 20 s.
DEFAULT_HOLD_LEAD_IN_S = 8.0

DEFAULT_SETTLE_THRESHOLD_M = 2.0

# Well above the 0.4 m one 10 Hz tick covers at closing speed, so which tick got sampled cannot
# change the answer.
DEFAULT_REACH_THRESHOLD_M = 3.0

def settle_time(samples: Sequence[Sample], threshold_m: float) -> float | None:
    """When the error drops under `threshold_m` for good — the last violation, not the first."""
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
    """First time the error drops under `threshold_m`; None when it never got that close."""
    for sample in samples:
        if sample.error_m < threshold_m:
            return sample.t_s
    return None


def hold_start(
    samples: Sequence[Sample],
    threshold_m: float,
    lead_in_s: float = DEFAULT_HOLD_LEAD_IN_S,
) -> float | None:
    """Settle time plus the lead-in; None if the run never settles or ends before then."""
    settled_at = settle_time(samples, threshold_m)
    if settled_at is None:
        return None
    start = settled_at + lead_in_s
    return start if any(sample.t_s >= start for sample in samples) else None
