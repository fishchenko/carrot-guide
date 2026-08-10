"""The one statistic both the flying loop and the analysis need.

`runner` reports the loop's lateness percentile while a run is in the air and `metrics`
reports the error percentile long afterwards, so this lives on its own rather than in
either of them: the flying path must not import the analysis layer (see `metrics` for
why), and the analysis must not import the loop.
"""

from __future__ import annotations

from typing import Sequence


def percentile(values: Sequence[float], fraction: float) -> float:
    """Nearest-rank percentile — no interpolation between neighbouring samples.

    Deliberately the simplest definition: every number this returns is a value the run
    actually produced, which is what makes a quoted p95 checkable against the log.
    """
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))
    return ordered[index]
