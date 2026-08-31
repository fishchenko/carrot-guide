from __future__ import annotations

from typing import Sequence


def percentile(values: Sequence[float], fraction: float) -> float:
    """Nearest rank, no interpolation — the result is always a value from the input."""
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))
    return ordered[index]
