"""Synthetic sample series, for the metrics tests that need a run but not a flight."""

from __future__ import annotations

import math

from carrot_guide.recording import Sample


def sample(t: float, error: float, vn: float = 0.0, ve: float = 0.0) -> Sample:
    return Sample(
        t_s=t,
        label="test",
        lat_deg=50.0,
        lon_deg=30.0,
        north_m=0.0,
        east_m=0.0,
        down_m=-20.0,
        vn=vn,
        ve=ve,
        vd=0.0,
        cmd_vn=0.0,
        cmd_ve=0.0,
        cmd_vd=0.0,
        error_m=error,
        lateness_ms=0.0,
        mode="GUIDED",
        armed=True,
    )


def approach_then_hold(hold_error: float = 0.4, seconds: float = 40.0) -> list[Sample]:
    """A run shaped like a real one: an approach, an exponential tail, then a steady hold.

    Eighteen samples close from 20 m to 3 m, the error crosses the 2 m threshold at
    t = 1.8 s, and from there it decays towards `hold_error` with the outer loop's own
    1.25 s time constant. The tail is the point: it is what a window opening at the
    threshold crossing averages in, and what the lead-in exists to leave out.
    """
    approach = [sample(index * 0.1, 20.0 - index) for index in range(18)]
    hold = [
        sample(1.8 + index * 0.1, hold_error + 1.5 * math.exp(-(index * 0.1) / 1.25))
        for index in range(int(seconds / 0.1))
    ]
    return approach + hold
