"""Plots for a recorded run: the flown track, and the error against time.

matplotlib is an optional dependency — flying and measuring must not require it, so
the import lives inside the function and the failure message says what to install.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

from carrot_guide.recording import Sample


def plot_run(
    samples: Sequence[Sample],
    path: str | Path,
    title: str = "",
    centre: tuple[float, float] | None = None,
    radius_m: float | None = None,
) -> Path:
    """Write a two-panel figure: track on the left, tracking error on the right."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as error:  # pragma: no cover - depends on the environment
        raise RuntimeError("plotting needs matplotlib: pip install '.[plots]'") from error

    if not samples:
        raise ValueError("nothing to plot")

    figure, (track, error) = plt.subplots(1, 2, figsize=(12, 5))

    east = [s.east_m for s in samples]
    north = [s.north_m for s in samples]
    track.plot(east, north, linewidth=1.2, label="track")
    track.plot(east[0], north[0], "o", color="tab:green", label="start")
    track.plot(east[-1], north[-1], "o", color="tab:red", label="end")

    if centre is not None:
        track.plot(centre[1], centre[0], "x", color="black", label="target")
        if radius_m:
            circle = plt.Circle(
                (centre[1], centre[0]), radius_m, fill=False, linestyle="--", color="black"
            )
            track.add_patch(circle)

    track.set_xlabel("east, m")
    track.set_ylabel("north, m")
    track.set_aspect("equal", adjustable="datalim")
    track.grid(alpha=0.3)
    track.legend(loc="best", fontsize="small")

    error.plot([s.t_s for s in samples], [s.error_m for s in samples], linewidth=1.2)
    error.set_xlabel("t, s")
    error.set_ylabel("tracking error, m")
    error.grid(alpha=0.3)

    if title:
        figure.suptitle(title)
    figure.tight_layout()

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=130)
    plt.close(figure)
    return output
