"""Plots for a recorded run: the flown track, and the error against time.

matplotlib is an optional dependency — flying and measuring must not require it, so
the import lives inside the function and the failure message says what to install.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

from carrot_guide.metrics import DEFAULT_REACH_THRESHOLD_M, reach_time
from carrot_guide.recording import Sample
from carrot_guide.state import NED


def plot_run(
    samples: Sequence[Sample],
    path: str | Path,
    title: str = "",
    centre: NED | None = None,
    radius_m: float | None = None,
    target_start: NED | None = None,
    target_velocity: NED | None = None,
) -> Path:
    """Write a two-panel figure: track on the left, tracking error on the right.

    `centre` marks something that stays put; `target_start` with `target_velocity` marks
    something that does not, and the two are drawn differently on purpose. Against a
    mover the aim point is a line rather than a cross, and where the two tracks come
    together is the whole result — a lead course meets it, a tail chase arrives behind
    it and turns for the rest of the run.
    """
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
        track.plot(centre.east, centre.north, "x", color="black", label="target")
        if radius_m:
            circle = plt.Circle(
                (centre.east, centre.north), radius_m, fill=False, linestyle="--", color="black"
            )
            track.add_patch(circle)

    if target_start is not None:
        # Recomputed from the run's own parameters rather than logged per sample: the
        # target is a pure function of time, so this cannot drift from what the law was
        # actually aiming at, and the log stays the vehicle's story alone.
        velocity = target_velocity if target_velocity is not None else NED(0.0, 0.0, 0.0)
        times = [s.t_s for s in samples]
        target_east = [target_start.east + velocity.east * t for t in times]
        target_north = [target_start.north + velocity.north * t for t in times]
        track.plot(target_east, target_north, "--", linewidth=1.2, color="black", label="target")
        track.plot(target_east[0], target_north[0], "x", color="black")

        # Where it got there, by `reach_time` rather than the deepest approach: a law
        # that stays alongside its target goes on making new minima, and marking the
        # lowest of them puts the star minutes away from the pass it is meant to show.
        reached = reach_time(samples, DEFAULT_REACH_THRESHOLD_M)
        if reached is not None:
            at = min(range(len(samples)), key=lambda i: abs(samples[i].t_s - reached))
            track.plot(
                target_east[at],
                target_north[at],
                "*",
                color="tab:orange",
                markersize=13,
                label=f"reached, {reached:.1f} s",
            )
            error.axvline(reached, color="tab:orange", linestyle=":", linewidth=1.2)

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
