"""Rendering tests: they draw real figures, because a plot has no other output to check.

What is asserted is that each branch produces a PNG at the path it was given — enough to
catch the way this code fails, which is an index or an attribute error inside one overlay
that nothing else draws.
"""

import pytest

from carrot_guide.metrics import DEFAULT_REACH_THRESHOLD_M
from carrot_guide.plots import plot_run
from carrot_guide.state import NED

from tests.runs import sample
PNG_MAGIC = b"\x89PNG"

CLOSING = [sample(0.0, 20.0), sample(5.0, 8.0), sample(10.0, DEFAULT_REACH_THRESHOLD_M - 1.0)]


def rendered(path) -> bytes:
    data = path.read_bytes()
    assert data.startswith(PNG_MAGIC)
    return data


@pytest.mark.unit
def test_a_run_is_rendered_to_the_path_it_was_given(tmp_path):
    path = tmp_path / "run.png"
    assert plot_run(CLOSING, path, title="hold") == path
    assert len(rendered(path)) > 0


@pytest.mark.unit
def test_the_figure_directory_is_created_on_demand(tmp_path):
    path = tmp_path / "nested" / "deeper" / "run.png"
    plot_run(CLOSING, path)
    rendered(path)


@pytest.mark.unit
def test_an_empty_run_is_refused(tmp_path):
    with pytest.raises(ValueError):
        plot_run([], tmp_path / "run.png")


@pytest.mark.unit
def test_a_static_aim_point_is_drawn_with_its_circle(tmp_path):
    path = tmp_path / "orbit.png"
    plot_run(CLOSING, path, centre=NED(10.0, -5.0), radius_m=25.0)
    rendered(path)


@pytest.mark.unit
def test_a_moving_target_is_marked_at_the_moment_it_was_reached(tmp_path):
    # The last sample is inside the reach threshold, so the star branch runs.
    path = tmp_path / "intercept.png"
    plot_run(CLOSING, path, target_start=NED(40.0, 0.0), target_velocity=NED(0.0, 3.0))
    rendered(path)


@pytest.mark.unit
def test_a_moving_target_that_was_never_reached_still_renders(tmp_path):
    path = tmp_path / "miss.png"
    never = [sample(0.0, 40.0), sample(5.0, 30.0)]
    plot_run(never, path, target_start=NED(40.0, 0.0))
    rendered(path)
