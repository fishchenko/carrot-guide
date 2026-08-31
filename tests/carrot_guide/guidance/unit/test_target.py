import pytest

from carrot_guide.guidance import Target
from carrot_guide.state import NED


@pytest.mark.unit
def test_a_target_moves_along_its_velocity():
    target = Target(start=NED(10.0, 0.0, -20.0), velocity=NED(0.0, 3.0))
    assert target.at(0.0) == NED(10.0, 0.0, -20.0)
    assert target.at(4.0) == NED(10.0, 12.0, -20.0)


@pytest.mark.unit
def test_a_target_with_no_velocity_stays_put():
    target = Target(start=NED(10.0, -5.0))
    assert target.at(120.0) == target.start


@pytest.mark.unit
def test_the_collision_triangle_is_solved_exactly():
    # Target 40 m north crossing east at 3 m/s, pursuer at 5 m/s. Meeting at time t
    # needs 40^2 + (3t)^2 = (5t)^2, so 16t^2 = 1600 and t = 10 s.
    target = Target(start=NED(40.0, 0.0), velocity=NED(0.0, 3.0))
    assert target.intercept_time(NED(0.0, 0.0), 5.0) == pytest.approx(10.0)


@pytest.mark.unit
def test_a_stationary_target_is_met_after_the_straight_line_time():
    target = Target(start=NED(0.0, 60.0))
    assert target.intercept_time(NED(0.0, 0.0), 4.0) == pytest.approx(15.0)


@pytest.mark.unit
def test_a_target_that_outruns_the_pursuer_has_no_intercept():
    fleeing = Target(start=NED(50.0, 0.0), velocity=NED(6.0, 0.0))
    assert fleeing.intercept_time(NED(0.0, 0.0), 4.0) is None


@pytest.mark.unit
def test_an_equally_fast_target_is_still_met_if_it_is_closing():
    # Same speed both ways: the quadratic degenerates, and only a target with a
    # component towards the pursuer can be met at all.
    closing = Target(start=NED(40.0, 0.0), velocity=NED(-4.0, 0.0))
    assert closing.intercept_time(NED(0.0, 0.0), 4.0) == pytest.approx(5.0)
    opening = Target(start=NED(40.0, 0.0), velocity=NED(4.0, 0.0))
    assert opening.intercept_time(NED(0.0, 0.0), 4.0) is None
