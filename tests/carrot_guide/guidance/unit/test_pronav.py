import math

import pytest

from carrot_guide.guidance import ProNav, Pursuit, Target
from carrot_guide.state import NED

from tests.carrot_guide.guidance.toy_vehicle import AT_REST, CROSSING, closest_approach


@pytest.mark.unit
def test_the_line_of_sight_rate_is_zero_on_a_collision_course():
    # Target 40 m north crossing east at 3 m/s; a pursuer at 5 m/s on the collision
    # triangle flies 3 m/s east and 4 m/s north, which holds the bearing steady.
    law = ProNav(target=Target(start=NED(40.0, 0.0), velocity=NED(0.0, 3.0)), speed_mps=5.0)
    assert law.line_of_sight_rate(NED(0.0, 0.0), NED(4.0, 3.0), 0.0) == pytest.approx(0.0)


@pytest.mark.unit
def test_the_line_of_sight_rate_turns_with_the_target():
    # Sitting still while the target crosses east 40 m away: the bearing swings east,
    # which is clockwise, at v/r = 3/40 rad/s.
    law = ProNav(target=Target(start=NED(40.0, 0.0), velocity=NED(0.0, 3.0)), speed_mps=4.0)
    assert law.line_of_sight_rate(NED(0.0, 0.0), AT_REST, 0.0) == pytest.approx(3.0 / 40.0)


@pytest.mark.unit
def test_a_steady_bearing_leaves_the_heading_alone():
    """The whole point of the law: constant bearing means the course is already right."""
    law = ProNav(target=Target(start=NED(40.0, 0.0), velocity=NED(0.0, 3.0)), speed_mps=5.0)
    on_course = NED(4.0, 3.0)
    command = law.command(NED(0.0, 0.0), on_course, 0.0)
    assert command.velocity.north == pytest.approx(4.0)
    assert command.velocity.east == pytest.approx(3.0)


@pytest.mark.unit
def test_the_turn_follows_the_bearing_and_is_n_times_larger():
    law = ProNav(
        target=Target(start=NED(40.0, 0.0), velocity=NED(0.0, 3.0)),
        speed_mps=4.0,
        n=3.0,
        response_s=0.2,
    )
    # Flying due north at 4 m/s: the bearing swings clockwise at 3/40 rad/s, so the
    # commanded heading is offset clockwise by N * rate * response = 0.045 rad.
    command = law.command(NED(0.0, 0.0), NED(4.0, 0.0), 0.0)
    offset = 3.0 * (3.0 / 40.0) * 0.2
    assert command.velocity.north == pytest.approx(4.0 * math.cos(offset), abs=1e-9)
    assert command.velocity.east == pytest.approx(4.0 * math.sin(offset), abs=1e-9)


@pytest.mark.unit
def test_a_bearing_turning_the_other_way_turns_the_command_the_other_way():
    law = ProNav(target=Target(start=NED(40.0, 0.0), velocity=NED(0.0, -3.0)), speed_mps=4.0)
    assert law.command(NED(0.0, 0.0), NED(4.0, 0.0), 0.0).velocity.east < 0.0


@pytest.mark.unit
def test_with_no_heading_yet_the_law_starts_out_as_pursuit():
    """A hovering vehicle has no velocity vector to rotate; it sets off at the target."""
    law = ProNav(target=Target(start=NED(0.0, 40.0), velocity=NED(0.0, 3.0)), speed_mps=4.0)
    command = law.command(NED(0.0, 0.0), AT_REST, 0.0)
    assert command.velocity.east == pytest.approx(4.0)
    assert command.velocity.north == pytest.approx(0.0)


@pytest.mark.unit
def test_proportional_navigation_intercepts_sooner_than_pursuit():
    """The result the experiment exists for, on the toy vehicle before the simulator.

    Both laws fly at 4 m/s at a target crossing at 3 m/s, and both eventually run it
    down — a pursuer that is simply faster always does. What separates them is when:
    pure pursuit spends its turn rate following a line of sight that keeps rotating and
    arrives from behind, while nulling that rotation puts the vehicle on the collision
    triangle and gets it there in close to the least time the geometry allows.
    """
    optimum = CROSSING.intercept_time(NED(0.0, 0.0), 4.0)
    pursuit_miss, pursuit_t = closest_approach(
        Pursuit(target=CROSSING, speed_mps=4.0), NED(0.0, 0.0), steps=600
    )
    pronav_miss, pronav_t = closest_approach(
        ProNav(target=CROSSING, speed_mps=4.0), NED(0.0, 0.0), steps=600
    )

    assert optimum == pytest.approx(15.06, abs=0.05)
    assert pursuit_miss < 0.5 and pronav_miss < 0.5  # both arrive, given long enough
    assert pronav_t < pursuit_t - 5.0
    # Within a fifth of the best any constant-speed law could have done, where pure
    # pursuit takes better than half as long again.
    assert pronav_t < optimum * 1.2
    assert pursuit_t > optimum * 1.5


@pytest.mark.unit
def test_inside_a_short_run_only_proportional_navigation_arrives():
    """Same geometry, same speed, twenty seconds: one law is there and one is not."""
    steps = 200  # 20 s at the 0.1 s step
    pursuit_miss, _ = closest_approach(
        Pursuit(target=CROSSING, speed_mps=4.0), NED(0.0, 0.0), steps=steps
    )
    pronav_miss, _ = closest_approach(
        ProNav(target=CROSSING, speed_mps=4.0), NED(0.0, 0.0), steps=steps
    )
    assert pronav_miss < 1.0
    assert pursuit_miss > 3.0


@pytest.mark.unit
def test_a_navigation_constant_of_zero_never_turns_at_all():
    """The degenerate case the constant is named for: no turn is proportional to nothing."""
    law = ProNav(target=CROSSING, speed_mps=4.0, n=0.0)
    command = law.command(NED(0.0, 0.0), NED(4.0, 0.0), 5.0)
    assert command.velocity.north == pytest.approx(4.0)
    assert command.velocity.east == pytest.approx(0.0)
