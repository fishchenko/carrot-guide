"""Unit tests for the intercept laws, measured against a toy vehicle with a lag.

The lag is not decoration: `ProNav` works by asking for a heading offset and letting
the vehicle's response turn it into a turn *rate*, so on a vehicle that snaps to its
command the effective navigation constant would be set by the step size of the
integration loop rather than by anything in the law.

The laws whose target stays put are in `test_guidance.py`.
"""

import math

import pytest

from carrot_guide.guidance import ProNav, Pursuit, Target
from carrot_guide.state import NED

from conftest import AT_REST, simulate


def closest_approach(law, start: NED, steps: int, dt: float = 0.1, response_s: float = 0.22):
    """Miss distance and the time of it, the two numbers an intercept run reports."""
    track = simulate(law, start, steps, dt, response_s)
    ranges = [(law.tracking_error(p, i * dt), i * dt) for i, p in enumerate(track)]
    return min(ranges)


# -- moving targets ------------------------------------------------------------


def test_a_target_moves_along_its_velocity():
    target = Target(start=NED(10.0, 0.0, -20.0), velocity=NED(0.0, 3.0))
    assert target.at(0.0) == NED(10.0, 0.0, -20.0)
    assert target.at(4.0) == NED(10.0, 12.0, -20.0)


def test_a_target_with_no_velocity_stays_put():
    target = Target(start=NED(10.0, -5.0))
    assert target.at(120.0) == target.start


def test_the_collision_triangle_is_solved_exactly():
    # Target 40 m north crossing east at 3 m/s, pursuer at 5 m/s. Meeting at time t
    # needs 40^2 + (3t)^2 = (5t)^2, so 16t^2 = 1600 and t = 10 s.
    target = Target(start=NED(40.0, 0.0), velocity=NED(0.0, 3.0))
    assert target.intercept_time(NED(0.0, 0.0), 5.0) == pytest.approx(10.0)


def test_a_stationary_target_is_met_after_the_straight_line_time():
    target = Target(start=NED(0.0, 60.0))
    assert target.intercept_time(NED(0.0, 0.0), 4.0) == pytest.approx(15.0)


def test_a_target_that_outruns_the_pursuer_has_no_intercept():
    fleeing = Target(start=NED(50.0, 0.0), velocity=NED(6.0, 0.0))
    assert fleeing.intercept_time(NED(0.0, 0.0), 4.0) is None


def test_an_equally_fast_target_is_still_met_if_it_is_closing():
    # Same speed both ways: the quadratic degenerates, and only a target with a
    # component towards the pursuer can be met at all.
    closing = Target(start=NED(40.0, 0.0), velocity=NED(-4.0, 0.0))
    assert closing.intercept_time(NED(0.0, 0.0), 4.0) == pytest.approx(5.0)
    opening = Target(start=NED(40.0, 0.0), velocity=NED(4.0, 0.0))
    assert opening.intercept_time(NED(0.0, 0.0), 4.0) is None


# -- pursuit -------------------------------------------------------------------


CROSSING = Target(start=NED(60.0, -40.0, 0.0), velocity=NED(0.0, 3.0))


def test_pursuit_aims_straight_at_the_target_now():
    law = Pursuit(target=Target(start=NED(0.0, 50.0), velocity=NED(0.0, 3.0)), speed_mps=4.0)
    # At t = 0 the target is due east, so the whole command is east at the set speed.
    command = law.command(NED(0.0, 0.0), AT_REST, 0.0)
    assert command.velocity.east == pytest.approx(4.0)
    assert command.velocity.north == pytest.approx(0.0)


def test_pursuit_follows_the_target_rather_than_where_it_started():
    law = Pursuit(target=Target(start=NED(50.0, 0.0), velocity=NED(0.0, 5.0)), speed_mps=4.0)
    # Ten seconds in the target is 50 m east of where it began, at 45 degrees.
    command = law.command(NED(0.0, 0.0), AT_REST, 10.0)
    assert command.velocity.north == pytest.approx(command.velocity.east)
    assert law.tracking_error(NED(0.0, 0.0), 10.0) == pytest.approx(math.hypot(50.0, 50.0))


def test_pursuit_flies_at_the_speed_it_was_given():
    law = Pursuit(target=CROSSING, speed_mps=4.0)
    speeds = [
        law.command(NED(n, 0.0), AT_REST, 0.0).velocity.horizontal_norm for n in (0.0, 20.0, 55.0)
    ]
    assert speeds == pytest.approx([4.0, 4.0, 4.0])


def test_pursuit_holds_the_target_altitude():
    high = Target(start=NED(40.0, 0.0, -30.0))
    command = Pursuit(target=high, speed_mps=4.0).command(NED(0.0, 0.0, -10.0), AT_REST, 0.0)
    assert command.velocity.down < 0.0  # down is negative upwards: it climbs


def test_pursuit_runs_a_slower_target_down():
    miss, _ = closest_approach(Pursuit(target=CROSSING, speed_mps=4.0), NED(0.0, 0.0), steps=600)
    assert miss < 0.5


# -- proportional navigation ---------------------------------------------------


def test_the_line_of_sight_rate_is_zero_on_a_collision_course():
    # Target 40 m north crossing east at 3 m/s; a pursuer at 5 m/s on the collision
    # triangle flies 3 m/s east and 4 m/s north, which holds the bearing steady.
    law = ProNav(target=Target(start=NED(40.0, 0.0), velocity=NED(0.0, 3.0)), speed_mps=5.0)
    assert law.line_of_sight_rate(NED(0.0, 0.0), NED(4.0, 3.0), 0.0) == pytest.approx(0.0)


def test_the_line_of_sight_rate_turns_with_the_target():
    # Sitting still while the target crosses east 40 m away: the bearing swings east,
    # which is clockwise, at v/r = 3/40 rad/s.
    law = ProNav(target=Target(start=NED(40.0, 0.0), velocity=NED(0.0, 3.0)), speed_mps=4.0)
    assert law.line_of_sight_rate(NED(0.0, 0.0), AT_REST, 0.0) == pytest.approx(3.0 / 40.0)


def test_a_steady_bearing_leaves_the_heading_alone():
    """The whole point of the law: constant bearing means the course is already right."""
    law = ProNav(target=Target(start=NED(40.0, 0.0), velocity=NED(0.0, 3.0)), speed_mps=5.0)
    on_course = NED(4.0, 3.0)
    command = law.command(NED(0.0, 0.0), on_course, 0.0)
    assert command.velocity.north == pytest.approx(4.0)
    assert command.velocity.east == pytest.approx(3.0)


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


def test_a_bearing_turning_the_other_way_turns_the_command_the_other_way():
    law = ProNav(target=Target(start=NED(40.0, 0.0), velocity=NED(0.0, -3.0)), speed_mps=4.0)
    assert law.command(NED(0.0, 0.0), NED(4.0, 0.0), 0.0).velocity.east < 0.0


def test_with_no_heading_yet_the_law_starts_out_as_pursuit():
    """A hovering vehicle has no velocity vector to rotate; it sets off at the target."""
    law = ProNav(target=Target(start=NED(0.0, 40.0), velocity=NED(0.0, 3.0)), speed_mps=4.0)
    command = law.command(NED(0.0, 0.0), AT_REST, 0.0)
    assert command.velocity.east == pytest.approx(4.0)
    assert command.velocity.north == pytest.approx(0.0)


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


def test_a_navigation_constant_of_zero_never_turns_at_all():
    """The degenerate case the constant is named for: no turn is proportional to nothing."""
    law = ProNav(target=CROSSING, speed_mps=4.0, n=0.0)
    command = law.command(NED(0.0, 0.0), NED(4.0, 0.0), 5.0)
    assert command.velocity.north == pytest.approx(4.0)
    assert command.velocity.east == pytest.approx(0.0)

