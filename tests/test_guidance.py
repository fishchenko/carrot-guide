"""Unit tests for the guidance laws, including a closed loop over a toy vehicle.

The toy vehicle is deliberately crude — a point mass that follows the commanded
velocity, either exactly or through a first-order lag. It cannot validate tuning
against a real airframe, but it does prove the laws converge and stay bounded, which
is what breaks when the geometry or a sign is wrong.
"""

import math

import pytest

from carrot_guide.guidance import (
    HoldPoint,
    Limits,
    Orbit,
    ProNav,
    Pursuit,
    Target,
    bearing_deg,
)
from carrot_guide.state import NED

AT_REST = NED(0.0, 0.0, 0.0)


def simulate(law, start: NED, steps: int, dt: float = 0.1, response_s: float = 0.0) -> list[NED]:
    """Integrate a law over a toy vehicle, optionally one that takes time to obey.

    With `response_s` at zero the vehicle takes up the commanded velocity instantly,
    which is all the station-keeping laws need. The intercept laws are measured against
    a first-order lag instead, because `ProNav` works by asking for a heading offset and
    letting the lag turn it into a turn *rate* — on a vehicle that snaps to its command
    the effective navigation constant would be set by the step size of this loop rather
    than by anything in the law.
    """
    position = start
    velocity = AT_REST
    track = [position]
    for step in range(steps):
        command = law.command(position, velocity, step * dt).velocity
        if response_s > 0.0:
            velocity = velocity + (command - velocity).scaled(min(1.0, dt / response_s))
        else:
            velocity = command
        position = position + velocity.scaled(dt)
        track.append(position)
    return track


def closest_approach(law, start: NED, steps: int, dt: float = 0.1, response_s: float = 0.22):
    """Miss distance and the time of it, the two numbers an intercept run reports."""
    track = simulate(law, start, steps, dt, response_s)
    ranges = [(law.tracking_error(p, i * dt), i * dt) for i, p in enumerate(track)]
    return min(ranges)


# -- bearings ------------------------------------------------------------------


@pytest.mark.parametrize(
    "offset, expected",
    [
        (NED(1.0, 0.0), 0.0),
        (NED(0.0, 1.0), 90.0),
        (NED(-1.0, 0.0), 180.0),
        (NED(0.0, -1.0), 270.0),
        (NED(1.0, 1.0), 45.0),
    ],
)
def test_bearing_is_clockwise_from_north(offset, expected):
    assert bearing_deg(offset) == pytest.approx(expected)


# -- hold point ----------------------------------------------------------------


def test_hold_commands_nothing_when_already_on_target():
    law = HoldPoint(target=NED(10.0, -5.0, -20.0))
    command = law.command(NED(10.0, -5.0, -20.0), AT_REST)
    assert command.velocity.horizontal_norm == pytest.approx(0.0)
    assert command.velocity.down == pytest.approx(0.0)


def test_hold_pulls_towards_the_target():
    law = HoldPoint(target=NED(50.0, 0.0))
    command = law.command(NED(0.0, 0.0), AT_REST)
    assert command.velocity.north > 0.0
    assert command.velocity.east == pytest.approx(0.0)


def test_hold_respects_the_speed_limit():
    law = HoldPoint(target=NED(500.0, 500.0), limits=Limits(max_horizontal_speed=4.0))
    command = law.command(NED(0.0, 0.0), AT_REST)
    assert command.velocity.horizontal_norm == pytest.approx(4.0)


def test_hold_respects_the_vertical_limit():
    law = HoldPoint(target=NED(0.0, 0.0, -100.0), limits=Limits(max_vertical_speed=1.5))
    command = law.command(NED(0.0, 0.0, 0.0), AT_REST)
    assert command.velocity.down == pytest.approx(-1.5)


def test_damping_reduces_the_command_when_already_moving_in():
    law = HoldPoint(target=NED(50.0, 0.0), limits=Limits(max_horizontal_speed=100.0))
    still = law.command(NED(0.0, 0.0), AT_REST).velocity.north
    closing = law.command(NED(0.0, 0.0), NED(5.0, 0.0)).velocity.north
    assert closing < still


def test_hold_faces_the_target_only_when_it_is_worth_turning():
    law = HoldPoint(target=NED(0.0, 40.0), face_target=True)
    assert law.command(NED(0.0, 0.0), AT_REST).yaw_deg == pytest.approx(90.0)
    assert law.command(NED(0.0, 39.5), AT_REST).yaw_deg is None


def test_hold_converges_and_stays():
    law = HoldPoint(target=NED(40.0, -30.0, -20.0))
    track = simulate(law, NED(0.0, 0.0, 0.0), steps=600)
    final_errors = [law.tracking_error(p) for p in track[-50:]]
    assert max(final_errors) < 0.1


def test_hold_does_not_overshoot_wildly():
    law = HoldPoint(target=NED(40.0, 0.0), limits=Limits(max_horizontal_speed=5.0))
    track = simulate(law, NED(0.0, 0.0), steps=400)
    assert max(p.north for p in track) < 41.0


# -- orbit ---------------------------------------------------------------------


def test_orbit_rejects_a_non_positive_radius():
    with pytest.raises(ValueError):
        Orbit(centre=NED(0.0, 0.0), radius_m=0.0)


def test_orbit_tangent_is_clockwise_seen_from_above():
    law = Orbit(centre=NED(0.0, 0.0), radius_m=20.0, speed_mps=3.0)
    # Due north of the centre, clockwise motion heads east.
    command = law.command(NED(20.0, 0.0), AT_REST)
    assert command.velocity.east == pytest.approx(3.0)
    assert command.velocity.north == pytest.approx(0.0)


def test_orbit_direction_flips_when_asked():
    law = Orbit(centre=NED(0.0, 0.0), radius_m=20.0, speed_mps=3.0, clockwise=False)
    command = law.command(NED(20.0, 0.0), AT_REST)
    assert command.velocity.east == pytest.approx(-3.0)


def test_orbit_pushes_out_from_inside_and_pulls_in_from_outside():
    law = Orbit(centre=NED(0.0, 0.0), radius_m=20.0, speed_mps=0.0)
    assert law.command(NED(10.0, 0.0), AT_REST).velocity.north > 0.0
    assert law.command(NED(30.0, 0.0), AT_REST).velocity.north < 0.0


def test_orbit_radial_error_is_positive_inside_the_circle():
    law = Orbit(centre=NED(0.0, 0.0), radius_m=20.0)
    assert law.radial_error(NED(10.0, 0.0)) == pytest.approx(10.0)
    assert law.radial_error(NED(30.0, 0.0)) == pytest.approx(-10.0)
    assert law.tracking_error(NED(30.0, 0.0)) == pytest.approx(10.0)


def test_orbit_from_the_centre_still_produces_a_command():
    law = Orbit(centre=NED(5.0, 5.0), radius_m=20.0, speed_mps=3.0)
    command = law.command(NED(5.0, 5.0), AT_REST)
    assert command.velocity.horizontal_norm > 0.0


def test_orbit_faces_the_centre():
    law = Orbit(centre=NED(0.0, 0.0), radius_m=20.0)
    # Sitting north of the centre, facing it means looking south.
    assert law.command(NED(20.0, 0.0), AT_REST).yaw_deg == pytest.approx(180.0)


def test_lookahead_of_zero_changes_nothing():
    without = Orbit(centre=NED(0.0, 0.0), radius_m=20.0, speed_mps=3.0)
    with_zero = Orbit(centre=NED(0.0, 0.0), radius_m=20.0, speed_mps=3.0, lookahead_s=0.0)
    position = NED(14.0, 14.0)
    assert without.command(position, AT_REST) == with_zero.command(position, AT_REST)


def test_lookahead_aims_the_tangent_further_around_the_circle():
    law = Orbit(centre=NED(0.0, 0.0), radius_m=20.0, speed_mps=4.0, lookahead_s=0.25)
    # Angular rate is 0.2 rad/s, so a quarter-second lookahead turns the aim by 0.05 rad
    # clockwise: the commanded velocity at the north point now leans slightly inward.
    command = law.command(NED(20.0, 0.0), AT_REST)
    assert command.velocity.east == pytest.approx(4.0 * math.cos(0.05), abs=1e-6)
    assert command.velocity.north == pytest.approx(-4.0 * math.sin(0.05), abs=1e-6)


def test_lookahead_leans_the_other_way_going_counter_clockwise():
    law = Orbit(
        centre=NED(0.0, 0.0),
        radius_m=20.0,
        speed_mps=4.0,
        lookahead_s=0.25,
        clockwise=False,
    )
    command = law.command(NED(20.0, 0.0), AT_REST)
    assert command.velocity.east == pytest.approx(-4.0 * math.cos(0.05), abs=1e-6)
    assert command.velocity.north == pytest.approx(-4.0 * math.sin(0.05), abs=1e-6)


def test_the_orbit_rate_follows_from_speed_and_radius():
    assert Orbit(centre=NED(0.0, 0.0), radius_m=25.0, speed_mps=5.0).rate_rad_s == pytest.approx(
        0.2
    )


def test_orbit_settles_onto_the_requested_radius():
    law = Orbit(
        centre=NED(0.0, 0.0, -20.0),
        radius_m=25.0,
        speed_mps=3.0,
        limits=Limits(max_horizontal_speed=6.0),
    )
    track = simulate(law, NED(5.0, 0.0, -20.0), steps=900)
    radii = [math.hypot(p.north, p.east) for p in track[-200:]]
    assert min(radii) > 24.0
    assert max(radii) < 26.0


def test_orbit_actually_goes_around():
    law = Orbit(centre=NED(0.0, 0.0), radius_m=20.0, speed_mps=4.0)
    track = simulate(law, NED(20.0, 0.0), steps=400)
    unwrapped = math.degrees(
        sum(
            _angle_step(track[i], track[i + 1])
            for i in range(len(track) - 1)
        )
    )
    # 400 steps at 0.1 s and 4 m/s is ~160 m of arc on a 126 m circumference, so more
    # than a full turn. Clockwise means the bearing angle grows.
    assert unwrapped > 300.0


def _angle_step(a: NED, b: NED) -> float:
    angle_a = math.atan2(a.east, a.north)
    angle_b = math.atan2(b.east, b.north)
    step = angle_b - angle_a
    while step > math.pi:
        step -= 2 * math.pi
    while step < -math.pi:
        step += 2 * math.pi
    return step


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
