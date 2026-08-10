"""Unit tests for the guidance laws, including a closed loop over a toy vehicle.

The toy vehicle is deliberately crude — a point mass that follows the commanded
velocity exactly. It cannot validate tuning against a real airframe, but it does
prove the laws converge and stay bounded, which is what breaks when the geometry or
a sign is wrong.
"""

import math

import pytest

from carrot_guide.guidance import HoldPoint, Limits, Orbit, bearing_deg
from carrot_guide.state import NED

AT_REST = NED(0.0, 0.0, 0.0)


def simulate(law, start: NED, steps: int, dt: float = 0.1) -> list[NED]:
    """Integrate a law over a vehicle that tracks the commanded velocity exactly."""
    position = start
    velocity = AT_REST
    track = [position]
    for _ in range(steps):
        velocity = law.command(position, velocity).velocity
        position = position + velocity.scaled(dt)
        track.append(position)
    return track


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
