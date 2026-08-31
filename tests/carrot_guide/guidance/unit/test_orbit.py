import math

import pytest

from carrot_guide.guidance import Limits, Orbit
from carrot_guide.state import NED

from tests.carrot_guide.guidance.toy_vehicle import AT_REST, simulate


@pytest.mark.unit
def test_orbit_rejects_a_non_positive_radius():
    with pytest.raises(ValueError):
        Orbit(centre=NED(0.0, 0.0), radius_m=0.0)


@pytest.mark.unit
def test_orbit_tangent_is_clockwise_seen_from_above():
    law = Orbit(centre=NED(0.0, 0.0), radius_m=20.0, speed_mps=3.0)
    # Due north of the centre, clockwise motion heads east.
    command = law.command(NED(20.0, 0.0), AT_REST)
    assert command.velocity.east == pytest.approx(3.0)
    assert command.velocity.north == pytest.approx(0.0)


@pytest.mark.unit
def test_orbit_direction_flips_when_asked():
    law = Orbit(centre=NED(0.0, 0.0), radius_m=20.0, speed_mps=3.0, clockwise=False)
    command = law.command(NED(20.0, 0.0), AT_REST)
    assert command.velocity.east == pytest.approx(-3.0)


@pytest.mark.unit
def test_orbit_pushes_out_from_inside_and_pulls_in_from_outside():
    law = Orbit(centre=NED(0.0, 0.0), radius_m=20.0, speed_mps=0.0)
    assert law.command(NED(10.0, 0.0), AT_REST).velocity.north > 0.0
    assert law.command(NED(30.0, 0.0), AT_REST).velocity.north < 0.0


@pytest.mark.unit
def test_orbit_radial_error_is_positive_inside_the_circle():
    law = Orbit(centre=NED(0.0, 0.0), radius_m=20.0)
    assert law.radial_error(NED(10.0, 0.0)) == pytest.approx(10.0)
    assert law.radial_error(NED(30.0, 0.0)) == pytest.approx(-10.0)
    assert law.tracking_error(NED(30.0, 0.0)) == pytest.approx(10.0)


@pytest.mark.unit
def test_orbit_from_the_centre_still_produces_a_command():
    law = Orbit(centre=NED(5.0, 5.0), radius_m=20.0, speed_mps=3.0)
    command = law.command(NED(5.0, 5.0), AT_REST)
    assert command.velocity.horizontal_norm > 0.0


@pytest.mark.unit
def test_orbit_faces_the_centre():
    law = Orbit(centre=NED(0.0, 0.0), radius_m=20.0)
    # Sitting north of the centre, facing it means looking south.
    assert law.command(NED(20.0, 0.0), AT_REST).yaw_deg == pytest.approx(180.0)


@pytest.mark.unit
def test_lookahead_of_zero_changes_nothing():
    without = Orbit(centre=NED(0.0, 0.0), radius_m=20.0, speed_mps=3.0)
    with_zero = Orbit(centre=NED(0.0, 0.0), radius_m=20.0, speed_mps=3.0, lookahead_s=0.0)
    position = NED(14.0, 14.0)
    assert without.command(position, AT_REST) == with_zero.command(position, AT_REST)


@pytest.mark.unit
def test_lookahead_aims_the_tangent_further_around_the_circle():
    law = Orbit(centre=NED(0.0, 0.0), radius_m=20.0, speed_mps=4.0, lookahead_s=0.25)
    # Angular rate is 0.2 rad/s, so a quarter-second lookahead turns the aim by 0.05 rad
    # clockwise: the commanded velocity at the north point now leans slightly inward.
    command = law.command(NED(20.0, 0.0), AT_REST)
    assert command.velocity.east == pytest.approx(4.0 * math.cos(0.05), abs=1e-6)
    assert command.velocity.north == pytest.approx(-4.0 * math.sin(0.05), abs=1e-6)


@pytest.mark.unit
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


@pytest.mark.unit
def test_the_orbit_rate_follows_from_speed_and_radius():
    assert Orbit(centre=NED(0.0, 0.0), radius_m=25.0, speed_mps=5.0).rate_rad_s == pytest.approx(
        0.2
    )


@pytest.mark.unit
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


@pytest.mark.unit
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
