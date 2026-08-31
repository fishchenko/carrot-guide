import math

import pytest

from carrot_guide.guidance import Pursuit, Target
from carrot_guide.state import NED

from tests.carrot_guide.guidance.toy_vehicle import AT_REST, CROSSING, closest_approach


@pytest.mark.unit
def test_pursuit_aims_straight_at_the_target_now():
    law = Pursuit(target=Target(start=NED(0.0, 50.0), velocity=NED(0.0, 3.0)), speed_mps=4.0)
    # At t = 0 the target is due east, so the whole command is east at the set speed.
    command = law.command(NED(0.0, 0.0), AT_REST, 0.0)
    assert command.velocity.east == pytest.approx(4.0)
    assert command.velocity.north == pytest.approx(0.0)


@pytest.mark.unit
def test_pursuit_follows_the_target_rather_than_where_it_started():
    law = Pursuit(target=Target(start=NED(50.0, 0.0), velocity=NED(0.0, 5.0)), speed_mps=4.0)
    # Ten seconds in the target is 50 m east of where it began, at 45 degrees.
    command = law.command(NED(0.0, 0.0), AT_REST, 10.0)
    assert command.velocity.north == pytest.approx(command.velocity.east)
    assert law.tracking_error(NED(0.0, 0.0), 10.0) == pytest.approx(math.hypot(50.0, 50.0))


@pytest.mark.unit
def test_pursuit_flies_at_the_speed_it_was_given():
    law = Pursuit(target=CROSSING, speed_mps=4.0)
    speeds = [
        law.command(NED(n, 0.0), AT_REST, 0.0).velocity.horizontal_norm for n in (0.0, 20.0, 55.0)
    ]
    assert speeds == pytest.approx([4.0, 4.0, 4.0])


@pytest.mark.unit
def test_pursuit_holds_the_target_altitude():
    high = Target(start=NED(40.0, 0.0, -30.0))
    command = Pursuit(target=high, speed_mps=4.0).command(NED(0.0, 0.0, -10.0), AT_REST, 0.0)
    assert command.velocity.down < 0.0  # down is negative upwards: it climbs


@pytest.mark.unit
def test_pursuit_runs_a_slower_target_down():
    miss, _ = closest_approach(Pursuit(target=CROSSING, speed_mps=4.0), NED(0.0, 0.0), steps=600)
    assert miss < 0.5
