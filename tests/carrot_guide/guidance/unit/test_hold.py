import pytest

from carrot_guide.guidance import HoldPoint, Limits
from carrot_guide.state import NED

from tests.carrot_guide.guidance.toy_vehicle import AT_REST, simulate


@pytest.mark.unit
def test_hold_commands_nothing_when_already_on_target():
    law = HoldPoint(target=NED(10.0, -5.0, -20.0))
    command = law.command(NED(10.0, -5.0, -20.0), AT_REST)
    assert command.velocity.horizontal_norm == pytest.approx(0.0)
    assert command.velocity.down == pytest.approx(0.0)


@pytest.mark.unit
def test_hold_pulls_towards_the_target():
    law = HoldPoint(target=NED(50.0, 0.0))
    command = law.command(NED(0.0, 0.0), AT_REST)
    assert command.velocity.north > 0.0
    assert command.velocity.east == pytest.approx(0.0)


@pytest.mark.unit
def test_hold_respects_the_speed_limit():
    law = HoldPoint(target=NED(500.0, 500.0), limits=Limits(max_horizontal_speed=4.0))
    command = law.command(NED(0.0, 0.0), AT_REST)
    assert command.velocity.horizontal_norm == pytest.approx(4.0)


@pytest.mark.unit
def test_hold_respects_the_vertical_limit():
    law = HoldPoint(target=NED(0.0, 0.0, -100.0), limits=Limits(max_vertical_speed=1.5))
    command = law.command(NED(0.0, 0.0, 0.0), AT_REST)
    assert command.velocity.down == pytest.approx(-1.5)


@pytest.mark.unit
def test_damping_reduces_the_command_when_already_moving_in():
    law = HoldPoint(target=NED(50.0, 0.0), limits=Limits(max_horizontal_speed=100.0))
    still = law.command(NED(0.0, 0.0), AT_REST).velocity.north
    closing = law.command(NED(0.0, 0.0), NED(5.0, 0.0)).velocity.north
    assert closing < still


@pytest.mark.unit
def test_hold_faces_the_target_only_when_it_is_worth_turning():
    law = HoldPoint(target=NED(0.0, 40.0), face_target=True)
    assert law.command(NED(0.0, 0.0), AT_REST).yaw_deg == pytest.approx(90.0)
    assert law.command(NED(0.0, 39.5), AT_REST).yaw_deg is None


@pytest.mark.unit
def test_hold_converges_and_stays():
    law = HoldPoint(target=NED(40.0, -30.0, -20.0))
    track = simulate(law, NED(0.0, 0.0, 0.0), steps=600)
    final_errors = [law.tracking_error(p) for p in track[-50:]]
    assert max(final_errors) < 0.1


@pytest.mark.unit
def test_hold_does_not_overshoot_wildly():
    law = HoldPoint(target=NED(40.0, 0.0), limits=Limits(max_horizontal_speed=5.0))
    track = simulate(law, NED(0.0, 0.0), steps=400)
    assert max(p.north for p in track) < 41.0
