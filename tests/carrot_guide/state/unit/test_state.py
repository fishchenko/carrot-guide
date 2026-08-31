import math

import pytest

from carrot_guide.state import NED, GlobalPosition, from_local_ned, to_local_ned
KYIV = GlobalPosition(50.4501, 30.5234, 120.0)


@pytest.mark.unit
def test_ned_arithmetic():
    a = NED(1.0, 2.0, 3.0)
    b = NED(0.5, 0.5, 0.5)
    assert a + b == NED(1.5, 2.5, 3.5)
    assert a - b == NED(0.5, 1.5, 2.5)
    assert a.scaled(2.0) == NED(2.0, 4.0, 6.0)


@pytest.mark.unit
def test_horizontal_norm_ignores_vertical():
    assert NED(3.0, 4.0, 100.0).horizontal_norm == pytest.approx(5.0)


@pytest.mark.unit
def test_clamp_keeps_direction_and_vertical_component():
    clamped = NED(30.0, 40.0, -1.5).clamped_horizontally(5.0)
    assert clamped.horizontal_norm == pytest.approx(5.0)
    assert clamped.north / clamped.east == pytest.approx(30.0 / 40.0)
    assert clamped.down == -1.5


@pytest.mark.unit
def test_clamp_is_a_no_op_below_the_limit():
    velocity = NED(1.0, 1.0, 0.0)
    assert velocity.clamped_horizontally(5.0) is velocity


@pytest.mark.unit
def test_clamp_survives_a_zero_vector():
    assert NED(0.0, 0.0, 2.0).clamped_horizontally(5.0) == NED(0.0, 0.0, 2.0)


@pytest.mark.unit
def test_projection_round_trip():
    offset = NED(123.0, -45.0, -20.0)
    position = from_local_ned(offset, KYIV)
    recovered = to_local_ned(position, KYIV)
    assert recovered.north == pytest.approx(offset.north, abs=1e-6)
    assert recovered.east == pytest.approx(offset.east, abs=1e-6)
    assert recovered.down == pytest.approx(offset.down, abs=1e-9)


@pytest.mark.unit
def test_origin_projects_to_zero():
    assert to_local_ned(KYIV, KYIV).horizontal_norm == pytest.approx(0.0)


@pytest.mark.unit
def test_one_degree_of_latitude_is_about_111_km():
    north = to_local_ned(GlobalPosition(51.4501, 30.5234, 120.0), KYIV).north
    assert north == pytest.approx(111_320.0, rel=1e-3)


@pytest.mark.unit
def test_longitude_scale_shrinks_with_latitude():
    east_here = to_local_ned(GlobalPosition(50.4501, 31.5234, 120.0), KYIV).east
    equator = GlobalPosition(0.0, 30.5234, 0.0)
    east_at_equator = to_local_ned(GlobalPosition(0.0, 31.5234, 0.0), equator).east
    assert east_here == pytest.approx(east_at_equator * math.cos(math.radians(50.4501)), rel=1e-9)


@pytest.mark.unit
def test_down_is_positive_below_the_origin():
    below = GlobalPosition(50.4501, 30.5234, 100.0)
    assert to_local_ned(below, KYIV).down == pytest.approx(20.0)
