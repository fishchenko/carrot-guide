import pytest

from tests.simulator import ALTITUDE_M, requires_simulator


@pytest.mark.integration
@requires_simulator
def test_takeoff_puts_the_vehicle_where_it_was_asked(flight):
    assert flight.state.armed
    assert flight.state.mode == "GUIDED"
    assert flight.state.position.alt_m == pytest.approx(ALTITUDE_M, abs=1.5)
    # The local frame is anchored at home altitude, so `down` mirrors the climb.
    assert flight.position_ned.down == pytest.approx(-ALTITUDE_M, abs=1.5)
