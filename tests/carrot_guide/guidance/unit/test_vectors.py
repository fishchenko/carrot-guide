import pytest

from carrot_guide.guidance import bearing_deg
from carrot_guide.state import NED


@pytest.mark.unit
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
