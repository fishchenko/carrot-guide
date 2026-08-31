import pytest

from carrot_guide.commands.report import ReportCommand
from carrot_guide.state import NED


@pytest.mark.unit
@pytest.mark.parametrize(
    "text, expected",
    [
        (None, (None, None)),
        ("", (None, None)),
        ("25,0", (NED(25.0, 0.0), None)),
        ("0,0,25", (NED(0.0, 0.0), 25.0)),
        ("-10.5,3,12.25", (NED(-10.5, 3.0), 12.25)),
    ],
)
def test_circle_overlay_parsing(text, expected):
    assert ReportCommand._parse_circle(text) == expected


@pytest.mark.unit
def test_a_malformed_circle_is_refused():
    with pytest.raises(SystemExit):
        ReportCommand._parse_circle("1,2,3,4")
