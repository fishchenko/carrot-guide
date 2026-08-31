import pytest

from carrot_guide.telemetry import mode_name


@pytest.mark.unit
def test_an_unmapped_mode_is_reported_rather_than_swallowed():
    assert mode_name(27) == "MODE_27"
