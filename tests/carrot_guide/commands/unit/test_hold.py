import pytest

from carrot_guide.commands.hold import HoldCommand

from tests.carrot_guide.commands.parsers import parse


@pytest.mark.unit
def test_the_hold_run_keeps_its_settle_threshold():
    assert parse(HoldCommand("hold", "")).settle_threshold == 2.0


@pytest.mark.unit
def test_hold_carries_both_gains_because_its_law_reads_both():
    args = parse(HoldCommand("hold", ""))
    assert hasattr(args, "kp")
    assert hasattr(args, "kd")
