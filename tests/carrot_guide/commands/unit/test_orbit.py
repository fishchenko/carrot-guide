import pytest

from carrot_guide.commands.orbit import OrbitCommand

from tests.carrot_guide.commands.parsers import parse


@pytest.mark.unit
def test_orbit_defaults_are_the_ones_the_readme_quotes():
    args = parse(OrbitCommand("orbit", ""))
    assert args.radius == 25.0
    assert args.rate == 10.0
    assert args.counter_clockwise is False


@pytest.mark.unit
def test_the_orbit_run_keeps_its_settle_threshold():
    assert parse(OrbitCommand("orbit", "")).settle_threshold == 2.0


@pytest.mark.unit
def test_orbit_carries_no_derivative_gain_because_its_law_reads_none():
    """`--kd` drifted onto this parser once and was read by nothing for a campaign."""
    assert hasattr(parse(OrbitCommand("orbit", "")), "kp")
    assert not hasattr(parse(OrbitCommand("orbit", "")), "kd")
