import pytest

from carrot_guide.commands.intercept import InterceptCommand
from carrot_guide.guidance import DEFAULT_RESPONSE_S

from tests.carrot_guide.commands.parsers import parse
INTERCEPT = InterceptCommand("intercept", "")


@pytest.mark.unit
def test_intercept_defaults_to_proportional_navigation_over_the_whole_run():
    args = parse(INTERCEPT)
    assert args.law == "pronav"
    # Zero, so the summary window is `whole run`: an intercept never settles anywhere.
    assert args.settle_threshold == 0.0
    # The measured command-to-reaction latency, the same number `--lookahead` is set to.
    assert args.response == DEFAULT_RESPONSE_S


@pytest.mark.unit
def test_intercept_carries_no_horizontal_gain_because_neither_law_reads_one():
    assert not hasattr(parse(INTERCEPT), "kp")


@pytest.mark.unit
def test_the_target_track_comes_from_the_course_it_was_given():
    args = parse(
        INTERCEPT,
        ["--north", "50", "--east", "0", "--target-speed", "3", "--target-heading", "90"],
    )
    target = INTERCEPT._target(args)
    assert target.velocity.north == pytest.approx(0.0, abs=1e-9)
    assert target.velocity.east == pytest.approx(3.0)
    assert target.at(10.0).east == pytest.approx(30.0)


@pytest.mark.unit
def test_the_intercept_summary_carries_the_geometric_optimum():
    """The yardstick the measured time is read against, worked out from the run's own
    parameters rather than quoted from somewhere."""
    args = parse(
        INTERCEPT,
        [
            "--north", "40",
            "--east", "0",
            "--target-speed", "3",
            "--target-heading", "90",
            "--speed", "5",
        ],
    )
    described = INTERCEPT.describe(args)
    assert described["law"] == "pronav"
    assert described["optimal_intercept_s"] == pytest.approx(10.0)
