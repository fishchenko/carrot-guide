import pytest

from carrot_guide.utils import percentile


@pytest.mark.unit
def test_a_percentile_is_always_a_value_the_run_produced():
    values = [3.0, 1.0, 2.0, 5.0, 4.0]
    assert percentile(values, 0.95) in values
    assert percentile(values, 0.5) in values
    assert percentile(values, 0.0) == 1.0


@pytest.mark.unit
def test_an_empty_series_summarises_rather_than_raising():
    """`runner.LoopStats` summarises a loop that never ticked; it must not crash there.

    This is also why `statistics.quantiles` is not used: it raises below two points.
    """
    assert percentile([], 0.95) == 0.0
