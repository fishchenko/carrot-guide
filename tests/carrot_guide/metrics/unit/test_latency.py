import pytest

from carrot_guide.metrics import summarise_latency


@pytest.mark.unit
def test_latency_summary():
    summary = summarise_latency([0.2, 0.1, 0.3])
    assert summary.trials == 3
    assert summary.median_ms == pytest.approx(200.0)
    assert summary.min_ms == pytest.approx(100.0)
    assert summary.max_ms == pytest.approx(300.0)


@pytest.mark.unit
def test_latency_summary_needs_at_least_one_trial():
    with pytest.raises(ValueError):
        summarise_latency([])
