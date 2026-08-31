import math

import pytest

from carrot_guide.metrics import (
    WINDOW_HOLD,
    WINDOW_POST_SETTLE,
    WINDOW_WHOLE_RUN,
    summarise,
)

from tests.runs import approach_then_hold, sample


@pytest.mark.unit
def test_summary_reports_the_hold_and_not_the_tail_of_the_approach():
    summary = summarise(approach_then_hold(hold_error=0.4), settle_threshold_m=2.0)
    assert summary.window == WINDOW_HOLD
    assert summary.settle_time_s == pytest.approx(1.8, abs=1e-9)
    assert summary.hold_start_s == pytest.approx(9.8, abs=1e-9)
    # Neither the 20 m start nor the decay behind it survives into the average.
    assert summary.mean_error_m == pytest.approx(0.4, abs=1e-3)
    assert summary.max_error_m == pytest.approx(0.4, abs=1e-2)


@pytest.mark.unit
def test_a_window_opening_at_the_crossing_would_have_measured_the_approach_instead():
    # Why the lead-in exists: the same run summarised from the threshold crossing
    # reports several times the error the vehicle was actually holding to. This is the
    # regression that made a 60 s run look four times worse than a 600 s one.
    # A hold error two orders of magnitude under the settle threshold, as on the real
    # runs: that ratio is what makes the decay dominate the average.
    samples = approach_then_hold(hold_error=0.01)
    honest = summarise(samples, settle_threshold_m=2.0)
    naive = summarise(samples, settle_threshold_m=2.0, hold_lead_in_s=0.0)
    assert naive.mean_error_m > 3.0 * honest.mean_error_m


@pytest.mark.unit
def test_a_run_too_short_for_a_hold_window_says_which_window_it_used():
    samples = approach_then_hold(hold_error=0.4, seconds=3.0)
    summary = summarise(samples, settle_threshold_m=2.0)
    assert summary.window == WINDOW_POST_SETTLE
    assert summary.hold_start_s is None
    assert summary.settle_time_s == pytest.approx(1.8, abs=1e-9)


@pytest.mark.unit
def test_summary_falls_back_to_the_whole_run_when_it_never_settled():
    samples = [sample(index * 0.1, 10.0) for index in range(10)]
    summary = summarise(samples, settle_threshold_m=2.0)
    assert summary.window == WINDOW_WHOLE_RUN
    assert summary.settle_time_s is None
    assert summary.hold_start_s is None
    assert summary.measured_samples == 10
    assert summary.mean_error_m == pytest.approx(10.0)


@pytest.mark.unit
def test_rms_error_punishes_spikes_harder_than_the_mean():
    samples = [sample(0.0, 0.0), sample(0.1, 4.0)]
    summary = summarise(samples, settle_threshold_m=10.0)
    assert summary.mean_error_m == pytest.approx(2.0)
    assert summary.rms_error_m == pytest.approx(math.sqrt(8.0))


@pytest.mark.unit
def test_max_speed_is_horizontal_ground_speed():
    samples = [sample(0.0, 0.0, vn=3.0, ve=4.0), sample(0.1, 0.0, vn=1.0)]
    assert summarise(samples).max_speed_mps == pytest.approx(5.0)


@pytest.mark.unit
def test_an_empty_run_is_an_error():
    with pytest.raises(ValueError):
        summarise([])


@pytest.mark.unit
def test_the_summary_reports_the_closest_the_vehicle_ever_came_and_when():
    samples = [sample(0.0, 20.0), sample(1.0, 3.0), sample(2.0, 0.4), sample(3.0, 7.0)]
    summary = summarise(samples, settle_threshold_m=0.0)
    assert summary.min_error_m == pytest.approx(0.4)
    assert summary.min_error_t_s == pytest.approx(2.0)


@pytest.mark.unit
def test_the_summary_carries_the_first_approach_rather_than_the_deepest_one():
    samples = [sample(0.0, 20.0), sample(2.0, 1.0), sample(5.0, 2.5), sample(9.0, 0.2)]
    summary = summarise(samples, settle_threshold_m=0.0, reach_threshold_m=3.0)
    assert summary.reach_time_s == pytest.approx(2.0)
    assert summary.min_error_t_s == pytest.approx(9.0)


@pytest.mark.unit
def test_a_target_never_reached_has_no_reach_time_in_the_summary():
    samples = [sample(float(index), 40.0) for index in range(5)]
    assert summarise(samples, settle_threshold_m=0.0).reach_time_s is None


@pytest.mark.unit
def test_a_threshold_of_zero_leaves_the_whole_run_as_the_window():
    """How an intercept is summarised: a pass has no hold, so there is nothing to find.

    Without this the last sample under the threshold would open a `hold` window over
    the far side of the pass, and the run would average its own retreat.
    """
    samples = [sample(0.0, 20.0), sample(1.0, 3.0), sample(2.0, 0.4), sample(3.0, 7.0)]
    summary = summarise(samples, settle_threshold_m=0.0)
    assert summary.window == WINDOW_WHOLE_RUN
    assert summary.settle_time_s is None
    assert summary.measured_samples == 4


@pytest.mark.unit
def test_the_closest_approach_is_taken_over_the_named_window_like_everything_else():
    """`window` has to describe every error field, or one of them means something else."""
    samples = approach_then_hold(hold_error=0.4)
    summary = summarise(samples)
    assert summary.window == WINDOW_HOLD
    measured = [s for s in samples if s.t_s >= summary.hold_start_s]
    assert summary.min_error_m == pytest.approx(min(s.error_m for s in measured))
