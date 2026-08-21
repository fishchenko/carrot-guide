import math

import pytest

from carrot_guide.metrics import (
    WINDOW_HOLD,
    WINDOW_POST_SETTLE,
    WINDOW_WHOLE_RUN,
    hold_start,
    settle_time,
    summarise,
    summarise_latency,
)
from carrot_guide.recording import Sample


def sample(t: float, error: float, vn: float = 0.0, ve: float = 0.0) -> Sample:
    return Sample(
        t_s=t,
        label="test",
        lat_deg=50.0,
        lon_deg=30.0,
        north_m=0.0,
        east_m=0.0,
        down_m=-20.0,
        vn=vn,
        ve=ve,
        vd=0.0,
        cmd_vn=0.0,
        cmd_ve=0.0,
        cmd_vd=0.0,
        error_m=error,
        lateness_ms=0.0,
        mode="GUIDED",
        armed=True,
    )


def approach_then_hold(hold_error: float = 0.4, seconds: float = 40.0) -> list[Sample]:
    """A run shaped like a real one: an approach, an exponential tail, then a steady hold.

    Eighteen samples close from 20 m to 3 m, the error crosses the 2 m threshold at
    t = 1.8 s, and from there it decays towards `hold_error` with the outer loop's own
    1.25 s time constant. The tail is the point: it is what a window opening at the
    threshold crossing averages in, and what the lead-in exists to leave out.
    """
    approach = [sample(index * 0.1, 20.0 - index) for index in range(18)]
    hold = [
        sample(1.8 + index * 0.1, hold_error + 1.5 * math.exp(-(index * 0.1) / 1.25))
        for index in range(int(seconds / 0.1))
    ]
    return approach + hold


def test_settle_time_is_when_the_error_stops_violating_the_threshold():
    assert settle_time(approach_then_hold(), threshold_m=2.0) == pytest.approx(1.8, abs=1e-9)


def test_a_late_excursion_means_the_run_never_settled():
    # t = 41.8 s is past the last hold sample at 41.7 s, so the series stays ordered
    # and the excursion really is late — at t = 10.0 it was neither.
    samples = approach_then_hold() + [sample(41.8, 5.0)]
    assert settle_time(samples, threshold_m=2.0) is None


def test_a_run_that_starts_settled_settles_at_zero():
    samples = [sample(index * 0.1, 0.2) for index in range(10)]
    assert settle_time(samples, threshold_m=2.0) == pytest.approx(0.0)


def test_the_hold_window_opens_a_lead_in_after_the_vehicle_arrives():
    samples = approach_then_hold()
    assert settle_time(samples, threshold_m=2.0) == pytest.approx(1.8, abs=1e-9)
    assert hold_start(samples, threshold_m=2.0, lead_in_s=8.0) == pytest.approx(9.8, abs=1e-9)


def test_summary_reports_the_hold_and_not_the_tail_of_the_approach():
    summary = summarise(approach_then_hold(hold_error=0.4), settle_threshold_m=2.0)
    assert summary.window == WINDOW_HOLD
    assert summary.settle_time_s == pytest.approx(1.8, abs=1e-9)
    assert summary.hold_start_s == pytest.approx(9.8, abs=1e-9)
    # Neither the 20 m start nor the decay behind it survives into the average.
    assert summary.mean_error_m == pytest.approx(0.4, abs=1e-3)
    assert summary.max_error_m == pytest.approx(0.4, abs=1e-2)


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


def test_a_run_too_short_for_a_hold_window_says_which_window_it_used():
    samples = approach_then_hold(hold_error=0.4, seconds=3.0)
    summary = summarise(samples, settle_threshold_m=2.0)
    assert summary.window == WINDOW_POST_SETTLE
    assert summary.hold_start_s is None
    assert summary.settle_time_s == pytest.approx(1.8, abs=1e-9)


def test_summary_falls_back_to_the_whole_run_when_it_never_settled():
    samples = [sample(index * 0.1, 10.0) for index in range(10)]
    summary = summarise(samples, settle_threshold_m=2.0)
    assert summary.window == WINDOW_WHOLE_RUN
    assert summary.settle_time_s is None
    assert summary.hold_start_s is None
    assert summary.measured_samples == 10
    assert summary.mean_error_m == pytest.approx(10.0)


def test_rms_error_punishes_spikes_harder_than_the_mean():
    samples = [sample(0.0, 0.0), sample(0.1, 4.0)]
    summary = summarise(samples, settle_threshold_m=10.0)
    assert summary.mean_error_m == pytest.approx(2.0)
    assert summary.rms_error_m == pytest.approx(math.sqrt(8.0))


def test_max_speed_is_horizontal_ground_speed():
    samples = [sample(0.0, 0.0, vn=3.0, ve=4.0), sample(0.1, 0.0, vn=1.0)]
    assert summarise(samples).max_speed_mps == pytest.approx(5.0)


def test_an_empty_run_is_an_error():
    with pytest.raises(ValueError):
        summarise([])


def test_latency_summary():
    summary = summarise_latency([0.2, 0.1, 0.3])
    assert summary.trials == 3
    assert summary.median_ms == pytest.approx(200.0)
    assert summary.min_ms == pytest.approx(100.0)
    assert summary.max_ms == pytest.approx(300.0)


def test_latency_summary_needs_at_least_one_trial():
    with pytest.raises(ValueError):
        summarise_latency([])
