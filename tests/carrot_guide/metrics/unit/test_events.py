import pytest

from carrot_guide.metrics import hold_start, reach_time, settle_time

from tests.runs import approach_then_hold, sample


@pytest.mark.unit
def test_settle_time_is_when_the_error_stops_violating_the_threshold():
    assert settle_time(approach_then_hold(), threshold_m=2.0) == pytest.approx(1.8, abs=1e-9)


@pytest.mark.unit
def test_a_late_excursion_means_the_run_never_settled():
    # t = 41.8 s is past the last hold sample at 41.7 s, so the series stays ordered
    # and the excursion really is late — at t = 10.0 it was neither.
    samples = approach_then_hold() + [sample(41.8, 5.0)]
    assert settle_time(samples, threshold_m=2.0) is None


@pytest.mark.unit
def test_a_run_that_starts_settled_settles_at_zero():
    samples = [sample(index * 0.1, 0.2) for index in range(10)]
    assert settle_time(samples, threshold_m=2.0) == pytest.approx(0.0)


@pytest.mark.unit
def test_the_hold_window_opens_a_lead_in_after_the_vehicle_arrives():
    samples = approach_then_hold()
    assert settle_time(samples, threshold_m=2.0) == pytest.approx(1.8, abs=1e-9)
    assert hold_start(samples, threshold_m=2.0, lead_in_s=8.0) == pytest.approx(9.8, abs=1e-9)


@pytest.mark.unit
def test_reach_time_is_the_first_approach_not_the_deepest_one():
    """The distinction the closest approach cannot make on a law that stays alongside.

    Both dips are inside the threshold and the later one is deeper, so the closest
    approach lands at 9 s while the vehicle has plainly been there since 2 s.
    """
    samples = [sample(0.0, 20.0), sample(2.0, 1.0), sample(5.0, 2.5), sample(9.0, 0.2)]
    assert reach_time(samples, threshold_m=3.0) == pytest.approx(2.0)


@pytest.mark.unit
def test_a_target_never_reached_has_no_reach_time():
    samples = [sample(float(index), 40.0) for index in range(5)]
    assert reach_time(samples, threshold_m=3.0) is None
