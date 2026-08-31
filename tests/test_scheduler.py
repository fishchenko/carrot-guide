"""Timing tests for `FixedRateLoop`, run against a fake clock.

Nothing here sleeps: the clock only moves when the test says it does, so a
twenty-minute schedule is checked in a millisecond and the assertions are exact
instead of tolerant.
"""

import pytest

from carrot_guide.runner.scheduler import FixedRateLoop, measure_sleep_overshoot

from conftest import FakeClock, SpinningClock


def make_loop(clock: FakeClock, hz: float = 10.0, spin_slack_s: float = 0.0) -> FixedRateLoop:
    return FixedRateLoop(
        hz=hz, monotonic=clock.monotonic, sleep=clock.sleep, spin_slack_s=spin_slack_s
    )


def test_loop_runs_the_expected_number_of_ticks():
    clock = FakeClock()
    loop = make_loop(clock)
    assert len([t.index for t in loop.ticks(1.0)]) == 11  # t = 0.0 .. 1.0 inclusive


def test_loop_holds_its_period_when_the_body_costs_time():
    clock = FakeClock()
    loop = make_loop(clock)
    for _ in loop.ticks(2.0):
        clock.advance(0.03)  # the body takes 30 ms of the 100 ms budget
    assert loop.periods == pytest.approx([0.1] * len(loop.periods))
    assert max(loop.lateness) == pytest.approx(0.0)


def test_loop_does_not_drift_over_a_long_run():
    clock = FakeClock()
    loop = make_loop(clock, hz=20.0)
    elapsed = []
    for tick in loop.ticks(600.0):
        elapsed.append(tick.elapsed_s)
        clock.advance(0.011)  # a body that costs a fifth of the period
    # Ten minutes at 20 Hz: every tick still lands on its own multiple of the period.
    # Sleeping for a fixed period instead would have lost 0.011 s per cycle — over two
    # minutes of drift by the end of this run.
    assert len(elapsed) == 12_001
    assert elapsed[-1] == pytest.approx(600.0, abs=1e-9)
    assert elapsed == pytest.approx([index * 0.05 for index in range(12_001)], abs=1e-9)


def test_a_body_that_overruns_shows_up_as_lateness():
    clock = FakeClock()
    loop = make_loop(clock)
    for tick in loop.ticks(0.5):
        if tick.index == 1:
            clock.advance(0.15)  # one and a half periods
        else:
            clock.advance(0.01)
    assert max(loop.lateness) == pytest.approx(0.05)


def test_a_badly_overrunning_body_resynchronises_instead_of_bursting():
    clock = FakeClock()
    loop = make_loop(clock)
    stalled = False
    ticks = []
    for tick in loop.ticks(2.0):
        ticks.append(tick.elapsed_s)
        if not stalled:
            stalled = True
            clock.advance(0.55)  # a stalled link swallows five and a half periods
        else:
            clock.advance(0.01)
    gaps = [b - a for a, b in zip(ticks, ticks[1:])]
    # One long gap for the stall, and no catch-up burst of zero-length cycles after it.
    assert max(gaps) == pytest.approx(0.55)
    assert min(gaps) == pytest.approx(0.1)


def test_a_stall_is_counted_because_lateness_cannot_see_it():
    # Re-basing the schedule moves the deadline the next tick is measured against, so a
    # stall long enough to trigger a resync reports zero lateness. Without the counters
    # a run could quote a sub-millisecond worst case having silently dropped cycles.
    clock = FakeClock()
    loop = make_loop(clock)
    stalled = False
    for _ in loop.ticks(2.0):
        if not stalled:
            stalled = True
            clock.advance(0.55)  # five and a half periods inside one body
        else:
            clock.advance(0.01)
    stats = loop.stats()
    assert stats.max_lateness_s == pytest.approx(0.0)
    assert stats.resyncs == 1
    # A 0.55 s body starting on the tick at t = 0 runs past the deadlines at 0.2, 0.3,
    # 0.4 and 0.5; the one at 0.1 is not skipped but served late, at 0.55.
    assert stats.skipped_cycles == 4
    assert stats.as_dict()["skipped_cycles"] == 4.0
    # Ticks are counted as they are served, so dropped cycles never reach the count.
    assert stats.ticks == len(list(loop.periods)) + 1


def test_an_overrun_too_small_to_resynchronise_is_still_reported_as_lateness():
    clock = FakeClock()
    loop = make_loop(clock)
    for tick in loop.ticks(0.5):
        clock.advance(0.15 if tick.index == 1 else 0.01)
    stats = loop.stats()
    assert stats.resyncs == 0
    assert stats.max_lateness_s == pytest.approx(0.05)


def test_a_clean_run_reports_no_resyncs():
    clock = FakeClock()
    loop = make_loop(clock)
    for _ in loop.ticks(1.0):
        clock.advance(0.02)
    assert loop.stats().resyncs == 0
    assert loop.stats().skipped_cycles == 0


def test_the_loop_sleeps_away_only_the_time_the_body_left_over():
    clock = FakeClock()
    loop = make_loop(clock, spin_slack_s=0.0)
    for _ in loop.ticks(0.3):
        clock.advance(0.02)
    assert clock.sleeps == pytest.approx([0.08] * len(clock.sleeps))
    assert len(clock.sleeps) >= 3


def test_spin_slack_is_taken_off_the_sleep_and_spun_through():
    clock = SpinningClock()
    loop = make_loop(clock, spin_slack_s=0.015)
    for _ in loop.ticks(0.3):
        clock.advance(0.02)
    # 100 ms period, 20 ms body, 15 ms held back to spin through instead of sleeping.
    assert clock.sleeps[0] == pytest.approx(0.065, abs=0.005)
    assert max(loop.lateness) < 0.005


def test_stats_describe_an_ideal_run():
    clock = FakeClock()
    loop = make_loop(clock)
    for _ in loop.ticks(1.0):
        clock.advance(0.02)
    stats = loop.stats()
    # Ticks served, not the intervals between them — the same 11 the loop yielded above.
    assert stats.ticks == 11
    assert stats.mean_period_s == pytest.approx(0.1)
    assert stats.jitter_stdev_s == pytest.approx(0.0)
    assert stats.as_dict()["mean_hz"] == pytest.approx(10.0)


def test_stats_on_an_unstarted_loop_are_empty_rather_than_a_crash():
    stats = FixedRateLoop(10.0).stats()
    assert stats.ticks == 0
    assert stats.as_dict()["mean_hz"] == 0.0


def test_sleep_overshoot_is_measured_not_assumed():
    clock = FakeClock(sleep_overshoot=0.009)
    overshoot = measure_sleep_overshoot(
        samples=4, request_s=0.09, monotonic=clock.monotonic, sleep=clock.sleep
    )
    assert overshoot == pytest.approx(0.009)


def test_calibrated_slack_never_eats_the_whole_period(monkeypatch):
    monkeypatch.setattr("carrot_guide.runner.scheduler.measure_sleep_overshoot", lambda **_: 10.0)
    loop = FixedRateLoop.calibrated(10.0)
    assert loop.spin_slack_s == pytest.approx(0.1 * FixedRateLoop.MAX_SPIN_SHARE)
