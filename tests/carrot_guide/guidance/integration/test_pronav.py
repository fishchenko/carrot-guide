import pytest

from carrot_guide.guidance import Limits, ProNav, Target
from carrot_guide.metrics import WINDOW_WHOLE_RUN, summarise
from carrot_guide.runner import GuidanceRunner
from carrot_guide.state import NED

from tests.simulator import ALTITUDE_M, requires_simulator


@pytest.mark.integration
@requires_simulator
def test_proportional_navigation_meets_a_target_that_will_not_stay_put(vehicle):
    """The one thing the fast tests cannot show: the law works against a real autopilot.

    The toy vehicle the unit tests fly is a point mass with a first-order lag, and the
    lag is the whole mechanism `ProNav.response_s` leans on. Here the lag is whatever the link,
    the autopilot and the airframe actually add up to.
    """
    start = vehicle.position_ned
    target = Target(start=NED(50.0, -35.0, -ALTITUDE_M), velocity=NED(0.0, 3.0))
    law = ProNav(target=target, speed_mps=4.0, limits=Limits(max_horizontal_speed=6.0))
    runner = GuidanceRunner(vehicle.link, vehicle.tracker, rate_hz=10.0)
    report = runner.fly(law, duration_s=40.0, label="intercept")

    # Threshold of zero: a pass has no hold window to find, so the whole run is the
    # window and `min_error_m` is the miss distance rather than a mean over a stretch.
    summary = summarise(report.samples, settle_threshold_m=0.0)
    assert summary.window == WINDOW_WHOLE_RUN
    assert summary.min_error_m < 3.0

    # And it got there in something like the time the geometry allows, rather than
    # trailing the target across the sky the way pure pursuit does.
    #
    # `reach_time_s`, not `min_error_t_s`: the latter is the deepest approach, and on a
    # run that stays near the target afterwards it wanders to whichever dip went lowest.
    # Three repeats of this geometry reached the target at 1.09, 1.09 and 1.10 times the
    # optimum, so 1.5 leaves room for the acceleration out of the hover to vary. It is
    # also tight enough to mean something: pursuit over the same geometry runs 1.60.
    optimum = target.intercept_time(start, 4.0)
    assert optimum is not None
    assert summary.reach_time_s is not None
    assert summary.reach_time_s < optimum * 1.5
