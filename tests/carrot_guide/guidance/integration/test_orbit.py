import pytest

from carrot_guide.guidance import Limits, Orbit
from carrot_guide.metrics import summarise
from carrot_guide.runner import GuidanceRunner
from carrot_guide.state import NED

from tests.simulator import ALTITUDE_M, requires_simulator


@pytest.mark.integration
@requires_simulator
def test_the_vehicle_flies_a_circle_of_the_requested_radius(vehicle):
    law = Orbit(
        centre=NED(0.0, 0.0, -ALTITUDE_M),
        radius_m=20.0,
        speed_mps=4.0,
        limits=Limits(max_horizontal_speed=6.0),
    )
    runner = GuidanceRunner(vehicle.link, vehicle.tracker, rate_hz=10.0)
    report = runner.fly(law, duration_s=45.0, label="orbit")

    summary = summarise(report.samples, settle_threshold_m=2.0)
    assert summary.settle_time_s is not None, "never reached the circle"
    settled = [s for s in report.samples if s.t_s >= summary.settle_time_s]
    assert max(s.error_m for s in settled) < 2.0

    # And it went round, rather than parking on the circle at one bearing.
    quadrants = {(s.north_m > 0.0, s.east_m > 0.0) for s in settled}
    assert len(quadrants) == 4
