import pytest

from carrot_guide.guidance import HoldPoint, Limits
from carrot_guide.metrics import summarise
from carrot_guide.runner import GuidanceRunner
from carrot_guide.state import NED

from tests.simulator import ALTITUDE_M, requires_simulator


@pytest.mark.integration
@requires_simulator
def test_the_vehicle_holds_a_point_it_had_to_fly_to(vehicle):
    law = HoldPoint(
        target=NED(20.0, -15.0, -ALTITUDE_M),
        limits=Limits(max_horizontal_speed=5.0),
    )
    runner = GuidanceRunner(vehicle.link, vehicle.tracker, rate_hz=10.0)
    report = runner.fly(law, duration_s=30.0, label="hold")

    summary = summarise(report.samples, settle_threshold_m=2.0)
    assert summary.settle_time_s is not None, "never got within 2 m of the target"
    assert summary.mean_error_m < 1.0
    assert report.loop.as_dict()["mean_hz"] == pytest.approx(10.0, abs=0.1)


@pytest.mark.integration
@requires_simulator
def test_the_vehicle_holds_the_point_with_the_wind_on(vehicle):
    vehicle.link.set_param("SIM_WIND_SPD", 6.0)
    vehicle.link.set_param("SIM_WIND_DIR", 90.0)
    try:
        law = HoldPoint(target=NED(0.0, 0.0, -ALTITUDE_M))
        runner = GuidanceRunner(vehicle.link, vehicle.tracker, rate_hz=10.0)
        report = runner.fly(law, duration_s=30.0, label="hold-wind")
        summary = summarise(report.samples, settle_threshold_m=2.0)
        assert summary.mean_error_m < 1.0
    finally:
        vehicle.link.set_param("SIM_WIND_SPD", 0.0)
