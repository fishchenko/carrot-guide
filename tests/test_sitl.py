"""Integration tests against a running ArduPilot SITL instance.

Skipped unless `CARROT_SITL_URL` is set, so `pytest` in a bare checkout still runs
the whole unit suite. Start the simulator with `docker compose up -d sitl` and point
the variable at it, or run `make test-sitl`.

These are slow (a real takeoff, in real time) and there are deliberately few of them:
the maths is already covered by the fast tests, so what is left to prove here is that
the MAVLink side agrees with a real autopilot.
"""

from __future__ import annotations

import os

import pytest

from carrot_guide.guidance import HoldPoint, Limits, Orbit
from carrot_guide.metrics import summarise
from carrot_guide.mission import airborne, measure_command_latency
from carrot_guide.runner import GuidanceRunner
from carrot_guide.state import NED

pytestmark = pytest.mark.sitl

SITL_URL = os.environ.get("CARROT_SITL_URL")

requires_sitl = pytest.mark.skipif(
    not SITL_URL, reason="set CARROT_SITL_URL to run the simulator tests"
)

ALTITUDE_M = 15.0


@pytest.fixture(scope="module")
def vehicle():
    with airborne(SITL_URL or "", altitude_m=ALTITUDE_M) as flying:
        yield flying


@requires_sitl
def test_takeoff_puts_the_vehicle_where_it_was_asked(vehicle):
    state = vehicle.tracker.snapshot()
    assert state.armed
    assert state.mode == "GUIDED"
    assert state.position.alt_m == pytest.approx(ALTITUDE_M, abs=1.5)
    # The local frame is anchored at home altitude, so `down` mirrors the climb.
    assert vehicle.position_ned.down == pytest.approx(-ALTITUDE_M, abs=1.5)


@requires_sitl
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


@requires_sitl
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


@requires_sitl
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


@requires_sitl
def test_a_velocity_command_is_acted_on_within_a_second(vehicle):
    latencies = measure_command_latency(vehicle, trials=3, step_speed_mps=3.0)
    assert len(latencies) == 3
    assert max(latencies) < 1.0
