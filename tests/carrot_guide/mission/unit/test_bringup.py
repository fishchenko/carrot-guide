"""Tests for the launch sequence, against a vehicle that misbehaves on purpose.

Every failure reproduced here was seen coming out of a real cold simulator: GUIDED
refused outright, GUIDED accepted and then dropped while arming, takeoff refused
after a successful arm. They are cheap to fake and expensive to rediscover.
"""

import pytest

from carrot_guide.link import CommandFailed
from carrot_guide.mission import launch
from carrot_guide.telemetry import TelemetryTracker

from tests.doubles.vehicles import ColdVehicle


@pytest.mark.unit
def test_a_cooperative_vehicle_takes_off_on_the_first_attempt():
    vehicle, tracker = ColdVehicle(), TelemetryTracker()
    launch(vehicle, tracker, altitude_m=15.0, timeout_s=5.0)
    assert vehicle.calls == ["set_mode:GUIDED", "arm", "takeoff:15"]


@pytest.mark.unit
def test_a_refused_mode_is_retried_rather_than_fatal():
    vehicle, tracker = ColdVehicle(mode_refusals=2), TelemetryTracker()
    launch(vehicle, tracker, altitude_m=15.0, timeout_s=5.0)
    assert vehicle.calls.count("set_mode:GUIDED") == 3
    assert vehicle.calls[-1] == "takeoff:15"


@pytest.mark.unit
def test_a_mode_that_falls_back_while_arming_restarts_the_sequence():
    vehicle, tracker = ColdVehicle(fallbacks_while_arming=1), TelemetryTracker()
    launch(vehicle, tracker, altitude_m=15.0, timeout_s=5.0)
    # No takeoff was commanded in the wrong mode: the whole sequence began again.
    assert vehicle.calls == ["set_mode:GUIDED", "arm", "set_mode:GUIDED", "arm", "takeoff:15"]


@pytest.mark.unit
def test_a_refused_takeoff_leads_to_arming_again_not_to_asking_again():
    vehicle, tracker = ColdVehicle(takeoff_refusals=1), TelemetryTracker()
    launch(vehicle, tracker, altitude_m=15.0, timeout_s=5.0)
    assert vehicle.calls == [
        "set_mode:GUIDED",
        "arm",
        "takeoff:15",
        "set_mode:GUIDED",
        "arm",
        "takeoff:15",
    ]


@pytest.mark.unit
def test_giving_up_says_what_the_last_refusal_was():
    vehicle, tracker = ColdVehicle(takeoff_refusals=float("inf")), TelemetryTracker()
    tracker.status_texts.append("PreArm: Gyros inconsistent")

    with pytest.raises(CommandFailed) as failure:
        launch(vehicle, tracker, altitude_m=15.0, timeout_s=0.2)

    message = str(failure.value)
    assert "could not get airborne" in message
    assert "command 22 refused" in message
    assert "Gyros inconsistent" in message
