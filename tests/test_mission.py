"""Tests for the launch sequence, against a vehicle that misbehaves on purpose.

Every failure reproduced here was seen coming out of a real cold simulator: GUIDED
refused outright, GUIDED accepted and then dropped while arming, takeoff refused
after a successful arm. They are cheap to fake and expensive to rediscover.
"""

import pytest

from carrot_guide.link import CommandFailed
from carrot_guide.mission import launch
from carrot_guide.telemetry import TelemetryTracker


class ColdVehicle:
    """A link-shaped stub whose early answers are refusals."""

    def __init__(
        self,
        mode_refusals: int = 0,
        fallbacks_while_arming: int = 0,
        takeoff_refusals: float = 0,  # float so a test can pass inf for "always"
    ) -> None:
        self.mode_refusals = mode_refusals
        self.fallbacks_while_arming = fallbacks_while_arming
        self.takeoff_refusals = takeoff_refusals
        self.calls: list[str] = []

    def set_mode(self, name, tracker, timeout_s=10.0):
        self.calls.append(f"set_mode:{name}")
        if self.mode_refusals > 0:
            self.mode_refusals -= 1
            raise CommandFailed(f"timed out after {timeout_s:g} s waiting for mode {name}")
        tracker.mode = name

    def arm(self, tracker, timeout_s=90.0, retry_every_s=2.0):
        self.calls.append("arm")
        if self.fallbacks_while_arming > 0:
            self.fallbacks_while_arming -= 1
            # The estimator gave up on GUIDED while the motors were being armed.
            tracker.mode = "STABILIZE"
        tracker.armed = True

    def takeoff(self, altitude_m, tracker, timeout_s=60.0, accept_timeout_s=60.0, retry_every_s=2.0):
        self.calls.append(f"takeoff:{altitude_m:g}")
        if self.takeoff_refusals > 0:
            self.takeoff_refusals -= 1
            tracker.armed = False  # it disarmed itself while being asked
            raise CommandFailed("command 22 refused 3 times in 6 s")

    def drain(self, tracker, budget_s=0.0):
        return 0


def test_a_cooperative_vehicle_takes_off_on_the_first_attempt():
    vehicle, tracker = ColdVehicle(), TelemetryTracker()
    launch(vehicle, tracker, altitude_m=15.0, timeout_s=5.0)
    assert vehicle.calls == ["set_mode:GUIDED", "arm", "takeoff:15"]


def test_a_refused_mode_is_retried_rather_than_fatal():
    vehicle, tracker = ColdVehicle(mode_refusals=2), TelemetryTracker()
    launch(vehicle, tracker, altitude_m=15.0, timeout_s=5.0)
    assert vehicle.calls.count("set_mode:GUIDED") == 3
    assert vehicle.calls[-1] == "takeoff:15"


def test_a_mode_that_falls_back_while_arming_restarts_the_sequence():
    vehicle, tracker = ColdVehicle(fallbacks_while_arming=1), TelemetryTracker()
    launch(vehicle, tracker, altitude_m=15.0, timeout_s=5.0)
    # No takeoff was commanded in the wrong mode: the whole sequence began again.
    assert vehicle.calls == ["set_mode:GUIDED", "arm", "set_mode:GUIDED", "arm", "takeoff:15"]


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


def test_giving_up_says_what_the_last_refusal_was():
    vehicle, tracker = ColdVehicle(takeoff_refusals=float("inf")), TelemetryTracker()
    tracker.status_texts.append("PreArm: Gyros inconsistent")

    with pytest.raises(CommandFailed) as failure:
        launch(vehicle, tracker, altitude_m=15.0, timeout_s=0.2)

    message = str(failure.value)
    assert "could not get airborne" in message
    assert "command 22 refused" in message
    assert "Gyros inconsistent" in message
