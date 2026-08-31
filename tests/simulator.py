"""What every test against a running ArduPilot SITL instance needs to know about it.

Skipped unless `CARROT_SITL_URL` is set, so `pytest` in a bare checkout still runs the
whole unit suite. Start the simulator with `docker compose up -d sitl` and point the
variable at it, or run `make test-sitl`.

These are slow (a real takeoff, in real time) and there are deliberately few of them:
the maths is already covered by the fast tests, so what is left to prove here is that
the MAVLink side agrees with a real autopilot.
"""

from __future__ import annotations

import os
from typing import NamedTuple

import pytest

from carrot_guide.mission import Vehicle
from carrot_guide.state import NED, VehicleState

SITL_URL = os.environ.get("CARROT_SITL_URL")

ALTITUDE_M = 15.0

requires_simulator = pytest.mark.skipif(
    not SITL_URL, reason="set CARROT_SITL_URL to run the simulator tests"
)


class Flight(NamedTuple):
    """The airborne vehicle, plus the state bringup left behind.

    The takeoff state is carried rather than read on demand because the test that
    asserts on it is not necessarily the first one to fly.
    """

    vehicle: Vehicle
    state: VehicleState
    position_ned: NED
