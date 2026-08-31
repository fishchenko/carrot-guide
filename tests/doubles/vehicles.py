"""The two stand-ins for `MavlinkLink`: one flies its commands, one refuses them.

`FakeVehicle` integrates whatever velocity it is commanded and publishes the result to
the tracker, which is what makes a closed loop testable without a socket. `ColdVehicle`
answers the way a cold autopilot does — GUIDED refused, GUIDED dropped while arming,
takeoff refused after a successful arm — every one of them seen on a real simulator.
"""

from __future__ import annotations

from carrot_guide.link import CommandFailed
from carrot_guide.state import NED, GlobalPosition, from_local_ned
from carrot_guide.telemetry import TelemetryTracker

from tests.doubles.clocks import FakeClock

# Kyiv, the simulator's home: the frame `FakeVehicle` publishes its position in.
ORIGIN = GlobalPosition(50.4501, 30.5234, 0.0)


class FakeVehicle:
    """Stands in for `MavlinkLink`, integrating commands into the tracker's state."""

    def __init__(
        self,
        tracker: TelemetryTracker,
        dt: float,
        start: NED,
        goes_silent_after: int | None = None,
    ) -> None:
        self.tracker = tracker
        self.dt = dt
        self.position = start
        self.commands: list[tuple[NED, float | None]] = []
        # Cycles to fly before the position stream dies, for the failsafe test.
        self.goes_silent_after = goes_silent_after
        self.silent = False
        self._publish(NED(0.0, 0.0, 0.0))

    def _publish(self, velocity: NED) -> None:
        if self.silent:
            return
        self.tracker.position = from_local_ned(self.position, ORIGIN)
        self.tracker.velocity = velocity
        self.tracker.armed = True
        self.tracker.mode = "GUIDED"
        self.tracker.position_updates += 1

    def drain(self, tracker: TelemetryTracker, budget_s: float = 0.0) -> int:
        return 0 if self.silent else 1

    def send_velocity(self, velocity: NED, yaw_deg: float | None = None) -> None:
        self.commands.append((velocity, yaw_deg))
        self.position = self.position + velocity.scaled(self.dt)
        self._publish(velocity)
        if len(self.commands) == self.goes_silent_after:
            self.silent = True


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
        self.clock = FakeClock()
        self.monotonic = self.clock.monotonic  # `launch` times itself off the link's clock

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

    def takeoff(
        self, altitude_m, tracker, timeout_s=60.0, accept_timeout_s=60.0, retry_every_s=2.0
    ):
        self.calls.append(f"takeoff:{altitude_m:g}")
        if self.takeoff_refusals > 0:
            self.takeoff_refusals -= 1
            tracker.armed = False  # it disarmed itself while being asked
            raise CommandFailed("command 22 refused 3 times in 6 s")

    def drain(self, tracker, budget_s=0.0):
        # `launch` uses a drain as its backoff, so draining for a budget has to cost it.
        self.clock.advance(budget_s)
        return 0
