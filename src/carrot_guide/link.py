"""MAVLink transport: the only module that talks to a socket.

Everything above it (guidance, telemetry parsing, the loop scheduler) works on plain
values, so this is the single seam that has to be faked or replaced in tests.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Callable, Iterable

from pymavlink import mavutil

from carrot_guide.state import NED
from carrot_guide.telemetry import MODE_NUMBER_BY_NAME, TelemetryTracker
from carrot_guide.utils import Deadline

# SET_POSITION_TARGET_LOCAL_NED type masks. A set bit means "ignore this field", so
# both masks keep the velocity triplet and drop position, acceleration and yaw rate.
IGNORE_ALL_BUT_VELOCITY = 0b110111000111
IGNORE_ALL_BUT_VELOCITY_AND_YAW = 0b100111000111

# Messages the control loop depends on, with the rate each is asked to arrive at.
STREAM_RATES_HZ: dict[str, float] = {
    "GLOBAL_POSITION_INT": 20.0,
    "HEARTBEAT": 2.0,
    "SYS_STATUS": 2.0,
}

MESSAGE_IDS: dict[str, int] = {
    "GLOBAL_POSITION_INT": 33,
    "HEARTBEAT": 0,
    "SYS_STATUS": 1,
}

# Capped well under any wait it serves, so a lapsed deadline is noticed when it lapses
# rather than one whole read later.
MAX_BLOCKING_READ_S = 0.5

# One spelling for the whole project: the default here used to be a seventh endpoint
# ("udp:127.0.0.1:14550") that nothing else named.
DEFAULT_SIMULATOR_URL = "tcp:127.0.0.1:5760"

# MAVLink's documented token for param2 of MAV_CMD_COMPONENT_ARM_DISARM: disarm even
# though the vehicle believes it is still flying.
FORCE_DISARM_MAGIC = 21196.0


class LinkError(RuntimeError):
    """Connection or protocol level failure."""


class CommandFailed(LinkError):
    """The vehicle rejected a command, or never acknowledged it."""


@dataclass
class MavlinkLink:
    """Thin, blocking wrapper over a pymavlink connection.

    Commands are sent and then confirmed — either by their COMMAND_ACK or by the
    state change they are supposed to cause. A fire-and-forget wrapper reads fine in
    a demo and then hides every failure the moment the simulator is under load.
    """

    connection: Any
    ack_timeout_s: float = 5.0

    @classmethod
    def connect(cls, url: str = DEFAULT_SIMULATOR_URL, timeout_s: float = 60.0) -> "MavlinkLink":
        connection = mavutil.mavlink_connection(url, autoreconnect=True)
        link = cls(connection)
        link.wait_heartbeat(timeout_s)
        return link

    # -- plumbing ---------------------------------------------------------------

    @property
    def target(self) -> tuple[int, int]:
        return self.connection.target_system, self.connection.target_component

    def wait_heartbeat(self, timeout_s: float = 60.0) -> None:
        if self.connection.wait_heartbeat(timeout=timeout_s) is None:
            raise LinkError(f"no HEARTBEAT within {timeout_s:g} s")

    def request_streams(self, rates_hz: dict[str, float] | None = None) -> None:
        """Ask for the messages the loop needs at the rate it needs them."""
        for name, rate in (rates_hz or STREAM_RATES_HZ).items():
            interval_us = int(1e6 / rate)
            self._send_command(
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                MESSAGE_IDS[name],
                interval_us,
                require_ack=False,
            )

    def drain(self, tracker: TelemetryTracker, budget_s: float = 0.0) -> int:
        """Feed every message already queued into `tracker`; return how many.

        With `budget_s` at zero this never blocks, which is what the control loop
        wants: read what has arrived, act on it, and keep the period stable.
        """
        deadline = Deadline.after(budget_s)
        count = 0
        while True:
            message = self.connection.recv_match(blocking=False)
            if message is None:
                if deadline.expired:
                    return count
                time.sleep(0.001)
                continue
            tracker.handle(message)
            count += 1

    def _recv_matching(
        self,
        kind: str,
        timeout_s: float,
        tracker: TelemetryTracker | None = None,
    ) -> Any | None:
        """Wait for one message of `kind`, feeding everything else to `tracker`.

        pymavlink's own type filter is destructive: `recv_match(type=...)` consumes and
        throws away every message that does not match while it looks for one that does.
        Waiting for an ACK that way quietly ate the STATUSTEXT sitting next to it in the
        same burst — which is the one message that says *why* a command was refused, and
        the one the failure paths here promise to quote. So the wait reads everything and
        filters afterwards, leaving the tracker as the single place messages accumulate.
        """
        deadline = Deadline.after(timeout_s)
        while not deadline.expired:
            message = self.connection.recv_match(
                blocking=True, timeout=deadline.slice(MAX_BLOCKING_READ_S)
            )
            if message is None:
                continue
            if tracker is not None:
                tracker.handle(message)
            if message.get_type() == kind:
                return message
        return None

    def wait_for(
        self,
        predicate: Callable[[TelemetryTracker], bool],
        tracker: TelemetryTracker,
        timeout_s: float,
        description: str,
    ) -> None:
        deadline = Deadline.after(timeout_s)
        while not deadline.expired:
            self.drain(tracker, budget_s=0.05)
            if predicate(tracker):
                return
        raise CommandFailed(f"timed out after {timeout_s:g} s waiting for {description}")

    # -- commands ---------------------------------------------------------------

    def _send_command(
        self,
        command: int,
        *params: float,
        tracker: TelemetryTracker | None = None,
        require_ack: bool = True,
    ) -> None:
        values: Iterable[float] = list(params) + [0.0] * (7 - len(params))
        system, component = self.target
        self.connection.mav.command_long_send(system, component, command, 0, *values)
        if require_ack:
            self._expect_ack(command, tracker)

    def _expect_ack(self, command: int, tracker: TelemetryTracker | None = None) -> None:
        deadline = Deadline.after(self.ack_timeout_s)
        while not deadline.expired:
            ack = self._recv_matching("COMMAND_ACK", deadline.slice(MAX_BLOCKING_READ_S), tracker)
            if ack is None or ack.command != command:
                continue
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return
            raise CommandFailed(f"command {command} rejected with result {ack.result}")
        raise CommandFailed(f"command {command} was never acknowledged")

    def set_mode(self, name: str, tracker: TelemetryTracker, timeout_s: float = 10.0) -> None:
        if name not in MODE_NUMBER_BY_NAME:
            raise ValueError(f"unknown copter mode {name!r}")
        self.connection.set_mode(MODE_NUMBER_BY_NAME[name])
        self.wait_for(lambda t: t.mode == name, tracker, timeout_s, f"mode {name}")

    def wait_ready_to_arm(self, tracker: TelemetryTracker, timeout_s: float = 120.0) -> None:
        """Wait for a position estimate good enough that GUIDED will accept an arm."""
        self.wait_for(lambda t: t.has_position, tracker, timeout_s, "a position estimate")

    def _send_until_accepted(
        self,
        command: int,
        *params: float,
        tracker: TelemetryTracker,
        timeout_s: float,
        retry_every_s: float = 2.0,
    ) -> None:
        """Repeat a command until the vehicle accepts it, or the deadline passes.

        A freshly started autopilot refuses to arm — and, for a moment after arming,
        to take off — while its own checks are still settling, and says so with a bare
        rejection rather than a reason. Those refusals are transient, so a single
        attempt is a race against start-up rather than an answer. Retrying is what a
        ground station does, and what lets the integration tests run against a cold
        simulator instead of one that happens to have been up for a while.
        """
        deadline = Deadline.after(timeout_s)
        refusals = 0
        while not deadline.expired:
            try:
                self._send_command(command, *params, tracker=tracker)
                return
            except CommandFailed:
                refusals += 1
                # Keep the telemetry flowing while the checks have another go.
                self.drain(tracker, budget_s=retry_every_s)
        explanation = tracker.last_status
        raise CommandFailed(
            f"command {command} refused {refusals} times in {timeout_s:g} s"
            + (f"; vehicle says: {explanation!r}" if explanation else "")
        )

    def arm(
        self,
        tracker: TelemetryTracker,
        timeout_s: float = 90.0,
        retry_every_s: float = 2.0,
    ) -> None:
        self.drain(tracker)
        if tracker.armed:
            return
        deadline = Deadline.after(timeout_s)
        self._send_until_accepted(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            1,
            tracker=tracker,
            timeout_s=timeout_s,
            retry_every_s=retry_every_s,
        )
        # Inside the same budget, not on top of it: a caller that allowed 30 s for
        # arming meant 30 s in total, and `launch()` re-establishes the whole sequence
        # on its own schedule.
        self.wait_for(lambda t: t.armed, tracker, deadline.slice(10.0), "the vehicle to arm")

    def disarm(
        self,
        tracker: TelemetryTracker,
        timeout_s: float = 10.0,
        force: bool = False,
    ) -> None:
        """Command a disarm. SPEC F2 asks for it; the flying paths reach it through `land`.

        `force` is the in-air disarm, which the autopilot refuses without the magic token
        because it is how you drop a vehicle out of the sky.
        """
        self._send_command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            FORCE_DISARM_MAGIC if force else 0.0,
            tracker=tracker,
        )
        self.wait_for(lambda t: not t.armed, tracker, timeout_s, "the vehicle to disarm")

    def takeoff(
        self,
        altitude_m: float,
        tracker: TelemetryTracker,
        timeout_s: float = 60.0,
        accept_timeout_s: float = 60.0,
        retry_every_s: float = 2.0,
    ) -> None:
        """Climb to `altitude_m` above home and wait until it is essentially reached."""
        self._send_until_accepted(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude_m,
            tracker=tracker,
            timeout_s=accept_timeout_s,
            retry_every_s=retry_every_s,
        )
        self.wait_for(
            lambda t: t.has_position and t.position.alt_m >= altitude_m * 0.95,
            tracker,
            timeout_s,
            f"climb to {altitude_m:g} m",
        )

    def land(self, tracker: TelemetryTracker, timeout_s: float = 120.0) -> None:
        self.set_mode("LAND", tracker)
        self.wait_for(lambda t: not t.armed, tracker, timeout_s, "touchdown and disarm")

    def set_param(
        self,
        name: str,
        value: float,
        tracker: TelemetryTracker | None = None,
        timeout_s: float = 5.0,
    ) -> float:
        """Set a parameter and read back what the vehicle actually stored.

        Used to switch the simulator's wind on for the disturbance runs, so the
        conditions a measurement was taken under are set by the test, not by hand.
        """
        system, component = self.target
        self.connection.mav.param_set_send(
            system,
            component,
            name.encode("ascii"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = Deadline.after(timeout_s)
        while not deadline.expired:
            message = self._recv_matching(
                "PARAM_VALUE", deadline.slice(MAX_BLOCKING_READ_S), tracker
            )
            if message is not None and message.param_id.strip("\x00") == name:
                return message.param_value
        raise CommandFailed(f"parameter {name} was not confirmed")

    def send_velocity(self, velocity: NED, yaw_deg: float | None = None) -> None:
        """Command a velocity in the local NED frame for one control cycle.

        ArduPilot treats these targets as short-lived: stop sending and the vehicle
        brakes, which is the failsafe the control loop relies on.
        """
        system, component = self.target
        mask = IGNORE_ALL_BUT_VELOCITY if yaw_deg is None else IGNORE_ALL_BUT_VELOCITY_AND_YAW
        self.connection.mav.set_position_target_local_ned_send(
            0,
            system,
            component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            0.0,
            0.0,
            0.0,
            velocity.north,
            velocity.east,
            velocity.down,
            0.0,
            0.0,
            0.0,
            0.0 if yaw_deg is None else math.radians(yaw_deg),
            0.0,
        )

    def close(self) -> None:
        self.connection.close()
