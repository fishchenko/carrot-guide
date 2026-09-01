"""A pymavlink connection that answers the way an autopilot does.

Commands come back with a COMMAND_ACK and with the state change that follows, refusals
come off a list the test hands in, and `recv_match` discards what it was not asked for
— which is the behaviour `MavlinkLink` is built around. A blocking read that finds
nothing costs its timeout on the fake clock, the way the socket costs it in wall time.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from pymavlink import mavutil

from carrot_guide.state import NED
from carrot_guide.telemetry import MAV_MODE_FLAG_SAFETY_ARMED

from tests.doubles.clocks import FakeClock
from tests.doubles.messages import Message

ARM_COMMAND = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
ACCEPTED = mavutil.mavlink.MAV_RESULT_ACCEPTED
FAILED = mavutil.mavlink.MAV_RESULT_FAILED


@dataclass(frozen=True)
class SentTarget:
    """What reached the wire.

    The mask is kept because it is the actual contract: a yaw of 0.0 means nothing
    until you know whether the mask told the autopilot to ignore it.
    """

    velocity: NED
    yaw_rad: float
    mask: int


def heartbeat(armed: bool, mode: int = 4) -> Message:
    return Message(
        "HEARTBEAT",
        base_mode=MAV_MODE_FLAG_SAFETY_ARMED if armed else 0,
        custom_mode=mode,
    )


class FakeConnection:
    """Stands in for a pymavlink connection, one queued message at a time."""

    target_system = 1
    target_component = 1

    def __init__(self, arm_results: list[int] | None = None) -> None:
        self.clock = FakeClock()
        self.mav = self
        self.inbox: deque[Message] = deque()
        self.commands: list[tuple[int, tuple[float, ...]]] = []
        self.targets: list[SentTarget] = []
        self.params: list[tuple[str, float]] = []
        self.arm_results = deque(arm_results or [ACCEPTED])
        self.reject: set[int] = set()  # commands answered with a refusal
        self.silent = False  # an autopilot that never answers at all
        self.closed = False

    # -- the bits of the pymavlink API this project actually uses ----------------

    def command_long_send(self, system, component, command, confirmation, *params):
        self.commands.append((command, params))
        if self.silent:
            return
        result = FAILED if command in self.reject else ACCEPTED
        if command == ARM_COMMAND and params[0] == 1 and command not in self.reject:
            result = self.arm_results.popleft() if self.arm_results else ACCEPTED
        self.inbox.append(Message("COMMAND_ACK", command=command, result=result))
        if command == ARM_COMMAND and result == ACCEPTED:
            self.inbox.append(heartbeat(armed=bool(params[0])))

    def set_position_target_local_ned_send(self, *args):
        self.targets.append(SentTarget(NED(args[8], args[9], args[10]), args[14], args[4]))

    def param_set_send(self, system, component, name, value, param_type):
        self.params.append((name.decode("ascii"), value))

    def recv_match(self, type=None, blocking=False, timeout=None):
        while self.inbox:
            message = self.inbox.popleft()
            if type is None or message.get_type() == type:
                return message
        # A blocking read that finds nothing costs its timeout, the way the socket does.
        # Without that the link's deadlines never move and every wait loops forever.
        if blocking and timeout:
            self.clock.advance(timeout)
        return None

    def close(self):
        self.closed = True
