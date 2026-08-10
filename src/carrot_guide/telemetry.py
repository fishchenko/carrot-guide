"""Turn the raw MAVLink message stream into a typed vehicle state.

The tracker keeps the last value seen for each message it cares about and hands out
an immutable `VehicleState` snapshot. It never touches a socket, so the parsing is
unit-tested against hand-built messages rather than against a live simulator.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any

from carrot_guide.state import GlobalPosition, NED, VehicleState

# MAV_MODE_FLAG_SAFETY_ARMED. Spelled out rather than imported so this module stays
# free of pymavlink and can be exercised with plain stubs.
ARMED_FLAG = 0b1000_0000

# ArduCopter custom_mode values, the subset this project ever asks for or asserts on.
COPTER_MODES: dict[int, str] = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    16: "POSHOLD",
    20: "GUIDED_NOGPS",
}

MODE_NUMBERS: dict[str, int] = {name: number for number, name in COPTER_MODES.items()}


def mode_name(custom_mode: int) -> str:
    return COPTER_MODES.get(custom_mode, f"MODE_{custom_mode}")


class TelemetryError(RuntimeError):
    """Raised when a state snapshot is asked for before the stream can supply one."""


@dataclass
class TelemetryTracker:
    """Last-known-value view of the vehicle, fed one MAVLink message at a time."""

    position: GlobalPosition | None = None
    velocity: NED = NED(0.0, 0.0, 0.0)
    heading_deg: float = 0.0
    armed: bool = False
    mode: str = "UNKNOWN"
    battery_pct: float | None = None
    timestamp_s: float = 0.0
    origin: GlobalPosition | None = None
    # Counts position reports rather than messages of any kind: a link that still
    # heartbeats while the position stream has died is exactly the case the control
    # loop has to notice.
    position_updates: int = 0
    # The autopilot explains a refusal in STATUSTEXT, not in the COMMAND_ACK, so the
    # reason is only ever in the message stream. Keeping the recent ones means an
    # error can quote the vehicle instead of just reporting a number.
    status_texts: deque[str] = field(default_factory=lambda: deque(maxlen=10))

    def handle(self, message: Any) -> None:
        """Absorb one message; unknown types are ignored."""
        kind = message.get_type()
        if kind == "GLOBAL_POSITION_INT":
            self._handle_global_position(message)
        elif kind == "HEARTBEAT":
            self.armed = bool(message.base_mode & ARMED_FLAG)
            self.mode = mode_name(message.custom_mode)
        elif kind == "STATUSTEXT":
            self.status_texts.append(message.text.strip())
        elif kind == "SYS_STATUS":
            remaining = message.battery_remaining
            self.battery_pct = None if remaining < 0 else float(remaining)

    def _handle_global_position(self, message: Any) -> None:
        # ArduPilot reports lat/lon in 1e7 degrees, altitudes in mm and speeds in cm/s.
        # `relative_alt` is measured from the home point, which is also where the local
        # frame is anchored, so the two agree without an extra AMSL correction.
        self.position = GlobalPosition(
            lat_deg=message.lat / 1e7,
            lon_deg=message.lon / 1e7,
            alt_m=message.relative_alt / 1000.0,
        )
        self.velocity = NED(
            north=message.vx / 100.0,
            east=message.vy / 100.0,
            down=message.vz / 100.0,
        )
        self.heading_deg = (message.hdg / 100.0) % 360.0 if message.hdg != 65535 else self.heading_deg
        self.timestamp_s = message.time_boot_ms / 1000.0
        self.position_updates += 1

    @property
    def last_status(self) -> str | None:
        """The most recent thing the vehicle said about itself, if anything."""
        return self.status_texts[-1] if self.status_texts else None

    @property
    def has_position(self) -> bool:
        return self.position is not None

    def set_origin(self, position: GlobalPosition | None = None) -> GlobalPosition:
        """Anchor the local frame, defaulting to wherever the vehicle is right now."""
        origin = position or self.position
        if origin is None:
            raise TelemetryError("no position received yet, cannot anchor the local frame")
        self.origin = origin
        return origin

    def snapshot(self) -> VehicleState:
        if self.position is None:
            raise TelemetryError("no position received yet")
        return VehicleState(
            position=self.position,
            velocity=self.velocity,
            heading_deg=self.heading_deg,
            armed=self.armed,
            mode=self.mode,
            battery_pct=self.battery_pct,
            timestamp_s=self.timestamp_s,
        )
