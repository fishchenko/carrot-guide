from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any

from carrot_guide.state import GlobalPosition, NED, VehicleState
from carrot_guide.telemetry.modes import HEADING_UNKNOWN, MAV_MODE_FLAG_SAFETY_ARMED, mode_name


class TelemetryError(RuntimeError):
    """Raised when the stream has not yet supplied a position."""


@dataclass
class TelemetryTracker:
    position: GlobalPosition | None = None
    velocity: NED = NED(0.0, 0.0, 0.0)
    heading_deg: float = 0.0
    armed: bool = False
    mode: str = "UNKNOWN"
    battery_pct: float | None = None
    timestamp_s: float = 0.0
    origin: GlobalPosition | None = None
    # Position reports only: a link can keep heartbeating with the position stream dead.
    position_updates: int = 0
    # Refusal reasons arrive in STATUSTEXT, not in COMMAND_ACK.
    status_texts: deque[str] = field(default_factory=lambda: deque(maxlen=10))

    def handle(self, message: Any) -> None:
        """Unknown message types are ignored."""
        kind = message.get_type()
        if kind == "GLOBAL_POSITION_INT":
            self._handle_global_position(message)
        elif kind == "HEARTBEAT":
            self.armed = bool(message.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
            self.mode = mode_name(message.custom_mode)
        elif kind == "STATUSTEXT":
            self.status_texts.append(message.text.strip())
        elif kind == "SYS_STATUS":
            remaining = message.battery_remaining
            self.battery_pct = None if remaining < 0 else float(remaining)

    def _handle_global_position(self, message: Any) -> None:
        # lat/lon in 1e7 degrees, altitudes in mm, speeds in cm/s. `relative_alt` is measured
        # from home, where the local frame is anchored, so no AMSL correction is needed.
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
        if message.hdg != HEADING_UNKNOWN:
            self.heading_deg = (message.hdg / 100.0) % 360.0
        self.timestamp_s = message.time_boot_ms / 1000.0
        self.position_updates += 1

    @property
    def last_status(self) -> str | None:
        return self.status_texts[-1] if self.status_texts else None

    @property
    def has_position(self) -> bool:
        return self.position is not None

    def set_origin(self, position: GlobalPosition | None = None) -> GlobalPosition:
        """Defaults to the vehicle's current position."""
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
