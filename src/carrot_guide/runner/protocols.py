from __future__ import annotations

from typing import Protocol

from carrot_guide.guidance import VelocityCommand
from carrot_guide.state import NED
from carrot_guide.telemetry import TelemetryTracker


class GuidanceLaw(Protocol):
    """`t_s` is seconds since the run began; laws with a fixed target ignore it."""

    def command(self, position: NED, velocity: NED, t_s: float) -> VelocityCommand: ...

    def tracking_error(self, position: NED, t_s: float) -> float: ...


class VehicleLink(Protocol):
    def drain(self, tracker: TelemetryTracker, budget_s: float = 0.0) -> int: ...

    def send_velocity(self, velocity: NED, yaw_deg: float | None = None) -> None: ...
