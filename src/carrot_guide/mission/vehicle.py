from __future__ import annotations

from dataclasses import dataclass

from carrot_guide.link import MavlinkLink
from carrot_guide.state import GlobalPosition, NED, to_local_ned
from carrot_guide.telemetry import TelemetryError, TelemetryTracker


@dataclass
class Vehicle:
    link: MavlinkLink
    tracker: TelemetryTracker

    @property
    def origin(self) -> GlobalPosition:
        if self.tracker.origin is None:
            raise TelemetryError("local frame is not anchored; call tracker.set_origin() first")
        return self.tracker.origin

    @property
    def position_ned(self) -> NED:
        return to_local_ned(self.tracker.snapshot().position, self.origin)
