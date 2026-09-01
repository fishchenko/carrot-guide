from __future__ import annotations

from dataclasses import dataclass

from carrot_guide.guidance.closing import closing_command
from carrot_guide.guidance.target import Target
from carrot_guide.guidance.values import Gains, Limits, VelocityCommand
from carrot_guide.guidance.vectors import horizontal_unit
from carrot_guide.state import NED


@dataclass(frozen=True)
class Pursuit:
    target: Target
    speed_mps: float = 4.0
    gains: Gains = Gains()
    limits: Limits = Limits()
    face_target: bool = True

    def error(self, position: NED, t_s: float) -> NED:
        return self.target.at(t_s) - position

    def tracking_error(self, position: NED, t_s: float) -> float:
        """Horizontal range; its minimum over a run is the miss distance."""
        return self.error(position, t_s).horizontal_norm

    def command(self, position: NED, velocity: NED, t_s: float) -> VelocityCommand:
        offset = self.error(position, t_s)
        return closing_command(
            horizontal_unit(offset),
            offset,
            self.speed_mps,
            self.gains,
            self.limits,
            self.face_target,
        )
