from __future__ import annotations

from dataclasses import dataclass

from carrot_guide.guidance.values import Gains, Limits, VelocityCommand
from carrot_guide.guidance.vectors import YAW_DEADBAND_M, bearing_deg
from carrot_guide.state import NED


@dataclass(frozen=True)
class HoldPoint:
    target: NED
    gains: Gains = Gains()
    limits: Limits = Limits()
    face_target: bool = False

    def error(self, position: NED) -> NED:
        return self.target - position

    def tracking_error(self, position: NED, t_s: float = 0.0) -> float:
        """Horizontal distance to the target — the number the run is judged on."""
        return self.error(position).horizontal_norm

    def _pd_step(self, error: NED, velocity: NED) -> NED:
        """Unsaturated: `command` applies `self.limits`."""
        return NED(
            self.gains.kp_horizontal * error.north - self.gains.kd_horizontal * velocity.north,
            self.gains.kp_horizontal * error.east - self.gains.kd_horizontal * velocity.east,
            self.gains.kp_vertical * error.down,
        )

    def command(self, position: NED, velocity: NED, t_s: float = 0.0) -> VelocityCommand:
        error = self.error(position)
        yaw = None
        if self.face_target and error.horizontal_norm > YAW_DEADBAND_M:
            yaw = bearing_deg(error)
        return VelocityCommand(self.limits.apply(self._pd_step(error, velocity)), yaw)
