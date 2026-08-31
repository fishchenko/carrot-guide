from __future__ import annotations

from dataclasses import dataclass

from carrot_guide.state import NED


@dataclass(frozen=True)
class Limits:
    max_horizontal_speed: float = 5.0
    max_vertical_speed: float = 2.0

    def apply(self, velocity: NED) -> NED:
        capped = velocity.clamped_horizontally(self.max_horizontal_speed)
        down = max(-self.max_vertical_speed, min(self.max_vertical_speed, capped.down))
        return NED(capped.north, capped.east, down)


@dataclass(frozen=True)
class Gains:
    kp_horizontal: float = 0.8
    kd_horizontal: float = 0.4
    kp_vertical: float = 0.8


@dataclass(frozen=True)
class VelocityCommand:
    velocity: NED
    yaw_deg: float | None = None
