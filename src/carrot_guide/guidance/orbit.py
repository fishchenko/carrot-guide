from __future__ import annotations

from dataclasses import dataclass

from carrot_guide.guidance.values import Gains, Limits, VelocityCommand
from carrot_guide.guidance.vectors import bearing_deg, _rotated
from carrot_guide.state import NED


@dataclass(frozen=True)
class Orbit:
    """`lookahead_s` cancels the turn-lag radius bias; use the measured link latency (0.22 s)."""

    centre: NED
    radius_m: float
    speed_mps: float = 3.0
    clockwise: bool = True
    gains: Gains = Gains()
    limits: Limits = Limits()
    face_centre: bool = True
    lookahead_s: float = 0.0

    def __post_init__(self) -> None:
        if self.radius_m <= 0.0:
            raise ValueError("orbit radius must be positive")

    def radial_error(self, position: NED) -> float:
        """Signed distance to the circle: positive when the vehicle is inside it."""
        return self.radius_m - (position - self.centre).horizontal_norm

    def tracking_error(self, position: NED, t_s: float = 0.0) -> float:
        return abs(self.radial_error(position))

    @property
    def rate_rad_s(self) -> float:
        return self.speed_mps / self.radius_m

    def _advanced(self, outward: NED) -> NED:
        if self.lookahead_s <= 0.0:
            return outward
        angle = self.rate_rad_s * self.lookahead_s * (1.0 if self.clockwise else -1.0)
        return _rotated(outward, angle)

    def command(self, position: NED, velocity: NED, t_s: float = 0.0) -> VelocityCommand:
        # No derivative term here, so `velocity` and `gains.kd_horizontal` go unread; the
        # signature is what `runner.GuidanceLaw` requires of either law.
        offset = position - self.centre
        distance = offset.horizontal_norm

        # On the centre the tangent is undefined; break the tie towards north.
        if distance < 1e-6:
            outward = NED(1.0, 0.0)
        else:
            outward = NED(offset.north / distance, offset.east / distance)

        # Outward rotated +90 degrees in the north->east sense is the clockwise tangent.
        aim = self._advanced(outward)
        tangent = NED(-aim.east, aim.north)
        if not self.clockwise:
            tangent = tangent.scaled(-1.0)

        radial = outward.scaled(self.gains.kp_horizontal * self.radial_error(position))
        vertical = NED(0.0, 0.0, self.gains.kp_vertical * (self.centre.down - position.down))

        command = tangent.scaled(self.speed_mps) + radial + vertical
        yaw = bearing_deg(outward.scaled(-1.0)) if self.face_centre else None
        return VelocityCommand(self.limits.apply(command), yaw)
