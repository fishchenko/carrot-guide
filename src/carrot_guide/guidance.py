"""Guidance laws: pure math, no I/O, no MAVLink.

Every law turns the vehicle's current local position and velocity into a velocity
command. Keeping this module free of transport concerns is what lets the whole of
the control math be unit-tested without starting a simulator.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from carrot_guide.state import NED


@dataclass(frozen=True)
class Limits:
    """Saturation applied to every command before it leaves a law."""

    max_horizontal_speed: float = 5.0
    max_vertical_speed: float = 2.0

    def apply(self, velocity: NED) -> NED:
        capped = velocity.clamped_horizontally(self.max_horizontal_speed)
        down = max(-self.max_vertical_speed, min(self.max_vertical_speed, capped.down))
        return NED(capped.north, capped.east, down)


@dataclass(frozen=True)
class Gains:
    """Proportional-derivative gains of the outer (position) loop.

    The vehicle's own controller closes the inner velocity loop, so a PD outer loop
    is enough: `kp` pulls towards the target, `kd` damps the approach and stops the
    overshoot that a pure P law produces at higher speed limits.
    """

    kp_horizontal: float = 0.8
    kd_horizontal: float = 0.4
    kp_vertical: float = 0.8


@dataclass(frozen=True)
class VelocityCommand:
    """What a law asks the vehicle to do for the next control cycle."""

    velocity: NED
    yaw_deg: float | None = None


def _pd_step(error: NED, velocity: NED, gains: Gains) -> NED:
    """Raw (unsaturated) PD response to a position error."""
    return NED(
        gains.kp_horizontal * error.north - gains.kd_horizontal * velocity.north,
        gains.kp_horizontal * error.east - gains.kd_horizontal * velocity.east,
        gains.kp_vertical * error.down,
    )


def bearing_deg(offset: NED) -> float:
    """Compass bearing of a horizontal offset, degrees clockwise from north."""
    return math.degrees(math.atan2(offset.east, offset.north)) % 360.0


# Below this distance the bearing to the target is mostly noise, and yaw would chase
# it; the vehicle holds its heading instead. Only the yaw is deadbanded — the position
# command keeps working all the way in.
YAW_DEADBAND_M = 1.0


@dataclass(frozen=True)
class HoldPoint:
    """Fly to a point and stay on it.

    The command is a damped pull towards the target, saturated by `limits`; with the
    vehicle sitting on the target the command is zero, so wind rejection is whatever
    the loop rate and gains buy — which is exactly what the measurements report.
    """

    target: NED
    gains: Gains = Gains()
    limits: Limits = Limits()
    face_target: bool = False

    def error(self, position: NED) -> NED:
        return self.target - position

    def tracking_error(self, position: NED) -> float:
        """Horizontal distance to the target — the number the run is judged on."""
        return self.error(position).horizontal_norm

    def command(self, position: NED, velocity: NED) -> VelocityCommand:
        error = self.error(position)
        yaw = None
        if self.face_target and error.horizontal_norm > YAW_DEADBAND_M:
            yaw = bearing_deg(error)
        return VelocityCommand(self.limits.apply(_pd_step(error, velocity, self.gains)), yaw)


@dataclass(frozen=True)
class Orbit:
    """Fly a circle of a fixed radius around a centre point.

    The command is the sum of two terms: a tangential one that carries the vehicle
    around the circle at `speed_mps`, and a radial one that corrects the distance to
    the centre. Splitting them this way means radius error decays independently of
    how fast the orbit is flown.

    `lookahead_s` aims the tangent at where the vehicle should be that many seconds
    from now instead of where it is. The vehicle's velocity loop needs time to turn, so
    a tangent aimed at the present position is always slightly stale and the orbit
    settles a steady margin outside the requested radius. Rotating the aim point
    forward by the same lag cancels that bias; set it to the measured
    command-to-reaction latency.
    """

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

    def tracking_error(self, position: NED) -> float:
        """How far off the circle the vehicle is — the number the run is judged on."""
        return abs(self.radial_error(position))

    @property
    def rate_rad_s(self) -> float:
        """Angular rate of a vehicle flying this orbit at the requested speed."""
        return self.speed_mps / self.radius_m

    def _advanced(self, outward: NED) -> NED:
        """Rotate an outward unit vector forward along the orbit by the lookahead."""
        if self.lookahead_s <= 0.0:
            return outward
        angle = self.rate_rad_s * self.lookahead_s * (1.0 if self.clockwise else -1.0)
        cos, sin = math.cos(angle), math.sin(angle)
        return NED(
            outward.north * cos - outward.east * sin,
            outward.north * sin + outward.east * cos,
        )

    def command(self, position: NED, velocity: NED) -> VelocityCommand:
        # `velocity` is unread here, and so is `gains.kd_horizontal`: this law has no
        # derivative term at all. Turn lag is handled by aiming ahead (`lookahead_s`),
        # which cancels the steady radius bias rather than damping anything. The
        # parameter stays because `runner.GuidanceLaw` is what lets the loop drive
        # either law without knowing which one it holds.
        offset = position - self.centre
        distance = offset.horizontal_norm

        # Sitting exactly on the centre leaves the tangent undefined; break the tie
        # towards north so the law still produces a sane outward command.
        if distance < 1e-6:
            outward = NED(1.0, 0.0)
        else:
            outward = NED(offset.north / distance, offset.east / distance)

        # Rotating the outward unit vector by +90 degrees in the north->east sense
        # gives the clockwise tangent seen from above.
        aim = self._advanced(outward)
        tangent = NED(-aim.east, aim.north)
        if not self.clockwise:
            tangent = tangent.scaled(-1.0)

        radial = outward.scaled(self.gains.kp_horizontal * self.radial_error(position))
        vertical = NED(0.0, 0.0, self.gains.kp_vertical * (self.centre.down - position.down))

        command = tangent.scaled(self.speed_mps) + radial + vertical
        yaw = bearing_deg(outward.scaled(-1.0)) if self.face_centre else None
        return VelocityCommand(self.limits.apply(command), yaw)
