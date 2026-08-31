"""Guidance laws: pure math, no I/O, no MAVLink.

Every law turns the vehicle's current local position and velocity into a velocity
command. Keeping this module free of transport concerns is what lets the whole of
the control math be unit-tested without starting a simulator.

A law is also handed `t_s`, the seconds since the run began, because a law aimed at
something that moves cannot work out where it is without a clock. The two laws whose
target never moves ignore it and default it to zero, so their call sites stay honest
about not having one; the two that chase a target require it, and forgetting it there
is a `TypeError` rather than a run silently aimed at where the target started.
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


@dataclass(frozen=True)
class Target:
    """Something to intercept: where it was when the run began, and how it is moving.

    A pure function of time rather than a position that gets updated, for the same
    reason the laws are pure functions of state — the whole intercept geometry is then
    reproducible from the run's parameters alone, and the miss distance a test asserts
    on is exact rather than whatever the last update happened to say.
    """

    start: NED
    velocity: NED = NED(0.0, 0.0, 0.0)

    def at(self, t_s: float) -> NED:
        return self.start + self.velocity.scaled(t_s)

    def intercept_time(self, position: NED, speed_mps: float) -> float | None:
        """Soonest a pursuer at `position` holding `speed_mps` could meet this target.

        The collision triangle solved exactly, and the reason it is here rather than in
        a note somewhere: a measured time to intercept means nothing on its own, and
        everything against the best a constant-speed law could possibly have done.

        None when no such course exists — a target faster than the pursuer and opening
        cannot be met at all, and a law that reports a closest approach in that case is
        reporting how near it got, not a near miss.
        """
        offset = self.start - position
        # Horizontal only, like `tracking_error`: the vehicle flies these runs level.
        a = self.velocity.horizontal_norm ** 2 - speed_mps**2
        b = 2.0 * (offset.north * self.velocity.north + offset.east * self.velocity.east)
        c = offset.horizontal_norm**2
        if abs(a) < 1e-12:
            # Equal speeds: the quadratic degenerates to a line, and a pursuer that is
            # exactly as fast as its target meets it at most once.
            return -c / b if b < 0.0 else None
        discriminant = b * b - 4.0 * a * c
        if discriminant < 0.0:
            return None
        root = math.sqrt(discriminant)
        times = [t for t in ((-b + root) / (2.0 * a), (-b - root) / (2.0 * a)) if t > 0.0]
        return min(times) if times else None


def _pd_step(error: NED, velocity: NED, gains: Gains) -> NED:
    """Raw (unsaturated) PD response to a position error."""
    return NED(
        gains.kp_horizontal * error.north - gains.kd_horizontal * velocity.north,
        gains.kp_horizontal * error.east - gains.kd_horizontal * velocity.east,
        gains.kp_vertical * error.down,
    )


def _rotated(vector: NED, angle_rad: float) -> NED:
    """Turn the horizontal part of `vector` clockwise seen from above, keeping `down`.

    Clockwise is the north->east sense, which is the direction bearings increase in.
    """
    cos, sin = math.cos(angle_rad), math.sin(angle_rad)
    return NED(
        vector.north * cos - vector.east * sin,
        vector.north * sin + vector.east * cos,
        vector.down,
    )


def _horizontal_unit(offset: NED) -> NED | None:
    """Unit vector along the horizontal part of `offset`, or None if there is none.

    None rather than a fallback direction: the two callers want different things when
    the vector vanishes, and a silent default would pick one of them for both.
    """
    norm = offset.horizontal_norm
    if norm < 1e-6:
        return None
    return NED(offset.north / norm, offset.east / norm)


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

    def tracking_error(self, position: NED, t_s: float = 0.0) -> float:
        """Horizontal distance to the target — the number the run is judged on."""
        return self.error(position).horizontal_norm

    def command(self, position: NED, velocity: NED, t_s: float = 0.0) -> VelocityCommand:
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

    def tracking_error(self, position: NED, t_s: float = 0.0) -> float:
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
        return _rotated(outward, angle)

    def command(self, position: NED, velocity: NED, t_s: float = 0.0) -> VelocityCommand:
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


# Below this speed the vehicle has no heading worth rotating: at the start of a run it
# is hovering, and a law that works by turning the velocity vector has nothing to turn.
# It flies straight at the target until it is moving — the launch phase that every
# terminal guidance law is preceded by, rather than a special case invented here.
PRONAV_MIN_HEADING_SPEED_MPS = 0.5

# The lead time a wanted turn rate is asked for as, in seconds. Named here because
# `ProNav` and the CLI both need it, and a value spelled in two places drifts. Swept, not
# measured, and deliberately not the `make latency` figure it started from — see `ProNav`.
DEFAULT_RESPONSE_S = 0.66


def _closing_command(
    heading: NED | None,
    offset: NED,
    speed_mps: float,
    gains: Gains,
    limits: Limits,
    face_target: bool,
) -> VelocityCommand:
    """Fly at `speed_mps` along `heading`, closing the target's altitude on the way.

    Shared by the two intercept laws, which differ only in the heading they work out:
    everything downstream of that — the vertical term, the saturation, the yaw — is the
    same, and writing it twice is how the two arms of a comparison quietly stop being
    comparable. A `heading` of None means there is no sensible direction to fly (the
    vehicle is on top of the target); the command is then vertical only.
    """
    horizontal = NED(0.0, 0.0) if heading is None else heading.scaled(speed_mps)
    vertical = NED(0.0, 0.0, gains.kp_vertical * offset.down)
    yaw = None
    if face_target and offset.horizontal_norm > YAW_DEADBAND_M:
        yaw = bearing_deg(offset)
    return VelocityCommand(limits.apply(horizontal + vertical), yaw)


@dataclass(frozen=True)
class Pursuit:
    """Chase a moving target by aiming straight at where it is now.

    The baseline the intercept experiment measures `ProNav` against, and deliberately
    the crudest thing that could work. Constant speed rather than the hold law's damped
    pull, so that the only difference between the two laws is the direction commanded:
    two laws that also flew at different speeds would not be comparing much.

    Pure pursuit arrives from behind. Aiming at where the target is keeps the line of
    sight turning, the vehicle spends its turn rate following that rotation instead of
    closing, and against a target crossing fast enough it never arrives at all. That is
    the prediction the measurement is there to confirm or break.
    """

    target: Target
    speed_mps: float = 4.0
    gains: Gains = Gains()
    limits: Limits = Limits()
    face_target: bool = True

    def error(self, position: NED, t_s: float) -> NED:
        return self.target.at(t_s) - position

    def tracking_error(self, position: NED, t_s: float) -> float:
        """Horizontal range to the target; its minimum over a run is the miss distance."""
        return self.error(position, t_s).horizontal_norm

    def command(self, position: NED, velocity: NED, t_s: float) -> VelocityCommand:
        offset = self.error(position, t_s)
        return _closing_command(
            _horizontal_unit(offset),
            offset,
            self.speed_mps,
            self.gains,
            self.limits,
            self.face_target,
        )


@dataclass(frozen=True)
class ProNav:
    """Intercept by turning at a multiple of the rate the line of sight is turning.

    Proportional navigation is written as a lateral acceleration, `a = N · Vc · Ω`, and
    everything in this module commands a velocity — the autopilot closes the inner loop.
    For a vehicle held at constant speed the two are the same statement: a lateral
    acceleration `a` is a turn at `a / V`, so the law reduces to turning the velocity
    vector at `N · Ω`, and the closing speed folds into `N`.

    A turn *rate* still has to be asked for as a velocity *direction*, and that is what
    `response_s` converts: a heading offset of `N · Ω · response_s` comes out as a turn
    of roughly `N · Ω`.

    It started from the same `make latency` figure as `Orbit.lookahead_s`, and that
    figure — 0.22 s — is too small for this use. Command-to-reaction latency is when the
    vehicle *starts* to move; fitting the hover-to-speed step in the SITL logs puts 63%
    of a commanded velocity step at 1.4 s and the first-order constant near 1.8 s, which
    is the timescale this offset stands in for. Sweeping it against the geometric
    optimum bears that out: 0.22 reaches the target at 1.50 times the optimum, 0.66 at
    1.10, 1.00 at 1.08, 1.40 at 1.06, and a second engagement geometry agrees. The curve
    is flat from about 0.7 on, so the default sits at the near edge of that flat rather
    than at the best single number measured — past it the gain is tenths of a percent
    and the lead grows for nothing.

    The heading being rotated is the vehicle's own, not the line of sight. Rotating the
    line of sight by zero would command flying straight at the target, which is
    `Pursuit`; holding the *current* heading while the bearing is steady is what keeps a
    collision course a collision course.
    """

    target: Target
    speed_mps: float = 4.0
    # Classically 3 to 5. Below 2 the law cannot out-turn the line of sight it is
    # chasing and degenerates towards pursuit; well above 5 it answers measurement noise
    # in the bearing with the whole turn rate it has.
    n: float = 3.0
    response_s: float = DEFAULT_RESPONSE_S
    gains: Gains = Gains()
    limits: Limits = Limits()
    face_target: bool = True

    def error(self, position: NED, t_s: float) -> NED:
        return self.target.at(t_s) - position

    def tracking_error(self, position: NED, t_s: float) -> float:
        """Horizontal range to the target; its minimum over a run is the miss distance."""
        return self.error(position, t_s).horizontal_norm

    def line_of_sight_rate(self, position: NED, velocity: NED, t_s: float) -> float:
        """How fast the bearing to the target is turning, rad/s, positive clockwise.

        The one quantity this law needs, and the reason it is the terminal guidance law
        rather than a curiosity: a bearing rate is what a seeker measures directly,
        without ever knowing the range. It is worked out from the geometry here only
        because this project's target state is exact.

        Zero means the bearing is not moving, and a bearing that does not move while the
        range closes is a collision. Driving this to zero and leaving it there is the
        whole of the law.
        """
        offset = self.error(position, t_s)
        distance = offset.horizontal_norm
        if distance < 1e-6:
            return 0.0
        relative = self.target.velocity - velocity
        cross = offset.north * relative.east - offset.east * relative.north
        return cross / (distance * distance)

    def command(self, position: NED, velocity: NED, t_s: float) -> VelocityCommand:
        offset = self.error(position, t_s)
        line_of_sight = _horizontal_unit(offset)
        heading = _horizontal_unit(velocity)
        if heading is None or velocity.horizontal_norm < PRONAV_MIN_HEADING_SPEED_MPS:
            aim = line_of_sight
        else:
            rate = self.line_of_sight_rate(position, velocity, t_s)
            aim = _rotated(heading, self.n * rate * self.response_s)
        return _closing_command(
            aim, offset, self.speed_mps, self.gains, self.limits, self.face_target
        )
