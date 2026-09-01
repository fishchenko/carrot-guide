from __future__ import annotations

from dataclasses import dataclass

from carrot_guide.guidance.closing import closing_command
from carrot_guide.guidance.target import Target
from carrot_guide.guidance.values import Gains, Limits, VelocityCommand
from carrot_guide.guidance.vectors import horizontal_unit, rotated
from carrot_guide.state import NED


# Below this speed a hovering vehicle has no heading to rotate; it flies straight at the target.
PRONAV_MIN_HEADING_SPEED_MPS = 0.5

# Lead time the wanted turn rate is asked for as, in seconds. Swept against the geometric
# optimum: 0.22 s arrives at 1.50x it, 0.66 at 1.10, 1.00 at 1.08, 1.40 at 1.06 — flat from
# about 0.7, so the default sits at the near edge of the flat.
DEFAULT_RESPONSE_S = 0.66


@dataclass(frozen=True)
class ProNav:
    """Rotates the vehicle's own heading, not the line of sight, at `n` times the LOS rate."""

    target: Target
    speed_mps: float = 4.0
    # Classically 3 to 5: below 2 it degenerates to pursuit, above 5 it turns on bearing noise.
    n: float = 3.0
    response_s: float = DEFAULT_RESPONSE_S
    gains: Gains = Gains()
    limits: Limits = Limits()
    face_target: bool = True

    def error(self, position: NED, t_s: float) -> NED:
        return self.target.at(t_s) - position

    def tracking_error(self, position: NED, t_s: float) -> float:
        """Horizontal range; its minimum over a run is the miss distance."""
        return self.error(position, t_s).horizontal_norm

    def line_of_sight_rate(self, position: NED, velocity: NED, t_s: float) -> float:
        """Rad/s, positive clockwise; zero while the range closes is a collision course."""
        offset = self.error(position, t_s)
        distance = offset.horizontal_norm
        if distance < 1e-6:
            return 0.0
        relative = self.target.velocity - velocity
        cross = offset.north * relative.east - offset.east * relative.north
        return cross / (distance * distance)

    def command(self, position: NED, velocity: NED, t_s: float) -> VelocityCommand:
        offset = self.error(position, t_s)
        line_of_sight = horizontal_unit(offset)
        heading = horizontal_unit(velocity)
        if heading is None or velocity.horizontal_norm < PRONAV_MIN_HEADING_SPEED_MPS:
            aim = line_of_sight
        else:
            rate = self.line_of_sight_rate(position, velocity, t_s)
            aim = rotated(heading, self.n * rate * self.response_s)
        return closing_command(
            aim, offset, self.speed_mps, self.gains, self.limits, self.face_target
        )
