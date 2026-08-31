from __future__ import annotations

import math
from dataclasses import dataclass

from carrot_guide.state import NED


@dataclass(frozen=True)
class Target:
    start: NED
    velocity: NED = NED(0.0, 0.0, 0.0)

    def at(self, t_s: float) -> NED:
        return self.start + self.velocity.scaled(t_s)

    def intercept_time(self, position: NED, speed_mps: float) -> float | None:
        """Soonest a pursuer at `position` holding `speed_mps` meets the target; None if never."""
        offset = self.start - position
        # Horizontal only: the vehicle flies these runs level.
        a = self.velocity.horizontal_norm ** 2 - speed_mps**2
        b = 2.0 * (offset.north * self.velocity.north + offset.east * self.velocity.east)
        c = offset.horizontal_norm**2
        if abs(a) < 1e-12:
            # Equal speeds: the quadratic degenerates to a line, met at most once.
            return -c / b if b < 0.0 else None
        discriminant = b * b - 4.0 * a * c
        if discriminant < 0.0:
            return None
        root = math.sqrt(discriminant)
        times = [t for t in ((-b + root) / (2.0 * a), (-b - root) / (2.0 * a)) if t > 0.0]
        return min(times) if times else None
