from __future__ import annotations

import math

from carrot_guide.state import NED


def rotated(vector: NED, angle_rad: float) -> NED:
    """Clockwise seen from above, i.e. the north->east sense bearings increase in."""
    cos, sin = math.cos(angle_rad), math.sin(angle_rad)
    return NED(
        vector.north * cos - vector.east * sin,
        vector.north * sin + vector.east * cos,
        vector.down,
    )


def horizontal_unit(offset: NED) -> NED | None:
    """None when the horizontal part vanishes; callers pick their own fallback."""
    norm = offset.horizontal_norm
    if norm < 1e-6:
        return None
    return NED(offset.north / norm, offset.east / norm)


def bearing_deg(offset: NED) -> float:
    """Degrees clockwise from north."""
    return math.degrees(math.atan2(offset.east, offset.north)) % 360.0


# Below this range the bearing is mostly noise; only yaw is deadbanded, not the position command.
YAW_DEADBAND_M = 1.0
