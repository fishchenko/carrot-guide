from __future__ import annotations

from carrot_guide.guidance.values import Gains, Limits, VelocityCommand
from carrot_guide.guidance.vectors import YAW_DEADBAND_M, bearing_deg
from carrot_guide.state import NED


def _closing_command(
    heading: NED | None,
    offset: NED,
    speed_mps: float,
    gains: Gains,
    limits: Limits,
    face_target: bool,
) -> VelocityCommand:
    """A `heading` of None (vehicle on top of the target) gives a vertical-only command."""
    horizontal = NED(0.0, 0.0) if heading is None else heading.scaled(speed_mps)
    vertical = NED(0.0, 0.0, gains.kp_vertical * offset.down)
    yaw = None
    if face_target and offset.horizontal_norm > YAW_DEADBAND_M:
        yaw = bearing_deg(offset)
    return VelocityCommand(limits.apply(horizontal + vertical), yaw)
