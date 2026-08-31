from __future__ import annotations

import argparse
from dataclasses import astuple
from typing import Any

from carrot_guide.commands.flight import FlightCommand
from carrot_guide.guidance import Gains, HoldPoint
from carrot_guide.runner import GuidanceLaw


class HoldCommand(FlightCommand):
    def add_law_arguments(self, sub: argparse.ArgumentParser) -> None:
        sub.add_argument("--kp", type=float, default=0.8)
        sub.add_argument("--kd", type=float, default=0.4)
        sub.add_argument("--north", type=float, default=30.0, help="target offset north, m")
        sub.add_argument("--east", type=float, default=0.0, help="target offset east, m")
        sub.add_argument("--seconds", type=float, default=60.0)

    def build_law(self, args: argparse.Namespace) -> GuidanceLaw:
        return HoldPoint(
            target=self._point(args),
            gains=Gains(kp_horizontal=args.kp, kd_horizontal=args.kd),
            limits=self._limits(args),
            face_target=True,
        )

    def describe(self, args: argparse.Namespace) -> dict[str, Any]:
        return {"target_ned": list(astuple(self._point(args)))}
