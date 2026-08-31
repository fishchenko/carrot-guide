from __future__ import annotations

import argparse
from dataclasses import astuple
from typing import Any

from carrot_guide.commands.flight import FlightCommand
from carrot_guide.guidance import Gains, Orbit
from carrot_guide.runner import GuidanceLaw


class OrbitCommand(FlightCommand):
    def add_law_arguments(self, sub: argparse.ArgumentParser) -> None:
        sub.add_argument("--kp", type=float, default=0.8, help="gain of the radial term")
        sub.add_argument("--north", type=float, default=0.0, help="circle centre north, m")
        sub.add_argument("--east", type=float, default=0.0, help="circle centre east, m")
        sub.add_argument("--radius", type=float, default=25.0)
        sub.add_argument("--speed", type=float, default=3.0, help="tangential speed, m/s")
        sub.add_argument("--counter-clockwise", action="store_true")
        sub.add_argument(
            "--lookahead",
            type=float,
            default=0.0,
            help="aim the tangent this many seconds ahead, to cancel the vehicle's turn lag",
        )
        sub.add_argument("--seconds", type=float, default=90.0)

    def build_law(self, args: argparse.Namespace) -> GuidanceLaw:
        return Orbit(
            centre=self._point(args),
            radius_m=args.radius,
            speed_mps=args.speed,
            clockwise=not args.counter_clockwise,
            gains=Gains(kp_horizontal=args.kp),
            limits=self._limits(args),
            lookahead_s=args.lookahead,
        )

    def describe(self, args: argparse.Namespace) -> dict[str, Any]:
        return {"centre_ned": list(astuple(self._point(args))), "radius_m": args.radius}
