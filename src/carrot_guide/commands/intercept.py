from __future__ import annotations

import argparse
import math
from dataclasses import astuple
from typing import Any

from carrot_guide.commands.flight import FlightCommand
from carrot_guide.guidance import DEFAULT_RESPONSE_S, ProNav, Pursuit, Target
from carrot_guide.runner import GuidanceLaw
from carrot_guide.state import NED


class InterceptCommand(FlightCommand):
    # A pass has no hold: a threshold would average the retreat past the closest point.
    settle_threshold_default = 0.0

    def add_law_arguments(self, sub: argparse.ArgumentParser) -> None:
        sub.add_argument(
            "--law", choices=("pursuit", "pronav"), default="pronav", help="which law to fly"
        )
        sub.add_argument("--north", type=float, default=60.0, help="target start north, m")
        sub.add_argument("--east", type=float, default=-40.0, help="target start east, m")
        sub.add_argument("--speed", type=float, default=4.0, help="own speed, m/s")
        sub.add_argument("--target-speed", type=float, default=3.0, help="target speed, m/s")
        sub.add_argument(
            "--target-heading",
            type=float,
            default=90.0,
            help="target course, deg clockwise from north",
        )
        sub.add_argument(
            "--nav-constant", type=float, default=3.0, help="N, the navigation constant"
        )
        sub.add_argument(
            "--response",
            type=float,
            default=DEFAULT_RESPONSE_S,
            help="lead time the turn rate is asked for as, s",
        )
        sub.add_argument("--seconds", type=float, default=45.0)

    def _target(self, args: argparse.Namespace) -> Target:
        heading = math.radians(args.target_heading)
        return Target(
            start=self._point(args),
            velocity=NED(
                args.target_speed * math.cos(heading), args.target_speed * math.sin(heading)
            ),
        )

    def build_law(self, args: argparse.Namespace) -> GuidanceLaw:
        target = self._target(args)
        if args.law == "pursuit":
            return Pursuit(target=target, speed_mps=args.speed, limits=self._limits(args))
        return ProNav(
            target=target,
            speed_mps=args.speed,
            n=args.nav_constant,
            response_s=args.response,
            limits=self._limits(args),
        )

    def describe(self, args: argparse.Namespace) -> dict[str, Any]:
        target = self._target(args)
        # Best a constant-speed law could do from takeoff — the yardstick for min_error_t_s.
        optimum = target.intercept_time(NED(0.0, 0.0, -args.altitude), args.speed)
        return {
            "law": args.law,
            "target_start_ned": list(astuple(target.start)),
            "target_velocity_ned": [round(part, 4) for part in astuple(target.velocity)],
            "own_speed_mps": args.speed,
            "optimal_intercept_s": None if optimum is None else round(optimum, 2),
        }
