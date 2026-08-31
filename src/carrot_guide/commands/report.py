from __future__ import annotations

import argparse
from typing import Any

from carrot_guide.metrics import DEFAULT_SETTLE_THRESHOLD_M, summarise
from carrot_guide.recording import load_samples
from carrot_guide.state import NED
from carrot_guide.utils import Command, emit_json


class ReportCommand(Command):
    def add_arguments(self, sub: argparse.ArgumentParser) -> None:
        sub.add_argument("log")
        sub.add_argument("--plot", default=None, help="write a figure to this path")
        sub.add_argument(
            "--circle",
            default=None,
            help="overlay the target as north,east or north,east,radius",
        )
        sub.add_argument(
            "--target",
            default=None,
            help="overlay a moving target as north,east,vn,ve",
        )
        sub.add_argument("--settle-threshold", type=float, default=DEFAULT_SETTLE_THRESHOLD_M)
        self._add_summary(sub)

    def run(self, args: argparse.Namespace) -> int:
        samples = load_samples(args.log)
        summary = summarise(samples, settle_threshold_m=args.settle_threshold)
        payload: dict[str, Any] = {"log": args.log, "tracking": summary.as_dict()}
        if args.plot:
            from carrot_guide.plots import plot_run

            centre, radius = self._parse_circle(args.circle)
            start, velocity = self._parse_target(args.target)
            payload["plot"] = str(
                plot_run(
                    samples,
                    args.plot,
                    title=summary.label,
                    centre=centre,
                    radius_m=radius,
                    target_start=start,
                    target_velocity=velocity,
                )
            )
        emit_json(payload, args.summary)
        return 0

    @staticmethod
    def _parse_circle(text: str | None) -> tuple[NED | None, float | None]:
        if not text:
            return None, None
        parts = [float(part) for part in text.split(",")]
        if len(parts) == 2:
            return NED(parts[0], parts[1]), None
        if len(parts) == 3:
            return NED(parts[0], parts[1]), parts[2]
        raise SystemExit("--circle takes north,east or north,east,radius")

    @staticmethod
    def _parse_target(text: str | None) -> tuple[NED | None, NED | None]:
        if not text:
            return None, None
        parts = [float(part) for part in text.split(",")]
        if len(parts) != 4:
            raise SystemExit("--target takes north,east,vn,ve")
        return NED(parts[0], parts[1]), NED(parts[2], parts[3])
