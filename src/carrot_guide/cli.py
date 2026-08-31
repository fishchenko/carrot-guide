"""Command line: one subcommand per experiment — every one in the spec, and `intercept`.

Each flying command writes a CSV log and prints a JSON summary, so a run is
reproducible and its numbers can go straight into the README without retyping.
"""

from __future__ import annotations

import argparse
from typing import Sequence

from carrot_guide.commands.hold import HoldCommand
from carrot_guide.commands.intercept import InterceptCommand
from carrot_guide.commands.latency import LatencyCommand
from carrot_guide.commands.orbit import OrbitCommand
from carrot_guide.commands.report import ReportCommand
from carrot_guide.commands.telemetry import TelemetryCommand
from carrot_guide.utils import Command


COMMANDS: tuple[Command, ...] = (
    TelemetryCommand("telemetry", "print live telemetry"),
    HoldCommand("hold", "fly to a point and hold it"),
    OrbitCommand("orbit", "fly a circle"),
    InterceptCommand("intercept", "fly at a moving target"),
    LatencyCommand("latency", "measure command-to-reaction latency"),
    ReportCommand("report", "summarise and plot a recorded log"),
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="carrot-guide", description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command in COMMANDS:
        command.register(subparsers)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.handler(args))


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
