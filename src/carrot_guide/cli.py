"""Command line: every experiment in the spec, one subcommand each.

Each flying command writes a CSV log and prints a JSON summary, so a run is
reproducible and its numbers can go straight into the README without retyping.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import astuple, dataclass
from pathlib import Path
from typing import Any, Sequence

from carrot_guide.guidance import Gains, HoldPoint, Limits, Orbit
from carrot_guide.link import MavlinkLink
from carrot_guide.metrics import summarise, summarise_latency
from carrot_guide.mission import Vehicle, airborne, measure_command_latency
from carrot_guide.recording import CsvRecorder, load_samples
from carrot_guide.runner import GuidanceLaw, GuidanceRunner, RunReport
from carrot_guide.state import NED
from carrot_guide.telemetry import TelemetryTracker

DEFAULT_URL = "tcp:127.0.0.1:5760"
LOG_DIR = Path("logs")


@dataclass(frozen=True)
class Command:
    """One subcommand: the options it takes, and what it does with them."""

    name: str
    help: str

    def add_arguments(self, sub: argparse.ArgumentParser) -> None:
        raise NotImplementedError

    def run(self, args: argparse.Namespace) -> int:
        raise NotImplementedError

    def register(self, subparsers: argparse._SubParsersAction) -> None:
        sub = subparsers.add_parser(self.name, help=self.help)
        self.add_arguments(sub)
        sub.set_defaults(handler=self.run)

    @staticmethod
    def _add_link(sub: argparse.ArgumentParser) -> None:
        sub.add_argument("--url", default=DEFAULT_URL, help="MAVLink endpoint of the simulator")
        sub.add_argument("--timeout", type=float, default=180.0, help="connect/arm timeout, s")

    @staticmethod
    def _add_summary(sub: argparse.ArgumentParser) -> None:
        sub.add_argument(
            "--summary",
            default=None,
            help="also write the JSON summary here, out of reach of anything else on stdout",
        )

    @staticmethod
    def _emit(payload: dict[str, Any], summary_path: str | None = None) -> None:
        """Print the run's summary, and write it somewhere exact if asked to.

        `--summary` exists because capturing stdout is not safe enough for an artefact the
        README quotes: run under `memray`, the profiler's own banner lands on stdout either
        side of the JSON, and two of the committed measurement files were silently
        unparseable because of it. Writing the file directly puts the summary out of reach
        of anything else that prints.
        """
        text = json.dumps(payload, indent=2, ensure_ascii=False)
        sys.stdout.write(text + "\n")
        if summary_path:
            path = Path(summary_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text + "\n", encoding="utf-8")


class TelemetryCommand(Command):
    """F1: connect and show what the vehicle is reporting."""

    def add_arguments(self, sub: argparse.ArgumentParser) -> None:
        self._add_link(sub)
        sub.add_argument("--seconds", type=float, default=10.0)

    def run(self, args: argparse.Namespace) -> int:
        link = MavlinkLink.connect(args.url, timeout_s=args.timeout)
        tracker = TelemetryTracker()
        link.request_streams()
        deadline = time.monotonic() + args.seconds
        try:
            while time.monotonic() < deadline:
                link.drain(tracker, budget_s=0.5)
                if not tracker.has_position:
                    print("waiting for a position estimate...")
                    continue
                state = tracker.snapshot()
                battery = "--" if state.battery_pct is None else f"{state.battery_pct:3.0f}%"
                print(
                    f"t={state.timestamp_s:8.1f}s  mode={state.mode:<10} armed={state.armed!s:<5} "
                    f"lat={state.position.lat_deg:.6f} lon={state.position.lon_deg:.6f} "
                    f"alt={state.position.alt_m:6.1f} m  "
                    f"v={state.velocity.horizontal_norm:5.2f} m/s  hdg={state.heading_deg:5.1f}  "
                    f"batt={battery}"
                )
        finally:
            link.close()
        return 0


class FlightCommand(Command):
    """Takes off, flies one guidance law for the requested time, lands, reports.

    The law comes from a subclass hook because it can only be constructed once the
    vehicle is up and the conditions are set; everything either experiment does around
    that is identical.
    """

    def add_arguments(self, sub: argparse.ArgumentParser) -> None:
        self._add_link(sub)
        sub.add_argument("--altitude", type=float, default=20.0, help="takeoff altitude, m")
        sub.add_argument("--rate", type=float, default=10.0, help="control loop rate, Hz")
        sub.add_argument("--max-speed", type=float, default=5.0, help="horizontal speed cap, m/s")
        sub.add_argument("--kp", type=float, default=0.8)
        sub.add_argument("--wind", type=float, default=0.0, help="simulated wind speed, m/s")
        sub.add_argument("--wind-dir", type=float, default=90.0, help="wind direction, deg")
        sub.add_argument(
            "--turbulence",
            type=float,
            default=0.0,
            help="simulated wind turbulence (SIM_WIND_TURB); 0 is steady wind",
        )
        sub.add_argument("--settle-threshold", type=float, default=2.0)
        sub.add_argument("--log", default=None, help="where to write the CSV log")
        self._add_summary(sub)
        sub.add_argument(
            "--stream-only",
            action="store_true",
            help="do not keep cycles in memory; the CSV log becomes the only record",
        )
        self.add_law_arguments(sub)

    def add_law_arguments(self, sub: argparse.ArgumentParser) -> None:
        raise NotImplementedError

    def build_law(self, args: argparse.Namespace) -> GuidanceLaw:
        raise NotImplementedError

    def describe(self, args: argparse.Namespace) -> dict[str, Any]:
        """What the run was aiming at, for the summary."""
        return {}

    def run(self, args: argparse.Namespace) -> int:
        log_path = Path(args.log or LOG_DIR / f"{self.name}.csv")
        with airborne(
            args.url, altitude_m=args.altitude, connect_timeout_s=args.timeout
        ) as vehicle:
            conditions = self._apply_wind(vehicle, args)
            runner = GuidanceRunner(
                vehicle.link,
                vehicle.tracker,
                rate_hz=args.rate,
                retain_samples=not args.stream_only,
            )
            with CsvRecorder(log_path) as recorder:
                report = runner.fly(
                    self.build_law(args), args.seconds, sink=recorder, label=self.name
                )
        self._emit(
            {
                "run": self.name,
                "log": str(log_path),
                "tracking": self._tracking(args, report),
                "conditions": conditions,
                **self.describe(args),
                "loop": report.loop.as_dict(),
            },
            args.summary,
        )
        return 0

    @staticmethod
    def _limits(args: argparse.Namespace) -> Limits:
        return Limits(max_horizontal_speed=args.max_speed, max_vertical_speed=2.0)

    @staticmethod
    def _point(args: argparse.Namespace) -> NED:
        """`down` is minus the altitude above home, matching MAV_FRAME_LOCAL_NED."""
        return NED(args.north, args.east, -args.altitude)

    @staticmethod
    def _apply_wind(vehicle: Vehicle, args: argparse.Namespace) -> dict[str, float]:
        """Set the simulator's wind to exactly what this run asked for.

        Steady wind alone is a soft test: the autopilot simply leans into it and the
        position error stays near zero. Turbulence is what actually pushes the vehicle
        off the target, so the gusty runs are the ones worth quoting.

        Always sent, including for a calm run. Skipping it when the speed is zero left
        the still-air run flying in whatever wind the previous run had set — the
        simulator keeps its parameters until something changes them, and
        `scripts/measure.sh` ends on a storm. That made the calm column a claim about
        the container's history rather than a measurement, and its recorded conditions
        an empty dict either way.
        """
        link, tracker = vehicle.link, vehicle.tracker
        return {
            "SIM_WIND_SPD": link.set_param("SIM_WIND_SPD", args.wind, tracker),
            "SIM_WIND_DIR": link.set_param("SIM_WIND_DIR", args.wind_dir, tracker),
            "SIM_WIND_TURB": link.set_param("SIM_WIND_TURB", args.turbulence, tracker),
        }

    def _tracking(
        self, args: argparse.Namespace, report: RunReport
    ) -> dict[str, float | str | None] | None:
        """Summarise a finished run — unless it was told not to keep it in memory.

        Settle time is found by scanning the series backwards, so a summary needs the
        whole run at once. In `--stream-only` mode that is exactly what must not happen
        inside the flying process, so the analysis moves out of it: `carrot-guide report
        <log>` gives the same numbers afterwards, in a process that is not holding an
        aircraft up.
        """
        if not report.samples:
            return None
        return summarise(
            report.samples, label=self.name, settle_threshold_m=args.settle_threshold
        ).as_dict()


class HoldCommand(FlightCommand):
    """F3: fly to a point and hold it, optionally in wind."""

    def add_law_arguments(self, sub: argparse.ArgumentParser) -> None:
        # Only the hold law has a derivative term; see OrbitCommand.build_law.
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


class OrbitCommand(FlightCommand):
    """F4: fly a circle around the takeoff point."""

    def add_law_arguments(self, sub: argparse.ArgumentParser) -> None:
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
        # No kd: this law is a tangential term plus a proportional radial one, and never
        # reads the damping gain. It used to be offered on the command line all the same,
        # where it silently did nothing.
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


class LatencyCommand(Command):
    """Measure command-to-reaction latency over several velocity steps."""

    def add_arguments(self, sub: argparse.ArgumentParser) -> None:
        self._add_link(sub)
        sub.add_argument("--altitude", type=float, default=20.0)
        sub.add_argument("--trials", type=int, default=5)
        sub.add_argument("--step-speed", type=float, default=3.0)
        self._add_summary(sub)

    def run(self, args: argparse.Namespace) -> int:
        with airborne(
            args.url, altitude_m=args.altitude, connect_timeout_s=args.timeout
        ) as vehicle:
            latencies = measure_command_latency(
                vehicle, trials=args.trials, step_speed_mps=args.step_speed
            )
        summary = summarise_latency(latencies).as_dict()
        self._emit({"run": self.name, "latency": summary}, args.summary)
        return 0


class ReportCommand(Command):
    """Re-derive the numbers (and optionally the plot) from a recorded log."""

    def add_arguments(self, sub: argparse.ArgumentParser) -> None:
        sub.add_argument("log")
        sub.add_argument("--plot", default=None, help="write a figure to this path")
        sub.add_argument(
            "--circle",
            default=None,
            help="overlay the target as north,east or north,east,radius",
        )
        sub.add_argument("--settle-threshold", type=float, default=2.0)
        self._add_summary(sub)

    def run(self, args: argparse.Namespace) -> int:
        samples = load_samples(args.log)
        summary = summarise(samples, settle_threshold_m=args.settle_threshold)
        payload: dict[str, Any] = {"log": args.log, "tracking": summary.as_dict()}
        if args.plot:
            from carrot_guide.plots import plot_run

            centre, radius = self._parse_circle(args.circle)
            payload["plot"] = str(
                plot_run(samples, args.plot, title=summary.label, centre=centre, radius_m=radius)
            )
        self._emit(payload, args.summary)
        return 0

    @staticmethod
    def _parse_circle(text: str | None) -> tuple[tuple[float, float] | None, float | None]:
        """Parse `north,east[,radius]` so a plot can show what was being aimed at."""
        if not text:
            return None, None
        parts = [float(part) for part in text.split(",")]
        if len(parts) == 2:
            return (parts[0], parts[1]), None
        if len(parts) == 3:
            return (parts[0], parts[1]), parts[2]
        raise SystemExit("--circle takes north,east or north,east,radius")


COMMANDS: tuple[Command, ...] = (
    TelemetryCommand("telemetry", "print live telemetry"),
    HoldCommand("hold", "fly to a point and hold it"),
    OrbitCommand("orbit", "fly a circle"),
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
