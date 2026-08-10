"""Command line: every experiment in the spec, one subcommand each.

Each flying command writes a CSV log and prints a JSON summary, so a run is
reproducible and its numbers can go straight into the README without retyping.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Callable, Sequence

from carrot_guide.guidance import Gains, HoldPoint, Limits, Orbit
from carrot_guide.link import MavlinkLink
from carrot_guide.metrics import summarise, summarise_latency
from carrot_guide.mission import airborne, measure_command_latency
from carrot_guide.recording import CsvRecorder, load_samples
from carrot_guide.runner import GuidanceLaw, GuidanceRunner, RunReport
from carrot_guide.state import NED
from carrot_guide.telemetry import TelemetryTracker

DEFAULT_URL = "tcp:127.0.0.1:5760"
LOG_DIR = Path("logs")


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


def _apply_wind(
    link: MavlinkLink,
    tracker: TelemetryTracker,
    speed_mps: float,
    direction_deg: float,
    turbulence: float,
) -> dict[str, float]:
    """Set the simulator's wind to exactly what this run asked for.

    Steady wind alone is a soft test: the autopilot simply leans into it and the
    position error stays near zero. Turbulence is what actually pushes the vehicle off
    the target, so the gusty runs are the ones worth quoting.

    Always sent, including for a calm run. Skipping it when the speed is zero left the
    still-air run flying in whatever wind the previous run had set — the simulator keeps
    its parameters until something changes them, and `scripts/measure.sh` ends on a
    storm. That made the calm column a claim about the container's history rather than a
    measurement, and its recorded conditions an empty dict either way.
    """
    return {
        "SIM_WIND_SPD": link.set_param("SIM_WIND_SPD", speed_mps, tracker),
        "SIM_WIND_DIR": link.set_param("SIM_WIND_DIR", direction_deg, tracker),
        "SIM_WIND_TURB": link.set_param("SIM_WIND_TURB", turbulence, tracker),
    }


def _limits(args: argparse.Namespace) -> Limits:
    return Limits(max_horizontal_speed=args.max_speed, max_vertical_speed=2.0)


def _tracking(
    args: argparse.Namespace, report: RunReport, label: str
) -> dict[str, float | str | None] | None:
    """Summarise a finished run — unless it was told not to keep it in memory.

    Settle time is found by scanning the series backwards, so a summary needs the whole
    run at once. In `--stream-only` mode that is exactly what must not happen inside the
    flying process, so the analysis moves out of it: `carrot-guide report <log>` gives
    the same numbers afterwards, in a process that is not holding an aircraft up.
    """
    if not report.samples:
        return None
    return summarise(
        report.samples, label=label, settle_threshold_m=args.settle_threshold
    ).as_dict()


def cmd_telemetry(args: argparse.Namespace) -> int:
    """F1: connect and show what the vehicle is reporting."""
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


def _fly(
    args: argparse.Namespace,
    label: str,
    default_log: str,
    build_law: Callable[[argparse.Namespace], GuidanceLaw],
) -> tuple[Path, RunReport, dict[str, float]]:
    """Take off, fly one law for the requested time, land, and hand back the results.

    The law is built by a callback rather than passed in because it can only be
    constructed once the vehicle is up and the conditions are set — everything either
    experiment does around that is identical.
    """
    log_path = Path(args.log or LOG_DIR / default_log)
    with airborne(args.url, altitude_m=args.altitude, connect_timeout_s=args.timeout) as vehicle:
        conditions = _apply_wind(
            vehicle.link, vehicle.tracker, args.wind, args.wind_dir, args.turbulence
        )
        runner = GuidanceRunner(
            vehicle.link,
            vehicle.tracker,
            rate_hz=args.rate,
            retain_samples=not args.stream_only,
        )
        with CsvRecorder(log_path) as recorder:
            report = runner.fly(build_law(args), args.seconds, sink=recorder, label=label)
    return log_path, report, conditions


def cmd_hold(args: argparse.Namespace) -> int:
    """F3: fly to a point and hold it, optionally in wind."""
    log_path, report, conditions = _fly(
        args,
        "hold",
        "hold.csv",
        lambda a: HoldPoint(
            target=NED(a.north, a.east, -a.altitude),
            gains=Gains(kp_horizontal=a.kp, kd_horizontal=a.kd),
            limits=_limits(a),
            face_target=True,
        ),
    )
    _emit(
        {
            "run": "hold",
            "log": str(log_path),
            "tracking": _tracking(args, report, "hold"),
            "conditions": conditions,
            "target_ned": [args.north, args.east, -args.altitude],
            "loop": report.loop.as_dict(),
        },
        args.summary,
    )
    return 0


def cmd_orbit(args: argparse.Namespace) -> int:
    """F4: fly a circle around the takeoff point."""
    log_path, report, conditions = _fly(
        args,
        "orbit",
        "orbit.csv",
        # No kd: the orbit law is a tangential term plus a proportional radial one, and
        # never reads the damping gain. It used to be offered on the command line all
        # the same, where it silently did nothing.
        lambda a: Orbit(
            centre=NED(a.north, a.east, -a.altitude),
            radius_m=a.radius,
            speed_mps=a.speed,
            clockwise=not a.counter_clockwise,
            gains=Gains(kp_horizontal=a.kp),
            limits=_limits(a),
            lookahead_s=a.lookahead,
        ),
    )
    _emit(
        {
            "run": "orbit",
            "log": str(log_path),
            "tracking": _tracking(args, report, "orbit"),
            "conditions": conditions,
            "centre_ned": [args.north, args.east, -args.altitude],
            "radius_m": args.radius,
            "loop": report.loop.as_dict(),
        },
        args.summary,
    )
    return 0


def cmd_latency(args: argparse.Namespace) -> int:
    """Measure command-to-reaction latency over several velocity steps."""
    with airborne(args.url, altitude_m=args.altitude, connect_timeout_s=args.timeout) as vehicle:
        latencies = measure_command_latency(
            vehicle, trials=args.trials, step_speed_mps=args.step_speed
        )
    _emit({"run": "latency", "latency": summarise_latency(latencies).as_dict()}, args.summary)
    return 0


def cmd_report(args: argparse.Namespace) -> int:
    """Re-derive the numbers (and optionally the plot) from a recorded log."""
    samples = load_samples(args.log)
    summary = summarise(samples, settle_threshold_m=args.settle_threshold)
    payload: dict[str, Any] = {"log": args.log, "tracking": summary.as_dict()}
    if args.plot:
        from carrot_guide.plots import plot_run

        centre, radius = _parse_circle(args.circle)
        payload["plot"] = str(
            plot_run(samples, args.plot, title=summary.label, centre=centre, radius_m=radius)
        )
    _emit(payload, args.summary)
    return 0


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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="carrot-guide", description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    def _add_summary(sub: argparse.ArgumentParser) -> None:
        sub.add_argument(
            "--summary",
            default=None,
            help="also write the JSON summary here, out of reach of anything else on stdout",
        )

    def add_common(sub: argparse.ArgumentParser) -> None:
        sub.add_argument("--url", default=DEFAULT_URL, help="MAVLink endpoint of the simulator")
        sub.add_argument("--timeout", type=float, default=180.0, help="connect/arm timeout, s")

    def add_flight(sub: argparse.ArgumentParser) -> None:
        add_common(sub)
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
        _add_summary(sub)
        sub.add_argument(
            "--stream-only",
            action="store_true",
            help="do not keep cycles in memory; the CSV log becomes the only record",
        )

    telemetry = subparsers.add_parser("telemetry", help="print live telemetry")
    add_common(telemetry)
    telemetry.add_argument("--seconds", type=float, default=10.0)
    telemetry.set_defaults(handler=cmd_telemetry)

    hold = subparsers.add_parser("hold", help="fly to a point and hold it")
    add_flight(hold)
    # Only the hold law has a derivative term; see cmd_orbit.
    hold.add_argument("--kd", type=float, default=0.4)
    hold.add_argument("--north", type=float, default=30.0, help="target offset north, m")
    hold.add_argument("--east", type=float, default=0.0, help="target offset east, m")
    hold.add_argument("--seconds", type=float, default=60.0)
    hold.set_defaults(handler=cmd_hold)

    orbit = subparsers.add_parser("orbit", help="fly a circle")
    add_flight(orbit)
    orbit.add_argument("--north", type=float, default=0.0, help="circle centre north, m")
    orbit.add_argument("--east", type=float, default=0.0, help="circle centre east, m")
    orbit.add_argument("--radius", type=float, default=25.0)
    orbit.add_argument("--speed", type=float, default=3.0, help="tangential speed, m/s")
    orbit.add_argument("--counter-clockwise", action="store_true")
    orbit.add_argument(
        "--lookahead",
        type=float,
        default=0.0,
        help="aim the tangent this many seconds ahead, to cancel the vehicle's turn lag",
    )
    orbit.add_argument("--seconds", type=float, default=90.0)
    orbit.set_defaults(handler=cmd_orbit)

    latency = subparsers.add_parser("latency", help="measure command-to-reaction latency")
    add_common(latency)
    latency.add_argument("--altitude", type=float, default=20.0)
    latency.add_argument("--trials", type=int, default=5)
    latency.add_argument("--step-speed", type=float, default=3.0)
    _add_summary(latency)
    latency.set_defaults(handler=cmd_latency)

    report = subparsers.add_parser("report", help="summarise and plot a recorded log")
    report.add_argument("log")
    report.add_argument("--plot", default=None, help="write a figure to this path")
    report.add_argument(
        "--circle",
        default=None,
        help="overlay the target as north,east or north,east,radius",
    )
    report.add_argument("--settle-threshold", type=float, default=2.0)
    _add_summary(report)
    report.set_defaults(handler=cmd_report)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.handler(args))


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
