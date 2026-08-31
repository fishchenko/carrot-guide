from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from carrot_guide.commands.vehicle import VehicleCommand
from carrot_guide.guidance import Limits
from carrot_guide.metrics import DEFAULT_SETTLE_THRESHOLD_M, summarise
from carrot_guide.mission import Vehicle, airborne
from carrot_guide.recording import CsvRecorder
from carrot_guide.runner import GuidanceLaw, GuidanceRunner, RunReport
from carrot_guide.state import NED
from carrot_guide.utils import emit_json


LOG_DIR = Path("logs")


class FlightCommand(VehicleCommand):
    """Takes off, flies one guidance law for the requested time, lands, reports."""

    settle_threshold_default = DEFAULT_SETTLE_THRESHOLD_M

    def add_arguments(self, sub: argparse.ArgumentParser) -> None:
        self._add_link(sub)
        sub.add_argument("--altitude", type=float, default=20.0, help="takeoff altitude, m")
        sub.add_argument("--rate", type=float, default=10.0, help="control loop rate, Hz")
        sub.add_argument("--max-speed", type=float, default=5.0, help="horizontal speed cap, m/s")
        sub.add_argument("--wind", type=float, default=0.0, help="simulated wind speed, m/s")
        sub.add_argument("--wind-dir", type=float, default=90.0, help="wind direction, deg")
        sub.add_argument(
            "--turbulence",
            type=float,
            default=0.0,
            help="simulated wind turbulence (SIM_WIND_TURB); 0 is steady wind",
        )
        sub.add_argument(
            "--settle-threshold",
            type=float,
            default=self.settle_threshold_default,
            help="error under which the vehicle counts as arrived; zero leaves the whole "
            "run as the window the statistics are taken over",
        )
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
        emit_json(
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
        return NED(args.north, args.east, -args.altitude)

    @staticmethod
    def _apply_wind(vehicle: Vehicle, args: argparse.Namespace) -> dict[str, float]:
        """Always written, including at zero: the simulator keeps the previous run's wind."""
        link, tracker = vehicle.link, vehicle.tracker
        return {
            "SIM_WIND_SPD": link.set_param("SIM_WIND_SPD", args.wind, tracker),
            "SIM_WIND_DIR": link.set_param("SIM_WIND_DIR", args.wind_dir, tracker),
            "SIM_WIND_TURB": link.set_param("SIM_WIND_TURB", args.turbulence, tracker),
        }

    def _tracking(
        self, args: argparse.Namespace, report: RunReport
    ) -> dict[str, float | str | None] | None:
        """None under --stream-only: a summary needs the whole run, so `report` does it later."""
        if not report.samples:
            return None
        return summarise(
            report.samples, label=self.name, settle_threshold_m=args.settle_threshold
        ).as_dict()
