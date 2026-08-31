from __future__ import annotations

import argparse

from carrot_guide.commands.vehicle import VehicleCommand
from carrot_guide.metrics import summarise_latency
from carrot_guide.mission import airborne, measure_command_latency
from carrot_guide.utils import emit_json


class LatencyCommand(VehicleCommand):
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
        emit_json({"run": self.name, "latency": summary}, args.summary)
        return 0
