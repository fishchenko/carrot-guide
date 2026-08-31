from __future__ import annotations

import argparse

from carrot_guide.commands.vehicle import VehicleCommand
from carrot_guide.link import MavlinkLink
from carrot_guide.telemetry import TelemetryTracker
from carrot_guide.utils import Deadline


class TelemetryCommand(VehicleCommand):
    def add_arguments(self, sub: argparse.ArgumentParser) -> None:
        self._add_link(sub)
        sub.add_argument("--seconds", type=float, default=10.0)

    def run(self, args: argparse.Namespace) -> int:
        link = MavlinkLink.connect(args.url, timeout_s=args.timeout)
        tracker = TelemetryTracker()
        link.request_streams()
        deadline = Deadline.after(args.seconds)
        try:
            while not deadline.expired:
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
