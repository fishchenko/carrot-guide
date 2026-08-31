from __future__ import annotations

import argparse

from carrot_guide.link import DEFAULT_SIMULATOR_URL
from carrot_guide.utils import Command


class VehicleCommand(Command):
    @staticmethod
    def _add_link(sub: argparse.ArgumentParser) -> None:
        sub.add_argument(
            "--url", default=DEFAULT_SIMULATOR_URL, help="MAVLink endpoint of the simulator"
        )
        sub.add_argument("--timeout", type=float, default=180.0, help="connect/arm timeout, s")
