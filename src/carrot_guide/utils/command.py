from __future__ import annotations

import argparse
from dataclasses import dataclass


@dataclass(frozen=True)
class Command:
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
    def _add_summary(sub: argparse.ArgumentParser) -> None:
        sub.add_argument(
            "--summary",
            default=None,
            help="also write the JSON summary here, out of reach of anything else on stdout",
        )
