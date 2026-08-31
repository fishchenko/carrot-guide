"""Parses argv against one command alone, the way `cli` registers it.

A command's options only exist once it is registered on a subparser, and going through
`cli.build_parser()` for that would register all six: a failure in another command's
options would then fail these tests, and this one's would not be tested in isolation.
"""

from __future__ import annotations

import argparse

from carrot_guide.utils import Command


def parse(command: Command, argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    command.register(subparsers)
    return parser.parse_args([command.name, *(argv or [])])
