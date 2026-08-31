"""Tests for the parser `cli` assembles out of `COMMANDS`."""

import pytest

from carrot_guide.cli import COMMANDS, build_parser


@pytest.mark.unit
def test_the_parser_knows_every_experiment():
    parser = build_parser()
    for command in ("telemetry", "hold", "orbit", "intercept", "latency", "report"):
        args = parser.parse_args([command, *(["x.csv"] if command == "report" else [])])
        # A `Namespace` is truthy whatever is in it, so assert on the parse.
        assert args.command == command
    with pytest.raises(SystemExit):
        parser.parse_args(["fly-to-the-moon"])


@pytest.mark.unit
def test_every_command_is_reachable_under_its_own_name():
    """A command class that never makes it into COMMANDS is code nothing can run."""
    parser = build_parser()
    for command in COMMANDS:
        argv = [command.name, *(["x.csv"] if command.name == "report" else [])]
        assert parser.parse_args(argv).handler.__self__ is command
