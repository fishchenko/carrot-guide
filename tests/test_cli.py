"""Tests for the parts of the command line that do not need a vehicle."""

import json

import pytest

from carrot_guide.cli import COMMANDS, ReportCommand, build_parser, main
from carrot_guide.recording import write_samples
from carrot_guide.state import NED


def test_the_parser_knows_every_experiment():
    parser = build_parser()
    for command in ("telemetry", "hold", "orbit", "latency", "report"):
        args = parser.parse_args([command, *(["x.csv"] if command == "report" else [])])
        # A `Namespace` is truthy whatever is in it, so assert on the parse.
        assert args.command == command
    with pytest.raises(SystemExit):
        parser.parse_args(["fly-to-the-moon"])


def test_every_command_is_reachable_under_its_own_name():
    """A command class that never makes it into COMMANDS is code nothing can run."""
    parser = build_parser()
    for command in COMMANDS:
        argv = [command.name, *(["x.csv"] if command.name == "report" else [])]
        assert parser.parse_args(argv).handler.__self__ is command


def test_orbit_defaults_are_the_ones_the_readme_quotes():
    args = build_parser().parse_args(["orbit"])
    assert args.radius == 25.0
    assert args.rate == 10.0
    assert args.counter_clockwise is False


@pytest.mark.parametrize(
    "text, expected",
    [
        (None, (None, None)),
        ("", (None, None)),
        ("25,0", (NED(25.0, 0.0), None)),
        ("0,0,25", (NED(0.0, 0.0), 25.0)),
        ("-10.5,3,12.25", (NED(-10.5, 3.0), 12.25)),
    ],
)
def test_circle_overlay_parsing(text, expected):
    assert ReportCommand._parse_circle(text) == expected


def test_a_malformed_circle_is_refused():
    with pytest.raises(SystemExit):
        ReportCommand._parse_circle("1,2,3,4")


def test_report_summarises_a_log_without_touching_a_socket(tmp_path, capsys, sample_row):
    log = tmp_path / "run.csv"
    write_samples(log, [sample_row, sample_row])

    assert main(["report", str(log)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["log"] == str(log)
    assert payload["tracking"]["mean_error_m"] == pytest.approx(sample_row.error_m)


def test_report_can_draw_the_figure_too(tmp_path, capsys, sample_row):
    log = tmp_path / "run.csv"
    write_samples(log, [sample_row, sample_row])
    figure = tmp_path / "run.png"

    assert main(["report", str(log), "--plot", str(figure), "--circle", "25,0"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["plot"] == str(figure)
    assert figure.stat().st_size > 0
