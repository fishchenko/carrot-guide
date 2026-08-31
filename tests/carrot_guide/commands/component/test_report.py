"""`report` run the way the command line runs it: argv in, JSON and a figure out.

The one subcommand that needs no vehicle, so the whole path — parser, log, metrics,
plots — is exercised in process.
"""

import json

import pytest

from carrot_guide.cli import main
from carrot_guide.recording import write_samples


@pytest.mark.component
def test_report_summarises_a_log_without_touching_a_socket(tmp_path, capsys, sample_row):
    log = tmp_path / "run.csv"
    write_samples(log, [sample_row, sample_row])

    assert main(["report", str(log)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["log"] == str(log)
    assert payload["tracking"]["mean_error_m"] == pytest.approx(sample_row.error_m)


@pytest.mark.component
def test_report_can_draw_the_figure_too(tmp_path, capsys, sample_row):
    log = tmp_path / "run.csv"
    write_samples(log, [sample_row, sample_row])
    figure = tmp_path / "run.png"

    assert main(["report", str(log), "--plot", str(figure), "--circle", "25,0"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["plot"] == str(figure)
    assert figure.stat().st_size > 0
