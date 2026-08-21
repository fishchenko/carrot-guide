"""Tests for the general pieces with no aircraft in them.

`Deadline` is driven by a fake clock for the same reason the control loop is: every
timeout in `link` and `mission` is built from it, and none of them should cost a test
the wall-clock time they allow in flight.
"""

import dataclasses
import json

import pytest

from carrot_guide.recording import Sample
from carrot_guide.utils import Deadline, emit_json, parse_bool, parsers_for, percentile

from conftest import FakeClock


def test_a_deadline_counts_down_against_its_own_clock():
    clock = FakeClock()
    deadline = Deadline.after(5.0, monotonic=clock.monotonic)

    assert not deadline.expired
    clock.advance(3.0)
    assert deadline.remaining_s == pytest.approx(2.0)
    clock.advance(2.0)
    assert deadline.expired


def test_a_slice_never_reaches_past_the_deadline():
    clock = FakeClock()
    deadline = Deadline.after(1.0, monotonic=clock.monotonic)

    assert deadline.slice(0.5) == pytest.approx(0.5)
    clock.advance(0.8)
    # Only 0.2 s of budget is left, so that is all a read may block for.
    assert deadline.slice(0.5) == pytest.approx(0.2)


def test_a_lapsed_deadline_slices_to_zero_rather_than_to_a_negative_timeout():
    """A negative timeout means "no wait" to one socket layer and "forever" to another."""
    clock = FakeClock()
    deadline = Deadline.after(1.0, monotonic=clock.monotonic)
    clock.advance(9.0)

    assert deadline.expired
    assert deadline.slice(0.5) == 0.0


def test_a_deadline_is_an_instant_not_a_timer():
    """A callee cannot re-base its caller's budget; `mission.launch` relies on it."""
    clock = FakeClock()
    deadline = Deadline.after(30.0, monotonic=clock.monotonic)

    with pytest.raises(dataclasses.FrozenInstanceError):
        deadline.at_s = clock.now + 600.0


def test_the_string_false_does_not_read_back_as_true():
    # The trap `bool(text)` falls into, and the reason parse_bool exists.
    assert parse_bool("True") is True
    assert parse_bool("False") is False
    assert parse_bool("") is False


def test_columns_are_parsed_by_declared_type_not_by_name():
    # `Sample` alone cannot show this: its only bool column is called `armed`, so
    # dispatching on the name would give the same answer. These names would all be
    # parsed as strings by a name-based table.
    @dataclasses.dataclass(frozen=True)
    class Row:
        landed: bool
        altitude: float
        note: str

    parsers = parsers_for(Row)
    assert parsers["landed"] is parse_bool
    assert parsers["altitude"] is float
    assert parsers["note"] is str
    assert parsers_for(Sample)["armed"] is parse_bool


def test_a_percentile_is_always_a_value_the_run_produced():
    values = [3.0, 1.0, 2.0, 5.0, 4.0]
    assert percentile(values, 0.95) in values
    assert percentile(values, 0.5) in values
    assert percentile(values, 0.0) == 1.0


def test_an_empty_series_summarises_rather_than_raising():
    """`runner.LoopStats` summarises a loop that never ticked; it must not crash there.

    This is also why `statistics.quantiles` is not used: it raises below two points.
    """
    assert percentile([], 0.95) == 0.0


def test_a_summary_reaches_the_file_even_when_something_else_owns_stdout(tmp_path, capsys):
    """The reason emit_json writes twice: the file must not depend on what else printed."""
    path = tmp_path / "nested" / "run.json"
    print("a profiler banner nobody asked for")
    emit_json({"mean_error_m": 0.01}, str(path))
    print("and a trailing one")

    # Captured stdout is unparseable, which is exactly the failure the file sidesteps.
    with pytest.raises(json.JSONDecodeError):
        json.loads(capsys.readouterr().out)
    assert json.loads(path.read_text(encoding="utf-8")) == {"mean_error_m": 0.01}


def test_without_a_path_a_summary_only_goes_to_stdout(tmp_path, capsys):
    emit_json({"trials": 5})
    assert json.loads(capsys.readouterr().out) == {"trials": 5}
    assert list(tmp_path.iterdir()) == []
