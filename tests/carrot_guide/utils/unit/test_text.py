import dataclasses

import pytest

from carrot_guide.utils import parse_bool, parsers_for


@pytest.mark.unit
def test_the_string_false_does_not_read_back_as_true():
    # The trap `bool(text)` falls into, and the reason parse_bool exists.
    assert parse_bool("True") is True
    assert parse_bool("False") is False
    assert parse_bool("") is False


@pytest.mark.unit
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
