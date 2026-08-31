import pytest

from carrot_guide.recording import CsvRecorder, Sample, load_samples
from carrot_guide.utils import parse_bool, parsers_for


@pytest.mark.unit
def test_a_log_round_trips_through_csv(tmp_path, sample_row):
    path = tmp_path / "run.csv"
    with CsvRecorder(path) as recorder:
        recorder.write(sample_row)
        recorder.write(sample_row)

    restored = load_samples(path)
    assert restored == [sample_row, sample_row]


@pytest.mark.unit
def test_a_bool_column_is_read_back_as_a_bool():
    # `armed` is the only one, and "False" read as a string is truthy.
    assert parsers_for(Sample)["armed"] is parse_bool
