import pytest

from carrot_guide.recording import COLUMNS, CsvRecorder, MemorySink, Sample


@pytest.mark.unit
def test_every_field_becomes_a_column(tmp_path, sample_row):
    path = tmp_path / "run.csv"
    with CsvRecorder(path) as recorder:
        recorder.write(sample_row)
    header = path.read_text(encoding="utf-8").splitlines()[0]
    assert header.split(",") == COLUMNS


@pytest.mark.unit
def test_rows_are_flushed_so_a_killed_run_keeps_its_log(tmp_path, sample_row):
    path = tmp_path / "run.csv"
    recorder = CsvRecorder(path)
    recorder.write(sample_row)
    # No close(): the file still has to hold the row.
    assert len(path.read_text(encoding="utf-8").splitlines()) == 2


@pytest.mark.unit
def test_the_log_directory_is_created_on_demand(tmp_path, sample_row):
    path = tmp_path / "nested" / "deeper" / "run.csv"
    with CsvRecorder(path) as recorder:
        recorder.write(sample_row)
    assert path.exists()


@pytest.mark.unit
def test_the_memory_sink_keeps_order(sample_row):
    sink = MemorySink()
    first = sample_row
    second = Sample(**{**sample_row.__dict__, "t_s": 2.0})
    sink.write(first)
    sink.write(second)
    assert [s.t_s for s in sink.samples] == pytest.approx([1.25, 2.0])
