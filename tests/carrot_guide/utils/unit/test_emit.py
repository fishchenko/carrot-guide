import json

import pytest

from carrot_guide.utils import emit_json


@pytest.mark.unit
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


@pytest.mark.unit
def test_without_a_path_a_summary_only_goes_to_stdout(tmp_path, capsys):
    emit_json({"trials": 5})
    assert json.loads(capsys.readouterr().out) == {"trials": 5}
    assert list(tmp_path.iterdir()) == []
