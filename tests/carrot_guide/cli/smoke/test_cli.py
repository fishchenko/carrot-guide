"""The installed console script, run the way a user runs it.

Everything under `tests/unit` imports from `src/` (`pythonpath` in `pyproject.toml`), so
it stays green with a broken install — a stale editable one after the checkout is moved,
say. These two go through what pip wrote instead, and fail rather than skip when there
is nothing installed: that is the gap they exist to close.
"""

import subprocess
import sys
from pathlib import Path

import pytest
# Beside the interpreter running the tests, not whatever `carrot-guide` PATH resolves to:
# `make smoke` calls the venv's pytest without activating the venv.
SCRIPT = Path(sys.executable).with_name("carrot-guide")


def help_text(*argv: str) -> str:
    completed = subprocess.run(
        argv, capture_output=True, text=True, timeout=60, check=True
    )
    return completed.stdout


@pytest.mark.smoke
def test_the_installed_console_script_prints_its_help():
    assert SCRIPT.exists(), f"{SCRIPT} is missing: make venv"
    assert "usage: carrot-guide" in help_text(str(SCRIPT), "--help")


@pytest.mark.smoke
def test_the_module_entry_point_the_scripts_use_still_runs():
    # `scripts/measure.sh` and half the Makefile go through `-m carrot_guide.cli`, which
    # resolves the installed package rather than the source tree.
    assert "usage: carrot-guide" in help_text(sys.executable, "-m", "carrot_guide.cli", "--help")
