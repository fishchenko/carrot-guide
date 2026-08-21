"""Flight log: one row per control cycle, written as CSV.

Plain CSV on purpose — the runs are minutes long, and a format that any tool can
open is worth more here than a compact binary one.
"""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import IO, Iterable, Protocol

from carrot_guide.utils import parsers_for


@dataclass(frozen=True)
class Sample:
    """One control cycle: what was measured, what was commanded, how late it was."""

    t_s: float
    label: str
    lat_deg: float
    lon_deg: float
    north_m: float
    east_m: float
    down_m: float
    vn: float
    ve: float
    vd: float
    cmd_vn: float
    cmd_ve: float
    cmd_vd: float
    error_m: float
    lateness_ms: float
    mode: str
    armed: bool


# One parser per column, keyed by the field's declared type rather than by its name:
# `armed` is read back as a bool because it is declared one, not because it is called
# that. A column annotated with a type the table does not cover raises here.
_COLUMN_PARSERS = parsers_for(Sample)

# Derived from the parser table rather than from `fields(Sample)` a second time: two
# derivations of one list agree until the day they do not, and the CSV header and the
# CSV reader are exactly the pair that must never disagree.
COLUMNS = list(_COLUMN_PARSERS)


class SampleSink(Protocol):
    """Where the control loop puts a finished cycle; see the two below."""

    def write(self, sample: Sample) -> None: ...


class CsvRecorder:
    """Append-only CSV sink. Flushes every row: a killed run keeps its log."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle: IO[str] = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=COLUMNS)
        self._writer.writeheader()

    def write(self, sample: Sample) -> None:
        self._writer.writerow(asdict(sample))
        self._handle.flush()

    def close(self) -> None:
        self._handle.close()

    def __enter__(self) -> "CsvRecorder":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


class MemorySink:
    """In-memory sink, so a test can read back what the loop wrote without a file."""

    def __init__(self) -> None:
        self.samples: list[Sample] = []

    def write(self, sample: Sample) -> None:
        self.samples.append(sample)


def load_samples(path: str | Path) -> list[Sample]:
    """Read a log back; the plotting and metrics tools work off this."""
    with Path(path).open(newline="", encoding="utf-8") as handle:
        return [_sample_from_row(row) for row in csv.DictReader(handle)]


def _sample_from_row(row: dict[str, str]) -> Sample:
    """Rebuild one cycle from its CSV row, parsing each column by its declared type."""
    return Sample(**{name: parse(row[name]) for name, parse in _COLUMN_PARSERS.items()})


def write_samples(path: str | Path, samples: Iterable[Sample]) -> None:
    with CsvRecorder(path) as recorder:
        for sample in samples:
            recorder.write(sample)
