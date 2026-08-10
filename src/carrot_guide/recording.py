"""Flight log: one row per control cycle, written as CSV.

Plain CSV on purpose — the runs are minutes long, and a format that any tool can
open is worth more here than a compact binary one.
"""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import IO, Iterable, Protocol


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


COLUMNS = [field.name for field in fields(Sample)]


class SampleSink(Protocol):
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
    """In-memory sink, used by tests and by short runs that only need the summary."""

    def __init__(self) -> None:
        self.samples: list[Sample] = []

    def write(self, sample: Sample) -> None:
        self.samples.append(sample)


_FLOAT_FIELDS = {
    field.name for field in fields(Sample) if field.type in ("float", float)
}


def load_samples(path: str | Path) -> list[Sample]:
    """Read a log back; the plotting and metrics tools work off this."""
    with Path(path).open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    return [
        Sample(
            **{
                name: (
                    float(row[name])
                    if name in _FLOAT_FIELDS
                    else row[name] == "True"
                    if name == "armed"
                    else row[name]
                )
                for name in COLUMNS
            }
        )
        for row in rows
    ]


def write_samples(path: str | Path, samples: Iterable[Sample]) -> None:
    with CsvRecorder(path) as recorder:
        for sample in samples:
            recorder.write(sample)
