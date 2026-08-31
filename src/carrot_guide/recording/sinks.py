from __future__ import annotations

import csv
from dataclasses import asdict
from pathlib import Path
from typing import IO, Iterable, Protocol

from carrot_guide.recording.sample import COLUMNS, Sample


class SampleSink(Protocol):
    def write(self, sample: Sample) -> None: ...


class CsvRecorder:
    """Flushes every row, so a killed run keeps its log."""

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
    def __init__(self) -> None:
        self.samples: list[Sample] = []

    def write(self, sample: Sample) -> None:
        self.samples.append(sample)


def write_samples(path: str | Path, samples: Iterable[Sample]) -> None:
    with CsvRecorder(path) as recorder:
        for sample in samples:
            recorder.write(sample)
