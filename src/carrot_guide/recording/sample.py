from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

from carrot_guide.utils import parsers_for


@dataclass(frozen=True)
class Sample:
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


# Keyed by each field's declared type, not its name; an uncovered annotation raises here.
_COLUMN_PARSERS = parsers_for(Sample)

COLUMNS = list(_COLUMN_PARSERS)


def load_samples(path: str | Path) -> list[Sample]:
    with Path(path).open(newline="", encoding="utf-8") as handle:
        return [_sample_from_row(row) for row in csv.DictReader(handle)]


def _sample_from_row(row: dict[str, str]) -> Sample:
    return Sample(**{name: parse(row[name]) for name, parse in _COLUMN_PARSERS.items()})
