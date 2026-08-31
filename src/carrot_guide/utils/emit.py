from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any


def emit_json(payload: dict[str, Any], path: str | None = None) -> None:
    """Also to `path`: a wrapper's own output on stdout (memray's banner) corrupts a capture."""
    text = json.dumps(payload, indent=2, ensure_ascii=False)
    sys.stdout.write(text + "\n")
    if path:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text + "\n", encoding="utf-8")
