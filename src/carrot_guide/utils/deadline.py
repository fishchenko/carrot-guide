from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class Deadline:
    at_s: float
    monotonic: Callable[[], float] = time.monotonic

    @classmethod
    def after(cls, seconds: float, monotonic: Callable[[], float] = time.monotonic) -> "Deadline":
        return cls(monotonic() + seconds, monotonic)

    @property
    def remaining_s(self) -> float:
        """Time left, negative once the deadline has passed."""
        return self.at_s - self.monotonic()

    @property
    def expired(self) -> bool:
        return self.remaining_s <= 0.0

    def slice(self, cap_s: float) -> float:
        """Floored at zero: a blocking read takes a negative timeout as no-wait or as forever."""
        return max(0.0, min(cap_s, self.remaining_s))
