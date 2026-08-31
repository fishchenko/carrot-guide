from __future__ import annotations


class FakeClock:
    """A clock that advances only on `sleep`, plus whatever the test adds.

    Shared because three test files each grew their own under this name with three
    different capabilities, so a test moving between files silently changed clock.
    """

    def __init__(self, start: float = 1000.0, sleep_overshoot: float = 0.0) -> None:
        self.now = start
        self.sleep_overshoot = sleep_overshoot
        self.sleeps: list[float] = []

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        self.now += max(0.0, seconds) + self.sleep_overshoot

    def advance(self, seconds: float) -> None:
        self.now += seconds


class SpinningClock(FakeClock):
    """A clock where reading the time costs time, so a busy-wait terminates."""

    def __init__(self, step: float = 0.0005, **kwargs: float) -> None:
        super().__init__(**kwargs)
        self.step = step

    def monotonic(self) -> float:
        self.now += self.step
        return self.now
