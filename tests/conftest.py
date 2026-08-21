import pytest

from carrot_guide.recording import Sample


class Message:
    """A hand-built MAVLink message: `get_type()` plus whatever fields a test needs.

    This is all the tracker and the link layer ever see of a message, which is what
    keeps the field scaling — 1e7 degrees, millimetres, centimetres per second — under
    test without a simulator anywhere near it. Shared because both the parsing tests
    and the transport tests build the same stub.
    """

    def __init__(self, kind: str, **fields: object) -> None:
        self._kind = kind
        self.__dict__.update(fields)

    def get_type(self) -> str:
        return self._kind


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


@pytest.fixture
def sample_row() -> Sample:
    """One plausible log row, for tests that need a log but not a flight."""
    return Sample(
        t_s=1.25,
        label="hold",
        lat_deg=50.4501,
        lon_deg=30.5234,
        north_m=24.5,
        east_m=-0.75,
        down_m=-15.0,
        vn=0.1,
        ve=-0.2,
        vd=0.0,
        cmd_vn=0.4,
        cmd_ve=0.0,
        cmd_vd=-0.1,
        error_m=0.52,
        lateness_ms=0.3,
        mode="GUIDED",
        armed=True,
    )
