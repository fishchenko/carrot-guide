from __future__ import annotations


class Message:
    """A hand-built MAVLink message: `get_type()` plus whatever fields a test needs.

    This is all the tracker and the link layer ever see of a message, which is what
    keeps the field scaling — 1e7 degrees, millimetres, centimetres per second — under
    test without a simulator anywhere near it.
    """

    def __init__(self, kind: str, **fields: object) -> None:
        self._kind = kind
        self.__dict__.update(fields)

    def get_type(self) -> str:
        return self._kind


def global_position(**overrides: float) -> Message:
    """A GLOBAL_POSITION_INT with plausible field scaling: 1e7 degrees, mm, cm/s, cdeg."""
    fields: dict[str, float] = {
        "lat": int(50.4501 * 1e7),
        "lon": int(30.5234 * 1e7),
        "relative_alt": 20_000,
        "vx": 150,
        "vy": -50,
        "vz": 25,
        "hdg": 9_000,
        "time_boot_ms": 12_345,
    }
    fields.update(overrides)
    return Message("GLOBAL_POSITION_INT", **fields)
