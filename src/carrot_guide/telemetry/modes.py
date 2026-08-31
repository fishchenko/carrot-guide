from __future__ import annotations

MAV_MODE_FLAG_SAFETY_ARMED = 0b1000_0000
HEADING_UNKNOWN = 65535

# ArduCopter custom_mode numbers; the gaps are modes this project never uses.
MODE_NAME_BY_NUMBER: dict[int, str] = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    16: "POSHOLD",
    20: "GUIDED_NOGPS",
}

MODE_NUMBER_BY_NAME: dict[str, int] = {
    name: number for number, name in MODE_NAME_BY_NUMBER.items()
}


def mode_name(custom_mode: int) -> str:
    return MODE_NAME_BY_NUMBER.get(custom_mode, f"MODE_{custom_mode}")
