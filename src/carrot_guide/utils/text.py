from __future__ import annotations

from typing import Callable, get_type_hints


def parse_bool(text: str) -> bool:
    """Inverse of `str(bool)`: `bool("False")` is true, this is not."""
    return text == "True"


# An unlisted annotation raises in `parsers_for`, rather than reading the column back as str.
TEXT_PARSER_BY_TYPE: dict[type, Callable[[str], object]] = {
    float: float,
    str: str,
    bool: parse_bool,
}


def parsers_for(cls: type) -> dict[str, Callable[[str], object]]:
    """`get_type_hints`, not a field's `.type`: postponed annotations make that a string."""
    return {name: TEXT_PARSER_BY_TYPE[hint] for name, hint in get_type_hints(cls).items()}
