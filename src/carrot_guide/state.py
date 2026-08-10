"""Vehicle state and the local frame used by the guidance laws."""

from __future__ import annotations

import math
from dataclasses import dataclass

EARTH_RADIUS_M = 6_378_137.0


@dataclass(frozen=True)
class GlobalPosition:
    """WGS84 position as reported over MAVLink."""

    lat_deg: float
    lon_deg: float
    alt_m: float


@dataclass(frozen=True)
class NED:
    """Offset in the local north-east-down frame, metres."""

    north: float
    east: float
    down: float = 0.0

    def __add__(self, other: NED) -> NED:
        return NED(self.north + other.north, self.east + other.east, self.down + other.down)

    def __sub__(self, other: NED) -> NED:
        return NED(self.north - other.north, self.east - other.east, self.down - other.down)

    def scaled(self, factor: float) -> NED:
        return NED(self.north * factor, self.east * factor, self.down * factor)

    @property
    def horizontal_norm(self) -> float:
        return math.hypot(self.north, self.east)

    def clamped_horizontally(self, limit: float) -> NED:
        """Cap horizontal magnitude at `limit`, keeping direction and vertical part."""
        norm = self.horizontal_norm
        if norm <= limit or norm == 0.0:
            return self
        factor = limit / norm
        return NED(self.north * factor, self.east * factor, self.down)


@dataclass(frozen=True)
class VehicleState:
    """Everything the control loop needs for one iteration."""

    position: GlobalPosition
    velocity: NED
    heading_deg: float
    armed: bool
    mode: str
    battery_pct: float | None = None
    timestamp_s: float = 0.0


def to_local_ned(position: GlobalPosition, origin: GlobalPosition) -> NED:
    """Flat-earth projection of `position` relative to `origin`.

    Accurate to well under a metre over the ranges this project flies (hundreds of
    metres), which keeps the guidance laws free of a full geodesy dependency.
    """
    lat_rad = math.radians(origin.lat_deg)
    north = math.radians(position.lat_deg - origin.lat_deg) * EARTH_RADIUS_M
    east = math.radians(position.lon_deg - origin.lon_deg) * EARTH_RADIUS_M * math.cos(lat_rad)
    down = origin.alt_m - position.alt_m
    return NED(north, east, down)


def from_local_ned(offset: NED, origin: GlobalPosition) -> GlobalPosition:
    """Inverse of `to_local_ned`."""
    lat_rad = math.radians(origin.lat_deg)
    lat = origin.lat_deg + math.degrees(offset.north / EARTH_RADIUS_M)
    lon = origin.lon_deg + math.degrees(offset.east / (EARTH_RADIUS_M * math.cos(lat_rad)))
    return GlobalPosition(lat, lon, origin.alt_m - offset.down)
