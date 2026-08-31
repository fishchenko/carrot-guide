from __future__ import annotations

import math
from dataclasses import dataclass

EARTH_RADIUS_M = 6_378_137.0


@dataclass(frozen=True)
class GlobalPosition:
    lat_deg: float
    lon_deg: float
    alt_m: float


@dataclass(frozen=True)
class NED:
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
        """Cap horizontal magnitude at `limit`; `down` passes through unchanged."""
        norm = self.horizontal_norm
        if norm <= limit or norm == 0.0:
            return self
        factor = limit / norm
        return NED(self.north * factor, self.east * factor, self.down)


@dataclass(frozen=True)
class VehicleState:
    position: GlobalPosition
    velocity: NED
    heading_deg: float
    armed: bool
    mode: str
    battery_pct: float | None = None
    timestamp_s: float = 0.0


def to_local_ned(position: GlobalPosition, origin: GlobalPosition) -> NED:
    """Flat-earth projection: sub-metre error over the few hundred metres flown here."""
    lat_rad = math.radians(origin.lat_deg)
    north = math.radians(position.lat_deg - origin.lat_deg) * EARTH_RADIUS_M
    east = math.radians(position.lon_deg - origin.lon_deg) * EARTH_RADIUS_M * math.cos(lat_rad)
    down = origin.alt_m - position.alt_m
    return NED(north, east, down)


def from_local_ned(offset: NED, origin: GlobalPosition) -> GlobalPosition:
    lat_rad = math.radians(origin.lat_deg)
    lat = origin.lat_deg + math.degrees(offset.north / EARTH_RADIUS_M)
    lon = origin.lon_deg + math.degrees(offset.east / (EARTH_RADIUS_M * math.cos(lat_rad)))
    return GlobalPosition(lat, lon, origin.alt_m - offset.down)
