import pytest

from carrot_guide.mission import airborne
from carrot_guide.recording import Sample

from tests.simulator import ALTITUDE_M, SITL_URL, Flight


@pytest.fixture(scope="session")
def flight() -> Flight:
    """One takeoff for every simulator test: each flight leaves the vehicle where the
    next one starts, and a takeoff per file would cost more than the flights themselves.

    Lives here rather than beside the tests because they are spread across the packages
    they exercise, and this is the only conftest above all of them.
    """
    with airborne(SITL_URL or "", altitude_m=ALTITUDE_M) as vehicle:
        yield Flight(vehicle, vehicle.tracker.snapshot(), vehicle.position_ned)


@pytest.fixture(scope="session")
def vehicle(flight: Flight):
    return flight.vehicle


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
