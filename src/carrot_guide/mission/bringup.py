from __future__ import annotations

from contextlib import contextmanager
from typing import Iterator

from carrot_guide.link import DEFAULT_SIMULATOR_URL, CommandFailed, MavlinkLink
from carrot_guide.mission.vehicle import Vehicle
from carrot_guide.state import GlobalPosition
from carrot_guide.telemetry import TelemetryTracker
from carrot_guide.utils import Deadline


def launch(
    link: MavlinkLink,
    tracker: TelemetryTracker,
    altitude_m: float,
    timeout_s: float = 120.0,
) -> None:
    """A retry loop, not a sequence: GUIDED drops back to STABILIZE until the estimator is
    happy, pre-arm refuses for the first half-minute, and an armed vehicle self-disarms in ~10 s.

    Times itself off `link.monotonic`, so a fake clock in the link fakes the whole sequence."""
    deadline = Deadline.after(timeout_s, link.monotonic)
    last_error: Exception | None = None

    while not deadline.expired:
        try:
            link.set_mode("GUIDED", tracker, timeout_s=deadline.slice(5.0))
            link.arm(tracker, timeout_s=deadline.slice(30.0))

            if tracker.mode != "GUIDED":
                continue

            # The climb wait defaults to a minute; clamped, or the retry runs past timeout_s.
            link.takeoff(
                altitude_m,
                tracker,
                timeout_s=deadline.slice(60.0),
                accept_timeout_s=deadline.slice(6.0),
                retry_every_s=1.0,
            )
            return
        except CommandFailed as error:
            last_error = error
            # Doubles as the backoff before re-establishing the sequence from the top.
            link.drain(tracker, budget_s=1.0)

    raise CommandFailed(
        f"could not get airborne within {timeout_s:g} s; last: {last_error}"
        + (f"; vehicle says: {tracker.last_status!r}" if tracker.last_status else "")
    )


@contextmanager
def airborne(
    url: str = DEFAULT_SIMULATOR_URL,
    altitude_m: float = 20.0,
    connect_timeout_s: float = 120.0,
    land_on_exit: bool = True,
) -> Iterator[Vehicle]:
    """Lands on the way out of the block even if it raised."""
    link = MavlinkLink.connect(url, timeout_s=connect_timeout_s)
    tracker = TelemetryTracker()
    try:
        link.request_streams()
        link.wait_ready_to_arm(tracker, timeout_s=connect_timeout_s)
        launch(link, tracker, altitude_m, timeout_s=connect_timeout_s)

        # Takeoff point but *home* altitude, matching MAV_FRAME_LOCAL_NED: a target's `down`
        # is minus the altitude above home, not above wherever the climb stopped.
        here = tracker.snapshot().position
        tracker.set_origin(GlobalPosition(here.lat_deg, here.lon_deg, 0.0))
        yield Vehicle(link=link, tracker=tracker)
    finally:
        try:
            if land_on_exit:
                link.land(tracker)
        finally:
            link.close()
