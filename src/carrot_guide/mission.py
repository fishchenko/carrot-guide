"""Getting the vehicle airborne, and the experiments flown once it is.

The guidance laws are the interesting part; this module is the boring, failure-prone
part around them — mode changes, arming, climb — kept in one place so a failure has
one obvious owner.
"""

from __future__ import annotations

import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Iterator

from carrot_guide.guidance import HoldPoint
from carrot_guide.link import DEFAULT_SIMULATOR_URL, CommandFailed, MavlinkLink
from carrot_guide.state import NED, GlobalPosition, to_local_ned
from carrot_guide.telemetry import TelemetryError, TelemetryTracker
from carrot_guide.utils import Deadline


@dataclass
class Vehicle:
    """A connected vehicle plus the local frame anchored at its takeoff point."""

    link: MavlinkLink
    tracker: TelemetryTracker

    @property
    def origin(self) -> GlobalPosition:
        """Where the local frame is anchored.

        Read through to the tracker rather than kept alongside it: the runner projects
        positions against `tracker.origin`, and a second copy here would be a second
        answer to the same question the moment anything re-anchored the frame.
        """
        if self.tracker.origin is None:
            raise TelemetryError("local frame is not anchored; call tracker.set_origin() first")
        return self.tracker.origin

    @property
    def position_ned(self) -> NED:
        return to_local_ned(self.tracker.snapshot().position, self.origin)


def launch(
    link: MavlinkLink,
    tracker: TelemetryTracker,
    altitude_m: float,
    timeout_s: float = 120.0,
) -> None:
    """Get from cold to hovering, re-establishing each step instead of assuming it.

    Written as a loop because none of the three steps stays done on its own while a
    cold autopilot is still settling:

    - GUIDED is accepted and then quietly dropped back to STABILIZE if the estimator
      is not yet happy, and takeoff is refused in any other mode;
    - pre-arm checks refuse for the first half-minute, with complaints like
      "Gyros inconsistent" that clear themselves;
    - once armed, the vehicle disarms itself after about ten seconds if nothing has
      taken off, so a long retry on takeoff invalidates the arming that preceded it.

    A straight sequence — set mode, arm, take off — passes against a simulator that
    has been up a while and fails against a fresh one. This checks the state it needs
    immediately before it needs it.
    """
    deadline = Deadline.after(timeout_s)
    last_error: Exception | None = None

    while not deadline.expired:
        try:
            link.set_mode("GUIDED", tracker, timeout_s=deadline.slice(5.0))
            link.arm(tracker, timeout_s=deadline.slice(30.0))

            if tracker.mode != "GUIDED":
                # It fell back while we were arming; start over rather than command a
                # takeoff the vehicle will refuse.
                continue

            # Short attempts on purpose: if takeoff is refused, the fix is to
            # re-establish mode and arming, not to keep asking in the same state.
            #
            # The climb wait is clamped too. It defaults to a minute, and left
            # unclamped it is the one step that can carry the whole retry loop well
            # past the deadline the caller set — a `launch(timeout_s=…)` that reports
            # giving up a minute after the time it was given is not a timeout.
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
            # Whatever refused, the answer is the same: let the vehicle get on with
            # settling, then re-establish the whole sequence from the top.
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
    """Connect, take off, hand back a flying vehicle, and land it afterwards.

    Landing runs on the way out of the block whatever happened inside it: a test that
    fails mid-flight should still leave the simulator in a state the next test can use.
    """
    link = MavlinkLink.connect(url, timeout_s=connect_timeout_s)
    tracker = TelemetryTracker()
    try:
        link.request_streams()
        link.wait_ready_to_arm(tracker, timeout_s=connect_timeout_s)
        launch(link, tracker, altitude_m, timeout_s=connect_timeout_s)

        # Anchor the local frame at the takeoff point but at *home* altitude, matching
        # MAV_FRAME_LOCAL_NED. That way a target's `down` reads as minus the altitude
        # above home, instead of being measured from wherever the climb happened to
        # stop — which silently doubles the requested altitude.
        here = tracker.snapshot().position
        tracker.set_origin(GlobalPosition(here.lat_deg, here.lon_deg, 0.0))
        yield Vehicle(link=link, tracker=tracker)
    finally:
        try:
            if land_on_exit:
                link.land(tracker)
        finally:
            link.close()


def measure_command_latency(
    vehicle: Vehicle,
    trials: int = 5,
    step_speed_mps: float = 3.0,
    reaction_fraction: float = 0.1,
    timeout_s: float = 5.0,
) -> list[float]:
    """Time from sending a velocity target to seeing the vehicle respond.

    Deliberately end-to-end: it contains the link, the autopilot's own loop, the
    vehicle's inertia and the telemetry rate. That makes it a useful upper bound on
    how stale the loop's picture of the world can be — not a measure of link latency
    alone, and the README says so.
    """
    link, tracker = vehicle.link, vehicle.tracker
    threshold = step_speed_mps * reaction_fraction
    latencies: list[float] = []

    for trial in range(trials):
        _settle_at_rest(vehicle)
        # Alternate the axis so a steady wind cannot flatter or spoil every trial.
        command = NED(step_speed_mps, 0.0) if trial % 2 == 0 else NED(0.0, step_speed_mps)

        sent_at = time.monotonic()
        deadline = Deadline(sent_at + timeout_s)
        while not deadline.expired:
            link.send_velocity(command)
            link.drain(tracker, budget_s=0.02)
            if tracker.velocity.horizontal_norm >= threshold:
                latencies.append(time.monotonic() - sent_at)
                break
        else:
            raise TimeoutError(f"no reaction to a {step_speed_mps:g} m/s step within {timeout_s:g} s")

    _settle_at_rest(vehicle)
    return latencies


def _settle_at_rest(vehicle: Vehicle, tolerance_mps: float = 0.2, timeout_s: float = 20.0) -> None:
    """Hold the current point until the vehicle is genuinely still."""
    link, tracker = vehicle.link, vehicle.tracker
    link.drain(tracker, budget_s=0.1)
    hold = HoldPoint(target=vehicle.position_ned)

    deadline = Deadline.after(timeout_s)
    while not deadline.expired:
        link.drain(tracker, budget_s=0.05)
        state = tracker.snapshot()
        command = hold.command(vehicle.position_ned, state.velocity)
        link.send_velocity(command.velocity)
        if state.velocity.horizontal_norm <= tolerance_mps:
            return
    raise TimeoutError("vehicle would not settle at rest")
