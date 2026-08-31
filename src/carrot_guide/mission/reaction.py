from __future__ import annotations

import time

from carrot_guide.guidance import HoldPoint
from carrot_guide.mission.vehicle import Vehicle
from carrot_guide.state import NED
from carrot_guide.utils import Deadline


def measure_command_latency(
    vehicle: Vehicle,
    trials: int = 5,
    step_speed_mps: float = 3.0,
    reaction_fraction: float = 0.1,
    timeout_s: float = 5.0,
) -> list[float]:
    """End-to-end: link, autopilot loop, inertia and telemetry rate, not link latency alone."""
    link, tracker = vehicle.link, vehicle.tracker
    threshold = step_speed_mps * reaction_fraction
    latencies: list[float] = []

    for trial in range(trials):
        _settle_at_rest(vehicle)
        # Alternate the axis so a steady wind cannot skew every trial.
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
