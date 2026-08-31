from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from carrot_guide.recording import Sample, SampleSink
from carrot_guide.runner.protocols import GuidanceLaw, VehicleLink
from carrot_guide.runner.scheduler import FixedRateLoop
from carrot_guide.runner.stats import LoopStats
from carrot_guide.state import NED, VehicleState, to_local_ned
from carrot_guide.telemetry import TelemetryTracker


class StaleTelemetry(RuntimeError):
    """Raised after `stale_after_s` with no position report; zero velocity is sent first."""


@dataclass
class RunReport:
    samples: list[Sample]
    loop: LoopStats


@dataclass
class GuidanceRunner:
    link: VehicleLink
    tracker: TelemetryTracker
    rate_hz: float = 10.0
    # Longer than this without a position report and the loop stops rather than dead-reckon.
    stale_after_s: float = 1.0
    # ~700 B per retained cycle, unbounded over a long run; off, the CSV sink is the only record.
    retain_samples: bool = True
    loop_factory: Callable[[float], FixedRateLoop] = FixedRateLoop.calibrated

    def fly(
        self,
        law: GuidanceLaw,
        duration_s: float,
        sink: SampleSink | None = None,
        label: str = "",
    ) -> RunReport:
        if self.tracker.origin is None:
            raise RuntimeError("local frame is not anchored; call tracker.set_origin() first")

        origin = self.tracker.origin
        loop = self.loop_factory(self.rate_hz)
        samples: list[Sample] = []

        seen_updates = self.tracker.position_updates
        fresh_at = 0.0

        for tick in loop.ticks(duration_s):
            self.link.drain(self.tracker)

            if self.tracker.position_updates != seen_updates:
                seen_updates = self.tracker.position_updates
                fresh_at = tick.elapsed_s
            elif tick.elapsed_s - fresh_at > self.stale_after_s:
                self.link.send_velocity(NED(0.0, 0.0, 0.0))
                raise StaleTelemetry(
                    f"no position report for {tick.elapsed_s - fresh_at:.2f} s; stopped commanding"
                )

            state: VehicleState = self.tracker.snapshot()
            position = to_local_ned(state.position, origin)
            command = law.command(position, state.velocity, tick.elapsed_s)
            self.link.send_velocity(command.velocity, command.yaw_deg)

            sample = Sample(
                t_s=tick.elapsed_s,
                label=label,
                lat_deg=state.position.lat_deg,
                lon_deg=state.position.lon_deg,
                north_m=position.north,
                east_m=position.east,
                down_m=position.down,
                vn=state.velocity.north,
                ve=state.velocity.east,
                vd=state.velocity.down,
                cmd_vn=command.velocity.north,
                cmd_ve=command.velocity.east,
                cmd_vd=command.velocity.down,
                error_m=law.tracking_error(position, tick.elapsed_s),
                lateness_ms=tick.lateness_s * 1000.0,
                mode=state.mode,
                armed=state.armed,
            )
            if self.retain_samples:
                samples.append(sample)
            if sink is not None:
                sink.write(sample)

        # Stop commanding: ArduPilot brakes when velocity targets stop arriving.
        self.link.send_velocity(NED(0.0, 0.0, 0.0))
        return RunReport(samples=samples, loop=loop.stats())
