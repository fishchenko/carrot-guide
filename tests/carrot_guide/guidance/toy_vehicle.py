"""A point mass that flies whatever a guidance law commands.

It cannot validate tuning against a real airframe, but it does prove the laws converge
and stay bounded, which is what breaks when the geometry or a sign is wrong. It sits
here rather than in one of the test files because the station-keeping laws and the
intercept laws are all integrated this way, and two copies of a toy vehicle is two
different toy vehicles.
"""

from __future__ import annotations

from carrot_guide.guidance import Target
from carrot_guide.state import NED

AT_REST = NED(0.0, 0.0, 0.0)

# The crossing target both intercept laws are measured against, and the geometry
# `make intercept` flies: 72 m out on the bow, running east at 3 m/s.
CROSSING = Target(start=NED(60.0, -40.0, 0.0), velocity=NED(0.0, 3.0))


def simulate(law, start: NED, steps: int, dt: float = 0.1, response_s: float = 0.0) -> list[NED]:
    """Integrate a law over the toy vehicle, optionally one that takes time to obey.

    With `response_s` at zero the vehicle takes up the commanded velocity instantly,
    which is all the station-keeping laws need. The intercept laws are measured against
    a first-order lag instead, because `ProNav` works by asking for a heading offset and
    letting the lag turn it into a turn *rate* — on a vehicle that snaps to its command
    the effective navigation constant would be set by the step size of this loop rather
    than by anything in the law.
    """
    position = start
    velocity = AT_REST
    track = [position]
    for step in range(steps):
        command = law.command(position, velocity, step * dt).velocity
        if response_s > 0.0:
            velocity = velocity + (command - velocity).scaled(min(1.0, dt / response_s))
        else:
            velocity = command
        position = position + velocity.scaled(dt)
        track.append(position)
    return track


def closest_approach(law, start: NED, steps: int, dt: float = 0.1, response_s: float = 0.22):
    """Miss distance and the time of it, the two numbers an intercept run reports."""
    track = simulate(law, start, steps, dt, response_s)
    ranges = [(law.tracking_error(p, i * dt), i * dt) for i, p in enumerate(track)]
    return min(ranges)
