"""`Deadline` on a fake clock, for the same reason the control loop is: every timeout in
`link` and `mission` is built from it, and none of them should cost a test the
wall-clock time they allow in flight.
"""

import dataclasses

import pytest

from carrot_guide.utils import Deadline

from tests.doubles.clocks import FakeClock


@pytest.mark.unit
def test_a_deadline_counts_down_against_its_own_clock():
    clock = FakeClock()
    deadline = Deadline.after(5.0, monotonic=clock.monotonic)

    assert not deadline.expired
    clock.advance(3.0)
    assert deadline.remaining_s == pytest.approx(2.0)
    clock.advance(2.0)
    assert deadline.expired


@pytest.mark.unit
def test_a_slice_never_reaches_past_the_deadline():
    clock = FakeClock()
    deadline = Deadline.after(1.0, monotonic=clock.monotonic)

    assert deadline.slice(0.5) == pytest.approx(0.5)
    clock.advance(0.8)
    # Only 0.2 s of budget is left, so that is all a read may block for.
    assert deadline.slice(0.5) == pytest.approx(0.2)


@pytest.mark.unit
def test_a_lapsed_deadline_slices_to_zero_rather_than_to_a_negative_timeout():
    """A negative timeout means "no wait" to one socket layer and "forever" to another."""
    clock = FakeClock()
    deadline = Deadline.after(1.0, monotonic=clock.monotonic)
    clock.advance(9.0)

    assert deadline.expired
    assert deadline.slice(0.5) == 0.0


@pytest.mark.unit
def test_a_deadline_is_an_instant_not_a_timer():
    """A callee cannot re-base its caller's budget; `mission.launch` relies on it."""
    clock = FakeClock()
    deadline = Deadline.after(30.0, monotonic=clock.monotonic)

    with pytest.raises(dataclasses.FrozenInstanceError):
        deadline.at_s = clock.now + 600.0
