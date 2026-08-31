"""Tests for the MAVLink layer, driven by a fake connection.

The fake answers commands the way an autopilot does — with a COMMAND_ACK, and with
the state change that follows — so the retry and confirmation logic is exercised
without a socket, a simulator or a wait. The link's clock is the fake one too, so the
timeouts are the ones the tests name rather than whatever the machine was doing.
"""

import pytest
from pymavlink import mavutil

from carrot_guide.link import (
    IGNORE_ALL_BUT_VELOCITY,
    IGNORE_ALL_BUT_VELOCITY_AND_YAW,
    CommandFailed,
    MavlinkLink,
)
from carrot_guide.state import NED
from carrot_guide.telemetry import TelemetryTracker

from tests.doubles.connection import ACCEPTED, ARM_COMMAND, FAILED, FakeConnection, heartbeat
from tests.doubles.messages import Message, global_position


def build(
    arm_results: list[int] | None = None,
) -> tuple[MavlinkLink, FakeConnection, TelemetryTracker]:
    connection = FakeConnection(arm_results)
    link = MavlinkLink(
        connection,
        ack_timeout_s=0.05,
        monotonic=connection.clock.monotonic,
        sleep=connection.clock.sleep,
    )
    return link, connection, TelemetryTracker()


@pytest.mark.unit
def test_arming_confirms_the_state_change_not_just_the_ack():
    link, connection, tracker = build()
    link.arm(tracker, timeout_s=1.0, retry_every_s=0.01)
    assert tracker.armed is True
    assert [command for command, _ in connection.commands] == [ARM_COMMAND]


@pytest.mark.unit
def test_arming_retries_while_the_pre_arm_checks_still_refuse():
    # A cold autopilot refuses twice, then lets it through.
    link, connection, tracker = build([FAILED, FAILED, ACCEPTED])
    link.arm(tracker, timeout_s=5.0, retry_every_s=0.01)
    assert tracker.armed is True
    assert len(connection.commands) == 3


@pytest.mark.unit
def test_arming_gives_up_with_a_message_that_counts_the_refusals():
    link, connection, tracker = build([FAILED] * 50)
    with pytest.raises(CommandFailed, match="refused [0-9]+ times"):
        link.arm(tracker, timeout_s=0.3, retry_every_s=0.01)
    assert len(connection.commands) > 1


@pytest.mark.unit
def test_takeoff_is_retried_too_since_it_is_refused_right_after_arming():
    link, connection, tracker = build()
    connection.reject.add(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    attempts = []

    def stop_refusing_after_two(*args, **kwargs):
        attempts.append(1)
        if len(attempts) > 2:
            connection.reject.discard(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        result = original(*args, **kwargs)
        if len(attempts) > 2:
            # Queued after the ack, because searching for one message discards the
            # others — exactly as pymavlink's recv_match does.
            connection.inbox.append(global_position())
        return result

    original = connection.command_long_send
    connection.command_long_send = stop_refusing_after_two

    link.takeoff(20.0, tracker, timeout_s=1.0, accept_timeout_s=1.0, retry_every_s=0.01)
    assert len(attempts) == 3


@pytest.mark.unit
def test_a_command_that_is_refused_to_the_end_is_not_silently_swallowed():
    link, connection, tracker = build()
    connection.reject.add(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
    with pytest.raises(CommandFailed, match="refused"):
        link.takeoff(20.0, tracker, timeout_s=0.1, accept_timeout_s=0.1, retry_every_s=0.01)


@pytest.mark.unit
def test_a_command_that_is_never_answered_fails_rather_than_hangs():
    link, connection, tracker = build()
    connection.silent = True
    with pytest.raises(CommandFailed):
        link.takeoff(20.0, tracker, timeout_s=0.1, accept_timeout_s=0.2, retry_every_s=0.01)


@pytest.mark.unit
def test_the_reason_for_a_refusal_survives_the_wait_for_the_ack():
    # The autopilot explains itself in STATUSTEXT and refuses in COMMAND_ACK, and both
    # arrive in the same burst. A type-filtered read would consume the explanation while
    # hunting for the ack, leaving the error with nothing to quote.
    link, connection, tracker = build()
    connection.reject.add(ARM_COMMAND)
    original = connection.command_long_send

    def explain_then_refuse(*args, **kwargs):
        connection.inbox.append(Message("STATUSTEXT", text="PreArm: Gyros inconsistent"))
        return original(*args, **kwargs)

    connection.command_long_send = explain_then_refuse

    with pytest.raises(CommandFailed, match="Gyros inconsistent"):
        link.arm(tracker, timeout_s=0.2, retry_every_s=0.01)
    assert tracker.last_status == "PreArm: Gyros inconsistent"


@pytest.mark.unit
def test_a_parameter_read_back_does_not_swallow_the_telemetry_around_it():
    link, connection, tracker = build()
    connection.inbox.append(heartbeat(armed=True))
    connection.inbox.append(Message("PARAM_VALUE", param_id="SIM_WIND_SPD", param_value=6.0))

    assert link.set_param("SIM_WIND_SPD", 6.0, tracker=tracker) == pytest.approx(6.0)
    assert tracker.armed is True  # the heartbeat ahead of it was not thrown away


@pytest.mark.unit
def test_velocity_targets_carry_the_command_and_the_yaw():
    link, connection, _ = build()
    link.send_velocity(NED(1.0, -2.0, 0.5), yaw_deg=90.0)
    target = connection.targets[-1]
    assert target.velocity == NED(1.0, -2.0, 0.5)
    assert target.yaw_rad == pytest.approx(1.5708, abs=1e-4)  # radians on the wire
    assert target.mask == IGNORE_ALL_BUT_VELOCITY_AND_YAW


@pytest.mark.unit
def test_a_velocity_target_without_yaw_masks_the_yaw_field():
    link, connection, _ = build()
    link.send_velocity(NED(1.0, 0.0, 0.0))
    target = connection.targets[-1]
    # The mask is what makes this a masked field rather than a commanded zero.
    assert target.mask == IGNORE_ALL_BUT_VELOCITY
    assert target.yaw_rad == 0.0


@pytest.mark.unit
def test_draining_feeds_every_queued_message_to_the_tracker():
    link, connection, tracker = build()
    connection.inbox.extend([heartbeat(armed=True), Message("SYS_STATUS", battery_remaining=64)])
    assert link.drain(tracker) == 2
    assert tracker.armed is True
    assert tracker.battery_pct == pytest.approx(64.0)


@pytest.mark.unit
def test_an_unknown_mode_is_refused_before_anything_is_sent():
    link, connection, tracker = build()
    with pytest.raises(ValueError):
        link.set_mode("HOVER_MODE_THAT_DOES_NOT_EXIST", tracker)
    assert connection.commands == []
