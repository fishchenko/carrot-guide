"""Parsing tests driven by hand-built MAVLink messages.

See `tests.doubles.messages` for why the tracker never needs more of one.
"""

import pytest

from carrot_guide.telemetry import MAV_MODE_FLAG_SAFETY_ARMED, TelemetryError, TelemetryTracker

from tests.doubles.messages import Message, global_position


@pytest.mark.unit
def test_position_is_scaled_out_of_mavlink_integers():
    tracker = TelemetryTracker()
    tracker.handle(global_position())
    state = tracker.snapshot()
    assert state.position.lat_deg == pytest.approx(50.4501)
    assert state.position.lon_deg == pytest.approx(30.5234)
    assert state.position.alt_m == pytest.approx(20.0)
    assert state.timestamp_s == pytest.approx(12.345)


@pytest.mark.unit
def test_velocity_is_scaled_from_centimetres_per_second():
    tracker = TelemetryTracker()
    tracker.handle(global_position())
    velocity = tracker.snapshot().velocity
    assert velocity.north == pytest.approx(1.5)
    assert velocity.east == pytest.approx(-0.5)
    assert velocity.down == pytest.approx(0.25)


@pytest.mark.unit
def test_heading_is_scaled_from_centidegrees():
    tracker = TelemetryTracker()
    tracker.handle(global_position(hdg=35_999))
    assert tracker.snapshot().heading_deg == pytest.approx(359.99)


@pytest.mark.unit
def test_a_heading_of_a_full_turn_wraps_to_zero():
    tracker = TelemetryTracker()
    tracker.handle(global_position(hdg=36_000))
    assert tracker.snapshot().heading_deg == pytest.approx(0.0)


@pytest.mark.unit
def test_an_unknown_heading_keeps_the_last_good_one():
    tracker = TelemetryTracker()
    tracker.handle(global_position(hdg=9_000))
    tracker.handle(global_position(hdg=65_535))
    assert tracker.snapshot().heading_deg == pytest.approx(90.0)


@pytest.mark.unit
def test_heartbeat_carries_the_arm_flag_and_the_mode():
    tracker = TelemetryTracker()
    tracker.handle(Message("HEARTBEAT", base_mode=MAV_MODE_FLAG_SAFETY_ARMED | 0b1, custom_mode=4))
    assert tracker.armed is True
    assert tracker.mode == "GUIDED"

    tracker.handle(Message("HEARTBEAT", base_mode=0b1, custom_mode=9))
    assert tracker.armed is False
    assert tracker.mode == "LAND"


@pytest.mark.unit
def test_battery_of_minus_one_means_unknown_not_empty():
    tracker = TelemetryTracker()
    tracker.handle(Message("SYS_STATUS", battery_remaining=-1))
    assert tracker.battery_pct is None
    tracker.handle(Message("SYS_STATUS", battery_remaining=87))
    assert tracker.battery_pct == pytest.approx(87.0)


@pytest.mark.unit
def test_unknown_messages_are_ignored():
    tracker = TelemetryTracker()
    tracker.handle(Message("VIBRATION", vibration_x=1.0))
    assert not tracker.has_position


@pytest.mark.unit
def test_a_snapshot_before_the_first_position_is_an_error_not_a_zero():
    tracker = TelemetryTracker()
    tracker.handle(Message("HEARTBEAT", base_mode=0, custom_mode=0))
    with pytest.raises(TelemetryError):
        tracker.snapshot()


@pytest.mark.unit
def test_the_frame_cannot_be_anchored_before_a_position_arrives():
    with pytest.raises(TelemetryError):
        TelemetryTracker().set_origin()


@pytest.mark.unit
def test_the_frame_anchors_on_the_current_position_by_default():
    tracker = TelemetryTracker()
    tracker.handle(global_position())
    origin = tracker.set_origin()
    assert origin == tracker.snapshot().position
