"""Parsing tests driven by hand-built MAVLink messages.

The `Message` stub lives in conftest; see it for why the tracker never needs more.
"""

import pytest

from carrot_guide.telemetry import (
    ARMED_FLAG,
    TelemetryError,
    TelemetryTracker,
    mode_name,
)

from conftest import Message


def global_position(**overrides: float) -> Message:
    fields: dict[str, float] = {
        "lat": int(50.4501 * 1e7),
        "lon": int(30.5234 * 1e7),
        "relative_alt": 20_000,
        "vx": 150,
        "vy": -50,
        "vz": 25,
        "hdg": 9_000,
        "time_boot_ms": 12_345,
    }
    fields.update(overrides)
    return Message("GLOBAL_POSITION_INT", **fields)


def test_position_is_scaled_out_of_mavlink_integers():
    tracker = TelemetryTracker()
    tracker.handle(global_position())
    state = tracker.snapshot()
    assert state.position.lat_deg == pytest.approx(50.4501)
    assert state.position.lon_deg == pytest.approx(30.5234)
    assert state.position.alt_m == pytest.approx(20.0)
    assert state.timestamp_s == pytest.approx(12.345)


def test_velocity_is_scaled_from_centimetres_per_second():
    tracker = TelemetryTracker()
    tracker.handle(global_position())
    velocity = tracker.snapshot().velocity
    assert velocity.north == pytest.approx(1.5)
    assert velocity.east == pytest.approx(-0.5)
    assert velocity.down == pytest.approx(0.25)


def test_heading_is_scaled_and_wrapped():
    tracker = TelemetryTracker()
    tracker.handle(global_position(hdg=35_999))
    assert tracker.snapshot().heading_deg == pytest.approx(359.99)


def test_an_unknown_heading_keeps_the_last_good_one():
    tracker = TelemetryTracker()
    tracker.handle(global_position(hdg=9_000))
    tracker.handle(global_position(hdg=65_535))
    assert tracker.snapshot().heading_deg == pytest.approx(90.0)


def test_heartbeat_carries_the_arm_flag_and_the_mode():
    tracker = TelemetryTracker()
    tracker.handle(Message("HEARTBEAT", base_mode=ARMED_FLAG | 0b1, custom_mode=4))
    assert tracker.armed is True
    assert tracker.mode == "GUIDED"

    tracker.handle(Message("HEARTBEAT", base_mode=0b1, custom_mode=9))
    assert tracker.armed is False
    assert tracker.mode == "LAND"


def test_an_unmapped_mode_is_reported_rather_than_swallowed():
    assert mode_name(27) == "MODE_27"


def test_battery_of_minus_one_means_unknown_not_empty():
    tracker = TelemetryTracker()
    tracker.handle(Message("SYS_STATUS", battery_remaining=-1))
    assert tracker.battery_pct is None
    tracker.handle(Message("SYS_STATUS", battery_remaining=87))
    assert tracker.battery_pct == pytest.approx(87.0)


def test_unknown_messages_are_ignored():
    tracker = TelemetryTracker()
    tracker.handle(Message("VIBRATION", vibration_x=1.0))
    assert not tracker.has_position


def test_a_snapshot_before_the_first_position_is_an_error_not_a_zero():
    tracker = TelemetryTracker()
    tracker.handle(Message("HEARTBEAT", base_mode=0, custom_mode=0))
    with pytest.raises(TelemetryError):
        tracker.snapshot()


def test_the_frame_cannot_be_anchored_before_a_position_arrives():
    with pytest.raises(TelemetryError):
        TelemetryTracker().set_origin()


def test_the_frame_anchors_on_the_current_position_by_default():
    tracker = TelemetryTracker()
    tracker.handle(global_position())
    origin = tracker.set_origin()
    assert origin == tracker.snapshot().position
