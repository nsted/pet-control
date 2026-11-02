"""Round-trip tests for petctl.wire."""

from __future__ import annotations

import dataclasses
import json

import pytest

from petctl import wire
from petctl.types import (
    GestureFrame,
    ImuReading,
    ModuleSensors,
    PowerTelemetry,
    RobotState,
    ServoCommand,
)


def _full_state() -> RobotState:
    return RobotState(
        timestamp=123.456,
        sensor_timestamp=123.4,
        sensors={
            0: ModuleSensors(
                module_id=0,
                touch_left_pads=(0.1, 0.2, 0.3, 0.4),
                touch_right_pads=(0.0, 0.0, 0.1, 0.0),
                touch_middle_pads=(0.8, 0.5, 0.3, 0.2, 0.1, 0.0),
                pressure_left=0.1, pressure_right=0.2, pressure_middle=0.3,
            ),
            3: ModuleSensors(module_id=3),
        },
        servo_positions={1: 0.5, 2: -1.25, 7: 3.0},
        motor_velocities={1: 0.1, 2: -0.2},               # deliberately a different key set
        motor_torques={2: 0.7, 7: -0.3},
        motor_temperatures={1: 41, 2: -3},
        motor_winding_temperatures={1: 44},
        motor_err_codes={2: 5},
        power_telemetry=PowerTelemetry(
            voltage_raw_v=11.9, voltage_ema_v=None, system_state="RUNNING",
            motor_states={1: "NORMAL", 2: "WARNING"},
            motor_disable_reasons={2: "temp"},
            motor_compliance_scales={1: 1.0, 2: 0.5},
            events=["a", "b"],
            per_motor_estimated_current={1: 0.3},
            active_motor_ids=[1, 2], pending_motor_ids=[7],
        ),
        battery_current_raw=1234, battery_voltage_raw=567,
        active_modules=[0, 1, 2, 3],
        active_servo_ids={1, 2, 7},
        connected=True,
        imu={7: ImuReading(module_id=7, qr=1.0, ax=0.5, gz=-2.0, timestamp=9.5)},
    )


def _hardware_facts_only(s: RobotState) -> RobotState:
    """The state as it should look after a round trip (brain-derived fields reset)."""
    return dataclasses.replace(
        s, gesture=None, is_motion_active=False, servo_commanded_positions={}, dt=0.0
    )


def test_state_round_trip() -> None:
    s = _full_state()
    out = wire.decode_state(wire.encode_state(s))
    assert out == _hardware_facts_only(s)
    assert isinstance(out.active_servo_ids, set)
    assert isinstance(out.sensors[0].touch_left_pads, tuple)
    assert all(isinstance(k, int) for k in out.power_telemetry.motor_states)  # type: ignore[union-attr]


def test_brain_derived_fields_not_on_the_wire() -> None:
    s = _full_state()
    s.gesture = GestureFrame()
    s.is_motion_active = True
    s.servo_commanded_positions = {1: 9.9}
    s.dt = 0.005
    raw = json.loads(wire.encode_state(s))
    for key in ("gesture", "is_motion_active", "servo_commanded_positions", "dt"):
        assert key not in raw
    out = wire.decode_state(wire.encode_state(s))
    assert out.gesture is None and out.is_motion_active is False
    assert out.servo_commanded_positions == {} and out.dt == 0.0


def test_empty_state_round_trip() -> None:
    s = RobotState()
    assert wire.decode_state(wire.encode_state(s)) == _hardware_facts_only(s)


def test_joints_are_jointstate_shaped() -> None:
    j = json.loads(wire.encode_state(_full_state()))["joints"]
    assert j["name"] == ["servo_1", "servo_2", "servo_7"]
    assert j["position"] == [0.5, -1.25, 3.0]
    assert j["velocity"] == [0.1, -0.2, None]
    assert j["effort"] == [None, 0.7, -0.3]


def test_state_size_is_reasonable() -> None:
    assert len(wire.encode_state(_full_state())) < 8_000


def test_commands_round_trip() -> None:
    cmds = [
        ServoCommand(servo_id=1, position=0.5, velocity=None, kp=0.8, kd=0.1, torque_ff=0.2),
        ServoCommand(servo_id=2, position=None, velocity=1.5, kp=0.0, kd=0.0, torque_ff=0.0),
    ]
    out, sent_at = wire.decode_commands(wire.encode_commands(cmds, sent_at=42.5))
    assert out == cmds
    assert sent_at == 42.5
    assert wire.decode_commands(wire.encode_commands([]))[0] == []
    assert wire.decode_commands(wire.encode_commands([]))[1] is None


@pytest.mark.parametrize(
    "name,args",
    [
        ("enable_motor", {"motor_id": 3}),
        ("disable_motor", {"motor_id": 3}),
        ("disable_torques", {}),
        ("write_home_offsets", {}),
        ("set_stagger", {"motor_id": 2, "delay_s": 0.01}),
        ("calibrate_touch", {}),
        ("set_touch_threshold", {"touch12": 5, "release12": 2}),
        ("set_touch_threshold", {"touch12": 5, "release12": 2, "touch3": 6, "release3": 3}),
        ("power_reset", {}),
    ],
)
def test_each_service_request_round_trip(name: str, args: dict) -> None:
    assert wire.decode_request(wire.encode_request(name, **args)) == (name, args)


def test_every_service_has_a_test_case() -> None:
    covered = {"enable_motor", "disable_motor", "disable_torques", "write_home_offsets",
               "set_stagger", "calibrate_touch", "set_touch_threshold", "power_reset"}
    assert set(wire.SERVICES) == covered


def test_int_delay_accepted_as_float() -> None:
    assert wire.decode_request(wire.encode_request("set_stagger", motor_id=1, delay_s=1)) == (
        "set_stagger", {"motor_id": 1, "delay_s": 1.0})


@pytest.mark.parametrize(
    "name,args",
    [
        ("nope", {}),
        ("enable_motor", {}),
        ("enable_motor", {"motor_id": "3"}),
        ("enable_motor", {"motor_id": True}),
        ("enable_motor", {"motor_id": 1, "extra": 2}),
    ],
)
def test_bad_requests_rejected(name: str, args: dict) -> None:
    with pytest.raises(ValueError):
        wire.encode_request(name, **args)


def test_decode_request_rejects_untrusted_payload() -> None:
    bad = json.dumps({"v": wire.WIRE_VERSION, "service": "enable_motor", "args": {"motor_id": "x"}})
    with pytest.raises(ValueError):
        wire.decode_request(bad)


def test_reply_round_trip_and_error() -> None:
    assert wire.decode_reply(wire.encode_reply(True)) is True
    assert wire.decode_reply(wire.encode_reply()) is None
    with pytest.raises(RuntimeError, match="boom"):
        wire.decode_reply(wire.encode_reply(error="boom"))


def test_wrong_version_rejected() -> None:
    bad = json.dumps({"v": 999, "cmds": []})
    with pytest.raises(ValueError):
        wire.decode_commands(bad)


def test_service_key() -> None:
    assert wire.service_key("power_reset") == "pet/svc/power_reset"
