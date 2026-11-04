"""
Wire format for the petctl driver ↔ brain split.

This is the ONLY module that serializes anything for the bus. Transports
(Zenoh, in-process tests, a future MuJoCo driver) move opaque ``bytes``;
everything here is plain JSON (UTF-8), versioned by ``WIRE_VERSION``.

Three message kinds:

  state     driver → brain   hardware facts only (RobotState minus the fields
                             the brain derives itself: gesture, is_motion_active,
                             servo_commanded_positions, dt)
  commands  brain  → driver  a list of ServoCommand
  service   brain  ↔ driver  request/reply for the non-streaming Backend calls

Joint data is shaped like ROS ``sensor_msgs/JointState``: parallel arrays
``name`` / ``position`` / ``velocity`` / ``effort`` (plus drive/winding
temperature and ERR code), with ``null`` where a motor has no value.
"""

from __future__ import annotations

import json
from dataclasses import asdict
from typing import Any, Optional

from petctl.types import ImuReading, ModuleSensors, PowerTelemetry, RobotState, ServoCommand

WIRE_VERSION = 1

# Bus keys (transports use these; defined here so both ends agree).
KEY_STATE = "pet/state"
KEY_COMMANDS = "pet/cmd"
KEY_SERVICE_PREFIX = "pet/svc/"

_JOINT_PREFIX = "servo_"


def _dumps(obj: Any) -> bytes:
    return json.dumps(obj, separators=(",", ":")).encode("utf-8")


def _loads(data: bytes | str, kind: str) -> dict[str, Any]:
    obj = json.loads(data)
    if not isinstance(obj, dict):
        raise ValueError(f"{kind}: expected a JSON object")
    if obj.get("v") != WIRE_VERSION:
        raise ValueError(f"{kind}: wire version {obj.get('v')!r} != {WIRE_VERSION}")
    return obj


# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------

def encode_state(state: RobotState) -> bytes:
    """Serialize the hardware-fact fields of a RobotState."""
    ids = sorted(set(state.servo_positions) | set(state.motor_velocities) | set(state.motor_torques)
                 | set(state.motor_temperatures) | set(state.motor_winding_temperatures)
                 | set(state.motor_err_codes))
    joints = {
        "name": [f"{_JOINT_PREFIX}{i}" for i in ids],
        "position": [state.servo_positions.get(i) for i in ids],
        "velocity": [state.motor_velocities.get(i) for i in ids],
        "effort": [state.motor_torques.get(i) for i in ids],
        "drive_temp": [state.motor_temperatures.get(i) for i in ids],
        "winding_temp": [state.motor_winding_temperatures.get(i) for i in ids],
        "err_code": [state.motor_err_codes.get(i) for i in ids],
    }
    return _dumps({
        "v": WIRE_VERSION,
        "timestamp": state.timestamp,
        "sensor_timestamp": state.sensor_timestamp,
        "connected": state.connected,
        "joints": joints,
        "sensors": {str(m): s.as_dict() for m, s in state.sensors.items()},
        "imu": {str(m): _imu_to_dict(r) for m, r in state.imu.items()},
        "battery_current_raw": state.battery_current_raw,
        "battery_voltage_raw": state.battery_voltage_raw,
        "active_modules": list(state.active_modules),
        "active_servo_ids": sorted(state.active_servo_ids),
        "power_telemetry": asdict(state.power_telemetry) if state.power_telemetry is not None else None,
    })


def decode_state(data: bytes | str) -> RobotState:
    """Rebuild a RobotState. Brain-derived fields are left at their defaults."""
    o = _loads(data, "state")
    j = o["joints"]

    def column(key: str, cast: Any) -> dict[int, Any]:
        out: dict[int, Any] = {}
        for name, val in zip(j["name"], j[key]):
            if val is not None:
                out[_joint_id(name)] = cast(val)
        return out

    pt = o["power_telemetry"]
    return RobotState(
        timestamp=o["timestamp"],
        sensor_timestamp=o["sensor_timestamp"],
        sensors={int(m): _sensors_from_dict(int(m), d) for m, d in o["sensors"].items()},
        servo_positions=column("position", float),
        motor_velocities=column("velocity", float),
        motor_torques=column("effort", float),
        motor_temperatures=column("drive_temp", int),
        motor_winding_temperatures=column("winding_temp", int),
        motor_err_codes=column("err_code", int),
        power_telemetry=_telemetry_from_dict(pt) if pt is not None else None,
        battery_current_raw=o["battery_current_raw"],
        battery_voltage_raw=o["battery_voltage_raw"],
        active_modules=list(o["active_modules"]),
        active_servo_ids=set(o["active_servo_ids"]),
        connected=o["connected"],
        imu={int(m): ImuReading(module_id=int(m), **d) for m, d in o["imu"].items()},
    )


def _joint_id(name: str) -> int:
    if not name.startswith(_JOINT_PREFIX):
        raise ValueError(f"unexpected joint name {name!r}")
    return int(name[len(_JOINT_PREFIX):])


def _imu_to_dict(r: ImuReading) -> dict[str, float]:
    d = asdict(r)
    d.pop("module_id", None)
    return d


def _sensors_from_dict(module_id: int, d: dict[str, Any]) -> ModuleSensors:
    return ModuleSensors(
        module_id=module_id,
        touch_left_pads=tuple(d["touch_left_pads"]),
        touch_right_pads=tuple(d["touch_right_pads"]),
        touch_middle_pads=tuple(d["touch_middle_pads"]),
        pressure_left=d["pressure_left"],
        pressure_right=d["pressure_right"],
        pressure_middle=d["pressure_middle"],
    )


def _telemetry_from_dict(d: dict[str, Any]) -> PowerTelemetry:
    d = dict(d)
    for key in ("motor_states", "motor_disable_reasons", "motor_compliance_scales",
                "per_motor_estimated_current", "per_motor_torque_cap_nm",
                "per_motor_sustained_current_a"):
        d[key] = {int(k): v for k, v in d.get(key, {}).items()}   # .get: older senders lack the newer keys
    return PowerTelemetry(**d)


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def encode_commands(commands: list[ServoCommand], sent_at: Optional[float] = None) -> bytes:
    """Serialize a command batch. ``sent_at`` (sender's monotonic clock) is for latency logging."""
    return _dumps({
        "v": WIRE_VERSION,
        "sent_at": sent_at,
        "cmds": [
            {
                "servo_id": c.servo_id,
                "position": c.position,
                "velocity": c.velocity,
                "kp": c.kp,
                "kd": c.kd,
                "torque_ff": c.torque_ff,
            }
            for c in commands
        ],
    })


def decode_commands(data: bytes | str) -> tuple[list[ServoCommand], Optional[float]]:
    """Return (commands, sent_at)."""
    o = _loads(data, "commands")
    cmds = [
        ServoCommand(
            servo_id=int(c["servo_id"]),
            position=c["position"],
            velocity=c["velocity"],
            kp=c["kp"],
            kd=c["kd"],
            torque_ff=c["torque_ff"],
        )
        for c in o["cmds"]
    ]
    return cmds, o.get("sent_at")


# ---------------------------------------------------------------------------
# Services
# ---------------------------------------------------------------------------

# name -> {arg: (type, required)}
SERVICES: dict[str, dict[str, tuple[type, bool]]] = {
    "enable_motor": {"motor_id": (int, True)},
    "disable_motor": {"motor_id": (int, True)},
    "disable_torques": {},
    "write_home_offsets": {},
    "set_stagger": {"motor_id": (int, True), "delay_s": (float, True)},
    "calibrate_touch": {},
    "set_touch_threshold": {
        "touch12": (int, True),
        "release12": (int, True),
        "touch3": (int, False),
        "release3": (int, False),
    },
    "power_reset": {},
}


def service_key(name: str) -> str:
    """Bus key for a service."""
    return f"{KEY_SERVICE_PREFIX}{name}"


def encode_request(name: str, **args: Any) -> bytes:
    """Serialize a service request, validating name and arguments."""
    spec = _spec(name)
    unknown = set(args) - set(spec)
    if unknown:
        raise ValueError(f"{name}: unknown argument(s) {sorted(unknown)}")
    out: dict[str, Any] = {}
    for arg, (typ, required) in spec.items():
        if arg not in args or args[arg] is None:
            if required:
                raise ValueError(f"{name}: missing argument {arg!r}")
            continue
        val = args[arg]
        if typ is float and isinstance(val, int) and not isinstance(val, bool):
            val = float(val)
        if not isinstance(val, typ) or isinstance(val, bool):
            raise ValueError(f"{name}: argument {arg!r} must be {typ.__name__}, got {type(val).__name__}")
        out[arg] = val
    return _dumps({"v": WIRE_VERSION, "service": name, "args": out})


def decode_request(data: bytes | str) -> tuple[str, dict[str, Any]]:
    """Return (service_name, args) after validating against SERVICES."""
    o = _loads(data, "request")
    name = o["service"]
    args = o.get("args", {})
    # Re-validate through the encoder's rules so a bad peer can't smuggle types.
    encode_request(name, **args)
    return name, args


def encode_reply(result: Any = None, error: Optional[str] = None) -> bytes:
    """Serialize a service reply: either ``result`` or ``error``."""
    return _dumps({"v": WIRE_VERSION, "result": result, "error": error})


def decode_reply(data: bytes | str) -> Any:
    """Return the reply's result, or raise RuntimeError carrying the remote error."""
    o = _loads(data, "reply")
    if o.get("error") is not None:
        raise RuntimeError(o["error"])
    return o.get("result")


def _spec(name: str) -> dict[str, tuple[type, bool]]:
    try:
        return SERVICES[name]
    except KeyError:
        raise ValueError(f"unknown service {name!r}") from None
