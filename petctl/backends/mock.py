"""
MockBackend — run petctl with no robot connected.

Two primary modes:

  interactive (default)
    Servo positions are driven by whatever Motion source is active
    (keyboard, passthrough, etc.).  Sensor values come from a JSON
    file if one is given, otherwise zero.  Hot-reloads the file on
    change so you can edit sensor values while the visualizer runs.

  file
    Both sensors and servo positions are loaded from the JSON file.
    Servo commands are accepted but ignored (the file wins).

  sine
    Smooth sine-wave sensor data.  Servo positions track commands.

  noise
    Random sensor data.  Servo positions track commands.

JSON state file format (all fields optional):
{
  "modules": {
    "0": {
      "touch_left_pads":   [0.1, 0.2, 0.3, 0.4],
      "touch_right_pads":  [0.0, 0.0, 0.1, 0.0],
      "touch_middle_pads": [0.8, 0.5, 0.3, 0.2, 0.1, 0.0],
      "pressure_middle": 0.5,
      "pressure_left": 0.1,
      "pressure_right": 0.0
    },
    "1": { ... }
  },
  "servos": {
    "1": 0.0,
    "2": 0.35
  }
}
"""

from __future__ import annotations

import json
import logging
import math
import os
import random
import time
from dataclasses import dataclass
from typing import Literal, Optional

from petctl.config import LOOP_LIMITS, MOCK_DYNAMICS, MOTOR_LIMITS
from petctl.protocols import Backend
from petctl.types import ModuleSensors, RobotState, ServoCommand

logger = logging.getLogger(__name__)

_PRESSURE_FIELDS = ("pressure_middle", "pressure_left", "pressure_right")


@dataclass
class _ServoSetpoint:
    """Latest MIT-mode setpoint recorded from `send_commands()` for one servo."""

    position: float
    velocity: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    torque_ff: float = 0.0


class MockBackend(Backend):
    """
    A backend that simulates the robot locally.

    Servo positions integrate a second-order MIT-law joint model (`MOCK_DYNAMICS`
    in `config.py`) rather than teleporting to commanded targets, so velocity- and
    torque-consuming code (filters, PowerManager, contact classifiers) can be
    exercised offline. A servo with no command yet runs at zero torque (passive).

    Args:
        mode:        "interactive" | "file" | "mock-sensor-sine" | "noise"
        state_file:  Path to a JSON file (see module docstring for format).
                     In "interactive" mode the file provides sensor values;
                     servo positions come from control scheme commands.
                     In "file" mode everything comes from the file.
                     Hot-reloaded whenever the file changes on disk.
        num_modules: Number of simulated modules (used when no file given).
                     Module 0 is the head (no servo); servos are IDs 1..(num_modules-1).
        sine_hz:     Oscillation frequency for "sine" mode.
    """

    def __init__(
        self,
        mode: Literal["interactive", "file", "mock-sensor-sine", "noise"] = "interactive",
        state_file: Optional[str] = None,
        num_modules: int = 8,
        sine_hz: float = 0.2,
    ) -> None:
        self.mode = mode
        self.state_file = state_file
        self.num_modules = num_modules
        self.sine_hz = sine_hz

        # Module 0 is the head (no servo); servos are IDs 1..(num_modules-1)
        self._servo_positions: dict[int, float] = {
            i + 1: 0.0 for i in range(num_modules - 1)
        }
        self._servo_velocities: dict[int, float] = {
            i + 1: 0.0 for i in range(num_modules - 1)
        }
        self._servo_torques: dict[int, float] = {
            i + 1: 0.0 for i in range(num_modules - 1)
        }
        # Latest commanded setpoint per servo. Missing entry = no command received
        # yet = zero torque (passive/coasting), matching a motor with no MIT frame sent.
        self._servo_setpoints: dict[int, _ServoSetpoint] = {}

        # File cache
        self._file_mtime: float = 0.0
        self._file_data: dict = {}

        self._start_time = time.monotonic()
        self._last_timestamp = time.monotonic()
        self._last_sensor_ts: float = 0.0
        self._connected = False

    # ------------------------------------------------------------------
    # Backend interface
    # ------------------------------------------------------------------

    async def connect(self) -> bool:
        self._connected = True
        self._start_time = time.monotonic()
        self._last_timestamp = time.monotonic()
        self._last_sensor_ts = time.monotonic()
        if self.state_file:
            self._reload_file()
        return True

    async def disconnect(self) -> None:
        self._connected = False

    async def get_state(self) -> RobotState:
        now = time.monotonic()
        dt = now - self._last_timestamp
        self._last_timestamp = now
        elapsed = now - self._start_time

        # Reload JSON file if it changed
        if self.state_file:
            self._reload_file_if_changed()

        sensor_period = 1.0 / LOOP_LIMITS.sensor_poll_hz
        if now - self._last_sensor_ts >= sensor_period:
            self._last_sensor_ts = now

        self._step_dynamics(dt)

        sensors = self._build_sensors(elapsed)
        # In "file" mode this overrides the just-integrated positions with the
        # file's values, same as before dynamics existed.
        servo_positions = self._build_servo_positions()

        return RobotState(
            timestamp=now,
            sensor_timestamp=self._last_sensor_ts,
            sensors=sensors,
            servo_positions=servo_positions,
            active_modules=list(sensors.keys()),
            active_servo_ids=set(self._servo_positions.keys()),
            motor_velocities=dict(self._servo_velocities),
            motor_torques=dict(self._servo_torques),
            connected=self._connected,
            dt=dt,
        )

    async def send_commands(self, commands: list[ServoCommand]) -> None:
        """Record the latest MIT setpoint per servo; get_state() integrates it.

        Does not move anything directly — dynamics are advanced once per tick
        in `_step_dynamics()`. In 'file' mode the recorded setpoint is still
        used for integration, but `_build_servo_positions()` overrides the
        result with the file's values every tick, so the file wins as before.
        """
        for cmd in commands:
            if cmd.position is None:
                continue
            self._servo_setpoints[cmd.servo_id] = _ServoSetpoint(
                position=cmd.position,
                velocity=cmd.velocity if cmd.velocity is not None else 0.0,
                kp=cmd.kp,
                kd=cmd.kd,
                torque_ff=cmd.torque_ff,
            )

    def _step_dynamics(self, dt: float) -> None:
        """Integrate one MIT-law step per servo: tau -> accel -> velocity -> position.

        A servo with no recorded setpoint runs at zero torque (kp=kd=torque_ff=0),
        i.e. passive/coasting — matching a motor with no MIT command sent yet.
        """
        if dt <= 0.0:
            return
        cfg = MOCK_DYNAMICS
        for sid in self._servo_positions:
            sp = self._servo_setpoints.get(sid)
            p = self._servo_positions[sid]
            v = self._servo_velocities.get(sid, 0.0)

            if sp is not None:
                tau = sp.kp * (sp.position - p) + sp.kd * (sp.velocity - v) + sp.torque_ff
            else:
                tau = 0.0
            tau = max(MOTOR_LIMITS.torque_min, min(MOTOR_LIMITS.torque_max, tau))

            accel = (tau - cfg.viscous_friction_nm_s_per_rad * v) / cfg.inertia_kg_m2
            # Physical joint speed cap, not MOTOR_LIMITS.vel_min/max (that's the
            # MIT wire-encoding range for the v_des feedforward field — see
            # backends/robot.py's ramp filter, which keeps the two separate).
            v = max(-LOOP_LIMITS.max_speed_rad_s, min(LOOP_LIMITS.max_speed_rad_s, v + accel * dt))
            p = p + v * dt

            self._servo_positions[sid] = p
            self._servo_velocities[sid] = v
            self._servo_torques[sid] = tau

    async def write_home_offsets(self) -> None:
        """Mark the current commanded positions as home (all report as 0)."""
        self._servo_positions = {k: 0.0 for k in self._servo_positions}

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _build_sensors(self, elapsed: float) -> dict[int, ModuleSensors]:
        sensors: dict[int, ModuleSensors] = {}

        # Determine how many modules to produce
        module_ids = list(range(self.num_modules))
        if self.mode == "file" and "modules" in self._file_data:
            try:
                module_ids = [int(k) for k in self._file_data["modules"].keys()]
            except (ValueError, TypeError):
                pass

        for mod_id in module_ids:
            if self.mode == "mock-sensor-sine":
                phase = (mod_id / max(1, len(module_ids))) * 2 * math.pi
                val = (math.sin(2 * math.pi * self.sine_hz * elapsed + phase) + 1) / 2
                sensors[mod_id] = ModuleSensors(
                    module_id=mod_id,
                    touch_left_pads=tuple(val * 0.6 for _ in range(4)),
                    touch_right_pads=tuple(val * 0.3 for _ in range(4)),
                    touch_middle_pads=tuple(val for _ in range(6)),
                    pressure_middle=val * 0.8,
                    pressure_left=val * 0.4,
                    pressure_right=val * 0.2,
                )
            elif self.mode == "noise":
                sensors[mod_id] = ModuleSensors(
                    module_id=mod_id,
                    touch_left_pads=tuple(random.random() for _ in range(4)),
                    touch_right_pads=tuple(random.random() for _ in range(4)),
                    touch_middle_pads=tuple(random.random() for _ in range(6)),
                    **{f: random.random() for f in _PRESSURE_FIELDS},
                )
            else:
                # interactive or file — read from file data if available
                file_mod: dict = {}
                if "modules" in self._file_data:
                    file_mod = self._file_data["modules"].get(str(mod_id), {})
                sensors[mod_id] = ModuleSensors(
                    module_id=mod_id,
                    touch_left_pads=tuple(float(v) for v in file_mod.get("touch_left_pads", [0.0] * 4)),
                    touch_right_pads=tuple(float(v) for v in file_mod.get("touch_right_pads", [0.0] * 4)),
                    touch_middle_pads=tuple(float(v) for v in file_mod.get("touch_middle_pads", [0.0] * 6)),
                    **{f: float(file_mod.get(f, 0.0)) for f in _PRESSURE_FIELDS},
                )

        return sensors

    def _build_servo_positions(self) -> dict[int, float]:
        if self.mode == "file" and "servos" in self._file_data:
            try:
                file_servos = {
                    int(k): float(v)
                    for k, v in self._file_data["servos"].items()
                }
                # Update internal state so schemes see the file values
                self._servo_positions.update(file_servos)
            except (ValueError, TypeError):
                pass
        return dict(self._servo_positions)

    def _reload_file(self) -> None:
        """Force-load the state file."""
        if not self.state_file or not os.path.isfile(self.state_file):
            return
        try:
            with open(self.state_file) as f:
                self._file_data = json.load(f)
            self._file_mtime = os.path.getmtime(self.state_file)
        except (OSError, json.JSONDecodeError) as e:
            logger.warning("[MockBackend] Could not load %s: %s", self.state_file, e)

    def _reload_file_if_changed(self) -> None:
        if not self.state_file or not os.path.isfile(self.state_file):
            return
        try:
            mtime = os.path.getmtime(self.state_file)
            if mtime > self._file_mtime:
                self._reload_file()
        except OSError:
            pass
