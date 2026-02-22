"""
MockBackend — run petctl with no robot connected.

Two primary modes:

  interactive (default)
    Servo positions are driven by whatever ControlScheme is active
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
      "touch_middle": 0.8,
      "touch_left": 0.2,
      "touch_right": 0.0,
      "pressure_middle": 0.5,
      "pressure_left": 0.1,
      "pressure_right": 0.0
    },
    "1": { ... }
  },
  "servos": {
    "1": 2048,
    "2": 1500
  }
}
"""

from __future__ import annotations

import json
import math
import os
import random
import time
from typing import Literal, Optional

from petctl.protocols import RobotBackend
from petctl.types import ModuleSensors, RobotState, ServoCommand

_SENSOR_FIELDS = (
    "touch_middle", "touch_left", "touch_right",
    "pressure_middle", "pressure_left", "pressure_right",
)


class MockBackend(RobotBackend):
    """
    A backend that simulates the robot locally.

    Args:
        mode:        "interactive" | "file" | "sine" | "noise"
        state_file:  Path to a JSON file (see module docstring for format).
                     In "interactive" mode the file provides sensor values;
                     servo positions come from control scheme commands.
                     In "file" mode everything comes from the file.
                     Hot-reloaded whenever the file changes on disk.
        num_modules: Number of simulated modules (used when no file given).
        num_servos:  Number of simulated servos.
        sine_hz:     Oscillation frequency for "sine" mode.
    """

    def __init__(
        self,
        mode: Literal["interactive", "file", "sine", "noise"] = "interactive",
        state_file: Optional[str] = None,
        num_modules: int = 4,
        num_servos: int = 8,
        sine_hz: float = 0.2,
    ) -> None:
        self.mode = mode
        self.state_file = state_file
        self.num_modules = num_modules
        self.num_servos = num_servos
        self.sine_hz = sine_hz

        # Internal servo positions updated by send_commands()
        self._servo_positions: dict[int, int] = {
            i + 1: 2048 for i in range(num_servos)
        }

        # File cache
        self._file_mtime: float = 0.0
        self._file_data: dict = {}

        self._start_time = time.monotonic()
        self._last_timestamp = time.monotonic()
        self._connected = False

    # ------------------------------------------------------------------
    # RobotBackend interface
    # ------------------------------------------------------------------

    async def connect(self) -> bool:
        self._connected = True
        self._start_time = time.monotonic()
        self._last_timestamp = time.monotonic()
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

        sensors = self._build_sensors(elapsed)
        servo_positions = self._build_servo_positions(elapsed)

        return RobotState(
            timestamp=now,
            sensors=sensors,
            servo_positions=servo_positions,
            active_modules=list(sensors.keys()),
            active_servo_ids=set(self._servo_positions.keys()),
            connected=self._connected,
            dt=dt,
        )

    async def send_commands(self, commands: list[ServoCommand]) -> None:
        """Update internal servo state. In 'file' mode servos are still
        tracked so the scheme can read back its own commands."""
        for cmd in commands:
            if cmd.position is not None:
                self._servo_positions[cmd.servo_id] = max(0, min(4095, cmd.position))
            elif cmd.speed is not None:
                # In speed mode just nudge position proportionally
                current = self._servo_positions.get(cmd.servo_id, 2048)
                self._servo_positions[cmd.servo_id] = max(
                    0, min(4095, current + cmd.speed)
                )

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
            if self.mode == "sine":
                phase = (mod_id / max(1, len(module_ids))) * 2 * math.pi
                val = (math.sin(2 * math.pi * self.sine_hz * elapsed + phase) + 1) / 2
                sensors[mod_id] = ModuleSensors(
                    module_id=mod_id,
                    touch_middle=val,
                    touch_left=val * 0.6,
                    touch_right=val * 0.3,
                    pressure_middle=val * 0.8,
                    pressure_left=val * 0.4,
                    pressure_right=val * 0.2,
                )
            elif self.mode == "noise":
                sensors[mod_id] = ModuleSensors(
                    module_id=mod_id,
                    **{f: random.random() for f in _SENSOR_FIELDS},
                )
            else:
                # interactive or file — read from file data if available
                file_mod = {}
                if "modules" in self._file_data:
                    file_mod = self._file_data["modules"].get(str(mod_id), {})
                sensors[mod_id] = ModuleSensors(
                    module_id=mod_id,
                    **{f: float(file_mod.get(f, 0.0)) for f in _SENSOR_FIELDS},
                )

        return sensors

    def _build_servo_positions(self, elapsed: float) -> dict[int, int]:
        if self.mode == "sine":
            # Animate servos with staggered sine waves (±30° around center)
            amplitude = 4095 * 30 / 360  # 30 degrees in raw units
            for i in range(self.num_servos):
                phase = (i / max(1, self.num_servos)) * 2 * math.pi
                offset = int(amplitude * math.sin(2 * math.pi * self.sine_hz * elapsed + phase))
                self._servo_positions[i + 1] = max(0, min(4095, 2048 + offset))
        elif self.mode == "file" and "servos" in self._file_data:
            try:
                file_servos = {
                    int(k): max(0, min(4095, int(v)))
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
            print(f"[MockBackend] Could not load {self.state_file}: {e}")

    def _reload_file_if_changed(self) -> None:
        if not self.state_file or not os.path.isfile(self.state_file):
            return
        try:
            mtime = os.path.getmtime(self.state_file)
            if mtime > self._file_mtime:
                self._reload_file()
        except OSError:
            pass
