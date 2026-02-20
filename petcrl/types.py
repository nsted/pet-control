"""
Core data types for petcrl.

These are the contracts that flow between backends, control schemes,
and visualizers. Everything is kept in plain dataclasses so there are
no circular imports and types remain trivially serializable.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from math import pi
from typing import Dict, List, Optional


@dataclass
class ModuleSensors:
    """Sensor readings for a single robot module, normalized 0-1."""

    module_id: int
    touch_middle: float = 0.0
    touch_left: float = 0.0
    touch_right: float = 0.0
    pressure_middle: float = 0.0
    pressure_left: float = 0.0
    pressure_right: float = 0.0

    @property
    def touch_total(self) -> float:
        return self.touch_middle + self.touch_left + self.touch_right

    @property
    def pressure_total(self) -> float:
        return self.pressure_middle + self.pressure_left + self.pressure_right

    def as_dict(self) -> Dict[str, float]:
        return {
            "touch_middle": self.touch_middle,
            "touch_left": self.touch_left,
            "touch_right": self.touch_right,
            "pressure_middle": self.pressure_middle,
            "pressure_left": self.pressure_left,
            "pressure_right": self.pressure_right,
        }


@dataclass
class RobotState:
    """
    Complete snapshot of robot state passed to control schemes and visualizers
    on every tick.
    """

    timestamp: float = field(default_factory=time.monotonic)
    # Keyed by module_id (int). Sensor values are normalized 0-1.
    sensors: Dict[int, ModuleSensors] = field(default_factory=dict)
    # Keyed by servo_id (int). Raw position 0-4095.
    servo_positions: Dict[int, int] = field(default_factory=dict)
    # Module IDs currently detected on the robot
    active_modules: List[int] = field(default_factory=list)
    connected: bool = False
    # Seconds since last update — useful for time-based control schemes
    dt: float = 0.0

    @classmethod
    def empty(cls) -> "RobotState":
        """Return a disconnected, all-zero state."""
        return cls()


@dataclass
class ServoCommand:
    """
    A command to move a single servo.

    Exactly one of `position` or `speed` should be set:
      - position: move to an absolute position (0-4095)
      - speed:    continuous rotation (signed int, for wheel mode)

    Use `ServoCommand.from_angle()` for human-friendly angle control.
    """

    servo_id: int
    position: Optional[int] = None   # raw 0-4095
    speed: Optional[int] = None      # signed, for wheel/speed mode
    acceleration: int = 50

    @classmethod
    def from_angle(
        cls,
        servo_id: int,
        angle_deg: float,
        acceleration: int = 50,
    ) -> "ServoCommand":
        """
        Map degrees in [-180, 180] to raw servo position [0, 4095].

        Center (0°) = 2048. Full 360° span = 4096 ticks.
        Result is clamped to [0, 4095].
        """
        raw = int((angle_deg / 360.0) * 4096) + 2048
        raw = max(0, min(4095, raw))
        return cls(servo_id=servo_id, position=raw, acceleration=acceleration)

    @staticmethod
    def position_to_angle(raw: int) -> float:
        """Convert a raw position [0, 4095] back to degrees [-180, 180]."""
        return ((raw - 2048) / 4096.0) * 360.0

    @staticmethod
    def position_to_radians(raw: int) -> float:
        return ServoCommand.position_to_angle(raw) * pi / 180.0
