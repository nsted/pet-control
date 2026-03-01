"""
Core data types for petctl.

These are the contracts that flow between backends, control schemes,
and visualizers. Everything is kept in plain dataclasses so there are
no circular imports and types remain trivially serializable.
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field

from petctl.config import SERVO_LIMITS, angle_to_raw, raw_to_angle, raw_to_radians


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

    def as_dict(self) -> dict[str, float]:
        """Return sensor values as a dict (excludes module_id)."""
        d = asdict(self)
        d.pop("module_id", None)
        return d


@dataclass
class RobotState:
    """
    Complete snapshot of robot state passed to control schemes and visualizers
    on every tick.
    """

    timestamp: float = field(default_factory=time.monotonic)
    # Keyed by module_id (int). Sensor values are normalized 0-1.
    sensors: dict[int, ModuleSensors] = field(default_factory=dict)
    # Keyed by servo_id (int). Raw ticks; home = SERVO_LIMITS.position_center.
    servo_positions: dict[int, int] = field(default_factory=dict)
    # Module IDs currently detected on the robot
    active_modules: list[int] = field(default_factory=list)
    # Servo IDs confirmed to exist — used by schemes to avoid sending commands
    # to non-existent servos (which cause protocol timeouts).
    active_servo_ids: set[int] = field(default_factory=set)
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
      - position: signed raw ticks from home (center = 0)
      - speed:    continuous rotation (signed int, for wheel mode)

    Use `ServoCommand.from_angle()` for human-friendly angle control.
    """

    servo_id: int
    position: int | None = None   # raw ticks; home = SERVO_LIMITS.position_center
    speed: int | None = None      # signed, for wheel/speed mode
    acceleration: int = SERVO_LIMITS.acceleration_default

    @classmethod
    def from_angle(
        cls,
        servo_id: int,
        angle_deg: float,
        acceleration: int = SERVO_LIMITS.acceleration_default,
    ) -> "ServoCommand":
        """
        Convert degrees to a raw servo position (0° = SERVO_LIMITS.position_center = home).

        Positive angles move in the positive joint direction.
        No clamping — servos are multi-turn with no software angle limit.
        """
        return cls(servo_id=servo_id, position=angle_to_raw(angle_deg), acceleration=acceleration)

    @staticmethod
    def position_to_angle(raw: int) -> float:
        """Convert raw ticks to degrees (position_center → 0° = home)."""
        return raw_to_angle(raw)

    @staticmethod
    def position_to_radians(raw: int) -> float:
        """Convert raw ticks to radians (position_center → 0 rad = home)."""
        return raw_to_radians(raw)
