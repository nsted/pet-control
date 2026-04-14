"""
Core data types for petctl.

These are the contracts that flow between backends, control schemes,
and visualizers. Everything is kept in plain dataclasses so there are
no circular imports and types remain trivially serializable.
"""

from __future__ import annotations

import time
import math
from dataclasses import asdict, dataclass, field

from petctl.config import MOTOR_LIMITS


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
    # Keyed by servo_id (int). Position in radians; home = 0.0.
    servo_positions: dict[int, float] = field(default_factory=dict)
    # Per-motor feedback from hardware (all keyed by servo_id).
    motor_velocities: dict[int, float] = field(default_factory=dict)  # rad/s
    motor_torques: dict[int, float] = field(default_factory=dict)      # Nm
    # Head-only battery telemetry raw ADC values.
    battery_current_raw: int = 0
    battery_voltage_raw: int = 0
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

    Use `ServoCommand.from_angle()` for human-friendly angle control.
    """

    servo_id: int
    position: float | None = None
    kp: float = MOTOR_LIMITS.kp_default
    kd: float = MOTOR_LIMITS.kd_default
    torque_ff: float = 0.0

    @classmethod
    def from_angle(
        cls,
        servo_id: int,
        angle_deg: float,
        kp: float = MOTOR_LIMITS.kp_default,
        kd: float = MOTOR_LIMITS.kd_default,
    ) -> "ServoCommand":
        """
        Convert degrees to a position command in radians (0 rad = home).
        """
        return cls(
            servo_id=servo_id,
            position=math.radians(angle_deg),
            kp=kp,
            kd=kd,
        )

    @staticmethod
    def position_to_angle(raw: float) -> float:
        """Convert radians to degrees."""
        return math.degrees(raw)

    @staticmethod
    def position_to_radians(raw: float) -> float:
        """Positions are already represented in radians."""
        return raw
