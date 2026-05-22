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
from typing import Optional

from petctl.config import BATTERY_CONFIG, MOTOR_LIMITS


@dataclass
class PowerTelemetry:
    """Snapshot of power management state for a single control tick."""

    voltage_raw_v: float = 0.0
    voltage_ema_v: Optional[float] = None   # heavy EMA — for display only, not safety
    voltage_state: str = "NORMAL"
    current_amps_raw: float = 0.0
    current_amps_filtered: float = 0.0
    current_drive_scale: float = 1.0        # 1.0 = full drive; <1.0 = current-limited
    system_state: str = "RUNNING"
    # Per-motor state (keyed by servo_id)
    motor_states: dict[int, str] = field(default_factory=dict)
    motor_disable_reasons: dict[int, str] = field(default_factory=dict)
    motor_compliance_scales: dict[int, float] = field(default_factory=dict)
    # State transition events since last tick
    events: list[str] = field(default_factory=list)


@dataclass
class ModuleSensors:
    """Sensor readings for a single robot module, normalized 0-1.

    Each face has individual capacitive pad readings:
      left / right: 4 pads each (rectangular faces)
      middle:       6 pads      (top triangular face)
    Each face also has one FSR (force-sensitive resistor).

    The touch_left / touch_right / touch_middle properties return the
    mean across pads on that face — convenient for behavior code that
    needs a single 0-1 activation level per face.
    """

    module_id: int
    # Per-pad capacitive readings, each pad normalized 0-1
    touch_left_pads: tuple[float, ...] = (0., 0., 0., 0.)
    touch_right_pads: tuple[float, ...] = (0., 0., 0., 0.)
    touch_middle_pads: tuple[float, ...] = (0., 0., 0., 0., 0., 0.)
    # One FSR per face, normalized 0-1
    pressure_left: float = 0.0
    pressure_right: float = 0.0
    pressure_middle: float = 0.0

    # ------------------------------------------------------------------
    # Aggregate helpers — mean across all pads on each face (0-1)
    # ------------------------------------------------------------------

    @property
    def touch_left(self) -> float:
        return sum(self.touch_left_pads) / len(self.touch_left_pads) if self.touch_left_pads else 0.0

    @property
    def touch_right(self) -> float:
        return sum(self.touch_right_pads) / len(self.touch_right_pads) if self.touch_right_pads else 0.0

    @property
    def touch_middle(self) -> float:
        return sum(self.touch_middle_pads) / len(self.touch_middle_pads) if self.touch_middle_pads else 0.0

    @property
    def touch_total(self) -> float:
        all_pads = (*self.touch_left_pads, *self.touch_right_pads, *self.touch_middle_pads)
        return sum(all_pads) / len(all_pads) if all_pads else 0.0

    @property
    def pressure_total(self) -> float:
        return self.pressure_left + self.pressure_right + self.pressure_middle

    def as_dict(self) -> dict[str, object]:
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
    motor_velocities: dict[int, float] = field(default_factory=dict)              # rad/s
    motor_torques: dict[int, float] = field(default_factory=dict)                 # Nm
    motor_temperatures: dict[int, int] = field(default_factory=dict)              # drive temp °C (byte 6, signed)
    motor_winding_temperatures: dict[int, int] = field(default_factory=dict)      # motor winding temp °C (byte 7, signed)
    motor_err_codes: dict[int, int] = field(default_factory=dict)                 # ERR nibble (upper 4 bits of byte 0)
    # Power management telemetry — set by Controller after PowerManager.update().
    power_telemetry: Optional[PowerTelemetry] = field(default=None)
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

    @property
    def battery_current_amps(self) -> float:
        """Battery discharge current in Amps (positive = discharging)."""
        cfg = BATTERY_CONFIG
        return -(self.battery_current_raw * cfg.ads_v_per_bit - cfg.zero_v) / cfg.sensitivity_v_per_a

    @property
    def battery_voltage_v(self) -> float:
        """Battery supply voltage in Volts, two-point calibrated for ESP32 ADC non-linearity."""
        cfg = BATTERY_CONFIG
        return (cfg.voltage_slope * self.battery_voltage_raw + cfg.voltage_offset_mv) / 1000.0

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
