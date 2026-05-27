"""
Core data types for petctl.

These are the contracts that flow between backends, control schemes,
and visualizers. Everything is kept in plain dataclasses so there are
no circular imports and types remain trivially serializable.
"""

from __future__ import annotations

import time
import math
from dataclasses import asdict, dataclass, field, fields
from typing import TYPE_CHECKING, Optional

from petctl.config import BATTERY_CONFIG, MOTOR_LIMITS

if TYPE_CHECKING:
    from petctl.perception.stroke import CradleReading, HoldReading, StrokeReading
    from petctl.perception.contact import ContactReading


@dataclass
class PowerTelemetry:
    """Snapshot of power management state for a single control tick."""

    voltage_raw_v: float = 0.0
    voltage_ema_v: Optional[float] = None   # heavy EMA — for display only, not safety
    voltage_state: str = "NORMAL"
    current_amps_raw: float = 0.0
    current_amps_filtered: float = 0.0
    current_drive_scale: float = 1.0        # safety backstop scale (1.0 = full drive)
    current_target_scale: float = 1.0       # budget-targeting scale (>1.0 = boost, <1.0 = cut)
    system_state: str = "RUNNING"
    # Per-motor state (keyed by servo_id)
    motor_states: dict[int, str] = field(default_factory=dict)
    motor_disable_reasons: dict[int, str] = field(default_factory=dict)
    motor_compliance_scales: dict[int, float] = field(default_factory=dict)
    # State transition events since last tick
    events: list[str] = field(default_factory=list)


@dataclass
class GestureFrame:
    """Per-tick gesture classification, populated by the controller.

    At most one of cradle, stroke, hold is non-None when contact is detected.
    cradle takes highest priority. contact is always non-None when hold is
    non-None (it wraps the hold reading with a sub-type).
    """

    cradle: CradleReading | None = None
    stroke: StrokeReading | None = None
    hold: HoldReading | None = None
    contact: ContactReading | None = None


@dataclass
class GestureEvent:
    """Lifecycle-aware gesture event, emitted on type transitions.

    Passed to async consumers (e.g. an LLM valence driver) via
    Controller.touch_events.  to_dict() is the primary LLM interface
    (JSON function calling); describe() produces a natural-language phrase
    for prompt injection.

    Fields are the common envelope; type-specific fields are None when not
    applicable to that touch type.
    """

    touch_type: str            # cradle | stroke | hold | squeeze | restrict | budge | twist | wrench | touch | none
    timestamp: float
    duration: float            # seconds since contact onset; 0.0 for instantaneous / stroke start
    intensity: float           # 0.0–1.0 mean pad activation
    centroid: float | None     # body-axis position (0.0 = head, 7.0 = tail)
    side: str                  # active face(s): "top" | "left" | "right" | combinations
    modules: list[int]         # active module IDs in body order
    velocity: float | None     # stroke only: signed body-units/s (+ = head→tail)
    direction: str | None      # stroke only: "head_to_tail" | "tail_to_head"
    confidence: float | None   # stroke only: R² of linear centroid fit
    pressure_peak: float | None  # squeeze: peak FSR value (0–1)
    torque_peak: float | None    # restrict/wrench: peak torque in Nm
    affected_servos: list[int] = field(default_factory=list)  # restrict/wrench/twist
    status: str = "complete"  # "started" | "running" | "complete" | "promoted"
    promoted_from: str | None = None  # set when status=="promoted": the previous touch_type

    @classmethod
    def from_gesture_frame(
        cls,
        event: GestureFrame,
        timestamp: float,
        session_duration: float = 0.0,
        status: str = "complete",
    ) -> GestureEvent:
        """Build a GestureEvent from a per-tick GestureFrame."""
        cradle = event.cradle
        stroke = event.stroke
        hold = event.hold
        contact = event.contact

        if cradle is not None:
            if contact is not None:
                # restrict-during-cradle: use contact type/motor data, cradle geometry
                return cls(
                    touch_type=contact.contact_type.value,
                    timestamp=timestamp,
                    duration=cradle.duration,
                    intensity=cradle.intensity,
                    centroid=cradle.centroid,
                    side=cradle.side,
                    modules=list(cradle.modules),
                    velocity=None,
                    direction=None,
                    confidence=None,
                    pressure_peak=contact.pressure_peak or None,
                    torque_peak=contact.torque_peak or None,
                    affected_servos=list(contact.affected_servos),
                    status=status,
                )
            return cls(
                touch_type="cradle",
                timestamp=timestamp,
                duration=cradle.duration,
                intensity=cradle.intensity,
                centroid=cradle.centroid,
                side=cradle.side,
                modules=list(cradle.modules),
                velocity=None,
                direction=None,
                confidence=None,
                pressure_peak=None,
                torque_peak=None,
                status=status,
            )

        if stroke is not None:
            modules: list[int] = []
            for blob in stroke.blobs:
                for m in blob.modules:
                    if m not in modules:
                        modules.append(m)
            return cls(
                touch_type="stroke",
                timestamp=timestamp,
                duration=session_duration,
                intensity=stroke.intensity,
                centroid=stroke.centroid,
                side=stroke.side,
                modules=modules,
                velocity=stroke.velocity,
                direction=stroke.direction,
                confidence=stroke.confidence,
                pressure_peak=None,
                torque_peak=None,
                status=status,
            )

        if contact is not None:
            centroid: float | None
            if hold is not None:
                centroid = hold.centroid
                side = hold.side
                intensity = hold.intensity
                duration = hold.duration
                modules = []
                for blob in hold.q_blobs:
                    for m in blob.modules:
                        if m not in modules:
                            modules.append(m)
            else:
                centroid = contact.centroid
                side = contact.side
                intensity = 0.0
                duration = session_duration
                modules = []
            return cls(
                touch_type=contact.contact_type.value,
                timestamp=timestamp,
                duration=duration,
                intensity=intensity,
                centroid=centroid,
                side=side,
                modules=modules,
                velocity=None,
                direction=None,
                confidence=None,
                pressure_peak=contact.pressure_peak or None,
                torque_peak=contact.torque_peak or None,
                affected_servos=list(contact.affected_servos),
                status=status,
            )

        return cls(
            touch_type="none",
            timestamp=timestamp,
            duration=session_duration,
            intensity=0.0,
            centroid=None,
            side="",
            modules=[],
            velocity=None,
            direction=None,
            confidence=None,
            pressure_peak=None,
            torque_peak=None,
            status=status,
        )

    def to_dict(self) -> dict:
        """JSON-serializable dict, omitting None-valued fields."""
        result: dict = {}
        for f in fields(self):
            val = getattr(self, f.name)
            if val is not None:
                result[f.name] = val
        return result

    def describe(self, status_override: str | None = None) -> str:
        """Human-readable one-line description for LLM prompt injection."""
        if self.touch_type == "none":
            return "contact ended"
        _TAG = {"started": "[start]", "running": "[ongoing]", "complete": "[end]", "promoted": "[→]"}
        status = status_override if status_override is not None else self.status
        parts: list[str] = [f"gesture={self.touch_type}"]
        parts.insert(0, _TAG.get(status, f"[{status}]"))
        if self.direction and self.velocity is not None:
            arrow = "→" if self.direction == "head_to_tail" else "←"
            parts.append(f"{arrow} {abs(self.velocity):.1f} mod/s")
        if self.centroid is not None:
            parts.append(f"centroid={self.centroid:.1f}")
        if self.side:
            parts.append(f"{self.side} face")
        if self.modules:
            lo, hi = min(self.modules), max(self.modules)
            parts.append(f"mod {lo}–{hi}" if lo != hi else f"mod {lo}")
        parts.append(f"{self.duration:.1f}s")
        if self.pressure_peak:
            parts.append(f"pressure={self.pressure_peak:.2f}")
        if self.torque_peak:
            parts.append(f"torque={self.torque_peak:.2f}Nm")
        if self.affected_servos:
            parts.append(f"servos={self.affected_servos}")
        return ", ".join(parts)


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
    # Last commanded position per servo after the slew filter (rad). Populated by
    # Controller each tick so contact classification can compute displacement from
    # commanded rather than from home.
    servo_commanded_positions: dict[int, float] = field(default_factory=dict)
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
    # Gesture classification for this tick — populated by Controller before motion.update().
    gesture: GestureFrame | None = field(default=None)
    # Module IDs currently detected on the robot
    active_modules: list[int] = field(default_factory=list)
    # Servo IDs confirmed to exist — used by motion sources to avoid sending commands
    # to non-existent servos (which cause protocol timeouts).
    active_servo_ids: set[int] = field(default_factory=set)
    connected: bool = False
    # True when the active motion source is commanding autonomous motion (set by Controller
    # each tick via motion.is_active()). Suppresses twist detection in ContactClassifier.
    is_motion_active: bool = False
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
