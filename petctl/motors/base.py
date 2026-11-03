"""
petctl.motors.base — MotorProfile ABC (SHAPE_LAYER Stage 1.7).

Everything specific to one motor/driver combination — MIT-mode wire encoding,
control gains, thermal limits, and the electrical power model — lives behind
one `MotorProfile`. A motor swap means writing a new `MotorProfile` subclass
and pointing the active configuration at it; nothing above the joint servo
(backends, control schemes, the future behavior layer) should need to change.

Encoding is behavior, not just data: a different driver may pack an entirely
different frame layout, not merely different numeric bounds, so it is exposed
as methods. Gains, thermal limits, and the power model are plain data, so
they are frozen dataclasses callers can read without a method call.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import ClassVar, Optional


@dataclass(frozen=True)
class MotorEncodingRanges:
    """MIT-mode wire encoding ranges — NOT physical limits.

    These are the min/max a driver's CAN packing scales floats against for
    each field. A geared or differently-driven motor can use an entirely
    different span here with no change to what the joint can physically do.
    Physical joint speed is `ControlLoopLimits.max_speed_rad_s`
    (`petctl/config.py`), not this `vel_min`/`vel_max` (that pair bounds only
    the MIT `v_des` feedforward field on the wire).
    """

    pos_min: float
    pos_max: float
    vel_min: float
    vel_max: float
    torque_min: float
    torque_max: float
    kp_wire_max: float
    kd_wire_max: float


@dataclass(frozen=True)
class MotorGains:
    """Control gains for this motor.

    Once the behavior layer lands (SHAPE_LAYER Stage 2), `JointIntent.
    stiffness`/`damping` stay normalized 0-1 scales; these are what 1.0
    resolves to in Nm/rad for this motor, chosen so felt stiffness stays
    comparable across a motor swap.
    """

    kp_default: float
    kd_default: float
    kp_max: float
    kd_max: float


@dataclass(frozen=True)
class MotorThermalLimits:
    """Per-motor thermal protection thresholds (`petctl.power_manager`)."""

    temp_soft_warning_c: float
    temp_hard_cutoff_c: float
    temp_global_emergency_c: float
    temp_hysteresis_recovery_c: float
    temp_hysteresis_cooldown_s: float


@dataclass(frozen=True)
class MotorPowerModel:
    """Per-motor electrical power model (`petctl.power_manager`) and the mock
    joint-dynamics constants used by `MockBackend` (SHAPE_LAYER Stage 1.3)."""

    per_motor_base_a: float
    per_motor_torque_coeff: float
    per_motor_mech_coeff: float
    per_motor_worst_case_w: float
    mock_inertia_kg_m2: float
    mock_viscous_friction_nm_s_per_rad: float


@dataclass(frozen=True)
class MotorFeedback:
    """One decoded CAN feedback frame."""

    motor_id: int
    pos: float
    vel: float
    torque: float
    drive_temp: int
    motor_temp: int
    err_code: int


class MotorProfile(ABC):
    """Everything about one motor/driver combination."""

    name: ClassVar[str]
    encoding: ClassVar[MotorEncodingRanges]
    gains: ClassVar[MotorGains]
    thermal: ClassVar[MotorThermalLimits]
    power: ClassVar[MotorPowerModel]

    @abstractmethod
    def encode_command(
        self, motor_id: int, pos: float, vel: float, kp: float, kd: float, torque: float
    ) -> str:
        """Encode one MIT-mode command frame (SLCAN text)."""
        ...

    @abstractmethod
    def encode_enable(self, motor_id: int) -> str:
        """Encode the enter-motor-mode frame."""
        ...

    @abstractmethod
    def encode_disable(self, motor_id: int) -> str:
        """Encode the exit-motor-mode frame."""
        ...

    @abstractmethod
    def encode_set_zero(self, motor_id: int) -> str:
        """Encode the write-current-position-to-EEPROM-as-zero frame."""
        ...

    def encode_zero_torque(self, motor_id: int) -> str:
        """Encode a zero-torque command (kp=kd=torque=0) — the idle/query frame."""
        return self.encode_command(motor_id, pos=0.0, vel=0.0, kp=0.0, kd=0.0, torque=0.0)

    @abstractmethod
    def decode_feedback(self, can_id: int, payload: list[int]) -> Optional[MotorFeedback]:
        """Decode one already-SLCAN-parsed (can_id, payload bytes) frame.

        Returns None if this frame isn't a motor feedback frame for this
        driver (wrong CAN ID convention, short payload, etc).
        """
        ...


def float_to_uint(value: float, bits: int, min_value: float, max_value: float) -> int:
    """Scale a float into an unsigned integer of `bits` width.

    Shared bit-packing math — generic across drivers; field widths and frame
    layout (which bits go where) are per-driver, in each `MotorProfile`.
    """
    span = max_value - min_value
    clipped = max(min_value, min(max_value, value))
    scale = (1 << bits) - 1
    return int((clipped - min_value) * scale / span)


def uint_to_float(value: int, bits: int, min_value: float, max_value: float) -> float:
    span = max_value - min_value
    scale = (1 << bits) - 1
    return min_value + float(value) * span / float(scale)


def byte_to_int8(b: int) -> int:
    """Reinterpret an unsigned byte as a signed int8 (two's complement)."""
    return b if b < 128 else b - 256
