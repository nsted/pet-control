"""
petctl.config — Hardware limits and safety constants.

Single source of truth for all physical constraints on the robot.
Every module that issues servo commands or reads sensor data should
import limits from here rather than defining its own constants.

Servo hardware: Feetech SCS series (HLS protocol)
  - Multi-turn position, center = 0 (home after EEPROM calibration)
  - 4096 ticks per full rotation = 360°
  - No hard position limits in hardware — angle limits enforced in software
  - Torque register: 6.5 mA per unit
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ServoLimits:
    """Hard limits for servo commands. Frozen — cannot be modified at runtime."""

    # --- Position / Rotation ---
    ticks_per_rotation: int = 4096   # ticks for one full 360° rotation
    position_center: int = 2048      # raw ticks reported at home (mid-range = encoder never wraps
                                     # for ±45° moves; must match EEPROM offset target in
                                     # write_home_offsets and the rotations in robot_assembly.json)

    # --- Speed ---
    # WritePosEx speed parameter. Higher = faster movement.
    # 0 = maximum speed (no limit) — never use this.
    speed_min: int = 10
    speed_max: int = 50
    speed_default: int = 20

    # --- Acceleration ---
    acceleration_min: int = 0
    acceleration_max: int = 254
    acceleration_default: int = 50

    # --- Torque / Current ---
    # TARGET_TORQUE register, units = 6.5 mA.
    # 100 = 650 mA, 980 = max rated torque.
    # Must be > 0 on firmware v43+ (0 = zero torque).
    torque_min: int = 10             # floor — never go below this
    torque_max: int = 100            # ceiling for normal operation
    torque_default: int = 100        # ~650 mA, gentle
    torque_absolute_max: int = 980   # hardware max — only for calibration/testing

    # Protection current (EPROM register 0x1C), units = 6.5 mA.
    protection_current_max: int = 500  # ~3250 mA

    # --- Temperature ---
    temp_warning_c: float = 55.0
    temp_shutdown_c: float = 65.0


@dataclass(frozen=True)
class ControlLoopLimits:
    """Timing and rate limits for the control loop."""

    poll_hz_min: float = 5.0
    poll_hz_max: float = 50.0
    poll_hz_default: float = 20.0

    # Maximum angle change per tick (degrees).
    # Prevents a misbehaving scheme from commanding a full-range jump in one frame.
    max_angle_step_per_tick_deg: float = 10.0

    # Maximum commands per tick (prevents flooding the servo bus)
    max_commands_per_tick: int = 10


@dataclass(frozen=True)
class BehaviorLimits:
    """Limits that behaviors must respect."""

    # Maximum angle contribution from any single behavior (degrees)
    max_behavior_angle_deg: float = 45.0

    # Maximum blended angle after summing all behaviors (degrees).
    # This is the final clamp before conversion to servo commands.
    max_blended_angle_deg: float = 60.0

    # Behavior weight range
    weight_min: float = 0.0
    weight_max: float = 1.0

    # Intensity and speed parameter range
    intensity_min: float = 0.0
    intensity_max: float = 1.0
    speed_min: float = 0.0
    speed_max: float = 1.0


# ---------------------------------------------------------------------------
# Singleton instances — import these
# ---------------------------------------------------------------------------

SERVO_LIMITS = ServoLimits()
LOOP_LIMITS = ControlLoopLimits()
BEHAVIOR_LIMITS = BehaviorLimits()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def angle_to_raw(angle_deg: float) -> int:
    """Convert degrees to raw servo ticks (0° → position_center = home)."""
    return SERVO_LIMITS.position_center + int(angle_deg / 360.0 * SERVO_LIMITS.ticks_per_rotation)


def raw_to_angle(raw: int) -> float:
    """Convert raw servo ticks to degrees (position_center → 0° = home)."""
    return (raw - SERVO_LIMITS.position_center) / SERVO_LIMITS.ticks_per_rotation * 360.0


def raw_to_radians(raw: int) -> float:
    """Convert raw servo ticks to radians (position_center → 0 rad = home)."""
    import math
    return (raw - SERVO_LIMITS.position_center) / SERVO_LIMITS.ticks_per_rotation * 2.0 * math.pi


def clamp_torque(torque: int) -> int:
    """Clamp torque to safe operating range."""
    return max(SERVO_LIMITS.torque_min, min(SERVO_LIMITS.torque_max, torque))


def clamp_speed(speed: int) -> int:
    """Clamp speed to safe range."""
    return max(SERVO_LIMITS.speed_min, min(SERVO_LIMITS.speed_max, speed))
