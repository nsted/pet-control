"""
petctl.config — Hardware limits and safety constants.

SINGLE SOURCE OF TRUTH for all physical constraints on the robot.
Every module that issues servo commands or reads sensor data should
import limits from here rather than defining its own constants.

These values protect the hardware from damage.  Do not override
them in behavior code, control schemes, or CLI defaults without
understanding the consequences.

Servo hardware: Feetech SCS series (HLS protocol)
  - Position range: 0–4095 (12-bit), center = 2048
  - Full rotation: 4096 ticks = 360°
  - Torque register: 6.5 mA per unit
  - Speed register: units vary by mode
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class ServoLimits:
    """Hard limits for servo commands.  Frozen — cannot be modified at runtime."""

    # --- Position ---
    # Multi-turn servos: center = 0, negative values allowed.
    # No hard position limits in hardware — limits are enforced
    # in software via angle constraints below.
    position_center: int = 0

    # --- Angle (degrees from center) ---
    # Max deflection from home in either direction.
    # 150° is close to mechanical limit on the PET modules.
    # Behaviors should use a lower soft limit (e.g. 45°).
    angle_hard_limit_deg: float = 150.0

    # Soft limit for behaviors — can be overridden per-behavior
    # but never above hard limit.
    angle_soft_limit_deg: float = 45.0

    # --- Speed ---
    # WritePosEx speed parameter.  Higher = faster movement.
    # 0 = maximum speed (no limit) — never use this.
    speed_min: int = 10
    speed_max: int = 500
    speed_default: int = 100

    # --- Acceleration ---
    acceleration_min: int = 0
    acceleration_max: int = 254
    acceleration_default: int = 50

    # --- Torque / Current ---
    # TARGET_TORQUE register, units = 6.5 mA.
    # 100 = 650 mA, 980 = max rated torque.
    # Must be > 0 on firmware v43+ (0 = zero torque).
    torque_min: int = 10            # floor — never go below this
    torque_max: int = 300           # ceiling for normal operation
    torque_default: int = 100       # ~650 mA, gentle
    torque_absolute_max: int = 980  # hardware max — only for calibration/testing

    # Protection current (EPROM register 0x1C), units = 6.5 mA.
    # Sets the electrical safety ceiling.  This is a hardware-level
    # limit that TARGET_TORQUE cannot exceed.
    protection_current_max: int = 500   # ~3250 mA

    # --- Temperature ---
    # Servo temperature shutdown threshold (°C).
    # Feetech servos have internal thermal protection, but we should
    # monitor and back off before hitting it.
    temp_warning_c: float = 55.0
    temp_shutdown_c: float = 65.0


@dataclass(frozen=True)
class ControlLoopLimits:
    """Timing and rate limits for the control loop."""

    # Control loop frequency
    poll_hz_min: float = 5.0
    poll_hz_max: float = 50.0
    poll_hz_default: float = 20.0

    # Maximum angle change per tick (degrees).
    # Prevents a misbehaving behavior from commanding a sudden
    # full-range jump in a single frame.
    max_angle_step_per_tick_deg: float = 10.0

    # Maximum commands per tick (prevents flooding the bus)
    max_commands_per_tick: int = 10


@dataclass(frozen=True)
class BehaviorLimits:
    """Limits that behaviors must respect."""

    # Maximum angle contribution from any single behavior (degrees)
    max_behavior_angle_deg: float = 45.0

    # Maximum blended angle after summing all behaviors (degrees)
    # This is the final clamp before conversion to servo commands.
    max_blended_angle_deg: float = 60.0

    # Behavior weight range
    weight_min: float = 0.0
    weight_max: float = 1.0

    # Intensity and speed parameter range (from agent/params)
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

def clamp_angle_deg(angle: float, limit: float | None = None) -> float:
    """Clamp angle to ±limit degrees (default: servo hard limit)."""
    lim = limit or SERVO_LIMITS.angle_hard_limit_deg
    return max(-lim, min(lim, angle))


def clamp_torque(torque: int) -> int:
    """Clamp torque to safe operating range."""
    return max(SERVO_LIMITS.torque_min, min(SERVO_LIMITS.torque_max, torque))


def clamp_speed(speed: int) -> int:
    """Clamp speed to safe range."""
    return max(SERVO_LIMITS.speed_min, min(SERVO_LIMITS.speed_max, speed))
