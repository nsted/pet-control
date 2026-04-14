"""
petctl.config — Hardware limits and safety constants.

Single source of truth for physical constraints. Import limits from here rather
than duplicating values in other modules.

Actuators: CubeMars GL40 II in MIT mode (CAN via WebSocket SLCAN text). Joint
positions in software are radians (`ServoCommand.position`,
`RobotState.servo_positions`); the wire format uses scaled floats packed to
16-bit fields within `MOTOR_LIMITS.pos_min`..`pos_max` (see `backends/robot.py`).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MotorLimits:
    """Hard limits for CubeMars GL40 II MIT-mode commands."""

    pos_min: float = -12.5
    pos_max: float = 12.5
    vel_min: float = -30.0
    vel_max: float = 30.0
    torque_min: float = -10.0
    torque_max: float = 10.0
    # Softer defaults — high kp tracks each MIT setpoint sharply (feels “poppy”).
    kp_default: float = 0.4
    kd_default: float = 0.035


@dataclass(frozen=True)
class ControlLoopLimits:
    """Timing and rate limits for the control loop."""

    poll_hz_min: float = 5.0
    poll_hz_max: float = 50.0
    poll_hz_default: float = 50.0

    # First-order smoothing of commanded position toward the scheme (see Controller).
    # Larger tau = softer motion; 0 disables (only max_angle_step_per_tick_deg applies).
    command_smoothing_tau_s: float = 0.10
    # Cap on fraction of remaining error closed per tick (avoids one big snap after a slow loop).
    command_smoothing_max_alpha: float = 0.28

    # Hard cap on single-tick commanded delta (degrees) after smoothing (safety).
    max_angle_step_per_tick_deg: float = 0.5

    # Maximum commands per tick (prevents flooding the bus)
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

MOTOR_LIMITS = MotorLimits()
LOOP_LIMITS = ControlLoopLimits()
BEHAVIOR_LIMITS = BehaviorLimits()
