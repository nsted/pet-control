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
    # Softer defaults — high kp tracks each MIT setpoint sharply (feels "poppy").
    kp_max: float = 1.5
    kd_max: float = 0.04
    kp_default: float = 0.4
    kd_default: float = 0.035


@dataclass(frozen=True)
class ControlLoopLimits:
    """Timing and rate limits for the control loop."""

    # First-order smoothing of commanded position toward the scheme (see Controller).
    # Larger tau = softer motion; 0 disables (only max_angle_step_per_tick_deg applies).
    command_smoothing_tau_s: float = 0.10
    # Cap on fraction of remaining error closed per tick (avoids one big snap after a slow loop).
    command_smoothing_max_alpha: float = 0.28

    # Hard cap on single-tick commanded delta (degrees) after smoothing (safety).
    max_angle_step_per_tick_deg: float = 4.0

    # Maximum slew rate for the backend position ramping filter (rad/s).
    # Caps how fast p_des_commanded can move toward p_des_target each cycle.
    # Acts as a final safety net after scheme-level smoothing.
    max_speed_rad_s: float = 5.0

    # Maximum commands per tick (prevents flooding the bus)
    max_commands_per_tick: int = 10

    # Seconds after the last position command before the TX loop reverts a motor
    # to zero-torque (idle) mode.  Prevents motors from holding position indefinitely
    # after a scheme goes quiet.  Should be long enough for the slew filter to settle
    # (command_smoothing_tau_s × ~5) plus a small margin.
    idle_hold_s: float = 60.0

    # Anti-windup window for the backend ramp-filter integrator (_last_mit_abs_pos).
    # During a physical occlusion the integrator is clamped to within this distance
    # of the motor's actual position, keeping holding torque = kp × anti_windup_rad.
    # 0.3 rad ≈ 17° — ~5 motor-TX ticks of max_speed budget.
    anti_windup_rad: float = 0.3

    # Background sensor poll rate (touch + FSR).
    sensor_poll_hz: float = 20.0
    sensor_poll_hz_min: float = 0.5
    sensor_poll_hz_max: float = 30.0

    # Master rate for the motor TX task (independent of sensor polling).
    # One ws.send() per motor per tick; lower values reduce Arduino WS server load directly.
    motor_update_hz: float = 50.0



@dataclass(frozen=True)
class BehaviorLimits:
    """Limits for the BehaviorEngine (when implemented).

    These are advisory for standalone Motion subclasses — the controller
    does not clamp direct motion output against these values. Only LOOP_LIMITS
    (slew rate, per-tick delta) and MOTOR_LIMITS (encoding ceiling) apply to all
    motion sources unconditionally.
    """

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


@dataclass(frozen=True)
class SensorLimits:
    """Hardware limits for onboard sensors.

    Calibrated from sensor survey (2026-05-18, modules 4 and 6, full activation).
    """

    # FSR: max raw ADC value observed under firm hand pressure across working sensors.
    # Hardware ceiling is ~1320–1330; 4095 (12-bit max) is never approached.
    fsr_max_raw: int = 1330

    # Cap touch: max nibble (0–15) observed under firm touch across all pads.
    # Hardware ceiling is nibble 11; nibbles 12–15 are never reached.
    # Normalize as: min(raw_nibble_value / cap_full_scale, 1.0)
    cap_full_scale: float = 11 / 15

    # Sliding-window size for the per-pad cap moving average (number of sensor frames).
    # At 10 Hz default poll rate: 5 frames = 500 ms of smoothing.
    cap_filter_window: int = 5


@dataclass(frozen=True)
class BatteryConfig:
    """Conversion constants for head-board battery telemetry.

    Current sensor: ACS37041KLHBLT-010B3 (±10 A, 3.3 V supply).
    ADC: ADS1015 external (head.ino) at GAIN_ONE (±4.096 V) → 2 mV/bit.
    Sensor is oriented so discharge current is in the negative direction;
    we negate so positive = discharge (battery draining).

    I_amps = -(raw * ads_v_per_bit - zero_v) / sensitivity_v_per_a

    Voltage sensor: ESP32 onboard ADC via esp_adc_cal_raw_to_voltage() —
    firmware sends calibrated millivolts, not raw ADC counts.
    100 kΩ / 10 kΩ voltage divider scales the battery voltage down to ADC range.
    ESP32 ADC non-linearity requires two-point calibration; single-ratio fit
    is inaccurate across the battery discharge range.

    Two calibration points (raw_mV → V_actual):
        1318 mV → 14.700 V  (adapter connected)
        1068 mV → 12.320 V  (on battery)

    V_battery = (voltage_slope * raw_mV + voltage_offset_mv) / 1000
    """

    sensitivity_v_per_a: float = 0.132   # 132 mV/A (ACS37041 at 3.3 V VCC)
    zero_v: float = 1.65                  # VCC/2 at zero current
    ads_v_per_bit: float = 0.002         # ADS1015 GAIN_ONE: 2 mV/LSB

    voltage_slope: float = 9.52          # mV_actual / mV_raw — two-point empirical fit
    voltage_offset_mv: float = 2152.64   # mV — two-point empirical fit


# ---------------------------------------------------------------------------
# Singleton instances — import these
# ---------------------------------------------------------------------------

MOTOR_LIMITS = MotorLimits()
LOOP_LIMITS = ControlLoopLimits()
BEHAVIOR_LIMITS = BehaviorLimits()
SENSOR_LIMITS = SensorLimits()
BATTERY_CONFIG = BatteryConfig()
