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
    vel_min: float = -0.5
    vel_max: float = 0.5
    torque_min: float = -1.0
    torque_max: float = 1.0
    # Softer defaults — high kp tracks each MIT setpoint sharply (feels "poppy").
    kp_max: float = 1.5
    kd_max: float = 0.04
    kp_default: float = 0.8
    kd_default: float = 0.035


@dataclass(frozen=True)
class ControlLoopLimits:
    """Timing and rate limits for the control loop."""

    # First-order smoothing of commanded position toward the scheme (see Controller).
    # Larger tau = softer motion; 0 disables.
    command_smoothing_tau_s: float = 0.10
    # Cap on fraction of remaining error closed per tick (avoids one big snap after a slow loop).
    command_smoothing_max_alpha: float = 0.28

    # Maximum joint velocity (rad/s). Applied as a per-tick delta cap in both the
    # controller slew filter and the robot backend ramp filter (both use max_speed_rad_s * dt).
    # Scaled by Controller.speed_gain at the controller level; backend uses it as a hard safety net.
    # 2.0 rad/s ≈ 115°/s — lets snuggle (±40° at 0.4 Hz, peak ~100°/s) track at full amplitude.
    max_speed_rad_s: float = 6.0

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
    # At 50 Hz push rate: 5 frames = 100 ms of smoothing — enough to reject single-frame
    # spikes while keeping gesture detection responsive to brief touches.
    cap_filter_window: int = 5

    # Consecutive hardware-zero frames required to force a pad to 0.0, overriding
    # the sliding-window average.  The symmetric average can stay elevated indefinitely
    # if sporadic post-release noise (or MPR121 baseline drift) interleaves 1s with 0s.
    # 3 frames at 50 Hz = 60 ms — fast enough for responsive release, long enough to
    # survive legitimate brief inter-pad gaps during a stroke.
    cap_clear_frames: int = 3

    # ── Auto-recalibration: stuck-pad detection ───────────────────────────────
    # A module is "stuck" when any face average exceeds cap_stuck_min_face AND
    # all FSRs are below cap_stuck_max_fsr (cap active but no physical pressure).
    # After cap_stuck_timeout_s of continuous stuck state the MPR121 baselines
    # are reset via calibratetouch.  cap_stuck_recal_cooldown_s prevents hammering.

    # Face average threshold to consider a pad "stuck" (must be high — genuinely
    # stuck pads report hardware 1 continuously → filtered value near 1.0).
    cap_stuck_min_face: float = 0.6

    # FSR ceiling: below this the pad can't be a real touch (no physical pressure).
    cap_stuck_max_fsr: float = 0.05

    # Seconds a module must be continuously stuck before triggering recalibration.
    cap_stuck_timeout_s: float = 20.0

    # Minimum seconds between consecutive auto-recalibrations.
    cap_stuck_recal_cooldown_s: float = 60.0


@dataclass(frozen=True)
class BatteryConfig:
    """Conversion constants for head-board battery telemetry.

    Current sensor: ACS37041KLHBLT-010B3 (±10 A, 3.3 V supply).
    ADC: ADS1015 external (head.ino) at GAIN_ONE (±4.096 V) → 2 mV/bit.
    Sensor is oriented so discharge current is in the negative direction;
    we negate so positive = discharge (battery draining).

    I_amps = -(raw * ads_v_per_bit - zero_v) / sensitivity_v_per_a

    Voltage sensor: ESP32 onboard ADC via analogReadMilliVolts() —
    firmware sends calibrated millivolts, not raw ADC counts.
    100 kΩ / 10 kΩ voltage divider scales the battery voltage down to ADC range.
    ESP32 ADC non-linearity requires two-point calibration; single-ratio fit
    is inaccurate across the battery discharge range.

    Two calibration points (analogReadMilliVolts → V_actual):
        1328.5 mV → 14.650 V  (adapter connected, derived from 14.8V display at 14.65V actual)
        1107.9 mV → 12.120 V  (on battery, derived from 12.7V display at 12.12V actual)

    V_battery = (voltage_slope * raw_mV + voltage_offset_mv) / 1000
    """

    sensitivity_v_per_a: float = 0.132   # 132 mV/A (ACS37041 at 3.3 V VCC)
    zero_v: float = 1.65                  # VCC/2 at zero current
    ads_v_per_bit: float = 0.002         # ADS1015 GAIN_ONE: 2 mV/LSB

    voltage_slope: float = 11.47         # mV_actual / mV_raw — two-point empirical fit
    voltage_offset_mv: float = -587.9    # mV — two-point empirical fit


@dataclass(frozen=True)
class PowerBudgetConfig:
    """Predictive power budget constants for current management.

    To switch between dev and production modes, change `max_bus_current_a`:
      dev (0.5A Overcurrent):   max_bus_current_a=2.0, max_peak_current_a=3.0
      prod UPS (1.5A Overcurrent): max_bus_current_a=4.0, max_peak_current_a=5.5
      prod wall:                   max_bus_current_a=6.0, max_peak_current_a=8.0

    The UPS passthrough (TZT DC UPS) is 4A continuous / 6A hardware peak; we
    stay below the hardware peak with 5.5A. Wall supply is 15A but the slip
    ring (4×2A conductors) limits aggregate to 8A (brief transient spikes tolerated).
    """

    # Continuous budget ceiling — single value to change between dev and production
    max_bus_current_a: float = 4.0          # dev: 2.0; prod UPS: 4.0; prod wall: 6.0
    max_peak_current_a: float = 5.5         # dev: 3.0; prod UPS: 5.5; prod wall: 8.0

    # Per-motor bus current model using actual torque + velocity feedback:
    #   I ≈ base + torque_coeff × τ² × (V_nom / V_bus)   [copper-loss term]
    #         + mech_coeff × |τ × ω| / V_bus              [mechanical power term]
    #
    # torque_coeff: clamped-motor calibration 2026-06-25, VMAX=0.5/TMAX=1 (correct
    # encoding). Motor 7, kp=0 kd_max, τ_ff swept 0.02→0.12 Nm both directions,
    # other motors relaxed.  R²=0.9939, residual rms=0.023 A, n=1008, V_bus=14.70 V.
    # Previous value 0.71 was fit to 10× inflated torque readings (TMAX mismatch).
    # mech_coeff: uncalibratable from free-spin (τ² and τ×ω collinear on free rotor).
    # 0.3 is a conservative estimate; the reactive EMA backstop covers residual error.
    per_motor_base_a: float = 0.06
    per_motor_torque_coeff: float = 72.52
    per_motor_mech_coeff: float = 0.3        # calibration unreliable; reactive backstop covers residual error
    bus_voltage_nominal_v: float = 12.0
    # Worst-case power per motor (watts) used as a floor when feedback is unavailable
    # (cold start, first tick after a move begins). Yields 2.0A at 12V, ~1.6A at 14.6V.
    # This ensures slot 0 holds at most 2–3 motors under realistic operating conditions.
    per_motor_worst_case_w: float = 24.0

    # Stagger: over-budget large-delta motors are held for N TX ticks (largest first)
    stagger_interval_s: float = 0.020       # 1 TX tick at 50 Hz
    max_stagger_motors: int = 4             # cap stagger depth

    # Reactive EMA backstop (belt-and-suspenders over the predictive model)
    # Thresholds are relative to the effective budget so they scale with power source.
    reactive_backstop_factor: float = 0.85  # P term starts at 85% of budget (compensates for fast-EMA lag)
    reactive_cutoff_factor: float = 1.10    # P term = 1.0 (full cut) at budget × 1.10 — steep ramp
    reactive_ema_alpha: float = 0.033       # ~30 sample window (~1.0 s at 30 Hz); drives I term
    reactive_ema_alpha_fast: float = 0.5    # ~2 sample window (~33 ms at 30 Hz); drives P term
    reactive_recovery_multiplier: float = 1.0  # EMA drains N× faster when current is falling
    reactive_integral_ki: float = 1.5           # I gain: how fast integral builds when over budget
    reactive_integral_ki_drain_ratio: float = 0.15  # drain speed = ki × this; slow to maintain suppression between spikes
    reactive_scale_max_rate: float = 20.0       # max scale decrease per second (attack); direct ratio cap handles spikes
    reactive_scale_recovery_rate: float = 0.5  # max scale increase per second; slow to let setpoint-tracking work
    enable_per_motor_torque_cap: bool = True    # predictive cap: τ_max = sqrt((budget/n - base) / torque_coeff)

    # Power source auto-detection via bus voltage
    # Wall supply ≈14.7V, 3S LiPo max ≈12.6V — clearly distinguishable.
    # Detection uses the voltage EMA (not raw ADC) to reject noise.
    # Hysteresis: enter wall at wall_voltage_threshold_v, exit at wall_return_threshold_v
    # so ADC wobble around the entry threshold cannot cause rapid transitions.
    wall_voltage_threshold_v: float = 14.3   # EMA must exceed this to confirm wall
    wall_return_threshold_v: float = 13.8    # EMA must drop below this to return to battery
    wall_confirm_ticks: int = 50             # ~0.25 s at 200 Hz controller rate
    wall_max_bus_current_a: float = 6.0      # dev: 2.5A; prod: 6.0A (slip ring ceiling)
    wall_max_peak_current_a: float = 8.0

    # Sustained peak cutoff: global emergency stop when measured current exceeds
    # max_peak_current_a (or wall equivalent) for this long without dropping back.
    peak_current_cutoff_s: float = 0.5

    # Low-voltage motor cutoff (3S LiPo minimum safe discharge ≈10V)
    low_voltage_cutoff_v: float = 10.0
    low_voltage_recovery_v: float = 10.5    # hysteresis to prevent flapping


# ---------------------------------------------------------------------------
# Singleton instances — import these
# ---------------------------------------------------------------------------

MOTOR_LIMITS = MotorLimits()
LOOP_LIMITS = ControlLoopLimits()
BEHAVIOR_LIMITS = BehaviorLimits()
SENSOR_LIMITS = SensorLimits()
BATTERY_CONFIG = BatteryConfig()
POWER_BUDGET = PowerBudgetConfig()

# Total number of modules: head (0) + 6 middle + tail (7).
NUM_MODULES: int = 8
