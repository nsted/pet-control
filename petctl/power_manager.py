"""
petctl.power_manager — Motor thermal and voltage protection + feedback-driven power budget.

Safety-critical pure-logic module. No I/O, no async. All hardware actions are
communicated back to the Controller via drain_disable_events() / drain_voltage_cutoff(),
which then calls the backend. PowerManager only consumes RobotState and produces decisions.

Protection layers (in priority order):
  1. Thermal: per-motor temperature state machine (WARNING → DISABLED → EMERGENCY)
  2. Voltage cutoff: disables all motor torques below low_voltage_cutoff_v
  3. Feedback budget: motors are bin-packed into an active set and a pending queue
     ordered by BinPackPolicy.priority. The initial active bin is seeded from the
     front of the queue until the budget would be exceeded (using a worst-case
     per-motor current floor). Pending motors are promoted one step at a time as
     the measured bus current EMA shows headroom below budget × headroom_factor.
  4. Reactive EMA backstop: PI controller on measured bus current — scales all active
     motor commands uniformly when EMA exceeds budget. P term engages at budget,
     reaches full cut at budget × reactive_cutoff_factor. I term integrates sustained
     overage to prevent the scale from recovering prematurely.
  5. Sustained peak cutoff: global emergency stop when raw current exceeds
     max_peak_current_a continuously for peak_current_cutoff_s (0.5 s). Timer resets
     the moment current drops back below the peak limit.

ERR nibble codes from the GL40 II reply frame (upper 4 bits of byte 0):
    0 = Disable, 1 = Enable, 9 = Under-voltage, A = Over-current,
    B = MOS over-temperature, C = Motor winding over-temperature,
    D = Communication loss, E = Overload
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Optional

from petctl.config import POWER_BUDGET
from petctl.types import PowerTelemetry, RobotState, ServoCommand

logger = logging.getLogger(__name__)

# ERR codes that mean the driver itself detected overtemperature
_ERR_OVERTEMP_CODES: frozenset[int] = frozenset({0xB, 0xC})


@dataclass
class BinPackPolicy:
    """Controls which motors are prioritized in the active bin and when to promote pending motors.

    priority: explicit motor ID ordering — first IDs enter the active bin first.
              None = ascending motor ID. Default is middle-out (4,3,5,2,6,1,7).
    add_step: how many motors to promote per tick when headroom is confirmed.
    headroom_factor: promote when bus current EMA < budget × this value.
    """

    priority: list[int] | None = None
    add_step: int = 1
    headroom_factor: float = 0.8


class MotorThermalState(Enum):
    NORMAL = "NORMAL"
    WARNING = "WARNING"
    DISABLED = "DISABLED"


class VoltageState(Enum):
    NORMAL = "NORMAL"
    LOW_WARNING = "LOW_WARNING"
    CUTOFF = "CUTOFF"


class PowerSource(Enum):
    BATTERY = "battery"
    WALL = "wall"


class SystemState(Enum):
    RUNNING = "RUNNING"
    EMERGENCY_STOPPED = "EMERGENCY_STOPPED"


@dataclass(frozen=True)
class PowerThresholds:
    """Thermal protection thresholds. Tune here without touching logic."""

    # --- Thermal (per motor; applied to max(drive_temp, winding_temp)) ---
    temp_soft_warning_c: float = 55.0       # reduce Kp/Kd/τ_ff by 50%
    temp_hard_cutoff_c: float = 65.0        # exit motor mode for this motor
    temp_global_emergency_c: float = 75.0   # exit motor mode for ALL motors
    temp_hysteresis_recovery_c: float = 50.0
    temp_hysteresis_cooldown_s: float = 30.0

    # --- Voltage display ---
    voltage_low_warning_v: float = 10.8    # 3.6 V/cell — informational
    voltage_ema_alpha: float = 0.02        # ~50-sample window (~1.7 s at 30 Hz)

    # --- ADC sanity (applied before EMA) ---
    voltage_sanity_min_v: float = 5.0
    voltage_sanity_max_v: float = 40.0


@dataclass
class _MotorPowerState:
    thermal_state: MotorThermalState = MotorThermalState.NORMAL
    disable_reason: str = ""
    compliance_scale: float = 1.0
    cool_since: Optional[float] = None
    last_err_overtemp_time: float = 0.0


class PowerManager:
    """
    Motor protection and feedback-driven power budget state machine.

    Pure logic — no I/O. Feed it RobotState every tick; drain disable events
    and query compliance scales to gate motor commands.

    Power budget flow (called by Controller each tick after update()):
      1. is_motor_enabled() / get_compliance_scale() — thermal gating
      2. allocate_budget(commands, state) — returns (active_commands, pending_ids)
         active_commands: commands for motors in the current bin (may be scaled)
         pending_ids: motor IDs held back; Controller should send zero-torque to these
    """

    def __init__(
        self,
        thresholds: PowerThresholds = PowerThresholds(),
        reactive_ema_alpha: float | None = None,
        reactive_ema_alpha_fast: float | None = None,
        bin_policy: BinPackPolicy | None = None,
        reactive_integral_ki: float | None = None,
        reactive_integral_ki_drain_ratio: float | None = None,
        reactive_scale_recovery_rate: float | None = None,
        budget_override: float | None = None,
    ) -> None:
        self.thresholds = thresholds
        self._budget_override = budget_override
        # Allow override for testing (defaults come from POWER_BUDGET)
        self._reactive_ema_alpha: float = (
            reactive_ema_alpha if reactive_ema_alpha is not None else POWER_BUDGET.reactive_ema_alpha
        )
        self._reactive_ema_alpha_fast: float = (
            reactive_ema_alpha_fast if reactive_ema_alpha_fast is not None else POWER_BUDGET.reactive_ema_alpha_fast
        )
        self._reactive_integral_ki: float = (
            reactive_integral_ki if reactive_integral_ki is not None else POWER_BUDGET.reactive_integral_ki
        )
        self._reactive_integral_ki_drain_ratio: float = (
            reactive_integral_ki_drain_ratio if reactive_integral_ki_drain_ratio is not None
            else POWER_BUDGET.reactive_integral_ki_drain_ratio
        )
        self._reactive_scale_recovery_rate: float = (
            reactive_scale_recovery_rate if reactive_scale_recovery_rate is not None else POWER_BUDGET.reactive_scale_recovery_rate
        )

        self._motor_states: dict[int, _MotorPowerState] = {}
        self._system_state: SystemState = SystemState.RUNNING
        self._system_disable_reason: str = ""

        # Voltage tracking
        self._voltage_ema: Optional[float] = None
        self._voltage_state: VoltageState = VoltageState.NORMAL
        self._voltage_cutoff_active: bool = False
        self._pending_voltage_cutoff: bool = False

        # Power source detection
        self._power_source: PowerSource = PowerSource.BATTERY
        self._wall_confirm_count: int = 0  # positive = confirming wall; negative = confirming return to battery

        # Reactive PI backstop
        self._current_ema: float = 0.0       # slow EMA — drives I term
        self._current_ema_fast: float = 0.0  # fast EMA — drives P term
        self._last_current_a: float = 0.0
        self._reactive_scale: float = 1.0
        self._current_integral: float = 0.0   # I term; maps to scale reduction [0, 1]
        self._last_current_t: float = 0.0
        self._last_status_log_t: float = 0.0

        # Peak current tracking
        self._peak_current_a: float = 0.0    # highest reading seen since last drop below budget
        self._window_peak_a: float = 0.0     # max current seen in the current 1-s status window
        self._peak_overage_start: float | None = None  # monotonic time when current first exceeded peak limit

        # Budget state (populated by allocate_budget)
        self._budget_scale: float = 1.0
        self._budget_total_est: float = 0.0
        self._budget_per_motor_est: dict[int, float] = {}
        self._last_motor_torques: dict[int, float] = {}  # snapshot for status log

        # Bin-pack state: active bin + pending queue
        self._active_motor_set: set[int] = set()
        self._pending_queue: list[int] = []
        self._bin_policy: BinPackPolicy = bin_policy or BinPackPolicy(priority=[4, 3, 5, 2, 6, 1, 7])

        # Pending actions — drained once per tick by the Controller
        self._pending_disable_motor_ids: list[int] = []
        self._pending_global_emergency: bool = False
        self._pending_events: list[str] = []

    # ------------------------------------------------------------------
    # Main update (call every control tick)
    # ------------------------------------------------------------------

    def update(self, state: RobotState, now: float) -> None:
        """Evaluate all protection conditions against the latest state."""
        if self._system_state == SystemState.EMERGENCY_STOPPED:
            return

        self._update_voltage(state.battery_voltage_v)
        self._update_current(state.battery_current_amps, now)
        self._update_motors(state, now)

    # ------------------------------------------------------------------
    # Voltage — display EMA, cutoff, and power source detection
    # ------------------------------------------------------------------

    def _update_voltage(self, raw_v: float) -> None:
        t = self.thresholds
        b = POWER_BUDGET

        if raw_v < t.voltage_sanity_min_v or raw_v > t.voltage_sanity_max_v:
            return

        # Display EMA
        if self._voltage_ema is None:
            self._voltage_ema = raw_v
        else:
            self._voltage_ema = t.voltage_ema_alpha * raw_v + (1.0 - t.voltage_ema_alpha) * self._voltage_ema

        # Low-voltage cutoff (motor kill at battery floor) — uses EMA to ignore transient sags
        if self._voltage_ema < b.low_voltage_cutoff_v:
            if not self._voltage_cutoff_active:
                self._voltage_cutoff_active = True
                self._pending_voltage_cutoff = True
                self._voltage_state = VoltageState.CUTOFF
                self._log_event(f"voltage_cutoff: {self._voltage_ema:.2f}V < {b.low_voltage_cutoff_v:.1f}V — disabling all motors")
                logger.warning("[PowerManager] Voltage cutoff at %.2fV (EMA)", self._voltage_ema)
        elif self._voltage_cutoff_active and self._voltage_ema >= b.low_voltage_recovery_v:
            self._voltage_cutoff_active = False
            self._log_event(f"voltage_cutoff_recovered: {self._voltage_ema:.2f}V >= {b.low_voltage_recovery_v:.1f}V")
            logger.info("[PowerManager] Voltage recovered: %.2fV (EMA)", self._voltage_ema)
            # Fall through to update display state below

        # Power source detection (wall ≈14.7V, 3S LiPo max ≈12.6V).
        # Uses the EMA (not raw ADC) so noise cannot drive rapid transitions.
        # Hysteresis: separate entry/exit thresholds (wall_voltage_threshold_v /
        # wall_return_threshold_v) create a dead band around the entry point.
        ema_v = self._voltage_ema  # guaranteed non-None: set above
        if ema_v >= b.wall_voltage_threshold_v:
            if self._power_source == PowerSource.BATTERY:
                self._wall_confirm_count += 1
                if self._wall_confirm_count >= b.wall_confirm_ticks:
                    old_budget = b.max_bus_current_a
                    new_budget = b.wall_max_bus_current_a
                    self._power_source = PowerSource.WALL
                    self._wall_confirm_count = 0  # reset so wall→battery takes the same window
                    self._log_event(
                        f"power_source: battery→wall "
                        f"(budget {old_budget:.1f}A→{new_budget:.1f}A "
                        f"peak {b.max_peak_current_a:.1f}A→{b.wall_max_peak_current_a:.1f}A)"
                    )
                    logger.info("[PowerManager] Power source: wall supply (budget %.1fA→%.1fA)", old_budget, new_budget)
            # else: already wall — keep confirm count pinned
        elif ema_v < b.wall_return_threshold_v:
            if self._power_source == PowerSource.WALL:
                self._wall_confirm_count -= 1
                if self._wall_confirm_count <= -b.wall_confirm_ticks:
                    old_budget = b.wall_max_bus_current_a
                    new_budget = b.max_bus_current_a
                    self._power_source = PowerSource.BATTERY
                    self._wall_confirm_count = 0
                    self._log_event(
                        f"power_source: wall→battery "
                        f"(budget {old_budget:.1f}A→{new_budget:.1f}A)"
                    )
                    logger.info("[PowerManager] Power source: battery (budget %.1fA→%.1fA)", old_budget, new_budget)
            else:
                # Battery mode — reset any partial wall-confirm count
                self._wall_confirm_count = 0
        # else: EMA is in the dead band (wall_return_threshold_v ≤ ema_v < wall_voltage_threshold_v)
        # — do not change the confirm counter; hysteresis holds current source stable

        # Low warning display (below cutoff this is superseded by CUTOFF state)
        if not self._voltage_cutoff_active and self._voltage_ema is not None:
            if self._voltage_ema < t.voltage_low_warning_v:
                if self._voltage_state != VoltageState.LOW_WARNING:
                    self._log_event(f"voltage_low_warning: {self._voltage_ema:.2f}V")
                    self._voltage_state = VoltageState.LOW_WARNING
            else:
                if self._voltage_state == VoltageState.LOW_WARNING:
                    self._log_event(f"voltage_normal: {self._voltage_ema:.2f}V")
                self._voltage_state = VoltageState.NORMAL

    # ------------------------------------------------------------------
    # Reactive EMA backstop
    # ------------------------------------------------------------------

    def _update_current(self, current_a: float, now: float) -> None:
        b = POWER_BUDGET
        alpha = self._reactive_ema_alpha
        # Slow EMA: ~30-sample window; drives I term (sustained over-budget detection).
        if current_a >= self._current_ema:
            self._current_ema = alpha * current_a + (1.0 - alpha) * self._current_ema
        else:
            decay_alpha = min(1.0, alpha * b.reactive_recovery_multiplier)
            self._current_ema = decay_alpha * current_a + (1.0 - decay_alpha) * self._current_ema
        # Fast EMA: ~3-sample window; drives P term for rapid spike response.
        alpha_fast = self._reactive_ema_alpha_fast
        if current_a >= self._current_ema_fast:
            self._current_ema_fast = alpha_fast * current_a + (1.0 - alpha_fast) * self._current_ema_fast
        else:
            decay_alpha_fast = min(1.0, alpha_fast * b.reactive_recovery_multiplier)
            self._current_ema_fast = decay_alpha_fast * current_a + (1.0 - decay_alpha_fast) * self._current_ema_fast
        self._last_current_a = current_a

        dt = min(now - self._last_current_t, 0.1) if self._last_current_t > 0.0 else 0.0
        self._last_current_t = now

        # Peak tracking: log immediately when current exceeds budget and sets a new high.
        budget_now = self._effective_budget()
        self._window_peak_a = max(self._window_peak_a, current_a)
        if current_a > budget_now and current_a > self._peak_current_a:
            self._peak_current_a = current_a
            logger.warning(
                "[PowerManager] Current spike: %.2fA (budget=%.1fA, over by %.2fA)",
                current_a, budget_now, current_a - budget_now,
            )
        elif current_a <= budget_now and self._peak_current_a > 0.0:
            self._peak_current_a = 0.0  # reset peak once we're back under budget

        # Sustained peak cutoff: emergency stop if current stays above the peak limit.
        peak_limit = self._effective_peak_budget()
        if current_a > peak_limit:
            if self._peak_overage_start is None:
                self._peak_overage_start = now
                logger.warning(
                    "[PowerManager] Current exceeded peak limit: %.2fA > %.1fA — starting %.1fs cutoff timer",
                    current_a, peak_limit, POWER_BUDGET.peak_current_cutoff_s,
                )
            elif now - self._peak_overage_start >= POWER_BUDGET.peak_current_cutoff_s:
                self._trigger_global_emergency(
                    f"peak_current_sustained: {current_a:.2f}A > {peak_limit:.1f}A "
                    f"for {now - self._peak_overage_start:.2f}s"
                )
                return
        else:
            if self._peak_overage_start is not None:
                logger.info(
                    "[PowerManager] Peak current overage cleared (%.2fA <= %.1fA, lasted %.3fs)",
                    current_a, peak_limit, now - self._peak_overage_start,
                )
            self._peak_overage_start = None

        budget = self._effective_budget()
        start = budget * b.reactive_backstop_factor
        zero  = budget * b.reactive_cutoff_factor

        # P term: use max(current_a, EMA_fast) so P fires on the actual current with
        # no lag on the first spike tick. EMA_fast handles recovery (when current drops,
        # EMA_fast > current_a keeps P non-zero while the scale is still suppressed).
        # Sustained overcurrent is handled by the I term.
        v = max(current_a, self._current_ema_fast)
        if v >= zero:
            p_term = 1.0
        elif v >= start:
            p_term = (v - start) / (zero - start)
        else:
            p_term = 0.0

        # I term: asymmetric integrator — builds fast when over budget, drains slowly when under.
        # Slow drain (ki_drain_ratio << 1) keeps the scale slightly suppressed after an overage,
        # preventing the motor from rushing to catch its setpoint and re-spiking current.
        # Zero steady-state error is preserved: the integral will eventually drain fully.
        error = current_a - budget
        ki_eff = self._reactive_integral_ki if error > 0 else (
            self._reactive_integral_ki * self._reactive_integral_ki_drain_ratio
        )
        self._current_integral = max(0.0, min(1.0,
            self._current_integral + ki_eff * error * dt
        ))

        new_scale = max(0.0, 1.0 - p_term - self._current_integral)

        # Asymmetric rate limits: attack fast, recover slowly to prevent hunting.
        max_down = b.reactive_scale_max_rate * dt
        max_up = self._reactive_scale_recovery_rate * dt
        new_scale = max(new_scale, self._reactive_scale - max_down)
        new_scale = min(new_scale, self._reactive_scale + max_up)

        # Direct ratio cap: when actual current exceeds budget, bypass the rate limiter
        # and immediately snap scale to budget/current. This handles the 1-tick latency
        # gap where the rate limit alone can't respond fast enough to a sudden spike.
        if current_a > budget:
            new_scale = min(new_scale, budget / current_a)

        if new_scale != self._reactive_scale:
            self._log_event(
                f"reactive_backstop: {self._reactive_scale:.2f}→{new_scale:.2f} "
                f"(I_ema={self._current_ema:.2f}A I_ema_fast={self._current_ema_fast:.2f}A budget={budget:.1f}A)"
            )
            if self._reactive_scale == 1.0 and new_scale < 1.0:
                logger.debug(
                    "[PowerManager] Current limit reached — reactive backstop engaged "
                    "(I=%.2fA EMA=%.2fA EMA_fast=%.2fA budget=%.1fA scale→%.2f)",
                    current_a, self._current_ema, self._current_ema_fast, budget, new_scale,
                )
            elif self._reactive_scale < 1.0 and new_scale == 1.0:
                logger.debug(
                    "[PowerManager] Reactive backstop cleared (I=%.2fA EMA=%.2fA EMA_fast=%.2fA budget=%.1fA)",
                    current_a, self._current_ema, self._current_ema_fast, budget,
                )
            else:
                # Log each time scale crosses a significant threshold so depth is visible.
                for threshold in (0.9, 0.75, 0.5, 0.25):
                    if self._reactive_scale > threshold >= new_scale:
                        logger.debug(
                            "[PowerManager] Backstop deepening: scale→%.2f "
                            "(I=%.2fA EMA=%.2fA EMA_fast=%.2fA budget=%.1fA)",
                            new_scale, current_a, self._current_ema, self._current_ema_fast, budget,
                        )
                    elif self._reactive_scale < threshold <= new_scale:
                        logger.debug(
                            "[PowerManager] Backstop recovering: scale→%.2f "
                            "(I=%.2fA EMA=%.2fA EMA_fast=%.2fA budget=%.1fA)",
                            new_scale, current_a, self._current_ema, self._current_ema_fast, budget,
                        )
        self._reactive_scale = new_scale

        if now - self._last_status_log_t >= 1.0:
            self._last_status_log_t = now
            tau_str = " ".join(
                f"m{mid}:{tau:.2f}" for mid, tau in sorted(self._last_motor_torques.items())
            )
            window_peak = self._window_peak_a
            self._window_peak_a = 0.0
            log_fn = logger.debug
            log_fn(
                "[PowerManager] I=%.2fA peak=%.2fA budget=%.1fA scale=%.2f EMA=%.2fA integral=%.2f  τ[Nm]:%s",
                current_a, window_peak, budget, self._reactive_scale, self._current_ema,
                self._current_integral, tau_str or "—",
            )

    # ------------------------------------------------------------------
    # Thermal protection
    # ------------------------------------------------------------------

    def _update_motors(self, state: RobotState, now: float) -> None:
        t = self.thresholds

        for motor_id in state.active_servo_ids:
            ms = self._motor_states.setdefault(motor_id, _MotorPowerState())

            if ms.thermal_state == MotorThermalState.DISABLED:
                self._track_cooling(motor_id, ms, state, now)
                continue

            drive_temp = state.motor_temperatures.get(motor_id, 0)
            winding_temp = state.motor_winding_temperatures.get(motor_id, 0)
            err_code = state.motor_err_codes.get(motor_id, 0)
            peak_temp = max(drive_temp, winding_temp)

            if err_code in _ERR_OVERTEMP_CODES:
                ms.last_err_overtemp_time = now
                self._disable_motor(
                    motor_id, ms,
                    f"err_overtemp: ERR=0x{err_code:X} "
                    f"(drive={drive_temp}°C winding={winding_temp}°C)",
                )
                continue

            if peak_temp >= t.temp_global_emergency_c:
                self._trigger_global_emergency(
                    f"motor_{motor_id}_global_overtemp: {peak_temp}°C "
                    f">= {t.temp_global_emergency_c}°C"
                )
                return

            if peak_temp >= t.temp_hard_cutoff_c:
                self._disable_motor(
                    motor_id, ms,
                    f"thermal_cutoff: {peak_temp}°C >= {t.temp_hard_cutoff_c}°C",
                )
                continue

            if peak_temp >= t.temp_soft_warning_c:
                if ms.thermal_state != MotorThermalState.WARNING:
                    self._log_event(
                        f"motor_{motor_id}_thermal_warning: "
                        f"{peak_temp}°C >= {t.temp_soft_warning_c}°C"
                    )
                ms.thermal_state = MotorThermalState.WARNING
                ms.compliance_scale = 0.5
                ms.cool_since = None
            else:
                if ms.thermal_state == MotorThermalState.WARNING:
                    self._log_event(f"motor_{motor_id}_thermal_recovered: {peak_temp}°C")
                ms.thermal_state = MotorThermalState.NORMAL
                ms.compliance_scale = 1.0
                ms.cool_since = None

    def _track_cooling(
        self, motor_id: int, ms: _MotorPowerState, state: RobotState, now: float
    ) -> None:
        drive_temp = state.motor_temperatures.get(motor_id, 0)
        winding_temp = state.motor_winding_temperatures.get(motor_id, 0)
        peak_temp = max(drive_temp, winding_temp)
        t = self.thresholds

        if peak_temp < t.temp_hysteresis_recovery_c:
            if ms.cool_since is None:
                ms.cool_since = now
        else:
            ms.cool_since = None

    # ------------------------------------------------------------------
    # Predictive budget allocation
    # ------------------------------------------------------------------

    def set_policy(self, policy: BinPackPolicy) -> None:
        """Replace the active bin-pack policy. Takes effect on the next allocate_budget() call."""
        self._bin_policy = policy

    def allocate_budget(
        self, commands: list[ServoCommand], state: RobotState
    ) -> tuple[list[ServoCommand], list[int]]:
        """Feedback-driven bin-pack: seed an active bin then promote as measured current allows.

        Per-motor current is estimated from actual torque and velocity reported
        by the MIT reply frame, floored at a worst-case power draw so cold-start
        bin-packing is conservative:
          I = max(base + torque_coeff × τ² × (V_nom/V) + mech_coeff × |τ×ω| / V,
                  per_motor_worst_case_w / V)

        On each tick:
          1. Evict motors no longer commanded from the active set and pending queue.
          2. Insert newly commanded motors into the pending queue in priority order
             (BinPackPolicy.priority; None = ascending ID).
          3. Seed the active bin (if empty) by greedily filling from the front of
             the queue until budget would be exceeded.
          4. Promote up to add_step motors from the pending queue whenever the
             measured bus current EMA is below budget × headroom_factor.
          5. Apply reactive EMA scale to active motor commands (heaviest-first).
          6. Scale any individual active motor whose estimate alone exceeds budget.

        Returns (active_commands, pending_ids).
          active_commands: commands for motors in the active bin, possibly scaled.
          pending_ids: motor IDs held back this tick; the Controller idles these.
        """
        b = POWER_BUDGET
        V = max(state.battery_voltage_v, 8.0)
        budget = self._effective_budget()
        policy = self._bin_policy
        worst_case_a = b.per_motor_worst_case_w / V

        # 1. Estimate per-motor bus current (feedback + worst-case floor).
        estimates: dict[int, float] = {}
        for cmd in commands:
            tau_fb = abs(state.motor_torques.get(cmd.servo_id, 0.0))
            omega_fb = abs(state.motor_velocities.get(cmd.servo_id, 0.0))
            i_fb = (
                b.per_motor_base_a
                + b.per_motor_torque_coeff * tau_fb ** 2 * (b.bus_voltage_nominal_v / V)
                + b.per_motor_mech_coeff * tau_fb * omega_fb / V
            )
            estimates[cmd.servo_id] = max(i_fb, worst_case_a)

        self._budget_per_motor_est = dict(estimates)
        commanded_ids = set(estimates)
        self._last_motor_torques = {mid: state.motor_torques.get(mid, 0.0) for mid in commanded_ids}

        # 2. Evict motors no longer commanded.
        self._active_motor_set &= commanded_ids
        self._pending_queue = [mid for mid in self._pending_queue if mid in commanded_ids]

        # 3. Insert newly commanded motors into the pending queue in priority order.
        known = self._active_motor_set | set(self._pending_queue)
        new_motors = commanded_ids - known
        if new_motors:
            all_pending = set(self._pending_queue) | new_motors
            if policy.priority:
                p_idx = {mid: i for i, mid in enumerate(policy.priority)}
                self._pending_queue = sorted(all_pending, key=lambda m: p_idx.get(m, len(policy.priority)))
            else:
                self._pending_queue = sorted(all_pending)

        # 4. Seed active bin from the front of the pending queue if currently empty.
        just_seeded = False
        if not self._active_motor_set:
            accumulated = 0.0
            to_promote: list[int] = []
            for mid in list(self._pending_queue):
                i_est = estimates.get(mid, worst_case_a)
                if accumulated + i_est <= budget:
                    to_promote.append(mid)
                    accumulated += i_est
                else:
                    break
            for mid in to_promote:
                self._active_motor_set.add(mid)
                self._pending_queue.remove(mid)
            if to_promote:
                just_seeded = True
                logger.debug(
                    "[PowerManager] bin seeded: %s (%.2fA est / %.1fA budget)",
                    to_promote, accumulated, budget,
                )

        # 5. Promote from pending when measured bus current EMA shows headroom.
        # Skip on the same tick as seeding — the seed motors haven't fired yet so
        # the EMA doesn't reflect their draw. Promotion is purely EMA-gated on
        # subsequent ticks; the reactive backstop handles any post-promotion spike.
        promoted: list[int] = []
        if not just_seeded and self._pending_queue and self._current_ema < budget * policy.headroom_factor:
            for _ in range(policy.add_step):
                if not self._pending_queue:
                    break
                candidate = self._pending_queue.pop(0)
                self._active_motor_set.add(candidate)
                promoted.append(candidate)
            if promoted:
                logger.debug("[PowerManager] promoted motor(s) %s (I_ema=%.2fA)", promoted, self._current_ema)

        self._budget_total_est = sum(estimates.get(m, worst_case_a) for m in self._active_motor_set)

        # 6. Reactive EMA scale — apply uniformly across all active motors.
        cmd_map = {cmd.servo_id: cmd for cmd in commands}
        #    Heaviest-first distribution was causing lurches: noisy tau_fb rankings
        #    changed each tick, snapping individual motor scales discontinuously.
        self._budget_scale = self._reactive_scale

        # Predictive per-motor torque cap: solve τ_max from I ≈ base + coeff × τ²
        # so each motor cannot draw more than its budget share even before any EMA reacts.
        #
        # Uses the ACTUAL reported torque (tau_actual from the MIT reply) rather than
        # cmd.kp × pos_error, because the effective motor gain in MIT mode doesn't map
        # linearly to N·m/rad in our kp units. tau_actual reflects what the motor is
        # truly applying — including fight-back against clamps or user-induced forces.
        # kp is scaled down proportionally so the motor becomes compliant when over budget.
        if b.enable_per_motor_torque_cap and self._active_motor_set:
            n_active = len(self._active_motor_set)
            budget_per_motor = budget / max(1, n_active)
            headroom = max(0.0, budget_per_motor - b.per_motor_base_a)
            tau_max = math.sqrt(headroom / b.per_motor_torque_coeff) if b.per_motor_torque_coeff > 0 else float("inf")
            for mid in list(self._active_motor_set):
                cmd = cmd_map.get(mid)
                if cmd is None:
                    continue
                tau_actual = abs(state.motor_torques.get(mid, 0.0))
                tau_ff_capped = math.copysign(min(abs(cmd.torque_ff), tau_max), cmd.torque_ff) if cmd.torque_ff != 0.0 else 0.0
                if tau_actual > tau_max:
                    # Motor is over-torquing (fighting clamp or external force).
                    # Scale kp down so next command applies at most tau_max total torque.
                    kp_scale = tau_max / tau_actual
                    cmd_map[mid] = replace(cmd, kp=cmd.kp * kp_scale, torque_ff=tau_ff_capped)
                elif abs(cmd.torque_ff) > tau_max:
                    cmd_map[mid] = replace(cmd, torque_ff=tau_ff_capped)

        active_commands: list[ServoCommand] = []
        for mid in self._active_motor_set:
            cmd = cmd_map.get(mid)
            if cmd is None:
                continue
            if self._reactive_scale < 1.0:
                s = self._reactive_scale
                # Anti-windup setpoint tracking: blend commanded position toward
                # actual to prevent error from accumulating during suppression.
                # Without this, motors surge to close the backlog when scale releases,
                # causing the next current spike. The blend releases proportionally
                # as scale recovers, so the motor closes error gradually.
                if cmd.position is not None:
                    actual_pos = state.servo_positions.get(mid, cmd.position)
                    blended_pos = actual_pos + s * (cmd.position - actual_pos)
                    cmd = replace(cmd, position=blended_pos, kp=cmd.kp * s, kd=cmd.kd * s, torque_ff=cmd.torque_ff * s)
                else:
                    cmd = replace(cmd, kp=cmd.kp * s, kd=cmd.kd * s, torque_ff=cmd.torque_ff * s)
            active_commands.append(cmd)

        return active_commands, list(self._pending_queue)

    def _effective_budget(self) -> float:
        """Current budget ceiling based on detected power source."""
        if self._budget_override is not None:
            return self._budget_override
        b = POWER_BUDGET
        return b.wall_max_bus_current_a if self._power_source == PowerSource.WALL else b.max_bus_current_a

    def _effective_peak_budget(self) -> float:
        """Peak current limit above which sustained draw triggers an emergency stop."""
        b = POWER_BUDGET
        return b.wall_max_peak_current_a if self._power_source == PowerSource.WALL else b.max_peak_current_a

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _disable_motor(
        self, motor_id: int, ms: _MotorPowerState, reason: str
    ) -> None:
        ms.thermal_state = MotorThermalState.DISABLED
        ms.compliance_scale = 0.0
        ms.disable_reason = reason
        ms.cool_since = None
        self._pending_disable_motor_ids.append(motor_id)
        self._log_event(f"motor_{motor_id}_disabled: {reason}")
        logger.warning("Motor %d disabled: %s", motor_id, reason)

    def _trigger_global_emergency(self, reason: str) -> None:
        if self._system_state == SystemState.EMERGENCY_STOPPED:
            return
        self._system_state = SystemState.EMERGENCY_STOPPED
        self._system_disable_reason = reason
        self._pending_global_emergency = True
        for ms in self._motor_states.values():
            ms.thermal_state = MotorThermalState.DISABLED
            ms.compliance_scale = 0.0
            if not ms.disable_reason:
                ms.disable_reason = f"global_emergency: {reason}"
        self._log_event(f"EMERGENCY_STOPPED: {reason}")
        logger.error("Global emergency stop: %s", reason)

    def _log_event(self, msg: str) -> None:
        self._pending_events.append(msg)

    # ------------------------------------------------------------------
    # Control-loop API
    # ------------------------------------------------------------------

    def drain_disable_events(self) -> tuple[list[int], bool]:
        """Return (per_motor_ids_to_disable, global_emergency_triggered).
        Clears the pending queue. Call once per tick after update().
        """
        ids = list(self._pending_disable_motor_ids)
        is_global = self._pending_global_emergency
        self._pending_disable_motor_ids.clear()
        self._pending_global_emergency = False
        return ids, is_global

    def drain_voltage_cutoff(self) -> bool:
        """Return True (and clear) if a voltage cutoff event just occurred.
        Controller should call backend.disable_torques() when this returns True.
        """
        result = self._pending_voltage_cutoff
        self._pending_voltage_cutoff = False
        return result

    def is_motor_enabled(self, motor_id: int) -> bool:
        """False if this motor or the whole system is in DISABLED/EMERGENCY state."""
        if self._system_state == SystemState.EMERGENCY_STOPPED:
            return False
        ms = self._motor_states.get(motor_id)
        return ms is None or ms.thermal_state != MotorThermalState.DISABLED

    def get_compliance_scale(self, motor_id: int) -> float:
        """Thermal compliance scale for this motor. Range [0.0, 1.0].
        Budget scaling is handled separately by allocate_budget().
        """
        if self._system_state == SystemState.EMERGENCY_STOPPED:
            return 0.0
        ms = self._motor_states.get(motor_id)
        return ms.compliance_scale if ms is not None else 1.0

    def operator_reset(self, now: float | None = None) -> bool:
        """Attempt to clear emergency state and re-enable all motors.

        Succeeds only when all disabled motors have cooled below
        temp_hysteresis_recovery_c for temp_hysteresis_cooldown_s seconds
        AND have had no ERR=B/C for that same window.
        Returns True if reset succeeded; False with a log message if conditions not met.
        """
        if now is None:
            now = time.monotonic()
        t = self.thresholds

        for motor_id, ms in self._motor_states.items():
            if ms.thermal_state != MotorThermalState.DISABLED:
                continue
            if ms.cool_since is None:
                logger.warning(
                    "[PowerManager] Reset denied: motor %d not yet below %d°C",
                    motor_id, t.temp_hysteresis_recovery_c,
                )
                return False
            cooldown_met = now - ms.cool_since >= t.temp_hysteresis_cooldown_s
            if not cooldown_met:
                remaining = t.temp_hysteresis_cooldown_s - (now - ms.cool_since)
                logger.warning(
                    "[PowerManager] Reset denied: motor %d needs %.0fs more cooldown",
                    motor_id, remaining,
                )
                return False
            err_clear = (
                ms.last_err_overtemp_time == 0.0
                or now - ms.last_err_overtemp_time >= t.temp_hysteresis_cooldown_s
            )
            if not err_clear:
                logger.warning(
                    "[PowerManager] Reset denied: motor %d ERR overtemp cleared %.0fs ago (need %.0fs)",
                    motor_id, now - ms.last_err_overtemp_time, t.temp_hysteresis_cooldown_s,
                )
                return False

        for ms in self._motor_states.values():
            ms.thermal_state = MotorThermalState.NORMAL
            ms.compliance_scale = 1.0
            ms.disable_reason = ""
            ms.cool_since = None
        self._system_state = SystemState.RUNNING
        self._system_disable_reason = ""
        if self._voltage_state not in (VoltageState.LOW_WARNING, VoltageState.CUTOFF):
            self._voltage_state = VoltageState.NORMAL
        self._log_event("operator_reset: system re-enabled")
        logger.info("[PowerManager] Operator reset: system running")
        return True

    # ------------------------------------------------------------------
    # Telemetry snapshot
    # ------------------------------------------------------------------

    def get_telemetry(self, voltage_raw_v: float) -> PowerTelemetry:
        """Build a PowerTelemetry snapshot and drain the event buffer."""
        events = list(self._pending_events)
        self._pending_events.clear()
        return PowerTelemetry(
            voltage_raw_v=voltage_raw_v,
            voltage_ema_v=self._voltage_ema,
            voltage_state=self._voltage_state.value,
            current_amps_raw=self._last_current_a,
            current_amps_filtered=self._current_ema,
            current_amps_filtered_fast=self._current_ema_fast,
            current_amps_peak=self._window_peak_a,
            current_drive_scale=self._reactive_scale,
            system_state=self._system_state.value,
            motor_states={
                mid: ms.thermal_state.value for mid, ms in self._motor_states.items()
            },
            motor_disable_reasons={
                mid: ms.disable_reason for mid, ms in self._motor_states.items()
            },
            motor_compliance_scales={
                mid: ms.compliance_scale for mid, ms in self._motor_states.items()
            },
            events=events,
            power_source=self._power_source.value,
            estimated_total_current_a=self._budget_total_est,
            per_motor_estimated_current=dict(self._budget_per_motor_est),
            budget_scale_applied=self._budget_scale,
            voltage_cutoff_active=self._voltage_cutoff_active,
            active_motor_ids=sorted(self._active_motor_set),
            pending_motor_ids=list(self._pending_queue),
        )
