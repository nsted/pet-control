"""
petctl.power_manager — Motor thermal and voltage protection + feedback-driven power budget.

Safety-critical pure-logic module. No I/O, no async. All hardware actions are
communicated back to the Controller via drain_disable_events() / drain_voltage_cutoff(),
which then calls the backend. PowerManager only consumes RobotState and produces decisions.

Protection layers (in priority order):
  1. Thermal: per-motor temperature state machine (WARNING → DISABLED → EMERGENCY)
  2. Voltage cutoff: disables all motor torques below low_voltage_cutoff_v
  3. Feedback budget: estimates per-motor current from actual torque + velocity
     feedback (I ≈ copper-loss + mechanical-power/V); bin-packs motors into time
     slots so each slot stays within budget; stagger releases progressively as
     slot-0 torques settle; individual motors only scaled if they alone exceed budget
  4. Reactive EMA backstop: measured bus current safety net — reduction is
     concentrated on the heaviest-drawing motors so lighter ones keep full torque

ERR nibble codes from the GL40 II reply frame (upper 4 bits of byte 0):
    0 = Disable, 1 = Enable, 9 = Under-voltage, A = Over-current,
    B = MOS over-temperature, C = Motor winding over-temperature,
    D = Communication loss, E = Overload
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Optional

from petctl.config import POWER_BUDGET
from petctl.types import PowerTelemetry, RobotState, ServoCommand

logger = logging.getLogger(__name__)

# ERR codes that mean the driver itself detected overtemperature
_ERR_OVERTEMP_CODES: frozenset[int] = frozenset({0xB, 0xC})


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
    Motor protection and predictive power budget state machine.

    Pure logic — no I/O. Feed it RobotState every tick; drain disable events
    and query compliance scales to gate motor commands.

    Power budget flow (called by Controller each tick after update()):
      1. is_motor_enabled() / get_compliance_scale() — thermal gating
      2. allocate_budget(commands, state) — predictive scale + stagger schedule
      3. take_stagger_schedule() — stagger delays to pass to backend.set_stagger()
    """

    def __init__(
        self,
        thresholds: PowerThresholds = PowerThresholds(),
        reactive_ema_alpha: float | None = None,
    ) -> None:
        self.thresholds = thresholds
        # Allow override for testing (default comes from POWER_BUDGET.reactive_ema_alpha)
        self._reactive_ema_alpha: float = (
            reactive_ema_alpha if reactive_ema_alpha is not None else POWER_BUDGET.reactive_ema_alpha
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

        # Reactive EMA backstop (belt-and-suspenders behind predictive model)
        self._current_ema: float = 0.0
        self._last_current_a: float = 0.0
        self._reactive_scale: float = 1.0

        # Budget state (populated by allocate_budget, drained by get_telemetry)
        self._budget_scale: float = 1.0
        self._budget_total_est: float = 0.0
        self._budget_per_motor_est: dict[int, float] = {}
        self._pending_stagger: dict[int, float] = {}

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
        self._update_current(state.battery_current_amps)
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

        # Low-voltage cutoff (motor kill at battery floor)
        if raw_v < b.low_voltage_cutoff_v:
            if not self._voltage_cutoff_active:
                self._voltage_cutoff_active = True
                self._pending_voltage_cutoff = True
                self._voltage_state = VoltageState.CUTOFF
                self._log_event(f"voltage_cutoff: {raw_v:.2f}V < {b.low_voltage_cutoff_v:.1f}V — disabling all motors")
                logger.warning("[PowerManager] Voltage cutoff at %.2fV", raw_v)
        elif self._voltage_cutoff_active and raw_v >= b.low_voltage_recovery_v:
            self._voltage_cutoff_active = False
            self._log_event(f"voltage_cutoff_recovered: {raw_v:.2f}V >= {b.low_voltage_recovery_v:.1f}V")
            logger.info("[PowerManager] Voltage recovered: %.2fV", raw_v)
            # Fall through to update display state below

        # Power source detection (wall ≈14.7V, clearly above battery range)
        if raw_v >= b.wall_voltage_threshold_v:
            if self._power_source == PowerSource.BATTERY:
                self._wall_confirm_count += 1
                if self._wall_confirm_count >= b.wall_confirm_ticks:
                    old_budget = b.max_bus_current_a
                    new_budget = b.wall_max_bus_current_a
                    self._power_source = PowerSource.WALL
                    self._wall_confirm_count = b.wall_confirm_ticks  # clamp
                    self._log_event(
                        f"power_source: battery→wall "
                        f"(budget {old_budget:.1f}A→{new_budget:.1f}A "
                        f"peak {b.max_peak_current_a:.1f}A→{b.wall_max_peak_current_a:.1f}A)"
                    )
                    logger.info("[PowerManager] Power source: wall supply (budget %.1fA→%.1fA)", old_budget, new_budget)
            # else: already wall — keep confirm count pinned
        else:
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

    def _update_current(self, current_a: float) -> None:
        b = POWER_BUDGET
        alpha = self._reactive_ema_alpha
        self._current_ema = alpha * current_a + (1.0 - alpha) * self._current_ema
        self._last_current_a = current_a

        budget = self._effective_budget()
        start = budget * b.reactive_backstop_factor
        zero = budget * b.reactive_cutoff_factor

        v = self._current_ema
        if v >= zero:
            new_scale = 0.0
        elif v >= start:
            new_scale = 1.0 - (v - start) / (zero - start)
        else:
            new_scale = 1.0

        if new_scale != self._reactive_scale:
            self._log_event(
                f"reactive_backstop: {self._reactive_scale:.2f}→{new_scale:.2f} "
                f"(I_ema={v:.2f}A budget={budget:.1f}A)"
            )
        self._reactive_scale = new_scale

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

    def allocate_budget(
        self, commands: list[ServoCommand], state: RobotState
    ) -> tuple[list[ServoCommand], dict[int, float]]:
        """Feedback-driven current estimation with slot-based stagger when over budget.

        Per-motor current is estimated from actual torque and velocity reported
        by the MIT reply frame, not from commanded pos_error:
          I ≈ base + torque_coeff × τ² × (V_nom/V)   (copper loss)
                   + mech_coeff × |τ × ω| / V         (mechanical power delivery)

        When estimated total current exceeds budget, motors are bin-packed into
        time slots (heaviest draw first) so each slot stays within budget. Later
        slots are staggered by stagger_interval_s per slot index. The stagger
        is refreshed each tick while over budget, releasing progressively as
        slot-0 torques settle and total current drops. Individual motors are
        only scaled when a single motor alone exceeds the budget ceiling.

        Reactive EMA backstop reduction is distributed non-uniformly: the
        heaviest motors absorb the cut first so lighter motors keep full torque.

        Returns (scaled_commands, stagger_schedule) where stagger_schedule maps
        motor_id → delay_s.
        """
        b = POWER_BUDGET
        V = max(state.battery_voltage_v, 8.0)  # guard divide-by-zero on dead battery
        budget = self._effective_budget()

        # 1. Estimate per-motor bus current from actual torque + velocity feedback.
        # τ and ω are one frame delayed (last MIT reply) — fine for budget decisions.
        # Motors with no feedback yet report 0 → default to base_a (conservatively low).
        estimates: dict[int, float] = {}
        for cmd in commands:
            tau = abs(state.motor_torques.get(cmd.servo_id, 0.0))
            omega = abs(state.motor_velocities.get(cmd.servo_id, 0.0))
            i_copper = b.per_motor_base_a + b.per_motor_torque_coeff * tau ** 2 * (b.bus_voltage_nominal_v / V)
            i_mech = b.per_motor_mech_coeff * tau * omega / V
            estimates[cmd.servo_id] = i_copper + i_mech

        i_total = sum(estimates.values())
        self._budget_total_est = i_total
        self._budget_per_motor_est = dict(estimates)

        # 2. Per-motor reactive scale: distribute _reactive_scale reduction onto
        # heaviest motors first so lighter motors keep full torque.
        per_motor_reactive: dict[int, float] = {}
        if self._reactive_scale < 1.0 and estimates:
            reduction_remaining = i_total * (1.0 - self._reactive_scale)
            for motor_id, i_est in sorted(estimates.items(), key=lambda x: x[1], reverse=True):
                if reduction_remaining <= 0:
                    per_motor_reactive[motor_id] = 1.0
                else:
                    cut = min(reduction_remaining, i_est)
                    per_motor_reactive[motor_id] = max(0.0, (i_est - cut) / i_est) if i_est > 0 else 1.0
                    reduction_remaining -= cut

        # 3. Stagger schedule: bin-pack motors into time slots when over budget.
        # Slot 0 fires immediately; later slots are delayed to spread current draw.
        stagger_schedule: dict[int, float] = {}
        per_motor_predictive: dict[int, float] = {}

        if i_total > budget:
            sorted_motors = sorted(estimates.items(), key=lambda x: x[1], reverse=True)
            slots: list[list[int]] = []
            slot_totals: list[float] = []

            for motor_id, i_est in sorted_motors:
                placed = False
                for slot_idx in range(len(slots)):
                    if slot_totals[slot_idx] + i_est <= budget:
                        slots[slot_idx].append(motor_id)
                        slot_totals[slot_idx] += i_est
                        placed = True
                        break
                if not placed:
                    slots.append([motor_id])
                    slot_totals.append(i_est)

            for slot_idx, slot in enumerate(slots):
                if slot_idx > 0:
                    for motor_id in slot:
                        stagger_schedule[motor_id] = slot_idx * b.stagger_interval_s

            if stagger_schedule:
                logger.debug(
                    "[PowerManager] budget: %.2fA > %.1fA — staggering %d motors across %d slots",
                    i_total, budget, len(stagger_schedule), len(slots),
                )

            # Only scale individual motors that alone exceed the full budget
            for motor_id, i_est in estimates.items():
                per_motor_predictive[motor_id] = budget / i_est if i_est > budget else 1.0
        else:
            per_motor_predictive = {mid: 1.0 for mid in estimates}

        # 4. Combine predictive and reactive scales per motor (most restrictive wins)
        per_motor_scale: dict[int, float] = {
            mid: min(per_motor_predictive.get(mid, 1.0), per_motor_reactive.get(mid, 1.0))
            for mid in estimates
        }
        self._budget_scale = min(per_motor_scale.values()) if per_motor_scale else 1.0

        # 5. Apply per-motor scales to commands
        if all(s >= 1.0 for s in per_motor_scale.values()):
            return commands, stagger_schedule

        scaled: list[ServoCommand] = []
        for cmd in commands:
            s = per_motor_scale.get(cmd.servo_id, 1.0)
            if s < 1.0:
                cmd = replace(cmd,
                    kp=cmd.kp * s,
                    kd=cmd.kd * s,
                    torque_ff=cmd.torque_ff * s,
                )
            scaled.append(cmd)
        return scaled, stagger_schedule

    def _effective_budget(self) -> float:
        """Current budget ceiling based on detected power source."""
        b = POWER_BUDGET
        return b.wall_max_bus_current_a if self._power_source == PowerSource.WALL else b.max_bus_current_a

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
        )
