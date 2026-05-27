"""
petctl.power_manager — Motor thermal and voltage protection.

Safety-critical pure-logic module. No I/O, no async. All hardware actions are
communicated back to the Controller via drain_disable_events(), which then calls
the backend. PowerManager only consumes RobotState and produces decisions.

All thresholds live in PowerThresholds — tune there without touching the logic.

ERR nibble codes from the GL40 II reply frame (upper 4 bits of byte 0):
    0 = Disable, 1 = Enable, 9 = Under-voltage, A = Over-current,
    B = MOS over-temperature, C = Motor winding over-temperature,
    D = Communication loss, E = Overload
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from petctl.types import PowerTelemetry, RobotState

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


class SystemState(Enum):
    RUNNING = "RUNNING"
    EMERGENCY_STOPPED = "EMERGENCY_STOPPED"


@dataclass(frozen=True)
class PowerThresholds:
    """All protection thresholds in one place. Tune here without touching logic."""

    # --- Thermal (per motor; applied to max(drive_temp, winding_temp)) ---
    temp_soft_warning_c: float = 55.0       # reduce Kp/Kd/τ_ff by 50%
    temp_hard_cutoff_c: float = 65.0        # exit motor mode for this motor
    temp_global_emergency_c: float = 75.0   # exit motor mode for ALL motors
    temp_hysteresis_recovery_c: float = 50.0
    temp_hysteresis_cooldown_s: float = 30.0

    # --- Voltage (3S LiPo: full=12.6 V, nominal=11.1 V; or 15 V adapter) ---
    # Voltage is display-only — no safety actions are taken based on voltage.
    voltage_low_warning_v: float = 10.8    # 3.6 V/cell — informational only
    voltage_ema_alpha: float = 0.02        # ~50-sample window (~1.7 s at 30 Hz)

    # --- ADC sanity (applied before EMA) ---
    # 5 V floor rejects pre-sensor startup garbage (e.g. 2.15 V before the
    # head board ADC initialises).
    voltage_sanity_min_v: float = 5.0
    voltage_sanity_max_v: float = 40.0

    # --- Current limiting (safety backstop — applied globally) ---
    current_limit_start_a: float = 3.5    # begin scaling at this bus current
    current_limit_zero_a: float = 4.0     # compliance reaches 0.0 at this current
    current_ema_alpha: float = 0.1        # ~10-sample window (~0.33 s at 30 Hz)

    # --- Current budget targeting (servo around this operating point) ---
    # Under target: global boost ramps up slowly so motors work harder.
    # Over target: per-motor cut applied fast; highest-torque motors cut most.
    current_target_a: float = 1.0
    current_target_deadband_a: float = 0.05  # no adjustment within ±50mA of target
    current_target_boost_max: float = 1.5    # max boost multiplier
    current_target_boost_rate: float = 0.02  # scale increase per tick (slow)
    current_target_cut_rate: float = 0.5     # max scale drop per tick (fast)


@dataclass
class _MotorPowerState:
    thermal_state: MotorThermalState = MotorThermalState.NORMAL
    disable_reason: str = ""
    compliance_scale: float = 1.0
    current_budget_scale: float = 1.0   # per-motor scale from current budget targeting
    # Monotonic time when temp first dropped below recovery threshold (None = not yet)
    cool_since: Optional[float] = None
    # Monotonic time of last ERR=B/C event; 0.0 = none seen
    last_err_overtemp_time: float = 0.0


class PowerManager:
    """
    Motor thermal and voltage protection state machine.

    Pure logic — no I/O. Feed it RobotState every tick; drain disable events
    and query compliance scales to gate motor commands.
    """

    def __init__(self, thresholds: PowerThresholds = PowerThresholds()) -> None:
        self.thresholds = thresholds

        self._motor_states: dict[int, _MotorPowerState] = {}
        self._system_state: SystemState = SystemState.RUNNING
        self._system_disable_reason: str = ""

        # Voltage — EMA for display only; no safety actions taken on voltage
        self._voltage_ema: Optional[float] = None
        self._voltage_state: VoltageState = VoltageState.NORMAL

        # Current limiting (safety backstop)
        self._current_ema: float = 0.0
        self._current_drive_scale: float = 1.0
        self._last_current_a: float = 0.0

        # Current budget targeting
        self._current_target_global: float = 1.0

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
            return  # Frozen until operator_reset()

        self._update_voltage(state.battery_voltage_v)
        self._update_current(state.battery_current_amps)
        self._update_current_target(state)
        self._update_motors(state, now)

    # ------------------------------------------------------------------
    # Voltage — EMA filter for display only; no safety actions
    # ------------------------------------------------------------------

    def _update_voltage(self, raw_v: float) -> None:
        t = self.thresholds
        if raw_v < t.voltage_sanity_min_v or raw_v > t.voltage_sanity_max_v:
            return
        if self._voltage_ema is None:
            self._voltage_ema = raw_v
        else:
            self._voltage_ema = t.voltage_ema_alpha * raw_v + (1.0 - t.voltage_ema_alpha) * self._voltage_ema
        if self._voltage_ema < t.voltage_low_warning_v:
            if self._voltage_state != VoltageState.LOW_WARNING:
                self._log_event(f"voltage_low_warning: {self._voltage_ema:.2f}V")
                self._voltage_state = VoltageState.LOW_WARNING
        else:
            if self._voltage_state == VoltageState.LOW_WARNING:
                self._log_event(f"voltage_normal: {self._voltage_ema:.2f}V")
            self._voltage_state = VoltageState.NORMAL

    # ------------------------------------------------------------------
    # Current limiting
    # ------------------------------------------------------------------

    def _update_current(self, current_a: float) -> None:
        t = self.thresholds
        self._current_ema = (
            t.current_ema_alpha * current_a
            + (1.0 - t.current_ema_alpha) * self._current_ema
        )
        self._last_current_a = current_a
        v = self._current_ema
        if v >= t.current_limit_zero_a:
            new_scale = 0.0
        elif v >= t.current_limit_start_a:
            new_scale = 1.0 - (v - t.current_limit_start_a) / (
                t.current_limit_zero_a - t.current_limit_start_a
            )
        else:
            new_scale = 1.0
        if new_scale != self._current_drive_scale:
            self._log_event(
                f"current_scale: {self._current_drive_scale:.2f}->{new_scale:.2f} "
                f"(I={v:.2f}A)"
            )
        self._current_drive_scale = new_scale

    # ------------------------------------------------------------------
    # Current budget targeting
    # ------------------------------------------------------------------

    def _update_current_target(self, state: RobotState) -> None:
        """Servo total bus current toward current_target_a.

        Under target: ramp global boost up slowly so all motors work harder.
        Over target: cut fast; highest-torque motors (biggest current consumers)
        are cut most, low-torque motors are spared proportionally.
        """
        t = self.thresholds
        current = self._current_ema
        target = t.current_target_a

        if current <= 0.0:
            return

        if abs(current - target) <= t.current_target_deadband_a:
            return  # within dead-band — hold current scale

        if current < target:
            new_global = min(
                self._current_target_global + t.current_target_boost_rate,
                t.current_target_boost_max,
            )
            self._current_target_global = new_global
            for ms in self._motor_states.values():
                ms.current_budget_scale = new_global
            for mid in state.active_servo_ids:
                ms = self._motor_states.setdefault(mid, _MotorPowerState())
                ms.current_budget_scale = new_global
        else:
            # Proportional target; rate-limit how fast we drop
            desired = target / current
            new_global = max(
                self._current_target_global - t.current_target_cut_rate,
                desired,
            )
            self._current_target_global = new_global
            if new_global != self._current_target_global:
                self._log_event(
                    f"current_target_scale: {new_global:.2f} (I={current:.2f}A)"
                )

            # Distribute cut proportionally: high-torque motors absorb more.
            # scale_i = global + (1 - global) * (1 - torque_i / max_torque)
            # → max-torque motor gets exactly global; zero-torque motor stays at 1.0.
            torques = {
                mid: abs(state.motor_torques.get(mid, 0.0))
                for mid in state.active_servo_ids
            }
            max_torque = max(torques.values(), default=0.0)
            for mid in state.active_servo_ids:
                ms = self._motor_states.setdefault(mid, _MotorPowerState())
                if max_torque > 0.0:
                    share = torques.get(mid, 0.0) / max_torque
                    ms.current_budget_scale = new_global + (1.0 - new_global) * (1.0 - share)
                else:
                    ms.current_budget_scale = new_global

    # ------------------------------------------------------------------
    # Thermal protection
    # ------------------------------------------------------------------

    def _update_motors(self, state: RobotState, now: float) -> None:
        t = self.thresholds

        for motor_id in state.active_servo_ids:
            ms = self._motor_states.setdefault(motor_id, _MotorPowerState())

            if ms.thermal_state == MotorThermalState.DISABLED:
                # Track cooling while disabled (needed for hysteresis check)
                self._track_cooling(motor_id, ms, state, now)
                continue

            drive_temp = state.motor_temperatures.get(motor_id, 0)
            winding_temp = state.motor_winding_temperatures.get(motor_id, 0)
            err_code = state.motor_err_codes.get(motor_id, 0)
            peak_temp = max(drive_temp, winding_temp)

            # ERR=B (MOS over-temp) or ERR=C (winding over-temp): immediate hard cutoff
            if err_code in _ERR_OVERTEMP_CODES:
                ms.last_err_overtemp_time = now
                self._disable_motor(
                    motor_id, ms,
                    f"err_overtemp: ERR=0x{err_code:X} "
                    f"(drive={drive_temp}°C winding={winding_temp}°C)",
                )
                continue

            # Global emergency threshold
            if peak_temp >= t.temp_global_emergency_c:
                self._trigger_global_emergency(
                    f"motor_{motor_id}_global_overtemp: {peak_temp}°C "
                    f">= {t.temp_global_emergency_c}°C"
                )
                return

            # Hard per-motor cutoff
            if peak_temp >= t.temp_hard_cutoff_c:
                self._disable_motor(
                    motor_id, ms,
                    f"thermal_cutoff: {peak_temp}°C >= {t.temp_hard_cutoff_c}°C",
                )
                continue

            # Soft warning
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
        """Update cool_since for a disabled motor's hysteresis recovery check."""
        drive_temp = state.motor_temperatures.get(motor_id, 0)
        winding_temp = state.motor_winding_temperatures.get(motor_id, 0)
        peak_temp = max(drive_temp, winding_temp)
        t = self.thresholds

        if peak_temp < t.temp_hysteresis_recovery_c:
            if ms.cool_since is None:
                ms.cool_since = now
        else:
            ms.cool_since = None  # Temp rose again — restart the timer

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
        """
        Return (per_motor_ids_to_disable, global_emergency_triggered).
        Clears the pending queue. Call once per tick after update().
        """
        ids = list(self._pending_disable_motor_ids)
        is_global = self._pending_global_emergency
        self._pending_disable_motor_ids.clear()
        self._pending_global_emergency = False
        return ids, is_global

    def is_motor_enabled(self, motor_id: int) -> bool:
        """False if this motor or the whole system is in DISABLED/EMERGENCY state."""
        if self._system_state == SystemState.EMERGENCY_STOPPED:
            return False
        ms = self._motor_states.get(motor_id)
        return ms is None or ms.thermal_state != MotorThermalState.DISABLED

    def get_compliance_scale(self, motor_id: int) -> float:
        """Product of thermal, safety current-limiting, and budget-targeting scales."""
        if self._system_state == SystemState.EMERGENCY_STOPPED:
            return 0.0
        ms = self._motor_states.get(motor_id)
        thermal_scale = ms.compliance_scale if ms is not None else 1.0
        budget_scale = ms.current_budget_scale if ms is not None else self._current_target_global
        return thermal_scale * self._current_drive_scale * budget_scale

    def operator_reset(self, now: float | None = None) -> bool:
        """
        Attempt to clear emergency state and re-enable all motors.

        Succeeds only when all disabled motors have cooled below
        temp_hysteresis_recovery_c for temp_hysteresis_cooldown_s seconds
        AND have had no ERR=B/C for that same window.
        Returns True if reset succeeded; False with a log message if conditions not met.

        Args:
            now: Current monotonic time. Defaults to time.monotonic(). Pass explicitly
                 in tests to keep time consistent with the value passed to update().
        """
        if now is None:
            now = time.monotonic()
        t = self.thresholds

        # Every disabled motor must be fully cooled and ERR-clear
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
                remaining = t.temp_hysteresis_cooldown_s - (now - ms.last_err_overtemp_time)
                logger.warning(
                    "[PowerManager] Reset denied: motor %d ERR overtemp cleared %.0fs ago (need %.0fs)",
                    motor_id, now - ms.last_err_overtemp_time, t.temp_hysteresis_cooldown_s,
                )
                return False

        # All conditions met
        for ms in self._motor_states.values():
            ms.thermal_state = MotorThermalState.NORMAL
            ms.compliance_scale = 1.0
            ms.disable_reason = ""
            ms.cool_since = None
        self._system_state = SystemState.RUNNING
        self._system_disable_reason = ""
        # Preserve LOW_WARNING — it resolves when voltage recovers naturally
        if self._voltage_state != VoltageState.LOW_WARNING:
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
            current_drive_scale=self._current_drive_scale,
            current_target_scale=self._current_target_global,
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
        )
