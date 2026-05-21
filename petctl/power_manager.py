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

import collections
import logging
import statistics
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
    CRITICAL_LOW = "CRITICAL_LOW"
    SPIKE_RATE_EMERGENCY = "SPIKE_RATE_EMERGENCY"
    ABSOLUTE_EMERGENCY = "ABSOLUTE_EMERGENCY"


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

    # --- Voltage (tuned for 3S LiPo: full=12.6 V, nominal=11.1 V) ---
    voltage_low_warning_v: float = 10.8    # 3.6 V/cell
    voltage_critical_low_v: float = 10.2   # 3.4 V/cell
    voltage_spike_threshold_v: float = 16.0
    voltage_spike_rate_window_s: float = 60.0
    voltage_spike_rate_emergency_count: int = 5
    voltage_absolute_emergency_v: float = 30.0

    # --- ADC sanity (applied before median filter and spike detection) ---
    # 5 V floor rejects pre-sensor startup garbage (e.g. 2.15 V before the
    # head board ADC initialises) that would otherwise fill the median window.
    voltage_sanity_min_v: float = 5.0
    voltage_sanity_max_v: float = 40.0
    voltage_median_window: int = 5

    # --- Low-voltage confirmation: require this many consecutive filtered
    #     readings below threshold before transitioning to WARNING or triggering
    #     CRITICAL_LOW emergency.  Suppresses single-sample noise.
    voltage_low_consec_required: int = 3


@dataclass
class _MotorPowerState:
    thermal_state: MotorThermalState = MotorThermalState.NORMAL
    disable_reason: str = ""
    compliance_scale: float = 1.0
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

        # Voltage state
        self._voltage_window: collections.deque[float] = collections.deque(
            maxlen=thresholds.voltage_median_window
        )
        self._voltage_filtered: Optional[float] = None
        self._voltage_state: VoltageState = VoltageState.NORMAL

        # Consecutive below-threshold counter (reset when voltage recovers)
        self._voltage_low_consec: int = 0

        # Spike tracking
        self._spike_times: collections.deque[float] = collections.deque()
        self._spike_count_total: int = 0
        self._last_spike_peak_v: float = 0.0

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

        self._update_voltage(state.battery_voltage_v, now)
        if self._system_state == SystemState.EMERGENCY_STOPPED:
            return  # Voltage check may have tripped emergency
        self._update_motors(state, now)

    # ------------------------------------------------------------------
    # Voltage protection
    # ------------------------------------------------------------------

    def _update_voltage(self, raw_v: float, now: float) -> None:
        t = self.thresholds

        # Sanity bounds — discard physically impossible readings entirely
        if raw_v < t.voltage_sanity_min_v or raw_v > t.voltage_sanity_max_v:
            return

        # Spike detection on raw (pre-filter) value
        if raw_v > t.voltage_spike_threshold_v:
            self._spike_count_total += 1
            self._last_spike_peak_v = max(self._last_spike_peak_v, raw_v)
            self._spike_times.append(now)
            self._log_event(
                f"voltage_spike: {raw_v:.2f}V (total={self._spike_count_total})"
            )

        # Absolute emergency — individual sanity-passed sample above hard limit
        if raw_v > t.voltage_absolute_emergency_v:
            self._trigger_global_emergency(
                f"voltage_absolute_emergency: {raw_v:.2f}V > {t.voltage_absolute_emergency_v}V"
            )
            return

        # Age out spikes outside the rolling window
        cutoff = now - t.voltage_spike_rate_window_s
        while self._spike_times and self._spike_times[0] < cutoff:
            self._spike_times.popleft()

        # Spike rate emergency
        if len(self._spike_times) > t.voltage_spike_rate_emergency_count:
            self._trigger_global_emergency(
                f"voltage_spike_rate_emergency: {len(self._spike_times)} spikes "
                f"in {t.voltage_spike_rate_window_s:.0f}s"
            )
            return

        # Feed median filter
        self._voltage_window.append(raw_v)
        if len(self._voltage_window) == t.voltage_median_window:
            self._voltage_filtered = statistics.median(self._voltage_window)
        else:
            self._voltage_filtered = None
            return  # Window not full yet — no threshold checks

        v = self._voltage_filtered

        if v < t.voltage_low_warning_v:
            self._voltage_low_consec += 1
        else:
            self._voltage_low_consec = 0
            if self._voltage_state == VoltageState.LOW_WARNING:
                self._log_event(f"voltage_normal: {v:.2f}V")
            self._voltage_state = VoltageState.NORMAL

        if self._voltage_low_consec >= t.voltage_low_consec_required:
            if v < t.voltage_critical_low_v:
                self._trigger_global_emergency(
                    f"voltage_critical_low: {v:.2f}V < {t.voltage_critical_low_v}V"
                )
            elif self._voltage_state != VoltageState.LOW_WARNING:
                self._log_event(
                    f"voltage_low_warning: {v:.2f}V < {t.voltage_low_warning_v}V"
                )
                self._voltage_state = VoltageState.LOW_WARNING

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
        """1.0 (normal), 0.5 (thermal warning), or 0.0 (disabled / emergency)."""
        if self._system_state == SystemState.EMERGENCY_STOPPED:
            return 0.0
        ms = self._motor_states.get(motor_id)
        return ms.compliance_scale if ms is not None else 1.0

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

        # Voltage must be in a safe state
        if self._voltage_state in (
            VoltageState.CRITICAL_LOW,
            VoltageState.SPIKE_RATE_EMERGENCY,
            VoltageState.ABSOLUTE_EMERGENCY,
        ):
            logger.warning(
                "[PowerManager] Reset denied: voltage state = %s",
                self._voltage_state.value,
            )
            return False

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
        if self._voltage_state == VoltageState.LOW_WARNING:
            pass  # Preserve low-warning; it resolves when voltage recovers
        else:
            self._voltage_state = VoltageState.NORMAL
        self._spike_times.clear()
        self._voltage_low_consec = 0
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
            voltage_filtered_v=self._voltage_filtered,
            voltage_state=self._voltage_state.value,
            voltage_spike_count=self._spike_count_total,
            voltage_spike_rate_per_min=len(self._spike_times),
            voltage_last_spike_peak_v=self._last_spike_peak_v,
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
