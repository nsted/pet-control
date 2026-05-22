"""
petctl.bus_safety — Bus current protection via impedance modulation.

Pure logic — no I/O. Feed it RobotState every tick; read modulation_factor
to scale kp/kd on motor commands. No velocity cap — modulation acts only on
impedance so motion is never abruptly interrupted.

Real-time voltage modulation has been removed: the bus voltage ADC on the
head board reads high under motor load due to ESP32 ground-reference shift
(confirmed artifact, not a real overvoltage). Voltage safety is retained as
two lightweight checks:
  - check_boot_voltage(): call once at startup when quiescent.
  - update_idle_voltage_check(): rate-gated, only samples when current is low.
"""

from __future__ import annotations

import collections
import logging
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from petctl.config import BUS_SAFETY, BusSafetyLimits
from petctl.types import PowerTelemetry, RobotState

logger = logging.getLogger(__name__)


class BusSafetyState(Enum):
    NORMAL = "NORMAL"
    MODULATING = "MODULATING"


@dataclass
class _BusSafetyInternalState:
    """Snapshot of internal state for debugging — not part of the public API."""
    i_factor_target: float
    current_stale: bool


class BusSafetyMonitor:
    """
    Bus current protection with graduated impedance modulation.

    Pure logic — no I/O. Call update() every control tick, then read
    modulation_factor to scale kp/kd on motor commands.
    """

    def __init__(self, limits: BusSafetyLimits = BUS_SAFETY) -> None:
        self.limits = limits

        # Modulation factor target (hysteresis-gated)
        self._i_factor_target: float = 1.0
        self._i_recovering: bool = True

        # Rate-limited output
        self._modulation_factor: float = 1.0

        # Current median filter
        self._current_window: collections.deque[float] = collections.deque(
            maxlen=limits.current_median_window
        )

        # Sensor staleness tracking
        self._last_valid_current_t: Optional[float] = None
        self._last_current_raw: Optional[int] = None
        self._current_stuck_since: Optional[float] = None
        self._current_stale: bool = False

        # State for transition logging
        self._state: BusSafetyState = BusSafetyState.NORMAL
        self._last_log_factor: float = 1.0

        # Idle voltage check state
        self._last_idle_voltage_check_t: Optional[float] = None

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    @property
    def modulation_factor(self) -> float:
        """Current smoothed modulation factor in [0,1]. 1.0 = normal, 0.0 = full compliance."""
        return self._modulation_factor

    def update(self, state: RobotState, now: float) -> None:
        """Evaluate bus current; update modulation factor. Call every tick."""
        dt = max(state.dt, 1e-3)
        self._update_current(state.battery_current_amps, state.battery_current_raw, now, dt)
        self._update_modulation_factor(self._i_factor_target, dt)
        self._update_state_and_log(now)

    def update_telemetry(self, telem: PowerTelemetry) -> None:
        """Populate bus safety fields on an existing PowerTelemetry snapshot."""
        telem.bus_modulation_factor = self._modulation_factor
        if self._current_window:
            telem.bus_current_filtered_a = _median(list(self._current_window))

    def check_boot_voltage(self, voltage_v: float) -> bool:
        """
        One-shot LVC check at startup (call when system is quiescent).

        Returns True if voltage is above the low-voltage cutoff threshold.
        Logs a warning if below; caller decides whether to block startup.
        """
        lim = self.limits
        if voltage_v < 5.0 or voltage_v > 40.0:
            logger.warning(
                "[BusSafety] Boot voltage reading out of range (%.2fV) — ADC not ready?", voltage_v
            )
            return True  # don't block on a bad reading
        if voltage_v < lim.voltage_lvc_v:
            logger.warning(
                "[BusSafety] Battery LOW at boot: %.2fV (LVC=%.1fV) — charge before use",
                voltage_v, lim.voltage_lvc_v,
            )
            return False
        logger.info("[BusSafety] Boot voltage OK: %.2fV", voltage_v)
        return True

    def update_idle_voltage_check(self, voltage_v: float, current_a: float, now: float) -> None:
        """
        Rate-gated idle voltage sample (call every tick; method self-gates).

        Only samples when current is below the idle threshold so the ADC
        ground-shift artifact doesn't corrupt the reading. Logs an SoC estimate.
        """
        lim = self.limits
        if self._last_idle_voltage_check_t is not None:
            if now - self._last_idle_voltage_check_t < lim.idle_voltage_check_interval_s:
                return

        if current_a >= lim.idle_voltage_current_threshold_a:
            return  # not idle enough — skip this window

        if voltage_v < 5.0 or voltage_v > 40.0:
            return  # out-of-range reading

        self._last_idle_voltage_check_t = now

        # Rough 3S SoC estimate: 12.6V=full, 10.5V=empty
        soc_pct = max(0.0, min(100.0, (voltage_v - 10.5) / (12.6 - 10.5) * 100.0))
        logger.info(
            "[BusSafety] Idle voltage sample: %.2fV  (~%.0f%% SoC)",
            voltage_v, soc_pct,
        )

    # ------------------------------------------------------------------
    # Internal — current
    # ------------------------------------------------------------------

    def _update_current(
        self, i_raw: float, current_raw_adc: int, now: float, dt: float
    ) -> None:
        lim = self.limits

        # Stuck-value detection (ADC not updating)
        if current_raw_adc == self._last_current_raw:
            if self._current_stuck_since is None:
                self._current_stuck_since = now
            elif now - self._current_stuck_since > lim.current_stale_timeout_s:
                if not self._current_stale:
                    self._current_stale = True
                    logger.warning("[BusSafety] Bus current sensor stuck — disabling current modulation")
                self._i_factor_target = 1.0
                return
        else:
            self._current_stuck_since = None

        self._last_current_raw = current_raw_adc

        # Sanity check
        if i_raw < lim.current_sanity_min_a or i_raw > lim.current_sanity_max_a:
            if self._last_valid_current_t is not None:
                age = now - self._last_valid_current_t
                if age > lim.current_stale_timeout_s and not self._current_stale:
                    self._current_stale = True
                    logger.warning("[BusSafety] Bus current sensor stale (%.1fs)", age)
            self._i_factor_target = 1.0
            return

        self._last_valid_current_t = now
        self._current_stale = False
        self._current_window.append(i_raw)

        if len(self._current_window) < lim.current_median_window:
            self._i_factor_target = 1.0
            return

        i_filtered = _median(list(self._current_window))

        onset = lim.current_onset_a
        ceiling = lim.current_ceiling_a
        recovery_threshold = onset - lim.current_hysteresis_a

        if i_filtered >= ceiling:
            target = 0.0
            self._i_recovering = False
        elif i_filtered >= onset:
            target = 1.0 - (i_filtered - onset) / (ceiling - onset)
            self._i_recovering = False
        else:
            if i_filtered <= recovery_threshold:
                self._i_recovering = True
            target = 1.0 if self._i_recovering else self._i_factor_target

        self._i_factor_target = max(0.0, min(1.0, target))

    # ------------------------------------------------------------------
    # Internal — rate-limited output
    # ------------------------------------------------------------------

    def _update_modulation_factor(self, target: float, dt: float) -> None:
        lim = self.limits
        if target < self._modulation_factor:
            step = lim.modulation_ramp_down_per_s * dt
            self._modulation_factor = max(self._modulation_factor - step, target)
        else:
            step = lim.modulation_ramp_up_per_s * dt
            self._modulation_factor = min(self._modulation_factor + step, target)
        self._modulation_factor = max(0.0, min(1.0, self._modulation_factor))

    # ------------------------------------------------------------------
    # Internal — state tracking / logging
    # ------------------------------------------------------------------

    def _update_state_and_log(self, now: float) -> None:
        new_state = (
            BusSafetyState.MODULATING
            if self._modulation_factor < 0.99
            else BusSafetyState.NORMAL
        )

        if new_state != self._state:
            logger.info(
                "[BusSafety] State: %s → %s  factor=%.3f  i_tgt=%.3f",
                self._state.value,
                new_state.value,
                self._modulation_factor,
                self._i_factor_target,
            )
            self._state = new_state

        f = self._modulation_factor
        prev = self._last_log_factor
        if prev >= 0.95 and f < 0.95:
            logger.warning(
                "[BusSafety] Modulation engaging: factor=%.3f  i_tgt=%.3f",
                f, self._i_factor_target,
            )
        elif prev < 0.50 and f >= 0.50:
            logger.info("[BusSafety] Modulation easing: factor=%.3f", f)
        elif prev < 0.99 and f >= 0.99:
            logger.info("[BusSafety] Modulation cleared: factor=%.3f", f)
        self._last_log_factor = f


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

def _median(values: list[float]) -> float:
    """Return the median of a non-empty list."""
    s = sorted(values)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 else (s[mid - 1] + s[mid]) / 2.0
