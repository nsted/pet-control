"""
petctl.bus_safety — Bus voltage and current protection with source inference.

Pure logic — no I/O. Feed it RobotState every tick; read modulation_factor
to gate motor commands. Runs alongside PowerManager (which handles thermal
protection and extreme voltage events); the two are combined multiplicatively
in the Controller.

Design rationale:
  - Current is the leading indicator of the regen-spike fault mode.
    Current onset threshold (3.5A) acts before voltage breaks the ceiling.
  - Voltage modulation onset depends on inferred power source: battery (~13V)
    vs. wall supply (~14.8V). 15V is the absolute ceiling regardless of source.
  - Modulation factor is continuous [0,1] — no hard cutoffs, always smooth.
  - Restrict fast (1.0/s), recover slowly (0.15/s) to prevent oscillation.
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


class SourceInference(Enum):
    BATTERY = "BATTERY"
    SUPPLY = "SUPPLY"
    UNKNOWN = "UNKNOWN"  # during initial boot convergence only


class BusSafetyState(Enum):
    NORMAL = "NORMAL"
    MODULATING = "MODULATING"
    SENSOR_STALE = "SENSOR_STALE"


@dataclass
class _BusSafetyInternalState:
    """Snapshot of internal state for debugging — not part of the public API."""
    source_score: float
    v_factor_target: float
    i_factor_target: float
    combined_target: float
    voltage_stale: bool
    current_stale: bool


class BusSafetyMonitor:
    """
    Bus voltage/current protection with graduated modulation and source inference.

    Pure logic — no I/O. Call update() every control tick, then read
    modulation_factor to apply to velocity caps and impedance gains.
    """

    def __init__(self, limits: BusSafetyLimits = BUS_SAFETY) -> None:
        self.limits = limits

        # Source inference: score in [0,1]; 0=battery, 1=supply.
        # Boot conservative (battery) so modulation onset is tighter until
        # voltage history establishes the source.
        self._source_score: float = 0.0
        self._inferred_source: SourceInference = SourceInference.BATTERY
        self._source_initialized_t: Optional[float] = None

        # Modulation factor targets (hysteresis-gated)
        self._v_factor_target: float = 1.0
        self._i_factor_target: float = 1.0
        self._v_recovering: bool = True   # True = currently in recovery zone (below onset)
        self._i_recovering: bool = True

        # Rate-limited output
        self._modulation_factor: float = 1.0

        # Current median filter
        self._current_window: collections.deque[float] = collections.deque(
            maxlen=limits.current_median_window
        )

        # Sensor staleness tracking
        self._last_valid_voltage_t: Optional[float] = None
        self._last_valid_current_t: Optional[float] = None
        self._last_current_raw: Optional[int] = None
        self._current_stuck_since: Optional[float] = None
        self._voltage_stale: bool = False
        self._current_stale: bool = False

        # State for transition logging
        self._state: BusSafetyState = BusSafetyState.NORMAL
        self._last_log_factor: float = 1.0

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    @property
    def modulation_factor(self) -> float:
        """Current smoothed modulation factor in [0,1]. 1.0 = normal, 0.0 = full limp."""
        return self._modulation_factor

    @property
    def inferred_source(self) -> SourceInference:
        return self._inferred_source

    def update(self, state: RobotState, now: float) -> None:
        """Evaluate bus voltage and current; update modulation factor. Call every tick."""
        dt = max(state.dt, 1e-3)

        if self._source_initialized_t is None:
            self._source_initialized_t = now

        self._update_voltage(state.battery_voltage_v, now, dt)
        self._update_current(state.battery_current_amps, state.battery_current_raw, now, dt)

        combined_target = self._v_factor_target * self._i_factor_target
        self._update_modulation_factor(combined_target, dt)
        self._update_state_and_log(now)

    def update_telemetry(self, telem: PowerTelemetry) -> None:
        """Populate bus safety fields on an existing PowerTelemetry snapshot."""
        telem.bus_modulation_factor = self._modulation_factor
        telem.bus_source_inference = self._inferred_source.value
        if self._current_window:
            telem.bus_current_filtered_a = _median(list(self._current_window))

    # ------------------------------------------------------------------
    # Internal — voltage
    # ------------------------------------------------------------------

    def _update_voltage(self, v_raw: float, now: float, dt: float) -> None:
        lim = self.limits

        # Sanity check (reuse PowerManager's voltage sanity bounds as floor/ceiling)
        if v_raw < 5.0 or v_raw > 40.0:
            # Don't update source score on garbage readings
            self._check_voltage_staleness(now)
            return

        self._last_valid_voltage_t = now
        self._voltage_stale = False

        # Source inference: slow EMA, only in stable range
        if v_raw > lim.source_supply_vote_v:
            score_target = 1.0
        elif v_raw < lim.source_battery_vote_v:
            score_target = 0.0
        else:
            score_target = None  # gray band — hold current score

        if score_target is not None:
            alpha = dt / lim.source_tau_s
            self._source_score += alpha * (score_target - self._source_score)
            self._source_score = max(0.0, min(1.0, self._source_score))

        # Classify with hysteresis
        if self._source_score > lim.source_supply_threshold:
            self._inferred_source = SourceInference.SUPPLY
        elif self._source_score < lim.source_battery_threshold:
            self._inferred_source = SourceInference.BATTERY
        # else: hold current classification (hysteresis band)

        # Check for prolonged ambiguity → default to battery
        elapsed = (now - self._source_initialized_t) if self._source_initialized_t else 0.0
        if elapsed > lim.source_tau_s * 9.0 and self._inferred_source == SourceInference.UNKNOWN:
            logger.warning(
                "[BusSafety] Source inference ambiguous after %.0fs — defaulting to BATTERY",
                elapsed,
            )
            self._inferred_source = SourceInference.BATTERY

        # Voltage modulation factor
        onset = (
            lim.voltage_supply_onset_v
            if self._inferred_source == SourceInference.SUPPLY
            else lim.voltage_battery_onset_v
        )
        ceiling = lim.voltage_ceiling_v
        recovery_threshold = onset - lim.voltage_hysteresis_v

        if v_raw >= ceiling:
            target = 0.0
            self._v_recovering = False
        elif v_raw >= onset:
            target = 1.0 - (v_raw - onset) / (ceiling - onset)
            self._v_recovering = False
        else:
            # Below onset: only actually recover if below hysteresis band
            if v_raw <= recovery_threshold:
                self._v_recovering = True
            target = 1.0 if self._v_recovering else self._v_factor_target

        self._v_factor_target = max(0.0, min(1.0, target))

    def _check_voltage_staleness(self, now: float) -> None:
        lim = self.limits
        if self._last_valid_voltage_t is not None:
            age = now - self._last_valid_voltage_t
            if age > lim.voltage_stale_timeout_s and not self._voltage_stale:
                self._voltage_stale = True
                logger.warning(
                    "[BusSafety] Bus voltage sensor stale (%.1fs) — applying gentle modulation",
                    age,
                )
        if self._voltage_stale:
            self._v_factor_target = 0.5

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

        if len(self._current_window) < self.limits.current_median_window:
            # Not enough samples yet — no current modulation
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

    def _update_modulation_factor(self, combined_target: float, dt: float) -> None:
        lim = self.limits
        if combined_target < self._modulation_factor:
            step = lim.modulation_ramp_down_per_s * dt
            self._modulation_factor = max(self._modulation_factor - step, combined_target)
        else:
            step = lim.modulation_ramp_up_per_s * dt
            self._modulation_factor = min(self._modulation_factor + step, combined_target)
        self._modulation_factor = max(0.0, min(1.0, self._modulation_factor))

    # ------------------------------------------------------------------
    # Internal — state tracking / logging
    # ------------------------------------------------------------------

    def _update_state_and_log(self, now: float) -> None:
        if self._voltage_stale or self._current_stale:
            new_state = BusSafetyState.SENSOR_STALE
        elif self._modulation_factor < 0.99:
            new_state = BusSafetyState.MODULATING
        else:
            new_state = BusSafetyState.NORMAL

        if new_state != self._state:
            logger.info(
                "[BusSafety] State: %s → %s  factor=%.3f  v_tgt=%.3f  i_tgt=%.3f  source=%s",
                self._state.value,
                new_state.value,
                self._modulation_factor,
                self._v_factor_target,
                self._i_factor_target,
                self._inferred_source.value,
            )
            self._state = new_state

        # Log transitions around key factor levels
        f = self._modulation_factor
        prev = self._last_log_factor
        if prev >= 0.95 and f < 0.95:
            logger.warning(
                "[BusSafety] Modulation engaging: factor=%.3f  v_tgt=%.3f  i_tgt=%.3f  source=%s",
                f, self._v_factor_target, self._i_factor_target, self._inferred_source.value,
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
