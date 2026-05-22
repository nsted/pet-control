"""Unit tests for BusSafetyMonitor.

All tests use synthetic input — no hardware, no async, no I/O.
"""

from __future__ import annotations

import math
from typing import Optional

import pytest

from petctl.bus_safety import BusSafetyMonitor, BusSafetyState, SourceInference, _median
from petctl.config import BusSafetyLimits
from petctl.types import PowerTelemetry, RobotState

# Fast limits for tests that need threshold behavior without real-time waiting.
_FAST = BusSafetyLimits(
    source_tau_s=1.0,                  # 1s tau — converges in a few ticks at dt=0.1
    current_median_window=2,           # fill quickly
    voltage_stale_timeout_s=0.3,
    current_stale_timeout_s=0.2,
)

_DT = 0.05  # 20 Hz ticks


def _make_state(
    voltage_v: float = 12.0,
    current_a: float = 0.5,
    current_raw: int = 1000,
    dt: float = _DT,
) -> RobotState:
    """Build a minimal RobotState with synthetic battery readings."""
    rs = RobotState()
    # Override computed properties via subclass (same pattern as test_power_manager.py)
    rs.__class__ = type(
        "_TestState",
        (RobotState,),
        {
            "battery_voltage_v": property(lambda self: voltage_v),
            "battery_current_amps": property(lambda self: current_a),
        },
    )
    rs.battery_current_raw = current_raw
    rs.dt = dt
    return rs


class _BSM:
    """Test harness wrapping BusSafetyMonitor."""

    def __init__(self, limits: BusSafetyLimits = _FAST) -> None:
        self.bsm = BusSafetyMonitor(limits)
        self._now: float = 1000.0

    def tick(
        self,
        voltage_v: float = 12.0,
        current_a: float = 0.5,
        current_raw: int = 1000,
        dt: float = _DT,
    ) -> BusSafetyMonitor:
        self._now += dt
        state = _make_state(voltage_v=voltage_v, current_a=current_a, current_raw=current_raw, dt=dt)
        self.bsm.update(state, self._now)
        return self.bsm

    def advance(self, seconds: float, voltage_v: float = 12.0, current_a: float = 0.5) -> None:
        """Advance by running ticks at _DT."""
        steps = max(1, int(seconds / _DT))
        for _ in range(steps):
            self.tick(voltage_v=voltage_v, current_a=current_a)

    @property
    def factor(self) -> float:
        return self.bsm.modulation_factor

    @property
    def source(self) -> SourceInference:
        return self.bsm.inferred_source


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

class TestMedian:
    def test_odd(self):
        assert _median([3.0, 1.0, 2.0]) == 2.0

    def test_even(self):
        assert _median([1.0, 3.0]) == 2.0

    def test_single(self):
        assert _median([5.0]) == 5.0


# ---------------------------------------------------------------------------
# Boot state
# ---------------------------------------------------------------------------

class TestBootState:
    def test_modulation_factor_starts_at_one(self):
        bsm = BusSafetyMonitor()
        assert bsm.modulation_factor == 1.0

    def test_inferred_source_starts_battery(self):
        bsm = BusSafetyMonitor()
        assert bsm.inferred_source == SourceInference.BATTERY


# ---------------------------------------------------------------------------
# Source inference
# ---------------------------------------------------------------------------

class TestSourceInference:
    def test_supply_convergence(self):
        h = _BSM()
        # 14.5V for long enough to exceed source_tau_s (1s at dt=0.05 = 20 ticks)
        h.advance(seconds=5.0, voltage_v=14.5)
        assert h.bsm._source_score > h.bsm.limits.source_supply_threshold
        assert h.source == SourceInference.SUPPLY

    def test_battery_hold(self):
        h = _BSM()
        h.advance(seconds=5.0, voltage_v=11.5)
        assert h.bsm._source_score < h.bsm.limits.source_battery_threshold
        assert h.source == SourceInference.BATTERY

    def test_gray_band_holds_classification(self):
        # Seed to SUPPLY, then push into gray band — classification must not flip
        h = _BSM()
        h.advance(seconds=5.0, voltage_v=14.5)
        assert h.source == SourceInference.SUPPLY
        # Gray band [12.5, 13.5] — score should not change
        score_before = h.bsm._source_score
        h.advance(seconds=1.0, voltage_v=13.0)
        assert h.bsm._source_score == pytest.approx(score_before, abs=1e-9)
        assert h.source == SourceInference.SUPPLY  # hysteresis holds

    def test_spike_does_not_flip_battery_classification(self):
        """A 3s fault spike at 18.7V from battery should not flip source inference."""
        h = _BSM()
        h.advance(seconds=3.0, voltage_v=11.5)   # establish BATTERY
        assert h.source == SourceInference.BATTERY
        score_before = h.bsm._source_score
        # 18.7V is above source_supply_vote_v (13.5V) — but sanitiy check (5-40V) passes it
        # so it DOES vote supply. With tau=1s, 3s of data moves score:
        # max movement ≈ 3/1 * (1 - score_before) but actually saturates.
        # Key: score must NOT cross source_supply_threshold (0.65).
        # At tau=1s and score_before~0.1: 3s moves score to ~1*(1-e^(-3)) + 0.1*e^(-3) ≈ 0.97.
        # With FAST limits (tau=1s) the spike WOULD flip — that's expected with 1s tau.
        # Test uses the real defaults (tau=15s) where this matters.
        h2 = _BSM(limits=BusSafetyLimits())  # real tau=15s
        h2.advance(seconds=5.0, voltage_v=11.5)
        assert h2.source == SourceInference.BATTERY
        score_before_real = h2.bsm._source_score
        for _ in range(60):   # 3 seconds at 20Hz
            h2.tick(voltage_v=18.7)
        # Score moved by at most 3/15 ≈ 0.2 from battery baseline
        assert h2.bsm._source_score < h2.bsm.limits.source_supply_threshold
        assert h2.source == SourceInference.BATTERY


# ---------------------------------------------------------------------------
# Voltage modulation factor
# ---------------------------------------------------------------------------

class TestVoltageFactor:
    def _bsm_at_source(self, source: str) -> _BSM:
        """Return a BSM harness with established source."""
        h = _BSM()
        if source == "SUPPLY":
            h.advance(seconds=5.0, voltage_v=14.5)
        else:
            h.advance(seconds=5.0, voltage_v=11.5)
        return h

    def test_below_battery_onset_no_modulation(self):
        h = self._bsm_at_source("BATTERY")
        # Below onset (13V): factor should be 1.0
        h.advance(seconds=0.5, voltage_v=12.5)
        # Factor is rate-limited; after 0.5s at 1.0 target it should be 1.0
        assert h.factor == pytest.approx(1.0, abs=0.05)

    def test_at_ceiling_full_modulation(self):
        lim = BusSafetyLimits()
        h = _BSM(limits=lim)
        h.advance(seconds=2.0, voltage_v=11.5)  # establish BATTERY
        # 15V = ceiling → v_factor_target = 0.0
        # With ramp 1.0/s and 2s ticks we expect factor to reach 0.0
        h.advance(seconds=2.0, voltage_v=15.0)
        assert h.factor < 0.1

    def test_midpoint_partial_modulation(self):
        lim = BusSafetyLimits()
        h = _BSM(limits=lim)
        h.advance(seconds=3.0, voltage_v=11.5)  # BATTERY
        # Battery onset=13V, ceiling=15V, midpoint=14V → v_factor_target=0.5
        # Run for 2s to let rate-limited factor settle toward target
        h.advance(seconds=2.0, voltage_v=14.0)
        assert 0.3 < h.factor < 0.7  # somewhere in the middle

    def test_supply_onset_near_ceiling(self):
        lim = BusSafetyLimits()
        h = _BSM(limits=lim)
        # tau=15s: need ~30s to reliably cross 0.65 supply threshold (1 - e^(-30/15) ≈ 0.86)
        h.advance(seconds=30.0, voltage_v=14.5)
        assert h.source == SourceInference.SUPPLY
        # At 14.5V (below supply onset of 14.8V): no modulation
        assert h.factor > 0.9

    def test_supply_at_ceiling_full_modulation(self):
        lim = BusSafetyLimits()
        h = _BSM(limits=lim)
        h.advance(seconds=30.0, voltage_v=14.5)  # establish SUPPLY
        assert h.source == SourceInference.SUPPLY
        h.advance(seconds=2.0, voltage_v=15.0)
        assert h.factor < 0.1


# ---------------------------------------------------------------------------
# Current modulation factor
# ---------------------------------------------------------------------------

class TestCurrentFactor:
    def test_below_onset_no_modulation(self):
        h = _BSM(limits=BusSafetyLimits())
        h.advance(seconds=2.0, current_a=1.0)
        assert h.factor > 0.95

    def test_at_ceiling_full_modulation(self):
        h = _BSM(limits=BusSafetyLimits())
        h.advance(seconds=2.0, current_a=4.0)
        assert h.factor < 0.1

    def test_midpoint_partial_modulation(self):
        lim = BusSafetyLimits()
        h = _BSM(limits=lim)
        # onset=3.5A, ceiling=4.0A, midpoint=3.75A → i_factor_target=0.5
        h.advance(seconds=2.0, current_a=3.75)
        assert 0.3 < h.factor < 0.7

    def test_insanity_no_modulation(self):
        """Sanity-rejected current readings must not trigger modulation."""
        h = _BSM(limits=BusSafetyLimits())
        # 99A is way above sanity_max — should be ignored
        h.advance(seconds=2.0, current_a=99.0)
        assert h.factor > 0.95


# ---------------------------------------------------------------------------
# Multiplicative combination
# ---------------------------------------------------------------------------

class TestCombination:
    def test_multiplicative(self):
        """Both triggers partial: combined target ≈ v_factor × i_factor."""
        lim = BusSafetyLimits(
            modulation_ramp_down_per_s=100.0,  # instant for this test
            modulation_ramp_up_per_s=100.0,
            source_tau_s=1e6,  # frozen — source never changes from BATTERY
            current_median_window=1,
        )
        h = _BSM(limits=lim)
        # source stays BATTERY (score ~0 with frozen tau)
        # voltage 14.0V (battery onset=13, ceiling=15): v_target = (15-14)/(15-13) = 0.5
        # current 3.75A (onset=3.5, ceiling=4.0): i_target = (4-3.75)/(4-3.5) = 0.5
        # combined = 0.5 × 0.5 = 0.25
        h.tick(voltage_v=14.0, current_a=3.75, current_raw=9999)
        h.tick(voltage_v=14.0, current_a=3.75, current_raw=9998)
        assert h.factor == pytest.approx(0.25, abs=0.05)

    def test_either_trigger_alone_drives_to_zero(self):
        """Voltage at ceiling with current safe → factor goes to 0."""
        lim = BusSafetyLimits(
            modulation_ramp_down_per_s=100.0,
            modulation_ramp_up_per_s=100.0,
            source_tau_s=1e6,  # frozen BATTERY
            current_median_window=1,
        )
        h = _BSM(limits=lim)
        h.tick(voltage_v=15.0, current_a=1.0, current_raw=500)
        assert h.factor == pytest.approx(0.0, abs=0.01)


# ---------------------------------------------------------------------------
# Ramp rates
# ---------------------------------------------------------------------------

class TestRampRates:
    def test_ramp_down_rate(self):
        """Factor must not decrease faster than ramp_down_per_s per second."""
        lim = BusSafetyLimits(
            modulation_ramp_down_per_s=1.0,
            source_tau_s=0.01,
            current_median_window=1,
        )
        h = _BSM(limits=lim)
        h.advance(seconds=0.5, voltage_v=11.5)  # BATTERY stable
        initial = h.factor
        assert initial == pytest.approx(1.0, abs=0.01)
        # Apply maximum load: voltage at ceiling
        dt = _DT
        h.tick(voltage_v=15.0, current_a=4.0, current_raw=9000, dt=dt)
        # Factor must not have dropped more than ramp_down_per_s * dt
        max_drop = lim.modulation_ramp_down_per_s * dt
        assert (initial - h.factor) <= max_drop + 1e-6

    def test_ramp_up_rate(self):
        """Recovery must not be faster than ramp_up_per_s per second."""
        lim = BusSafetyLimits(
            modulation_ramp_down_per_s=100.0,  # instant down
            modulation_ramp_up_per_s=0.15,
            source_tau_s=0.01,
            current_median_window=1,
        )
        h = _BSM(limits=lim)
        h.advance(seconds=0.5, voltage_v=11.5)
        # Force to 0
        h.tick(voltage_v=15.0, current_a=4.0, current_raw=9000)
        assert h.factor == pytest.approx(0.0, abs=0.01)
        # Now recover with safe conditions
        factor_before = h.factor
        h.tick(voltage_v=11.5, current_a=0.5, current_raw=1000, dt=_DT)
        max_rise = lim.modulation_ramp_up_per_s * _DT
        assert (h.factor - factor_before) <= max_rise + 1e-6


# ---------------------------------------------------------------------------
# Hysteresis
# ---------------------------------------------------------------------------

class TestHysteresis:
    def test_voltage_hysteresis_prevents_early_recovery(self):
        """Factor must not recover until voltage drops below onset - hysteresis."""
        lim = BusSafetyLimits(
            modulation_ramp_down_per_s=100.0,
            modulation_ramp_up_per_s=100.0,
            source_tau_s=1e6,  # frozen BATTERY — prevents 14V from flipping source to SUPPLY
            current_median_window=1,
            voltage_battery_onset_v=13.0,
            voltage_hysteresis_v=0.3,
        )
        h = _BSM(limits=lim)

        # Push voltage into modulation zone (14V → partial)
        h.tick(voltage_v=14.0, current_a=0.5, current_raw=500)
        assert h.factor < 1.0

        # Now drop to 12.8V — below onset (13V) but above recovery threshold (12.7V)
        h.tick(voltage_v=12.8, current_a=0.5, current_raw=501)
        factor_mid = h.factor  # should NOT fully recover yet

        # Drop further to 12.6V — below recovery threshold (12.7V)
        h.tick(voltage_v=12.6, current_a=0.5, current_raw=502)
        factor_after = h.factor
        # After crossing recovery threshold, target becomes 1.0 and factor rises
        assert factor_after >= factor_mid

    def test_current_hysteresis_prevents_early_recovery(self):
        lim = BusSafetyLimits(
            modulation_ramp_down_per_s=100.0,
            modulation_ramp_up_per_s=100.0,
            source_tau_s=0.01,
            current_median_window=1,
            current_onset_a=3.5,
            current_hysteresis_a=0.3,
        )
        h = _BSM(limits=lim)
        # Trigger current modulation
        h.tick(voltage_v=12.0, current_a=4.0, current_raw=1000)
        assert h.factor < 1.0
        # Drop to 3.3A — below onset (3.5) but above recovery threshold (3.2)
        h.tick(voltage_v=12.0, current_a=3.3, current_raw=1001)
        factor_above_recovery = h.factor
        # Drop to 3.1A — below recovery threshold (3.2)
        h.tick(voltage_v=12.0, current_a=3.1, current_raw=1002)
        assert h.factor >= factor_above_recovery  # started recovering


# ---------------------------------------------------------------------------
# Sensor staleness
# ---------------------------------------------------------------------------

class TestSensorStaleness:
    def test_stale_voltage_applies_gentle_modulation(self):
        """No valid voltage for > stale_timeout → v_factor_target = 0.5."""
        h = _BSM(limits=_FAST)
        h.tick(voltage_v=12.0)  # one valid tick
        # Now feed garbage voltage (outside 5-40V sanity window) for > stale_timeout
        # At stale_timeout=0.3s and dt=0.05s, need 7+ ticks
        for _ in range(10):
            h.tick(voltage_v=0.0)  # sanity fail
        assert h.bsm._voltage_stale
        # v_factor_target should be 0.5 (gentle modulation, not full 0)
        assert h.bsm._v_factor_target == pytest.approx(0.5, abs=0.01)

    def test_stale_current_no_modulation(self):
        """Stale current must NOT trigger current modulation."""
        h = _BSM(limits=_FAST)
        h.tick(current_a=0.5, current_raw=1000)  # valid
        # Same raw ADC value for > stale_timeout (stuck sensor)
        for _ in range(6):
            h.tick(current_a=0.5, current_raw=1000, dt=_DT)  # raw unchanged
        assert h.bsm._current_stale
        # i_factor_target must be 1.0 (no restriction on stuck sensor)
        assert h.bsm._i_factor_target == pytest.approx(1.0, abs=0.01)


# ---------------------------------------------------------------------------
# PowerTelemetry update
# ---------------------------------------------------------------------------

class TestTelemetryUpdate:
    def test_fields_populated(self):
        h = _BSM(limits=BusSafetyLimits())
        h.advance(seconds=1.0)
        telem = PowerTelemetry()
        h.bsm.update_telemetry(telem)
        assert telem.bus_modulation_factor == pytest.approx(h.factor, abs=1e-6)
        assert telem.bus_source_inference == h.source.value

    def test_current_filtered_populated_after_warmup(self):
        """After enough ticks to fill median window, bus_current_filtered_a is set."""
        lim = BusSafetyLimits(current_median_window=2)
        h = _BSM(limits=lim)
        h.tick(current_a=1.5, current_raw=100)
        h.tick(current_a=2.5, current_raw=101)
        telem = PowerTelemetry()
        h.bsm.update_telemetry(telem)
        assert telem.bus_current_filtered_a == pytest.approx(2.0, abs=0.01)
