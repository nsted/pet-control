"""Unit tests for BusSafetyMonitor.

All tests use synthetic input — no hardware, no async, no I/O.
"""

from __future__ import annotations

from typing import Optional

import pytest

from petctl.bus_safety import BusSafetyMonitor, BusSafetyState, _median
from petctl.config import BusSafetyLimits
from petctl.types import PowerTelemetry, RobotState

# Limits used for tests that need threshold behaviour without real-time waiting.
_FAST = BusSafetyLimits(
    current_median_window=2,
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
        h.advance(seconds=2.0, current_a=9.0)
        assert h.factor < 0.1

    def test_midpoint_partial_modulation(self):
        lim = BusSafetyLimits()
        h = _BSM(limits=lim)
        # onset=6.0A, ceiling=9.0A, midpoint=7.5A → i_factor_target=0.5
        h.advance(seconds=2.0, current_a=7.5)
        assert 0.3 < h.factor < 0.7

    def test_insanity_no_modulation(self):
        """Sanity-rejected current readings must not trigger modulation."""
        h = _BSM(limits=BusSafetyLimits())
        h.advance(seconds=2.0, current_a=99.0)
        assert h.factor > 0.95


# ---------------------------------------------------------------------------
# Ramp rates
# ---------------------------------------------------------------------------

class TestRampRates:
    def test_ramp_down_rate(self):
        """Factor must not decrease faster than ramp_down_per_s per second."""
        lim = BusSafetyLimits(
            modulation_ramp_down_per_s=2.0,
            current_median_window=1,
        )
        h = _BSM(limits=lim)
        h.advance(seconds=0.5)
        initial = h.factor
        assert initial == pytest.approx(1.0, abs=0.01)
        dt = _DT
        h.tick(current_a=9.0, current_raw=9000, dt=dt)
        max_drop = lim.modulation_ramp_down_per_s * dt
        assert (initial - h.factor) <= max_drop + 1e-6

    def test_ramp_up_rate(self):
        """Recovery must not be faster than ramp_up_per_s per second."""
        lim = BusSafetyLimits(
            modulation_ramp_down_per_s=100.0,  # instant down
            modulation_ramp_up_per_s=0.5,
            current_median_window=1,
        )
        h = _BSM(limits=lim)
        h.advance(seconds=0.5)
        h.tick(current_a=9.0, current_raw=9000)
        assert h.factor == pytest.approx(0.0, abs=0.01)
        factor_before = h.factor
        h.tick(current_a=0.5, current_raw=1000, dt=_DT)
        max_rise = lim.modulation_ramp_up_per_s * _DT
        assert (h.factor - factor_before) <= max_rise + 1e-6


# ---------------------------------------------------------------------------
# Hysteresis
# ---------------------------------------------------------------------------

class TestHysteresis:
    def test_current_hysteresis_prevents_early_recovery(self):
        lim = BusSafetyLimits(
            modulation_ramp_down_per_s=100.0,
            modulation_ramp_up_per_s=100.0,
            current_median_window=1,
            current_onset_a=6.0,
            current_hysteresis_a=0.5,
        )
        h = _BSM(limits=lim)
        # Trigger current modulation
        h.tick(current_a=9.0, current_raw=1000)
        assert h.factor < 1.0
        # Drop to 5.7A — below onset (6.0) but above recovery threshold (5.5)
        h.tick(current_a=5.7, current_raw=1001)
        factor_above_recovery = h.factor
        # Drop to 5.3A — below recovery threshold (5.5)
        h.tick(current_a=5.3, current_raw=1002)
        assert h.factor >= factor_above_recovery  # started recovering


# ---------------------------------------------------------------------------
# Sensor staleness
# ---------------------------------------------------------------------------

class TestSensorStaleness:
    def test_stale_current_no_modulation(self):
        """Stale current must NOT trigger current modulation."""
        h = _BSM(limits=_FAST)
        h.tick(current_a=0.5, current_raw=1000)
        for _ in range(6):
            h.tick(current_a=0.5, current_raw=1000, dt=_DT)  # raw unchanged
        assert h.bsm._current_stale
        assert h.bsm._i_factor_target == pytest.approx(1.0, abs=0.01)


# ---------------------------------------------------------------------------
# Boot voltage check
# ---------------------------------------------------------------------------

class TestBootVoltageCheck:
    def test_above_lvc_returns_true(self):
        bsm = BusSafetyMonitor()
        assert bsm.check_boot_voltage(12.0) is True

    def test_at_lvc_boundary_returns_false(self):
        bsm = BusSafetyMonitor()
        lvc = bsm.limits.voltage_lvc_v
        assert bsm.check_boot_voltage(lvc - 0.1) is False

    def test_exactly_at_lvc_returns_true(self):
        bsm = BusSafetyMonitor()
        lvc = bsm.limits.voltage_lvc_v
        assert bsm.check_boot_voltage(lvc) is True

    def test_out_of_range_does_not_block(self):
        """Bad ADC reading should not block startup."""
        bsm = BusSafetyMonitor()
        assert bsm.check_boot_voltage(0.0) is True
        assert bsm.check_boot_voltage(50.0) is True


# ---------------------------------------------------------------------------
# Idle voltage check
# ---------------------------------------------------------------------------

class TestIdleVoltageCheck:
    def test_not_sampled_when_current_too_high(self):
        """No idle sample when current >= threshold."""
        bsm = BusSafetyMonitor()
        t = 1000.0
        bsm.update_idle_voltage_check(12.0, current_a=1.0, now=t)
        assert bsm._last_idle_voltage_check_t is None

    def test_sampled_when_current_low(self):
        bsm = BusSafetyMonitor()
        t = 1000.0
        bsm.update_idle_voltage_check(12.0, current_a=0.1, now=t)
        assert bsm._last_idle_voltage_check_t == t

    def test_rate_gated(self):
        """Second call within interval must not re-sample."""
        bsm = BusSafetyMonitor()
        t = 1000.0
        bsm.update_idle_voltage_check(12.0, current_a=0.1, now=t)
        bsm.update_idle_voltage_check(12.0, current_a=0.1, now=t + 1.0)
        assert bsm._last_idle_voltage_check_t == t  # unchanged

    def test_samples_again_after_interval(self):
        bsm = BusSafetyMonitor()
        lim = bsm.limits
        t = 1000.0
        bsm.update_idle_voltage_check(12.0, current_a=0.1, now=t)
        t2 = t + lim.idle_voltage_check_interval_s + 1.0
        bsm.update_idle_voltage_check(12.0, current_a=0.1, now=t2)
        assert bsm._last_idle_voltage_check_t == t2

    def test_out_of_range_skipped(self):
        bsm = BusSafetyMonitor()
        bsm.update_idle_voltage_check(0.0, current_a=0.1, now=1000.0)
        assert bsm._last_idle_voltage_check_t is None


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

    def test_current_filtered_populated_after_warmup(self):
        """After enough ticks to fill median window, bus_current_filtered_a is set."""
        lim = BusSafetyLimits(current_median_window=2)
        h = _BSM(limits=lim)
        h.tick(current_a=1.5, current_raw=100)
        h.tick(current_a=2.5, current_raw=101)
        telem = PowerTelemetry()
        h.bsm.update_telemetry(telem)
        assert telem.bus_current_filtered_a == pytest.approx(2.0, abs=0.01)
