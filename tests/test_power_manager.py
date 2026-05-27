"""Unit tests for PowerManager state machines.

All tests use synthetic input streams — no hardware, no async, no I/O.
"""

from __future__ import annotations

import time
from typing import Optional

import pytest

from petctl.power_manager import (
    MotorThermalState,
    PowerManager,
    PowerThresholds,
    SystemState,
    VoltageState,
)
from petctl.types import RobotState

# Motor IDs used in tests
M1, M2 = 1, 2

# Tight thresholds so tests run without real wall-clock waits
_FAST_THRESHOLDS = PowerThresholds(
    temp_hysteresis_cooldown_s=0.05,  # 50ms — fast hysteresis for tests
)


def _state(
    *,
    motor_ids: list[int] | None = None,
    drive_temps: dict[int, int] | None = None,
    winding_temps: dict[int, int] | None = None,
    err_codes: dict[int, int] | None = None,
    motor_torques: dict[int, float] | None = None,
    voltage_v: float = 14.5,
    current_a: float = 0.0,
) -> RobotState:
    """Build a minimal RobotState for testing."""
    ids = motor_ids or []
    rs = RobotState(
        active_servo_ids=set(ids),
        motor_temperatures=drive_temps or {},
        motor_winding_temperatures=winding_temps or {},
        motor_err_codes=err_codes or {},
        motor_torques=motor_torques or {},
    )
    # Override computed properties for testing
    rs.__class__ = type(
        "_TestRobotState",
        (RobotState,),
        {
            "battery_voltage_v": property(lambda self: voltage_v),
            "battery_current_amps": property(lambda self: current_a),
        },
    )
    return rs


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

class _PM:
    """Thin wrapper to make tests more readable."""

    def __init__(self, thresholds: PowerThresholds = _FAST_THRESHOLDS) -> None:
        self.pm = PowerManager(thresholds)
        self._now = 1000.0  # arbitrary start time

    def tick(self, state: RobotState, dt: float = 0.02) -> PowerManager:
        self._now += dt
        self.pm.update(state, self._now)
        return self.pm

    def drain(self) -> tuple[list[int], bool]:
        return self.pm.drain_disable_events()

    @property
    def now(self) -> float:
        return self._now

    def advance(self, seconds: float) -> None:
        self._now += seconds


# ---------------------------------------------------------------------------
# Parser: _byte_to_int8
# ---------------------------------------------------------------------------

def test_byte_to_int8_positive() -> None:
    from petctl.backends.robot import _byte_to_int8
    assert _byte_to_int8(25) == 25
    assert _byte_to_int8(127) == 127
    assert _byte_to_int8(0) == 0


def test_byte_to_int8_negative() -> None:
    from petctl.backends.robot import _byte_to_int8
    assert _byte_to_int8(255) == -1
    assert _byte_to_int8(128) == -128
    assert _byte_to_int8(246) == -10


def test_byte_to_int8_boundary() -> None:
    from petctl.backends.robot import _byte_to_int8
    assert _byte_to_int8(127) == 127
    assert _byte_to_int8(128) == -128


# ---------------------------------------------------------------------------
# Parser: _handle_slcan_frame ERR field
# ---------------------------------------------------------------------------

def _mit_frame(byte0: int, drive_temp: int = 0, motor_temp: int = 0) -> str:
    """Build an 8-byte MIT reply SLCAN frame with centre pos/vel/torque."""
    # Byte layout: byte0 | pos(2) | vel+torque(3) | drive_temp | motor_temp
    # 0x8000 = position midpoint, 0x800000 = vel+torque at midpoint
    payload = f"{byte0:02X}8000800000{drive_temp & 0xFF:02X}{motor_temp & 0xFF:02X}"
    assert len(payload) == 16, f"bad payload length {len(payload)}"
    return f"t0008{payload}"


def test_slcan_err_nibble_extracted() -> None:
    from petctl.backends.robot import RobotBackend
    b = RobotBackend()
    # ERR=0xB, motor_id=1 → byte0 = 0xB1
    b._handle_slcan_frame(_mit_frame(0xB1))
    assert b._motor_state[1]["err_code"] == 0xB


def test_slcan_temperatures_signed() -> None:
    from petctl.backends.robot import RobotBackend
    b = RobotBackend()
    # drive_temp = 0xF6 = -10°C, motor_temp = 0x37 = 55°C, motor_id=1
    b._handle_slcan_frame(_mit_frame(0x01, drive_temp=0xF6, motor_temp=0x37))
    assert b._motor_state[1]["drive_temp"] == -10
    assert b._motor_state[1]["motor_temp"] == 55


def test_slcan_normal_enable_code() -> None:
    from petctl.backends.robot import RobotBackend
    b = RobotBackend()
    # ERR nibble = 0x1 (Enable), motor_id = 2 → byte0 = 0x12
    b._handle_slcan_frame(_mit_frame(0x12, drive_temp=0x1E, motor_temp=0x1E))
    assert b._motor_state[2]["err_code"] == 0x1
    assert b._motor_state[2]["drive_temp"] == 30
    assert b._motor_state[2]["motor_temp"] == 30


# ---------------------------------------------------------------------------
# Thermal state machine
# ---------------------------------------------------------------------------

class TestThermalWarning:
    def test_normal_below_threshold(self) -> None:
        w = _PM()
        st = _state(motor_ids=[M1], drive_temps={M1: 40})
        w.tick(st)
        assert w.pm.get_compliance_scale(M1) == 1.0
        assert w.pm.is_motor_enabled(M1)

    def test_soft_warning_fires(self) -> None:
        w = _PM()
        st = _state(motor_ids=[M1], drive_temps={M1: 58})
        w.tick(st)
        assert w.pm.get_compliance_scale(M1) == 0.5
        assert w.pm.is_motor_enabled(M1)

    def test_compliance_uses_max_of_drive_and_winding(self) -> None:
        """Winding temp above threshold should trigger warning even if drive is cool."""
        w = _PM()
        st = _state(motor_ids=[M1], drive_temps={M1: 40}, winding_temps={M1: 60})
        w.tick(st)
        assert w.pm.get_compliance_scale(M1) == 0.5

    def test_warning_recovers_when_temp_drops(self) -> None:
        w = _PM()
        w.tick(_state(motor_ids=[M1], drive_temps={M1: 58}))
        assert w.pm.get_compliance_scale(M1) == 0.5
        w.tick(_state(motor_ids=[M1], drive_temps={M1: 40}))
        assert w.pm.get_compliance_scale(M1) == 1.0


class TestThermalCutoff:
    def test_hard_cutoff_disables_motor(self) -> None:
        w = _PM()
        st = _state(motor_ids=[M1], drive_temps={M1: 67})
        w.tick(st)
        disable_ids, is_global = w.drain()
        assert M1 in disable_ids
        assert not is_global
        assert not w.pm.is_motor_enabled(M1)
        assert w.pm.get_compliance_scale(M1) == 0.0

    def test_disabled_motor_stays_disabled_on_subsequent_ticks(self) -> None:
        w = _PM()
        w.tick(_state(motor_ids=[M1], drive_temps={M1: 67}))
        w.drain()
        # Cool the motor but no reset yet
        for _ in range(5):
            w.tick(_state(motor_ids=[M1], drive_temps={M1: 30}))
        assert not w.pm.is_motor_enabled(M1)

    def test_second_motor_unaffected_by_first_disable(self) -> None:
        w = _PM()
        w.tick(_state(motor_ids=[M1, M2], drive_temps={M1: 67, M2: 30}))
        w.drain()
        assert not w.pm.is_motor_enabled(M1)
        assert w.pm.is_motor_enabled(M2)

    def test_err_overtemp_immediate_disable(self) -> None:
        """ERR=0xB must disable motor regardless of temperature reading."""
        w = _PM()
        st = _state(motor_ids=[M1], drive_temps={M1: 40}, err_codes={M1: 0xB})
        w.tick(st)
        disable_ids, _ = w.drain()
        assert M1 in disable_ids
        assert not w.pm.is_motor_enabled(M1)

    def test_err_c_immediate_disable(self) -> None:
        w = _PM()
        st = _state(motor_ids=[M1], err_codes={M1: 0xC})
        w.tick(st)
        disable_ids, _ = w.drain()
        assert M1 in disable_ids


class TestThermalGlobalEmergency:
    def test_global_emergency_on_75c(self) -> None:
        w = _PM()
        st = _state(motor_ids=[M1, M2], drive_temps={M1: 76, M2: 30})
        w.tick(st)
        _, is_global = w.drain()
        assert is_global
        assert not w.pm.is_motor_enabled(M1)
        assert not w.pm.is_motor_enabled(M2)
        assert w.pm._system_state == SystemState.EMERGENCY_STOPPED

    def test_emergency_frozen_until_reset(self) -> None:
        w = _PM()
        w.tick(_state(motor_ids=[M1], drive_temps={M1: 76}))
        w.drain()
        # Even after many cool ticks, system stays stopped
        for _ in range(10):
            w.tick(_state(motor_ids=[M1], drive_temps={M1: 20}))
        assert w.pm._system_state == SystemState.EMERGENCY_STOPPED


class TestHysteresisRecovery:
    def test_reset_denied_before_cooldown(self) -> None:
        w = _PM()
        w.tick(_state(motor_ids=[M1], drive_temps={M1: 67}))
        w.drain()
        # Cool motor below threshold (sets cool_since = now), but try reset at the same
        # tick time — 0 seconds elapsed since cooling started, well below cooldown window.
        w.tick(_state(motor_ids=[M1], drive_temps={M1: 40}))
        result = w.pm.operator_reset(w.now)
        assert not result
        assert not w.pm.is_motor_enabled(M1)

    def test_reset_succeeds_after_cooldown(self) -> None:
        w = _PM()
        w.tick(_state(motor_ids=[M1], drive_temps={M1: 67}))
        w.drain()
        # Cool motor below recovery threshold
        for _ in range(5):
            w.tick(_state(motor_ids=[M1], drive_temps={M1: 40}))
        # Advance fake clock past cooldown period
        w.advance(_FAST_THRESHOLDS.temp_hysteresis_cooldown_s + 0.01)
        result = w.pm.operator_reset(w.now)
        assert result
        assert w.pm.is_motor_enabled(M1)
        assert w.pm.get_compliance_scale(M1) == 1.0

    def test_reset_denied_if_still_hot(self) -> None:
        w = _PM()
        w.tick(_state(motor_ids=[M1], drive_temps={M1: 67}))
        w.drain()
        # Don't cool below recovery threshold
        for _ in range(5):
            w.tick(_state(motor_ids=[M1], drive_temps={M1: 52}))
        w.advance(_FAST_THRESHOLDS.temp_hysteresis_cooldown_s + 0.1)
        result = w.pm.operator_reset(w.now)
        assert not result

    def test_reset_denied_after_recent_err_overtemp(self) -> None:
        w = _PM()
        w.tick(_state(motor_ids=[M1], err_codes={M1: 0xB}))
        w.drain()
        # Cool motor below recovery threshold
        for _ in range(5):
            w.tick(_state(motor_ids=[M1], drive_temps={M1: 40}))
        # Advance past the cooldown window from when ERR was seen
        w.advance(_FAST_THRESHOLDS.temp_hysteresis_cooldown_s + 0.01)
        result = w.pm.operator_reset(w.now)
        assert result


# ---------------------------------------------------------------------------
# Voltage EMA (display only — no safety actions)
# ---------------------------------------------------------------------------

class TestVoltageEMA:
    def test_insane_reading_discarded(self) -> None:
        """Readings outside sanity bounds are silently dropped — EMA stays None."""
        w = _PM()
        for _ in range(10):
            w.tick(_state(voltage_v=50.0))
        assert w.pm._voltage_ema is None
        assert w.pm._system_state == SystemState.RUNNING

    def test_negative_voltage_discarded(self) -> None:
        w = _PM()
        for _ in range(10):
            w.tick(_state(voltage_v=-1.0))
        assert w.pm._voltage_ema is None

    def test_ema_seeds_on_first_valid_sample(self) -> None:
        w = _PM()
        w.tick(_state(voltage_v=14.5))
        assert w.pm._voltage_ema == pytest.approx(14.5)

    def test_ema_smooths_spike(self) -> None:
        """A single high-voltage reading should barely move the heavy EMA."""
        w = _PM()
        for _ in range(100):
            w.tick(_state(voltage_v=12.0))
        w.tick(_state(voltage_v=20.0))
        # alpha=0.02: 0.98*12 + 0.02*20 = 11.76 + 0.40 = 12.16
        assert w.pm._voltage_ema is not None
        assert w.pm._voltage_ema < 12.5
        assert w.pm._system_state == SystemState.RUNNING  # no emergency from voltage

    def test_high_voltage_never_triggers_emergency(self) -> None:
        """Voltage spikes — even extreme ones — must not trigger emergency stop."""
        w = _PM()
        for _ in range(20):
            w.tick(_state(voltage_v=30.0))
        assert w.pm._system_state == SystemState.RUNNING

    def test_low_warning_fires_after_ema_settles(self) -> None:
        """LOW_WARNING is informational only; fires once EMA drops below threshold."""
        w = _PM()
        for _ in range(200):
            w.tick(_state(voltage_v=10.0))
        assert w.pm._voltage_state == VoltageState.LOW_WARNING
        assert w.pm._system_state == SystemState.RUNNING  # warning, not emergency

    def test_normal_voltage_no_action(self) -> None:
        w = _PM()
        for _ in range(50):
            w.tick(_state(voltage_v=14.5))
        assert w.pm._system_state == SystemState.RUNNING
        assert w.pm._voltage_state == VoltageState.NORMAL


# ---------------------------------------------------------------------------
# Current-based compliance limiting
# ---------------------------------------------------------------------------

class TestCurrentLimiting:
    def test_below_threshold_scale_is_one(self) -> None:
        """Current well below safety limit (3.5A) → safety drive scale unaffected."""
        w = _PM()
        for _ in range(100):
            w.tick(_state(motor_ids=[M1], current_a=1.0))
        # Safety backstop is unaffected at 1A; budget targeting may boost above 1.0
        assert w.pm._current_drive_scale == pytest.approx(1.0)

    def test_above_limit_scale_is_zero(self) -> None:
        """Current saturated above limit → compliance zeroed."""
        w = _PM()
        for _ in range(100):
            w.tick(_state(motor_ids=[M1], current_a=5.0))
        assert w.pm._current_drive_scale == pytest.approx(0.0, abs=0.01)
        assert w.pm.get_compliance_scale(M1) == pytest.approx(0.0, abs=0.01)

    def test_midpoint_scale(self) -> None:
        """At 3.75A (midpoint of 3.5–4.0 range) → scale ≈ 0.5."""
        w = _PM()
        for _ in range(100):
            w.tick(_state(current_a=3.75))
        assert w.pm._current_drive_scale == pytest.approx(0.5, abs=0.02)

    def test_ema_softens_brief_current_spike(self) -> None:
        """A single-tick current spike should not immediately zero compliance."""
        w = _PM()
        for _ in range(50):
            w.tick(_state(current_a=1.0))
        w.tick(_state(current_a=10.0))
        # EMA: 0.1*10 + 0.9*~1.0 ≈ 1.9A — still well below 3.5A
        assert w.pm._current_drive_scale == pytest.approx(1.0)

    def test_current_scale_multiplied_with_thermal(self) -> None:
        """Compliance = thermal_scale × safety_current_scale × budget_scale."""
        t = PowerThresholds(
            temp_hysteresis_cooldown_s=0.05,
            current_ema_alpha=1.0,    # instant EMA for precise test
            current_target_cut_rate=0.5,
        )
        w = _PM(t)
        # Thermal warning (0.5) × safety midpoint (0.5) × budget cut (0.5 after 1 tick at 3.75A)
        # Budget: desired=1/3.75=0.267, rate-limited to max(1.0-0.5, 0.267)=0.5 → budget=0.5
        # Final: 0.5 × 0.5 × 0.5 = 0.125
        w.tick(_state(motor_ids=[M1], drive_temps={M1: 58}, current_a=3.75))
        assert w.pm.get_compliance_scale(M1) == pytest.approx(0.125, abs=0.01)


# ---------------------------------------------------------------------------
# Telemetry
# ---------------------------------------------------------------------------

class TestTelemetry:
    def test_telemetry_contains_motor_state(self) -> None:
        w = _PM()
        w.tick(_state(motor_ids=[M1], drive_temps={M1: 58}))
        t = w.pm.get_telemetry(14.5)
        assert t.motor_states.get(M1) == "WARNING"
        assert t.motor_compliance_scales.get(M1) == pytest.approx(0.5)

    def test_telemetry_drains_events(self) -> None:
        w = _PM()
        w.tick(_state(motor_ids=[M1], drive_temps={M1: 58}))
        t1 = w.pm.get_telemetry(14.5)
        assert len(t1.events) > 0
        t2 = w.pm.get_telemetry(14.5)
        assert len(t2.events) == 0  # drained on first call

    def test_telemetry_current_fields(self) -> None:
        w = _PM()
        w.tick(_state(current_a=2.0))
        t = w.pm.get_telemetry(14.5)
        assert t.current_amps_raw == pytest.approx(2.0)
        assert t.current_amps_filtered == pytest.approx(0.2, abs=0.01)  # 0.1 * 2.0
        assert t.current_drive_scale == pytest.approx(1.0)

    def test_telemetry_current_scale_saturated(self) -> None:
        w = _PM()
        for _ in range(100):
            w.tick(_state(current_a=5.0))
        t = w.pm.get_telemetry(14.5)
        assert t.current_drive_scale == pytest.approx(0.0, abs=0.01)

    def test_telemetry_voltage_ema(self) -> None:
        w = _PM()
        w.tick(_state(voltage_v=14.5))
        t = w.pm.get_telemetry(14.5)
        assert t.voltage_ema_v == pytest.approx(14.5)

    def test_telemetry_system_state(self) -> None:
        w = _PM()
        t = w.pm.get_telemetry(14.5)
        assert t.system_state == "RUNNING"


# ---------------------------------------------------------------------------
# Current budget targeting
# ---------------------------------------------------------------------------

# Instant EMA so tests can reason about exact current values.
_TARGET_THRESHOLDS = PowerThresholds(
    temp_hysteresis_cooldown_s=0.05,
    current_ema_alpha=1.0,
    current_target_a=1.0,
    current_target_boost_max=1.5,
    current_target_boost_rate=0.02,
    current_target_cut_rate=0.5,
)


class TestCurrentBudget:
    def test_under_target_boosts_scale(self) -> None:
        """Below 1A → scale gradually climbs above 1.0."""
        w = _PM(_TARGET_THRESHOLDS)
        for _ in range(10):
            w.tick(_state(motor_ids=[M1], current_a=0.5))
        assert w.pm._current_target_global > 1.0
        assert w.pm.get_compliance_scale(M1) > 1.0

    def test_boost_caps_at_boost_max(self) -> None:
        """Sustained under-current never exceeds boost_max."""
        w = _PM(_TARGET_THRESHOLDS)
        for _ in range(1000):
            w.tick(_state(motor_ids=[M1], current_a=0.1))
        assert w.pm._current_target_global == pytest.approx(
            _TARGET_THRESHOLDS.current_target_boost_max
        )

    def test_over_target_cuts_scale(self) -> None:
        """Above 1A → global scale drops below 1.0."""
        w = _PM(_TARGET_THRESHOLDS)
        w.tick(_state(motor_ids=[M1], current_a=2.0))
        assert w.pm._current_target_global < 1.0

    def test_over_target_proportional_cut(self) -> None:
        """Ideal cut at 2A target=1A → global approaches 0.5."""
        w = _PM(_TARGET_THRESHOLDS)
        for _ in range(20):
            w.tick(_state(motor_ids=[M1], current_a=2.0))
        # rate-limited at 0.5/tick, so after 2 ticks we'd be at 0.5; many ticks → stable
        assert w.pm._current_target_global == pytest.approx(0.5, abs=0.05)

    def test_high_torque_motor_cut_more(self) -> None:
        """When over budget, the motor with higher torque gets a lower scale."""
        w = _PM(_TARGET_THRESHOLDS)
        for _ in range(5):
            w.tick(_state(
                motor_ids=[M1, M2],
                current_a=2.0,
                motor_torques={M1: 10.0, M2: 1.0},
            ))
        scale_m1 = w.pm._motor_states[M1].current_budget_scale
        scale_m2 = w.pm._motor_states[M2].current_budget_scale
        assert scale_m1 < scale_m2, "high-torque motor should be cut more"

    def test_zero_torque_motor_spared(self) -> None:
        """A motor doing no work gets scale=1.0 even when system is over budget."""
        w = _PM(_TARGET_THRESHOLDS)
        for _ in range(5):
            w.tick(_state(
                motor_ids=[M1, M2],
                current_a=2.0,
                motor_torques={M1: 10.0, M2: 0.0},
            ))
        assert w.pm._motor_states[M2].current_budget_scale == pytest.approx(1.0)

    def test_boost_then_cut_responds_correctly(self) -> None:
        """Scale rises when under, then falls quickly when over."""
        w = _PM(_TARGET_THRESHOLDS)
        for _ in range(10):
            w.tick(_state(motor_ids=[M1], current_a=0.5))
        boosted = w.pm._current_target_global
        assert boosted > 1.0

        for _ in range(5):
            w.tick(_state(motor_ids=[M1], current_a=3.0))
        assert w.pm._current_target_global < boosted
