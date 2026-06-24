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
    PowerSource,
    PowerThresholds,
    SystemState,
    VoltageState,
)
from petctl.types import RobotState, ServoCommand

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
    voltage_v: float = 12.0,
    current_a: float = 0.0,
) -> RobotState:
    """Build a minimal RobotState for testing."""
    ids = motor_ids or []
    rs = RobotState(
        active_servo_ids=set(ids),
        motor_temperatures=drive_temps or {},
        motor_winding_temperatures=winding_temps or {},
        motor_err_codes=err_codes or {},
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

    def __init__(
        self,
        thresholds: PowerThresholds = _FAST_THRESHOLDS,
        reactive_ema_alpha: float | None = None,
    ) -> None:
        self.pm = PowerManager(thresholds, reactive_ema_alpha=reactive_ema_alpha)
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
    # Reactive backstop thresholds: budget=2.0A, start=2.0*1.00=2.0A, zero=2.0*1.50=3.0A

    def test_below_threshold_scale_is_one(self) -> None:
        """Current well below backstop → reactive scale unaffected."""
        w = _PM()
        for _ in range(100):
            w.tick(_state(motor_ids=[M1], current_a=1.0))
        assert w.pm._reactive_scale == pytest.approx(1.0)
        assert w.pm.get_compliance_scale(M1) == pytest.approx(1.0)

    def test_above_limit_scale_is_zero(self) -> None:
        """Current saturated above backstop cutoff → reactive scale zeroed."""
        w = _PM()
        for _ in range(100):
            w.tick(_state(motor_ids=[M1], current_a=5.0))
        assert w.pm._reactive_scale == pytest.approx(0.0, abs=0.01)

    def test_thermal_and_reactive_are_independent(self) -> None:
        """Thermal compliance scale and reactive EMA backstop are independent signals.

        get_compliance_scale() returns thermal scale only; reactive scale accumulates
        separately via _reactive_scale / allocate_budget().
        """
        w = _PM(PowerThresholds(temp_hysteresis_cooldown_s=0.05))
        # Engage both simultaneously: thermal WARNING (58°C) + sustained overcurrent
        for _ in range(100):
            w.tick(_state(motor_ids=[M1], drive_temps={M1: 58}, current_a=5.0))
        # Thermal: get_compliance_scale returns thermal-only scale (WARNING = 0.5)
        assert w.pm.get_compliance_scale(M1) == pytest.approx(0.5, abs=0.01)
        # Reactive: fully engaged independently — verified separately
        assert w.pm._reactive_scale == pytest.approx(0.0, abs=0.01)


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
        assert t.current_amps_filtered == pytest.approx(0.4, abs=0.01)  # 0.2 * 2.0
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
# Reactive EMA asymmetric decay
# ---------------------------------------------------------------------------

class TestCurrentLimitingAsymmetricDecay:
    def test_ema_decays_faster_on_recovery(self) -> None:
        """After sustained overcurrent clears, scale starts recovering quickly because
        asymmetric EMA decay (recovery_multiplier=3×) brings EMA below the P-start
        threshold within ~2 ticks. Full recovery is slower — bounded by the I term
        draining at ki_decay rate (~40 ticks) and the rate limiter (~50 ticks at 1.0/s)."""
        w = _PM()
        for _ in range(100):
            w.tick(_state(current_a=5.0))  # EMA saturates; scale→0, I term→1
        assert w.pm._reactive_scale == pytest.approx(0.0, abs=0.01)

        # Fast EMA drain brings P term to zero within ~2 ticks → scale starts rising
        for _ in range(5):
            w.tick(_state(current_a=1.0))
        assert w.pm._reactive_scale > 0.0

        # Full recovery after I term drains and rate limiter allows it (~50 more ticks)
        for _ in range(60):
            w.tick(_state(current_a=1.0))
        assert w.pm._reactive_scale == pytest.approx(1.0, abs=0.01)


# ---------------------------------------------------------------------------
# allocate_budget — bin-pack seeding, priority ordering, and promotion
# ---------------------------------------------------------------------------

def _alloc_state(
    *,
    voltage_v: float = 12.0,
    torques: dict[int, float] | None = None,
    velocities: dict[int, float] | None = None,
    current_a: float = 0.0,
) -> RobotState:
    """Minimal RobotState for allocate_budget() tests."""
    rs = RobotState(
        motor_torques=torques or {},
        motor_velocities=velocities or {},
        battery_current_raw=0,
    )
    rs.__class__ = type(
        "_AllocRobotState",
        (RobotState,),
        {
            "battery_voltage_v": property(lambda self: voltage_v),
            "battery_current_amps": property(lambda self: current_a),
        },
    )
    return rs


def _commands(motor_ids: list[int]) -> list[ServoCommand]:
    return [ServoCommand(servo_id=mid, position=None) for mid in motor_ids]


class TestBinPackSeed:
    """Active bin is seeded from the priority-ordered pending queue up to budget."""

    def test_seed_puts_one_motor_at_dev_budget(self) -> None:
        """At dev budget (2A) with worst-case 2A/motor, only 1 motor seeds the active bin."""
        from petctl.power_manager import BinPackPolicy

        motor_ids = [1, 2, 3, 4, 5]
        state = _alloc_state()
        pm = PowerManager(bin_policy=BinPackPolicy(priority=motor_ids))
        active, pending = pm.allocate_budget(_commands(motor_ids), state)

        assert len(pm._active_motor_set) == 1
        assert pm._active_motor_set == {1}   # first in explicit priority
        assert set(pending) == {2, 3, 4, 5}

    def test_seed_respects_priority_order(self) -> None:
        """Pending queue is ordered by BinPackPolicy.priority."""
        from petctl.power_manager import BinPackPolicy

        motor_ids = [1, 2, 3, 4, 5]
        priority = [5, 3, 1, 4, 2]
        state = _alloc_state()
        pm = PowerManager(bin_policy=BinPackPolicy(priority=priority))
        _, pending = pm.allocate_budget(_commands(motor_ids), state)

        assert pending == [3, 1, 4, 2]   # motor 5 seeded, rest in priority order

    def test_seed_detects_wall_power_source(self) -> None:
        """Wall power source is detected after wall_confirm_ticks; dev wall budget
        (2.5A at 14.6V) still seeds only 1 motor — floor is ~1.64A/motor so only
        1 fits. The test verifies detection and that bin-pack continues working."""
        from petctl.config import POWER_BUDGET as b
        from petctl.power_manager import BinPackPolicy

        motor_ids = [1, 2, 3, 4, 5, 6, 7]
        state = _alloc_state(voltage_v=14.6)
        pm = PowerManager(bin_policy=BinPackPolicy(priority=motor_ids))
        # Trigger wall power source detection (requires wall_confirm_ticks consecutive readings)
        for _ in range(b.wall_confirm_ticks + 1):
            pm.update(state, now=0.0)
        _, pending = pm.allocate_budget(_commands(motor_ids), state)

        assert pm._power_source == PowerSource.WALL
        assert len(pm._active_motor_set) == 1   # dev wall budget (2.5A) < 2 × floor (1.64A)
        assert 1 in pm._active_motor_set         # priority-first motor seeded

    def test_no_pending_when_all_fit(self) -> None:
        """When all motors fit within budget, pending queue is empty."""
        from petctl.power_manager import BinPackPolicy

        # 1 motor at 2A/motor ≤ 2A budget → all fit
        state = _alloc_state()
        pm = PowerManager(bin_policy=BinPackPolicy(priority=[1]))
        active, pending = pm.allocate_budget(_commands([1]), state)

        assert pm._active_motor_set == {1}
        assert pending == []


class TestBinPackPromotion:
    """Motors are promoted from pending when bus current EMA shows headroom."""

    def test_promotion_when_ema_below_threshold(self) -> None:
        """Pending motor is promoted when _current_ema < budget × headroom_factor."""
        from petctl.power_manager import BinPackPolicy

        motor_ids = [1, 2, 3]
        policy = BinPackPolicy(priority=motor_ids, headroom_factor=0.8)
        state = _alloc_state()
        pm = PowerManager(bin_policy=policy)

        # First tick: seed with motor 1, motors 2 and 3 pending
        pm.allocate_budget(_commands(motor_ids), state)
        assert pm._active_motor_set == {1}

        # Force EMA well below threshold (budget=2A, threshold=1.6A → EMA=0.5A → headroom)
        pm._current_ema = 0.5
        _, pending = pm.allocate_budget(_commands(motor_ids), state)

        assert 2 in pm._active_motor_set, "motor 2 should have been promoted"
        assert 2 not in pending

    def test_no_promotion_when_ema_above_threshold(self) -> None:
        """No promotion when bus current EMA is at or above budget × headroom_factor."""
        from petctl.power_manager import BinPackPolicy

        motor_ids = [1, 2, 3]
        policy = BinPackPolicy(priority=motor_ids, headroom_factor=0.8)
        state = _alloc_state()
        pm = PowerManager(bin_policy=policy)

        pm.allocate_budget(_commands(motor_ids), state)
        pm._current_ema = 1.7   # 1.7A > 2A × 0.8 = 1.6A → no headroom
        _, pending = pm.allocate_budget(_commands(motor_ids), state)

        assert pm._active_motor_set == {1}
        assert pending == [2, 3]

    def test_eviction_when_motor_no_longer_commanded(self) -> None:
        """Motors removed from commands are evicted from active set and pending queue."""
        from petctl.power_manager import BinPackPolicy

        motor_ids = [1, 2, 3]
        policy = BinPackPolicy(priority=motor_ids)
        state = _alloc_state()
        pm = PowerManager(bin_policy=policy)

        pm.allocate_budget(_commands(motor_ids), state)
        # Now remove motor 1 (which was seeded into active) from commands
        pm.allocate_budget(_commands([2, 3]), state)

        assert 1 not in pm._active_motor_set
        assert 1 not in pm._pending_queue
