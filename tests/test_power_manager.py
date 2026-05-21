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
    voltage_median_window=3,           # fewer samples to fill
)


def _state(
    *,
    motor_ids: list[int] | None = None,
    drive_temps: dict[int, int] | None = None,
    winding_temps: dict[int, int] | None = None,
    err_codes: dict[int, int] | None = None,
    voltage_v: float = 14.5,
) -> RobotState:
    """Build a minimal RobotState for testing."""
    ids = motor_ids or []
    # Patch battery_voltage_v by injecting raw values that produce the desired voltage.
    # Easier: subclass RobotState or just set via property workaround.
    # Since battery_voltage_v is a property computed from battery_voltage_raw, we monkeypatch
    # by overriding the attribute on the instance directly.
    rs = RobotState(
        active_servo_ids=set(ids),
        motor_temperatures=drive_temps or {},
        motor_winding_temperatures=winding_temps or {},
        motor_err_codes=err_codes or {},
    )
    # Override battery_voltage_v via __dict__ (property override for testing)
    rs.__class__ = type(
        "_TestRobotState",
        (RobotState,),
        {"battery_voltage_v": property(lambda self: voltage_v)},
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
# Voltage state machine
# ---------------------------------------------------------------------------

class TestVoltageSanityFilter:
    def test_insane_reading_discarded(self) -> None:
        """Readings outside [0, 40V] are silently dropped."""
        w = _PM()
        # Feed only insane readings
        for _ in range(10):
            w.tick(_state(voltage_v=50.0))
        # Window never fills → no threshold action
        assert w.pm._system_state == SystemState.RUNNING
        assert w.pm._voltage_filtered is None

    def test_negative_voltage_discarded(self) -> None:
        w = _PM()
        for _ in range(10):
            w.tick(_state(voltage_v=-1.0))
        assert w.pm._voltage_filtered is None

    def test_median_filter_smooths_spike(self) -> None:
        """A single spike in a window of 3 should not affect the median."""
        w = _PM(_FAST_THRESHOLDS)
        # Feed two normal samples
        w.tick(_state(voltage_v=14.5))
        w.tick(_state(voltage_v=14.5))
        # Feed one spike — median of [14.5, 14.5, 20.0] = 14.5
        w.tick(_state(voltage_v=20.0))
        assert w.pm._voltage_filtered == pytest.approx(14.5)


class TestVoltageLowWarning:
    def test_low_warning_fires(self) -> None:
        w = _PM()
        for _ in range(_FAST_THRESHOLDS.voltage_median_window):
            w.tick(_state(voltage_v=11.5))
        assert w.pm._voltage_state == VoltageState.LOW_WARNING
        assert w.pm._system_state == SystemState.RUNNING  # warning, not emergency

    def test_critical_low_triggers_emergency(self) -> None:
        w = _PM()
        for _ in range(_FAST_THRESHOLDS.voltage_median_window):
            w.tick(_state(voltage_v=9.0))
        assert w.pm._system_state == SystemState.EMERGENCY_STOPPED

    def test_normal_voltage_no_action(self) -> None:
        w = _PM()
        for _ in range(_FAST_THRESHOLDS.voltage_median_window):
            w.tick(_state(voltage_v=14.5))
        assert w.pm._system_state == SystemState.RUNNING
        assert w.pm._voltage_state == VoltageState.NORMAL


class TestVoltageSpikeDetection:
    def test_spike_logged(self) -> None:
        w = _PM()
        w.tick(_state(voltage_v=20.0))
        assert w.pm._spike_count_total == 1
        assert w.pm._last_spike_peak_v == pytest.approx(20.0)

    def test_sanity_rejected_not_counted_as_spike(self) -> None:
        """Readings above sanity_max_v are discarded before spike detection."""
        w = _PM()
        w.tick(_state(voltage_v=45.0))
        assert w.pm._spike_count_total == 0

    def test_absolute_emergency_single_sample(self) -> None:
        w = _PM()
        w.tick(_state(voltage_v=31.0))
        _, is_global = w.drain()
        assert is_global
        assert w.pm._system_state == SystemState.EMERGENCY_STOPPED

    def test_spike_rate_emergency(self) -> None:
        """More than voltage_spike_rate_emergency_count spikes in window → emergency."""
        t = PowerThresholds(
            voltage_spike_rate_emergency_count=3,
            voltage_spike_rate_window_s=60.0,
            voltage_median_window=3,
        )
        w = _PM(t)
        for _ in range(5):  # 5 spikes > threshold of 3
            w.tick(_state(voltage_v=20.0), dt=0.1)
        _, is_global = w.drain()
        assert is_global

    def test_old_spikes_age_out(self) -> None:
        """Spikes older than the window should not count toward rate."""
        t = PowerThresholds(
            voltage_spike_rate_emergency_count=3,
            voltage_spike_rate_window_s=1.0,  # very short window
            voltage_median_window=3,
            voltage_absolute_emergency_v=100.0,  # raise to not trip on raw spikes
        )
        w = _PM(t)
        # Fire 3 spikes
        for _ in range(3):
            w.tick(_state(voltage_v=20.0), dt=0.1)
        # Advance past the window
        w.advance(1.5)
        # One more spike — count in window should be 1, below threshold of 3
        w.tick(_state(voltage_v=20.0))
        assert w.pm._system_state == SystemState.RUNNING


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

    def test_telemetry_spike_tracking(self) -> None:
        w = _PM()
        w.tick(_state(voltage_v=20.0))
        t = w.pm.get_telemetry(20.0)
        assert t.voltage_spike_count == 1
        assert t.voltage_last_spike_peak_v == pytest.approx(20.0)

    def test_telemetry_system_state(self) -> None:
        w = _PM()
        t = w.pm.get_telemetry(14.5)
        assert t.system_state == "RUNNING"
