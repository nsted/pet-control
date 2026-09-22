"""Tests for MockBackend's second-order joint dynamics (SHAPE_LAYER Stage 1.3)."""

from __future__ import annotations

import json
import time

import pytest

from petctl.backends.mock import MockBackend
from petctl.config import LOOP_LIMITS, MOTOR_LIMITS
from petctl.types import ServoCommand

SERVO_ID = 1


def _make_backend(**kwargs) -> MockBackend:
    b = MockBackend(num_modules=2, **kwargs)  # servo 1 only
    b._connected = True
    return b


async def _tick(b: MockBackend, dt: float):
    """Force get_state()'s internal dt to a known value, then advance one step."""
    b._last_timestamp = time.monotonic() - dt
    return await b.get_state()


class TestPassiveDefault:
    @pytest.mark.asyncio
    async def test_no_command_stays_at_rest(self):
        """No setpoint recorded yet -> kp=kd=torque_ff=0 -> passive, no motion."""
        b = _make_backend()
        state = await _tick(b, dt=1.0 / 30.0)
        assert state.servo_positions[SERVO_ID] == 0.0
        assert state.motor_velocities[SERVO_ID] == 0.0
        assert state.motor_torques[SERVO_ID] == 0.0


class TestSendCommandsRecordsSetpointOnly:
    @pytest.mark.asyncio
    async def test_send_commands_does_not_teleport_position(self):
        """send_commands() only records the setpoint; get_state() integrates it."""
        b = _make_backend()
        await b.send_commands([
            ServoCommand(servo_id=SERVO_ID, position=1.0, kp=MOTOR_LIMITS.kp_default, kd=MOTOR_LIMITS.kd_default)
        ])
        assert b._servo_positions[SERVO_ID] == 0.0

    @pytest.mark.asyncio
    async def test_position_none_command_is_ignored(self):
        """A velocity/torque-only command (position=None) doesn't create a setpoint."""
        b = _make_backend()
        await b.send_commands([ServoCommand(servo_id=SERVO_ID, position=None, torque_ff=0.5)])
        assert SERVO_ID not in b._servo_setpoints


class TestMitLawIntegration:
    @pytest.mark.asyncio
    async def test_moves_toward_target_after_one_tick(self):
        b = _make_backend()
        await b.send_commands([
            ServoCommand(servo_id=SERVO_ID, position=1.0, kp=MOTOR_LIMITS.kp_default, kd=MOTOR_LIMITS.kd_default)
        ])
        state = await _tick(b, dt=1.0 / 50.0)
        assert 0.0 < state.servo_positions[SERVO_ID] < 1.0

    @pytest.mark.asyncio
    async def test_converges_toward_target_over_ticks(self):
        """Repeated ticks with a fixed setpoint settle near the target position."""
        b = _make_backend()
        target = 1.0
        await b.send_commands([
            ServoCommand(servo_id=SERVO_ID, position=target, kp=MOTOR_LIMITS.kp_default, kd=MOTOR_LIMITS.kd_default)
        ])

        dt = 1.0 / 50.0
        state = None
        for _ in range(500):  # 10 simulated seconds
            state = await _tick(b, dt)

        assert abs(state.servo_positions[SERVO_ID] - target) < 0.02

    @pytest.mark.asyncio
    async def test_velocity_and_torque_are_populated(self):
        """motor_velocities/motor_torques are real values, not the old hardcoded {}."""
        b = _make_backend()
        await b.send_commands([
            ServoCommand(servo_id=SERVO_ID, position=1.0, kp=MOTOR_LIMITS.kp_default, kd=MOTOR_LIMITS.kd_default)
        ])
        state = await _tick(b, dt=1.0 / 50.0)
        assert SERVO_ID in state.motor_velocities
        assert SERVO_ID in state.motor_torques
        assert state.motor_torques[SERVO_ID] != 0.0

    @pytest.mark.asyncio
    async def test_torque_clamped_to_motor_limits(self):
        """A huge position error must not produce torque beyond MOTOR_LIMITS."""
        b = _make_backend()
        await b.send_commands([
            ServoCommand(servo_id=SERVO_ID, position=100.0, kp=MOTOR_LIMITS.kp_max, kd=MOTOR_LIMITS.kd_max)
        ])
        state = await _tick(b, dt=1.0 / 50.0)
        assert MOTOR_LIMITS.torque_min <= state.motor_torques[SERVO_ID] <= MOTOR_LIMITS.torque_max

    @pytest.mark.asyncio
    async def test_velocity_clamped_to_physical_speed_limit(self):
        """A huge position error must not produce velocity beyond the physical

        speed cap (LOOP_LIMITS.max_speed_rad_s) — not MOTOR_LIMITS.vel_min/max,
        which is the MIT wire-encoding range for v_des, a much smaller number.
        """
        b = _make_backend()
        await b.send_commands([
            ServoCommand(servo_id=SERVO_ID, position=100.0, kp=MOTOR_LIMITS.kp_max, kd=MOTOR_LIMITS.kd_max)
        ])
        state = await _tick(b, dt=1.0 / 50.0)
        assert (
            -LOOP_LIMITS.max_speed_rad_s - 1e-9
            <= state.motor_velocities[SERVO_ID]
            <= LOOP_LIMITS.max_speed_rad_s + 1e-9
        )

    @pytest.mark.asyncio
    async def test_negative_dt_does_not_move_or_raise(self):
        """A clock hiccup (dt < 0) must be a no-op, not an integration in reverse."""
        b = _make_backend()
        await b.send_commands([
            ServoCommand(servo_id=SERVO_ID, position=1.0, kp=MOTOR_LIMITS.kp_default, kd=MOTOR_LIMITS.kd_default)
        ])
        state = await _tick(b, dt=-1.0)
        assert state.servo_positions[SERVO_ID] == 0.0


class TestFileModeOverridePreserved:
    @pytest.mark.asyncio
    async def test_file_position_wins_over_dynamics(self, tmp_path):
        """Existing 'file' mode behavior: the JSON file always wins for position."""
        state_file = tmp_path / "state.json"
        state_file.write_text(json.dumps({"servos": {"1": 2.5}}))

        b = MockBackend(mode="file", state_file=str(state_file), num_modules=2)
        await b.connect()
        await b.send_commands([
            ServoCommand(servo_id=SERVO_ID, position=0.0, kp=MOTOR_LIMITS.kp_max, kd=MOTOR_LIMITS.kd_max)
        ])

        state = await _tick(b, dt=1.0 / 50.0)
        assert state.servo_positions[SERVO_ID] == 2.5
