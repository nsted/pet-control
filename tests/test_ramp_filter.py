"""Tests for the per-motor position ramp filter in RobotBackend.send_commands()."""

from __future__ import annotations

import time
from unittest.mock import AsyncMock

import pytest

from petctl.backends.robot import RobotBackend, _parse_slcan, _uint_to_float
from petctl.config import LOOP_LIMITS, MOTOR_LIMITS
from petctl.types import ServoCommand

MOTOR_ID = 7


def _decode_pos(frame: str) -> float:
    """Extract p_des (radians) from a SLCAN MIT frame string."""
    _, payload = _parse_slcan(frame)
    p_raw = (payload[0] << 8) | payload[1]
    return _uint_to_float(p_raw, 16, MOTOR_LIMITS.pos_min, MOTOR_LIMITS.pos_max)


def _make_backend() -> RobotBackend:
    b = RobotBackend()
    b._connected = True
    b._discovered_motors = [MOTOR_ID]
    b._motor_state[MOTOR_ID] = {"pos": 0.0, "vel": 0.0, "torque": 0.0, "drive_temp": 0, "motor_temp": 0, "err_code": 0}
    return b


def _cmd(pos: float) -> ServoCommand:
    return ServoCommand(servo_id=MOTOR_ID, position=pos)


def _seed_ramp(b: RobotBackend, pos: float, dt_ago: float = 1.0 / 30.0) -> None:
    """Prime ramp state as if the last command was sent dt_ago seconds ago at pos."""
    b._last_mit_abs_pos[MOTOR_ID] = pos
    b._last_mit_wall_s[MOTOR_ID] = time.monotonic() - dt_ago


class TestRampFilterBasics:
    @pytest.mark.asyncio
    async def test_first_command_seeds_from_physical_position(self):
        """No prior ramp state → first sent position is current physical pos (no snap)."""
        b = _make_backend()
        # Physical position is 0.0; command is 3.0 rad.
        # Backend seeds ramp from physical position so the first frame holds 0.0
        # rather than snapping to the target.
        await b.send_commands([_cmd(3.0)])
        assert abs(_decode_pos(b._pending_frames[MOTOR_ID]) - 0.0) < 0.01

    @pytest.mark.asyncio
    async def test_large_jump_is_clamped(self):
        """A sudden large target jump is clamped to max_speed_rad_s * dt."""
        b = _make_backend()
        dt = 1.0 / 30.0
        _seed_ramp(b, pos=0.0, dt_ago=dt)

        await b.send_commands([_cmd(10.0)])

        pos = _decode_pos(b._pending_frames[MOTOR_ID])
        max_step = LOOP_LIMITS.max_speed_rad_s * dt
        assert 0.0 < pos <= max_step + 0.01

    @pytest.mark.asyncio
    async def test_negative_jump_is_clamped(self):
        """Ramp clamps in both directions."""
        b = _make_backend()
        _seed_ramp(b, pos=0.0, dt_ago=1.0 / 30.0)

        await b.send_commands([_cmd(-10.0)])

        pos = _decode_pos(b._pending_frames[MOTOR_ID])
        max_step = LOOP_LIMITS.max_speed_rad_s * (1.0 / 30.0)
        assert -max_step - 0.01 <= pos < 0.0

    @pytest.mark.asyncio
    async def test_small_step_passes_through(self):
        """A step within the per-cycle budget is not clipped."""
        b = _make_backend()
        dt = 1.0 / 30.0
        _seed_ramp(b, pos=0.0, dt_ago=dt)

        tiny = LOOP_LIMITS.max_speed_rad_s * dt * 0.5
        await b.send_commands([_cmd(tiny)])

        assert abs(_decode_pos(b._pending_frames[MOTOR_ID]) - tiny) < 0.01

    @pytest.mark.asyncio
    async def test_ramp_converges_to_target(self):
        """Repeated commands with fixed dt converge p_des_commanded to the target."""
        b = _make_backend()
        target = 2.0
        dt = 1.0 / 30.0
        _seed_ramp(b, pos=0.0, dt_ago=dt)

        for _ in range(200):
            _seed_ramp(b, pos=_decode_pos(b._pending_frames.get(MOTOR_ID, "t00780000000000000000")), dt_ago=dt)
            await b.send_commands([_cmd(target)])

        assert abs(_decode_pos(b._pending_frames[MOTOR_ID]) - target) < 0.01

    @pytest.mark.asyncio
    async def test_ramp_state_persists_between_calls(self):
        """_last_mit_abs_pos is updated each call so ramp accumulates correctly."""
        b = _make_backend()
        dt = 1.0 / 30.0
        _seed_ramp(b, pos=0.0, dt_ago=dt)

        await b.send_commands([_cmd(5.0)])
        pos1 = _decode_pos(b._pending_frames[MOTOR_ID])

        _seed_ramp(b, pos=pos1, dt_ago=dt)
        await b.send_commands([_cmd(5.0)])
        pos2 = _decode_pos(b._pending_frames[MOTOR_ID])

        assert pos2 > pos1  # still ramping toward target


class TestResetSeeding:
    @pytest.mark.asyncio
    async def test_set_home_seeds_ramp_at_physical_position(self):
        """set_home() seeds _last_mit_abs_pos from motor state, not None."""
        b = _make_backend()
        b._motor_state[MOTOR_ID]["pos"] = 2.0

        await b.set_home()

        assert abs(b._last_mit_abs_pos[MOTOR_ID] - 2.0) < 1e-6
        assert MOTOR_ID in b._last_mit_wall_s

    @pytest.mark.asyncio
    async def test_set_home_first_command_does_not_snap(self):
        """First command after set_home() is ramped, not snapped to target."""
        b = _make_backend()
        b._motor_state[MOTOR_ID]["pos"] = 2.0
        await b.set_home()

        # Command a 1.0 rad offset (absolute target = 1.0 + 2.0 = 3.0, delta = 1.0 from 2.0)
        await b.send_commands([_cmd(1.0)])

        pos = _decode_pos(b._pending_frames[MOTOR_ID])
        # dt is floored at 1/120 s → max_step ≈ 8/120 ≈ 0.067 rad from 2.0
        assert pos < 2.5, f"pos={pos:.4f} suggests a snap, not a ramp"
        assert pos >= 2.0

    @pytest.mark.asyncio
    async def test_reset_motor_zero_seeds_ramp(self):
        """reset_motor_zero() seeds ramp at current physical position."""
        b = _make_backend()
        b._motor_state[MOTOR_ID]["pos"] = 5.0

        b.reset_motor_zero(MOTOR_ID)

        assert abs(b._last_mit_abs_pos[MOTOR_ID] - 5.0) < 1e-6
        assert MOTOR_ID in b._last_mit_wall_s

    @pytest.mark.asyncio
    async def test_reset_motor_zero_no_snap(self):
        """First command after reset_motor_zero() ramps from current position."""
        b = _make_backend()
        b._motor_state[MOTOR_ID]["pos"] = 5.0
        b.reset_motor_zero(MOTOR_ID)

        # Command software position 2.0 → absolute target = 2.0 + 5.0 = 7.0, delta = 2.0
        await b.send_commands([_cmd(2.0)])

        pos = _decode_pos(b._pending_frames[MOTOR_ID])
        # dt floored at 1/120 → max_step ≈ 0.067 rad; pos should be ≈ 5.067, not 7.0
        assert pos < 6.0, f"pos={pos:.4f} suggests a snap, not a ramp"
        assert pos >= 5.0

    @pytest.mark.asyncio
    async def test_set_hardware_zero_seeds_at_zero(self):
        """set_hardware_zero() seeds ramp at 0.0 (the new motor coordinate origin)."""
        b = _make_backend()
        b._motor_state[MOTOR_ID]["pos"] = 3.0
        b._last_mit_abs_pos[MOTOR_ID] = 3.0
        b._last_mit_wall_s[MOTOR_ID] = time.monotonic() - 1.0
        b._ws = AsyncMock()

        await b.set_hardware_zero()

        assert abs(b._last_mit_abs_pos[MOTOR_ID] - 0.0) < 1e-6
        assert MOTOR_ID in b._last_mit_wall_s
        assert not b._angle_offsets

    @pytest.mark.asyncio
    async def test_set_hardware_zero_no_snap(self):
        """First command after set_hardware_zero() ramps from 0.0."""
        b = _make_backend()
        b._motor_state[MOTOR_ID]["pos"] = 3.0
        b._ws = AsyncMock()
        await b.set_hardware_zero()

        # Command software position 2.0; offset cleared → absolute target = 2.0, delta = 2.0 from 0.0
        await b.send_commands([_cmd(2.0)])

        pos = _decode_pos(b._pending_frames[MOTOR_ID])
        # dt floored at 1/120 → max_step ≈ 0.067 rad; pos should be ≈ 0.067, not 2.0
        assert pos < 0.5, f"pos={pos:.4f} suggests a snap, not a ramp"
        assert pos >= 0.0
