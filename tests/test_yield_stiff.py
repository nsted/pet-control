"""Tests for YieldStiffScheme: torque-driven position drift compliance."""

from __future__ import annotations

import pytest

from petctl.schemes.patterns import YieldStiffScheme
from petctl.types import RobotState


def _state(
    servo_positions: dict[int, float],
    motor_torques: dict[int, float],
    dt: float = 0.1,
) -> RobotState:
    return RobotState(
        timestamp=1.0,
        servo_positions=servo_positions,
        motor_torques=motor_torques,
        active_servo_ids=set(servo_positions.keys()),
        dt=dt,
        connected=True,
    )


ABOVE = YieldStiffScheme.TORQUE_BASELINE_NM * 3  # clearly above baseline
BELOW = YieldStiffScheme.TORQUE_BASELINE_NM * 0.4  # clearly below baseline


class TestYieldStiffScheme:

    def test_no_torque_no_drift(self):
        """Zero torque → commanded stays at 0."""
        scheme = YieldStiffScheme()
        cmds = scheme.update(_state({1: 0.3}, {1: 0.0}))
        assert cmds[0].position == pytest.approx(0.0)

    def test_torque_below_baseline_no_drift(self):
        """Torque below baseline → no drift."""
        scheme = YieldStiffScheme()
        cmds = scheme.update(_state({1: 0.3}, {1: BELOW}))
        assert cmds[0].position == pytest.approx(0.0)

    def test_torque_above_baseline_drifts_toward_actual(self):
        """Torque above baseline → commanded moves toward actual."""
        scheme = YieldStiffScheme()
        cmds = scheme.update(_state({1: 0.5}, {1: ABOVE}, dt=0.1))
        assert 0.0 < cmds[0].position <= 0.5

    def test_drift_direction_positive(self):
        """Positive displacement → commanded drifts positive."""
        scheme = YieldStiffScheme()
        cmds = scheme.update(_state({1: 0.5}, {1: ABOVE}, dt=0.1))
        assert cmds[0].position > 0.0

    def test_drift_direction_negative(self):
        """Negative displacement → commanded drifts negative."""
        scheme = YieldStiffScheme()
        cmds = scheme.update(_state({1: -0.5}, {1: ABOVE}, dt=0.1))
        assert cmds[0].position < 0.0

    def test_does_not_overshoot_actual(self):
        """Step is capped so commanded never goes past actual position."""
        scheme = YieldStiffScheme()
        # Large dt → large step budget, but actual is only 0.1 rad away
        cmds = scheme.update(_state({1: 0.1}, {1: ABOVE}, dt=100.0))
        assert cmds[0].position == pytest.approx(0.1)

    def test_torque_drop_stops_drift(self):
        """Once torque drops below baseline, commanded holds at current value."""
        scheme = YieldStiffScheme()
        scheme.update(_state({1: 0.5}, {1: ABOVE}, dt=0.1))
        held = scheme._commanded[1]
        assert held > 0.0

        scheme.update(_state({1: 0.5}, {1: BELOW}, dt=0.1))
        assert scheme._commanded[1] == pytest.approx(held)

    def test_holds_new_position_no_return_to_home(self):
        """After yield, position is held — no automatic return to home."""
        scheme = YieldStiffScheme()
        # Drive setpoint all the way to actual
        for _ in range(30):
            scheme.update(_state({1: 0.3}, {1: ABOVE}, dt=0.1))
        # Torque drops
        cmds = scheme.update(_state({1: 0.3}, {1: BELOW}, dt=0.1))
        assert cmds[0].position == pytest.approx(0.3, abs=0.02)

    def test_multiple_servos_yield_independently(self):
        """Each servo yields based on its own torque; others are unaffected."""
        scheme = YieldStiffScheme()
        cmds = scheme.update(_state(
            {1: 0.5, 2: 0.3},
            {1: ABOVE, 2: BELOW},
            dt=0.1,
        ))
        cmd = {c.servo_id: c for c in cmds}
        assert cmd[1].position > 0.0   # yielded
        assert cmd[2].position == pytest.approx(0.0)  # not yielded

    def test_successive_pushes_accumulate(self):
        """A second push after torque drops accumulates from the new resting position."""
        scheme = YieldStiffScheme()
        # First push to ~0.3
        for _ in range(30):
            scheme.update(_state({1: 0.3}, {1: ABOVE}, dt=0.1))
        scheme.update(_state({1: 0.3}, {1: BELOW}, dt=0.1))
        pos_after_first = scheme._commanded[1]

        # Second push further
        scheme.update(_state({1: 0.6}, {1: ABOVE}, dt=0.1))
        assert scheme._commanded[1] > pos_after_first
