"""
Contact sub-classifier — enriches a HoldReading with motor-state context.

Three sub-types extend plain HOLD:
  RESTRICT  — motors are pushing but the joint is not moving (stalled under load).
  WRENCH    — the user is actively driving a joint against the motor's torque direction.
  SQUEEZE   — hold with meaningful FSR pressure across the held modules.

Priority when multiple conditions fire: WRENCH > RESTRICT > SQUEEZE > HOLD.
All thresholds are initial guesses and should be tuned on real hardware.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from enum import Enum

from petctl.perception.stroke import HoldReading
from petctl.types import RobotState


class ContactType(str, Enum):
    HOLD = "hold"
    SQUEEZE = "squeeze"
    RESTRICT = "restrict"
    WRENCH = "wrench"


@dataclass
class ContactReading:
    """HoldReading enriched with contact sub-type and motor/pressure context."""
    hold: HoldReading
    contact_type: ContactType
    affected_servos: list[int] = field(default_factory=list)
    torque_peak: float = 0.0
    pressure_peak: float = 0.0


class ContactClassifier:
    """
    Classify a HoldReading into a ContactType using motor feedback and FSR data.

    Call classify() each tick while HoldDetector returns a non-None reading.
    Call reset() when contact ends so position history doesn't bleed into the
    next hold.
    """

    RESTRICT_TORQUE_NM: float = 0.5
    RESTRICT_VEL_THRESHOLD: float = 0.15

    # WRENCH: joint must move >= WRENCH_POS_DELTA_RAD opposite to torque direction
    # over the last WRENCH_WINDOW_FRAMES frames while torque is above threshold.
    # Requiring real position movement (not just instantaneous velocity) filters
    # out the sign-flip noise that occurs when the sine wave command reverses direction.
    WRENCH_TORQUE_NM: float = 0.5
    WRENCH_POS_DELTA_RAD: float = 0.06   # ~3.4° — tunable on real hardware
    WRENCH_WINDOW_FRAMES: int = 10        # ~0.33 s at 30 Hz

    SQUEEZE_PRESSURE: float = 0.15

    def __init__(self) -> None:
        self._pos_history: dict[int, deque[float]] = {}

    def reset(self) -> None:
        """Clear position history. Call when contact ends."""
        self._pos_history.clear()

    def classify(self, hold: HoldReading, state: RobotState) -> ContactReading:
        """Return a ContactReading for the given hold and robot state."""
        servo_ids = sorted({m for blob in hold.q_blobs for m in blob.modules if m >= 1})

        # Update rolling position history for held servos.
        for sid in servo_ids:
            pos = state.servo_positions.get(sid)
            if pos is None:
                continue
            if sid not in self._pos_history:
                self._pos_history[sid] = deque(maxlen=self.WRENCH_WINDOW_FRAMES)
            self._pos_history[sid].append(pos)

        # Check WRENCH first (highest priority).
        # A wrench requires: (a) high torque AND (b) position has actually moved
        # opposite to torque direction over the history window — not just a momentary
        # velocity blip from the motor reversing its sine command.
        wrench_servos: list[int] = []
        torque_peak = 0.0
        for sid in servo_ids:
            t = state.motor_torques.get(sid, 0.0)
            if abs(t) < self.WRENCH_TORQUE_NM:
                continue
            history = self._pos_history.get(sid)
            if not history or len(history) < 3:
                continue
            pos_delta = history[-1] - history[0]
            if (
                abs(pos_delta) > self.WRENCH_POS_DELTA_RAD
                and math.copysign(1.0, pos_delta) != math.copysign(1.0, t)
            ):
                wrench_servos.append(sid)
                torque_peak = max(torque_peak, abs(t))
        if wrench_servos:
            return ContactReading(
                hold=hold,
                contact_type=ContactType.WRENCH,
                affected_servos=wrench_servos,
                torque_peak=torque_peak,
            )

        # Check RESTRICT.
        restrict_servos: list[int] = []
        torque_peak = 0.0
        for sid in servo_ids:
            t = state.motor_torques.get(sid, 0.0)
            v = state.motor_velocities.get(sid, 0.0)
            if abs(t) > self.RESTRICT_TORQUE_NM and abs(v) < self.RESTRICT_VEL_THRESHOLD:
                restrict_servos.append(sid)
                torque_peak = max(torque_peak, abs(t))
        if restrict_servos:
            return ContactReading(
                hold=hold,
                contact_type=ContactType.RESTRICT,
                affected_servos=restrict_servos,
                torque_peak=torque_peak,
            )

        # Check SQUEEZE.
        held_modules = {m for blob in hold.q_blobs for m in blob.modules}
        pressure_total = sum(
            state.sensors[m].pressure_total for m in held_modules if m in state.sensors
        )
        pressure_peak = max(
            (state.sensors[m].pressure_total for m in held_modules if m in state.sensors),
            default=0.0,
        )
        if pressure_total > self.SQUEEZE_PRESSURE:
            return ContactReading(
                hold=hold,
                contact_type=ContactType.SQUEEZE,
                pressure_peak=pressure_peak,
            )

        return ContactReading(hold=hold, contact_type=ContactType.HOLD)
