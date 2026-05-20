"""
Contact sub-classifier — enriches a HoldReading with motor-state context.

Three sub-types extend plain HOLD:
  RESTRICT  — motors are pushing but the joint is not moving (stalled under load).
  WRENCH    — joint is being driven against the motor's torque direction (back-driven).
  SQUEEZE   — hold with meaningful FSR pressure across the held modules.

Priority when multiple conditions fire: WRENCH > RESTRICT > SQUEEZE > HOLD.
All thresholds are initial guesses and should be tuned on real hardware.
"""

from __future__ import annotations

import math
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

    Call classify() after HoldDetector returns a non-None HoldReading.
    Returns ContactReading with the most specific type that fits.
    """

    RESTRICT_TORQUE_NM: float = 0.5
    RESTRICT_VEL_THRESHOLD: float = 0.15

    WRENCH_TORQUE_NM: float = 0.3
    WRENCH_VEL_MIN: float = 0.10

    SQUEEZE_PRESSURE: float = 0.15

    def classify(self, hold: HoldReading, state: RobotState) -> ContactReading:
        """Return a ContactReading for the given hold and robot state."""
        servo_ids = sorted({m for blob in hold.q_blobs for m in blob.modules if m >= 1})

        # Check WRENCH first (highest priority).
        wrench_servos: list[int] = []
        torque_peak = 0.0
        for sid in servo_ids:
            t = state.motor_torques.get(sid, 0.0)
            v = state.motor_velocities.get(sid, 0.0)
            if (
                abs(t) > self.WRENCH_TORQUE_NM
                and abs(v) > self.WRENCH_VEL_MIN
                and math.copysign(1.0, t) != math.copysign(1.0, v)
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
