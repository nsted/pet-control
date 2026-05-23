"""
Contact sub-classifier — enriches a HoldReading with motor-state context.

Three sub-types extend plain HOLD:
  RESTRICT  — motors are pushing but the joint is not moving (stalled under load).
  WRENCH    — a joint is displaced from its commanded position and the motor is resisting.
  SQUEEZE   — hold with meaningful FSR pressure across the held modules.

Both WRENCH and RESTRICT use per-servo hysteresis so they stay latched once
triggered and don't flicker at threshold boundaries.

WRENCH semantics:
  Begins when displacement from the commanded position exceeds WRENCH_POS_ON_RAD
  while motor torque is above WRENCH_TORQUE_NM.
  Persists until the joint returns within WRENCH_POS_OFF_RAD of commanded.
  Call reset() when contact ends.

RESTRICT semantics:
  Begins when |torque| > RESTRICT_TORQUE_NM and |velocity| < RESTRICT_VEL_ON.
  Persists until torque drops below RESTRICT_TORQUE_OFF or velocity exceeds
  RESTRICT_VEL_OFF.

Priority: WRENCH > RESTRICT > SQUEEZE > HOLD.
All thresholds are initial guesses — tune on real hardware.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from petctl.perception.stroke import HoldReading
from petctl.types import RobotState

# Fixed-width labels for console log lines, keyed by contact type string.
CONTACT_LABELS: dict[str, str] = {
    "stroke":   "STROKE  ",
    "hold":     "HOLD    ",
    "squeeze":  "SQUEEZE ",
    "restrict": "RESTRICT",
    "wrench":   "WRENCH  ",
}

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
    Call reset() when contact ends to clear hysteresis state.
    """

    # WRENCH: joint displaced from commanded position while motor resists.
    WRENCH_TORQUE_NM: float = 0.4
    WRENCH_POS_ON_RAD: float = 0.15   # ~8.6° — displacement from commanded to enter wrench
    WRENCH_POS_OFF_RAD: float = 0.05  # ~2.9° — displacement from commanded to exit wrench

    # RESTRICT: motor stalled under load.
    RESTRICT_TORQUE_ON: float = 0.5   # enter when torque exceeds this
    RESTRICT_TORQUE_OFF: float = 0.25  # exit when torque drops below this
    RESTRICT_VEL_ON: float = 0.12     # enter when |velocity| is below this
    RESTRICT_VEL_OFF: float = 0.22    # exit when |velocity| rises above this

    SQUEEZE_PRESSURE: float = 0.15

    def __init__(self) -> None:
        self._wrench_active: set[int] = set()
        self._restrict_active: set[int] = set()

    def reset(self) -> None:
        """Clear hysteresis state. Call when contact ends."""
        self._wrench_active.clear()
        self._restrict_active.clear()

    def classify(self, hold: HoldReading, state: RobotState) -> ContactReading:
        """Return a ContactReading for the given hold and robot state."""
        servo_ids = sorted({m for blob in hold.q_blobs for m in blob.modules if m >= 1})

        # --- WRENCH hysteresis ---
        # Enter: joint clearly displaced from commanded position AND motor resisting.
        # Exit: joint returns near commanded position (motor succeeded or released).
        for sid in servo_ids:
            commanded = state.servo_commanded_positions.get(sid, 0.0)
            displacement = abs(state.servo_positions.get(sid, 0.0) - commanded)
            torque = abs(state.motor_torques.get(sid, 0.0))
            if sid in self._wrench_active:
                if displacement < self.WRENCH_POS_OFF_RAD:
                    self._wrench_active.discard(sid)
            else:
                if displacement > self.WRENCH_POS_ON_RAD and torque > self.WRENCH_TORQUE_NM:
                    self._wrench_active.add(sid)
        self._wrench_active &= set(servo_ids)

        wrench_servos = sorted(self._wrench_active)
        if wrench_servos:
            torque_peak = max(abs(state.motor_torques.get(s, 0.0)) for s in wrench_servos)
            return ContactReading(
                hold=hold,
                contact_type=ContactType.WRENCH,
                affected_servos=wrench_servos,
                torque_peak=torque_peak,
            )

        # --- RESTRICT hysteresis ---
        # Enter: high torque, near-zero velocity (motor stalled under load).
        # Exit: torque drops OR velocity rises (load removed or joint moves).
        for sid in servo_ids:
            t = abs(state.motor_torques.get(sid, 0.0))
            v = abs(state.motor_velocities.get(sid, 0.0))
            if sid in self._restrict_active:
                if t < self.RESTRICT_TORQUE_OFF or v > self.RESTRICT_VEL_OFF:
                    self._restrict_active.discard(sid)
            else:
                if t > self.RESTRICT_TORQUE_ON and v < self.RESTRICT_VEL_ON:
                    self._restrict_active.add(sid)
        self._restrict_active &= set(servo_ids)

        restrict_servos = sorted(self._restrict_active)
        if restrict_servos:
            torque_peak = max(abs(state.motor_torques.get(s, 0.0)) for s in restrict_servos)
            return ContactReading(
                hold=hold,
                contact_type=ContactType.RESTRICT,
                affected_servos=restrict_servos,
                torque_peak=torque_peak,
            )

        # --- SQUEEZE ---
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
