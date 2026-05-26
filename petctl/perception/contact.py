"""
Contact sub-classifier — enriches a HoldReading with motor-state context.

Classification chains
  One centroid:   TOUCH → STROKE | SQUEEZE | RESTRICT | BUDGE | TWIST
  Two centroids:  TOUCH → STROKE | HOLD → SQUEEZE | RESTRICT | BUDGE | TWIST | WRENCH

Sub-type semantics
  TOUCH    — one hand, static, no special condition.
  SQUEEZE  — any centroid count, meaningful FSR pressure.
  RESTRICT — any centroid count, motor stalled under load (high torque, near-zero velocity).
  HOLD     — two hands, static, no motor event.
  BUDGE    — any contact, brief or small joint displacement (travel < TWIST_MIN_TRAVEL_RAD).
  TWIST    — any contact, joint rotating passively with meaningful travel (≥ TWIST_MIN_TRAVEL_RAD).
  WRENCH   — two hands, joint displaced from commanded position while motor resists.

WRENCH, RESTRICT, and TWIST use per-servo hysteresis.
BUDGE is promoted to TWIST once cumulative travel threshold is met.

When multiple conditions are met simultaneously the priority is:
  WRENCH > RESTRICT > TWIST > SQUEEZE > HOLD > TOUCH > BUDGE.
All thresholds are initial guesses — tune on real hardware.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from petctl.perception.stroke import HoldReading
from petctl.types import RobotState

# Fixed-width labels for console log lines, keyed by contact type string.
CONTACT_LABELS: dict[str, str] = {
    "touch":    "TOUCH   ",
    "stroke":   "STROKE  ",
    "hold":     "HOLD    ",
    "squeeze":  "SQUEEZE ",
    "restrict": "RESTRICT",
    "budge":    "BUDGE   ",
    "twist":    "TWIST   ",
    "wrench":   "WRENCH  ",
    "cradle":   "CRADLE  ",
}

class ContactType(str, Enum):
    TOUCH    = "touch"
    HOLD     = "hold"
    SQUEEZE  = "squeeze"
    RESTRICT = "restrict"
    BUDGE    = "budge"
    TWIST    = "twist"
    WRENCH   = "wrench"
    CRADLE   = "cradle"


@dataclass
class ContactReading:
    """Contact classification with optional hold/motor context.

    For TOUCH: centroid and side describe the pad activity; hold may be populated.
    For all other types: hold is populated.
    """
    contact_type: ContactType
    hold: HoldReading | None = None
    centroid: float | None = None
    side: str = ""
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

    # RESTRICT: motor stalled under load (any centroid count).
    RESTRICT_TORQUE_ON: float = 0.5
    RESTRICT_TORQUE_OFF: float = 0.25
    RESTRICT_VEL_ON: float = 0.12
    RESTRICT_VEL_OFF: float = 0.22

    # TWIST/BUDGE: joint rotating passively (any centroid count).
    TWIST_VEL_ON: float = 0.3          # rad/s — enter twist/budge when velocity exceeds this
    TWIST_VEL_OFF: float = 0.15        # rad/s — exit twist/budge when velocity drops below this
    TWIST_TORQUE_MAX: float = 0.25     # Nm — above this the motor is actively driven, not passive
    TWIST_POS_ERR_MIN_RAD: float = 0.12  # rad — actual must deviate from commanded to enter twist/budge
    BUDGE_MIN_TRAVEL_RAD: float = 0.1  # rad — minimum travel before BUDGE fires
    TWIST_MIN_TRAVEL_RAD: float = 0.3  # rad — cumulative travel to promote BUDGE → TWIST

    SQUEEZE_PRESSURE: float = 0.15

    def __init__(self) -> None:
        self._wrench_active: set[int] = set()
        self._restrict_active: set[int] = set()
        self._twist_active: set[int] = set()
        self._twist_promoted: set[int] = set()  # servos that have crossed TWIST_MIN_TRAVEL_RAD
        self._servo_travel: dict[int, float] = {}
        self._servo_last_pos: dict[int, float] = {}

    def has_active_motion(self) -> bool:
        """True if any servo is currently above the velocity exit threshold."""
        return bool(self._twist_active)

    def reset(self) -> None:
        """Clear hysteresis state. Call when contact ends."""
        self._wrench_active.clear()
        self._restrict_active.clear()
        self._twist_active.clear()
        self._twist_promoted.clear()
        self._servo_travel.clear()
        self._servo_last_pos.clear()

    def classify_no_hold(self, state: RobotState) -> ContactReading | None:
        """Check for BUDGE/TWIST from motor state alone — no qualifying hold required.

        Returns a ContactReading (hold=None) if any servo is moving above the velocity
        threshold, else None.  WRENCH/RESTRICT/SQUEEZE are not checked.
        """
        servo_ids = sorted(sid for sid in state.motor_velocities if sid >= 1)
        if not servo_ids:
            return None

        if state.is_motion_active:
            self._twist_active.clear()
            self._servo_travel.clear()
            self._servo_last_pos.clear()
            self._twist_promoted.clear()
            return None

        for sid in servo_ids:
            pos = state.servo_positions.get(sid, 0.0)
            cmd = state.servo_commanded_positions.get(sid, pos)
            is_passive = abs(pos - cmd) >= self.TWIST_POS_ERR_MIN_RAD

            if sid in self._servo_last_pos and is_passive:
                self._servo_travel[sid] = self._servo_travel.get(sid, 0.0) + abs(pos - self._servo_last_pos[sid])
            self._servo_last_pos[sid] = pos

            v = abs(state.motor_velocities.get(sid, 0.0))
            t = abs(state.motor_torques.get(sid, 0.0))
            if sid in self._twist_active:
                if v < self.TWIST_VEL_OFF or t > self.TWIST_TORQUE_MAX:
                    self._twist_active.discard(sid)
                    self._servo_travel.pop(sid, None)
            else:
                if v >= self.TWIST_VEL_ON and t <= self.TWIST_TORQUE_MAX and is_passive:
                    self._twist_active.add(sid)
        self._twist_active &= set(servo_ids)

        if not self._twist_active:
            return None

        active_servos = sorted(self._twist_active)
        for s in active_servos:
            if self._servo_travel.get(s, 0.0) >= self.TWIST_MIN_TRAVEL_RAD:
                self._twist_promoted.add(s)
        if self._twist_promoted & set(active_servos):
            return ContactReading(contact_type=ContactType.TWIST, affected_servos=active_servos)
        max_travel = max(self._servo_travel.get(s, 0.0) for s in active_servos)
        if max_travel < self.BUDGE_MIN_TRAVEL_RAD:
            return None  # velocity active but not enough travel yet
        return ContactReading(contact_type=ContactType.BUDGE, affected_servos=active_servos)

    def classify(self, hold: HoldReading, state: RobotState) -> ContactReading:
        """Return a ContactReading for the given hold and robot state."""
        servo_ids = sorted({m for blob in hold.q_blobs for m in blob.modules if m >= 1})
        two_centroids = len(hold.q_blobs) >= 2

        # --- WRENCH hysteresis (two centroids required) ---
        # Always update hysteresis state even when not returning WRENCH, so exit
        # conditions are evaluated correctly when centroid count drops below 2.
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

        if two_centroids and self._wrench_active:
            wrench_servos = sorted(self._wrench_active)
            torque_peak = max(abs(state.motor_torques.get(s, 0.0)) for s in wrench_servos)
            return ContactReading(
                hold=hold,
                contact_type=ContactType.WRENCH,
                affected_servos=wrench_servos,
                torque_peak=torque_peak,
            )

        # --- RESTRICT hysteresis (any centroid count) ---
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

        if self._restrict_active:
            restrict_servos = sorted(self._restrict_active)
            torque_peak = max(abs(state.motor_torques.get(s, 0.0)) for s in restrict_servos)
            return ContactReading(
                hold=hold,
                contact_type=ContactType.RESTRICT,
                affected_servos=restrict_servos,
                torque_peak=torque_peak,
            )

        # --- TWIST/BUDGE hysteresis (passive joint rotation; any centroid count) ---
        # Motor velocity is the primary signal — blob geometry is unreliable when both
        # hands grip close together or one grip is too light to qualify as a second blob.
        # WRENCH and RESTRICT already handle high-torque cases above.
        # Cumulative travel per servo distinguishes a real twist (≥ TWIST_MIN_TRAVEL_RAD)
        # from a brief jostle (BUDGE).
        # Suppressed during autonomous motion (is_motion_active) so the robot's own
        # joint movement is not misread as a human twist gesture.
        if state.is_motion_active:
            self._twist_active.clear()
            self._servo_travel.clear()
            self._servo_last_pos.clear()
            self._twist_promoted.clear()
        else:
            for sid in servo_ids:
                pos = state.servo_positions.get(sid, 0.0)
                cmd = state.servo_commanded_positions.get(sid, pos)
                is_passive = abs(pos - cmd) >= self.TWIST_POS_ERR_MIN_RAD

                if sid in self._servo_last_pos and is_passive:
                    self._servo_travel[sid] = self._servo_travel.get(sid, 0.0) + abs(pos - self._servo_last_pos[sid])
                self._servo_last_pos[sid] = pos

                v = abs(state.motor_velocities.get(sid, 0.0))
                if sid in self._twist_active:
                    if v < self.TWIST_VEL_OFF:
                        self._twist_active.discard(sid)
                        self._servo_travel.pop(sid, None)
                else:
                    if v >= self.TWIST_VEL_ON and is_passive:
                        self._twist_active.add(sid)
            self._twist_active &= set(servo_ids)

            if self._twist_active:
                active_servos = sorted(self._twist_active)
                for s in active_servos:
                    if self._servo_travel.get(s, 0.0) >= self.TWIST_MIN_TRAVEL_RAD:
                        self._twist_promoted.add(s)

        # --- SQUEEZE (any centroid count, one blob's pressure sufficient) ---
        held_modules = {m for blob in hold.q_blobs for m in blob.modules}
        pressure_peak = max(
            (state.sensors[m].pressure_total for m in held_modules if m in state.sensors),
            default=0.0,
        )
        blob_pressure_max = max(
            (
                sum(state.sensors[m].pressure_total for m in blob.modules if m in state.sensors)
                for blob in hold.q_blobs
            ),
            default=0.0,
        )
        if blob_pressure_max > self.SQUEEZE_PRESSURE:
            return ContactReading(
                hold=hold,
                contact_type=ContactType.SQUEEZE,
                pressure_peak=pressure_peak,
            )

        # --- HOLD (two centroids) / TOUCH fallback (one centroid) ---
        if two_centroids:
            return ContactReading(hold=hold, contact_type=ContactType.HOLD)
        return ContactReading(
            hold=hold,
            contact_type=ContactType.TOUCH,
            centroid=hold.centroid,
            side=hold.side,
        )
