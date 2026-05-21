"""
CommandScheme — imperative movement API with touch-type callbacks.

Lets you command individual servos or all servos to target angles with
configurable speed and stiffness, and register callbacks that fire when
touch type transitions occur (stroke, hold, squeeze, restrict, wrench, idle).

Typical usage::

    from petctl.schemes.command import CommandScheme

    scheme = CommandScheme()

    @scheme.on_contact("stroke")
    def on_stroke(reading):
        scheme.move_all(30, speed=0.7)

    @scheme.on_contact("idle")
    def on_idle(_):
        scheme.home()

    # Pass scheme to Controller as normal.
"""

from __future__ import annotations

import math
import threading
from collections import defaultdict
from typing import TYPE_CHECKING, Any, Callable

from petctl.config import LOOP_LIMITS, MOTOR_LIMITS
from petctl.protocols import ControlScheme
from petctl.types import RobotState, ServoCommand

if TYPE_CHECKING:
    from petctl.controller import Controller


class CommandScheme(ControlScheme):
    """
    A control scheme that exposes an imperative movement API.

    Movements are commanded via move() / move_all() / home(). Each call
    sets a target angle; the scheme steps its internal position toward that
    target at the requested speed each tick, independent of the controller's
    slew filter.

    Touch callbacks registered via on_contact() fire on transitions between
    contact types (stroke, hold, squeeze, restrict, wrench, idle). Callbacks
    are invoked from the control loop thread inside update(), so they must be
    non-blocking. Calling move() or home() from inside a callback is safe.
    """

    name = "command"

    def __init__(self) -> None:
        from petctl.perception.contact import ContactClassifier
        from petctl.perception.stroke import HoldDetector, StrokeDetector

        self._stroke = StrokeDetector()
        self._hold = HoldDetector()
        self._clf = ContactClassifier()
        self._prev_contact: str | None = None

        self._callbacks: dict[str, list[Callable[[Any], None]]] = defaultdict(list)

        # Per-servo state — all protected by _lock (RLock for callback re-entry).
        self._lock = threading.RLock()
        self._target: dict[int, float] = {}        # radians
        self._current: dict[int, float] = {}       # radians — stepped each tick
        self._speed_rad_s: dict[int, float] = {}
        self._kp: dict[int, float] = {}
        self._kd: dict[int, float] = {}
        self._torque_ff: dict[int, float] = {}

        self._active_ids: set[int] = set()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def move(
        self,
        servo_id: int,
        angle_deg: float,
        *,
        speed: float = 1.0,
        kp: float | None = None,
        kd: float | None = None,
        torque_ff: float = 0.0,
    ) -> None:
        """Command servo_id to angle_deg.

        Args:
            servo_id:  MIT motor ID (1–7).
            angle_deg: Target angle in degrees (0 = home).
            speed:     Movement speed, 0–1 fraction of max_speed_rad_s (8 rad/s).
                       1.0 → 8 rad/s, 0.1 → 0.8 rad/s.
            kp:        Position stiffness gain. None → kp_default (0.4).
            kd:        Velocity damping gain. None → kd_default (0.035).
            torque_ff: Feed-forward torque (Nm).
        """
        target_rad = math.radians(angle_deg)
        speed_rad_s = max(0.001, speed) * LOOP_LIMITS.max_speed_rad_s
        with self._lock:
            self._target[servo_id] = target_rad
            self._speed_rad_s[servo_id] = speed_rad_s
            self._kp[servo_id] = kp if kp is not None else MOTOR_LIMITS.kp_default
            self._kd[servo_id] = kd if kd is not None else MOTOR_LIMITS.kd_default
            self._torque_ff[servo_id] = torque_ff

    def move_all(
        self,
        angle_deg: float,
        *,
        speed: float = 1.0,
        kp: float | None = None,
        kd: float | None = None,
        torque_ff: float = 0.0,
    ) -> None:
        """Command all active servos to angle_deg with the same parameters."""
        for sid in list(self._active_ids):
            self.move(sid, angle_deg, speed=speed, kp=kp, kd=kd, torque_ff=torque_ff)

    def home(self, servo_id: int | None = None) -> None:
        """Return servo(s) to 0° at default speed and stiffness.

        Args:
            servo_id: Motor ID to home, or None to home all active servos.
        """
        ids = [servo_id] if servo_id is not None else list(self._active_ids)
        for sid in ids:
            self.move(sid, 0.0)

    def on_contact(
        self,
        contact_type: str,
        callback: Callable[[Any], None],
    ) -> None:
        """Register a callback for a contact type transition.

        Args:
            contact_type: One of "stroke", "hold", "squeeze", "restrict",
                          "wrench", or "idle" (fired when contact ends).
            callback:     Called with the reading on transition start.
                          "idle" passes None. Must be non-blocking.
        """
        self._callbacks[contact_type].append(callback)

    # ------------------------------------------------------------------
    # ControlScheme interface
    # ------------------------------------------------------------------

    def on_start(self, controller: "Controller") -> None:
        self._active_ids = set(controller.state.active_servo_ids)
        print(
            "[CommandScheme] Ready.\n"
            "  Call scheme.move(servo_id, angle_deg, speed=..., kp=...) to move.\n"
            "  Call scheme.move_all(angle_deg) to move all servos.\n"
            "  Call scheme.home() to return to neutral.\n"
            "  Use scheme.on_contact(type, callback) to respond to touch."
        )

    def update(self, state: RobotState) -> list[ServoCommand]:
        self._active_ids = set(state.active_servo_ids)

        self._detect_and_fire(state)

        dt = state.dt if state.dt and state.dt > 0 else 0.033

        commands: list[ServoCommand] = []
        with self._lock:
            for sid in sorted(state.active_servo_ids):
                if sid not in self._target:
                    # No command yet — hold current reported position.
                    pos = state.servo_positions.get(sid, 0.0)
                    self._current.setdefault(sid, pos)
                    commands.append(ServoCommand(servo_id=sid, position=pos))
                    continue

                target = self._target[sid]
                current = self._current.get(sid, state.servo_positions.get(sid, 0.0))
                step = self._speed_rad_s[sid] * dt
                diff = target - current
                current = target if abs(diff) <= step else current + math.copysign(step, diff)
                self._current[sid] = current

                commands.append(ServoCommand(
                    servo_id=sid,
                    position=current,
                    kp=self._kp.get(sid, MOTOR_LIMITS.kp_default),
                    kd=self._kd.get(sid, MOTOR_LIMITS.kd_default),
                    torque_ff=self._torque_ff.get(sid, 0.0),
                ))

        return commands

    def on_stop(self) -> None:
        pass

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _detect_and_fire(self, state: RobotState) -> None:
        """Run touch detection and fire callbacks on contact type transitions."""
        from petctl.perception.contact import ContactReading

        stroke = self._stroke.update(state)
        hold = self._hold.update(state)

        if stroke is not None:
            curr = "stroke"
            if curr != self._prev_contact:
                self._fire(curr, stroke)
                self._clf.reset()
            self._prev_contact = curr
            return

        if hold is None:
            if self._prev_contact is not None:
                self._fire("idle", None)
            self._prev_contact = None
            self._clf.reset()
            return

        cr = self._clf.classify(hold, state)
        curr = cr.contact_type.value
        if curr != self._prev_contact:
            self._fire(curr, cr)
        self._prev_contact = curr

    def _fire(self, contact_type: str, reading: Any) -> None:
        for cb in self._callbacks.get(contact_type, []):
            try:
                cb(reading)
            except Exception as e:
                print(f"[CommandScheme] Callback error ({contact_type}): {e}")
