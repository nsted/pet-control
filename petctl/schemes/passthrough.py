"""
PassthroughControlScheme — programmatic servo control.

Set servo positions or angles from outside the control loop, e.g.:
  scheme = PassthroughControlScheme()
  scheme.set_angle(1, 30.0)
  scheme.set_angle(2, -15.0)
  # On the next tick, Controller will send those commands

Useful for:
  - Scripted sequences
  - Jupyter notebook interactive control
  - As a base class for more complex schemes
"""

from __future__ import annotations

import threading
from collections import deque

from petctl.protocols import ControlScheme
from petctl.types import RobotState, ServoCommand


class PassthroughControlScheme(ControlScheme):
    """
    Queues commands set externally and emits them on the next tick.

    Thread-safe: you can call set_angle() from any thread (e.g. a Jupyter
    cell) and it will be picked up by the async control loop.
    """

    name = "passthrough"

    def __init__(self) -> None:
        self._queue: deque[ServoCommand] = deque()
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # External API — call these from outside the loop
    # ------------------------------------------------------------------

    def set_position(self, servo_id: int, position: float, kp: float = 1.0, kd: float = 0.01, torque_ff: float = 0.0) -> None:
        """Queue a position command in radians."""
        with self._lock:
            self._queue.append(
                ServoCommand(servo_id=servo_id, position=position, kp=kp, kd=kd, torque_ff=torque_ff)
            )

    def set_angle(self, servo_id: int, angle_deg: float, kp: float = 1.0, kd: float = 0.01) -> None:
        """Queue a position command by angle in degrees."""
        with self._lock:
            self._queue.append(
                ServoCommand.from_angle(servo_id, angle_deg, kp=kp, kd=kd)
            )

    def clear(self) -> None:
        """Discard all queued commands."""
        with self._lock:
            self._queue.clear()

    # ------------------------------------------------------------------
    # ControlScheme interface
    # ------------------------------------------------------------------

    def update(self, state: RobotState) -> list[ServoCommand]:
        with self._lock:
            commands = list(self._queue)
            self._queue.clear()
        return commands
