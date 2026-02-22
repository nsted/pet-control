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

    def set_position(self, servo_id: int, position: int, acceleration: int = 50) -> None:
        """Queue a raw position command (0-4095)."""
        with self._lock:
            self._queue.append(
                ServoCommand(servo_id=servo_id, position=position, acceleration=acceleration)
            )

    def set_angle(self, servo_id: int, angle_deg: float, acceleration: int = 50) -> None:
        """Queue a position command by angle in degrees [-180, 180]."""
        with self._lock:
            self._queue.append(
                ServoCommand.from_angle(servo_id, angle_deg, acceleration)
            )

    def set_speed(self, servo_id: int, speed: int, acceleration: int = 50) -> None:
        """Queue a continuous-rotation speed command (signed integer)."""
        with self._lock:
            self._queue.append(
                ServoCommand(servo_id=servo_id, speed=speed, acceleration=acceleration)
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
