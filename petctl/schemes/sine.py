"""
SineControlScheme — drives all servos in a staggered sine wave.

Oscillates each servo ±amplitude_deg around its home position (raw 0),
with phases evenly distributed across the active servo set so the robot
produces a travelling-wave body motion.

Pass servo_id to target a single servo instead of all active servos.

Works with any backend:
  petctl run --backend mock   --control sine
  petctl run --backend robot  --control sine
  petctl run --backend robot  --control sine --servo-id 3

Press Ctrl-C to stop.
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING, Optional

from petctl.config import SERVO_LIMITS
from petctl.protocols import ControlScheme
from petctl.types import RobotState, ServoCommand

if TYPE_CHECKING:
    from petctl.controller import Controller


def compute_sine_positions(
    servo_ids: list[int],
    elapsed: float,
    amplitude_deg: float,
    hz: float,
) -> dict[int, int]:
    """
    Compute staggered sine-wave raw positions for a list of servo IDs.

    Returns a dict mapping servo_id → raw position (signed, centered on 0 = home).
    Phases are evenly distributed across the servo list order.
    """
    n = len(servo_ids)
    if n == 0:
        return {}
    amplitude_raw = int(SERVO_LIMITS.ticks_per_rotation * amplitude_deg / 360)
    return {
        servo_id: int(amplitude_raw * math.sin(2 * math.pi * hz * elapsed + (i / n) * 2 * math.pi))
        for i, servo_id in enumerate(servo_ids)
    }


class SineControlScheme(ControlScheme):
    """
    Travelling sine-wave control scheme.

    Args:
        amplitude_deg:  Peak deflection from home in degrees (default: 15°).
        hz:             Oscillation frequency in Hz (default: 0.2 Hz = 5 s period).
        servo_id:       If set, only this servo is driven. If None, all active
                        servos are driven with staggered phases.
    """

    name = "sine"

    def __init__(
        self,
        amplitude_deg: float = 30.0,
        hz: float = 0.2,
        servo_id: Optional[int] = None,
    ) -> None:
        self.amplitude_deg = amplitude_deg
        self.hz = hz
        self.servo_id = servo_id
        self._start_time: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start_time = time.monotonic()
        target = f"servo {self.servo_id}" if self.servo_id is not None else "all servos"
        print(f"[Sine] ±{self.amplitude_deg}° at {self.hz} Hz on {target}. Ctrl-C to stop.")

    def update(self, state: RobotState) -> list[ServoCommand]:
        elapsed = time.monotonic() - self._start_time
        if self.servo_id is not None:
            ids = [self.servo_id] if self.servo_id in state.active_servo_ids else []
        else:
            ids = sorted(state.active_servo_ids)
        positions = compute_sine_positions(ids, elapsed, self.amplitude_deg, self.hz)
        return [ServoCommand(servo_id=sid, position=raw) for sid, raw in positions.items()]
