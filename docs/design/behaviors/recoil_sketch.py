"""
Recoil behavior — pull away from touch contact.

The inverse of nestle: each module bends AWAY from the touched face.
Intensity controls how far, speed controls how sharp the initial
flinch is (fast onset, slower recovery).

Includes a "startle" component: on initial contact, the recoil
overshoots briefly then settles to a sustained avoidance posture.
This gives the flinch a natural, organic quality rather than a
mechanical linear response.
"""

from __future__ import annotations

import math
from petctl.behaviors.engine import Behavior, BehaviorParams
from petctl.types import RobotState

_NUM_MODULES = 8


class RecoilBehavior(Behavior):
    """Reactive recoil — body flinches away from touch."""

    name = "recoil"

    def __init__(
        self,
        gain: float = 35.0,
        spread: int = 2,
        falloff: float = 0.6,
        startle_overshoot: float = 1.8,
        startle_decay: float = 3.0,
    ) -> None:
        self.gain = gain
        self.spread = spread
        self.falloff = falloff
        self.startle_overshoot = startle_overshoot
        self.startle_decay = startle_decay

        # Track per-module startle state
        self._startle: dict[int, float] = {}  # remaining overshoot multiplier
        self._prev_touch: dict[int, float] = {}  # previous total touch

    def update(
        self,
        state: RobotState,
        params: BehaviorParams,
        dt: float,
    ) -> dict[int, float]:
        contributions: dict[int, float] = {}

        for mod_id in range(_NUM_MODULES):
            sensors = state.sensors.get(mod_id)
            if sensors is None:
                continue

            # Recoil = opposite of nestle: bend AWAY from touch
            lateral = sensors.touch_left - sensors.touch_right
            angle_local = -lateral * self.gain * params.intensity

            # Middle touch → extend outward (opposite of curl)
            middle = max(sensors.touch_middle, sensors.pressure_middle)
            # Push neighbors apart rather than together
            extend = middle * self.gain * 0.5 * params.intensity

            # Startle detection: sudden increase in total touch
            total_touch = sensors.touch_total
            prev = self._prev_touch.get(mod_id, 0.0)
            onset = max(0.0, total_touch - prev - 0.1)  # threshold
            self._prev_touch[mod_id] = total_touch

            if onset > 0.05:
                self._startle[mod_id] = self.startle_overshoot

            # Decay startle
            startle = self._startle.get(mod_id, 1.0)
            if startle > 1.0:
                startle = max(1.0, startle - self.startle_decay * dt)
                self._startle[mod_id] = startle

            # Apply startle multiplier
            angle_local *= startle

            # Spread to neighbors
            for offset in range(-self.spread, self.spread + 1):
                neighbor = mod_id + offset
                if neighbor < 0 or neighbor >= _NUM_MODULES:
                    continue
                weight = self.falloff ** abs(offset)
                contributions[neighbor] = (
                    contributions.get(neighbor, 0.0) + angle_local * weight
                )
                # Middle extension: push neighbors apart
                if offset != 0 and extend > 0.01:
                    push_sign = -1.0 if offset < 0 else 1.0
                    contributions[neighbor] = (
                        contributions.get(neighbor, 0.0)
                        + extend * push_sign * weight
                    )

        return contributions

    def reset(self) -> None:
        self._startle.clear()
        self._prev_touch.clear()
