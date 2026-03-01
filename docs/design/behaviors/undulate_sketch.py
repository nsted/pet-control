"""
Undulate behavior — continuous wave motion along the body.

This is the robot's "breathing" or "idle fidget."  A sine wave
travels down the chain, giving the body a constant sense of
aliveness even when no touch is happening.

The agent controls:
    - intensity: amplitude of the wave (subtle breathing vs dramatic writhing)
    - speed: frequency (calm slow breathing vs agitated fast oscillation)
    - extras["wavelength"]: how many modules per wave cycle (default: full body)
    - extras["asymmetry"]: 0.0 = pure symmetric, 1.0 = biased to one side

Undulate is designed to be ALWAYS active as a base layer, with
other behaviors (nestle, recoil) layered on top.  At low intensity
it provides subtle life; at high intensity it becomes writhing.
"""

from __future__ import annotations

import math
from petctl.behaviors.engine import Behavior, BehaviorParams
from petctl.types import RobotState

_NUM_MODULES = 8


class UndulateBehavior(Behavior):
    """Traveling wave along the body chain."""

    name = "undulate"

    def __init__(
        self,
        base_amplitude: float = 15.0,
        base_frequency: float = 0.3,
    ) -> None:
        """
        Args:
            base_amplitude:  Peak angle (degrees) at intensity=1.0.
            base_frequency:  Wave cycles per second at speed=0.5.
        """
        self.base_amplitude = base_amplitude
        self.base_frequency = base_frequency
        self._phase: float = 0.0

    def update(
        self,
        state: RobotState,
        params: BehaviorParams,
        dt: float,
    ) -> dict[int, float]:
        # Map speed param (0-1) to frequency.
        # 0.0 → very slow breathing, 1.0 → fast agitated oscillation
        freq = self.base_frequency * (0.2 + params.speed * 3.0)
        amplitude = self.base_amplitude * params.intensity

        # Wavelength: how many modules per full sine cycle
        wavelength = float(params.extras.get("wavelength", _NUM_MODULES))
        asymmetry = float(params.extras.get("asymmetry", 0.0))

        self._phase += freq * dt * 2.0 * math.pi

        contributions: dict[int, float] = {}
        for mod_id in range(_NUM_MODULES):
            # Phase offset along the chain → traveling wave
            spatial_phase = (mod_id / wavelength) * 2.0 * math.pi
            angle = amplitude * math.sin(self._phase + spatial_phase)

            # Asymmetry biases the wave to one side (for "leaning" moods)
            angle += asymmetry * amplitude * 0.3

            contributions[mod_id] = angle

        return contributions

    def reset(self) -> None:
        self._phase = 0.0
