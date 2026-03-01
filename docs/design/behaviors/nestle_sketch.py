"""
Nestle behavior — move toward active touch contact.

The core idea: each module computes a "touch pull" from its sensor
readings.  Left touch pulls the joint in one direction, right touch
pulls it the other way.  Adjacent modules curve sympathetically so
the whole body arcs toward the contact point.

This is NOT a trajectory replay — it's a continuous reactive policy
that responds to sensor data every tick.  If the user moves their
hand along the body, the nestle follows.  If they press harder,
the nestle deepens.

Joint geometry:
    All joints are single-axis revolute (Z-axis).
    Positive angle → bend one way, negative → the other.
    Left/right touch maps directly to joint direction.
    Middle touch (back face) drives a "curl inward" response
    by bending both the contacted module and its neighbors
    symmetrically — the body folds around the touch point.

Neighbor influence:
    Touch on module N also affects modules N-1 and N+1 (and
    further with falloff).  This creates a smooth arc rather
    than a single kinked joint.  The spread and falloff are
    tunable via the `spread` and `falloff` parameters.

    Think of it like a hand pressing on a flexible rod — the
    deformation isn't just at the contact point, it curves
    the whole region.
"""

from __future__ import annotations

import math
from petctl.behaviors.engine import Behavior, BehaviorParams
from petctl.types import RobotState

# How many modules exist in the chain
_NUM_MODULES = 8


class NestleBehavior(Behavior):
    """Reactive nestle — body curves toward touch."""

    name = "nestle"

    def __init__(
        self,
        gain: float = 30.0,
        spread: int = 2,
        falloff: float = 0.5,
        middle_curl_gain: float = 15.0,
    ) -> None:
        """
        Args:
            gain:              Max angle contribution (degrees) at full
                               touch + full intensity.  This is the peak
                               bend for the directly-contacted module.
            spread:            How many neighbors on each side are influenced.
            falloff:           Multiplier per step away from contact.
                               0.5 = neighbor gets half the effect, next
                               neighbor gets quarter, etc.
            middle_curl_gain:  Angle contribution for middle (back face)
                               touch.  Applied symmetrically to neighbors
                               to create a "curl around the finger" effect.
        """
        self.gain = gain
        self.spread = spread
        self.falloff = falloff
        self.middle_curl_gain = middle_curl_gain

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

            # --- Lateral touch: left/right → directional bend ---
            # Left touch  → positive angle (bend toward left)
            # Right touch → negative angle (bend toward right)
            # Net effect: body curves toward whichever side is being touched.
            lateral = sensors.touch_left - sensors.touch_right

            # Weight by pressure too — pressing harder deepens the nestle.
            # Use max of touch and pressure so either modality works alone.
            lateral_strength = max(
                abs(lateral),
                abs(sensors.pressure_left - sensors.pressure_right)
            )

            # Sign from touch direction, magnitude from combined strength
            if abs(lateral) > 0.01:
                lateral_signed = math.copysign(lateral_strength, lateral)
            else:
                lateral_signed = 0.0

            angle_local = lateral_signed * self.gain * params.intensity

            # --- Middle touch: curl inward ---
            # Back face touch can't map to a single joint direction, so
            # we create a "cupping" effect: the contacted module stays
            # neutral but its neighbors bend inward (toward each other),
            # wrapping the body around the touch point.
            middle_strength = max(sensors.touch_middle, sensors.pressure_middle)
            curl = middle_strength * self.middle_curl_gain * params.intensity

            # --- Apply to self + neighbors with falloff ---
            for offset in range(-self.spread, self.spread + 1):
                neighbor = mod_id + offset
                if neighbor < 0 or neighbor >= _NUM_MODULES:
                    continue

                dist = abs(offset)
                weight = self.falloff ** dist  # 1.0 at center, decays outward

                # Lateral contribution
                contributions[neighbor] = (
                    contributions.get(neighbor, 0.0) + angle_local * weight
                )

                # Middle curl: neighbors bend inward toward the contact.
                # Module below contact bends positive, module above bends
                # negative → they converge, cupping the touch point.
                if offset != 0 and curl > 0.01:
                    # Sign: bend toward the contacted module
                    curl_sign = 1.0 if offset < 0 else -1.0
                    contributions[neighbor] = (
                        contributions.get(neighbor, 0.0)
                        + curl * curl_sign * weight
                    )

        # If focus_modules is set, zero out everything else
        if params.focus_modules:
            focus = set(params.focus_modules)
            # Expand focus to include neighbors within spread
            expanded = set()
            for m in focus:
                for offset in range(-self.spread, self.spread + 1):
                    n = m + offset
                    if 0 <= n < _NUM_MODULES:
                        expanded.add(n)
            contributions = {
                k: v for k, v in contributions.items() if k in expanded
            }

        return contributions
