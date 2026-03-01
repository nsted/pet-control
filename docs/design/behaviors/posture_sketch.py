"""
Posture behaviors — go_limp and stiffen.

These don't react to touch directly.  They modify the robot's
overall "muscle tone" by driving all joints toward a target posture.

go_limp:   All joints drift toward 0° (neutral hanging).
           The agent uses this for exhaustion, submission, sleep,
           or "giving up."  At low intensity it's relaxed; at high
           intensity it's dramatic collapse.

stiffen:   All joints resist movement — holds current position firmly.
           Implemented by outputting the robot's CURRENT angles as the
           target, effectively fighting any other behavior's contribution.
           The agent uses this for alertness, fear-freeze, or bracing.

Both work through the blending system: go_limp at weight 0.3 mixed
with undulate at weight 0.7 produces a robot that's gently swaying
but with reduced amplitude — it looks drowsy.
"""

from __future__ import annotations

from petctl.behaviors.engine import Behavior, BehaviorParams
from petctl.types import RobotState, ServoCommand

_NUM_MODULES = 8


class GoLimpBehavior(Behavior):
    """Drive all joints toward neutral (0°)."""

    name = "go_limp"

    def __init__(self, return_speed: float = 10.0) -> None:
        """
        Args:
            return_speed:  Max degrees per second toward neutral.
                           Slow = gentle settling.  Fast = dramatic drop.
        """
        self.return_speed = return_speed
        self._targets: dict[int, float] = {}

    def update(
        self,
        state: RobotState,
        params: BehaviorParams,
        dt: float,
    ) -> dict[int, float]:
        contributions: dict[int, float] = {}
        speed = self.return_speed * (0.2 + params.speed * 4.0)

        for mod_id in range(_NUM_MODULES):
            # Read current angle from servo position
            raw = state.servo_positions.get(mod_id, 2048)
            current_deg = ServoCommand.position_to_angle(raw)

            # Drive toward 0° at controlled speed
            target = self._targets.get(mod_id, 0.0)
            diff = target - current_deg
            max_step = speed * dt * params.intensity
            if abs(diff) < max_step:
                contributions[mod_id] = target
            else:
                step = max_step if diff > 0 else -max_step
                contributions[mod_id] = current_deg + step

        return contributions

    def reset(self) -> None:
        self._targets.clear()


class StiffenBehavior(Behavior):
    """Hold current pose — resist other behaviors.

    Works by outputting the robot's current position as a strong
    contribution.  When blended with other behaviors, the stiffness
    weight determines how much the robot resists being moved.

    At full intensity + weight, the robot locks in place.
    At partial weight, it moves but sluggishly — it looks tense.
    """

    name = "stiffen"

    def __init__(self) -> None:
        self._locked_pose: dict[int, float] = {}
        self._pose_captured = False

    def update(
        self,
        state: RobotState,
        params: BehaviorParams,
        dt: float,
    ) -> dict[int, float]:
        # Capture pose on first call (or re-capture if no pose yet)
        if not self._pose_captured:
            for mod_id in range(_NUM_MODULES):
                raw = state.servo_positions.get(mod_id, 2048)
                self._locked_pose[mod_id] = ServoCommand.position_to_angle(raw)
            self._pose_captured = True

        # Output the locked pose — blending weight controls stiffness
        return {
            mod_id: angle * params.intensity
            for mod_id, angle in self._locked_pose.items()
        }

    def reset(self) -> None:
        self._locked_pose.clear()
        self._pose_captured = False
