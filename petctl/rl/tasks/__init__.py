"""Task registry: add a module here (subclass ``Task``) to define a new experiment."""

from __future__ import annotations

from petctl.rl.tasks.base import Task
from petctl.rl.tasks.goal_reach import GoalReach
from petctl.rl.tasks.goal_wall import GoalWall
from petctl.rl.tasks.goal_sequence import GoalSequence
from petctl.rl.tasks.jump import Jump
from petctl.rl.tasks.locomote import Locomote
from petctl.rl.tasks.reach_still import ReachStill
from petctl.rl.tasks.reach_up import ReachUp
from petctl.rl.tasks.walk_sideways import WalkSideways
from petctl.rl.tasks.walk_straight import WalkStraight

TASKS: dict[str, type[Task]] = {"locomote": Locomote, "walk_straight": WalkStraight, "walk_sideways": WalkSideways, "reach_up": ReachUp, "reach_still": ReachStill, "jump": Jump, "goal_reach": GoalReach, "goal_sequence": GoalSequence, "goal_wall": GoalWall}

__all__ = ["TASKS", "Task", "Locomote", "WalkStraight", "WalkSideways", "ReachUp", "ReachStill", "Jump", "GoalReach", "GoalSequence", "GoalWall"]
