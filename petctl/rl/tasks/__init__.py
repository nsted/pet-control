"""Task registry: add a module here (subclass ``Task``) to define a new experiment."""

from __future__ import annotations

from petctl.rl.tasks.base import Task
from petctl.rl.tasks.locomote import Locomote
from petctl.rl.tasks.walk_straight import WalkStraight

TASKS: dict[str, type[Task]] = {"locomote": Locomote, "walk_straight": WalkStraight}

__all__ = ["TASKS", "Task", "Locomote", "WalkStraight"]
