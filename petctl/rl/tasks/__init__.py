"""Task registry: add a module here (subclass ``Task``) to define a new experiment."""

from __future__ import annotations

from petctl.rl.tasks.base import Task
from petctl.rl.tasks.locomote import Locomote

TASKS: dict[str, type[Task]] = {"locomote": Locomote}

__all__ = ["TASKS", "Task", "Locomote"]
