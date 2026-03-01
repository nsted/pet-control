"""
Behavior primitives and blending engine.

The BehaviorEngine is a ControlScheme that manages a stack of active
behaviors, blends their outputs, and provides the interface that the
agent layer talks to.

Architecture:
    Agent  →  BehaviorEngine.request("nestle", intensity=0.7, ...)
                    ↓
              BehaviorEngine.update(state)
                    ↓
              blend active behaviors → list[ServoCommand]
                    ↓
              Controller sends to robot
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional

from petctl.protocols import ControlScheme
from petctl.types import RobotState, ServoCommand

if TYPE_CHECKING:
    from petctl.controller import Controller


# ---------------------------------------------------------------------------
# Behavior params — the vocabulary the agent speaks
# ---------------------------------------------------------------------------

@dataclass
class BehaviorParams:
    """Parameters the agent passes when requesting a behavior.

    These are behavior-agnostic — each Behavior reads what it needs
    and ignores the rest.
    """
    intensity: float = 0.5          # 0.0–1.0, how strong the response
    speed: float = 0.5              # 0.0–1.0, how fast to transition
    # Which modules are the focus (e.g. where touch is happening).
    # Empty = whole body.
    focus_modules: list[int] = field(default_factory=list)
    # Which sensor face triggered this (for directional behaviors).
    # "left", "right", "middle", or None for non-directional.
    contact_face: Optional[str] = None
    # Arbitrary extras for behavior-specific tuning
    extras: dict = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Base class for all behaviors
# ---------------------------------------------------------------------------

class Behavior(ABC):
    """A single movement primitive.

    Behaviors produce per-module angle *contributions* (in degrees),
    not absolute positions.  The BehaviorEngine sums weighted contributions
    from all active behaviors and converts to ServoCommands.

    This additive model means behaviors compose naturally:
        nestle(+10° on mod 3) + undulate(±5° everywhere) = both at once
    """

    name: str = "unnamed"

    @abstractmethod
    def update(
        self,
        state: RobotState,
        params: BehaviorParams,
        dt: float,
    ) -> dict[int, float]:
        """Return per-module angle contributions in degrees.

        Keys are module IDs (0-7).  Missing modules = 0° contribution.
        Positive = bend in the joint's positive direction.
        """
        ...

    def reset(self) -> None:
        """Called when this behavior is deactivated."""
        ...


# ---------------------------------------------------------------------------
# BehaviorEngine — the ControlScheme that blends behaviors
# ---------------------------------------------------------------------------

@dataclass
class ActiveBehavior:
    """A currently running behavior with its weight and params."""
    behavior: Behavior
    params: BehaviorParams
    weight: float = 1.0             # blend weight, 0–1
    target_weight: float = 1.0      # weight we're fading toward
    fade_speed: float = 2.0         # weight change per second


class BehaviorEngine(ControlScheme):
    """Mixes active behaviors and outputs blended servo commands.

    The agent layer calls request() / cancel() to control what behaviors
    are active.  The engine handles blending, fading, and conversion to
    servo commands.

    Usage:
        engine = BehaviorEngine()
        engine.request("nestle", BehaviorParams(intensity=0.7, focus_modules=[3]))
        # ... later ...
        engine.cancel("nestle")  # fades out smoothly
    """

    name = "behavior_engine"

    def __init__(self, angle_limit: float = 45.0) -> None:
        self.angle_limit = angle_limit  # max absolute angle any behavior can produce
        self._behaviors: dict[str, Behavior] = {}  # registry
        self._active: dict[str, ActiveBehavior] = {}  # currently running
        self._base_angles: dict[int, float] = {}  # current output angles (smoothed)

    def register(self, behavior: Behavior) -> None:
        """Add a behavior to the available vocabulary."""
        self._behaviors[behavior.name] = behavior

    def request(self, name: str, params: Optional[BehaviorParams] = None) -> None:
        """Activate a behavior (or update its params if already active)."""
        if name not in self._behaviors:
            print(f"[BehaviorEngine] Unknown behavior: {name}")
            return
        params = params or BehaviorParams()
        if name in self._active:
            self._active[name].params = params
            self._active[name].target_weight = 1.0
        else:
            self._active[name] = ActiveBehavior(
                behavior=self._behaviors[name],
                params=params,
                weight=0.0,         # fade in from zero
                target_weight=1.0,
                fade_speed=params.speed * 4.0 + 0.5,  # faster speed param = faster fade-in
            )

    def cancel(self, name: str, fade_speed: float = 2.0) -> None:
        """Fade out and deactivate a behavior."""
        if name in self._active:
            self._active[name].target_weight = 0.0
            self._active[name].fade_speed = fade_speed

    def cancel_all(self, fade_speed: float = 2.0) -> None:
        for name in list(self._active):
            self.cancel(name, fade_speed)

    # ------------------------------------------------------------------
    # ControlScheme interface
    # ------------------------------------------------------------------

    def on_start(self, controller: Controller) -> None:
        pass

    def update(self, state: RobotState) -> list[ServoCommand]:
        dt = state.dt or 0.05  # fallback to 20Hz

        # 1. Update blend weights (fade in/out)
        to_remove = []
        for name, active in self._active.items():
            if active.weight < active.target_weight:
                active.weight = min(active.target_weight,
                                    active.weight + active.fade_speed * dt)
            elif active.weight > active.target_weight:
                active.weight = max(active.target_weight,
                                    active.weight - active.fade_speed * dt)
            # Remove fully faded-out behaviors
            if active.target_weight == 0.0 and active.weight <= 0.01:
                to_remove.append(name)
        for name in to_remove:
            self._active[name].behavior.reset()
            del self._active[name]

        # 2. Collect weighted angle contributions from all active behaviors
        blended: dict[int, float] = {}
        for name, active in self._active.items():
            contributions = active.behavior.update(state, active.params, dt)
            for mod_id, angle in contributions.items():
                blended[mod_id] = blended.get(mod_id, 0.0) + angle * active.weight

        # 3. Clamp and convert to servo commands
        commands: list[ServoCommand] = []
        for mod_id in range(8):
            target = blended.get(mod_id, 0.0)
            target = max(-self.angle_limit, min(self.angle_limit, target))

            # Smooth toward target (simple exponential filter)
            current = self._base_angles.get(mod_id, 0.0)
            alpha = min(1.0, 5.0 * dt)  # ~5 Hz smoothing cutoff
            smoothed = current + alpha * (target - current)
            self._base_angles[mod_id] = smoothed

            servo_id = mod_id  # servo_id == module_id in your setup
            if servo_id in state.active_servo_ids:
                commands.append(ServoCommand.from_angle(servo_id, smoothed))

        return commands

    def on_stop(self) -> None:
        for active in self._active.values():
            active.behavior.reset()
        self._active.clear()
