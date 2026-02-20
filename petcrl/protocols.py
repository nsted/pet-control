"""
Abstract base classes (plugin protocols) for petcrl.

Implement these to add new backends, control schemes, or visualizers:

  class MyScheme(ControlScheme):
      def update(self, state: RobotState) -> List[ServoCommand]:
          ...

  class MyBackend(RobotBackend):
      async def get_state(self) -> RobotState: ...
      async def send_commands(self, cmds): ...
      ...
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, List

if TYPE_CHECKING:
    from petcrl.controller import Controller
    from petcrl.types import RobotState, ServoCommand


# ---------------------------------------------------------------------------
# RobotBackend
# ---------------------------------------------------------------------------

class RobotBackend(ABC):
    """
    Abstraction over a physical robot or a simulator.

    The Controller talks exclusively through this interface, so swapping
    GrappleBackend for MockBackend or a future MuJoCoBackend requires
    zero changes to control schemes or visualizers.
    """

    @abstractmethod
    async def connect(self) -> bool:
        """
        Establish connection to robot or initialize simulator.
        Returns True on success.
        """
        ...

    @abstractmethod
    async def disconnect(self) -> None:
        """Clean up connection or simulator resources."""
        ...

    @abstractmethod
    async def get_state(self) -> "RobotState":
        """
        Return a fresh RobotState snapshot.

        Called every tick by the Controller. Must be non-blocking on the
        happy path. If the connection is lost, implementations should:
          - attempt reconnect (GrappleBackend with auto_reconnect=True)
          - return the last known state with connected=False
        """
        ...

    @abstractmethod
    async def send_commands(self, commands: "List[ServoCommand]") -> None:
        """
        Apply servo commands to the robot or simulator.

        Called every tick after update(). Implementations should silently
        skip if not connected (reconnect in progress).
        """
        ...

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """True while the backend has an active connection."""
        ...


# ---------------------------------------------------------------------------
# ControlScheme
# ---------------------------------------------------------------------------

class ControlScheme(ABC):
    """
    Base class for all control schemes.

    A ControlScheme receives RobotState every tick and returns a list of
    ServoCommands to send. Empty list = no movement this tick.

    Lifecycle:
        on_start(controller)  — called once when scheme becomes active
        update(state)         — called every tick; return commands
        on_stop()             — called on shutdown or scheme swap

    Example (ML scheme):
        class MyAI(ControlScheme):
            name = "my_ai"

            def on_start(self, controller):
                self.model = load_model("weights.pth")

            def update(self, state: RobotState) -> List[ServoCommand]:
                features = build_features(state)
                actions = self.model(features)
                return [ServoCommand.from_angle(i+1, a) for i, a in enumerate(actions)]
    """

    name: str = "unnamed"

    def on_start(self, controller: "Controller") -> None:
        """Called once when this scheme is activated."""
        pass

    @abstractmethod
    def update(self, state: "RobotState") -> "List[ServoCommand]":
        """
        Called every tick. Return servo commands to execute.

        Args:
            state: Current robot state. state.dt is seconds since last call.

        Returns:
            List of ServoCommand objects. Empty list = no movement.
        """
        ...

    def on_stop(self) -> None:
        """Called on shutdown or when scheme is swapped out."""
        pass


# ---------------------------------------------------------------------------
# Visualizer
# ---------------------------------------------------------------------------

class Visualizer(ABC):
    """
    Base class for all visualizers.

    A Visualizer receives RobotState every tick and renders it however
    it chooses (Rerun, terminal, file log, etc.).

    Lifecycle:
        on_start(controller)  — open windows, initialize SDK
        update(state)         — render/log new state
        on_stop()             — clean up resources
    """

    name: str = "unnamed"

    def on_start(self, controller: "Controller") -> None:
        """Called once when visualizer is activated."""
        pass

    @abstractmethod
    def update(self, state: "RobotState") -> None:
        """Called every tick. Render or log the current state."""
        ...

    def on_stop(self) -> None:
        """Called on shutdown. Close windows, flush buffers, etc."""
        pass
