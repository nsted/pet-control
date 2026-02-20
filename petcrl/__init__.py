"""
petcrl — PET Robot Control Framework

Quick start (mock robot, keyboard control, Rerun visualization):

    from petcrl import Controller
    from petcrl.backends.mock import MockBackend
    from petcrl.schemes.keyboard import KeyboardControlScheme
    from petcrl.visualizers.rerun_viz import RerunVisualizer
    import asyncio

    asyncio.run(
        Controller(
            backend=MockBackend(),
            scheme=KeyboardControlScheme(),
            visualizers=[RerunVisualizer()],
        ).run()
    )

Custom ML control scheme:

    from petcrl import Controller, ControlScheme, RobotState, ServoCommand
    from petcrl.backends.grapple import GrappleBackend
    import asyncio

    class MyScheme(ControlScheme):
        name = "my_ml_scheme"

        def update(self, state: RobotState) -> list[ServoCommand]:
            # state.sensors[module_id].touch_middle etc. are 0-1 normalized
            # Return a list of ServoCommand objects
            ...

    asyncio.run(
        Controller(
            backend=GrappleBackend(),
            scheme=MyScheme(),
        ).run()
    )

CLI:
    petcrl run                      # mock backend, keyboard, Rerun viz
    petcrl run --backend grapple    # real robot
    petcrl info                     # print robot status
"""

from petcrl.controller import Controller
from petcrl.protocols import ControlScheme, RobotBackend, Visualizer
from petcrl.types import ModuleSensors, RobotState, ServoCommand

__version__ = "0.1.0"

__all__ = [
    "Controller",
    "ControlScheme",
    "RobotBackend",
    "Visualizer",
    "RobotState",
    "ModuleSensors",
    "ServoCommand",
]
