"""
petctl — PET Robot Control Framework

Quick start (mock robot, keyboard control, Rerun visualization):

    from petctl import Controller
    from petctl.backends.mock import MockBackend
    from petctl.schemes.keyboard import KeyboardControlScheme
    from petctl.visualizers.rerun_viz import RerunVisualizer
    import asyncio

    asyncio.run(
        Controller(
            backend=MockBackend(),
            scheme=KeyboardControlScheme(),
            visualizers=[RerunVisualizer()],
        ).run()
    )

Custom ML control scheme:

    from petctl import Controller, ControlScheme, RobotState, ServoCommand
    from petctl.backends.robot import RobotBackend
    import asyncio

    class MyScheme(ControlScheme):
        name = "my_ml_scheme"

        def update(self, state: RobotState) -> list[ServoCommand]:
            # state.sensors[module_id].touch_middle etc. are 0-1 normalized
            # Return a list of ServoCommand objects
            ...

    asyncio.run(
        Controller(
            backend=RobotBackend(),
            scheme=MyScheme(),
        ).run()
    )

CLI:
    petctl run                      # mock backend, keyboard, Rerun viz
    petctl run --backend robot      # real robot
    petctl info                     # print robot status
"""

from petctl.controller import Controller
from petctl.protocols import ControlScheme, RobotBackend, Visualizer
from petctl.types import ModuleSensors, RobotState, ServoCommand

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
