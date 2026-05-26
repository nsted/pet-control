"""
petctl — PET Robot Control Framework

Quick start (mock robot, keyboard control, Rerun visualization):

    from petctl import Controller
    from petctl.backends.mock import MockBackend
    from petctl.schemes.keyboard import KeyboardMotion
    from petctl.visualizers.rerun_viz import RerunVisualizer
    import asyncio

    asyncio.run(
        Controller(
            backend=MockBackend(),
            motion=KeyboardMotion(),
            visualizers=[RerunVisualizer()],
        ).run()
    )

Custom ML motion source:

    from petctl import Controller, Motion, RobotState, ServoCommand
    from petctl.backends.robot import RobotBackend
    import asyncio

    class MyAI(Motion):
        name = "my_ai"

        def update(self, state: RobotState) -> list[ServoCommand]:
            # state.sensors[module_id].touch_left_pads etc. — per-pad tuples (4/4/6)
            # state.sensors[module_id].touch_left etc.  — mean across pads (0-1)
            # Return a list of ServoCommand objects
            ...

    asyncio.run(
        Controller(
            backend=RobotBackend(),
            motion=MyAI(),
        ).run()
    )

CLI:
    petctl run                      # mock backend, keyboard, Rerun viz
    petctl run --backend robot      # real robot
    petctl info                     # print robot status
"""

from petctl.controller import Controller
from petctl.protocols import Backend, Motion, Visualizer
from petctl.types import ModuleSensors, RobotState, ServoCommand

__version__ = "0.1.0"

__all__ = [
    "Controller",
    "Motion",
    "Backend",
    "Visualizer",
    "RobotState",
    "ModuleSensors",
    "ServoCommand",
]
