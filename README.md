# petctl — PET Robot Control Framework

Modular Python control framework for the PET robot. Supports swappable control schemes (keyboard, ML/AI), real-time visualization via [Rerun.io](https://rerun.io), and a backend abstraction layer for future physics simulation.

## Quick start

```bash
pip install -e .
petctl run                          # mock robot, keyboard control, Rerun viz
petctl run --backend robot          # real robot
petctl run --backend robot --limp   # limp mode — joints move freely for calibration
petctl run --backend mock --mode sine  # animated demo, no robot needed
petctl info                         # connect and print robot status
```

## Architecture

```
Controller
├── RobotBackend       RobotBackend (real robot) | MockBackend (offline)
├── ControlScheme      KeyboardControlScheme | PassthroughControlScheme | custom ML
└── Visualizer         RerunVisualizer (3D pose + sensor charts)
```

All three components are swappable ABCs. Control schemes never touch the backend directly — they only see `RobotState` and emit `ServoCommand` objects.

## Keyboard controls

| Key | Action |
|-----|--------|
| `0`–`8` | Select module |
| `↑` / `↓` | Rotate selected joint ±4° |
| `r` | Reset all to 0° |
| `Cmd+`` ` `` ` | Save current positions as EEPROM home |
| `q` / `Esc` | Stop |

> macOS: grant Accessibility permission to your terminal for keyboard capture to work.

## Home calibration

The robot's home pose is defined as all servos at position 0. To calibrate:

1. Run in limp mode: `petctl run --backend robot --limp`
2. Physically position the robot at its desired home pose
3. Press **Cmd+`` ` ``** to write EEPROM offsets — each servo's current physical position is recorded as its new zero
4. Power-cycle the robot; all servos will now report 0 at the home pose

The `robot_assembly.json` kinematic chain is built around position 0 = home, so after calibration the Rerun visualizer will match the physical robot at rest.

## Custom ML control scheme

```python
from petctl import Controller, ControlScheme, RobotState, ServoCommand
from petctl.backends.robot import RobotBackend
import asyncio

class MyScheme(ControlScheme):
    name = "my_scheme"

    def update(self, state: RobotState) -> list[ServoCommand]:
        # state.sensors[mod_id].touch_middle, .pressure_left, etc.
        return [ServoCommand.from_angle(servo_id=1, angle_deg=30.0)]

asyncio.run(Controller(backend=RobotBackend(), scheme=MyScheme()).run())
```

See `examples/ml_control_example.py` for a full PyTorch integration example.

## Robot assembly

`petctl/assets/robot_assembly.json` defines the 8-module kinematic chain — joint axes, link offsets, mesh orientations, and OBJ model filenames. The visualizer reads this file to build the 3D hierarchy in Rerun.

OBJ mesh files are stored in `petctl/assets/3d_models/`.

## Installation

```bash
pip install -e .
# Optional: PyTorch for ML schemes
pip install -e ".[ml]"
```

Requires Python ≥ 3.10. Real-robot I/O uses `websockets` with text SLCAN/API frames to the grapple controller (CubeMars MIT); see `petctl/backends/robot.py`.
