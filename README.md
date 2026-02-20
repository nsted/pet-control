# petcrl — PET Robot Control Framework

Modular Python control framework for the PET robot. Supports swappable control schemes (keyboard, ML/AI), real-time visualization via [Rerun.io](https://rerun.io), and a backend abstraction layer for future physics simulation.

## Quick start

```bash
pip install -e .
petcrl run                          # mock robot, keyboard control, Rerun viz
petcrl run --backend grapple        # real robot
petcrl run --backend mock --mode sine  # animated demo, no robot needed
petcrl info                         # connect and print robot status
```

## Architecture

```
Controller
├── RobotBackend       GrappleBackend (real robot) | MockBackend (offline)
├── ControlScheme      KeyboardControlScheme | PassthroughControlScheme | custom ML
└── Visualizer         RerunVisualizer (3D pose + sensor charts)
```

All three components are swappable ABCs. Control schemes never touch the backend directly — they only see `RobotState` and emit `ServoCommand` objects.

## Keyboard controls

| Key | Action |
|-----|--------|
| `0`–`8` | Select module |
| `↑` / `↓` | Rotate selected joint ±5° |
| `r` | Reset all to 0° |
| `q` / `Esc` | Stop |

> macOS: grant Accessibility permission to your terminal for keyboard capture to work.

## Custom ML control scheme

```python
from petcrl import Controller, ControlScheme, RobotState, ServoCommand
from petcrl.backends.grapple import GrappleBackend
import asyncio

class MyScheme(ControlScheme):
    name = "my_scheme"

    def update(self, state: RobotState) -> list[ServoCommand]:
        # state.sensors[mod_id].touch_middle, .pressure_left, etc.
        return [ServoCommand.from_angle(servo_id=1, angle_deg=30.0)]

asyncio.run(Controller(backend=GrappleBackend(), scheme=MyScheme()).run())
```

See `examples/ml_control_example.py` for a full PyTorch integration example.

## Robot assembly

`petcrl/assets/robot_assembly.json` defines the 8-module kinematic chain — joint axes, link offsets, mesh orientations, and OBJ model filenames. The visualizer reads this file to build the 3D hierarchy in Rerun.

OBJ mesh files are loaded from `../python/grapple_ai/3d_models/prisms/` (not duplicated into this repo).

## Installation

```bash
pip install -e .
# Optional: PyTorch for ML schemes
pip install -e ".[ml]"
```

Requires Python ≥ 3.10. The `ftservo-python-websockets` SDK is included as a dependency for servo communication over WebSocket.
