# CLAUDE.md — petctl project instructions

## Project Overview

petctl is the control framework for PET, a robotic art installation exploring embodiment, affect, and AI through human-robot touch interaction. PET is an 8-module snake-like robot (head + 6 middle + tail) with triangular prism body segments. Each module has 3 sensor faces (left, right, middle) with touch and pressure sensors, and a single-axis revolute joint (Z-axis).

## Architecture

```
Controller
├── RobotBackend       RobotBackend (real robot) | MockBackend (offline)
├── ControlScheme      KeyboardControlScheme | BehaviorEngine | custom
└── Visualizer         RerunVisualizer (3D pose + sensor charts)
```

Components are swappable ABCs. Control schemes never touch the backend directly — they only see `RobotState` and emit `ServoCommand` objects.

## Key Technical Facts

- **Module 0 is the head — no servo.** Servo IDs are 1–7 (modules 1–7 each have one joint).
- **Odd modules have left/right sensors swapped** for body-frame consistency (handled in RobotBackend.\_read_sensors, transparent to everything above)
- **All joints are single-axis Z revolute** — positive angle = one direction, negative = the other
- **Position range:** multi-turn, center = 0, negative values allowed. No hard position limits — angle limits are enforced in software by petctl/config.py
- **Servo protocol:** Feetech SCS series over WebSocket (text frames for sensors/discovery, binary frames for servo commands via hls_scs)
- **Robot hostname:** pet-robot.local:8080
- **Control loop:** 20Hz async, all synchronous SDK calls run in a ThreadPoolExecutor

## Code Conventions

- Python 3.10+, type hints everywhere
- Dataclasses for data types, ABCs for plugin interfaces
- `from __future__ import annotations` in all files
- Docstrings on all public classes and methods
- Async for backend communication, sync for control schemes and behaviors
- Imports: standard library → third-party → petctl (separated by blank lines)
- No global mutable state — pass dependencies through constructors

## Running

```bash
pip install -e .
petctl run                              # mock backend, keyboard control, Rerun viz
petctl run --backend robot              # real robot
petctl run --backend robot --limp       # limp mode for calibration
petctl run --backend mock --mode sine   # animated demo
petctl info                             # print robot status
```

## Safety

Import all hardware limits from `petctl/config.py` — never hardcode servo limits, torque values, or speed values in other modules. See config.py for the full set of constraints.

## Design Reference

See `docs/design/` for architecture sketches and the staged implementation plan.

**These files are DESIGN REFERENCE** — read them for intent, patterns, and interfaces, but implement fresh following the existing code conventions in petctl/. Do not copy-paste from sketches.

- `docs/design/IMPLEMENTATION_PLAN.md` — staged build plan with prompts
- `docs/design/config.py` — safety limits (implement first as petctl/config.py)
- `docs/design/*_sketch.py` — behavior engine, nestle, recoil, undulate, posture, touch events, demo

### Behavior System (being implemented)

- Behaviors produce angle CONTRIBUTIONS (degrees), not absolute positions
- BehaviorEngine sums weighted contributions from all active behaviors
- Valence is continuous (-1 to +1), not discrete behavior switching
- Reflexes are short-duration overrides that fade back to valence-driven behavior
- Gain/spread/falloff values in sketches are initial guesses — will be tuned on real hardware

## Testing

```bash
pip install -e ".[dev]"
pytest
```

Tests are not yet written — add them as you implement new modules.

## Dependencies

- rerun-sdk: 3D visualization
- pynput: keyboard capture
- typer: CLI
- numpy: rotation math
- ftservo-python-websockets: servo communication over WebSocket
- Optional: torch (for future ML control schemes)

## Vendor SDK Reference

docs/vendor/ftservo-python-websockets/ — Feetech servo SDK source.
Read-only reference for register addresses and protocol details.
Do not modify.
