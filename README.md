# petctl — PET Robot Control Framework

Modular Python control framework for the PET robot. Supports swappable control schemes (keyboard, motion patterns, Ollama-driven LLM behavior), real-time visualization via [Rerun.io](https://rerun.io), and a backend abstraction layer for future physics simulation.

## Quick start

```bash
pip install -e .
petctl run                                    # mock backend, engage (curl-towards) pattern, Rerun viz
petctl run --backend robot                    # real robot
petctl run --backend robot --limp             # limp mode — joints move freely for calibration
petctl run --backend mock --mode mock-sensor-sine  # animated mock sensor data, no robot needed
petctl run --backend robot --control ollama   # real robot, LLM-driven behavior via Ollama
petctl info                                   # connect and print robot status
```

## CLI reference

petctl has four commands: `run`, `info`, `calibrate-touch`, and `touch-threshold`. `petctl <command> --help` prints the authoritative, up-to-date flag list — the tables below summarize it.

### `petctl run`

**Backend & connection**

| Flag | Default | Description |
|---|---|---|
| `--backend` | `mock` | `mock` (no robot) or `robot` (real robot) |
| `--host` | `pet-robot.local` | Robot hostname/IP (robot backend) |
| `--port` | `8080` | Robot WebSocket port (robot backend) |
| `--motors` | — | Comma-separated MIT motor IDs (e.g. `1,3`); skips CAN feedback discovery — use for partial hardware |
| `--calibrate` | off | Re-zero software offsets to current pose on connect |
| `--cap-recal` | off | Recalibrate MPR121 cap sensor baselines at launch |
| `--limp` | off | Disable motor torque after connect so joints move freely; visualizer still updates from read positions |
| `--dry-run` | off | Read sensors and run the scheme but never send servo commands |

**Mock backend** (only used when `--backend mock`)

| Flag | Default | Description |
|---|---|---|
| `--mode` | `interactive` | `interactive`, `file`, `mock-sensor-sine`, or `noise` |
| `--state` | — | Path to a `mock_state.json` for `file`/`interactive` modes |
| `--num-modules` | `8` | Number of simulated modules |

**Control scheme selection**

| Flag | Default | Description |
|---|---|---|
| `--control` | `engage` | Motion pattern or scheme — see below |

`--control` accepts: `keyboard`, `passthrough`, `sine`, `command`, `ollama`, plus every motion pattern in `petctl/schemes/patterns.py` — `wiggle`, `nuzzle`, `purr`, `contort`, `writhe`, `engage`, `withdraw`, `seek-touch`, `avoid-touch`, `yield`, `curl`, `snuggle`, `explore`, `twitch`, `struggle`, `idle`, `cascade`, `coil`, `curl-towards`, `freeze`, `pose`, `pulse`, `stroke`, `stroke-curl`, `balanced-torque`. See `docs/PATTERNS.md` for what each pattern actually does, touch-reactivity, and which parameters `--vel` scales.

| Flag | Default | Description |
|---|---|---|
| `--servo-id` | all active | Target a single servo with `sine` control |
| `--step` | `4.0` | Degrees per keypress for `keyboard` control |
| `--vel` | `1.0` | Default motion speed scale (`0.05`–`1.0`), clamped. For `ollama`: used until the LLM responds. For patterns: scales `hz`/speed at launch. |

**Ollama-specific**

| Flag | Default | Description |
|---|---|---|
| `--log-ollama-input` | off | Print the full JSON payload sent to Ollama on each LLM call |
| `--ollama-monitor` | off | Run Ollama and log its responses but do not apply them to robot motion (safe for watching behavior without moving the robot) |

**Startup sequence** — Ollama is a separate local server; petctl talks to it over HTTP and won't start it for you.

```bash
# One-time setup
brew install ollama              # or curl -fsSL https://ollama.com/install.sh | sh (Linux)
ollama pull gemma3:4b            # ~2.5 GB download, once

# Every session
ollama serve                     # leave running in its own terminal (auto-started if you use the macOS desktop app)
curl http://localhost:11434      # optional check — should print "Ollama is running"

# Then, in another terminal
petctl run --control ollama --backend mock         # or --backend robot
```

On startup `petctl run --control ollama` prints a connection line confirming the model and active servo count:

```
[Ollama] connected, model=gemma3:4b, 7 active servos.
```

If Ollama isn't running yet, you'll instead see:

```
[Ollama] server not reachable at http://localhost:11434/api/chat — start Ollama with 'ollama serve' then restart PET.
```

The robot still runs (in `freeze` mode) but won't respond to touch until Ollama is reachable — start `ollama serve` and restart `petctl run`. See `docs/OLLAMA_SETUP.md` for full model setup, prompt/personality config (`petctl/prompts/`), and response format.

**Logging & debugging** — logging goes to stdout at `INFO` level by default (set in `petctl/cli.py:main()`, no file handler). These flags enable extra `INFO`/`DEBUG` output for specific subsystems; none of them write to a log file, so redirect (`> run.log 2>&1`) or pipe (`| tee run.log`) if you want a persisted copy.

| Flag | Subsystem | What it prints |
|---|---|---|
| `--log-mit` | Motor feedback | MIT motor table (pos/vel/torque) every 2s |
| `--log-touch` | Touch/contact | Touch/contact type events to console |
| `--log-loop` | Control loop | Timing stats (Hz, min/mean/max ms) every 5s |
| `--log-robot` | `petctl.backends.robot` | Connection and discovery logs (sets logger to `DEBUG`) |
| `--log-viz` | `petctl.visualizers.rerun_viz` | Assembly load, viewer launch, IMU diagnostics (sets logger to `DEBUG`) |
| `--log-power` | `petctl.power_manager` | Bin seeding, promotion, thermal, voltage events (sets logger to `DEBUG`) |
| `--log-ollama-input` | Ollama scheme | Full JSON payload sent to the LLM on each call (see above) |

All `[Ollama]`-tagged lines (connection status, per-touch response, round-trip timing, parse warnings) are logged at `INFO` and print without any flag — filter them out of the stream with:

```bash
petctl run --control ollama --backend mock 2>&1 | grep -E "\[Ollama\]"
```

There is no global `--verbose`/`--log-level` flag — granularity is controlled per-subsystem via the `--log-*` flags above. To go beyond `INFO` for a subsystem that doesn't have a dedicated flag, set its logger to `DEBUG` directly, e.g. `petctl/schemes/ollama_scheme.py`'s `logger.debug(...)` calls (raw prompt text) require:

```python
logging.getLogger("petctl.schemes.ollama_scheme").setLevel(logging.DEBUG)
```

**Visualization & dev tools**

| Flag | Default | Description |
|---|---|---|
| `--no-viz` | off | Disable the Rerun visualizer (headless mode) |
| `--dev-ui` | off | Serve the pattern dev UI in browser (also disables the Ollama LLM — motion is driven by the UI instead) |
| `--ui-port` | `8765` | Port for the dev UI HTTP server |

### `petctl info`

Connects to the real robot and prints discovered modules and servo IDs. Takes `--host`, `--port`, `--motors` (same meaning as `run`).

### `petctl calibrate-touch`

Forces all MPR121 capacitive-touch ICs (head + body modules) to reinitialize and recapture a fresh baseline. Use when pads are stuck "on" after prolonged touch. Takes `--host`, `--port`.

### `petctl touch-threshold TOUCH12 RELEASE12 [--touch3] [--release3]`

Sets MPR121 touch/release thresholds on all modules (side-face electrodes via positional args, top/middle via `--touch3`/`--release3`, defaulting to the side-face values). Changes are volatile — reset on next reboot. Firmware defaults: `touch12=5, release12=2, touch3=6/3, release3=3/1` (head/body). Constraint: `touch > release > 0` (enforced client-side before sending). Takes `--host`, `--port`.

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
