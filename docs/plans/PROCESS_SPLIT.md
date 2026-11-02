# Process split — petctl driver + brain

Status: in progress. Branch `feat/process-split` (from `feat/shape-layer`; both untouched branches are the rollback).

## Goal

Split petctl into two programs on a local message bus:

- **brain** = `Controller` + motions + perception + visualizers (minus hardware and safety)
- **driver** = a backend (`RobotBackend` | `MockBackend`) wrapped by `SafetyBackend`, published on the bus

Later: a MuJoCo driver and an RL brain, neither able to bypass safety. **This plan does the split only** — no MuJoCo, no RL, no behavior change.

```
brain (Controller, motions, perception, Rerun)
   │  pet/cmd  (ServoCommand list, ~50 Hz)        ▲ pet/state (hardware facts)
   │  pet/svc/* (request/reply)                   │
   ▼                                              │
driver:  SafetyBackend( RobotBackend | MockBackend )   → later: MuJoCoBackend
```

## Rules

- Work only on `feat/process-split`. Never commit to `main` or `feat/shape-layer`.
- `.venv` is shared; `eclipse-zenoh` goes in optional extra `[split]`.
- **Behavior-neutral.** Slew filter stays in `Controller`; loop rate unchanged (SHAPE_LAYER 1.4, 1.4a, 1.5 stay open).
- In-process `petctl run --backend robot|mock` must work after every change (this is the rollback).
- Ask before installing anything (incl. zenoh), moving the robot, or any GitHub issue action.
- Hardware limits come from `petctl/config.py`.
- **Working method:** small increments; after each change run pytest + mypy, then run `petctl run --backend mock --control sine` **with Rerun on**, watch pose and charts, read logs. Report observations per stage.

## Findings from the Stage 0 code audit

- `enable_motor` is missing from the `Backend` ABC (only `RobotBackend`); Controller calls it unguarded in the power-reset path; `dev_ui.py` guards with `hasattr` and reads private `_discovered_motors`.
- `Controller` safety steps to move: `pm.update`, disable-event drain, voltage cutoff, `get_telemetry`, power reset, thermal gate / compliance scale / `allocate_budget` / idle cmds for pending motors.
- `controller.py` checks `type(backend).__name__ == "RobotBackend"` (dev power-budget warning) — breaks once wrapped.
- `dt` is set by the backend per `get_state()` call; slew filter uses it. `RemoteBackend` must compute `dt` per `get_state()` call.
- Recorder logs post-PM `commands_sent`; after Stage 2 Controller only sees pre-PM commands (differs only when PM modifies commands).
- Mock has battery V = 0 → voltage logic inactive; thermal data absent.
- Serialization hazards: int dict keys, `set`, tuples, enums, nested dataclasses.
- Scripts and `cli.py` `info/calibrate-touch/touch-threshold` use `RobotBackend` directly and stay that way.

## Stage 0 — Branch and baseline

- [x] Branch created by Nick.
- [x] SHAPE_LAYER notes committed; this plan added to README.
- [x] Add `--duration` to `petctl run`; baseline `pytest`, `mypy`, and a fixed-length mock run with `--control sine --record`.

Baseline results (2026-11-02, commit 0ce5768 + `--duration`):

- `pytest`: 83 passed. `mypy petctl`: 50 errors in 9 files (pre-existing; list saved for diffing — do not add new ones).
- `petctl run --backend mock --control sine --duration 10 --record …` (Rerun on): 1695 ticks recorded.
- **Finding:** MockBackend left `battery_*_raw` = 0 → PowerManager saw a constant 12.5 A and global-emergency-stopped after 0.5 s (joints never moved; max |pos| 0.003 rad). **Fixed (Nick approved):** mock now reports ≈12 V / ≈0 A idle telemetry (`backends/mock.py`).
- **Baseline after fix:** pytest 83 passed; mypy unchanged (50 errors); 10 s `--control sine` run, 1650 ticks all with commands, max |pos| 1.30 rad, PM state RUNNING, 11.99 V, 0 A, source=battery, no PM warnings. Recording kept in the session scratchpad (not committed). Rerun viewer launched and received data; visual confirmation of pose/charts is by Nick's eyes — Claude can only check logs and recordings.

## Stage 1 — Complete the `Backend` interface

- [ ] `enable_motor`, `power_reset`, and a hardware-capability property on the ABC with no-op defaults; MockBackend `enable_motor`; `dev_ui` stops using `hasattr`/private attrs.

Done when: pytest/mypy match baseline; power reset on mock logs no error.

## Stage 2 — Move safety out of the Controller

- [ ] `SafetyBackend(Backend)` wrapping another backend; owns `PowerManager`.
- [ ] Remove those steps from `Controller._loop`; route `request_power_reset` through the backend.
- [ ] `--backend robot|mock` construct `SafetyBackend(...)` in-process.

Done when: `tests/test_power_manager.py` passes unchanged; mock recording matches baseline within tolerance; thermal/voltage events disable motors in a mock test.

## Stage 3 — Wire format (`petctl/wire.py`)

- [ ] State = hardware facts only (no gesture / is_motion_active / servo_commanded_positions / dt). JSON codec, the only module touching serialization.
- [ ] Commands = `ServoCommand` fields. Services: `enable_motor`, `disable_motor`, `disable_torques`, `write_home_offsets`, `set_stagger`, `calibrate_touch`, `set_touch_threshold`, `power_reset`.

Done when: round-trip tests pass.

## Stage 4 — Driver and `RemoteBackend`

- [ ] Zenoh transport (ask before install), keys `pet/state`, `pet/cmd`, `pet/svc/<name>`; extra `[split]`.
- [ ] `petctl driver --backend robot|mock`; `RemoteBackend`; `petctl run --backend remote`.
- [ ] Note Controller 200 Hz loop vs cached 50 Hz state; log for SHAPE_LAYER 1.5.

Done when: mock driver + `--backend remote` brain runs the same scripted motion as baseline.

## Stage 5 — Liveness

- [ ] Driver command watchdog: smooth ramp to safe hold (timeout in `config.py`).
- [ ] Brain: driver silent → `connected=False`, last state, no crash.

## Stage 6 — Equivalence and latency

- [ ] In-process vs split recording comparison; loopback latency/jitter numbers recorded here.

## Stage 7 — Bench (needs Nick's explicit go-ahead; the robot moves)

- [ ] Real driver + brain over loopback: connect, stroke-curl, `--control ollama`, thermal/power reset, kill tests.
- [ ] Then resume SHAPE_LAYER 1.4a → 1.4 → 1.5 with the filter decision made inside the driver; then `MuJoCoBackend`.

## Out of scope

MuJoCo, RL, ROS 2, changing slew filters or loop rate, splitting the brain further, retiring `patterns.py`.
