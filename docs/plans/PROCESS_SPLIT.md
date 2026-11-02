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

- [x] `enable_motor`, `power_reset`, and a hardware-capability property on the ABC with no-op defaults; MockBackend `enable_motor`; `dev_ui` stops using `hasattr`/private attrs.

Done when: pytest/mypy match baseline; power reset on mock logs no error.

## Stage 2 — Move safety out of the Controller

- [x] `SafetyBackend(Backend)` wrapping another backend; owns `PowerManager` (`petctl/backends/safety.py`, `tests/test_safety_backend.py`).
- [x] Remove those steps from `Controller._loop`; `request_power_reset` now awaits `backend.power_reset()`.
- [x] `--backend robot|mock` construct `SafetyBackend(...)` in-process (`cli.py`). Note: a `Controller` built directly with a bare backend (scripts/tests) now has no power protection.

Done when: `tests/test_power_manager.py` passes unchanged; mock recording matches baseline within tolerance; thermal/voltage events disable motors in a mock test.

Stage 2 results (2026-11-02): pytest 91 passed (83 + 8 new), mypy 49 errors (one fewer than baseline, none new). 30 s mock sine run with Rerun: max |pos| 1.29 rad (baseline 1.30), ~166 ticks/s (same), PM RUNNING, 11.99 V, telemetry scales 1.0 in both. **Recorder difference:** `--record` now stores the Controller's pre-safety commands (constant kp 0.8) instead of the post-PM commands (baseline kp 0–0.8 from bin-pack scaling). A spy on the inner backend confirms the safety layer still emits the same kp spread (≈630 distinct values, 0–0.8). Power reset on mock logs success and no `enable_motor` error.

## Stage 3 — Wire format (`petctl/wire.py`)

- [x] State = hardware facts only (no gesture / is_motion_active / servo_commanded_positions / dt). JSON codec, the only module touching serialization.
- [x] Commands = `ServoCommand` fields. Services: `enable_motor`, `disable_motor`, `disable_torques`, `write_home_offsets`, `set_stagger`, `calibrate_touch`, `set_touch_threshold`, `power_reset`.

Done when: round-trip tests pass.

Stage 3 results (2026-11-02): `petctl/wire.py` + `tests/test_wire.py` (26 tests); pytest 117 passed; mypy 48 errors (two baseline errors gone, none new). JSON, stdlib only (no new dependency). A real 8-module mock state is ≈5.0 KB, encode+decode ≈0.15 ms per message (≈250 KB/s at 50 Hz). State is carried as ROS-JointState-style parallel arrays with `null` for missing values; `dt`, `gesture`, `is_motion_active`, `servo_commanded_positions` are not on the wire. Every message has `"v": 1` and a wrong version is rejected. Service requests are validated on both encode and decode. `info`, `set_home`, `set_hardware_zero`, `poll_positions`, `reset_motor_zero` stay local-only. Commands carry an optional `sent_at` for Stage 6 latency logging.

## Stage 4 — Driver and `RemoteBackend`

- [x] Zenoh transport, keys `pet/state`, `pet/cmd`, `pet/svc/<name>`; extra `[split]` (`petctl/transport.py`: `Bus` ABC, `ZenohBus`, in-memory `LoopbackBus`).
- [x] `petctl driver --backend robot|mock`; `RemoteBackend`; `petctl run --backend remote [--bus ENDPOINT]`.
- [x] Note Controller 200 Hz loop vs cached state; logged below for SHAPE_LAYER 1.5.

Done when: mock driver + `--backend remote` brain runs the same scripted motion as baseline.

Stage 4 results (2026-11-02): eclipse-zenoh 1.10.2 installed (Nick approved) into the shared `.venv`; imported lazily, so branches without the `[split]` extra are unaffected. Zenoh runs peer-to-peer on `tcp/127.0.0.1:7447` with multicast and gossip scouting disabled. pytest 132 passed, mypy no new errors. Two-process run (mock driver + `run --backend remote --control sine --duration 40`, Rerun on): 165.4 ticks/s vs 166.5 in-process, max |pos| 1.292 rad (identical), PM RUNNING at 11.99 V, driver dropped 0 command batches. The unmodified `Controller` also runs over `RemoteBackend → Driver → SafetyBackend → Mock` in `tests/test_remote_backend.py`.

Design decisions and caveats:
- **Driver state rate = 4 × `motor_update_hz`** (same cadence the PowerManager saw in-process; its EMA windows are tuned to it), ≈162 Hz measured. The brain also commands at ≈165 Hz, so the plan's "~50 Hz `pet/cmd`" is really ≈165 Hz to stay behavior-neutral.
- **SHAPE_LAYER 1.5 input:** Controller loop and driver state publisher are two unsynchronized ≈165 Hz loops, so a given state can be seen twice or skipped. `RemoteBackend` computes `dt` per `get_state()` call (as in-process) rather than per message.
- Commands are applied in order; the driver's queue (256) drops the oldest on overload.
- Brain-side `is_real_hardware` is False; the driver logs the dev-power-budget warning itself.
- `set_stagger` is fire-and-forget from the brain (the Backend method is synchronous).
- Service timeouts: 5 s for fast calls, 20 s (client) / 15 s (driver) for calibration and EEPROM writes.

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
