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

- [x] Driver command watchdog: smooth ramp to safe hold (`LIVENESS_LIMITS` in `config.py`).
- [x] Brain: driver silent → `connected=False`, last state, no crash.

Stage 5 results (2026-11-02): pytest 142 passed, mypy no new errors.
- `LIVENESS_LIMITS` (config.py): `driver_cmd_timeout_s` 0.5, `safe_hold_ramp_s` 1.0, `brain_state_timeout_s` 1.0.
- **Driver watchdog:** after 0.5 s without a command batch, kp/kd/torque_ff of each servo's last command ramp linearly to zero over 1.0 s while the last position target is held; any new batch resumes immediately. It never triggers before a brain has commanded. *The end state is zero gains (limp), not a stiff hold — confirm that is the right "safe hold" for the real robot at Stage 7 (gravity/curl loads).*
- **Kill test 1 (brain SIGKILL, mock driver, separate processes):** effort fell smoothly to 0.000 within ≈1.5 s of the kill (0.5 s timeout + 1.0 s ramp); joint stayed put (0.727 rad); driver logged "No commands for 0.51s — ramping" and "Safe hold reached". A brief effort blip (0.087 vs ≈0.045 normal) right after the kill is the frozen target against a still-moving joint.
- **Kill test 2 (driver SIGKILL, brain with Rerun):** brain logged "No state from the driver for 1.0s — reporting disconnected", kept running, and after the driver was restarted Zenoh reconnected on its own within ≈2 s ("Driver is back"). Unit test: a full Controller survives its driver dying and shuts down cleanly.
- **Bug found and fixed:** the brain flagged a false "disconnected" for one tick at startup because staleness used the time the event loop *processed* a state, and starting Rerun blocks the loop for >1 s. Staleness now uses the arrival time stamped on the bus thread (regression test fails without the fix).
- While the driver is silent the brain publishes no commands and service calls return immediately (void calls no-op, bool calls return False) so shutdown never hangs.

## Stage 6 — Equivalence and latency

- [x] In-process vs split recording comparison (`scripts/compare_recordings.py`); loopback latency/jitter numbers recorded below (`petctl/latency.py`, logged at driver/brain shutdown).

Stage 6 results (2026-11-02, mock backend, `--control sine`, 60 s runs, same Mac, Zenoh over loopback TCP):

**Equivalence** — `python scripts/compare_recordings.py in_process.jsonl split.jsonl` (aligns start phase within ±3 s, then reports residual):

| metric | result |
|---|---|
| aligned RMS, measured joint positions, all 7 servos | ≤ 0.0010 rad (0.06°) |
| aligned RMS, commanded positions, all 7 servos | ≤ 0.0045 rad (0.26°) |
| tracking lag cmd→pos (servo 1; 5 ms resolution) | 85 ms in-process, 90 ms split → split adds ≲ 5 ms |
| loop rate | 166.0 vs 168.6 ticks/s |
| peak \|pos\| after the first 2 s | 1.079 vs 1.077 rad |

An earlier cross-run comparison showed a 15 ms lag difference; paired runs under matching conditions show ≲ 5 ms, so that was noise. Raw start phases differ by 1–1.4 s between runs because the sine's clock starts in `on_start`, before the Rerun viewer finishes launching — hence the alignment step.

Note: the in-process mock showed a 1.426 rad peak at t = 0.6 s. Cause: the first `get_state()` came 1.36 s after `connect()` (viewer launch blocks the loop), so the mock integrated one huge step. The split run does not have this (the driver's loop is independent of the viewer; the brain's first `dt` is 0).

**Latency & jitter** — one-way, same monotonic clock on both processes (commands: `sent_at` → just before the driver applies, including its queue; state: driver `get_state()` timestamp → decoded on the brain's bus thread). ≈10 000 messages each:

| run | message | mean | std | p50 | p95 | p99 | max |
|---|---|---|---|---|---|---|---|
| Rerun on | command | 0.45 ms | 0.32 | 0.41 | 0.80 | 1.22 | 15.3 |
| Rerun on | state | 1.04 ms | 8.67 | 0.49 | 0.87 | 1.72 | 231 |
| headless | command | 0.77 ms | 0.52 | 0.65 | 1.53 | 2.38 | 14.8 |
| headless | state | 1.08 ms | 0.53 | 1.05 | 1.55 | 2.43 | 23.2 |

Inter-arrival (jitter) at ≈165 Hz: commands 5.9–6.2 ms mean, std 0.45–0.69 ms, p99 6.7–8.2 ms; state 5.9–6.2 ms mean, std 0.8–2.4 ms. The single 231 ms state-latency outlier appears only with Rerun on (23 ms headless), consistent with the viewer launch stalling the brain process once at startup (not independently verified).

Takeaway for SHAPE_LAYER 1.5: the bus itself costs ≈ 0.5–1 ms per hop (≈ 0.2 control ticks); the dominant effect of the split is that the brain's ≈165 Hz loop and the driver's ≈165 Hz state publisher are unsynchronised, so a state is occasionally seen twice or skipped (inter-arrival p99 ≈ 8 ms vs 6 ms mean).

## Stage 7 — Bench (needs Nick's explicit go-ahead; the robot moves)

- [x] Real driver + brain over loopback (2026-11-03): connect, limp, stroke-curl, sine on servos 4 and 7, kill tests, ollama connect, power_reset. Gaps noted below (loaded-body limp, LLM-driven motion, real thermal fault).

Stage 7 results so far (real robot, wall supply 14.4 V, driver + brain on one Mac over Zenoh loopback):
- First attempt hit the PowerManager low-voltage cutoff (bus steady at 7.2 V < 10 V, motors correctly disabled); after the supply was fixed the bus read 14.4 V and the cutoff cleared. Not a split artifact.
- Limp brain (15 s), stroke-curl (20 s, needs touch, no visible motion): no warnings or errors.
- `--control sine --servo-id 7` ±60° at 0.2 Hz, 15 s: commanded ±1.039 rad, measured −1.028..+1.031 rad; visibly moved (confirmed by Nick). 2674 command batches applied, 0 dropped.
- `--control sine --servo-id 4`: positive side tracked (+1.044 rad), negative side reached only −0.533 rad vs −1.039 commanded. Servo-specific (cause unknown: mechanical stop / neighbour segment?). TODO compare against in-process `run --backend robot`.
- Latency (one-way, real robot): command mean 0.20–0.22 ms, p99 ≈ 0.85 ms, max 1.8–4.2 ms; state mean 0.45 ms, p99 1.27 ms when the viewer does not stall the brain (up to 519 ms outliers at startup with Rerun on). Tick spacing 5.6–5.7 ms unchanged. Command→joint-motion delay not yet measured on hardware.
- Watchdog on exit: gains ramped to zero over 1 s, "Safe hold reached", no abrupt cutoff.
- **Kill test 1 (brain SIGKILL mid-sine, servo 7, real robot):** driver logged "No commands for 0.50s — ramping", then "Safe hold reached"; the joint decelerated and settled at −0.794 rad (no collapse), torque ≈ 0. Tail joint only: the limp end state has NOT been exercised on a loaded/curled body.
- **Kill test 2 (driver SIGKILL, brain holding still):** brain reported disconnected after 1.0 s and kept running; a new driver reconnected to the robot (firmware boot POWER_ON, ~6 s) and the brain logged "Driver is back". Done with a stationary hold on purpose: the repo shows no firmware command timeout, so a SIGKILLed driver may leave the last command latched. Not verified either way.
- **`--control ollama`:** gemma3:4b is not pulled (download needs permission); ran `qwen2.5:7b-instruct` via a scratchpad wrapper (model tag is hardcoded). Connected over the split, 60 s, no errors, state latency mean 0.49 ms. No touch-triggered LLM response was observed, so LLM-driven motion over the split is NOT verified.
- **Power/thermal:** telemetry crosses the split (motor temps 36–39 °C, per-motor states, compliance scales, 14.4 V wall source). `power_reset` service returned True on a healthy robot and servo 7 still tracked ±1.029 rad afterwards. A real thermal disable / emergency stop was not induced (unsafe on hardware); that path is covered by the mock unit tests only.
- Dropped by Nick: in-process comparison of servo 4's asymmetry.
- [ ] Then resume SHAPE_LAYER 1.4a → 1.4 → 1.5 with the filter decision made inside the driver; then `MuJoCoBackend`.

## Out of scope

MuJoCo, RL, ROS 2, changing slew filters or loop rate, splitting the brain further, retiring `patterns.py`.
