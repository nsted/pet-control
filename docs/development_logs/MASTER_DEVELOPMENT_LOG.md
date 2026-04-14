# petctl Development Log

**Period:** Feb 20 – Mar 1, 2026
**Repository:** `main` branch (single branch throughout — no feature branches)
**Total commits:** 36

---

## Legend

| Symbol | Meaning |
|--------|---------|
| 🟢 | Success — landed and working |
| 🔴 | Failed / reverted / superseded |
| 🟡 | Partial — fixed in subsequent commit |

---

## Phase 1 — Project Foundation (Feb 20)

| Date | Commit | Description | Files Changed | Status |
|------|--------|-------------|---------------|--------|
| Feb 20 | `6722dcc` | Placeholder initial commit — just a README stub | 1 | 🔴 Abandoned immediately |
| Feb 20 | `0fe907e` | **Full initial commit:** Controller, RobotBackend, MockBackend, KeyboardControlScheme, RerunVisualizer, WebSocket servo comms, passthrough scheme, types, protocols, CLI | 16 (+2254 lines) | 🟢 |
| Feb 20 | `7c6cc21` | Fix 3D visualization: 8 modules, correct rotation composition; add `robot_assembly.json` | 2 (+208 lines) | 🟢 |
| Feb 20 | `633dda6` | Update README; fix joint axes in `robot_assembly.json` | 2 | 🟢 |

---

## Phase 2 — Package Rename + First Hardware Session (Feb 21)

| Date | Commit | Description | Files Changed | Status |
|------|--------|-------------|---------------|--------|
| Feb 21 | `7d695c1` | Rename `petcrl → petctl`, `grapple → robot`; bundle 3D mesh assets (head, body, tail OBJ/MTL) into package | 23 (+9783 lines) | 🟢 |
| Feb 21 | `5ab41e7` | **Critical hardware fix:** servos require non-zero `TARGET_TORQUE` for firmware v43 — robot was completely unresponsive before this | 1 | 🟢 |
| Feb 21 | `e671d2e` | Remove software angle limits from keyboard control and head joint config — limits were over-constraining manual control | 2 | 🟢 |

---

## Phase 3 — Sensor Visualization (Feb 22)

| Date | Commit | Description | Files Changed | Status |
|------|--------|-------------|---------------|--------|
| Feb 22 | `1a5384d` | Add touch/pressure sensor overlay in Rerun: 45° middle face quaternion fix, default 8 mock modules | 3 (+88 lines) | 🟢 |
| Feb 22 | `5a4dc27` | Tune module heading offsets in `robot_assembly.json` to match physical robot | 1 | 🟢 |
| Feb 22 | `06d837f` | Fix sensor overlay: render pressure disc on top of touch disc; both individually selectable in Rerun | 1 | 🟢 |

---

## Phase 4 — Limp Mode + Servo-to-Joint Mapping (Feb 23)

| Date | Commit | Description | Files Changed | Status |
|------|--------|-------------|---------------|--------|
| Feb 23 | `c5e9ed0` | Add `--limp` CLI flag: disable motor torque for manual pose exploration; propagated through CLI → controller → backend → protocol | 4 (+47 lines) | 🟢 |
| Feb 23 | `c7e7c4e` | Fix servo-to-joint mapping off-by-one error; retune module heading rotations | 2 | 🟢 |

---

## Phase 5 — Sensor Normalization (Feb 26, morning)

Sensor values were not normalizing correctly on real hardware — required several iterations.

| Date | Commit | Description | Files Changed | Status |
|------|--------|-------------|---------------|--------|
| Feb 26 | `24090bf` | Attempt 1: Fix Rerun sensor charts — log `SeriesLines` alongside `Scalars` | 1 (+21 lines) | 🔴 Charts still broken |
| Feb 26 | `345357e` | Attempt 2: Fix sensor plots — send blueprint with explicit `TimeSeriesView` | 1 (+13 lines) | 🔴 Timeline still wrong |
| Feb 26 | `dca30d0` | **Fix sensor timeline:** use `duration=` instead of `timestamp=` for monotonic time | 1 | 🟢 |
| Feb 26 | `7fc4e9a` | Fix touch normalization: calibration min range 6554 → 200 counts | 1 | 🟢 |
| Feb 26 | `2bb77ae` | Add adaptive sensor normalization: self-scale to observed maximum response per face | 1 (+16 lines) | 🟢 |
| Feb 26 | `d5ff5eb` | Per-type calibration floors: touch=50 counts, pressure=200 counts | 1 | 🟢 |
| Feb 26 | `322e60d` | Lower touch floor 50 → 10 counts after testing on real hardware | 1 | 🟢 |

---

## Phase 6 — EEPROM Home Calibration + Viz Direction Fix (Feb 26, afternoon)

`write_home_offsets` needed to persist servo home positions through power cycles — required two attempts.

| Date | Commit | Description | Files Changed | Status |
|------|--------|-------------|---------------|--------|
| Feb 26 | `2cdad9a` | Fix viz mirror: negate servo angle to match Dynamixel direction convention | 1 | 🟢 |
| Feb 26 | `5a80e7f` | Add limp mode + `'s'` key to save current positions as EEPROM home | 4 (+75 lines) | 🟡 Logic flawed |
| Feb 26 | `f62520e` | Attempt 1: Fix `write_home_offsets` — read existing EEPROM offset before writing | 1 (+23 lines) | 🔴 Still incorrect |
| Feb 27 | `9803dc9` | **Attempt 2: Fix `write_home_offsets`** — zero offset first, read raw encoder, write raw as new offset | 4 | 🟢 |

---

## Phase 7 — Save-Home Key Churn (Feb 27)

Four commits in 6 minutes iterating on the save-home keybinding to avoid terminal and Rerun conflicts.

| Date | Commit | Description | Files Changed | Status |
|------|--------|-------------|---------------|--------|
| Feb 27 | `0353c94` | Change save-home key `'s'` → `Ctrl+S` — prevent accidental triggers | 1 | 🔴 `Ctrl+S` sends XOFF in some terminals |
| Feb 27 | `4a132a0` | Change key `Ctrl+S` → backtick — avoids XOFF and Rerun's `Cmd+S` conflict | 1 | 🔴 Backtick too easy to hit accidentally |
| Feb 27 | `f5b1608` | Change key backtick → `Cmd+\`` | 1 | 🟢 Final binding |
| Feb 27 | `c2674bf` | Update README: document limp mode, `Cmd+\`` save-home, calibration workflow | 1 (+13 lines) | 🟢 |
| Feb 27 | `8d9b9db` | Fix mock backend default position to 0; adjust assembly rotations to match | 3 | 🟢 |

---

## Phase 8 — Code Review + Coordinate System Overhaul (Feb 28)

| Date | Commit | Description | Files Changed | Status |
|------|--------|-------------|---------------|--------|
| Feb 28 | `5d7a40a` | Apply code quality improvements from review: assembly JSON, protocols, keyboard scheme, rerun viz | 4 | 🟢 |
| Feb 28 | `119a512` | **Coordinate system refactor:** switch to `center=0` (signed angles); fix viz rotations across 8 files | 8 | 🔴 Later reverted to 2048 |
| Feb 28 | `3f49252` | **Major commit:** Fix limp mode torque + servo sign-magnitude encoding + sine scheme; add `CLAUDE.md`, `config.py`, design docs (`docs/design/behaviors/`), vendor SDK aliases (`docs/servos/`) | 17 (+2143 lines) | 🟢 |
| Feb 28 | `79fed9a` | Fix `_write_speed`: was calling non-existent `WriteSpeed` SDK method — caused silent failure | 1 | 🟢 |
| Feb 28 | `6a07340` | Disable motor torque on clean shutdown | 1 | 🟢 |

---

## Phase 9 — GroupSync + Servo Feedback + Visualizer Overhaul (Mar 1)

| Date | Commit | Description | Files Changed | Status |
|------|--------|-------------|---------------|--------|
| Mar 1 | `431833f` | **Revert coordinate system:** restore `position_center=2048` as single source of truth across 7 files — the `center=0` refactor from Feb 28 caused downstream issues | 7 | 🟢 (reverts `119a512`) |
| Mar 1 | `81437c7` | Fix `write_home_offsets` (third time): use modulo offset for power-cycle persistence | 1 (+23 lines) | 🟢 |
| Mar 1 | `5bfb349` | Switch to `GroupSyncWrite` for batch servo writes; drop per-tick individual position reads — significant performance improvement | 1 (+103 lines) | 🟢 |
| Mar 1 | `d86dfe6` | Replace fixed-rate 20Hz tick loop with free-running loop + timing stats for diagnostics | 2 | 🟢 |
| Mar 1 | `5caeee8` | Add `GroupSyncRead`: separate commanded vs. actual positions in `RobotState`; servo feedback now tracked independently | 1 (+85 lines) | 🟢 |
| Mar 1 | `0ef253c` | Read servo state at two rates; add current draw, temperature, voltage to `RobotState` | 2 (+84 lines) | 🟢 |
| Mar 1 | `1c3e133` | Run visualizers in a background thread — decouple from control loop to prevent viz latency from affecting servo timing | 1 (+26 lines) | 🟢 |
| Mar 1 | `1658431` | Replace sensor time-series charts with servo health charts (current, temp, voltage per servo) in Rerun | 1 | 🟢 |

---

## Recurring Problem Areas

| Problem Area | Attempts | Resolution |
|---|---|---|
| Sensor chart rendering in Rerun | 3 commits | Fixed with `duration=` monotonic time + blueprint |
| `write_home_offsets` EEPROM logic | 3 commits | Zero → read raw → write raw as offset |
| Save-home keybinding | 4 commits | Settled on `Cmd+\`` |
| Servo coordinate system | Refactored to `center=0` (Feb 28), then reverted to `center=2048` (Mar 1) | `center=2048` is current truth |
| Servo unresponsive on hardware | 1 fix | `TARGET_TORQUE` must be non-zero for firmware v43 |

---

## Cumulative Scope

| Milestone | Date Reached |
|---|---|
| Working 3D visualization (mock) | Feb 20 |
| Servos moving on real hardware | Feb 21 |
| Touch/pressure sensor overlay | Feb 22 |
| Limp mode + manual pose | Feb 23 |
| Sensor normalization working | Feb 26 |
| EEPROM home calibration | Feb 27 |
| `config.py` + design docs in repo | Feb 28 |
| Servo health telemetry (current/temp/voltage) | Mar 1 |
| Commanded vs. actual position tracking | Mar 1 |
| Viz decoupled from control loop | Mar 1 |
