# Shape Layer — Implementation Plan

Status: **Stage 0 complete (2026-09-22), audit-only — nothing modified. Stage 1.2 and 1.3
complete (2026-09-22)**; 1.1, 1.4, 1.5 still open. Supersedes the sequencing in
`BEHAVIOR_SYSTEM.md` (0/17 complete since March); that document's Stage 1 is absorbed
into Stage 2 here.

## Why

Every motion source in `petctl/schemes/patterns.py` (27 classes, 2050 lines) reaches
from raw sensors to absolute per-servo positions in one hop. `Motion.update()` returns
`list[ServoCommand]` — final answers. Final answers don't compose, which is why
`set_motion()` must be a hard swap and why PET cannot breathe while curling.

The design sketches in `docs/design/behaviors/` already specify the fix: behaviours
return per-joint *contributions*, an engine sums them weighted, and conversion to MIT
happens once at the bottom. This plan builds that, extended to cover all five MIT
fields rather than position alone.

**Read for intent, implement fresh** (per CLAUDE.md): `engine_sketch.py`,
`nestle_sketch.py`, `undulate_sketch.py`, `recoil_sketch.py`, `posture_sketch.py`.
Do not copy-paste. The sketches predate the current code and contain three known
defects — see Stage 2.

## Target architecture

```
L5  Mood (ollama)      engine.request("curl", BehaviorParams(intensity=0.6))   ~0.5 Hz
L4  BehaviorEngine     weights, fades, Σ/Π composition                         every tick
L3  Primitives         (params, state) -> dict[int, JointIntent]               every tick
L2  Joint servo        scale, clamp, deg->rad, one MIT conversion              50 Hz
L1  Firmware           tau = kp*(p_des-p) + kd*(v_des-v) + tau_ff              20 kHz
```

`JointIntent` carries all five MIT fields, each with its own composition operator:

| Field          | -> MIT      | Operator      | Rationale                          |
|----------------|-------------|---------------|------------------------------------|
| `angle_deg`    | `p_des`     | sum, weighted | superposition: breathe while curled |
| `stiffness`    | `kp`        | product       | 0 annihilates, so limp dominates   |
| `damping`      | `kd`        | product       | lets purr raise kd above default   |
| `torque_ff`    | `tau_ff`    | sum, weighted | forces add physically              |
| —              | `v_des`     | derived       | backend already computes from ramp |

Gain scales are products (matching `PowerManager`'s existing `cmd.kp *= scale`),
clamped to `MOTOR_LIMITS` after composition.

---

## Stage 0 — Audit and cleanup

Goal: remove dead weight before adding a layer. No behaviour changes.
Work on `main`; land as one or more small commits before branching.

### Confirmed findings (already verified — do not re-derive)

- [x] **0.1** `MOTOR_LIMITS.kp_max` and `kd_max` are declared in `config.py` and
      referenced nowhere. `_encode_mit_packet` clamps only to the wire encoding range
      (kp 0–500, kd 0–5). Consequence: `PurrRippleMotion.KD_TARGET = 0.08` is 2x the
      declared `kd_max = 0.04` and has been passing through unclamped.
      Decide whether 0.04 is real (see Open decisions), then enforce in one place.
      **Re-verified 2026-09-22**, unchanged. Left as-is — this is Open decision #1
      (Nick's call) and its fix is explicitly Stage 1.1, not Stage 0.
- [x] **0.2** `CurlTowardsMotion` — its own docstring says "Identical to stroke-curl;
      provided as a named alias." `CurlAwayMotion` differs from it only by
      `DIRECTION_SIGN = -1.0`. `WalkMotion(SnuggleMotion)` overrides only `name`.
      **Re-verified 2026-09-22**, unchanged. Left as-is: collapsing these touches
      live motion code with no way to validate against hardware in this session
      (backend deps not installed here; robot moves need explicit permission
      anyway). Carrying forward to the Stage 4 migration map instead, since
      `curl.py` (3.2) subsumes all three regardless.
- [x] **0.3** `_check_stall` / `_do_reversal` / `_init_servo` exist in both
      `_WanderBase` and `NeighborAssistDriftMotion` as near-duplicates.
      **Re-verified 2026-09-22** — confirmed near-identical (same thresholds,
      same fields; `NeighborAssistDriftMotion` doesn't subclass `_WanderBase`
      because it adds neighbor-assist state, and `_do_reversal`'s return type
      differs: `tuple[float, float]` vs `None`). Same reasoning as 0.2: deferred
      to Stage 4, `drift.py` (3.5) replaces both.
- [x] **0.4** `docs/design/behaviors/config.py` is a stale copy of what shipped as
      `petctl/config.py`. Confirm it is reference-only, then note or remove.
      **Finding does not hold** — checked, it's already a 12-line stub that
      explicitly says "not imported by petctl... implement behaviors against
      petctl.config, not by copying values from here." No stale copy exists.
      No action needed.
- [x] **0.5** Four wave behaviours are one equation with different spatial phase:
      `snuggle` = (i/n)*4pi, `cascade` = frac*2pi + taper, `wiggle` = (-1)^i,
      `coil` = frac^2*2pi. Record this in the migration map (Stage 4), don't act yet.
      Recorded above; not acted on, per instruction.

### To investigate

- [x] **0.6** `scripts/` holds 13 one-off calibration and bench tools. Determine which
      are still runnable against current interfaces and which are historical. Move dead
      ones to `scripts/archive/` rather than deleting — several encode hard-won
      calibration method (`calibrate_power_model.py`, `sweep_kp_kd.py`).
      **Audited all 13.** Every `petctl.*` symbol they import (`RobotBackend`,
      `ROBOT_DEFAULT_HOST/PORT`, `MOTOR_LIMITS`, `LOOP_LIMITS`, `POWER_BUDGET`,
      `ControlLoopLimits`, `ServoCommand`, `ImuReading`, `RobotState`,
      `PowerManager`) still exists and resolves against current code (checked
      statically — this environment doesn't have `websockets` installed, so a
      live import wasn't possible; didn't install it without asking, per
      permissions). None are orphaned relative to `cli.py`/current interfaces.
      `calibrate_power_model.py` (free-spin) is methodologically superseded by
      `calibrate_power_model_static.py` (clamped-motor) for `torque_coeff` —
      see `config.py`'s own comment — but both remain valid reference for
      *how* the fit was done. Left all 13 in place; nothing moved to
      `scripts/archive/`.
- [x] **0.7** Audit `petctl/schemes/` for unreferenced modules: `sine.py`,
      `passthrough.py`, `command.py`. Check CLI wiring in `cli.py` before touching.
      All three are live `--control` options wired in `cli.py` (lines ~232-239)
      and documented in the CLI help text and README. Not dead. Left alone.
- [x] **0.8** Clear `__pycache__` from version control if tracked; confirm `.gitignore`.
      Already clean — `__pycache__/` and `.DS_Store` are both in `.gitignore`
      and `git ls-files` confirms neither is tracked anywhere in the tree.
      No action needed.
- [x] **0.9** Report anything else found. Do not delete outside the working folder.
      Two things found; both confirmed with Nick and actioned:
      1. **19 calibration data files (~17MB) were tracked in git** under `data/`,
         despite `.gitignore` excluding `data/` — added in `1526dcc` ("move
         script output to data/") before/without a matching `git rm --cached`,
         so the ignore rule never applied to them. Newer files in the same
         directory (the 2026-06-25 gesture/power-calib/sweep runs) were correctly
         untracked already. **Untracked with `git rm --cached`** — files remain
         on disk, `.gitignore` now actually applies to them going forward.
      2. **`docs/servos/*.alias`** — two tracked files were macOS Finder alias
         files (not symlinks, not portable, won't resolve on another machine),
         pointing at "FTServo_documentation" and "ftservo-python-websockets".
         FTServo is Feetech's servo line; current hardware per `CLAUDE.md` is
         CubeMars GL40 II — these were leftover references from before the
         motor swap. **Removed with `git rm`.**

**Deliverable:** a short written audit summary listing what was found, what was
removed, what was archived, and what was left alone with reasons. **Done — see
above; nothing was removed or archived. All findings confirmed accurate except
0.4. Two new findings (0.9) need Nick's call before any action.**

---

## Stage 1 — Branch and foundations

```
git checkout -b feat/shape-layer
```

Everything from here lands on that branch.

- [ ] **1.1 Enforce MOTOR_LIMITS at the encoder.** Clamp `kp`, `kd`, `torque_ff`,
      `vel` against `MOTOR_LIMITS` in `_encode_mit_packet` (or immediately upstream),
      log once per violation rather than per frame. Move `PurrRippleMotion.KD_TARGET`
      into `config.py`. This is the CLAUDE.md rule ("never hardcode servo limits")
      made enforceable.

- [x] **1.2 State recorder.** `petctl run --record <file>.jsonl` writing one line per
      tick: timestamp, sensors, servo positions/velocities/torques/temps, gesture frame,
      commands sent, power telemetry. Non-blocking (bounded queue + daemon thread,
      mirror the `_viz_worker` pattern — drop frames, never stall the loop).
      This is `BEHAVIOR_SYSTEM.md` 5.2, and it is a prerequisite for the post-ICRA
      touch-classifier training as well as for measuring whether Stage 2–3 helps.
      **Done 2026-09-22.** Built `petctl/recorder.py` — `StateRecorder(path, maxsize=64)`
      opens the output file line-buffered, `record(state, commands)` builds a
      JSON-serializable frame and does `queue.put_nowait`, dropping (and counting)
      on `queue.Full` rather than blocking; `close()` sends a sentinel, joins the
      writer thread (2 s timeout), and logs the drop count if any frames were lost.
      `ContactType` (a `(str, Enum)`) round-trips through `json.dumps` with no
      special-casing, as expected. Wired into `Controller`: new `record_file`
      constructor arg, `commands_sent` is exactly the `to_send` list actually handed
      to `backend.send_commands()` this tick (empty on dry-run or a send error), and
      `record()` is called once per tick right after the send block, before the
      hotkey-driven side effects (save-home, deactivate, etc.). `close()` runs in
      `_shutdown()`. CLI: `--record <path>.jsonl` on `petctl run`, passed through as
      `record_file`. Tests in `tests/test_recorder.py` (4): frame keys/values
      round-trip through JSON, `ContactType`/`GestureFrame` serialize as plain
      values, missing gesture/power_telemetry serialize as `null`, and a full queue
      (writer thread replaced with a no-op stub via `monkeypatch` so it never
      drains) drops frames and counts them without raising or blocking. No
      deviations from the plan.

- [x] **1.3 Mock dynamics.** `MockBackend.send_commands` currently does
      `self._servo_positions[sid] = cmd.position` — it teleports. Replace with a
      second-order joint model driven by the MIT law: integrate
      `tau = kp*(p_des-p) + kd*(v_des-v) + tau_ff` against a per-joint inertia and
      friction estimate. It does not need to be accurate; it needs to be *wrong in the
      same direction* as hardware so filter and blending work can be developed and
      tested offline. Expose inertia/friction as config so they can be fitted later
      from Stage 1.2 recordings.
      **Done 2026-09-22.** `send_commands()` now only records the latest setpoint
      per servo (`_ServoSetpoint`: position, velocity, kp, kd, torque_ff) — it moves
      nothing. `get_state()` calls `_step_dynamics(dt)` before building the returned
      positions, integrating `tau = clamp(kp*(p_des-p) + kd*(v_des-v) + tau_ff,
      MOTOR_LIMITS.torque_{min,max})`, `accel = (tau - friction*v) / inertia`,
      `v = clamp(v + accel*dt, MOTOR_LIMITS.vel_{min,max})`, `p += v*dt`, per servo.
      A servo with no setpoint yet runs at `kp=kd=torque_ff=0` (passive/coasting),
      matching a motor with no MIT frame sent. Added `MockDynamicsConfig`
      (`inertia_kg_m2=0.015`, `viscous_friction_nm_s_per_rad=0.03`) and singleton
      `MOCK_DYNAMICS` in `config.py`, commented as initial guesses to be fit from
      Stage 1.2 recordings, not measured values. `RobotState.motor_velocities` /
      `motor_torques` are now populated from the integrator instead of hardcoded
      `{}`. "file" mode is unaffected: `_build_servo_positions()` still overrides
      `self._servo_positions` from the JSON file every tick, after dynamics run, so
      the file wins exactly as before. Tests in `tests/test_mock_dynamics.py` (10):
      passive default, `send_commands` doesn't teleport, `position=None` commands
      are ignored, single-tick and converged (500-tick) MIT tracking, velocity/torque
      populated, both clamped to `MOTOR_LIMITS` under a large error, a clock-hiccup
      (negative dt) no-op, and the file-mode override still wins. Verified end-to-end
      with `petctl run --backend mock --record` for 3 s: 297 valid JSON frames
      written; PowerManager's emergency-stop path fired on the new nonzero
      torque/current estimate mid-run (dynamics now feeding real torque into the
      power model, as intended) and the controller still shut down cleanly. No
      deviations from the plan.

- [ ] **1.4 Resolve the layered slew filters.** Three first-order filters would sit in
      series once the engine lands, each with a different `dt` source:
      `Controller._apply_slew_to_commands` (tau=0.10 s LPF + delta cap, controller-tick dt),
      `RobotBackend.send_commands` (ramp + anti-windup, wall-clock dt), and the sketch's
      own `alpha = 5.0*dt` smoothing. Pick one owner. See Open decisions.
      Use 1.3 to compare candidate arrangements before touching hardware.

- [ ] **1.5 Control-loop rate.** `Controller._loop` sleeps `1/(motor_update_hz*4)`
      (~200 Hz) while `_pending_frames[sid]` is overwritten and consumed at 50 Hz —
      three of every four `motion.update()` results are discarded, and the slew filter's
      `dt` is 4x smaller than the true command period. Decide whether to run the loop at
      `motor_update_hz` or keep the oversample deliberately, and document why.

**Gate:** 1.1–1.3 must be green before Stage 2. 1.4–1.5 may be deferred if they prove
contentious, but record the decision either way — the engine inherits whatever is here.

---

## Stage 2 — BehaviorEngine

Build fresh in `petctl/behaviors/`. `patterns.py` stays untouched and working
throughout; the engine is an additional `--control behavior` option, not a replacement.

- [ ] **2.1** `petctl/behaviors/types.py` — `JointIntent` dataclass
      (`angle_deg`, `stiffness`, `damping`, `torque_ff`), and `BehaviorParams`
      (`intensity`, `speed`, `focus_modules`, `contact_face`, `extras`) carried over
      from the sketch.

- [ ] **2.2** `petctl/behaviors/base.py` — `Behavior` ABC:
      `update(state, params, dt) -> dict[int, JointIntent]`. Keys are **servo IDs 1–7**,
      not module IDs. Missing key means no contribution. Plus `reset()`.

- [ ] **2.3** `petctl/behaviors/engine.py` — `BehaviorEngine(Motion)`:
      registry, `request()`/`cancel()`/`cancel_all()`, weight fade in/out,
      Sum/Product composition per the table above, clamp against `BEHAVIOR_LIMITS`
      and `MOTOR_LIMITS`, single conversion to `ServoCommand`.

      **Three defects in `engine_sketch.py` to fix, not reproduce:**
      1. It subclasses `ControlScheme`; the shipped interface is `Motion`.
      2. It iterates `for mod_id in range(8)` with `servo_id = mod_id`. Module 0 is the
         head and has no servo; servos are 1–7. Make the module/servo mapping explicit.
      3. Its step-3 exponential smoothing is a third slew filter. Omit it, or make it
         *the* filter, per the Stage 1.4 decision.

- [ ] **2.4** `petctl run --control behavior` wiring in `cli.py`.

- [ ] **2.5** Tests: composition arithmetic (sum of angles, product of gains, clamping),
      fade in/out timing, empty-contribution handling, missing-servo handling.
      Pure functions on synthetic state — no backend needed.

---

## Stage 3 — Primitives

Each is a pure function of `(params, state)`. None holds per-servo bookkeeping; none
knows other primitives exist. Where perception is needed, read the value
`petctl/perception/` already computes — do not re-derive from raw pads.

- [ ] **3.1** `wave.py` — travelling wave. Params: spatial frequency (eta), amplitude,
      temporal frequency, envelope. Covers snuggle, walk, cascade, wiggle, coil, pulse.
- [ ] **3.2** `curl.py` — localized curl. Reads contact centroid and side from the
      gesture frame each tick (`qualifying_contact` / `StrokeReading.centroid` already
      provide this). Params: intensity, width, sign. Writes `angle_deg` only.
- [ ] **3.3** `stiffness.py` — writes `stiffness`/`damping` only, `angle_deg = 0`.
      Covers yield, freeze, idle, limp.
- [ ] **3.4** `purr.py` — writes `damping` only as a travelling wave, `angle_deg = 0`.
      Direct port of `PurrRippleMotion`'s existing approach; proves the gain channel.
- [ ] **3.5** `drift.py` — bounded random walk. Covers explore, contort, writhe.
- [ ] **3.6** Combination test: `wave` + `purr` + `curl` active simultaneously,
      verified against mock dynamics. This is the capability that does not exist today.

Not covered by primitives, and staying as standalone `Motion` classes: `twitch`,
`struggle`, `pose` — genuine state machines.

---

## Stage 4 — Migration

- [ ] **4.1** Migration map: each of the 27 `ALL_PATTERNS` entries to either a
      primitive + parameters, a standalone `Motion`, or retired. Write it down before
      moving anything.
- [ ] **4.2** Port the ollama scheme to call `engine.request()` with parameters instead
      of swapping named classes. This is the point of the whole exercise — the mood layer
      gains a continuous parameter space instead of 27 discrete labels plus `speed_gain`.
- [ ] **4.3** Hardware validation per primitive. **Ask before moving the robot.**
- [ ] **4.4** Retire superseded `patterns.py` classes only once their replacements are
      validated on hardware.

---

## Open decisions (Nick's call — ask, do not assume)

1. **Is `kd_max = 0.04` real?** Purr has been running at 0.08 since March with no
   apparent harm. Either raise the limit to match reality or lower purr to match the
   limit. Affects 1.1 and 3.4.
2. **Who owns the joint servo (1.4)?** Recommendation: the backend, since it already
   has anti-windup and the true per-motor timebase; strip the controller-level LPF to a
   pure safety clamp and let the engine do expressive smoothing. This changes how the
   robot feels, so it wants hardware time and probably a before/after recording.
3. **Does the motor swap land first?** If GIM3505/AK45 arrive during this work, any
   gain tuning against GL40 dynamics is discarded. If so, prioritise 1.3 and Stages 2–3
   (structure, testable offline) and defer 1.4 tuning until the new motors are in.
4. **Keep `patterns.py` indefinitely or retire it?** The plan assumes parallel operation
   through Stage 3 and selective retirement in Stage 4.

## Conventions

Per CLAUDE.md: Python 3.10+, type hints throughout, `from __future__ import annotations`,
dataclasses for data, ABCs for plugin interfaces, docstrings on public classes and
methods, stdlib/third-party/petctl import grouping, no global mutable state, all limits
imported from `petctl/config.py`.

Tests alongside each module as it is implemented.

**Always ask before creating, closing, or commenting on any GitHub issue.** Create
issues just-in-time as work starts on an item, link them inline here, and check the box
when the issue closes.

**Do not move the robot without explicitly asking permission.**
