# Shape Layer — Implementation Plan

Status: **Stage 0 complete (2026-09-22), audit-only — nothing modified. Stage 1.1, 1.2,
1.3, 1.6, 1.7, and 1.8 complete (2026-09-22)**; 1.4, 1.5, 1.9 still open. Supersedes the
sequencing in `BEHAVIOR_SYSTEM.md` (0/17 complete since March); that document's Stage 1
is absorbed into Stage 2 here.

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

- [x] **1.1 Enforce motor limits at the encoder.** Clamp `kp`, `kd`, `torque_ff`,
      `vel` in `_encode_mit_packet` (or immediately upstream), log once per violation
      rather than per frame. Move `PurrRippleMotion.KD_TARGET` into `config.py`.
      This is the CLAUDE.md rule ("never hardcode servo limits") made enforceable.
      Enforce against the **active motor profile** (1.7), not a module-level constant.

      **Done 2026-09-22.** `vel` was already fixed by 1.7 (clamped to
      `self._profile.encoding.vel_min/max`, the v_des wire range). Added
      `RobotBackend._clamp_to_profile_limits()`, called from `send_commands`
      immediately before `self._profile.encode_command(...)` — clamps `kp` to
      `[0, profile.gains.kp_max]`, `kd` to `[0, profile.gains.kd_max]`, and
      `torque_ff` to `[profile.encoding.torque_min, .torque_max]`. This is
      stricter than `encode_command`'s own wire-range clip (kp 0-500, kd 0-5 for
      GL40_II), which only guards packet overflow, not the declared safety
      ceiling — a kp=300 command used to reach the motor as kp≈300 (wire-clipped
      only at 500); it's now clamped to 1.5 before encoding. Violations log once
      via `_warn_once_per_violation` (a `(motor_id, field)` set, cleared on
      recovery so a later violation logs again) rather than every tick at
      `motor_update_hz`.
      `PurrRippleMotion.KD_TARGET` (0.08, 2x `kd_max`) is gone — per Nick's
      resolution of Open decision #1, purr's peak kd is now `MOTOR_LIMITS.kd_max`
      directly (itself profile-sourced since 1.7), not a separate class constant
      or a new config field for a value that must always equal `kd_max`.
      Verified: manual clamp test shows the wire frame actually encodes the
      clamped kp (not the raw 300), log fires once on violation onset, is
      silent on repeat and on recovery, and fires again on a later violation.
      All 83 tests still green.

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
      **Bugfix 2026-09-22:** `_step_dynamics` was clamping the integrator's
      physical velocity to `MOTOR_LIMITS.vel_min/vel_max` (±0.5 rad/s) — that's
      the MIT wire-encoding range for the `v_des` feedforward field (see 1.7),
      not a physical speed limit. `backends/robot.py`'s real ramp filter moves
      the physical position at `LOOP_LIMITS.max_speed_rad_s` (6.0 rad/s) and
      only clamps the separate `v_des` wire value to `MOTOR_LIMITS.vel_min/max`.
      The mock was capping simulated joint speed at ~12x slower than real
      hardware, defeating 1.3's stated purpose ("wrong in the same direction,"
      not wrong by over an order of magnitude). Fixed to clamp against
      `LOOP_LIMITS.max_speed_rad_s`; `tests/test_mock_dynamics.py`'s velocity
      test updated to match. Re-verified end-to-end with `--record`: 608 frames,
      no errors; PowerManager's worst-case-power floor now trips an emergency
      stop within the first tick on `stroke-curl` at 7 active motors under the
      dev budget (4.0A) — expected from the existing bin-pack model, not new.

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

- [x] **1.6 Log commanded vs actual position.** `RerunVisualizer` plots
      `motor_velocities`, `motor_torques` and temperatures per servo, but nothing logs
      `RobotState.servo_commanded_positions` — the post-slew setpoint the Controller
      already populates each tick from `_slew_last_sent_rad`. Without it the plots show
      where each joint *is* with no reference for where it was *told* to be, so filter
      lag and tracking error are invisible. Add a `motors/position/motor_N` series pair
      (commanded and actual) alongside the existing ones.
      Prerequisite for 1.4 — the gap between the two curves *is* the thing 1.4 is tuning.

      **Done 2026-09-22.** `Controller` already populated `servo_commanded_positions`
      every tick (`controller.py:623`) — the gap was purely in `RerunVisualizer`, which
      never logged it. Added `motors/position/motor_N/actual` and `.../commanded` as a
      `SeriesLines` pair per servo in `_setup_motor_series` (same pattern as the
      existing velocity/torque/temperature series, module 0 excluded since it has no
      servo), logged each tick from `state.servo_positions` / `state.servo_commanded_
      positions` in `_log_motor_state`, and added a `motors/position` `TimeSeriesView`
      to the blueprint alongside velocity/torque/temperature. Verified with a fake `rr`
      recorder (correct paths, correct values, module 0 correctly excluded) and against
      the real `rerun` SDK (`rr.init(spawn=False)` + the same log calls, no exceptions).

- [x] **1.7 Motor profile boundary — everything above L2 must be motor agnostic.**
      All hardware specifics belong behind one `MotorProfile`, selected by config/CLI,
      with `GL40_II` as the first instance. Nothing above the joint servo may import a
      motor constant.

      Currently GL40-specific and leaking:
      - `MotorLimits` is docstringed "Hard limits for CubeMars GL40 II MIT-mode
        commands". `pos +/-12.5`, `vel +/-0.5`, `torque +/-1.0` are that driver's MIT
        *encoding ranges*, not physical limits — a different driver packs different
        ranges (the SteadyWin GDS34 T_Max unit basis is still an open question).
      - `_encode_mit_packet` hardcodes 16/12/12/12/12 bit field widths.
      - `kp_default = 0.8`, `kd_default = 0.035` are tuned to the GL40's Kt (0.11 Nm/A).
        A geared candidate with far higher Kt needs different numbers for the same feel.
      - `MOCK_DYNAMICS` inertia and friction are per-motor.

      **A module, not a constant table** — some of it is behaviour, not data, so
      `MotorProfile` is an ABC (matching `protocols.py`) with `GL40_II` as the first
      implementation. Suggested home: `petctl/motors/base.py` + `petctl/motors/gl40.py`.

      The profile owns, in four groups:
      1. *Encoding* — ranges, field widths, `_encode_mit_packet` / `_parse_slcan`, and
         the enable/disable/set-zero magic frames (`...FFFC/FFFD/FFFE`). These are
         methods: another driver may pack a different layout entirely, not just
         different bounds.
      2. *Gains* — `kp_default`, `kd_default`, and the max values; what
         `JointIntent.stiffness = 1.0` resolves to in Nm/rad.
      3. *Thermal* — `temp_soft_warning_c` 55 / `temp_hard_cutoff_c` 65 /
         `temp_global_emergency_c` 75. These are GL40 numbers and they currently live
         in `power_manager.py`, not `config.py` — already a CLAUDE.md violation
         ("import all hardware limits from petctl/config.py"). Fix while moving.
      4. *Power model* — `per_motor_base_a`, `per_motor_torque_coeff` (72.52, fit to
         GL40 at a specific TMAX encoding), `per_motor_mech_coeff`,
         `per_motor_worst_case_w`, and mock inertia/friction. All per-motor, all
         currently inside `PowerBudgetConfig`.

      **Stays out of the profile:** everything in `PowerBudgetConfig` that describes
      PET's wiring rather than its motors — bus current ceilings, the 8A slip-ring
      limit, UPS vs wall thresholds, battery calibration. Those survive a motor swap
      unchanged. The split is the point: a motor swap should touch one module, and a
      rewiring should touch a different one. Consequence for the layer above: **`JointIntent.stiffness` and
      `damping` stay normalized 0–1 scales, never raw kp/kd** — already how they are
      specified in this plan, and the reason it survives a motor swap. The profile
      defines what 1.0 means in Nm/rad, chosen so felt stiffness stays comparable
      across motors. A behaviour written today then runs unchanged on GIM3505 or AK45.

      **Done 2026-09-22.** Built `petctl/motors/base.py` (`MotorProfile` ABC +
      `MotorEncodingRanges`/`MotorGains`/`MotorThermalLimits`/`MotorPowerModel`/
      `MotorFeedback` frozen dataclasses, plus the shared `float_to_uint`/
      `uint_to_float`/`byte_to_int8` bit-packing helpers) and
      `petctl/motors/gl40.py` (`GL40_II(MotorProfile)`, all four groups from the
      list above, plus `encode_command`/`encode_enable`/`encode_disable`/
      `encode_set_zero`/`encode_zero_torque`/`decode_feedback` — the 16/12/12/12/12
      field layout lives in these methods, not as a constant). `RobotBackend` now
      takes `motor_profile: MotorProfile = ACTIVE_MOTOR_PROFILE` and calls
      `self._profile.*` for every encode/decode; the module-level
      `_encode_mit_packet`/`_encode_mit_enable`/`_encode_mit_disable`/
      `_encode_mit_set_zero`/`_encode_mit_zero`/`_float_to_uint` functions are
      gone (`scripts/rate_test.py` updated, was calling `_encode_mit_zero`
      directly). Verified the new `encode_command`/`decode_feedback` produce
      byte-identical output to the pre-refactor functions across 2000 randomized
      inputs each, plus the enable/disable/set-zero frames.
      `config.py` gets `ACTIVE_MOTOR_PROFILE = GL40_II()`; `MotorLimits`,
      `MockDynamicsConfig`, and `PowerBudgetConfig`'s four `per_motor_*` fields
      now read their defaults from it instead of hardcoding GL40 numbers, with
      `MotorLimits`'s docstring corrected (encoding ranges, not physical limits).
      `power_manager.py`'s `PowerThresholds` thermal fields (`temp_soft_warning_c`
      / `temp_hard_cutoff_c` / `temp_global_emergency_c` / hysteresis) now default
      from `ACTIVE_MOTOR_PROFILE.thermal` — fixes the CLAUDE.md violation named
      above. All of `MOTOR_LIMITS`/`MOCK_DYNAMICS`/`POWER_BUDGET`/`PowerThresholds`
      keep their existing field names and numeric values, so no call site outside
      `petctl/motors/` and the four files touched here needed to change.

      **Deviations from the plan, both intentional:**
      1. `_parse_slcan` (SLCAN text → `(can_id, payload bytes)`) stayed in
         `backends/robot.py` rather than moving into the profile. It's generic
         CAN-over-SLCAN framing — true of any driver on this transport — not
         motor-specific; only *interpreting* the payload (field widths, motor ID
         nibble, err nibble) is GL40-specific, and that's `decode_feedback`.
      2. `types.py`/`schemes/patterns.py`/`schemes/command.py`/`schemes/
         passthrough.py` still read `MOTOR_LIMITS.kp_default`/`.kd_default`/
         `.pos_max` directly — i.e. "above the joint servo" still imports a
         motor constant, just indirectly through the config.py compatibility
         layer described above. Getting all 27 `patterns.py` classes off
         `MOTOR_LIMITS` and onto `ACTIVE_MOTOR_PROFILE` (or the future
         `JointIntent`) with no way to validate against hardware in this session
         is the same call Stage 0 (0.2) made about touching that file — deferred
         to the Stage 4 migration map, not attempted here. The seam
         (`petctl/motors/`) exists now; wiring everything above L2 through it is
         Stage 4.

- [x] **1.8 Triage the 5 red `test_power_manager.py` tests.** Pre-existing, not caused
      by this branch (confirmed via `git stash`). Almost certainly test drift: the test
      file is unchanged since `46780cb`, while `power_manager.py` has six commits after
      it — including `c6d7962` ("emergency stop when current exceeds peak limit for
      500ms"), which changed behaviour. For each failure decide whether the test or the
      implementation is wrong; do not simply update assertions to match current output,
      since this is the layer that owns thermal cutoff, current budget and emergency
      stop. Stage 2 rests on this layer being trustworthy.

      **Done 2026-09-22. Root cause: not `c6d7962` — `git bisect`-by-reading found
      `aff0cfb` ("power: switch to production UPS limits (4A budget, 5.5A peak, 6A
      wall)", Nick, 2026-06-25).** All 5 failures were `TestBinPackSeed`/
      `TestBinPackPromotion` cases that construct a bare `PowerManager(bin_policy=...)`
      with no `budget_override`, so `_effective_budget()` fell through to the live
      `POWER_BUDGET.max_bus_current_a` default. That default was intentionally moved
      from the dev value (2.0A) to the production UPS value (4.0A) in `aff0cfb` — a
      real, human-authored config change, not a bug — and every one of these 5 tests
      hardcoded arithmetic ("at dev budget (2A) ... only 1 motor seeds") that silently
      broke the moment the global default no longer matched what the test assumed.
      The implementation (`allocate_budget`'s bin-pack seeding/promotion) is correct
      and untouched.

      **Fix:** the 4 tests whose point is the seeding/promotion *algorithm* (not which
      real-world budget is configured) now pin `budget_override=2.0` explicitly, so
      they test the algorithm against a known number instead of silently trusting
      whatever `POWER_BUDGET` happens to default to — this is the fix that survives
      the next intentional budget change, not "update the assertion to match today's
      number" (which would just make this the same kind of landmine again).
      `test_seed_detects_wall_power_source` is different: its entire point is that
      `_effective_budget()` selects `wall_max_bus_current_a` over `max_bus_current_a`
      once `PowerSource.WALL` is detected, so it can't use `budget_override` (that
      bypasses power-source selection for both branches). Instead its expected active
      motor count is now derived from the live `POWER_BUDGET` (`wall_max_bus_current_a`
      / worst-case per-motor current) rather than the stale hardcoded "1", with an
      assertion that the fixture still exercises something (`1 <= expected_n < 7`) so
      a future budget change that makes the test vacuous (e.g. all 7 or 0 motors fit)
      fails loudly instead of silently passing.
      All 83 tests green (`pytest -q`); no production code changed for this item.

- [ ] **1.9 Minimal CI.** No `.github/workflows/`. A single job running `pytest` on push
      would have caught 1.8 six commits earlier, and matters more once Stage 2 starts
      adding a layer that the existing suite doesn't cover.

**Gate:** 1.1–1.3 and 1.7–1.8 must be green before Stage 2. 1.4–1.5 may be deferred if they prove
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

- [ ] **2.6** Log per-behaviour contributions to Rerun (`behaviors/<name>/motor_N`)
      alongside the blended result, so a blend can be read apart visually. Extends 1.6
      from two curves to N+1.---

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

1. ~~**Is `kd_max = 0.04` real?**~~ **Resolved 2026-09-22: 0.04 stands.**
   `PurrRippleMotion.KD_TARGET` comes down from 0.08 to comply, and moves into the
   GL40 II profile (1.7) rather than staying a class constant. Purr will feel weaker
   than at ICRA/ICSR — expect to recover the effect through the crest envelope
   (`CREST_POWER`) or ripple rate rather than peak kd. Affects 1.1 and 3.4.
2. **Who owns the joint servo (1.4)?** Recommendation: the backend, since it already
   has anti-windup and the true per-motor timebase; strip the controller-level LPF to a
   pure safety clamp and let the engine do expressive smoothing. This changes how the
   robot feels, so it wants hardware time and probably a before/after recording.
3. ~~**Does the motor swap land first?**~~ **Resolved 2026-09-22: staying on GL40 II
   for now.** 1.4 and 1.5 are unblocked — tune against GL40 dynamics. Nick's standing
   constraint: *everything here must be motor agnostic*, which is what 1.7 enforces.
   Gain values are GL40-specific and live in its profile; structure above L2 is not.
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
