# METHOD_REVIEWS — redesign log and method reviews

Rules (AUTONOMOUS_TRAINER.md): log every redesign of action space / task / reward / curriculum here with its reason; write a review every 10 runs and after a stage fails twice
with different causes. Newest first.

## 2026-11-26 Phase 2 design decisions (model checks; no reward / task change)
- `turn_obs` knob (obs.py): turn counts of joint angle / target appended last, because wrapped observations hid the multi-turn state. Default off.
- Anisotropic skin friction (`petctl/sim/skin.py`): lateral/forward ratio via an extra Coulomb force; `DomainRand.friction_aniso` default (1, 1) = isotropic for all training (owner decision: ABS is isotropic); anisotropy is an optional robustness check only (e.g. final gaits at ratios 1.5 and 2.5).
  Costs ~45 % throughput; stride 2 chosen after testing 2/3/4. Eval-suite scenarios now realise `friction_mean` and `friction_aniso` (suite files unchanged, hashes OK; only the generator comment changed
  from "forward/lateral" to "lateral/forward").
- Owner correction: motor = SteadyWin GIM3505-8 (CLAUDE.md / motor profile still say GL40 II). No sim change made yet; waiting for the owner's numbers.
- n_envs = 10. Details and the CAD-assumption list: PHASE2_MODEL_CHECKS.md.

## 2026-11-26 Phase 1 design decisions (safeguards; no reward or task change yet)
- **Flip is not a failure (env).** `allow_flip` is now honoured for every task in `PetEnv.step` (default 0 = old behaviour, so existing stages are unchanged); new work sets
  `allow_flip: 1`. Reason: brief says flipping never ends an episode; self-righting and rolling gaits need it.
- **Orientation diagnostics.** Every episode now logs `orient_inverted_s`, `orient_flips`, `orient_recoveries`, `orient_recovery_s`, `orient_end_inverted` ("inverted" = tail-IMU
  up . world-up < `flip_up_dot` 0.2, so lying on the side also counts; recovered = back above 0.5). Per-term episode sums `term_<name>` are now logged for every task (was goal_* only).
- **Start orientations.** Task knob `start_orient` (0 upright, 1 upside-down, 2 left side, 3 right side, 4 random roll) rolls the body about its long axis before settling.
- **Watchdog** (`petctl/rl/watchdog.py`, on by default, `PETCTL_WATCHDOG=0` disables): TensorBoard `terms/<name>_mean|_share`, `diag/*`; stop rules in `judge()`. Caveat: in-run
  it judges the TRAINING-episode task metric (stochastic policy) unless a decision-set evaluator writes `decision_metric.jsonl`; the decision set is not evaluated inside the loop yet
  (cost). Thresholds are conservative defaults, to be calibrated on the first pilots (a share-jump limit of 0.30 and a 4-snapshot trend window are guesses).
- **Eval suites** (`data/rl/eval_suite/`, `petctl/rl/evalsuite.py`): 6 skills x {decision, test} x 24 scenarios, specs only (no body length). Fields `command`, `friction_aniso`,
  `obstacle_density`, `pose_family` are frozen now but become live in Phase 2 / 7 / 8; until then the env realises start orientation, twist, terrain, goal and dr level. SHA-256 manifest +
  `scripts/rl/eval_suite.py check`; TEST needs `PETCTL_OPEN_TEST=1` + a reason (logged). `.gitignore` now un-ignores `data/rl/eval_suite/` (was under the blanket `data/`).
  Any later change to these files needs owner approval (approval 1).
- **Run gating.** `scripts/rl/launch_run.py pilot|full`, `scripts/rl/mark_pilot.py`, `scripts/rl/contact_sheet.py`; the hook proposal in `docs/proposals/phase1/` enforces them.
- ~~Known gap: n_modules 10-13 do not use the URDF generator~~ WRONG: the builder has always used `urdf_gen` for n != 8 (corrected in Phase 2, see PHASE2_MODEL_CHECKS.md).
- **Known pre-existing failure:** `tests/test_gate2.py::test_gate2_hard_fails` fails on f450903 and on this branch alike.
