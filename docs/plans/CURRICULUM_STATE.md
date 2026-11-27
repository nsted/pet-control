# Curriculum state (read this first; ~1 page; keep it current)

Updated: 2026-11-26. Governing brief: `AUTONOMOUS_TRAINER.md` (overrides older RL plans). Redesign log: `METHOD_REVIEWS.md`. Recent history: `CURRICULUM_LOG.md` tail; old: `archive/` (do not read).

## Where we are
- Branch `feat/autonomous-trainer` (from f450903; tag `attempt-1-end`). The failed skill-gating experiment is on `exp/skill-gating-wip`. Attempt-1 chain (A1..B5 nav stages, hug parked at 0.12) is
  reference/seed material only, not a baseline to beat; the rebuilt robot's length is DECIDED: 11 modules (owner, 2026-11-26).
- **Phase 0 DONE** (branches/tag, ladder scripts in `scripts/rl/ladders/`, dead drivers in `scripts/rl/archive/`, BEST_RUNS torque note fixed).
- **Phase 1 DONE except owner installs:** watchdog + reward-term logging (`petctl/rl/watchdog.py`), orientation diagnostics, `start_orient`, frozen eval suites (`data/rl/eval_suite/`,
  `scripts/rl/eval_suite.py check`), contact sheet (`scripts/rl/contact_sheet.py`), run gating (`launch_run.py`, `mark_pilot.py`), hook + subagent PROPOSALS in `docs/proposals/phase1/`
  (owner installs; rule 8). Waiting for the owner: install, review, go-ahead for Phase 2.
- **Phase 2 DONE (2026-11-26)**, see `PHASE2_MODEL_CHECKS.md`: URDFs 10-13 verified, 10 CAD assumptions listed for the owner, `turn_obs`, anisotropic skin friction, n_envs = 10. Waiting for the owner's CAD confirmations (results stay provisional).
- **OPEN (owner, 2026-11-26): the motor is the SteadyWin GIM3505-8, not the GL40 II** the sim parameters came from (kp/kd, friction, armature, delay, bus-current model). Need the owner's data before the length study: see PHASE2_MODEL_CHECKS.md item 7.
- **Motor data in (2026-11-26), `MOTOR_GIM3505.md`:** current is quadratic in torque, I ~ 0.02 + 2.4 tau^2 A (hold, 13 V), old model 5.5x too high; budgets bind only with ~7+ joints near the cap. Sim current model NOT yet changed: awaiting owner approval. GDK manual read: MIT law = ours, kp 0..100 / kd 0..5 default (registers, 12-bit), frame layout same as GL40 profile; still unknown: delay, friction, rotor inertia.
- Owner decisions 2026-11-26: gains are dynamic control variables (Phase 7 action space; `PetSim.set_gains` done); current model unvalidated -> proxy only, no gate. Open: GIM3505-8 gain range, measurements.
- **Phase 3 DONE (2026-11-26)**, `PHASE3_CPG_STATICS.md`: CPG basis validated, statics at 0.5 N.m (head lift ~19 cm, wrap r 9-11 cm, self-right from side ok / from inverted needs a dynamic move), no consistent length signal. Owner: static gap bridge dropped, dynamic platform-to-platform crossing instead; `cma` approved + installed; owner leans to n = 11. Next: Phase 4 (CMA-ES + MAP-Elites, seed from old repertoire, gap crossing + inverted self-righting as targets).
- (superseded) Next: Phase 3 (CPG basis in joint-pair coordinates + feasibility statics).

## Fixed rules (do not re-litigate)
Sim only, never move the robot. Torque cap 0.5 N.m; current 10 A peak / 4 A mean, shared bus. Flipping is never a failure. Std-runaway fix stays (ent_coef 0, log_std_init -1).
Gates / eval suites frozen without owner approval. Pilot (0.5-1.5 M steps) + contact sheet + adversarial reviewer before every full run. Max 3 knob iterations per stage, then design change,
then method review. Budget runs in steps (~2k steps/s with 10 envs), not hours.

## Token-cost rules (binding)
1. No polling crons: `scripts/rl/event_watch.sh` (plain shell) wakes Claude on STOP_* / DONE / driver dead / idle machine.
2. Routine decisions follow written stop rules; Opus subagents only for the adversarial review and method reviews.
3. Read this file + `tail -c 3000 CURRICULUM_LOG.md`; never `archive/` or the whole log.
4. REQUIRED: one contact-sheet review per pilot and per full-run end. Videos optional and event-driven (rendered after a run ends, never viewed by default).
5. Batch work; small edits in the main session; no subagent for a < 10 min task.
6. Log entries: result + decision, <= 5 lines.

- **LENGTH DECIDED: n = 11 (owner, 2026-11-26).** Basis: owner preference plus the pre-agreed tie-break (lengths within seed noise on gaits; mixed on poses; even lengths keep more speed when inverted, 57-67% vs 40% for 11, a cost the skill library absorbs). It was NOT decided by a pre-registered data study: Phase 6 (5 seeds per length, robustness, weighted score) is skipped by the owner's choice. Evidence: `PHASE4_GAIT_SEARCH.md`. All further work is at n = 11 only.
- **Phase 4 at n=11:** CMA-ES + MAP-Elites done, yaw metric fixed (head thrash artefact), dynamic gap crossing works at 10 and 15 cm (open loop), inverted self-righting search next, then Phase 5 poses.
- **Phase 7 (night 2026-11-26/27):** `petctl/rl/cpg_env.py` (policy outputs 11 CPG-parameter increments + 2 gain scales, tracks commanded body-frame velocity). Pilot 1 (old reward) ran 1M steps in 5 min; the adversarial reviewer found NO-GO: standing still earned ~0.47 (narrow kernel, 3/8 commands need no translation), yaw from head-tail chords gameable by thrashing, heading frame shape-dependent, no current model. Fixed: improvement-over-standing-still kernel (sigma 0.4|v|), standing earns 0 and moving on a stand command is penalised, yaw only when 3 chords agree, heading frame fixed per command segment, first second after a command change masked, bus-current proxy and 5x saturation penalty. `tests/test_cpg_env.py` (stand / zero / random < 0.05, thrash < 0.10). Pilot 2 with the new reward running.

- **State 2026-11-27 morning:** Phase 4 done at n=11 (robust library replaces the overfit one), self-righting (two-stage) and gap crossing to 20 cm work, staged pose entry 233 poses. Phase 7: 12 PPO pilots in CPG-parameter space, none beats the library selector (0.086); four adversarial reviews, last NO-GO, no long run launched. Next: (a) blend gaits for forward+turn and other missing combinations, (b) longer / different RL only after a pilot beats the selector in the corrected eval, (c) Phase 5 poses beyond staged heaps, (d) Phase 8 skill selector with transitions (self-right, cross gap, locomote). No ask-first items hit during the night.
