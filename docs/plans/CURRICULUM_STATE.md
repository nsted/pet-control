# Curriculum state (read this first; ~1 page; keep it current)

Updated: 2026-11-26. Governing brief: `AUTONOMOUS_TRAINER.md` (overrides older RL plans). Redesign log: `METHOD_REVIEWS.md`. Recent history: `CURRICULUM_LOG.md` tail; old: `archive/` (do not read).

## Where we are
- Branch `feat/autonomous-trainer` (from f450903; tag `attempt-1-end`). The failed skill-gating experiment is on `exp/skill-gating-wip`. Attempt-1 chain (A1..B5 nav stages, hug parked at 0.12) is
  reference/seed material only, not a baseline to beat; the rebuilt robot's length (10-13 modules) is undecided.
- **Phase 0 DONE** (branches/tag, ladder scripts in `scripts/rl/ladders/`, dead drivers in `scripts/rl/archive/`, BEST_RUNS torque note fixed).
- **Phase 1 DONE except owner installs:** watchdog + reward-term logging (`petctl/rl/watchdog.py`), orientation diagnostics, `start_orient`, frozen eval suites (`data/rl/eval_suite/`,
  `scripts/rl/eval_suite.py check`), contact sheet (`scripts/rl/contact_sheet.py`), run gating (`launch_run.py`, `mark_pilot.py`), hook + subagent PROPOSALS in `docs/proposals/phase1/`
  (owner installs; rule 8). Waiting for the owner: install, review, go-ahead for Phase 2.
- **Phase 2 DONE (2026-11-26)**, see `PHASE2_MODEL_CHECKS.md`: URDFs 10-13 verified, 10 CAD assumptions listed for the owner, `turn_obs`, anisotropic skin friction, n_envs = 10. Waiting for the owner's CAD confirmations (results stay provisional).
- **OPEN (owner, 2026-11-26): the motor is the SteadyWin GIM3505-8, not the GL40 II** the sim parameters came from (kp/kd, friction, armature, delay, bus-current model). Need the owner's data before the length study: see PHASE2_MODEL_CHECKS.md item 7.
- **Motor data in (2026-11-26), `MOTOR_GIM3505.md`:** current is quadratic in torque, I ~ 0.02 + 2.4 tau^2 A (hold, 13 V), old model 5.5x too high; budgets bind only with ~7+ joints near the cap. Sim current model NOT yet changed: awaiting owner approval; still unknown: MIT protocol / kp-kd range, delay, friction.
- Owner decisions 2026-11-26: gains are dynamic control variables (Phase 7 action space; `PetSim.set_gains` done); current model unvalidated -> proxy only, no gate. Open: GIM3505-8 gain range, measurements.
- Next: Phase 3 (CPG basis in joint-pair coordinates + feasibility statics).

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
