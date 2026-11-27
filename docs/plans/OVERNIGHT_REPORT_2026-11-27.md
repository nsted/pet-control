# Overnight report, 2026-11-26/27 (autonomous trainer, n = 11, sim only)

Videos: `docs/media/index.html` (git-ignored). Details: PHASE4_GAIT_SEARCH.md, PHASE5_POSES.md, PHASE7_CPG_RL.md, CURRICULUM_STATE.md. Nothing touched the robot, no installs, no GitHub issues, no destructive git, no frozen-suite or physical-limit changes (one constants addition: GIM3505_MIT_KP_MAX / KD_MAX in config.py, the MIT encoding ranges from the motor doc).

## Decided
- Length 11 modules (owner). Phase 6 study skipped by owner decision; basis: preference + tie-break, lengths indistinguishable on gaits (seed noise), mixed on poses.

## Solid (checked by video and perturbation tests)
- Robust gait library, n = 11 (`map_elites_robust.py`, `merge_library.py`): 99 elites that keep their velocity across friction x1.3 (39/43) and, less well, after another gait (18/43). Forward 16, reverse 26, left 18, right 23 cm/s, turns up to 1.3-1.9 rad/s, rolling gaits.
- Two-stage self-righting (rolling CPG gait until the whole body is upright, then ease to straight): 17/18 perturbed starts.
- Skill selector prototype (`skills_demo.py`): flipped -> self-right, else locomote with transitions; re-detects accidental flips.
- Dynamic gap crossing at 10 cm, robust (5/6 perturbed starts).

## Weak or negative
- Gap crossing 15 / 20 cm: only from the exact search start (fragile); no robust crossing found (best 2/4 and 1/4 conditions). Needs closed-loop heading alignment or a different objective.
- Poses: staged entry makes 233 / 5259 lattice poses reachable (72 with all joints at once), local search plateaus at 17.5 cm; they are compact heaps / lifted head. Tall structured poses are not reachable under the 0.5 N.m cap.
- RL in CPG-parameter space (13 PPO pilots, 4 adversarial reviews): no policy beats the library selector (corrected eval: worse on every translation command at pilot 13). RL steering paused. Long run never launched (review 4: NO-GO).
- Selector translation tracking is only moderate (0.05 per tick with commands switching every 6 s).

## Mistakes found and fixed (all mine)
1. First gait library overfit to one start state (5/34 gaits kept their velocity under friction x1.3 or after another gait): broke steering and RL. Rebuilt robustly.
2. Yaw metric aliased by a thrashing head (faked 5-11 rad/s turns): fixed (all chords must agree).
3. `eval_cpg.py` ran the same gait for every command: contact sheets / eval.json of pilots 4-12 were invalid (training curves fine); retracted in PHASE7_CPG_RL.md.
4. Self-righting "success" was a rolling body that happened to be upright at the end; criterion tightened to upright at every tick of the last 5 s.
5. Floor friction x0.7 does nothing in MuJoCo (max of the two contact frictions): friction randomisation only works upward.

## Needs you
- RL: is steering by the selector + skills enough for the installation, or do you want RL pursued (longer run, tighter residuals)? I would not continue it without a reason.
- Gap crossing beyond 10 cm and tall poses: worth the effort (closed-loop alignment; non-lattice angles / external support)? Or is 10 cm + head lift + wrap enough?
- Phase 8 remainder: terrain, obstacles, climbing, hug skills with the robust library; pose skills inside the selector.
- Still open from before: GIM3505-8 data (delay, joint friction, rotor inertia, battery voltage), CAD assumptions 2-9, memory-path exception for the guard hook.
