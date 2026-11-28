# TASK_QUEUE: the backlog the idle loop draws from

Status values: TODO | DOING | DONE | BLOCKED (owner). `deps` lists task ids that must be DONE first. The main trainer updates statuses; the `rl-robotics-expert` adds and reorders tasks (and writes the stage gate files). The `idle-check` agent only reads this file. Keep entries one line each; put detail in the plan or the stage gate file.

Plan: `docs/plans/RL_RESTART_PLAN.md`. Scope: movements, navigation, environment contact; human-contact work is deferred.

| id | status | deps | task |
|---|---|---|---|
| T1 | DONE | - | S1 audit (<= 20 lines): what PetEnv / obs / config / asym / capacitive / watchdog / evalsuite / launch_run already provide vs what is missing (thermal model, control rate 20 vs 30 Hz, stale GL40 velocity clip in obs.py, per-joint kp scale with tied kd, golden eval test, ladder levels) |
| T2 | DONE | T1 | Expert writes docs/plans/STAGE_1_GATES.md and STAGE_2_GATES.md; reviewer attacks them; show the owner |
| T3 | DONE (knob mit_frame; tests/test_mit_frame.py; rate = 20 Hz kept, open for owner) | T1 | Fix obs.py velocity clip for the GIM3505-8 and resolve the control-rate question (report the decision to the owner) |
| T4 | DONE (petctl/rl/thermal.py, knobs thermal + thermal_accel; tests/test_thermal.py) | T1 | Thermal RC model (I^2 R) + quantised T_coil observation + torque derating near the limit; randomised starting temperature; unit tests |
| T5 | DONE (action_mode delta_kp, logstd.py clamp; tests/test_delta_kp.py) | T1 | Action option: joint-target deltas with one kp scale per joint and kd tied to a damping ratio (no CPG parameters); tests incl. the gain-extreme stability test |
| T6 | TODO | T2 | Robustness-ladder level definitions + NEW frozen decision/test suites for S2 and S3 (never edit existing frozen suites) with SHA manifest |
| T7 | TODO | T6 | Golden evaluation test: a scripted policy must reproduce known per-scenario numbers; evaluation asserts each scenario's config was applied |
| T8 | DONE (preview; video docs/media/s2_hold/pd_baseline) | T2 | HEADROOM PREVIEW ONLY (not a gate denominator): run scripts/rl/pose_hold_baseline.py with a kp gain sweep over the S2 poses and disturbances; report where PD has headroom (write results to docs/plans/STAGE_2_BASELINE.md; render video) |
| T14 | TODO | T6,T7 | S2 gate baseline: zero-delta PD, 18-config grid, through the frozen PetEnv evaluator on s2_hold decision+test at L0/L2/L3; best PD per (pose, orientation) cell; oracle-gain PD and scripted PI references (STAGE_2_GATES Baselines) |
| T15 | TODO | T14 | S2 scripted hack policies (sag, pre-lean, redistribute, 0.94-cap PD, kp 3.2 / 5 max) + return-ordering test + no-termination test; cache the per-cell best-PD current table for the reward, alongside T9 |
| T16 | TODO | T10 | 30 Hz vs 20 Hz ablation on the S2 shove family (1 seed pilot each); report to the owner with the control-rate question |
| T9 | TODO | T8 | S2 hack tests in tests/ (stand-still, limp, thrash, random must score about nothing) |
| T10 | TODO | T4,T5,T9 | Reviewer attacks the S2 reward/eval; S2 pilot (<= 10 min, via launch_run.py); videos |
| T11 | TODO | T8 | Prediction side track: logged-rollout dataset generator + 2-layer MLP multi-horizon predictors (torque, tracking error, gravity, gyro, bus current, T_coil; 0.1 / 0.5 / 2 s); gate A (>= 30% better than no-change at 0.5 s) |
| T12 | BLOCKED (owner) | - | Friction fix: geom priority so the floor's friction wins (touches the contact model: owner approval needed) |
| T13 | BLOCKED (owner) | - | Real-motor measurements (command delay, joint friction, rotor inertia, thermal time constant, battery voltage and limits): needed before S3 |

When the queue runs low (fewer than 3 TODO), the expert adds the next tasks from the plan and the current stage's gate file (S2 full run, S3 baselines and teacher upper bound, S4 navigation and edge scenes, S5 distillation) so the loop is never starved.
