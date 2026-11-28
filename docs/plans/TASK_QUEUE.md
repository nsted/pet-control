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
| T6 | DONE (set A frozen, data/rl/s2/suite) | T2 | Robustness-ladder level definitions + NEW frozen decision/test suites for S2 and S3 (never edit existing frozen suites) with SHA manifest |
| T7 | DONE (tests/test_s2_golden.py, tests/test_s2_eval.py) | T6 | Golden evaluation test: a scripted policy must reproduce known per-scenario numbers; evaluation asserts each scenario's config was applied |
| T8 | DONE (preview; video docs/media/s2_hold/pd_baseline) | T2 | HEADROOM PREVIEW ONLY (not a gate denominator): run scripts/rl/pose_hold_baseline.py with a kp gain sweep over the S2 poses and disturbances; report where PD has headroom (write results to docs/plans/STAGE_2_BASELINE.md; render video) |
| T14 | DOING (non-yield baseline DONE, STAGE_2_BASELINE.md; yield press check failed, with expert) | T6,T7 | S2 gate baseline: zero-delta PD, 18-config grid, through the frozen PetEnv evaluator on s2_hold decision+test at L0/L2/L3; best PD per (pose, orientation) cell; oracle-gain PD and scripted PI references (STAGE_2_GATES Baselines) |
| T15 | BLOCKED (owner: S2 cut) | T14 | S2 scripted hack policies (sag, pre-lean, redistribute, 0.94-cap PD, kp 3.2 / 5 max) + return-ordering test + no-termination test; cache the per-cell best-PD current table for the reward, alongside T9 |
| T17 | DONE | - | Holdable-lift pose search for S2 (CMA-ES over 10 joints, <= 1 h CPU, n = 11, upright; q* = settled pose): two poses, head lift (max head-height gain, need >= 5 cm) and one arch (max module height, need >= 8 cm). Each must hold tau_peak <= 0.32 N.m (last 2 s) at mass x1.2 / derate 0.8 and E < 0.1 at nominal and at the corner under kp 3.2. Spec: METHOD_REVIEWS T6 outcome |
| T18 | DONE (arch 1, arch 2) | T17 | More S2 lifted poses (same CMA-ES set-up as T17, <= 1 h): 3 distinct upright lifted poses (e.g. tail lift, arch, mid-body lift; a module >= 8 cm up). Constraints: tau_peak <= 0.28 N.m at the DR corner (margin, as the T17 arch failed marginally), E < 0.1 at nominal and corner, limp E >= 0.27 rad (a real hold, limp H <= 0.1). Then rerun the rev. 9 validity rule and freeze set A |
| T16 | BLOCKED (S2 cut pending) | T10 | 30 Hz vs 20 Hz ablation on the S2 shove family (1 seed pilot each); report to the owner with the control-rate question |
| T9 | BLOCKED (owner: S2 cut) | T8 | S2 hack tests in tests/ (stand-still, limp, thrash, random must score about nothing) |
| T10 | BLOCKED (owner: S2 cut) | T4,T5,T9 | Reviewer attacks the S2 reward/eval; S2 pilot (<= 10 min, via launch_run.py); videos |
| T11 | TODO | T8 | Prediction side track: logged-rollout dataset generator + 2-layer MLP multi-horizon predictors (torque, tracking error, gravity, gyro, bus current, T_coil; 0.1 / 0.5 / 2 s); gate A (>= 30% better than no-change at 0.5 s) |
| T12 | BLOCKED (owner) | - | Friction fix: geom priority so the floor's friction wins (touches the contact model: owner approval needed) |
| T13 | BLOCKED (owner) | - | Real-motor measurements (command delay, joint friction, rotor inertia, thermal time constant, battery voltage and limits): needed before S3 |
| T19 | DONE (DEV task loco_cmd + s3/metrics.py + tests/test_loco_cmd.py) | - | S3 locomotion task (DEV, not frozen): body-frame velocity commands (7 families + stop), 2 x 10 s segments, segment-fixed frame, T metric = 1 - norm(v - v_cmd) / norm(v_cmd) with a 1 s mask and 3-chord yaw, S_stop, no termination; reuse Locomote / start_orient (STAGE_3_GATES_PROPOSAL.md) |
| T20 | DOING (DEV evaluator petctl/rl/s3/evaluator.py done; golden test + frozen read-back pending) | T19 | S3 evaluator on a DEV suite + golden test + per-scenario read-back (command, segment frame, start type, slope, terrain, push, DR); a test that 2 different commands give different trajectories |
| T21 | DONE (DEV): selector M 0.015 at the deployable slew vs 0.303 relaxed (see CURRICULUM_STATE); bridge to the PHASE7 metric not done; owner decision on the joint-rate limit needed | T20 | Library selector baseline through PetEnv |
| T22 | TODO | - | S3 ladder definitions (DR separate from disturbances: slopes, ridges, pushes during locomotion, head mass; widened T13 ranges) + effective-parameter read-back test |
| T23 | TODO | T19 | Start-state bank: settled upright / side / inverted, collapsed (S2 yield and press end states), post-gait end states (10 s of each library gait); cached and hashed |
| T24 | TODO | T19,T23 | S3 hack tests (reuse the S1 1.12 harness): stand-still, limp, thrash, random from all 6 start types, abs(M) <= 0.05 and normalised return <= 0.05; moving-on-stop penalty test |
| T25 | TODO | T19 | S3 env throughput at n = 11 with terrain, thermal, history, kp action: >= 2,000 steps/s with 10 envs, else profile |
| T26 | DONE (result in METHOD_REVIEWS.md: cut confirmed, re-entry = S3/S4 start-state requirement) | - | S2 yield diagnostic |
| T27 | BLOCKED (owner: S2 cut, S3 suites and thresholds) | T20,T22,T23 | Reviewer attacks STAGE_3_GATES_PROPOSAL.md; freeze the S3 suites (320 per level, decision / test) and turn the proposal into STAGE_3_GATES.md |
| T28 | BLOCKED (owner, via T27) | T27,T21,T24 | S3 baselines on the frozen suites, then the teacher run (headroom 15%), the kp ablation pilots, then the RL pilot via launch_run.py |

When the queue runs low (fewer than 3 TODO), the expert adds the next tasks from the plan and the current stage's gate file (S2 full run, S3 baselines and teacher upper bound, S4 navigation and edge scenes, S5 distillation) so the loop is never starved.
