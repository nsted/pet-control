# STAGE_1_GATES: S1 Foundations (pre-registered 2026-11-28, rev. 1 after STAGE_1_2_GATES_REVIEW.md, before any S1/S2 result)

Author: rl-robotics-expert. Plan: `RL_RESTART_PLAN.md` section 3 (S1). Audit: TASK_QUEUE T1. Changing a threshold here after results are seen needs the owner's approval.

**Task.** Make `PetEnv` at n = 11 match what the real robot can sense and do, and make the evaluation trustworthy, so that S2 can be judged. S1 has no learning gate. It passes when every REQUIRED item below passes its acceptance test in `tests/` (pytest at `n_modules = 11`). Timebox: 3 working days from T2 DONE. At the timebox, report what is left. T8 (`pose_hold_baseline.py`) may run in parallel, but only as a **headroom preview**: the S2 gate baseline is re-measured through the frozen evaluator after 1.10 passes (STAGE_2_GATES, Baselines).

## Decisions (logged in METHOD_REVIEWS.md 2026-11-28)

- **Control rate: 20 Hz for sim training. This conflicts with CLAUDE.md (30 Hz) and is OPEN for the owner; it blocks deployment, not S2.** Reasons for 20 Hz:
  - The kHz inner MIT loop does the fast rejection.
  - At equal step count, 30 Hz covers 2/3 of the simulated time (1.5M steps = 20.8 h vs 13.9 h).
  - The horizon is longer at 20 Hz (gamma 0.99 = 5 s vs 3.3 s).
  - Slew, history and PPO settings are calibrated at 20 Hz.
  - There is more margin for the 1-2 tick reply delay.

  The 30 Hz shove ablation is queued (T16). Would change my mind: that ablation beating 20 Hz beyond the seed CI, or a measured command delay over 50 ms.
- **Velocity:** use the direct quantised MIT velocity channel (+-30 rad/s, 12 bit, step 60/4096 rad/s, same delay and drops as the position field). This replaces the derived +-10-clipped velocity.
- **Torque field:** +-2 N.m, 12 bit (sim assumption about the registers; setting them on the robot is an owner item).
- **Position convention:** the actor sees the wrapped angle plus a turn count (`turn_obs` on), as the brain would rebuild them from consecutive frames. q* and the targets are multi-turn. Pose error is wrapped to (-pi, pi].
- **CUT / deferred:**
  - T_mos (T_coil only).
  - Frozen S3 suites, which move to the start of S3 (stage-order change).
  - The friction fix (T12, BLOCKED), which does not block S1.

## Acceptance tests (REQUIRED = blocks the S2 pilot)

| id | item | pass criterion | REQ |
|---|---|---|---|
| 1.1 | n = 11 | every new test and the S2 env preset build at n = 11; obs and privileged lengths match the formula; the S2 preset never terminates: on ALL 24 (pose, orientation) cells, independent of 1.9, 300 steps of limp and of thrash give `terminated == False` throughout, `truncated == True` exactly at step 300 and not before, and the VecEnv's `info["TimeLimit.truncated"] == True`, so the SB3 bootstrap is exercised | yes |
| 1.2 | velocity | (a) with delay = 0, drops = 0 and noise off: RMS(obs vel - true qvel) <= 0.05 rad/s on a 10 s random-target rollout; (b) the step is 60/4096; (c) separately, with delay d in {1, 2}: obs vel equals true qvel from d ticks earlier (to within the quantisation step), the same lag as the position field | yes |
| 1.3 | torque | step 4/4096 N.m, saturates at +-2 N.m; noise, delay and drops as before | yes |
| 1.4 | thermal RC | per-motor I^2 R first-order model; T_coil observed at 1 degC steps with the reply delay. Steady state within 1% of the analytic value; 63% step response at tau +-2%; derating monotone from the knee to zero at the limit; start temperature and tau randomised (assumption: tau 60-600 s, knee 80 degC, limit 100 degC; all UNVALIDATED until T13) | yes |
| 1.5 | history | stacked 10-tick sensor history plus the last action; slot k equals the obs from k ticks ago (checked by replay). The actor obs contains NO time or phase feature (asserted against the obs-layout spec) | yes |
| 1.6 | kp action | per-joint kp = kp_ref x s, s in [0.25, 6.25] (= 0.2-5.0 N.m/rad, widened 2026-11-28 so the policy reaches the best-PD gain) via an exponential map. kd = kd_ref sqrt(s) x zeta_m, with zeta_m fixed per episode (a DR parameter, not an action): training range [0.7, 1.4] from L1 up, 1 at L0 and in every evaluation. Stability: all joints at s = 6.25 and at s = 0.25, zeta_m in {0.5, 2}, random target jumps every 0.5 s for 60 s: no NaN, max abs(qvel) <= 50 rad/s, no MuJoCo warning | yes |
| 1.7 | log_std | clamped to [-3, -1] (init -1, ent_coef 0); unit test; note in METHOD_REVIEWS when first used | yes |
| 1.8 | ladder realised | the ladder file holds fixed ranges per level (STAGE_2_GATES). The test reads back the EFFECTIVE parameters during a rollout, not the spec: the friction actually used in `d.contact[i].friction`, masses, cap, delay, noise, zeta_m, start T_coil. Until T12 lands, module-friction factors below the floor's value are marked NOT REALISED in the ladder file (the realised range is max(floor, module)), and the test asserts this | yes |
| 1.9 | S2 suites | NEW frozen `s2_hold` decision and test suites built from the STAGE_2_GATES recipe, with a SHA manifest that lists the pose sources, the valid orientations and the excluded cells; existing suites untouched (`eval_suite.py check` passes) | yes |
| 1.10 | golden test | (a) **analytic check, independent of the evaluator:** gravity 0, contacts off, tail module welded (test only), straight pose, static force F on the head at known kp: steady-state joint deflections within 5% of J^T F / kp. (b) **regression:** zero-action PD at two gain settings reproduces stored per-scenario numbers within 1e-6 rad on the same machine and 1e-3 rad across platforms. (c) **read-back per scenario:** force vector, body, direction, onset, duration, tilt, mass, cap, orientation, gains, zeta_m, delay ticks, noise multiplier, effective friction, start T_coil, bus voltage, level. (d) **restoration:** before scenario k+1 is applied, every parameter reads back nominal. (e) two scenarios that differ only in the disturbance give different trajectories | yes |
| 1.11 | watchdog | `min_metric` is read from the stage's pre-registered baseline json; tested with a fake baseline | yes |
| 1.12 | hack policies | reusable policies in `tests/`: stand-still (a = 0), limp (kp 0), thrash (a = +-1 with the sign alternating every tick, random initial sign per joint; the kp channel alternates between its ends), random (a ~ U[-1, 1] iid per tick). Delta semantics: target_t = target_{t-1} + 0.25 a, with a clipped to [-1, 1] and then the slew limit of 0.35 rad/tick, so random is a random walk of the targets | yes |
| 1.13 | throughput | 10 envs, n = 11, all items on. >= 2,500 steps/s: pilot = 1.5M steps. 2,000-2,500: pilot = 1.2M steps (logged before the pilot). < 2,000: profile before the S2 pilot | yes |
| 1.14 | multi-turn | with a joint driven past +-12.5 rad (the MIT position-field range): the obs (wrapped angle + turn count) and E stay continuous | yes |
| 1.15 | bus clamp | the BusClamp substep matches sim_dt (the config comment says 2 ms, sim_dt is 4 ms); fix the code or the comment | yes |
| 1.16 | capacitive | still in the obs interface (post-only model), masked for S2; test that it is present and constant | no |
| 1.17 | deploy rate | a test that the brain's policy rate equals `EnvConfig.control_hz` (fails until the owner decides; non-blocking for S2) | no |

## NO-GO and authority

S1 is NO-GO for the S2 pilot while any REQUIRED item fails.
- The main trainer MAY: choose implementation details, tighten tolerances, split items, fix bugs, and report a failing item with its cause.
- The trainer MAY NOT: loosen a criterion, edit an existing frozen suite, change the gain or zeta ranges, add a kd or zeta action, change the floor friction, or start the S2 pilot. These go to the expert, and thresholds go to the owner.
