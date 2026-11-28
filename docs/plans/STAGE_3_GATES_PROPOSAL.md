# STAGE_3_GATES_PROPOSAL: S3 Locomotion with feedback (PROPOSAL, NOT pre-registered)

Status: DRAFT by rl-robotics-expert, 2026-11-28. This file becomes `STAGE_3_GATES.md` (pre-registered) only after the owner (1) approves the S2 cut, (2) approves the S3 frozen suites and thresholds below, and (3) has answered T13 or accepted the widened ranges. It needs the reviewer's attack before that. Until then, nothing here is a gate, and DEV numbers do not count.

Lessons carried over (PHASE7_CPG_RL.md, S2 method review):
- Standing still must earn 0.
- Yaw counts only when 3 chords agree.
- The heading frame is fixed per command segment.
- Gaits must work after other gaits (only 18/43 did).
- The eval must check that each command was actually applied (the bug that ran one gait for every command).
- A stiff PD cannot be beaten at holding, so holding is tested here only as a regression family.

## Task

Track a body-frame velocity command from ANY start state.
- **Rate and length:** n = 11, 20 Hz, 20 s episodes made of two 10 s command segments, so every episode contains one switch.
- **No termination:** truncation only, as in S2.
- **Actions:** joint-target deltas plus the per-joint kp action (0.2-5.0 N.m/rad); no CPG.
- **Observations:** IMU (gravity direction, gyro, accelerometer; no absolute yaw), MIT position / velocity / torque, T_coil, the 10-tick history, the last action, and the command. Capacitive is masked.
- **Optional:** a sin/cos clock ablation.

**Command families (7 + stop):**

| family | values |
|---|---|
| forward / back | 5-15 cm/s along the tail-to-head chord |
| left / right | 5-15 cm/s, lateral |
| turn left / right | 0.3-1.0 rad/s |
| combined | translation and turn together |
| stop | zero command |

The body frame is fixed at the start of each segment: the tail-to-head chord projected on the floor.

**Start types:**
- upright;
- left side;
- right side;
- inverted;
- collapsed: end states from the S2 yield and press draws (from the start-state bank, T23);
- post-gait: end states after 10 s of a library gait or of another command (from the start-state bank, T23).

## Metric

Per segment, after a 1 s mask, with v the true COM planar velocity over 1 s windows in the segment frame:

T = clip(1 - norm(v - v_cmd) / norm(v_cmd), -1, 1)

- Turns use the same formula on omega, counted only when 3 chords agree.
- Combined commands use the mean of the translation and yaw terms.
- Standing still gives T = 0, the wrong direction gives T < 0, and thrashing with no net displacement gives about 0.

**Main metric M:** the mean of T over the non-stop segments of a suite. Stop segments are scored separately as S_stop = clip(1 - speed / 3 cm/s, -1, 1). S_stop is gated only against the hack policies, never rewarded as a "do nothing" win.

**Homeostasis (all gated):**
- the demanded bus current holds the 2 s mean <= 4 A and the 50 ms peak <= 10 A in >= 99% of ticks;
- T_coil never exceeds its limit;
- the graded load function from S2 is reported.

Mean current is reported against the selector's. It is not gated, because it is an unvalidated proxy.

**Statistics:** CIs use the two-level cluster bootstrap (seeds, then scenarios) from S2.

## Baselines (measured first, through PetEnv and the frozen evaluator)

1. **Robust library selector (the speed bar):** the PHASE7 selector with f-scaling and the ease-down / ease-up transitions, re-played as joint-target deltas in PetEnv. From side and inverted starts it is preceded by the two-stage self-right skill.
   - Bridge test: on the old metric it must reproduce the PHASE7 numbers within the CI before it is scored on M.
   - The old 0.086 is NOT the bar. The bar is the selector's M on these suites.
2. **Privileged teacher:** PPO whose actor also sees contacts, the true velocity, friction and mass.
   - Headroom rule: if M_teacher < 1.15 x M_best_baseline on the L2 decision suite after a full-length teacher run (1 seed, <= 1 h), change the TASK, not the knobs.
3. **Best fixed-kp RL (kp ablation):** RL with the kp channel frozen at each of {0.8, 1.6, 3.2, 5.0}, one pilot each; the best one is the reference. The kp action stays in S3 only if full RL beats the best fixed-kp RL beyond the seed CI. Otherwise it is dropped, to keep S3 simple.

## Ladder (domain randomisation separate from disturbance magnitude; train at L2, evaluate at L2 and L3)

**Domain randomisation:**
- friction, mass, derate, zeta_m and start T_coil as in S2;
- widened until T13 arrives: command delay 1-3 ticks, joint friction 0-0.05 N.m, armature x0.5-2, noise x1-2;
- friction below the floor's value is NOT REALISED until T12. For locomotion this hole matters more than in S2, so T12 is a priority owner item.

**Disturbances (L0 / L1 / L2 / L3):**
- slope: 0 / <= 5 / <= 10 / <= 15 deg;
- terrain: flat / flat / small ridges / ridges;
- push during locomotion: <= 2 / 3 / 5 / 8 N;
- head mass: x1 / 1.5 / 2 / 3.

L0 is nominal randomisation with the L1 disturbances.

**Ladder score:** M_L3 / M_L0 >= 0.60, AND M_L3 >= the selector's M_L3 (absolute floor, so "robust by not moving" fails).

## Regression family (from the S2 method review)

1. Apex press or shove DURING locomotion (0.5-3 s, up to 1.2 x the S2 F_sat). Metric: T over the 3 s after release >= 0.7 x the same command's unpressed T.
2. Stop command with pushes at rest: S_stop, and lost (S2 definition) = 0.
3. Collapsed starts: M from collapsed starts >= 0.6 x M from upright starts.

## Suite size (decision and test, disjoint seeds, per level L0 / L2 / L3): 320 scenarios

- **Main part, 240 scenarios:**
  - 6 start types x 8 command pairs = 48 combinations. Each pair has 2 segments; every non-stop family appears in >= 2 pairs and at least one pair contains a stop segment.
  - x 5 draws of domain randomisation and disturbance each.
- **Regression family, 80 scenarios:** 40 press-during-locomotion, 20 stop-with-pushes, 20 extra collapsed starts.
- **Cost:** 320 x 400 steps is about 128k steps per level per policy, 1-2 min.
- Suite validity is fixed before freezing: limp, stand-still and thrash must have M <= 0.05.

## Gates (thresholds PROPOSED for the owner's approval)

| gate | pass |
|---|---|
| hack tests, every level and all 6 start types | stand-still, limp, thrash, random: abs(M) <= 0.05; their normalised training return <= 0.05 (the plan's rule applies literally here); a stop-command test: moving on stop is penalised |
| 2-min overfit | one command, upright: T >= 0.3 |
| pilot (seed 0, 1.2-1.5M steps per S1 1.13) | M >= 1.10 x M_best_baseline on the L2 decision suite, scenario-CI lower bound > M_best_baseline. Expected curve, UNSURE: M > 0.1 upright by 1.5M steps |
| full (seeds 0, 1, 2; each <= 1 h) | on both L2 suites: mean M >= max(1.3 x M_sel, M_sel + 0.10); worst seed >= M_sel + 0.05; CI lower bound > M_sel |
| full: coverage | T >= 0.2 in every command family, and >= the selector in >= 5 of 7 families |
| full: orientation | every start type has M >= 0.6 x upright M and >= 0.15 |
| full: robustness and regression | the ladder score above; the regression family above; homeostasis holds |
| student vs teacher | > 20% short of the teacher sends teacher-student (RMA) to a method review |

## Must come from the owner first

- Approve the S2 cut.
- Approve these suites and thresholds (they then become pre-registered).
- T13 real-motor measurements (command delay, joint friction, rotor inertia, thermal tau, battery voltage and limits), or acceptance of the widened ranges.
- T12 friction fix.
- 20 vs 30 Hz control rate.

Risk (honest): direct joint-space locomotion from scratch has not been shown on this body. In PHASE7, PPO never beat the selector in CPG-parameter space. The pilot bar may need a longer budget than 1.5M steps. If so, that is a stage-order or budget decision for the owner; the bar itself is not lowered.
