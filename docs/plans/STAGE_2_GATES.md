# STAGE_2_GATES: S2 Hold and comply (pre-registered 2026-11-28, rev. 9 (T6 pose rules) after the rev. 4 re-check in STAGE_1_2_GATES_REVIEW.md, before any baseline)

Author: rl-robotics-expert. Plan: `RL_RESTART_PLAN.md` section 3 (S2). Changing a threshold here after results are seen needs the owner's approval. Senses: proprioception only (IMU, MIT feedback, T_coil); capacitive masked.

## Task and episode

The policy holds a known pose q* under hidden disturbances. In the yield family it gives way under an overload, then returns.
- Actions: joint-target deltas plus one kp scale per joint (STAGE_1 items 1.6 and 1.12).
- Rate and length: n = 11, 20 Hz, 15 s episodes.
- Start: the episode starts in the **settled pose**, a cached sim state per (pose, orientation) made by the scripted entry and a 4 s settle under the reference PD (kp 3.2, zeta_m 1; see Pose rules). The policy never does the entry.
- Disturbance timing (hidden): onset ~ U[3, 7] s; it lasts 3 s (a shove lasts 0.1 s); recovery runs to the end of the episode (>= 5 s). The actor has no time feature (1.5).
- **No termination in S2: truncation at 15 s only.** Flip, lost and every other condition are metrics, never terminals; with a cost-only reward any terminal would pay the policy to end early. Test: STAGE_1 item 1.1 (all 24 cells, truncation exactly at step 300).

**Poses (sources listed in the suite manifest):**
- straight (zeros);
- head lift: q = alpha x the `data/rl/statics/results.json` n = 11 seed-0 vector (alpha rule below; the stored entries sit at the cap);
- lattice tall, mid and compact (`data/rl/pose_graph_n11.json`, sorted by height / footprint as in `pose_hold_baseline.py`);
- wrap: q = alpha x the `data/rl/statics/wrap.json` n = 11, r = 9 cm best held entry, held on the floor without the cylinder.

**Pose rules (rev. 6, T6 finding: with kp_ref 0.8 every lifted pose sags by tau/kp, so validity E < 0.1 needs tau < 0.08):**
- **Reference PD:** cell validity, holdability and the second settle use kp 3.2, zeta_m 1. The RL kp range and the best-PD grid are unchanged.
- **q* is the settled pose (rev. 8).** The stored statics vectors command angles beyond contact, so a PD presses into the floor (preload): the torque then grows with kp and says nothing about holding. Per cell, at nominal DR:
  1. scripted entry with the stored vector at kp 0.8, then a 4 s settle;
  2. q* := the measured joint angles;
  3. a 4 s settle with target q* under the reference PD;
  4. cache that state.

  q* is per cell, frozen in the manifest, and given to the policy as the commanded pose. E is then the gravity sag from q*, with no preload. A cell is dropped if q* differs from the commanded shape by more than 0.3 rad on any joint, because it did not reach the pose.
- **Orientations for lifted poses:** cells whose scripted entry does not reach the pose from a side or inverted start are dropped by the rule above; no staged side entry is built now. Straight keeps all four orientations, so the any-orientation rule is covered there.
- **Holdability is checked per (pose, orientation) cell.** (rev. 7) A cell passes if, settled at the L2 DR corner (mass x1.2, derate 0.8; NO head mass, which is a load-family disturbance, not DR), it has tau_peak <= 0.8 x cap_eff and no loss. tau_peak is measured over the LAST 2 s of the settle (the static hold), not during the entry ramp. The full corner (with head mass x2) is reported as a stress test only. In load scenarios, failing to hold is a legitimate outcome that E scores. A failing cell is excluded and listed in the manifest. A pose survives if >= 1 cell passes.
- **Alpha (head lift and wrap):** the largest alpha in {1.0, 0.9, ..., 0.2} for which a cell passes, subject to a floor so the pose stays itself:
  - head lift must keep a head-height gain of >= 5 cm over straight at nominal;
  - wrap must keep alpha >= 0.6 (below that it is no longer a wrap).
- **Lattice:** tall must have height >= 14 cm (the 15.7-17.5 cm band); mid must be within 9-13 cm and differ from tall by >= 3 cm; compact is the smallest footprint. Each takes the first pose in its sort order that passes inside its band, and otherwise it is dropped. A pose 10 places down the height order (11.6 cm) is a duplicate of mid, not a tall pose.
- **Minimum set:**
  - (rev. 9) >= 3 NON-TRIVIAL poses survive (limp-trivial cells, limp E < 0.1, do not count);
  - head lift survives (the lift is closest to PET_PURPOSE);
  - >= 2 poses with a lifted part have >= 1 valid, non-trivial cell each after the suite-validity rule (yield family).

  Otherwise the suite is NO-GO. The next step is then an OWNER decision on narrowing the L2 mass or derate ranges (a threshold change), not a further rule change by the expert.

**Orientations:** upright, left, right, inverted. A (pose, orientation) **cell** is valid if the reference PD (kp 3.2, zeta_m 1) settles with E < 0.1 rad and no loss. Validity is decided at suite generation and recorded.

**Families:**

| family | what it does | count per cell |
|---|---|---|
| push | sustained force; body from {head, mid, tail-1}; direction drawn from +-x/y/z in the module frame | 4 |
| shove | impulse, 0.1 s | 2 |
| load | downward force or extra head mass | 2 |
| tilt | gravity rotated about a random horizontal axis | 2 |
| derate | torque cap scaled | 1 |
| none | no disturbance | 1 |
| yield | overload at 1.2, 1.6 and 2.0 x the press that saturates >= 3 joints under the L2 best PD (best PD selected on the NON-yield scenarios first, then the press calibrated; one press per cell, used at every level; frozen in yield section B, see Two-phase freeze); only on cells with a lifted part (head lift, lattice tall, lattice mid), so that yielding lets the floor carry the press | 3 |

- Suite size: decision and test suites are built from this recipe with disjoint seeds, about 320 scenarios each per level.
- **Training mix:** push 0.20, shove 0.15, load 0.15, tilt 0.15, derate 0.10, yield 0.15, none 0.10. "Double yield" means yield 0.30, with the others scaled down in proportion. Training samples ONLY valid, non-excluded cells (the suite-validity set).

## Ladder: domain randomisation (DR) is separate from disturbance magnitude

S2 trains at L2 and is evaluated at L2 (the gate) and L3.

| level | friction x (realised >= floor until T12) | mass x | derate | delay (ticks) | noise | zeta_m | start T_coil | push / shove / load | tilt | head mass |
|---|---|---|---|---|---|---|---|---|---|---|
| L0 | 1 | 1 | 1 | 1 | nominal | 1 | 30 degC | <= 3 N / 8 N / 3 N | <= 5 deg | x1.5 |
| L1 | 0.7-1.4 | 0.9-1.1 | 0.9-1 | 1-2 | nominal | 0.7-1.4 | 25-50 | <= 3 N / 8 N / 3 N | <= 5 deg | x1-1.5 |
| L2 | 0.5-1.8 | 0.8-1.2 | 0.8-1 | 1-2 | nominal | 0.7-1.4 | 25-75 | <= 5 N / 12 N / 4 N | <= 10 deg | x1-2 |
| L3 | 0.4-2.0 | 0.7-1.3 | 0.7-1 | 1-3 | x2 | 0.7-1.4 | 25-85 | <= 8 N / 16 N / 6 N | <= 20 deg | x1-3 |

Evaluation always uses zeta_m = 1.

## Metrics (frozen evaluator, golden-tested; deterministic policy)

- **E** (rad): mean wrapped abs(q - q*) over the 10 joints and the WHOLE episode, settle included. Windows: E_settle = [0, onset), E_dist = [onset, onset + 4 s), E_rec = last 1 s. G is reported per window. **Lost:** any joint > 0.6 rad off for > 1 s anywhere in the episode. **Winding** (joints whose unwrapped error is > pi at the end) is reported only.
- **Current** (the exact code path): `bus_motor=1`, BusClamp in demand mode, voltage fixed at GIM3505_V_TEST. All checks use the DEMANDED (pre-clamp) current:
  - I = episode mean;
  - mean budget: max of the 2 s running mean (BUS_MEAN_WINDOW_S) <= 4 A;
  - peak budget: the 50 ms tick mean <= 10 A.

  This is an unvalidated proxy. Robustness check: re-evaluate RL and PD with GIM3505_MOVE_A_PER_NM2 in {2.4, 4.6, 7.0}; best PD stays the config selected at 4.6, only the currents are re-measured; the gate must pass in all three.
- **Best PD per cell AND per level:** the stable grid config (below) with the lowest sum of E over the cell's NON-yield decision scenarios AT THAT LEVEL (L0, L2, L3 each selected on their own decision suite), among configs that hold both budgets in every scenario. Its E and I are re-measured on the test suite of the same level. Every G, G_dist and current constraint at level k uses the level-k selection.
- **The L2 selection is the reference** for everything that is not level-specific: the reward's current table, the press calibration, the loaded joints, trimmed PD, the hack policies and the yield PD.
- **G (per seed, per cell c):** G_c = 1 - sum_s E_RL(s) / sum_s max(E_PD(s), 0.005), over the cell's non-yield scenarios. G is the equal-weight mean of G_c over the valid, non-excluded cells.
- **Disturbance-increment gain:** for each scenario, D = max(E_dist - E_settle, 0). G_dist = 1 - sum_s D_RL / sum_s max(D_PD, 0.005), per cell, then the equal-weight mean over cells. Static trim lowers E_settle and E_dist alike, so it does not earn G_dist. Raising E_settle to shrink D is blocked by the settle check.
- **Current constraint (an RL pass condition, not a comparator):** in every cell, I_RL <= 1.05 x I_PD + 0.05 A, with both averaged over the cell's NON-yield scenarios (yield current is checked only against the budgets, so yielding cannot pay for extra current elsewhere).
- **Hold score:** H = clip(1 - E / 0.3, 0, 1), aggregated like G.
- **Yield score:** Y = 0.5 (1 - load) + 0.5 clip(1 - E_rec / 0.1, 0, 1), with load = mean over overload ticks of [0.5 max_j f + 0.5 x (mean of the top 3 f over all 10 joints, chosen per tick)], where f(tau) = clip((abs(tau)/cap_eff - 0.5) / 0.4, 0, 1), tau is the TRUE applied actuator force (`d.actuator_force`, not the observed torque), and cap_eff is the derated cap. There is no cliff. A single joint pinned at the cap costs at least 0.67 of the load term.
- **Chatter (reported, soft flag):** hold-phase joint-velocity RMS and action-rate ratio against best PD. A ratio > 2 triggers a video check, not a gate.
- **CIs:** 95% two-level cluster bootstrap with 10,000 resamples (resample seeds, then scenarios within cells). The single-seed pilot interval is a "scenario CI".

## Two-phase freeze

1. At T6, SHA set A freezes:
   - the NON-yield decision and test suites (all levels, the cell set, the poses and alpha);
   - the yield draws (cell, onset, body, direction, multiplier 1.2/1.6/2.0, seeds) for every level.

   Only the press magnitude and loaded joints are written into B.
2. At T14 the yield section B is written ONCE, with its own SHA. It holds only the press magnitudes and the loaded joints per cell. The per-level best-PD selections and the current-reference table are T14 results, stored in the baseline json that the watchdog reads.
3. A test asserts all of the following:
   - SHA set A is unchanged after T14;
   - the yield draw records hash into A;
   - B contains no draw fields;
   - B was written only once (a second write fails).

## Baselines

1. **T8 (`pose_hold_baseline.py`) is a headroom preview only.** It runs in a different plant (no env pipeline, unwrapped error, other grid). Its numbers are never a gate denominator.
2. **Gate baseline (T14):** zero-delta PD through the frozen PetEnv evaluator, after 1.10 passes, on the L0, L2 and L3 decision and test suites. Grid: kp in {0.2, 0.4, 0.8, 1.6, 3.2, 5.0} N.m/rad x zeta_m in {0.5, 1, 2}, with kd = kd_ref sqrt(kp / kp_ref), giving 18 configs. Unstable configs are recorded. RL can reach at most kp 3.2, so keeping 5.0 in the grid is conservative.
3. **References (not gates):**
   - oracle-gain PD: the best config per scenario;
   - a scripted PI: a target-offset integrator on best PD. If G comes only from integral action, it is labelled as such.
4. **Privileged teacher:** the same PPO with the privileged vector in the actor. First a pilot (it must reach G >= 0.10, else suspect the pipeline), then one full-length run (1 seed, <= 1 h).

**Headroom rule.** The rule is judged on the FULL teacher run. If the teacher reaches G < 0.30, or best PD has E_dist <= 0.03 rad (disturbance window, not whole-episode E) with no losses, the task has no headroom. Then:
1. Change the task once: train at L3 and double the yield share.
2. If the teacher is still below 0.30, stop work on pose error. Report to the owner and propose cutting S2 to the yield family.

Oracle-gain G over best PD is reported alongside as a second headroom signal. The student is not trained while the rule fails.

## Gates

| gate | when | pass |
|---|---|---|
| suite validity | T6, before freezing; decided ONCE at L2 and the same cell set applied at L0 and L3 | (rev. 9) Cells where limp has E < 0.1 rad are LIMP-TRIVIAL. They are removed from the suite and from the denominator (reported only; they were never hold cells). If the aggregate (mean) limp H over the remaining cells is > 0.10, exclude further cells in ascending order of limp E until it is <= 0.10, provided >= 60% of the non-trivial scenarios remain; otherwise NO-GO (back to the expert). Exclusions are listed in the manifest |
| 2-min overfit | before the pilot | one scenario (head lift, push head 5 N, fixed onset), with the kp channel FROZEN at that scenario's best grid gain <= 3.2 (the RL range): E at least 10% below that PD within 2 min; otherwise it is a pipeline bug |
| pilot (seed 0, budget from 1.13) | GO to full | G >= 0.10 at L2 on the decision suite, with the scenario-CI lower bound > 0; current constraint and budgets hold; settle check (below); all hack tests pass; the reviewer has no open blocker. Anything else is NO-GO. Expected: G > 0 by 1/3 of the budget |
| full (seeds 0, 1, 2; each <= 1 h, ~6M steps) | stage pass | at L2, on both decision and test suites: mean G >= 0.30, worst seed >= 0.20, CI lower bound >= 0.20; G_dist >= 0.15 (mean over seeds) and worst seed G_dist >= 0.10; current constraint in every cell, under all three move coefficients; lost fraction <= best PD's; settle check: aggregate E_settle <= best PD's + 0.01 rad, and no cell worse by > 0.03 rad; yield: Y >= 0.80 and Y >= best-PD Y + 0.15; ladder: G_L3 >= 0.10, H_L3 / H_L0 >= 0.60 and H_L3 >= 0.30 |
| student vs teacher | after full | if the student's G is > 20% below the teacher's, teacher-student (RMA) goes to a method review |

## Hack tests (`tests/`, scored with the frozen evaluator at L0, L2 and L3)

A failure is NO-GO for the pilot.

**Scripted policies:**

| policy | must score |
|---|---|
| stand-still (a = 0, nominal gains) | G <= 0.05 on the TEST suite |
| thrash, random (definitions in 1.12) | G <= 0 and H <= 0.10 |
| limp | aggregate H <= 0.10 (suite validity) |
| sag: best PD with targets offset by 0.05 rad in each joint's gravity-torque direction at settle | G <= 0.05 |
| pre-lean: best PD with a fixed target offset that cancels best PD's steady deflection under the cell's downward load (the one fixed direction left), held all episode | G <= 0.05, or the settle check fails; reported beside the PI reference |
| trimmed PD: best PD with q* offset by its own settled error, per cell | G_dist <= 0.05 (reported: its G, as the trim reference) |
| partial drop: best PD with the lifted joints' targets moved 0.3 rad toward the floor | return-ordering test below |
| one joint pinned: on yield cells, all lifted joints relaxed (kp 0.2) except the most loaded one, held at >= 0.95 x cap through the overload | Y <= 0.70 |
| redistribute: on yield cells, PD with best PD's 3 most-loaded joints at kp 0.2 and the rest at 5.0 | Y <= 0.50 |
| max kp at 3.2 and at 5.0 (a = 0) | Y <= 0.50 |
| kp 5 PD with the cap scaled to 0.94 | Y <= 0.50 |

**Return ordering (reward side):** best PD must earn a higher training return than each of stand-still (when its gains differ from best PD), limp, thrash, random, sag, pre-lean and partial drop, on 1,000 L2 training draws. Otherwise the reward is NO-GO.

**Yield PD check (feasibility of the yield gate and the reward), at T14 before section B is frozen:** a scripted "yield PD" uses the L2 best PD, with the lifted joints at kp 0.2 while the privileged overload flag is on, then back to the best gains. It must:
- reach Y >= 0.80 on the L2 yield decision scenarios;
- earn a higher training return than best PD on 1,000 L2 yield training draws.

If the return check fails, the yield-load weight is re-set ONCE before the pilot. This is calibration and does not count as one of the 3 knob iterations. It is logged, and the reviewer checks it. After the re-set, re-run the yield PD return check AND the full return-ordering test (yield draws are 15% of the 1,000 draws there). If either fails, it is a design change via the expert, not a second re-set.

If Y < 0.80, the yield gate is not reachable even by a privileged scripted yielder. The press recipe goes back to the expert BEFORE section B is written, and the threshold is not changed.

**Reward self-tests (on 1,000 L2 training draws):**
- Best PD triggers the current-reference cost in < 1% of ticks.
- Best PD's saturation cost is exactly 0 on non-yield draws.

## Reward, seeds, budget

**Reward (pre-registered, with its starting weights; the reviewer attacks it before the pilot; the weights are the knobs of the 3 iterations).** Every term is <= 0. Per tick, with e_j the wrapped joint error:

| term | per-tick cost | weight |
|---|---|---|
| tracking | mean_j (max(abs(e_j) - 0.02, 0) / 0.05)^2; every tick, settle included | 1.0 |
| bus mean | ((I_2s - 4 A) / 1 A)^2 above 4 A | 0.5 |
| bus peak | ((I_tick - 10 A) / 1 A)^2 above 10 A | 0.5 |
| T_coil | ((T - knee) / 10 degC)^2 above the knee | 0.2 |
| action rate | mean (a_t - a_{t-1})^2 | 0.05 |
| current reference | ((I_2s - thr) / 0.5 A)^2 above thr | 0.3 |
| yield load | the same load function as Y, on OVERLOAD ticks only | 1.0 |

The current-reference threshold, per (cell, family), is thr = 1.05 x (the 99.5th percentile, over >= 200 L2 training draws, of best PD's 2 s running mean) + 0.05 A. It is cached at T14 and is not applied in yield scenarios.

The yield load term uses a privileged overload flag and is zero on every other tick, so static holding near the cap is not charged.

There is no positive term for low activity and no cost on kp itself. At most 3 knob iterations, then a design change, then a method review.

**Seeds and budget:** training seeds 0, 1 and 2; scenario seeds are fixed by the frozen suites. Pilot <= 10 min. Each full run <= 1 h. One job at a time.

**Test-suite policy:**
- Every opening is logged in METHOD_REVIEWS.md.
- The test numbers of a failed full run may not be cited when choosing the next change.
- After 3 openings, a fresh test suite is generated from this recipe with new seeds (owner approval, as for any new frozen suite).

## Authority

- The main trainer MAY:
  - run T8, T14, the hack tests and the 2-min overfit;
  - run the pilot via `launch_run.py`, after the reviewer's report;
  - render captioned videos (the best and worst scenario per run);
  - pick PPO settings within the plan's defaults.
- The trainer MAY NOT:
  - launch a full run without the expert's GO;
  - change the grid, metrics, thresholds, ladder, recipe or suites once T6 freezes them. The single exception is the one allowed write of yield section B at T14 (press magnitudes and loaded joints only);
  - add observations the real robot lacks;
  - tune using test-suite numbers.

## Rejected or modified findings

- F4, "tracking reward counts only after onset": REJECTED. It makes the settle free and recreates F3 (pre-leaning). Used instead: a cost-only reward that includes the settle, plus the return-ordering test and the test-suite stand-still G test.
- F6, "set the floor friction to the range minimum": REJECTED. The default floor friction is an owner-approved physical parameter. The NOT REALISED marking is used instead.
- F1: the envelope comparator was dropped entirely in favour of the reviewer's first option (best-E PD plus a current constraint). The tolerance-band envelope was not kept.
- F18: a fresh test suite after 3 openings is ACCEPTED, but generating it needs the owner's approval.
- Re-check N5 / R3: G_dist is now the disturbance increment, with a trimmed-PD hack test and a worst-seed floor. G over the scripted PI stays reported only: a learned integrator is legitimate feedback, and G_dist now checks the disturbance rejection.
- Re-check N6(c) / R2: the current-reference cost is kept, now per (cell, family) from best PD's 99.5th-percentile 2 s current on L2 training draws, rather than the max over decision scenarios. This avoids building a training quantity from the decision suite. The cached table ties the reward to T14, so T14 must finish before the pilot.
- R1: I chose the overload-only saturation cost over a deadband above best PD's hold load (simpler, and no per-joint table). The head-lift source cannot be replaced, because no stored entry is below the cap, so it is scaled by alpha instead.
- R4, R5: accepted as proposed.
- Rev. 3 re-check B1, B2 and item 3: accepted. The holdability check uses the nominal PD, with a drop fallback and a two-phase freeze. Best PD is selected per level, and the L2 selection is the reference. The yield PD check is added. Its re-weight is once only, outside the knob count. A failure of Y sends the press recipe back, not the threshold.
- Re-check N1-N4, N7, N8: accepted as proposed.
