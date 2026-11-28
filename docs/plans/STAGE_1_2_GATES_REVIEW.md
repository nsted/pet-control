# STAGE_1_2_GATES_REVIEW: adversarial review of the S1 / S2 gate files (2026-11-28)

Reviewer: Opus reviewer (fresh context). Inputs read: `docs/plans/STAGE_1_GATES.md`, `docs/plans/STAGE_2_GATES.md`, the 2026-11-28 entry in `docs/plans/METHOD_REVIEWS.md`, `docs/plans/RL_RESTART_PLAN.md`, `docs/PET_PURPOSE.md`, `scripts/rl/pose_hold_baseline.py`. I also spot-checked `petctl/config.py`, `petctl/rl/env.py`, `petctl/rl/config.py` and `petctl/sim/model_builder.py`. No pilot exists yet, so there is no contact sheet or `reward_terms.json` to check: this review covers the gate design only. I did not edit the gate files.

**What S2 is meant to teach (one sentence):** hold a commanded pose under hidden disturbances from any orientation, using proprioceptive feedback to adapt targets and stiffness, at no more current than the best fixed PD, and give way (and then come back) when overloaded.

## Verdicts

| file | verdict |
|---|---|
| STAGE_1_GATES.md | **FIX FIRST**. Items 1.2, 1.8 and 1.10 can pass while the thing they guard is broken (findings 6, 7, 9). |
| STAGE_2_GATES.md | **FIX FIRST**. The current-matched G can be gamed, the yield score can be gamed, the windows and aggregation are not fully defined, and L0 cannot meet the limp gate (findings 1-5, 8, 10-14). |
| METHOD_REVIEWS.md entry | Acceptable as a log. Add a note on how the plan's stand-still rule is reinterpreted for S2 (finding 4) and record the 20 Hz / CLAUDE.md conflict as open (finding 15). |

## Findings, ranked by severity

### Blockers

**1. The current-matched G picks its own comparator, and the policy can game the choice.** `E_PD*(I_RL)` is the minimum E over grid configs with mean current <= I_RL. "If none" falls back to the lowest-current config. The problem is physics: holding a static load needs a torque that the load sets, not the gains. So every PD config that actually holds the pose draws almost the same current (proxy ~ tau^2). The configs that draw less current are the ones that sag or collapse onto the floor (soft kp 0.2, lost), and those have large E. A policy that cuts current a little gets compared against a collapsed PD and earns a large G without holding any better. It can do this by bracing a segment on the floor, sagging slightly where the tracking kernel barely charges for it, or relaxing during the 4 s settle (I is averaged over the whole episode; E is not, see finding 3). The "if none" branch makes this worse: the lowest-current config is almost always the floppiest one. Expected signature: I_RL a few percent below the stiff PD configs, the comparator is kp 0.2-0.4, and G is close to 1 on lattice tall / head lift.
*Fix:* make current a constraint on the RL policy, not a way to choose the PD it is compared with. Use `G = 1 - E_RL / E_PD_best`, where E_PD_best is the best stable grid config by E (per pose, see finding 11), AND require `I_RL <= I(E_PD_best config) x 1.05 + 0.05 A` as a separate pass condition. If you keep the envelope, use a tolerance band (`I_c <= I_RL x 1.05 + 0.05 A`), and in the "none" case fall back to the best-E config, not the lowest-current one. *Test:* a scripted "PD + 0.05 rad sag toward the floor" policy must get G <= 0.05.

**2. The yield score Y can be maxed without yielding.** `sat_frac` counts joint-ticks at >= 0.95 x cap. A policy (or a PD with forcerange x 0.94) that pushes back at 0.90-0.94 x cap for the whole overload gets sat_frac = 0. It barely moves, so E_rec is about 0 too. That gives Y = 1.0 for "do not give way at all". It also clears "Y >= best-PD Y + 0.15", because a stiff PD on the 0.95 threshold scores worse. Expected signature: Y close to 1 with the mean |tau|/cap of loaded joints at about 0.9.
*Fix:* replace sat_frac with an effort measure that has no cliff, for example `load = mean over overload joint-ticks of clip((|tau|/cap - 0.5)/0.5, 0, 1)`, or require a minimum deflection along the press direction. Add a hack test: PD at kp 5 with the cap scaled to 0.94 must give Y <= 0.50, and the existing max-kp test should also run at kp 3.2 (the stiffest the RL action can reach, see finding 12).

**3. E ignores the 4 s settle, so pre-leaning is free. The push direction and onset time are fixed.** E is averaged only from disturbance onset to release + 3 s. In `pose_hold_baseline.py` every push is +y world and onset is always at the same time. If the suite copies this, a policy can hold a pose pre-biased against the push it always meets. It loses nothing in E (the settle is not scored), and also lowers I (finding 1). Because the disturbance is "hidden", leaning in the expected direction is the cheapest "adaptation". Expected signature: settle-phase error well above PD's, and a large G only on push / load scenarios.
*Fix:* (a) score E over the whole episode after entry, or add a gate that settle-phase E <= best-PD settle-phase E + 0.01 rad; (b) in the frozen suite, draw the push direction (both signs, 3 axes in the module frame) and the onset time (for example 3-7 s) per scenario; (c) a test that the observation has no time or phase feature.

**4. The plan's stand-still hack test means nothing as written for a hold task.** The plan says stand-still must score <= 0.05 "on every task". The gate turns this into "stand-still G <= 0.05". But stand-still is one of the grid configs, so on the decision suite G <= 0 by construction: the test checks the evaluator's arithmetic, not the reward. Meanwhile the bounded tracking kernel pays stand-still (and limp, on the straight pose lying on the floor) almost full reward in every undisturbed tick. That is a positive term with no task progress.
*Fix:* keep the G test, but add a reward-side hack test. The normalised training return of stand-still and of limp must each be <= X of a pre-registered reference: either the oracle-gain PD return, or, simpler, "tracking reward counts only after onset" (or is weighted by the disturbance window). Record in METHOD_REVIEWS that the plan's stand-still rule is applied as G plus this return test for S2. Also run the stand-still G test on the TEST suite, where it does mean something: selection was done on the decision suite, so stand-still can beat the selected envelope there.

**5. L0 cannot meet the suite-validity gate, and the H_L0 ratio has no defined window.** At L0 the push, shove, tilt and head mass are all "none". So the hold families have no disturbance, E's window "from disturbance onset" is undefined, and limp on the straight pose lying upright has E about 0 (H about 1). "Limp H <= 0.10 on the hold families at every level" then fails at L0 by construction. This pushes the trainer to "fix the suite" in some unregistered way. The baseline script's "none" disturbance has the same issue at every level.
*Fix:* separate the DR level (friction, mass, delay, noise) from the disturbance magnitude. Define L0 as nominal DR with the nominal family magnitudes (for example the L1 maxima). Define the E window for "none". Apply the limp gate per (pose, orientation) cell, drop or re-weight cells where limp E < 0.1 rad (they cannot tell controllers apart), and list those cells in the suite manifest.

**6. Item 1.8 (ladder realised) can pass while the friction DR does nothing.** Module geoms and the floor both default to friction 1.0 (`model_builder.py` floor_friction = 1.0, the same value used for module geoms). MuJoCo uses the larger of the two, so module friction 0.4-0.99 is a no-op until T12 lands. A read-back of `geom_friction` passes anyway. This is the same bug the plan already names for floor friction.
*Fix:* 1.8 must read back the effective `d.contact[i].friction` during a rollout. Until T12, the ladder should list friction x < 1 as "not realised" (or set the floor friction to the minimum of the range, so the max rule picks the module value), and the gate file must say so.

**7. Item 1.10 (golden test) only checks against itself, and the read-back list is incomplete.** The "stored per-scenario numbers" come from the same evaluator, so the test catches drift, not a wrong evaluator. The read-back list leaves out parameters that change E: start T_coil (L2), zeta, delay ticks, noise multiplier, friction (effective, finding 6), push direction, onset time, bus voltage, and the restoration of the nominal parameters after each scenario (the baseline mutates `opt.gravity`, `actuator_forcerange` and `body_mass` in place).
*Fix:* add one analytic check that does not depend on the evaluator: a straight pose with a static push of F on the head at known kp, steady-state joint deflections against the statics solution within 5%. Extend the read-back to the parameters above. Add "scenario k+1 starts with the nominal parameters" (read back before applying).

**8. T8 may run before the S2 evaluator exists, so the baseline and RL would be measured in different plants.** S1 lets the T8 baseline run in parallel "on the existing PD path". `pose_hold_baseline.py` uses `build_sim` directly. It has no env low-pass or slew, no reply delay or noise, no ladder DR, no bus model, no thermal derate, unwrapped `abs(q - q*)`, kp scales {1, 2, 4} only, zeta 1 only, cap x 0.7, tilt about one axis, +y pushes only, and head mass x3 (L3) mixed into the L2 list. An E_PD* taken from this script and set against RL measured in PetEnv is not a valid ratio.
*Fix:* T8 run now counts only as a headroom preview. The gate E_PD* must be re-measured with zero-delta actions through the frozen PetEnv evaluator, after 1.10 passes, on the 18-config grid, wrapped error, L2/L3 suites. Say this in STAGE_2_GATES "Baselines".

### High

**9. Test 1.2 (velocity) cannot pass as written.** Obs velocity is delayed by 1-2 ticks, as specified. On a random-target rollout, the RMS gap between delayed and true qvel is far above 0.05 rad/s. That invites someone to loosen the threshold, which 1.x rules forbid.
*Fix:* compare against true qvel delayed by the same tick count, with delay and drops set to 0 for the RMS part, and test the delay separately.

**10. G aggregation is not defined.** The file does not say whether G is a mean of per-scenario ratios or a ratio of means, whether I_RL is per scenario or per pose, or what "mean G" across seeds means. Per-scenario ratios blow up when E_PD is tiny (the "none" or easy cells give G of -2 or about +0.9 from millirad changes). They are bounded above by 1 and unbounded below, so the bootstrap CI is skewed.
*Fix:* per seed, `G = 1 - sum_s E_RL(s) / sum_s E_PD*(s)` over the scenarios of a pose, then the pose-weighted mean. Floor E_PD at the quantisation-plus-noise level (about 0.005 rad). I_RL per pose (the same aggregation as the envelope).

**11. The envelope is per pose, but orientation is known at t = 0.** Orientation is visible instantly from the IMU and fixed per episode. A policy that only picks a fixed kp per (pose, orientation) is a gain schedule, not feedback adaptation, yet it earns G > 0 over a per-pose envelope.
*Fix:* compute the envelope per (pose, orientation). Report a scripted PI (target-offset integrator) baseline as a non-gate reference, so a G that comes only from learning integral action is labelled as such.

**12. The gain range and the hack test do not match.** RL stiffness is 0.8 x [0.25, 4] = [0.2, 3.2] N.m/rad, while the PD grid and the max-kp hack use 5.0. Keeping 5.0 in the comparator is conservative, which is fine. But "max kp scores poorly on yield" is never tested at the stiffest policy RL can actually reach. The RL zeta distribution (training and eval) is also not specified: "zeta at grid ends" appears only in the stability test.
*Fix:* add the kp 3.2 max-kp hack test, and state the eval zeta (fixed at 1 per scenario) and the training zeta range in the gate file.

**13. The CI and seed statistics are weak.** Resampling (seed, scenario) pairs treats 3 seeds as exchangeable with hundreds of scenarios, so the lower bound mostly measures scenario spread and hides seed variance. The pilot CI uses seed 0 only.
*Fix:* use a two-level (cluster) bootstrap: resample seeds, then scenarios within each seed. Keep worst seed >= 0.20 (good). Call the pilot interval a "scenario CI". Pre-register the scenario count per family, pose, orientation and level for `s2_hold` (not specified now), and the training family mix that "double the yield share" refers to.

**14. The teacher headroom rule uses an underpowered teacher.** "Teacher G < 0.30" is judged after a <= 10 min teacher pilot, while the student must reach 0.30 after 3 x 1 h. A short teacher run can falsely trigger the task change. That change happens once and is irreversible in effect.
*Fix:* judge headroom with a full-length teacher run (1 seed, <= 1 h), or compare the teacher pilot against the pilot bar (G >= 0.10 at 1.5M steps). Also use the oracle-gain PD as a second headroom signal.

### Medium

**15. The 20 Hz decision conflicts with CLAUDE.md, and its ablation is never scheduled.** CLAUDE.md says "Control loop: 30Hz async", and `EnvConfig` says control_hz "must match the real brain". The rationale is acceptable for sim (horizon, existing calibration, delay margin), and point (b) is arithmetically right only per fixed step count. But until the owner decides, every S2 result is non-deployable, and the "would change my mind" ablation (30 Hz on the shove family) has no task.
*Fix:* add a TASK_QUEUE item for the 30 Hz shove ablation, plus a test that the deployed brain's policy rate equals `EnvConfig.control_hz`. Mark this in CURRICULUM_STATE as an owner item that blocks deployment (not S2).

**16. The episode timeline is inconsistent.** A 15 s episode holds the 4 s settle, 3 s of disturbance and 3 s of recovery, but the baseline's staged lattice entry takes 1 s per joint (up to 10 s) plus a 4 s settle. The gate does not say who enters the pose (a scripted ramp or the policy), or whether the episode starts in the settled pose. The shove window (0.1 s + 3 s) differs from the others and is not stated.
*Fix:* reset into the settled pose (cached state per pose and orientation), state the shove window, and make the lost rule (> 0.6 rad for > 1 s, counted over the whole episode as the script does) explicit.

**17. The bus-proxy and budget definitions conflict with the code.** The gate gives `I ~ 0.02 + 2.4 tau^2`. The env with `bus_motor=1` adds IDLE_ELECTRONICS_A, a speed-blended move coefficient of 4.6, and voltage scaling. The budget "mean over the episode <= 4 A" differs from config's 2 s running mean. If the sim clamp is on, the applied current never exceeds 10 A, so the peak budget must be checked on demanded, pre-clamp current (as config.py says). With the move coefficient, ordering is no longer "monotone in tau^2": a jittery policy and a static PD can swap order if the true move/hold ratio differs.
*Fix:* name the exact code path (BusClamp, demanded current, 2 s window for mean, 50 ms tick mean for peak, fixed voltage). Re-check G with move coefficient in {2.4, 4.6, 7.0} and require a pass in all three.

**18. Test-suite reuse lets results leak back into tuning.** "Opened once per full run", but up to 3 knob iterations plus a design change can each make a full run, so the test suite is seen several times.
*Fix:* log every opening. After a failed full run, its test numbers may not be cited in the next knob change. After 3 openings, generate a fresh frozen test suite (new seeds, same pre-registered recipe). Selecting the PD envelope on the decision suite is not leakage in RL's favour (it favours PD on the decision suite). It is fine as long as the test denominator is re-measured on the test suite with the decision-selected configs (say so: "applied unchanged" could be read as reusing decision-suite E values).

**19. Thrash and random tests are underspecified, and there is no chatter metric.** "Max magnitude" is undefined for unbounded targets. Whether a delta is a residual on q* or an increment on the previous target changes whether uniform random (after the low-pass) looks like stand-still (then H > 0.10 and a false NO-GO) or a random walk. Nothing stops a deterministic policy from chattering at 20 Hz while holding, which fails PET_PURPOSE guide 4 (alive to the touch, not buzzing).
*Fix:* define the delta semantics and the thrash bound (for example the slew limit per tick, alternating sign). Add a reported, non-gating hold-phase joint-velocity RMS and action-rate ratio against PD, with a soft flag at > 2x.

**20. The 2-min overfit test does not exercise the target-delta path.** Beating nominal PD (kp 0.8) by 20% on "push head 5 N" can be done by raising the kp scale alone.
*Fix:* add the requirement "E at least 10% below the best grid PD for that scenario", or run the overfit with the kp channel frozen.

**21. Multi-turn and encoder range.** Wrapped error is the right choice, given no angle limits. But the gate does not say whether q* and the position obs are multi-turn or wrapped, or what happens at the MIT position-field range (it wraps). A policy winding a joint by 2 pi during an overload is invisible in E and Y.
*Fix:* state the obs convention, and add a test at |q| beyond the position-field range.

**22. Undefined poses.** "One wrap-radius pose" has no source (it is not in the baseline script), and "head lift upright only" should also apply to the lattice poses that are not stable in some orientations, or list per pose which orientations are valid.
*Fix:* name the files and indices in the suite manifest.

## Minor

- METHOD_REVIEWS entry: "existing calibration is 20 Hz" is fine. Add "plan's stand-still <= 0.05 rule reinterpreted for S2 (see review finding 4)".
- 1.13: decide in advance what pilot step budget the 10-min scaling implies (steps, not minutes), so the pilot gate's "1.5M steps" stays well-defined.
- The bus comment in config.py says the clamp acts "per 2 ms substep", but sim_dt is 0.004. Check which one the BusClamp actually uses.

## What I would accept as GO

STAGE_1: GO once 1.2, 1.8 and 1.10 have been rewritten as in findings 9, 6 and 7, and T8 is labelled a preview (finding 8).
STAGE_2: GO once findings 1, 2, 3, 4, 5, 10 and 16 are fixed in the gate file and their scripted hack tests exist (sag policy, 0.94-cap PD, pre-lean policy, kp 3.2 max). Findings 11-14 and 17 should land before the full run, at the latest.

## Re-check (rev. 1), 2026-11-28

Reviewer: Opus reviewer (fresh context). Read: rev. 1 of `STAGE_1_GATES.md` and `STAGE_2_GATES.md`, `METHOD_REVIEWS.md` lines 7-14, `TASK_QUEUE.md` (T16), `petctl/rl/env.py` (termination, line 488), `petctl/rl/config.py` (`flip_up_dot`), `petctl/config.py` (GIM3505 bus constants). Gate files not edited.

### Earlier findings

| # | status | one line |
|---|---|---|
| 1 | FIXED | Best-E PD per cell plus a separate current constraint; the envelope and the lowest-current fallback are gone (new gaps in N4, N6). |
| 2 | PARTLY | Load is now graded (no 0.95 cliff), but it only counts 3 fixed joints, and the reward still has a saturation-fraction term (N2, N7). |
| 3 | FIXED | Whole-episode E, a settle check, random onset U[3,7] s and direction, and a no-time-feature test (1.5). The pre-lean hack test is now vacuous (N8). |
| 4 | FIXED | Return-ordering test, stand-still on the TEST suite, METHOD_REVIEWS note. Rejecting "count only after onset" is reasonable. |
| 5 | FIXED | DR is separate from disturbance magnitude, L0 has disturbances, cells are excluded by limp E, and exclusions go in the manifest (the cell set across levels is still open, N8). |
| 6 | FIXED | Effective `d.contact.friction` is read back, and friction below the floor value is marked NOT REALISED. Caveat: friction DR at L2/L3 is in effect 1.0-1.8 / 1.0-2.0 until T12, so the low-friction half is missing. |
| 7 | FIXED | Analytic J^T F / kp check, full read-back list, restoration check. |
| 8 | FIXED | T8 is only a preview, and T14 re-measures through the frozen evaluator. |
| 9 | FIXED | RMS is measured with delay and drops off, and the delay is tested on its own. |
| 10 | FIXED | Ratio of sums per cell, 0.005 floor, equal-weight cells. |
| 11 | FIXED | Comparator per (pose, orientation) cell, plus a PI reference. |
| 12 | FIXED | kp 3.2 max-kp test; eval zeta 1; training zeta range stated. |
| 13 | FIXED | Two-level bootstrap, counts per cell, training mix. (With 3 seeds the seed level is coarse, but the worst-seed gate covers that.) |
| 14 | FIXED | Headroom is judged on the full teacher run, with the oracle as a second signal. |
| 15 | FIXED | Open owner item, T16 queued, 1.17 non-blocking. |
| 16 | FIXED | Settled-pose start, shove window, lost rule over the whole episode. |
| 17 | FIXED | Exact code path, demanded current, 2 s and 50 ms windows, move coefficients {2.4, 4.6, 7.0}. Whether best PD is re-selected per coefficient is not said (N6). |
| 18 | FIXED | Openings logged, no citing failed-run test numbers, fresh suite after 3 openings. |
| 19 | FIXED | Delta semantics, thrash and random defined, chatter reported. |
| 20 | FIXED | kp frozen in the overfit test (but it can be frozen out of range, N8). |
| 21 | FIXED | Wrapped angle plus turn count, multi-turn targets, 1.14, winding reported. |
| 22 | FIXED | Pose sources named, valid orientations decided at suite generation. |

### New holes in rev. 1, ranked (max 8)

**N1 (blocker). The cost-only reward plus any termination pays the policy to end the episode early.** Every reward term is <= 0. Any early termination therefore cuts the remaining negative return to zero. `env.py:488` terminates on `terminate_on_flip` (up . world-up < 0.2) or `task.terminated`. The S2 "inverted" (and probably "left" / "right") cells would end at t = 0 if the S2 task keeps flip termination. If "lost" (> 0.6 rad for > 1 s) is ever used as a termination, collapsing becomes the best move in any episode the policy expects to lose. The expected signature is short episodes, a high lost fraction, and per-term sums that shrink as the episode gets shorter. *Fix:* the gate must state "S2: no termination, truncation at 15 s only; lost is a metric, not a terminal". Add an S1/S2 test: on every valid cell, including inverted, the S2 preset runs 300 steps under limp and thrash with `terminated == False`.

**N2 (high). The yield load term counts only the 3 joints that were most loaded under best PD, so the policy can move the load elsewhere.** It can stiffen the neighbouring joints so the 3 listed joints relax (load about 0), while other joints sit at the cap and nothing gives way. *Fix:* compute load per tick over all 10 joints, as the mean of the top-3 values of clip((|tau|/cap_eff - 0.5)/0.4, 0, 1) at that tick, where cap_eff is the derated cap. Hack test: a scripted "redistribute" PD (listed joints at kp 0.2, the others at 5.0) must get Y <= 0.50.

**N3 (high). Best-PD selection and the yield press calibration depend on each other.** Best PD is "lowest sum of E over the cell's decision scenarios", and that set includes the yield scenarios. But the yield press is defined as a multiple of the press that saturates >= 3 joints *under best PD*. *Fix:* select best PD on the non-yield scenarios only, then calibrate the press, then freeze both in the manifest.

**N4 (high). The headroom threshold was set for the old E window, and E now averages the whole episode.** E over 15 s, with 3 s of disturbance, is diluted 2-4x compared with disturbance-window E. So "best PD E <= 0.03 rad means no headroom" can fire falsely and trigger the one-time task change. *Fix:* apply the headroom rule to disturbance-window E, [onset, onset + 4 s], or re-register the threshold before T14 is seen.

**N5 (medium). G can be met by static trim alone.** With whole-episode E, most of the sum is steady-state gravity sag, which an integrator removes without rejecting any disturbance. The PI reference only labels this. *Fix:* report G in three windows (settle, disturbance, recovery). Also gate G_dist >= 0.15 on the disturbance window, or require G over the scripted PI >= 0.10. If neither is adopted, the stage summary must call the result "integral trim", not adaptation.

**N6 (medium). The current constraint is not fully defined, and nothing in training pushes toward it.** (a) Say which scenarios I_RL and I_PD average over: the cell's non-yield scenarios, with yield checked separately, so that yielding cannot pay for extra current elsewhere. (b) In the move-coefficient check, keep the best-PD selection fixed (selected at 4.6) and re-measure only the currents. (c) The reward costs current only above 4 A, while static holds draw about 1 A (0.2 A idle + 2.4 tau^2 x 10). The gate condition gets no training signal, so a stiffened, chattering policy will likely fail it, and the result will look like a knob problem. Either add a deadband cost on demanded current above a cached per-scenario best-PD current, or record this as an accepted risk before the pilot.

**N7 (medium). The reward and the gate disagree on saturation.** The reward's "torque-saturation fraction" keeps the 0.95 cliff that finding 2 removed from the gate, so training is pushed toward holding at 0.94 cap, which the gate then fails. *Fix:* use the same graded load function in the reward as in Y.

**N8 (low, bundle).**
- (a) The 2-min overfit freezes kp at "that scenario's best grid gain", which may be 5.0 and outside the RL range. Use the best grid gain <= 3.2.
- (b) Suite validity and its exclusions may differ by level, which makes H_L3 / H_L0 compare different cell sets. Decide the cell set once (at L2) and apply it to L0 and L3.
- (c) The pre-lean hack test (+y world) is vacuous now that push directions are random. Point it at the one fixed direction left, load = downward, and compare it with the PI reference.

### Verdicts (rev. 1)

| file | verdict |
|---|---|
| STAGE_1_GATES.md | **GO.** All earlier S1 findings are fixed. Recommended, not blocking: add the N1 no-termination test to 1.1's S2-preset check. |
| STAGE_2_GATES.md | **FIX FIRST.** N1 (state no termination, plus the test), N2 (load over all joints, plus the redistribute hack test), N3 (select on non-yield) and N4 (headroom window) must be in the gate file before T14. N5-N7 must be decided before the pilot. N8 can be fixed in passing. |

## Re-check (rev. 2), 2026-11-28

Reviewer: Opus reviewer (fresh context). Read: rev. 2 of `STAGE_2_GATES.md`, `STAGE_1_GATES.md` item 1.1, the rev. 2 lines of `METHOD_REVIEWS.md`, `petctl/rl/env.py` lines 488-508 (terminated / truncated), `petctl/rl/train.py` (SB3 PPO, which bootstraps on `TimeLimit.truncated`), and `data/rl/statics/results.json` (n = 11 head-lift entries). Gate files not edited.

### N1-N8

| # | status | one line |
|---|---|---|
| N1 | FIXED | "No termination, truncation at 15 s only", with the test in S2 line 12 and in S1 1.1. SB3 bootstraps truncation, so the missing time feature only adds value noise. |
| N2 | FIXED | Top-3 over all 10 joints per tick against cap_eff, plus the redistribute test. (In quasi-static holds the joint torques come from the geometry, not from kp, so the redistribute test is close to vacuous. The new top-3 hole is in R4.) |
| N3 | FIXED | Best PD is selected on non-yield scenarios, then the press is calibrated, then both are frozen. |
| N4 | FIXED | The headroom rule uses E_dist. |
| N5 | PARTLY | G_dist >= 0.15 is now a gate, but E_dist still contains best PD's steady sag, so static trim alone earns G_dist (R3). |
| N6 | PARTLY | (a) and (b) are fixed. (c) adds a cost, but its reference is on the wrong scale and penalises correct holding (R2). |
| N7 | FIXED as written, but it opened a hole | The reward uses the same graded function as Y. It is applied on EVERY tick of every scenario, so it now costs static holding (R1). My N7 fix was underspecified. |
| N8 | FIXED | (a) best gain <= 3.2, (b) one cell set chosen at L2, (c) pre-lean aimed at the downward load. |

### New holes in rev. 2

- **Saturation cost on every tick.** `results.json` head_lift n = 11 seed 0 (the S2 pose source) settles with tau_peak = 0.50 N.m, which is the cap, at 2.32 A. So f = 1 on at least one joint while the pose is held correctly. That means:
  - best PD pays the saturation cost on every tick of every head-lift episode;
  - the cheapest way to cut that cost is to drop the head a little;
  - none of the return-ordering policies is a "partial drop", so nothing catches this;
  - the term weights are not pre-registered, so it is not known whether tracking outweighs the saving.

  The same pose cannot be held under L2 mass x1.2, head mass x2 or derate 0.8, because it already sits at the cap at nominal.
- **The per-cell current cost has the wrong reference.** The threshold is the cell mean over all non-yield scenarios, compared against a 2 s running mean inside each episode. Push, load, tilt and head-mass scenarios (and L2 draws with mass x1.2, where current scales with tau^2, about 1.44x) legitimately exceed the cell mean. So the cost fires on correct holding during the disturbance window and teaches the policy to sag exactly when it is pushed. The table also needs an entry for every cell that training samples, and the gate does not say that training samples only valid, non-excluded cells.
- **The G_dist gate does not separate trim from rejection.** A policy that only cancels the settle sag lowers E_dist by about the same amount as E_settle. The claim "it is what static trim cannot earn" (line 61), and the rejected-findings note at line 148, do not hold.
- **Top-3 mean over 10 joints.** One joint pinned at the cap for the whole overload, with two at 0.5 cap, gives load = 1/3 and Y = 0.83 if recovery is clean. That passes Y >= 0.80 while the robot does not give way at the joint that carries the press. It is also not said whether tau is the true actuator force or the quantised, noisy observed torque.
- **No termination.** No exploit remains. 1.1 depends on the validity list from 1.9/T6 and should not.

### Verdicts (rev. 2)

| file | verdict |
|---|---|
| STAGE_1_GATES.md | **GO.** The 1.1 change is correct. The test hardening in R5 is recommended, not blocking. |
| STAGE_2_GATES.md | **FIX FIRST.** R1-R3 must be in the gate file before T14 and the pilot. R4 before the pilot. R5 in passing. |

### Remaining items (max 5)

**R1 (blocker). Saturation cost.**
- Apply the graded load cost only on overload ticks (yield scenarios, privileged flag). Alternatively, give it a deadband above best PD's cached per-cell, per-joint hold load.
- Pre-register the term weights.
- Add a "partial drop" scripted policy to the return-ordering test: best PD with the lifted joints' targets moved 0.3 rad toward the floor. Best PD must out-earn it.
- Replace the head-lift source with an entry whose settled tau_peak is <= 0.8 x cap (check the lattice poses the same way), or lower the head-lift DR so the cell stays holdable at L2.
- Test: best PD's per-episode saturation cost is 0 on non-yield L2 training draws.

**R2 (blocker). Current reference.**
- Build the table per (cell, family) at L2. Use as threshold the MAX over that family's decision scenarios of best PD's peak 2 s mean, x1.05 + 0.05 A. Or move the term to an episode-mean comparison applied once at truncation.
- State that training samples only valid, non-excluded cells.
- Test: on 1,000 L2 training draws, best PD triggers this term in < 1% of ticks.

**R3 (high). G_dist.**
- Define G_dist on the disturbance increment, per scenario max(E_dist - E_settle, 0), with the same 0.005 floor. Or define it against a scripted "trimmed PD": best PD with q* offset by its own settled error per cell.
- Hack test: trimmed PD gets G_dist <= 0.05.
- Gate G_dist with worst seed >= 0.10, not only the seed mean.

**R4 (medium). Yield load.**
- Use load = 0.5 max_j f + 0.5 top-3 mean f per tick, in both Y and the reward.
- Define tau as the true applied actuator force.
- Hack test: a scripted policy that relaxes all lifted joints but one, and holds that one at >= 0.95 x cap through the overload, gets Y <= 0.70.

**R5 (low). 1.1 hardening.**
- Run the no-termination test on all 24 (pose, orientation) cells, not only the valid ones, so it does not depend on 1.9.
- Assert `truncated == True` exactly at step 300 and not before, and `info["TimeLimit.truncated"]` as SB3's VecEnv sets it, so the value bootstrap is tested.

## Re-check (rev. 3), 2026-11-28

Reviewer: Opus reviewer (fresh context). Read: rev. 3 of `STAGE_2_GATES.md`, `STAGE_1_GATES.md` item 1.1, `TASK_QUEUE.md` T6 / T14 / T15. Gate files not edited.

### R1-R5

| # | status | one line |
|---|---|---|
| R1 | FIXED | Load cost only on overload ticks (privileged flag), weights pre-registered, partial-drop policy in return ordering, saturation self-test, head lift scaled by alpha with a holdability rule for every pose. The rule's ordering is broken, see B1. |
| R2 | FIXED | Per (cell, family) table, 1.05 x p99.5 + 0.05 A from L2 training draws, not applied in yield; training samples only valid cells; < 1% self-test. |
| R3 | FIXED | G_dist on max(E_dist - E_settle, 0), trimmed-PD test G_dist <= 0.05, worst seed >= 0.10. |
| R4 | FIXED | load = 0.5 max + 0.5 top-3 in Y and in the reward, tau = `d.actuator_force`. One joint pinned gives load >= 0.667, so Y <= 0.667 and the <= 0.70 test can pass. |
| R5 | FIXED | 1.1 covers all 24 cells (6 poses x 4 orientations) independent of 1.9, truncation exactly at step 300, `info["TimeLimit.truncated"]`. |

### Blockers that make T14 or the pilot invalid

- **B1. Best PD is used before it exists.** T6 freezes the suites. T14 runs on them and is the first step that selects best PD. Authority line 156 forbids changing suites after T6. Two things in the suite depend on best PD:
  - the holdability rule and alpha ("best PD, settled at the L2 worst corner", line 16);
  - the yield press ("calibrated" on best PD, then "frozen in the manifest", line 33).

  So T6 cannot be executed as written. The trainer either guesses a placeholder, which makes T14's yield numbers invalid, or rewrites a frozen suite after T14, which is forbidden and post hoc. The rule also has no fallback when no alpha in {1.0..0.5}, or no lattice replacement, passes. Without a fallback, the yield family can end up with zero cells.
- **B2. Which level best PD is selected at is not pre-registered.** Line 60 says "selected on the decision suite" but names no level. T14 runs at L0, L2 and L3. That level sets the denominators of the pilot G (L2) and of the full-run G_L3, and the per-cell current table. Left open, it is a choice made after seeing T14's numbers.

### Verdicts (rev. 3)

| file | verdict |
|---|---|
| STAGE_1_GATES.md (1.1) | **GO.** R5 fixed. |
| STAGE_2_GATES.md | **FIX FIRST.** B1 and B2 must be in the gate file before T6 and T14. R1-R5 are otherwise closed. |

### Remaining items (max 3)

1. **(B1, blocker) Ordering.**
   - Run the holdability and alpha check with the nominal PD that builds the settled cache, not best PD. In a quasi-static hold the torque comes from the geometry, so nothing is lost.
   - Fallback: if no alpha >= 0.5 passes, exclude the cell and record it. If no lifted-part cell passes, the yield family goes back to the expert.
   - Two-phase freeze. T6 freezes every non-yield scenario and the yield draws (cell, onset, body, direction, multiplier 1.2/1.6/2.0). T14 selects best PD on non-yield, calibrates the press, and writes it ONCE into a separately SHA'd yield section of the manifest. Add this one allowed write to Authority.
   - Test: `eval_suite.py check` confirms that the non-yield SHA is unchanged after T14 and that the yield section was written exactly once.
2. **(B2, blocker) Selection level.** State it. My recommendation: best PD is selected per level on that level's decision suite. The L2 selection feeds the reward's current table and the alpha / press calibration.
3. **(Before the full run, not the pilot) Is the yield gate reachable, and does the reward pay for it?** No scripted policy shows that Y >= 0.80 can be reached, or that the reward prefers yielding. Tracking (weight 1, about ((e - 0.02) / 0.05)^2 per joint) can outweigh the load cost (<= 1 per tick) wherever a stiff hold has an equilibrium short of the floor. Add a scripted "yield PD" to the return-ordering test and to the hack table: best PD whose lifted joints drop to kp 0.2 while the overload flag is on and are restored after it. It must reach Y >= 0.80 at L2 and must out-earn best PD on yield training draws. If it fails, re-weight before the pilot (the knobs are the weights).

## Re-check (rev. 4), 2026-11-28

Reviewer: Opus reviewer (fresh context). Read: rev. 4 of `STAGE_2_GATES.md`, `STAGE_1_GATES.md` item 1.1, the rev. 3 re-check above. Gate files not edited.

### B1, B2, item 3

| # | status | one line |
|---|---|---|
| B1 | PARTLY | The holdability check and alpha now use the nominal PD, there is a drop fallback (fewer than 4 of 6 poses means NO-GO), and there is a two-phase freeze with a write-once test. But set A (line 75) freezes only the NON-yield suites. The yield draws (cell, onset, body, direction, multiplier) are in neither set A nor set B, so they can still be drawn at T14 after best PD is known. |
| B2 | FIXED | Best PD is selected per level on that level's non-yield decision suite (line 61), and the L2 selection is the reference for the current table, the press, the loaded joints and the hack policies (line 62). |
| 3 | FIXED, one gap | The yield PD check is added (Y >= 0.80 and it must out-earn best PD on 1,000 L2 yield draws), with a once-only re-weight and the press sent back to the expert if Y fails. Gap: nothing requires the checks to be re-run after the re-weight, and nothing says what happens if they still fail. |

### Blockers that make T14 or the pilot invalid

- **K1 (T6/T14). Yield draws are not frozen.** Add to set A, line 75: "and the yield draws (cell, onset, body, direction, multiplier 1.2/1.6/2.0, seeds) for every level; only the press magnitude and loaded joints are written into B". Extend the line-80 test: the yield draw records hash into A, and B contains no draw fields.
- **K2 (pilot). The re-weight is not re-verified.** Add to line 130: "After the re-set, re-run the yield PD return check AND the full return-ordering test (yield draws are 15% of the 1,000 draws there). If either fails, it is a design change via the expert, not a second re-set." Without this the pilot can train on a reward that has not been shown to prefer yielding. This is likely to matter: during the overload, the tracking cost on the dropped lifted joints, about ((0.3 - 0.02) / 0.05)^2 x 3/10 per tick, can be several times the load cost (<= 1 per tick).

Not blocking:
- Authority (line 174) does not mention the one allowed write of section B at T14.
- The Two-phase freeze block sits inside the Baselines numbered list, between items 1 and 2. Move it out.
- If every lifted-part cell is orientation-invalid while the poses survive, the yield family has no cells and there is no NO-GO rule for that.

### Verdicts (rev. 4)

| file | verdict |
|---|---|
| STAGE_1_GATES.md (1.1) | **GO.** Unchanged since rev. 3, all 24 cells, truncation at step 300, `TimeLimit.truncated`. |
| STAGE_2_GATES.md | **FIX FIRST.** K1 is two lines, needed before T6. K2 is one line, needed before the pilot. After that, GO for T6, T14 and the pilot without another re-check, provided the edits match the wording above. |
