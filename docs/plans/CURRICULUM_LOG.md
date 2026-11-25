# Autonomous navigation curriculum - running log

Claude Code runs this loop on its own (train -> monitor -> frozen gate -> adjust). **Nick only reads this file and the stage outputs.** Newest entries at the bottom of "Log". Plan: `docs/plans/RL_LOCOMOTION.md` (stage list), code: `petctl/rl/curriculum/`, runs: `data/rl/curriculum_fast/` (current; the slower `curriculum_cap0p5_head/` runs are kept as the baseline).

## Operating rules (what I will and will not do on my own)
- **Will:** run sim trainings/evals, change declared stage knobs, add/modify/reorder stages and tasks, add tests, fix bugs, update this log and the plan doc, render videos into `docs/media/`.
- **Gates are frozen against rescuing a failing policy.** A gate or threshold changes only to fix a *demonstrated* mis-calibration (e.g. far too easy/hard, a metric bug), with the data in the log. Never to make a failing run pass.
- **Escalation ladder per stage:** (1) read the per-condition breakdown, (2) adjust knobs (entropy, steps, randomisation, curriculum range; max ~3 tries), (3) change the stage/task design (new intermediate stage, reward/obs change), (4) change the overall approach (e.g. primitives + selector). Every step is logged with the reason.
- **Will NOT without asking:** move the real robot, create/close/comment on GitHub issues, install anything from the internet, delete anything outside the working folder, root-level operations, or change hardware limits in `petctl/config.py`.
- Compute is local simulation only (10 cores). Real torque cap used everywhere: **0.5 N.m** (Nick, 2026-11-11).

## Definitions in force
- Task family `goal_reach` / `goal_sequence`; **success = head within 5 cm of the target**; progress shaping uses the centre of mass; goal direction/distance observed in the IMU frame (hardware will need an external localiser for the bearing).
- Early stop: a run stops as soon as its stage gate passes (checked when rolling *training* success reaches gate-0.10; training success uses the stochastic policy and understates the deterministic gate result).

## Stage status (slow gait 0.5/0.6: S1 100%, S1b 88%, S2 90% passed; the table below is the CURRENT fast-gait chain, all re-run)
| Stage | What it trains | Gate | Status |
|---|---|---|---|
| S1 goal ahead | basic directed gait | 80% | PASSED fast gait (nominal 0.90 / robust 0.97) |
| S1b random start | any initial body configuration | 75% | PASSED (0.82 / 0.82) |
| S2 any bearing | turning/reversing, goals anywhere | 70% | PASSED (0.68 / 0.72 on its gate) |
| S3 goal sequence | 4 random targets in order, next target visible | 60% | PASSED (0.70 / 0.75); S4 now training |
| S4 retarget | target jumps to a new random position mid-run | 60% | PASSED (0.72 / 0.72) |
| S5 far / robust | long legs, heavier randomisation | 60% | PASSED (0.85 / 0.65) |
| later | dynamic terrain, tactile obstacle sensing, obstacles, maze; click-to-navigate tool | - | not started |

## Open questions for Nick (no action needed from you unless you want to change them)
- The sim reward still has a 4 A bus-current penalty (`BUS_LIMIT_A`, `BUS_A_PER_NM2`); did that budget change with the 0.5 N.m cap?
- Goal bearing on the real robot needs an external localiser (IMU yaw drifts): is there one (camera/mocap/beacon)?

## Log

History up to the stage chain (2026-11-11 .. 2026-11-15 15:27) is archived in `docs/plans/archive/CURRICULUM_LOG_through_2026-11-15.md` - do NOT read it by default. Current state and next action: `docs/plans/CURRICULUM_STATE.md`. Keep new entries short (result + decision, <= 5 lines).

### Stage chain A1/A2 (2026-11-15)
A1 flat_nav attempt 1: PASS n=100 success 0.97 (Wilson lo 0.915 vs 0.80), t_goal median 12.25 s, bus mean 2.55 A, episodes over 10 A peak 0.05 (limit 0.05, no margin), glitch 0; skill models/skills/nav_chain/flat_nav. A2 flat_omni attempt 1: success 0.96 (lo 0.90), lateral median 0.67 m (97% >= 0.4), t_goal 5.1 s but FAIL on the current budget only: episodes over 10 A peak 0.06 > 0.05. Per the plan a passing policy that violates the current budget is a FAIL (not a knob) -> fix, not retune: added bus_peak_penalty 2.0 (existing env knob, hug v3 value) to the A2/B1/B2 stage params (A1 unchanged, passed); A2 attempt 2 relaunched as a continuation with the penalty (the aborted un-penalised attempt-2 start is in data/rl/contended_runs/).
A2 flat_omni attempt 2 (with bus_peak_penalty 2.0): PASS n=100 success 0.96 (Wilson lo 0.90 vs 0.70), t_goal median 5.05 s, lateral median 0.68 m (96% >= 0.4 m), current budget met; skill models/skills/nav_chain/flat_omni. B1 terrain_easy started 16:33.
B1 terrain_easy attempt 1 (17:08): success 0.99 (Wilson lo 0.946 vs 0.75) at terrain level 0.6, t_goal 4.95 s, bus mean 2.46 A, but FAIL: episodes over 10 A peak 0.06 (> 0.05) and glitch rate 0.01 (1 episode; gate requires 0). Plan rule: glitch > 0 -> stop and fix physics. Attempt 2 (identical continuation, driver default) is running; a Sonnet executor is replaying the glitch episode (docs/plans/B1_GLITCH_INVESTIGATION.md). B1 success is not the issue.
B1 terrain_easy attempt 2 (after the terrain root-lift fix, 17:42): PASS n=100 success 1.00 (Wilson lo 0.963 vs 0.75), t_goal 4.80 s, bus mean 2.32 A, episodes over 10 A peak 0.05 (limit 0.05), glitch 0; skill models/skills/nav_chain/terrain_easy. B2 terrain_full started 17:43. Video watcher fix: chain runs are now captioned/named by their stage dir (flat_nav / flat_omni / terrain_easy / terrain_full); before, progress videos of B1 were captioned 'chain_ab a2' and named 'flat_a2' (the group), which was misleading.
B2 terrain_full attempt 1 (18:27): success 0.93 (Wilson lo 0.863 vs 0.70) PASS, imu_off ablation drop 0.35 (CI 0.25-0.45 vs min 0.15) PASS, glitch 0 PASS, t_goal median 20.95 s, bus mean 2.55 A PASS, but FAIL on the PEAK current budget: 28% of episodes over 10 A demand (limit 5%; median peak 9.0 A) - rough terrain makes the policy demand current bursts. Per the plan a current-budget violation is a fix, not a retune: bus_peak_penalty raised 2.0 -> 10.0 for terrain_full (A2/B1 used 2.0 and passed at 5%); attempt 2 = continuation with the stronger penalty (driver3, 18:29).

### terrain_full soft current limiter (2026-11-23)
Owner request: attenuate current as it nears the limit instead of a hard cutoff. Added `bus_soft_knee` (sim/bus.py `soft_limit_current`, tanh roll-off from knee*peak to peak, C1, never exceeds peak; 0 = old hard clamp); set 0.7 for terrain_full only. Tests in tests/test_bus_delta.py (20 pass). Smoke: demand 9.59 A -> applied 9.09 A (hard clamp: 9.59). Chain relaunched in data/rl/chain_ab_soft (warm from the a2 policy, 2 attempts).

### Visual style (2026-11-23)
New `petctl/sim/style.py` (cosmetic only; build_mjcf byte-identical, physics verified identical): pink->violet->blue->teal robot gradient with gloss, warm sand floor, sky gradient, sun + shadows, sage/clay terrain, cream walls, violet obstacles, gold target/goal markers (mint when reached). Opt out: BuildParams.extra["plain_colors"]=1.

## 2026-11-23 A1-A2-B1-B2 chain finished (L=8)

  - flat_nav a1: success 0.970 (Wilson lo 0.915 vs 0.8), PASS, t_goal median 12.25
  - flat_omni a1: success 0.960 (Wilson lo 0.902 vs 0.7), FAIL, t_goal median 5.1000000000000005
  - flat_omni a2: success 0.960 (Wilson lo 0.902 vs 0.7), PASS, t_goal median 5.050000000000001
  - terrain_easy a1: success 0.990 (Wilson lo 0.946 vs 0.75), FAIL, t_goal median 4.95
  - terrain_easy a2: success 1.000 (Wilson lo 0.963 vs 0.75), PASS, t_goal median 4.800000000000001
  - terrain_full a1: success 0.930 (Wilson lo 0.863 vs 0.7), PASS, t_goal median 20.5
- Data: data/rl/chain_ab_soft

B2 terrain_full PASS (2026-11-23, soft current limiter knee 0.7, data/rl/chain_ab_soft/terrain_full/a1): success 0.93 (Wilson lo 0.863), imu_off drop 0.87, bus mean 2.49 A, episodes >10 A 1%, glitch 0; skill models/skills/nav_chain/terrain_full. Chain DONE.

## 2026-11-23 C1 barrier_touch set 1 (data/rl/chain_c): FAIL x2, success 0.00 (blind A1 walker 0.17), touch_off drop 0
Cause: bus_peak_penalty 10 (inherited from B2) swamped progress (-1150 vs +800 per episode); training success fell 0.17 -> 0 as the policy stopped moving (distance 1.26 -> 0.77 m).
Knob iteration 1/3: bus_peak_penalty 3.0, fresh root data/rl/chain_c2, warm from terrain_full. Also added: portrait videos, touch-blind blind baseline, barrier width/gap ranges.

## 2026-11-23 B3 terrain_rolling: FAIL x2 -> STOP (data/rl/chain_c_terrain_rolling)
- a1 (4.0M, warm terrain_full): success 0.76 (Wilson lo 0.668 vs 0.70), imu_off drop 0.63; a2 (2.5M, warm a1): 0.68 (lo 0.583), drop 0.57. Bus peak med ~9.6-10 A, 12-13% eps >10 A (reported only), glitch 0.
- Decision: stop per rule; no further runs pending owner direction (knob iteration 1/3 unused).

## 2026-11-23 B3 knob 1/3 (train at level 1.0, root chain_c_terrain_rolling_k1): FAIL x2
- a1 0.73 (lo 0.636), a2 0.75 (lo 0.657); imu_off drop 0.73-0.75. All gate failures are 40 s timeouts (no flips); 18 seeds fail in all 4 runs so far (stuck, terrain-specific).
- Knob 2/3: ent_coef 0.005 -> 0.01 for terrain_rolling, warm k1/a2, root chain_c_terrain_rolling_k2.

## 2026-11-23 B3 knob 2/3 (ent_coef 0.01, root _k2): FAIL x2 (0.68 lo 0.583; 0.63 lo 0.532). Knob 3/3 launched (ent_coef 0.003, warm k1/a2, root _k3).
- Diagnosis: all gate failures are 40 s timeouts; 16 seeds fail in all 6 gate runs. On those seeds the terrain rises a mean 7.3 cm along the start-goal line (others 3.0 cm): goals behind ridges. Success plateau 0.63-0.76 across 6 runs. If k3 fails: design change needed (terrain amplitude / goal sampling / policy memory), owner escalation.

## 2026-11-24 B3 design change d1 (owner-approved): knobs 1-3 failed (best 0.75, lo 0.657); persistent failures = goals behind ~7 cm ridges.
- Rolling terrain +-12 -> +-9 cm, slopes 25 -> 20 deg; ent_coef 0.005; budget 45 -> 120 min, cap 10M steps (owner OK'd 2 h). Warm k3/a1, root chain_c_terrain_rolling_d1. Gate unchanged.
- Next (owner): harder terrain stage = steep climb then abrupt drop within a few cm; pilot warm vs scratch.

## 2026-11-24 B3 terrain_rolling PASS (d1): success 0.86 (lo 0.779), imu_off drop 0.80 (CI lo 0.72), 10M steps / 77 min; skill models/skills/nav_chain/terrain_rolling.
- B4 terrain_ridges stage + generator added (tests pass). Launching warm pilot then scratch pilot (2 h budget each).
- 2026-11-24 B4 scratch pilot a1: success 0.01 (lo 0.002) after 7.1M steps/60 min vs warm 0.40/0.37; scratch a2 stopped by hand (priors matter). Keep warm line.
- B4 warm gate diagnosis (a1, n=100): success 0.68 with no ridge on the start-goal line (n=59), 0.00 with >=1 crossing (n=40), even 3-7 cm ridges. Policy crosses no ridge. Added ramp stage terrain_ridges_lo (level 0.5: ~3-6 cm, ~15-25 deg), warm from warm a1, root data/rl/chain_c_terrain_ridges_lo; then B4 again from its result.

## 2026-11-24 B4a terrain_ridges_lo PASS (a1, 10M steps / 75 min): success 0.72 (lo 0.625 vs 0.60), imu_off drop 0.38 (CI lo 0.28); skill models/skills/nav_chain/terrain_ridges_lo.
- First gate said FAIL only from my registration bug (median contact_frac >= 0.02 applied to obstacle-free ridge stages); fixed in stages.py and re-gated the same checkpoint -> PASS. The warm B4 failure (0.40/0.37) was real, unaffected.
- Next: B4 full ridges warm from B4a, root data/rl/chain_c_terrain_ridges_full.

## 2026-11-24 owner: policy avoided the ridges (line ridges can be walked around). Full-ridge B4 run (line ridges) stopped at ~7.5M steps.
- New terrain_ridges=2 mode: 4 closed wobbly ridge rings (r 0.95-2.45 m) around the start, climb outward then drop; every route to a 1.5-2.5 m goal crosses >= 2 (all directions cross 4). Stages terrain_rings_lo (level 0.5) then terrain_rings (level 1.0), gate 0.60 + imu_off 0.15, warm from B4a (ridges_lo a1); chained by script, roots data/rl/chain_c_terrain_rings_lo / chain_c_terrain_rings.
- 2026-11-24 rings_lo (level 0.5) a1: 0/100, 0 training successes in 6667 eps; sweep of that policy on rings: success 0.62 at level 0.05 and 0.1 (<~1.2 cm), 0.00 from level 0.2 (>= ~1.2 cm), stalls ~1 m out at the first ring. Earlier B4a pass (0.72) was probably only the ridge-free layouts. Policy never crossed a ridge above ~1.5 cm; may be a physical limit of the all-Z-joint body (cannot lift).
- Ladder (script): rings_lo level 0.15 -> rings_mid 0.3 -> rings 1.0, roots data/rl/chain_c_<stage>_ladder, warm from rings_lo a1; each gate 0.60 + imu_off 0.15. Stops at the first stage that fails twice; that fixes the empirical climb limit.
- 2026-11-24 ladder: rings_lo (level 0.15, ~1-2 cm) PASS a1 (0.73, lo 0.636, imu drop 0.73; skill terrain_rings_lo). rings_mid at level 0.3 (~1.8-3.6 cm) a1 FAIL 0/100 (stopped before a2). Bisecting: rings_mid level 0.22 (~1.3-2.6 cm), roots *_ladder2, warm from rings_lo a1; full-height rings follow only if it passes.
- 2026-11-24 owner saw the robot navigate then halt mid-run on rings_mid. Replay: head frozen (+-1 cm) from ~15 s at radius ~0.95 m (first ring) for the remaining 60 s while commands stay saturated (|a| 0.85, 75% at limit); render shows the body balled up in a tight self-locked coil in front of the ring. Not a height limit; a stuck local optimum that also wastes ~80% of samples. Fix: stall_s 20 / stall_gain_m 0.10 (existing GoalObstacles rule: end + charge the remaining time penalty) on terrain_rings_mid (level 0.22) and terrain_rings; verified it ends the replay at 35.9 s. Ladder restarted (roots *_ladder3, warm rings_lo ladder a1).
- 2026-11-24 rings_mid level 0.22 with stall rule (ladder3) a1: 0.04 (lo 0.016), worse than the level-0.15 policy zero-shot. Sweep of the passing level-0.15 policy (12 seeds/level): 0.75 @0.15, 0.58 @0.17, 0.33 @0.19, 0.42 @0.21, 0.17 @0.23: a gradual decline, not a cliff; the 0.15->0.22 step was too big. New small-step ladder (script, env PETCTL_RINGS_LEVEL): 0.19, 0.23, 0.27, 0.32, 0.38, 0.45, 0.55, 0.65, 0.8, 1.0; each stage gate 0.60 + imu_off 0.15, warm from the previous pass, roots data/rl/chain_c_rings_L<level>; stops at the first stage that fails twice = the empirical climbing limit.

## 2026-11-24 owner: switch from rings to a sandy-beach dune field ("dunes are perfect" after preview video docs/media/preview_dunes_L0.6_20261124.mp4)
- terrain_ridges=3 mode (petctl/sim/terrain.py::_dune_heights): close-packed asymmetric ripples (28-60 cm), p2p 9 cm x level, steep lee faces, mostly loose low-friction tiles, flat start disk only. Stage terrain_dunes (level via env PETCTL_DUNES_LEVEL; stall_s 20; gate 0.60 + imu_off 0.15 at the stage level).
- Ring ladder stopped at L0.19 (no gate yet; the rings_lo level-0.15 pass 0.73 stands, rings above ~0.17 fell off gradually). Dune ladder (script): levels 0.2, 0.3 ... 1.0, roots data/rl/chain_c_dunes_L<level>, warm from rings_lo ladder a1, each stage until pass; stops at the first stage that fails twice.
- 2026-11-24 owner: visual target back in videos. GoalReach.markers() put the gold ball at floor height (z=radius), buried under raised terrain (rolling base 15 cm + dunes); now rests on the terrain surface + a thin sphere-column beacon (render-only, no physics/training effect). Preview: docs/media/preview_dunes_target_L0.6_20261124.mp4.

## 2026-11-24 dunes L0.3: FAIL x2 (a1 0.66 lo 0.563; a2 0.58 lo 0.482); ladder stopped.
- Failures: mostly timeouts, plus 9/42 (a2) that never moved (stall at exactly 20 s, saturated commands from step 1). Not bearing (always 0), not start pose; correlates with low friction under the start (centre tile mu 0.40 vs 0.63 on successes; floor friction 0.85 vs 0.95). n small.
- Design change (stop rule): dunes use DUNE_LOOSE_MU (0.3, 0.5) instead of 0.2-0.4. Ladder 2 (script, roots data/rl/chain_c_dunes2_L<level>): levels 0.3 ... 1.0, warm from dunes_L0.3 a1 (best, 0.66).

## 2026-11-24 (night, owner asleep: continue autonomously until stopped)
- Dunes L0.3 on softer friction (ladder 2) a1: 0.43 (lo 0.337), 21/100 stuck-at-start (was 9 on the original friction): the friction hypothesis was wrong; softer friction hurt in every goal-angle bucket. Stuck/failing seeds concentrate where the goal is behind and to the left (body-frame angle 90-181 deg: n=22, success 0.18, 10 stuck) -> the policy cannot start a left turn-around.
- Queued (script dunes_ladder3.sh): after ladder 2 ends, if it stopped at a level, re-run from that level with the ORIGINAL sand friction (PETCTL_DUNE_SOFT_FRICTION=0, mu 0.2-0.4) and a TRAINING-only goal bearing of +-135 deg (PETCTL_DUNES_TRAIN_BEARING=135; gate stays +-180); roots data/rl/chain_c_dunes3_L<level>; warm from the last passing run (L0.3: dunes_L0.3 a1). Defaults (soft friction, bearing 180) unchanged so the running gate is consistent.
- 2026-11-24 night: dunes3 L0.3 (original friction, training bearing 135) a1 0.31 (lo 0.228): continued training keeps degrading the policy (0.66, 0.58, 0.43, 0.28, 0.31). ROOT CAUSE (mine): the stall rule charged time_penalty*remaining (~-27 at a 20 s stall) while a failed attempt costs ~-37 (timeout) plus tilt (~-14) and energy: quitting (coiling at the start) was cheaper than trying on hard goals (behind-left), so stuck-at-start grew with training. Fix: stall_mult 3.0 (GoalObstacles param, default 1.0 elsewhere; dunes only). Also reverted the friction default to the original (0.2-0.4) and training bearing back to +-180.
- Ladder 4 (script dunes_ladder4.sh; roots data/rl/chain_c_dunes4_L<level>): levels 0.3 ... 1.0, warm from the clean L0.2 pass, stops at the first stage failing twice.
- 2026-11-25 dunes4 L0.3 (stall_mult 3, original friction, bearing 180): a1 0.57 (lo 0.472), a2 0.58 (lo 0.482): FAIL x2. Start-stalls fixed (2-3 of ~42 failures, was 21/57). Failures now spread evenly over goal directions (0.42-0.69 per angle bucket) and end by the stall rule at a median ~45 s = real stalls mid-route on 2.7 cm dunes; plateau ~0.58. Step L0.2 (0.96) -> L0.3 probably too big.
- Ladder 5 (script dunes_ladder5.sh; roots data/rl/chain_c_dunes5_L<level>): levels 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0; warm from the clean L0.2 pass (dunes_L0.2 a1), each stage warm from the previous pass; stops at the first stage failing twice.
- 2026-11-25 dunes5 L0.25 PASS (see summary.jsonl in data/rl/chain_c_dunes5_L0.25); ladder 5 continues to L0.3.
- 2026-11-25 dunes5 L0.3 a1: 0.45 (lo 0.356) from the L0.25 pass (0.82): steep fall between ~2.3 and ~2.7 cm dune p2p regardless of path. a2 running.
- Queued (script dunes_ladder6.sh): after ladder 5 ends, if it stopped at L0.3: try L0.275 (roots data/rl/chain_c_dunes6_L0.275); then move on to C0 sparse_field warm from the best dune pass (root data/rl/chain_c_sparse_field_after_dunes). Dune limit then documented as bracketed.
- 2026-11-25 dunes6 L0.275: FAIL x2 (a1 0.56 lo 0.462; a2 see summary.jsonl). Dune curve with this policy/body: L0.2 0.96, L0.25 0.82 (PASS), L0.275 ~0.56, L0.3 0.45-0.50; reliable limit ~ level 0.25 (~2.3 cm p2p). Skill = dunes5_L0.25 a1. Script moved on to C0 sparse_field (root data/rl/chain_c_sparse_field_after_dunes), warm from that pass.

## 2026-11-25 ROOT CAUSE of the dune plateau / degradation: PPO action-std runaway
- driver.log std across the dune ladder: L0.2 3.5->4.2, dunes4 L0.3 4.2->10.9, dunes5 L0.3 6.6->14.2, C0 6.5->10.8 (B3 d1 had 0.33-0.58). With clipped actions (|a|<=1) a std > ~1 saturates the actions, the policy gradient on std vanishes and the entropy bonus (ent_coef 0.005) pushes it up without bound; the std was also inherited by every warm start. This (not terrain or friction) explains "continued training makes the policy worse" and C0 getting 0/100.
- Fix: NEW_STAGE ent_coef 0.0 and log_std_init -1.0 (re-applied after warm start; std 0.368 confirmed). C0 a1 (0/100) abandoned. Ladder 7 (script dunes_ladder7.sh; roots data/rl/chain_c_dunes7_L<level>): levels 0.3, 0.35, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0 from the L0.25 pass; then C0 sparse_field (root data/rl/chain_c_sparse_field_fixed). The dune "limit ~2.3 cm" is therefore NOT established; previous failures were largely the std runaway.
- Motion vocabulary: gait repertoire archive.json (179 cells) and archive_height.json (391 cells, head rise up to 12 cm dynamically); static pose search (poses.json, +-1 rad) found no held pose above the resting height (max module z 0.229 vs 0.232 m): vertical reach/bridge not holdable within +-1 rad / 0.5 N.m. Next: same search with +-2.5 rad.
