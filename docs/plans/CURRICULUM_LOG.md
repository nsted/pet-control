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
