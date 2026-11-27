# Phase 7: RL in CPG-parameter space (n = 11) - working log

Env `petctl/rl/cpg_env.py`, training `scripts/rl/train_cpg.py` (PPO, ent_coef 0, log_std_init -1), eval/video `scripts/rl/eval_cpg.py`, library relabelling `scripts/rl/relabel_elites.py`, tests `tests/test_cpg_env.py`.

| pilot | change | result |
|---|---|---|
| 1 | first reward (exp kernel sigma 0.06) | tracking 0.46 = a body that stands still (0.47); reviewer NO-GO |
| 2 | improvement over standing still, robust yaw, masks, current penalty | no learning (0.008): reward flat far from the target |
| 3 | + dense progress term, faster param steps | no learning |
| 4 | + library selector start gait, incremental actions | params drifted away from the library; worse than doing nothing |
| 5 | residual action around the selector | ~ selector baseline (0.09-0.11 on the older metric) |
| 6 | + f scaling | abandoned (reviewer 2: NO-GO; f scaling made tracking worse; penalties swamp the signal) |
| 7 | env-measured library labels (+ CMA specialists), symmetric clip, turn-following heading, per-mode success, light penalties | success 0.032 -> 0.043 (selector baseline ~0.03), but the deterministic policy writhes the same for every command: not tracking |
| 8 | + residual-magnitude penalty | flat at baseline (0.031); eval ~0 |
| 9 | + privileged window-velocity observation (needs a velocity estimator on the robot) | flat (0.029); contact sheet: the policy curls into a tight pile (lazy local optimum) |
| 10 | steady stage (one command per 20 s episode, upright, 3 s mask; selector scores 0.069 here: move 0.107, mixed 0.224, turn 0.008) | 0.038, below the selector; eval negative |
| 11 | + low-pass action (temporally coherent exploration) | running (0.007 at 640k) |

Adversarial reviews (2): found the stand-still dominance (0.47), gameable chord yaw, shape-dependent heading frame, flat gradients, library labels disagreeing with the env measurement (a 2-point unwrap aliased fast spins), penalties swamping the signal. All fixed, each with a test or probe.
Status: NO claim that a policy tracks commands. The library selector alone scores ~0.03 (move -0.01, turn +0.09, mixed +0.07 per tick), which is the number a learned policy must clearly beat.
Open: noisy 12-seed baselines; zero-action selector is weak because gaits need seconds to settle after each switch; no velocity feedback in the observation.

Review 3 (NO-GO for a 12M run on the env at that time): no lasting exploit that pays without moving, but (a) MuJoCo silently resets on QACC blow-ups (seen at kp x2 / kd x0.5) so the NaN check never fired and the reset jump counted as velocity, (b) watchdog min_metric 0.02 was below the baseline so a stuck run never stops, (c) no checkpoints, (d) current penalty too weak. Fixed: episodes end with a penalty on QACC warnings or a >0.1 m COM jump per tick, kd >= sqrt(kp scale) x nominal, current penalty 0.05/A, watchdog min_metric 0.06 by 3M steps, checkpoints every 500k, gain-extreme test.
Headroom checks: a hand-written amplitude feedback on the selector (scripts/rl/feedback_cpg.py) did not improve it (move 0.005 -> 0.007), so the library selector itself, not missing feedback, is the limit; gait switching (a new gait needs seconds to build speed, ends in a different body configuration) costs most of the tracking score (selector: switching 0.03, steady commands 0.07).
Next if pilot 11 fails: stop PPO variants; build steering as a classical selector with parameter blending between neighbouring library gaits and transitions through a neutral straight pose (the skill-transition problem of Phase 8), then bring RL back to refine it.
