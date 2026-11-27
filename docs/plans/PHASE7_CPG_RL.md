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
| 8 | + residual-magnitude penalty | running |

Adversarial reviews (2): found the stand-still dominance (0.47), gameable chord yaw, shape-dependent heading frame, flat gradients, library labels disagreeing with the env measurement (a 2-point unwrap aliased fast spins), penalties swamping the signal. All fixed, each with a test or probe.
Status: NO claim that a policy tracks commands. The library selector alone scores ~0.03 (move -0.01, turn +0.09, mixed +0.07 per tick), which is the number a learned policy must clearly beat.
Open: noisy 12-seed baselines; zero-action selector is weak because gaits need seconds to settle after each switch; no velocity feedback in the observation.
