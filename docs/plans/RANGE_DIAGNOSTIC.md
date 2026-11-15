# Action-range diagnostic (Step A)

Policy: `data/rl/hug_curr4/hug_curr_02` (L=8, action_scale_rad 1.0), 20 deterministic episodes each of cyl r3 and cyl r7 (gate seeds 10000+, dr 0.6, 20 s, scored by the gate params of hug_curr_02). Wrap-attempt tick = hug run >= 3 modules; in-run joints = the joints between the run's modules; saturated = |action| >= 0.95.

| shape | episodes with a wrap attempt | wrap-attempt ticks | saturated frac (joint-ticks) | any in-run joint saturated | all in-run joints saturated | mean |target - angle| (rad) | max |target| in run (rad) |
|---|---|---|---|---|---|---|---|
| cyl_r3 | 20/20 | 4095 | 0.391 | 0.801 | 0.051 | 0.139 | 1.000 |
| cyl_r7 | 20/20 | 4811 | 0.212 | 0.617 | 0.008 | 0.188 | 1.000 |
| pooled | 40/40 | 8906 | 0.295 | 0.701 | 0.028 | 0.165 | 1.000 |

Metric used for the decision: saturated_frac = mean over wrap-attempt ticks (hug run >= 3) of the fraction of in-run joints with |action| >= 0.95, pooled over both shapes.

**Decision: B** (saturated fraction 0.295 >= 0.2). Rule: >= 20 % -> Step B (hug_scale15); < 10 % -> skip B, go to Step C; 10-20 % -> Step B.

Data: `data/rl/range_diag/result.json`.
