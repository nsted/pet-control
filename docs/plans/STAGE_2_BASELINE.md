# STAGE_2_BASELINE: T14 gate baseline (zero-delta PD through the frozen evaluator)

Best PD per cell chosen on the DECISION suite of the same level (lowest sum E among configs that keep I_max_2s <= 4 A and tick peak <= 10 A in every scenario); numbers below are its TEST-suite re-measurement. E = mean wrapped joint error over the whole episode (rad); E_dist = the 4 s window after onset; lost = fraction of scenarios with a joint > 0.6 rad off for > 1 s; I = demanded bus current (A, proxy).

| level | cell | best config (kp, zeta) | E test | E_settle | E_dist | lost | I mean | worst config E | n |
|---|---|---|---|---|---|---|---|---|---|
| L0 | arch 1 right | 5.0, 2.0 | 0.035 | 0.032 | 0.036 | 0.00 | 0.61 | 0.153 | 12 |
| L0 | arch 1 upright | 5.0, 2.0 | 0.014 | 0.012 | 0.015 | 0.00 | 0.49 | 0.243 | 12 |
| L0 | arch 2 upright | 5.0, 0.5 | 0.015 | 0.013 | 0.015 | 0.00 | 0.50 | 0.063 | 12 |
| L0 | head lift upright | 5.0, 2.0 | 0.025 | 0.025 | 0.025 | 0.00 | 0.53 | 0.331 | 12 |
| L0 | lattice compact upright | 5.0, 2.0 | 0.005 | 0.004 | 0.005 | 0.00 | 0.46 | 0.022 | 12 |
| L0 | lattice mid upright | 5.0, 2.0 | 0.012 | 0.011 | 0.012 | 0.00 | 0.49 | 0.069 | 12 |
| L2 | arch 1 right | 5.0, 2.0 | 0.035 | 0.032 | 0.036 | 0.00 | 0.61 | 0.159 | 12 |
| L2 | arch 1 upright | 5.0, 2.0 | 0.014 | 0.013 | 0.015 | 0.00 | 0.49 | 0.219 | 12 |
| L2 | arch 2 upright | 5.0, 2.0 | 0.015 | 0.013 | 0.015 | 0.00 | 0.50 | 0.069 | 12 |
| L2 | head lift upright | 5.0, 2.0 | 0.025 | 0.025 | 0.025 | 0.00 | 0.54 | 0.322 | 12 |
| L2 | lattice compact upright | 5.0, 0.5 | 0.005 | 0.004 | 0.006 | 0.00 | 0.46 | 0.040 | 12 |
| L2 | lattice mid upright | 5.0, 2.0 | 0.012 | 0.011 | 0.012 | 0.00 | 0.50 | 0.069 | 12 |
| L3 | arch 1 right | 5.0, 2.0 | 0.034 | 0.032 | 0.035 | 0.00 | 0.61 | 0.176 | 12 |
| L3 | arch 1 upright | 5.0, 2.0 | 0.014 | 0.012 | 0.015 | 0.00 | 0.50 | 0.239 | 12 |
| L3 | arch 2 upright | 5.0, 2.0 | 0.016 | 0.013 | 0.017 | 0.00 | 0.51 | 0.088 | 12 |
| L3 | head lift upright | 5.0, 2.0 | 0.026 | 0.026 | 0.027 | 0.00 | 0.55 | 0.331 | 12 |
| L3 | lattice compact upright | 5.0, 1.0 | 0.006 | 0.004 | 0.008 | 0.00 | 0.48 | 0.025 | 12 |
| L3 | lattice mid upright | 5.0, 2.0 | 0.015 | 0.011 | 0.020 | 0.00 | 0.58 | 0.086 | 12 |

Headroom rule (STAGE_2_GATES): no headroom if best-PD E_dist <= 0.03 rad with no losses. Per level, cells meeting it: L0: 5/6; L2: 5/6; L3: 5/6
