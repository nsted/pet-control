# M1c - shape feasibility (static search, absolute-target PD)

Sim only. Question: for the shape-agnostic target family, which held-out shapes can a body of n modules wrap (hug metric v2: chain run >= H = 4 AND outward-normal span >= 180 deg, held 3 s with >= 90 % wrap ticks) under the existing absolute-target PD, torque cap and bus model?

## Protocol (`petctl/rl/shape_search.py`, `scripts/rl/m1c_shape_feasibility.py`)
- M1 protocol generalised to shapes: candidate = joint angles q (|q| <= 1.6 rad), object offset (+-5 cm), start looseness alpha, and the object's yaw (non-cylinders); body laid down at alpha*q, object placed at the fitted-circle centre + offset, ramp to q, settle 1 s, hold 3 s (20 Hz, real cap, BusClamp).
- Objective: maximise the HELD normal span (min over the 3 s hold of the span of the longest run, counted only while run >= 4), with soft penalties for penetration > 5 mm and bus mean > 4 A / applied peak > 10 A. "180 deg held" in the table = bus budget met, penetration <= 5 mm and wrap-tick duty >= 0.9.
- Search: differential evolution (~6000 evals, seeded with partial coils incl. gentle 0.35-0.7 rad curls for large objects) + (1+lambda) ES polish (1000 evals), seed 0, one restart, 6 worker processes. Single seed: a "no" is "not found by this search", not a proof of infeasibility.
- Scope (owner's time box, revised order): 4 shapes (cyl r 3 cm, cyl r 11 cm, square 8x8, rectangle 5x14) at n = 8 and 10, plus cyl r 11 at n = 11 and 12 to bracket the shortest covering length. NOT run: rounded 12x12 r2, the extremes (cyl r 2, cyl r 12, box 16x16).
- Limitation: absolute-target PD only (the delta-action path comes later); a coarse first run (1400 evals) agreed with the 6000-eval run on every row except the r11/n=8 value (122 vs 139 deg held span).

## Results
| shape | n | held run (min over 3 s) | held span (deg, min over 3 s) | wrap duty | 180 deg held (duty>=0.9) | bus mean/applied peak (A) | pen (mm) | max abs joint (rad) | evals | wall (s) |
|---|---|---|---|---|---|---|---|---|---|---|
| cyl r11.0 | 8 | 8 | 139 | 0.00 | no | 1.53 / 1.6 | 0.1 | 1.36 | 7008 | 83 |
| cyl r3.0 | 8 | 8 | 291 | 1.00 | yes | 1.07 / 1.1 | 0.1 | 1.28 | 7008 | 93 |
| box 5x14 | 8 | 6 | 180 | 1.00 | yes | 1.16 / 1.2 | 0.1 | 1.34 | 6948 | 92 |
| box 8x8 | 8 | 7 | 216 | 1.00 | yes | 1.87 / 1.9 | 0.2 | 1.46 | 6948 | 95 |
| cyl r11.0 | 10 | 10 | 180 | 0.00 | no | 2.55 / 2.6 | 0.1 | 0.91 | 7008 | 99 |
| cyl r3.0 | 10 | 9 | 302 | 1.00 | yes | 2.16 / 2.2 | 0.0 | 1.41 | 7008 | 141 |
| box 5x14 | 10 | 6 | 180 | 1.00 | yes | 2.31 / 2.3 | 0.0 | 1.38 | 6988 | 105 |
| box 8x8 | 10 | 9 | 270 | 1.00 | yes | 1.32 / 1.3 | 0.1 | 1.29 | 6988 | 124 |
| cyl r11.0 | 11 | 11 | 202 | 1.00 | yes | 1.42 / 1.4 | 0.0 | 1.00 | 6988 | 103 |
| cyl r11.0 | 12 | 11 | 177 | 0.00 | no | 4.19 / 4.4 | 0.2 | 0.83 | 6888 | 85 |

Raw: `data/rl/m1c_shape_feasibility/table.json`; contact sheets (top row oblique, bottom row top-down, one column per shape): `best_poses_n8.png`, `best_poses_n10.png` (4 shapes), `best_poses_n11.png`, `best_poses_n12.png` (cyl r11 only).

## Reading
- n = 8: feasible for cyl r3, square 8x8 and rectangle 5x14; **infeasible for cyl r11** (best held span 139 deg with a held run of 8, i.e. the whole body touches but cannot reach half-way round).
- n = 10: same three feasible with margin (cyl r3 302 deg, square 270 deg, rectangle 180 deg - the rectangle sits exactly on the threshold at both lengths, it is the tightest non-cylinder); **cyl r11 still not held** (best 180 deg peak but duty 0.00: the span is at the threshold only momentarily and the pose does not hold it for 3 s; all 10 modules in the run).
- cyl r11 needs n = 11: held span 202 deg, duty 1.0, bus 1.4 A, all 11 modules in the run (no slack). The n = 12 search (same budget) did not find a held wrap (177 deg, bus 4.2 A) - a search miss at a larger dimension, not evidence that 12 is worse; it should be repeated with more restarts before relying on it.
- Shortest L covering the family: **not 10** for r11. Within the shapes run, 11 modules wrap r11 with zero slack; with the requested >= 1 module of slack the answer is 12, but this was not confirmed by the search (r12 and 16x16 were not run, and would be harder than r11). Expert's expectation (10) holds for cyl r3, square 8x8 and rectangle 5x14 but fails for r11 in the absolute-PD static setting; the delta-action path or a torque-profile change could move this, so treat 10 vs 12 as open until the follow-up task.
