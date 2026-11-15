# M1d - odd-length (n = 9, 11) static feasibility and gate sets

Sim only. Protocol = M1c (docs/plans/M1C_SHAPE_FEASIBILITY.md): static DE + (1+lambda) polish, single seed 0, absolute-target PD, real cap, BusClamp, hold 3 s;
held = run >= 4 AND normal span >= 180 deg with penetration < 5 mm; "slack" = held run >= 5 modules. Driver `scripts/rl/m1d_odd_length_feasibility.py`, raw `data/rl/m1d_feasibility/table.json`.
Bodies: generated URDF variants (n = 9, 11; `tail_flip` auto), cached in data/urdf_variants.
**Budget reduced** vs M1c (7008 evals): ~3900-4000 evals per search (DE 3500 + polish 500), 6 workers, 45-85 s each (15 searches, 876 s total). A "no" is "not found by this search"
(see the r11 / n = 11 row: M1c found 202 deg with 7000 evals, this reduced run did not).

## Results (hold_ok = bus mean <= 4 A, applied peak <= 10 A, pen <= 5 mm, duty >= 0.9)
| shape | n | held run | held span (deg) | duty | bus mean / applied peak (A) | pen (mm) | held (hold_ok) | slack (run >= 5) |
|---|---|---|---|---|---|---|---|---|
| rounded 8x8 r2 | 8 | 8 | 324 | 1.00 | 0.66 / 0.7 | 0.0 | yes | yes |
| cyl r3 | 9 | 8 | 263 | 1.00 | 1.30 / 1.3 | 0.09 | yes | yes |
| cyl r7 | 9 | 9 | 200 | 1.00 | 3.33 / 3.4 | 0.14 | yes | yes |
| cyl r8 | 9 | 9 | 195 | 1.00 | 1.37 / 1.4 | 0.05 | yes | yes |
| square 8x8 | 9 | 9 | 270 | 1.00 | 1.27 / 1.3 | 0.06 | yes | yes |
| rect 5x14 | 9 | 6 | 180 | 1.00 | 1.59 / 1.6 | 0.10 | yes | yes |
| rounded 8x8 r2 | 9 | 9 | 328 | 1.00 | 0.77 / 0.8 | 0.0 | yes | yes |
| cyl r3 | 11 | 8 | 281 | 1.00 | 2.67 / 2.7 | 0.09 | yes | yes |
| cyl r7 | 11 | 11 | 252 | 1.00 | 1.45 / 1.5 | 0.07 | yes | yes |
| cyl r8 | 11 | 11 | 234 | 1.00 | 3.32 / 3.4 | 0.14 | yes | yes |
| cyl r9 | 11 | 11 | 229 | 1.00 | 2.83 / 2.8 | 0.12 | yes | yes |
| cyl r11 | 11 | 9 | 144 | 0.00 | 2.87 / 2.9 | 0.06 | no | no |
| square 8x8 | 11 | 8 | 270 | 1.00 | 1.92 / 1.9 | 0.11 | yes | yes |
| rect 5x14 | 11 | 5 | 180 | 1.00 | 2.64 / 2.7 | 0.12 | yes | yes (barely) |
| rounded 8x8 r2 | 11 | 10 | 233 | 1.00 | 0.93 / 0.9 | 0.0 | yes | yes |

## gate_r_max rule and gate sets
- **n = 8, rounded 8x8 r2: held run 8, 324 deg, duty 1.0, 0.7 A - slack yes.** The L = 8 gate shape is NOT a mis-calibration (it is the easiest shape of the set; the earlier "not run" is now closed). No change to the L = 8 set.
- n = 9: largest cylinder with slack among those tested = r8 (run 9, 195 deg); r9 was not tested at n = 9 (could be the gate radius; r8 is the proven one). r7 and r8 are both close to the 180 deg threshold margin (200 / 195 deg).
- n = 11: largest tested with slack = r9 (run 11, 229 deg, 49 deg margin); r11 not found at this budget (144 deg; M1c with 7000 evals did find 202 deg, run 11 of 11, no slack, so r11 has no slack at n = 11 either way). r10 not tested.
- Gate sets (same 5-shape structure; code `HELD_OUT_L9`, `HELD_OUT_L11` in `petctl/rl/target_shapes.py`, `held_out_set(n)`):
  - L = 9: cyl r3, cyl r8, square 8x8, rect 5x14, rounded 8x8 r2.
  - L = 11: cyl r3, cyl r9, square 8x8, rect 5x14, rounded 8x8 r2 (the L = 10 rounded 12x12 r2 was not searched at n = 11; 8x8 r2 was, with slack).
  - L = 8 and L = 10 sets unchanged.

## Caveats
- Single seed, ~4000 evals: "yes" rows are real (a held pose exists); "no" (r11 at n = 11) is a search miss at this budget as much as infeasibility.
- Bus mean for cyl r7 / r8 at n = 9 / 11 is 3.3 A, close to the 4 A budget used in hold_ok (continuous limit 4-6 A): these poses are not comfortable for hardware.
- rect 5x14 is on the 180 deg threshold at every n (run 5-6), the tightest shape; at n = 11 the slack is exactly 5 modules.
- Odd-n tail orientation (`tail_flip` auto) is an unverified physical assumption (docs/plans/URDF_GENERATOR.md).
