# M1: static hug feasibility (n_modules 8..15)

Sim only. Search code: `petctl/rl/hug_search.py`, driver `scripts/rl/m1_hug_feasibility.py`, raw results `data/rl/m1_hug_feasibility/table.json`, one contact-sheet PNG per length `data/rl/m1_hug_feasibility/best_pose_n{n}.png` (oblique + top-down at t=0.1, 1.0 s and 6 s).

## Protocol (what was searched and how a candidate is scored)

- Decision variables per candidate: `n-1` joint angles (bounded +-1.6 rad), the post start offset `(dx, dy)` (bounded +-5 cm from the fitted-circle centre of the strongly curled modules), and `alpha` in [0.3, 0.95] (start looseness).
- Placement: body laid down for 1.5 s at `alpha*q` with the post far away (a wide coil, so the post can be dropped inside it with no penetration), then the post is put at the circle centre + offset (rejected if it would penetrate > 3 cm). The command then ramps linearly `alpha*q -> q` over 2 s while the body tightens round the post, settles 1 s, and is HELD 3 s. Floor friction 1.0, kp/kd defaults, delay 0.08 s, `REAL_CAP_NM` = 0.5 N.m per joint, current model on with the 10 A spike clamp, 20 Hz ticks, MuJoCo dt 0.004.
- Score = (min over the 3 s hold of the debounced longest chain-contiguous hugging run, `hug_modules`) + 0.25 x mean run + 0.5 x touching fraction - 0.05 x mean bus A - penalties (mean bus > 4 A, applied peak > 10 A, post penetration > 1 cm). `held` below is the first term for the best candidate.
- Optimiser: scipy differential evolution, 2 restarts x ~1900 evaluations, population seeded with partial coils (window of consecutive joints at +-s, both signs), each restart followed by a (1+lambda) ES polish (~400 evaluations, sigma 0.02 rad / 2 mm). ~5400 evaluations per length in total (the polish was not in the brief's 2000-4000; it was added because the hug window is millimetre-scale). 8 worker processes.

## Results

| n | H | held hug_modules (3 s) | H reached | bus mean / applied peak (A) | max abs joint (rad) | wall (s) | geometry | notes |
|---|---|---|---|---|---|---|---|---|
| 8 | 5 | 3 | no | 1.50 / 1.5 | 1.29 | 151 | planar | wrap 67 deg, z-span 3.0 cm, pen 0.1 mm, robustness (+-5 mm post offset) held [2, 1, 2, 1], feasible True |
| 9 | 5 | 3 | no | 2.41 / 2.5 | 1.34 | 180 | planar | wrap 67 deg, z-span 3.1 cm, pen 0.1 mm, robustness (+-5 mm post offset) held [1, 1, 2, 2], feasible True |
| 10 | 5 | 2 | no | 1.24 / 1.2 | 1.28 | 185 | planar | wrap 29 deg, z-span 3.5 cm, pen 0.0 mm, robustness (+-5 mm post offset) held [2, 1, 1, 1], feasible True |
| 11 | 5 | 2 | no | 1.07 / 1.1 | 1.38 | 197 | planar | wrap 34 deg, z-span 3.0 cm, pen 0.2 mm, robustness (+-5 mm post offset) held [1, 1, 1, 1], feasible True |
| 12 | 5 | 2 | no | 0.87 / 0.9 | 1.23 | 208 | planar | wrap 34 deg, z-span 2.8 cm, pen 0.0 mm, robustness (+-5 mm post offset) held [0, 1, 2, 0], feasible True |
| 13 | 6 | 2 | no | 2.33 / 2.4 | 1.47 | 228 | planar | wrap 28 deg, z-span 3.9 cm, pen 0.0 mm, robustness (+-5 mm post offset) held [1, 1, 1, 0], feasible True |
| 14 | 6 | 3 | no | 2.25 / 2.3 | 1.11 | 237 | planar | wrap 65 deg, z-span 3.1 cm, pen 0.1 mm, robustness (+-5 mm post offset) held [1, 1, 2, 1], feasible True |
| 15 | 6 | 2 | no | 2.55 / 2.6 | 1.54 | 246 | planar | wrap 30 deg, z-span 3.1 cm, pen 0.0 mm, robustness (+-5 mm post offset) held [1, 1, 1, 1], feasible True |
| 11 (scripted reference) | 5 | 0 (9 touching transiently) | no | not recorded | 0.88 | - | planar coil | all joints +0.88: 9 contiguous modules, 274 deg wrap, NOT held 3 s (post pushes the body off); CURRICULUM_LOG.md |

Best pose angles (rad, servo 1..n-1), post offset (m) and alpha per length:

- n=8: q = [-0.208, -0.417, -1.259, -0.621, -0.12, -1.292, 0.023], offset = [-0.0461, -0.0183], alpha = 0.95
- n=9: q = [-1.278, -0.331, 1.341, 0.214, 0.827, 1.273, 0.701, 0.076], offset = [-0.0416, 0.0366], alpha = 0.70
- n=10: q = [0.146, -0.79, -0.829, -0.124, 0.998, 1.28, 0.701, -0.474, -0.907], offset = [-0.0156, 0.0452], alpha = 0.94
- n=11: q = [0.314, 0.563, -0.186, -0.394, -0.353, -0.534, -1.349, -1.382, -0.373, -1.313], offset = [0.0054, 0.0187], alpha = 0.92
- n=12: q = [-0.58, -0.584, -0.077, 0.442, -0.273, -0.627, 0.093, -0.655, 0.36, -0.502, 1.226], offset = [0.0352, 0.0012], alpha = 0.90
- n=13: q = [0.951, -0.459, -1.057, 0.436, -0.805, -0.142, 0.133, -0.213, -0.065, -1.187, -1.468, 0.086], offset = [0.036, 0.0177], alpha = 0.76
- n=14: q = [0.137, -0.129, -0.119, 0.118, 0.235, -0.27, -0.307, -0.426, -0.442, 0.304, 1.111, 0.887, 0.862], offset = [-0.0027, 0.0034], alpha = 0.87
- n=15: q = [0.586, 0.907, 0.669, 0.935, 0.006, 0.169, 0.948, -0.035, -1.154, 0.025, -0.25, -1.539, 0.309, 1.357], offset = [-0.029, 0.0001], alpha = 0.90

## Did the search beat the scripted reference?

Not on the number that matters. The scripted 11-module pose reached 9 contiguous modules (274 deg wrap) but only transiently; nothing was held. The search's best held values are 2-3 modules for every length, so it holds a little contact for 3 s (reference: 0) but is far below both the reference's transient 9 and the target H. The search never found a pose with H held; no length reaches H.

## What limits it (from the search and from targeted probes of the sim)

1. **Geometry / contact tolerance, not current.** Held current is 0.9-2.6 A mean (applied peak <= 2.6 A, demand never near 10 A) for the best poses, so the 4 A / 10 A budget is not binding at these static poses. Joint angles used are 1.1-1.5 rad (cap +-1.6), also not binding.
2. **The coil is a polygon, and contact needs a mm-accurate radius.** A coil of identical joints has module centres on a circle; the modules' side faces touch the post only when that radius is <= ~0.089 m, and the faces (35 deg cone) only line up near the chord mid-points. In probes (n=8, 5-6 curled joints, s 1.0-1.6, loose starts) the modules sat at r = 0.090-0.105 m: only the two coil-end modules touched the post (10-14 N each) and the middle ones stood 1-10 mm off, because friction (mu = 1) and the stiff chain (kp 0.8 N.m/rad, cap 0.5) keep the coil where it settled, and the posts pins it at its ends. Tightening the target further does not close the gap; it only raises the end forces or pushes the coil out.
3. **The post pushes the body away if the coil starts inside it.** Placing the coil kinematically on the post (penetration 1-2 cm) ejects the body to r ~0.10 m (this is the reference's 'post pushes body off'); so candidates must start loose and tighten, and the held result depends on path and friction.
4. **Length does not help in this protocol** (8-15: 2-3 held). Longer bodies close the ring on themselves (n >= 9 wraps >= 360 deg at r 8.5 cm: self-contact) or need helical stacking, which the search did not find (all best poses are planar: z-span <= 3.9 cm = the even/odd zig-zag).

## Doubts / caveats for the director

- The protocol and the objective's plateau structure may be the real limit: the feasible hug set seems to be a thin sliver (radius tolerance of ~1 mm, post centring ~1 mm). A longer/finer search or a protocol that allows compliant settling (e.g. progressively tightening while relaxing friction) could find holds the DE+ES did not; this was not run. Sim-to-real: real foam/pads are compliant, the sim hulls are rigid.
- Hug counts are over the debounced ground-truth face metric (35 deg cone). Edge/corner contacts of the prism hulls do not count (`any_contact` was sometimes higher), which may understate a 'physical' hug.
- I did not log the transient peak of the search poses (the reference's 9 is a transient peak), so the comparison reference-vs-search is held-vs-transient; a like-for-like transient comparison was not done.
- `HELIX_Z_M` was first set to 2.5 cm (below the 3 cm zig-zag) and mislabelled everything helical in the raw log lines; table.json was re-labelled from the stored z-span (planar everywhere). Helical poses may exist but were not found; the seed population contained no helical seeds beyond what the 45 deg joint axes give.
- Bounds, alpha range, seeds, start protocol are my choices (no design decisions intended); the run log is `data/rl/m1_hug_feasibility/run.log`.


## M1b: re-score under the proximity definition (2026-11-13)

Why: the geom-contact metric of M1 was unreachable by construction (flat 6 cm faces vs a 5 cm-radius post, rigid hulls; table above). New definition (plan sec 5.3): face-to-post-surface distance <= 15 mm (ground truth, fixed; the capacitive sensor model randomises its own 8-20 mm threshold per episode), module hugs if in >= 3 of the last 5 ticks, hug tick = longest chain-contiguous run >= H, **held = hug tick on >= 90 % of the ticks of a sliding 3 s window**, no squeeze/pressure condition (encirclement, normal force, wrap are diagnostics). Code: `petctl/rl/capacitive.py` (`PostProbe`, `D_PROX_M`), `petctl/rl/hug.py`, script `scripts/rl/m1b_rescore.py`, raw `data/rl/m1_hug_feasibility/m1b_rescore.json`, `m1b_sweep.json`, `m1b_sweep_n12_fine.json`. Same protocol as M1 (ramp alpha*q -> q over 2 s, settle 1 s, hold 3 s, 0.5 N.m cap, bus model with 10 A clamp, delay 0.08 s, friction 1.0).

**Implementation note (deviation from the hint).** MuJoCo `margin = gap = 0.015` on the post geom does NOT give force-free detection in MuJoCo 3.17: contacts reported `includemargin` = margin regardless of `gap` (tested gap 0 / 0.01 / 0.02 on post and hulls), so forces appeared at 10-15 mm separation and the trajectories changed. The proximity is therefore computed directly: `mj_geomDistance(hull_k, post, distmax 20 mm)` per module (about 0.01 ms for 11 modules), the nearest-point direction mapped to a face with the existing 35 deg cone; penetrating hulls still use the real contacts (force, penetration unchanged). Physics is unchanged (test: trajectories bit-identical with and without reading the probe); post geom margin/gap are not used.

### M1 best poses re-scored (the saved `x` of table.json, same protocol)

| pose | n | H | min / median run over 3 s hold | hug-tick duty | hold (>= 90 %) | bus mean / applied peak (A) | pen (mm) | wrap (deg) |
|---|---|---|---|---|---|---|---|---|
| M1 best n=8 | 8 | 5 | 5 / 5 | 1.00 | YES | 1.5 / 1.5 | 0.1 | 129 |
| M1 best n=9 | 9 | 5 | 6 / 6 | 1.00 | YES | 2.41 / 2.5 | 0.1 | 158 |
| M1 best n=10 | 10 | 5 | 3 / 3 | 0.00 | no | 1.24 / 1.2 | 0.0 | 59 |
| M1 best n=11 | 11 | 5 | 4 / 4 | 0.00 | no | 1.07 / 1.1 | 0.2 | 92 |
| M1 best n=12 | 12 | 5 | 4 / 4 | 0.00 | no | 0.87 / 0.9 | 0.0 | 94 |
| M1 best n=13 | 13 | 6 | 3 / 3 | 0.00 | no | 2.33 / 2.4 | 0.0 | 61 |
| M1 best n=14 | 14 | 6 | 5 / 5 | 0.00 | no | 2.25 / 2.3 | 0.1 | 131 |
| M1 best n=15 | 15 | 6 | 2 / 2 | 0.00 | no | 2.55 / 2.6 | 0.0 | 30 |
| scripted coil alpha=0.7 | 11 | 5 | 0 / 0 | 0.00 | no | 6.64 / 6.7 | 0.0 | 0 |
| scripted coil alpha=0.85 | 11 | 5 | 1 / 1 | 0.00 | no | 4.56 / 4.6 | 0.0 | 0 |
| scripted coil alpha=0.95 | 11 | 5 | 0 / 0 | 0.00 | no | 4.46 / 4.5 | 0.0 | 0 |
| scripted coil alpha=1.0 | 11 | 5 | 1 / 1 | 0.00 | no | 3.46 / 3.5 | 0.0 | 0 |

(M1 reported held 3, 3, 2, 2, 2, 2, 3, 2 modules for n = 8..15 under geom contact.) n=8 and n=9 poses already satisfy H with the 90 % duty rule (5 and 6 contiguous modules within 15 mm for the whole hold). n=10-15 poses were optimised for the old metric and are not hugs under the new one (run 2-5, duty 0). The scripted 11-module coil is not held: at the M1 placement protocol the post is pushed off / the body jams (bus 3.5-6.6 A, run 0-1), so it is only reachable dynamically (see below).

### Dynamic hold test: scripted coil, all joints at one angle (ramp-in alpha*q -> q), post at the fitted-circle centre

Angle grid 0.20..1.40 rad step 0.05, alpha in (0.7, 0.85, 0.95), n = 8, 10, 12 (75 evaluations per n). Feasible = bus mean <= 4 A, applied peak <= 10 A, penetration <= 10 mm.

| n | angle range with duty >= 0.9 and bus mean <= 4 A, peak <= 10 A, pen <= 10 mm (alpha 0.85 / 0.95) | best (angle, alpha): run, duty, bus mean A |
|---|---|---|
| 8 | 0.8..1.05 rad (8 of 75 grid points) | (0.8, 0.95): 8, 1.0, 0.82 |
| 10 | 0.85..1.0 rad (6 of 75 grid points) | (0.9, 0.95): 10, 1.0, 1.66 |
| 12 | none | - |

n=8: all 8 modules within 15 mm for the whole hold at angles 0.80-1.00 rad (alpha 0.85-0.95), bus 0.8-2.6 A, penetration 0.0 mm, wrap 200 deg. n=10: all 10 modules at 0.85-1.00 rad, bus 1.7-3.8 A, wrap 265 deg. n=12 (coarse grid): the only duty-1.0 points have bus 4.7-10 A or penetrate the post by ~10 mm; a finer 108-point grid (angle 0.70-1.10 step 0.025, alpha 0.86-0.98) gave 19 points with duty >= 0.9 but only 2 feasible, both with post penetration 9.9-11 mm (the coil closes on itself, wrap 383 deg, and the post sits in a rigid-hull overlap): the best feasible is angle 1.025, alpha 0.96, 12 modules, bus 3.46 A, pen 9.9 mm (at the 10 mm validity limit, i.e. borderline). So H is reached by a constant coil at n=8 and n=10 cleanly, at n=12 only at the penetration/current limits.

Caveats: single-shot results, the duty-1.0 region is narrow in alpha (start looseness: alpha 0.7 fails at 8/10), post placement is at the fitted circle centre (no offset search), and the sweep did not test robustness to post offset or friction. Straight body alongside the post: up to 3 modules (not 2 as the plan states) are within 15 mm (hull length 6 cm > pitch 5.1 cm; test `test_straight_body_alongside_post_is_no_hug`), still below H >= 5.
M1b outcome: H reachable under the proximity definition for n = 8, 9 (searched poses) and n = 8, 10 (constant coil, with margin); no new DE search was needed.
