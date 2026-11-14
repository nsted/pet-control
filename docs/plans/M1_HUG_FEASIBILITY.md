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
