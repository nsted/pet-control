# Phase 4 (in progress): CMA-ES gait search, validation at n = 11

Tool: `scripts/rl/gait_search_cma.py` (11 CPG params, 1500 evals, 0.5 N.m cap, isotropic, GIM current proxy). Check: `scripts/rl/verify_gaits.py` (world-frame COM displacement, independent of the search metrics).

| objective | n=10 | n=11 | n=12 | n=13 | n=11 seed 1 | n=11 fmax 4 Hz |
|---|---|---|---|---|---|---|
| forward cm/s (world) | 17.7 | 19.3 | 13.3 | 15.7 | 10.6 | 11.6 |
| sideways cm/s | - | 17.2 | - | - | 21.9 | 17.8 |
| turn rad/s | - | 1.96 | - | - | 1.77 | - |
| roll rad/s / travel cm/s | 4.1 / 13.6 | 5.6 / 21.8 | 5.8 / 27.0 | 4.9 / 20.8 | 2.1 / 9.5 | - |

Findings
- World-frame displacement matches the search metrics (within a few %), max module height < 16 cm, no net rise: not a launch or metric artefact. Rolling is whole-body (5 modules agree).
- Grid best was 3-4 cm/s; search gives 5x. Speeds are seed-dependent (forward 19 vs 11 at n=11), so one seed per length cannot rank lengths. No length signal yet.
- Relaxing f to 4 Hz did not raise speed: the limit is the 0.5 N.m torque cap, not the 2 Hz bound.
- Open: n=12 roll step 2.8 rad/tick is near the pi aliasing limit; real-robot feasibility (2 Hz, cap-saturated, unvalidated current proxy) untested.

## MAP-Elites (4 lengths x 2 seeds x 5000 evals; `map_elites.py`, `reeval_archive.py`, `render_archive.py`, `verify_archive.py`)
After the yaw-metric fix (all chords must agree; a thrashing head had faked 5-11 rad/s turns, real max ~2 rad/s): cells 62-82, QD 9.3-12.7, best speed 25-34 cm/s at every length; seed-to-seed spread as large as the spread between lengths; n=12 slightly slower (25/27 vs 29-34). 5-10 cells per archive roll > 2 rad/s while travelling > 10 cm/s at every length.

## Decision
Length settled at 11 modules by the owner (preference + tie-break), not by a pre-registered study. See CURRICULUM_STATE.md.

## Self-righting (n = 11)
One open-loop gait cannot do it: the first "success" was a rolling body that was upright only when the clock ended (rejected by video; criterion tightened to upright at EVERY tick of the last 5 s, which no single CPG gait met). Two-stage skill (`self_right2.py`): CPG rolling motion until the median module up-component > 0.8 for 0.3 s (computable from head IMU + joint angles), then amplitudes ease to zero (1.5 s) and the body lies straight and still. Succeeds from the inverted start (stop at ~4.9 s) and from the side start; found within 126 evals. Video selfright2_n11.mp4.
Dynamic gap crossing (open loop, 25 s, whole body ends on the far platform): n = 10, 11, 13 cross 10 cm; at 15 cm only n = 11 (sideways start) and n = 13 (rolling start) succeeded in 540 evals; n = 10, 12 did not. n = 11 at 20, 25, 30 cm running.

Gap crossing at n=11 (open loop, 25 s): 10, 15, 20 cm succeed (sideways-start gaits); 25 and 30 cm not found (3 starts x 900 evals). Video gap_cross_n11.mp4.

## Library flaw found during Phase 7 (2026-11-27): gaits overfit to one start state
`scripts/rl/gait_repro.py`: each elite re-measured in CPGEnv from the nominal start reproduces its label exactly (median error 0.00), but at floor friction x1.3 only 5/34 stay within 40% of the label velocity (median error 0.67) and after a 3 s run of another gait only 5/34 (median error 0.94). The first MAP-Elites / CMA-ES gaits were optimised from a single start state, so they exploit the exact body configuration and are not usable as a library for tracking, steering or RL (this explains the failure of pilots 1-11 and of the classical selector). Floor friction x0.7 changed nothing at all (MuJoCo uses the larger of the two contact frictions): the friction randomisation was half effective.
Fix: `scripts/rl/map_elites_robust.py` measures every candidate under 4 conditions (nominal, friction x1.3, after a 3 s pre-run of two different fixed gaits) and uses fitness = median speed - dispersion over conditions. Result below once done.

## Gap crossing robustness (2026-11-27)
`gap_robust.py`: the first open-loop crossings (10, 15, 20 cm) succeeded only from the exact search start (start heading +-0.15 / +-0.3 rad: 0/4 succeed; friction x1.3: 1/3), the same overfitting as the first gait library. `gap_cross_robust.py` scores each candidate over 4 starts (nominal, heading +-0.15, friction x1.3): at 10 cm it finds a crossing that succeeds from all 4 and from 5 of 6 test perturbations (fails at heading -0.3 rad). 15 and 20 cm: no crossing robust to all 4 conditions in 600 evals x (previous + 3 seeds) starts (best 0/4 and 1/4). A reliable 15-20 cm crossing needs closed-loop heading control (align to the platform edge) or a longer search.

## Self-righting robustness (2026-11-27)
Two-stage self-righting (n11_v2.json) re-tested from 9 start rolls (inverted, inverted +-0.3 rad, side, side +-0.3 rad, side -90 deg, 135 deg tilt both ways) at floor friction x1.0 and x1.3, same strict criterion (whole-body median up > 0.6 at every tick of the last 5 s): 17 / 18 succeed (only side -0.3 rad at friction 1.0 fails). Unlike the gaits and the gap crossings, this skill is robust, probably because it is closed-loop (the upright trigger) and the straight hold pose is a strong attractor.
