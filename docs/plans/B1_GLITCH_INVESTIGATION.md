# B1 attempt 1 gate: glitch and 10 A peak investigation (2026-11-15)

Gate: `data/rl/chain_ab/terrain_easy/a1/gate2_terrain_easy.json` (n=100, seeds 10000-10099, DR 0.6, terrain level 0.6).
Replays: single-process, policy `a1/policy.npz`, env config built exactly as `PolicyEvaluator.__call__` does. No code changed.

## 1. Glitch (seed 10085, steps = 1, return -303.6)
Which guard fired: **contact penetration > 1 cm, at reset (before the first policy action)**, not a launch (max module clearance 0.108 m,
CoM speed 0). Terrain seed 2120316457; terrain is exactly flat (h = 0.000) under the whole body (start disk r = 0.45 m).
- Right after the 0.75 s settle: min contact dist -0.0415 m, 58 contacts; deepest are hfield tile `terr11` vs `g6`/`g7` (modules 6 and 7), plus
  `floor` vs `g7`. Module 7 CoM z = -0.012 m, module 6 +0.006 m (below the surface). Step 0 already ends the episode (glitch penalty).
- Cause: **start pose, not terrain geometry.** `start_pose_rad = 0.6` draws joints in +-0.6 rad (this one: 0.43 -0.47 -0.45 -0.59 -1.1 0.26 0.01).
  The root is placed at `z0 = 0.103` (valid for the straight lying body), so the curled tail starts *inside* the ground: CoM z of modules 7, 8 =
  -0.036, -0.033 at t = 0 (initial penetration 7.7 cm).
- A flat plane pushes any penetrating geom out (half-space): resolved in 40 ms, settled min dist -0.0003. A heightfield does not: MuJoCo's
  hull-vs-hfield collision keeps resolving against prism faces and the tail stays 4 cm in the ground (minDist -0.0766 -> -0.0415 after 0.75 s, still
  decaying 0.5 mm per 10 ms). Heights are irrelevant: an all-zero hfield with the same pose reproduces it (-0.0415, identical CoM z); terrain
  seed 5 at level 0.6 too. The terrain-generator and the guard are fine.
- Frequency: 1/100 gate episodes (reset min dist < -3 mm in 1/100 seeds, 10085); with terrain on during training, ~1 % of episodes start
  with a -300 glitch at step 1 (training noise, and a failed gate for any policy).
- Guard calibration: the 1 cm threshold is correct (flat-plane settle |dist| <= 0.2 mm; legit hfield contact <= 0.2 mm in the other 99 seeds).
  **Do not change the guard.**

## 2. Proposed minimal fix (not applied)
At reset in `PetEnv` (`env.py:~283`, the start-pose loop), when a terrain is present: after `sim.reset(q0)`, if `mj_forward` shows penetration, lift the
root by the depth: `qpos[2] += -min(contact.dist) + 2 mm`. Verified on seed 10085 (depth 0.0766 m): no contacts after the lift, settled
min dist -0.0001, all CoM z >= 0.024. Alternative: resample q0 when the settled min dist < -5 mm (uses the existing resample loop, keeps the
random stream identical for the 99 % of unaffected episodes, drops ~1 % of poses).
- Both only act when a terrain exists and penetration > 0 at t = 0, so flat/plane runs (terrain None) and the 99 episodes without initial
  penetration are bit-identical. Not applied now: editing `env.py` would change the evaluation of the B1 attempt 2 run in flight (subprocesses
  re-import it). Apply after attempt 2 / or as a default-off task param plus a test (reset pose with q0 = the 10085 values, hfield 0 level:
  assert settled min dist > -2 mm). Re-gating seed 10085 only needs the one episode, not the whole gate.
- Expected result: glitch rate 0/100 on this gate for the attempt-1 policy (the only glitch is this seed).

## 3. The 10 A peak episodes (6/100, limit 5 %)
`per_seed` rows carry no bus data, so the 100 episodes were replayed (seeds 10000-10099, same setup; peak = predicted bus current before
the clamp, `bus_peak_a`). Quantiles of per-episode peak: p50 6.9, p75 8.2, p90 9.5, p95 10.2, p99 11.9 A; 16 episodes > 9 A.
Over-10 A episodes: seeds 10006 (10.7), 10022 (10.2), 10047 (10.6), 10063 (10.2), 10069 (11.9), 10088 (13.1). Each has only 1-3 ticks over 10 A
(0.6-2.3 % of ticks).
- Time in episode: **4 of 6 peak at tick 3-4 (0.10-0.13 s)**, i.e. the start transient (random start pose -> first policy targets; body at 0.1 m
  from the origin on flat ground). The other two: 10063 tick 82 (2.7 s, 0.5 m out), 10069 tick 106 (3.5 s, 0.7 m, body on a 6 mm rise) plus
  ticks 3 and 68.
- Terrain features: none correlate (bodies are on terrain height 0.000-0.007 m at every peak; slopes/bumps/loose tiles are not under the body).
- **Control run on the same seeds with terrain_level 0: also 6/100 over 10 A** (p50 7.0, p90 9.0, p95 10.05). The 6 % is a property of the
  policy's current profile, not of the terrain; the first 4 ticks of the episode dominate.
- Implication: terrain features are not the lever. Levers: stronger `bus_peak_penalty` on the first ~10 ticks (attempt 2 already trains with
  a penalty), start-of-episode action ramp/smoothing in training, or a policy with lower initial-step demand. Gate threshold unchanged.
