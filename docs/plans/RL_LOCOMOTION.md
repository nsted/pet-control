# RL locomotion: plan and status

Approved plan: `~/.claude/plans/i-want-to-create-velvety-flamingo.md` (rev 2, includes the RL-expert audit). Stack: Gymnasium + SB3 PPO,
position-target actions at 20 Hz with fixed kp 0.8 / kd 0.035, MuJoCo native PD actuators with per-joint `forcerange` as the torque cap.

## Status (2026-11-04)
- [x] Phase 1: `petctl/sim/model_builder.py` - free-body (and hanging) model, ground plane, `PetSim` with delayed targets and native PD actuators.
      Tests: `tests/test_model_builder.py` (hanging mode matches `gravity_model.equilibrium` to 0.004 rad / 0.0035 N.m; free body settles
      in the bench posture, modules 0,2,4,6 on the floor and 1,3,5,7 raised 1.2 cm; cap limits actuator force).
- [x] Phase 1.5 feasibility script: `scripts/feasibility.py`.
- [~] Phase 1.5 gate: PARTIALLY passed with the Fusion URDF meshes (2026-11-04) - see the RESULT section at the end.
- [x] Phase 2 template (installed with user approval: gymnasium 1.4.0, stable-baselines3 2.9.2, tensorboard 2.21.0; extra `rl` in pyproject):
      `petctl/rl/{config,obs,domain_rand,env,train,cli}.py`, `petctl/rl/tasks/{base,locomote}.py`; `petctl rl train|eval`; tests `tests/test_rl_env.py`
      (Gymnasium env_checker, determinism, torque-cap respect, obs builder). Smoke run: 8k steps, 4 workers, ~6 s; env ~3.7k policy steps/s/core.
      Still to do: asymmetric critic (privileged info is already in `info["privileged"]`), feedback-pipeline realism (48 Hz hold, dropped frames),
      reward weights tuning (random-action terms are dominated by action-rate/joint-acc penalties), `RLPolicyMotion` + `MuJoCoBackend` + slew option, hardware stages.
- [ ] Phase 3/4: blocked on the Phase 1.5 gate (collision geometry / friction).

## Phase 1.5 finding: joints 1-6 cannot move the lying body in the current model
From the CAD axes: every joint axis lies in the forward/up plane at 45 deg to the chain, alternating sign, and the module centres zig-zag +-0.6 cm
(so modules 0,2,4,6 are low and 1,3,5,7 high, matching the bench posture). With a 7 cm wide flat box per module (my first collision shape) and
kp 0.8 the model barely turns joints 1-6: a +0.3 rad command reaches 0.00-0.12 rad even with torque cap 5 N.m (kp*error is only 0.24 N.m), and
still only 0.07-0.12 rad at floor friction mu = 0.05, so it is NOT only friction. Only the tail (joint 7) moves freely.
Likely cause: a flat 7 cm footprint on a 45-degree axis pushes one side of the body into the floor (z displacement = 0.71 * 3.5 cm * theta),
so the box locks against the plane. The real triangular prism probably clears the floor; so the verdict depends on the true collision shape/orientation
(Fusion mesh) and the real floor friction. Do not train on this model until that is resolved.

Needed from the user (nothing has been run on hardware):
1. Fusion export of the assembly (ACDC4Robot MJCF/URDF, or the existing script's JSON) plus a note of which face of each prism touches the floor.
2. Weights of the head, a middle module and the tail.
3. A drag test on the carpet and on the bench (spring scale, robot lying in the bench posture) for the friction coefficient.
4. Optionally (hand ready, ask first): does the real robot lying on the floor actually reach +-0.3 rad at joints 1-6? That one observation validates or refutes the model.


## RESULT 2026-11-04: Fusion URDF export (`/Users/nsted/Documents/PET/body/URDF/PET_description`) used for collision shape
- `model_builder.load_urdf()` takes the module SHAPES from `meshes/*.stl` (convex hulls, <=200 vertices each (`BuildParams.max_hull_verts`, was 48; 400 is ~50% slower than 48, gait result unchanged), module-vs-module contact off, floor only).
  `BuildParams(urdf_dir=...)` switches it on; the box model remains the default. Needs scipy (added to the `sim` and `rl` extras).
- The URDF's own JOINT frames are not usable: its joints form a straight chain along z (pitch 50.7 mm) while its meshes form the real staircase in
  the CAD world frame at q=0, so they disagree. Instead the validated `gravity_model` joint axes are rotated into the mesh frame (rigid fit of
  its mass centres to the hull centroids, residual 11-24 mm), where they come out as +-y / +-z alternating (still the 45-degree skew axes). Anchors lie on those axis
  lines nearest the contact between neighbouring meshes. UNVALIDATED: anchor error is probably 1-2 cm; check with the real robot (see below).
- Mass/COM in the URDF (0.276-0.280 kg per module) agree with the assumed 0.275 kg, but its inertial origins do not match its meshes, so masses stay at
  0.275 kg with the hull centroids as COM until the modules are weighed.
- Settling: with the meshes the robot rests on modules 0,2,4,6 (bench posture), as predicted by the gravity model.
- Single-joint commands still do NOT move joints 1-6 (+-0.3 rad, either cap), so the box shape was not the only cause. Physically expected: each axis is 45 degrees to
  the chain, so one joint alone must lift everything downstream (about 1 N.m for the full chain against a 0.164 N.m cap). Adjacent joints in opposition can bend the body
  in plane: the zig-zag pose 0.5 rad reaches 0.42/-0.46 rad on joints 1,2 at cap 0.164.
- `scripts/serpenoid_baseline.py` (travelling wave q_k = +-A sin(2 pi f t - k phi), 10 s, mu 1.0), 80 runs:
  * cap 0.164 N.m: the body moves, best forward +3.8 cm (A 0.5, 0.5 Hz, phi +2.1, plain signs), best sideways +20.6 cm (A 0.5, 1.0 Hz, phi +2.1). Torque saturates 12-40 % of the time.
  * cap 0.05 N.m: nothing moves (< 0.1 cm).
  * Verdict: locomotion is mechanically possible only near the peak cap and the hand-tuned open-loop gaits are slow (~0.4 cm/s forward, 2 cm/s sideways).
    This is the baseline for the RL task; a sideways/sidewinding gait is probably the natural solution, so consider rewarding progress along ANY horizontal
    direction (or a chosen heading) rather than strictly "forward" if the policy discovers it.
- Remaining unknowns, all needing the user: module weights, carpet and bench friction (drag test), and one supervised floor test of whether the real robot
  moves joints 1-6 as the model says (zig-zag pose reaches ~0.4 rad on joints 1-2, single joints do not). That test is the real validation of this model.

## FIX 2026-11-04: lying-model joint axes were 90 deg off about x
`load_urdf` rotated the gravity-model axes into the mesh frame by a rigid fit that came out 90 deg wrong: the sim joints turned about z where the mating-face normal (motor cylinder axis) is y, and vice versa, so they bent perpendicular to the real revolute axis. Axes are now the SVD normal of the contact patch between B_{k-1} and B_k (alternating +y / -z in the robot-CAD frame), sign kept from the old convention rotated 90 deg about x (agrees with the URDF axis signs), anchor = patch centroid.
After the fix: single-joint 0.5 rad commands still do not move joints 1-6 at cap 0.164 (same 45-deg-to-chain argument); serpenoid A 0.5, 0.5 Hz, phi 2.1 plain now gives +13.6 cm forward in 10 s (was +3.8). Re-run the full sweep before trusting earlier baselines. Joint sign vs the real robot (curl right: odd +, even -) is not yet checked against the new axes.

## First end-to-end training test (2026-11-04) — `walk_straight`, PPO, 2M steps

A test of the pipeline, not a result to rely on. Artefacts: `models/rl/walk_urdf_2/` (inference-only `policy.npz`), video `docs/media/walk_urdf_2.mp4`.

- Run it with no training: `petctl rl run models/rl/walk_urdf_2` (or `mjpython scripts/view_policy.py models/rl/walk_urdf_2`; record with `scripts/record_policy.py`).
- 15 s episodes, mesh model. Nominal plant: 44 cm, cross-track 0.6 cm, 1.63 energy/m. Full domain randomisation: median 33 cm (IQR 23-41). Hand-built serpenoid wave on the same model: 17.8 cm, 2.94 energy/m.
- Lessons: (1) training on the default box model learns to stand still (the same wave moves 0.7 cm there vs 17.8 cm on the mesh model), so `EnvConfig.urdf_dir` / `petctl rl train --urdf` now defaults to the mesh model; (2) reward `progress` is a velocity (~1 cm/s), so its weight is 50 and the penalties are scaled down; (3) `sim_dt=0.004` is 1.75x faster with the same gait; (4) pin BLAS/OpenMP threads to 1 in workers (3.7x faster, done in `train()`).
- Not validated: joint signs on the real robot, weighed module masses, floor friction. Reward plateaued at ~250 after ~600k steps; the next iteration should change the reward/curriculum rather than just train longer.

## `walk_sideways` — translate perpendicular to the body's length (2026-11-04)

Same pipeline, task `walk_sideways` (`petctl rl train --task walk_sideways`). Artefacts: `models/rl/walk_sideways_1/`, video `docs/media/side_1.mp4`.

- Reward gotcha: a windowed |speed| progress term was hacked (rocking side to side: reward +327, only 4 cm net in 15 s). `progress` is now the growth of net |sideways| displacement from the start, which telescopes to the final net distance, so rocking earns nothing and either side counts.
- Result (15 s, mesh model, one 2M-step PPO run): nominal plant 82 cm sideways, straightness 0.94, 0.98 energy/m, 16 cm drift along the body (~11 deg off pure side-step). Full randomisation: median 44 cm (IQR 0.9-68), 1.05 energy/m. The weak quarter of randomised plants (probably low torque caps) is the obvious next target.
- Compared with `walk_straight` (44 cm, 1.63 energy/m), sideways is faster and cheaper on this body. The open-loop serpenoid sweep found almost no sideways motion (<=4.4 cm), so this gait is not a simple travelling wave.

## `reach_up` — raise the highest point of the body as high as possible (2026-11-04)

Task `reach_up`; artefacts `models/rl/reach_2/`, video `docs/media/reach_2.mp4`. Train: `petctl rl train --task reach_up --steps 5000000 --action-scale 3.1416 --episode-s 10 --dr-scale 0.3 --cap-nm 0.5`.

- Reward = growth of the running-max height of the highest collision-mesh vertex over all 8 modules (telescopes to the episode peak; no hold requirement). A peak only counts while no module origin is below 5 cm and contact penetration is under 5 mm (a naive height score was exploited by clipping the tail through the floor, and a head-site score by rolling the head).
- Result at cap 0.5 N.m, targets +-pi rad, no flip termination: peak 19.7 cm above the flat start (highest point ~27 cm above the floor), head end upright, rear modules on the floor; it cycles (reach ~16 cm at 2 s, relax, reach ~19 cm at 9 s). Brief airborne frames (20/200) during transitions, not at the peak: not checked for plausibility.
- Correction: an earlier static pose search found only ~5 cm and I concluded standing was infeasible. That was wrong: it bounded joints at +-2 rad, used a lower cap and a weak search. The geometry allows standing; the limits were in the setup (action range +-0.5 rad in the default env, randomised cap 0.05-0.164 N.m, tail-IMU flip termination).
- 0.5 N.m is far above the power-limited 0.164 N.m (2 A peak/motor); several saturated joints would exceed the 4 A bus budget. Treat as an upper bound, not a hardware gait. Next: find the lowest cap that still stands, and decide whether to require a hold.
