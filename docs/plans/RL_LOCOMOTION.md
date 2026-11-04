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
