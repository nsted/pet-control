# RL locomotion: plan and status

Approved plan: `~/.claude/plans/i-want-to-create-velvety-flamingo.md` (rev 2, includes the RL-expert audit). Stack: Gymnasium + SB3 PPO,
position-target actions at 20 Hz with fixed kp 0.8 / kd 0.035, MuJoCo native PD actuators with per-joint `forcerange` as the torque cap.

## Status (2026-11-04)
- [x] Phase 1: `petctl/sim/model_builder.py` - free-body (and hanging) model, ground plane, `PetSim` with delayed targets and native PD actuators.
      Tests: `tests/test_model_builder.py` (hanging mode matches `gravity_model.equilibrium` to 0.004 rad / 0.0035 N.m; free body settles
      in the bench posture, modules 0,2,4,6 on the floor and 1,3,5,7 raised 1.2 cm; cap limits actuator force).
- [x] Phase 1.5 feasibility script: `scripts/feasibility.py`.
- [ ] Phase 1.5 gate: NOT passed - see below.
- [ ] Phase 2/3/4 (RL env, tasks, policy scheme, MuJoCoBackend, hardware): not started; needs gymnasium + stable-baselines3 installs (ask first).

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
