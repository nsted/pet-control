# Phase 2 — model checks and speed (2026-11-26)

Branch `feat/autonomous-trainer`. Code: `petctl/sim/urdf_gen.py` (existing generator), `petctl/sim/skin.py` (new), `petctl/rl/obs.py` (`turn_obs`), `scripts/rl/bench_envs.py`.

## 1. URDFs for 10, 11, 12, 13 modules
Generated with `urdf_gen.variant_dir(n)` (the builder already did this for every n != 8; my Phase 1 note saying otherwise was wrong). Checked: zero-command 3 s settle is finite
(|q| <= 0.027 rad, |qdot| <= 0.008 rad/s), sim masses 2.750 / 3.025 / 3.300 / 3.575 kg, previews `data/urdf_variants/previews/n10_11_12_13.png` (head red, tail green, odd modules
orange, even blue). The generator's own tests (`tests/` urdf / n8-identity / tail-flip) pass.

### UNVERIFIED geometric / physical assumptions — owner to confirm in CAD or on the bench
1. **Odd-n tail orientation (the big one). CONFIRMED by the owner 2026-11-26: the tail is rolled 180 deg about the lengthwise axis for odd n (generator default `tail_flip` auto, unchanged).** Original note: For odd n the tail is turned 180 deg about the chain axis (`tail_flip` auto), derived from mating-patch coincidence. Confirm the physical
   tail module can be mounted in that orientation (and which orientation the real odd-length robot will use). Affects 11 and 13 only.
2. **Even-n tail position.** For even n the tail end sits ~0.2 cm off the pitch grid and the lying chain is tilted ~5 deg (frame from head->tail centres). Confirm the real even-length
   chain geometry and that the tilt is a frame artefact, not a real bend.
3. **Joint axes of new joints** (7..n-1) are extrapolated by the screw rule (+y, then minus the screw-rotated previous axis). Verified only by reproducing the owner's 8-module
   axes exactly. Confirm that the Rubik's-snake rule (neighbour axes 90 deg apart, 45 deg to the chain) holds at every added joint, i.e. no module in the extended chain is mirrored.
4. **Module masses:** every module 0.275 kg (uniform `MODULE_MASS_KG`); the CAD inertials say 0.27965 (middle) / 0.27607 (head, tail). Longer bodies add no wiring / battery / connector mass.
5. **Inertias and centre of mass of added modules** are copies of owner modules 5/6 (screw-moved); head/tail inertia identical.
6. **Collision geometry:** convex hulls of the CAD meshes (<= 200 vertices); adjacent hulls touch (0 mm), skip-one hulls keep 1.76 mm clearance for every n (same as the owner's 8).
   Real clearances with cables / fasteners / skin are unknown.
7. **Motor. CORRECTED by the owner 2026-11-26: the rebuilt robot uses the SteadyWin GIM3505-8 (8:1 planetary, CAN), NOT the CubeMars GL40 II** that CLAUDE.md, `petctl/motors/gl40.py`
   and `docs/plans/POWER_MANAGEMENT.md` describe. The sim's motor-dependent parameters were measured on / derived from the GL40 II and are therefore UNVERIFIED for the new motor:
   kp / kd defaults (0.8 / 0.035, MIT scale), joint coulomb friction (`COULOMB_NM`), armature (DR 1e-4..1e-3; an 8:1 reducer reflects rotor inertia x64), command delay (0.04-0.14 s),
   the bus-current model (13.3 A per (N.m)^2 per motor, measured at V_bus 14.7 V; the GIM3505-8 is a 24 V nominal part), the 24 W worst-case per motor, thermal limits, and the speed
   limit (retailer listings: 225-384 rpm after the reducer; slew limit in the sim is 6 rad/s). The 0.5 N.m torque cap stays (owner: correct). Retailer pages list rated torque ~0.65 N.m,
   rated power 29-37 W, 84 g motor; they disagree with each other, no official datasheet was found, no peak torque figure.
8. **Shared bus current model:** 13.3 A per (N.m)^2 per motor was measured on the 8-module robot (hanging tests); assumed to scale linearly in n with the same 10 A peak / 4 A mean
   limit shared by all motors. A longer body therefore has less current per joint on average.
9. **Multi-turn joints:** assumed physically possible without cable wind-up (slip ring or wireless joints). If not, the policy needs a turn limit after all.
10. **Skin friction** is unmeasured: `floor_friction` 0.4-1.5 (existing DR) is randomised in training; the skin is assumed isotropic (bare ABS). Anisotropy (1-2.5 lateral/forward, `skin.py`) is only a robustness check. Revisit if a fabric / silicone cover or printed texture is planned.

## 2. Multi-turn joint state in the observation
Finding: with unbounded joint actions the observed joint angles AND the commanded targets were wrapped to [-pi, pi) (`obs.py`), so two states 2 pi apart gave identical observations,
although the motor slews in unwrapped coordinates (it takes the long way to a target 2 pi away): the multi-turn state was hidden. Fix: task knob `turn_obs = 1` (needs
`action_unbounded = 1`) appends `turn_count(q)` and, in delta / hold modes, `turn_count(target)` (whole turns, clipped to +-3) as the last block `turns` (`ObsBuilder.blocks()`
includes it, so warm-start column mapping works). Default off, so existing runs are unchanged. Test: `tests/test_watchdog.py::test_turn_obs_makes_multiturn_state_visible`.
Phase 7 note: with a CPG action space the multi-turn quantity lives in the integrated bias / twist parameters, which are then part of the policy's own state and must be observed too.

## 3. Friction: randomisation and anisotropy
- Floor friction DR already existed (`DomainRand.floor_friction` 0.4-1.5); eval scenarios can now fix it (`task_params["floor_friction"]`, `["friction_aniso"]`, set by `evalsuite.task_params`).
- NEW anisotropic skin friction (`petctl/sim/skin.py`, `BuildParams.skin_aniso`, `DomainRand.friction_aniso`, default (1, 1) = isotropic, the default for all runs (owner decision 2026-11-26: ABS is isotropic); use (1.0, 2.5) only for occasional robustness checks).
  MuJoCo friction is isotropic, so the floor friction acts as the FORWARD (chain-axis) friction and the missing lateral share (r - 1) mu N is added as an extra Coulomb force per
  robot-vs-world contact along the lateral direction, with the chain axis fixed in each module's body frame (so it follows a rolling body). Per-contact force is capped so a body's
  contacts cannot reverse its slip within one hold interval (the uncapped force was unstable: explicit gain ~ 1000 N s/m against 0.27 kg modules).
- Validation (`tests/test_skin.py`, n = 10): isotropic slide matches v^2 / (2 mu g); with r = 2.5 the lateral slide is 0.35-0.37 of isotropic at 0.2-0.4 m/s (ideal 0.40), forward unchanged.
  Limits: stride 3 overshoots (0.25), stride 4 loses the effect, at 0.7 m/s and for odd n the lateral test tumbles instead of sliding, so it only checks stability + forward there.
  The anisotropy is an approximation of ventral-scale skins, not a measured model.

## 4. Throughput (`scripts/rl/bench_envs.py`, 10-core M-series, SubprocVecEnv, random actions, DR 0.3, n_modules 11, env steps/s, 600 vector steps each)
| n_envs | 4 | 6 | 8 | 10 | 12 |
|---|---|---|---|---|---|
| isotropic | 4631 | 5195 | 5412 | 5927 | **6145** |
| anisotropic skin (vectorised, stride 2) | 2906 | 2848 | 3261 | **3478** | 3382 |
Anisotropy costs ~40-45 %. (A first, un-vectorised skin implementation cost 70 %.)
**Use n_envs = 10** (flat optimum 10-12 on 10 cores; 10 is best with anisotropy on and within 4 % of the best without). PPO updates run in the main process, so real training throughput is lower than these simulator-only numbers (the brief's
~2k steps/s is the training figure); budget in steps from the measured training rate of each run. Idea not done yet: `mujoco.rollout` / process pools for black-box gait search (Phase 4).
