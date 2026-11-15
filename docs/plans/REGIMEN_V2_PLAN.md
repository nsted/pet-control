# REGIMEN V2 - from-scratch training plan: rough terrain + obstacle field -> target -> hug

Status: PLAN ONLY, revision 2 (2026-11-13, after the owner's answers). Nothing in this document has been run. Director: Fable (lead RL).
Executors: Sonnet subagents. Supersedes the stage chain in `docs/plans/CURRICULUM_LOG.md` / `RL_LOCOMOTION.md` for the new objective; old runs stay as baselines.

## 0. Objective and binding constraints (owner, verbatim intent + answers)

Train PET in simulation, from scratch, to cross rough terrain and an obstacle field **as fast as possible** to a target; once **any part** of the body
touches the target (a heavy, immovable post, ~10 cm diameter) it must **coil round it and hug** it. Any gait, including rolling; helical hugs allowed.

Binding:
1. Body length is a sim-only design study, **8..15 modules** (head + tail + up to 13 middle). Real robot today: 8 modules / 7 joints. Torque cap
   **0.5 N.m** per joint (always via `petctl/config.py` / `stages.REAL_CAP_NM`; never hardcoded). **Current budget per robot (not per motor): mean <= 4 A,
   spikes <= 10 A.**
2. Deployed policy senses only: **the relative goal vector (delta: bearing + distance, measured from the head, in the head frame)** and nothing else
   about the target; IMU (gravity direction, gyro; no absolute yaw); joint positions; previous actions; the **motor MIT reply** (torque, commanded-minus-
   measured position, with memory); and, **for the target/post only, the capacitive face sensors** (3 faces per module: left, right, middle; touch +
   pressure pads), which the post triggers exactly like a human touch. **Obstacles and terrain are felt via motor feedback only** (they never trigger
   the capacitive sensors). No vision, no terrain/obstacle map, no absolute position, no post geometry. Privileged information (obstacle map, terrain,
   post pose, true contacts) is allowed **only** for the critic, reward shaping and evaluation.
3. **Hug success is defined by face contact, not angle:** a sufficient number of chain-contiguous modules whose faces are in contact with the post,
   sustained for a hold time (sec 5.3). A wrap angle may be reported as an eval diagnostic only; it appears in no reward, gate criterion or observation.
4. Terrain: bumps up to ~1 body height (6 cm), slopes <= 15 deg, loose-texture patches; ramped from flat; the final gate mixes terrain and the field.
5. Flipping/rolling is never a failure. Reward fast arrival. Every video captions what is being trained. Gates frozen (changed only for demonstrated
   mis-calibration, with data); gate metrics independent of the training reward; >= 100 held-out episodes with confidence intervals; every sensing
   claim needs a `touch_off`-style ablation showing a real drop (the hug adds `capacitive_off`). No GitHub issue actions. Robot is never moved; the
   plan proceeds **without** a hardware reply-frame capture (sec 6, D9).
6. Resolved decisions: D1 head-frame delta YES; D2 head-only delta YES; D3 `sb3-contrib` may be installed if an LSTM is needed (no further ask);
   D5 helical hug YES; D6 post heavy/immovable; D7 replaced by the per-robot current budget; D8 gate thresholds frozen as proposed in sec 4.2.

## 1. Review of the existing scheme (what worked, what failed, root causes)

Sources: `CURRICULUM_LOG.md`, `RL_LOCOMOTION.md`, `petctl/rl/*`, `data/rl/curriculum_fast/audit/*.out`.

### 1.1 What worked (keep)
- **Plant and pipeline.** Mesh collision model (`petctl/sim/model_builder.py:109-149, 166-238`), native PD actuators with `forcerange` as the torque cap,
  command delay, DR (`petctl/rl/domain_rand.py`), shared `ObsBuilder` for train and deploy (`petctl/rl/obs.py:44-98`), inference-only `policy.npz`
  (`petctl/rl/policy.py:21-79`). Throughput: ~2.7k steps/s on 8 envs for flat stages (s7a: 3M steps in 1112 s), ~300-700 steps/s with obstacle layouts.
- **Gait discovery by flat PPO.** Directed locomotion emerged from scratch at 0.5 N.m (~9.5 cm/s with `action_scale 1.0 / ema 0.3`); any-bearing, random
  start, retargeting all passed (S1-S5). Lateral/reverse competence exists in S5 (audit: +-0.6-1.0 m lateral, -0.4..-0.6 m reverse in 8 s) but is slow
  (~0.1-0.2 m/s) and was lost in the S8 line (S8 iter_10 is a poor lateral mover).
- **Rolling can be learned** when it is the only thing rewarded and the reward is a ratchet (S7a: 11.9 tail / 11.5 head turns, two independent
  measurements agree). Lesson: *ratchet rewards (pay only for growth of a running peak) defeat oscillation exploits.*
- **Contact IS decodable from the motor reply** (held-out AUC 0.905 on the 84-d touch block; wall test AUC 0.93/0.94 for torque/error). Signature is
  **reduced motion/tracking**, not torque spikes - a body pressed on a post looks like "commanded motion not happening".
- **Shortest-path shaping with a privileged distance field** (`goal_obstacles.py:48-101`) first produced real contact navigation (S8 iter_09/10, s8c
  0.75 robust). Stall termination that *charges the remaining time penalty* fixed the "giving up pays" exploit (`goal_obstacles.py:269-271`).
- **Speed reward structure** (`success_bonus 150`, `speed_gain 3`, `time_penalty 0.5/s`) made arrival time matter; keep.
- **Process hygiene that caught exploits**: second independent measurement for every gate metric (roll_head_turns vs roll_turns), the obstacle "leak
  test" (move obstacles 100 m away, compare trajectories), the touch_off ablation, caption burn-in (`video.py:33-41`).

### 1.2 What failed and why
| Symptom | Root cause | Consequence for v2 |
|---|---|---|
| S7 roll gate "passed" with 4.3 turns while the body never tipped | reward and gate measured the same exploitable quantity (gyro-x integral) | every gate metric gets a second, independent ground-truth measurement; nothing is measured from integrated rates |
| s8a "avoidance" was no avoidance (identical trajectories with obstacles removed) | layout blocked the straight line, not the gait's actual arc; policy blind by construction | a layout is valid only if a **blind** policy fails it (measured), and gates require contact and a touch_off drop |
| explore bonus made wandering outearn arriving (S8 iter_03 collapse) | uncapped per-cell bonus | no exploration bonuses in v2; search is driven by path-shaped progress + asymmetric critic |
| flat obstacle policies score the same with touch zeroed (s8c 0.67/0.75 vs 0.72/0.70; barrier 0.25-0.30 either way) | the field gate is solvable ~70% blind; the barrier needs 20-60 s of lateral search, beyond a 5 s horizon (gamma 0.99) with a value function that cannot see the wall | gamma 0.997, **asymmetric critic that sees the obstacle map**, barrier as the discriminating gate |
| selector over bearing offsets learnt nothing in 300k steps (0.20 = always-0) | 1 Hz discrete offsets on a slow lateral primitive, sparse reward, no privileged value | hierarchy dropped; flat policy with long memory + privileged critic |
| wall stage S6: robot walked into the wall and buckled/flipped | straight-line progress gave nothing at the wall; two ticks of contact history | path-shaped shaping and 0.5/3/10/30 s EMAs already exist (`obs.py:18-19, 87-97`); keep |
| gates on 20-40 episodes, marginal passes (0.65 vs 0.60) | no confidence intervals | n >= 100, Wilson 95% CI, pass only if the lower bound clears the threshold |
| PPO collapse when lowering entropy on a continued run (S8 iter_05) | continuation with new hyper-parameters | fixed recipe per stage (sec 6); continuations keep ent/lr; one change per iteration |
| obstacle layouts cost 2.6 s each; 3 jobs shared 10 cores | expensive rejection sampling, oversubscription | layout cache; one training job at a time |
| folder move killed a run | absolute mesh paths | `PET_URDF_DIR` exists (`config.py:13`); no moves during runs |

### 1.3 Structural facts that shape v2 (verified in code)
- Joint axes are all **45 deg to the chain**, alternating sign (`gravity_model.AX`; verified: along-chain component -0.707 for every joint). Planar
  bending needs **adjacent joints in opposition** (+theta, -theta): ~0.707 rad of floor-plane curvature per rad of joint angle, zero net twist. The
  module centres of a body wrapped round a 10 cm post sit at ~8.5 cm radius (circumference ~0.53 m = ~9 modules at the 5.92 cm pitch), so **five
  chain-contiguous modules with faces on the post correspond to ~200 deg of the circumference** and demand ~0.65-0.95 rad on every joint of an
  8-module body; a 12-15 module body does it with ~0.5 rad per joint and spare modules. Helical stacking (climbing the post) uses the twist components
  and is allowed. This drives the body-length study (sec 3) and must be checked kinematically before any RL.
- `N_JOINTS = 7` is a module constant used everywhere (`config.py:10`, `obs.py`, `env.py`, `policy.py`, `model_builder.py:28-29`); the policy is an MLP
  (`train.py:97`) and is **not length-agnostic**. The URDF loader reads exactly `base_link` + `module_1..7` (`model_builder.py:122`); the tail is already
  extrapolated by repeating the pitch (`model_builder.py:77`).
- Goal observation today = unit vector head->goal **in the tail-IMU frame** + distance clipped to 2 m (`goal_reach.py:156-160`); v2 moves it to the
  head frame (D1).
- Bus-current model in the reward: `bus = 0.6 + 13.3 * sum(tau^2)` A over all motors, soft penalty above 4 A (`locomote.py:54, 65`, `config.py:14-15`).
  At the 0.5 N.m cap **seven saturated motors draw ~24 A**: the 10 A spike budget allows at most ~0.7 N.m^2 of summed squared torque (e.g. 2-3 motors at
  the cap, or all 7 at 0.32 N.m), and the 4 A mean allows RMS ~0.19 N.m per motor for 7 motors, ~0.14 N.m for 14. **The current budget, not the torque
  cap, is the binding actuation limit, and it tightens with module count** (sec 5.5).
- Capacitive sensors: 3 faces per module with touch + pressure pads; the odd-module left/right swap and pad-order reversals are corrected in
  `RobotBackend` and transparent above (CLAUDE.md). Body-frame convention (memory): front = head, top = the even modules' middle-pad side, left/right
  egocentric. The sim sensor model therefore emits the **already-corrected** body-frame ordering (sec 5.4).
- Default `DomainRand.cap_nm = (0.05, 0.164)` (`config.py:27`) is stale; the runner overrides it to 0.5 (`runner.py:52`). v2 sets the default from `REAL_CAP_NM`.
- Gate (`gate.py:48-64`): 40 episodes, no CI, no ablation. Extended in sec 4.1. `allow_flip` defaults on (`goal_reach.py:58`). `PetSim` is rebuilt
  at every reset (`env.py:89`), so terrain, post and layouts can be sampled per episode.

## 2. Observation / action design for v2 (deployable channels only)

**Goal channel (delta only, D1/D2).** Measured from the **head site**, expressed as a **3-D unit vector in the head frame** (needed because the robot may
roll; a 2-D yaw bearing is undefined upside down) plus two distances: `d/1 m` clipped at 2 (far field) and `min(d, 0.3)/0.3` (near field, centimetre
resolution after `VecNormalize`). Trained with **delta DR**: bias 0-5 cm, noise 1-2 cm, delay 0-0.15 s, so the policy cannot rely on the delta alone for
the last 10 cm and is robust to the real localiser.

**What the delta does and does not tell the policy at the post:**
- Head pressed on the post: d ~ r_post + half-width = **~0.08-0.09 m, never 0**. "Arrived" = near-field d in [0.06, 0.15] AND (capacitive face contact
  on head faces OR blocked-motion signature in the motor reply). The near-field channel resolves it; the face sensors confirm it unambiguously.
- Tail or middle module touches first: the delta still points head->post (0.2-0.4 m), the capacitive block says **which module and which face** is
  touching, so the policy knows where the post is relative to the whole body and can curl toward it. No tail delta is added.
- While wrapping, the head-frame delta is nearly **constant** (post stays "beside the head" at ~8 cm) and carries no progress information; progress is
  observable through **the capacitive coverage itself** (how many modules/faces report contact) and through proprioception (joint angles; gyro-z).
  This is why the hug is learnable under the delta-only rule.
- Hard case: localiser bias of a few cm is a large fraction of d at the post - hence the delta DR and the bias-perturbation eval (sec 4.3).

**Proprioception** as today (`obs.py:85`): gravity in IMU frame, gyro, joint pos, windowed velocity, cap, 4 past actions.
**Touch block (motor reply)** as today (`obs.py:87-97`: torque/cap and tracking error, current + previous tick, EMAs 0.5/3 s) **plus the 10/30 s EMAs
always on**. Reply-frame realism: assumed noise 0.02 N.m, 12-bit quantisation (`env.py:27-28`), **plus** DR of the noise (x0.5-2), reply delay 1-2 ticks,
and 5% dropped frames (hold-last), since no hardware capture is planned (D9).
**Capacitive block (new, post only)** per module k and face f in (left, right, middle), body-frame corrected: `touch[k,f]` in {0,1}, `pressure[k,f]`
in [0,1], and a 0.5 s EMA of touch (contact memory): `9 * n_modules` dims (72 for 8 modules), appended **after** the touch block so warm starts
zero-pad it. Present in every stage from A1 (reads all-zero until a post exists). Pressure is randomly zeroed in 30% of training episodes (FSRs may be
deprecated per the owner's earlier note) so the policy never depends on it. Ablation knob `capacitive_off` (block zeroed). Model in sec 5.4.
Obs size for 8 modules: 55 proprio + 5 goal + 84 touch + 72 capacitive = 216.

**Action** as today: `n_joints` targets in [-1, 1] -> +-1.0 rad about zero, EMA 0.3, slew 0.35 rad/step (`env.py:121-125`). Unchanged for sim-to-real.

**Privileged (critic/reward/eval only):** plant params (already in `info["privileged"]`), obstacle distance-field value/gradient at head and CoM,
terrain height/slope under the body, post pose, per-module distance to the post surface, **ground-truth face contacts and normal forces**, bus current.

## 3. Body-length study (sim-only design study, 8..15 modules)

**Goal:** choose L for the regimen and give the owner the evidence to decide whether to build a longer body. The current budget (4 A mean per robot)
makes this non-trivial: more modules = more mass to move and more motors sharing the same current, so speed may peak at an intermediate length while
the hug favours longer bodies.

**Code changes (Sonnet, with tests):**
1. `EnvConfig.n_modules` (default 8); `N_JOINTS` becomes `cfg.n_joints = n_modules - 1` everywhere (`config.py`, `obs.py`, `env.py`, `policy.py`,
   `tasks/*`, `video.py`). `policy.npz` stores `n_joints`; `GaitPolicy` refuses a mismatched config.
2. `BuildParams.n_modules`; `build_mjcf` repeats the **middle-module pattern**: hull of an existing middle module pair (`module_3_1`/`module_4_1`),
   translated by the measured 5.92 cm pitch with the alternating +-45 deg axis and +-0.6 cm zig-zag; head and tail hulls unchanged; 0.275 kg each.
   Tests: 8-module MJCF bit-identical to today's; 15-module build settles on the even modules; serpenoid baseline moves; cap respected.
3. Reward normalisation by length: `energy`, `action_rate`, `joint_acc`, `saturation` divided by `n_joints/7`; the **current model stays physical**
   (sec 5.5) because the budget is per robot.
4. **Static hug-feasibility search (no RL):** for each L in 8..15, optimise static joint targets (DE, as `scripts/rl/static_pose_search.py`) to maximise
   the number of chain-contiguous modules with ground-truth face contact on a 10 cm post in the settled sim (self-collision on, penetration < 5 mm,
   bus current <= 4 A at the held pose). Output per L: max contiguous hugging modules, joint angles needed, holding current. Cheap (~1 h total) and may
   already exclude the short lengths.

**Sweep protocol (coarse, then refine; revised after M1):** M1 showed the static limit is geometric (pitch vs post radius), not current or length
(bus 0.9-2.6 A; 14-15 modules no better than 8), so the long end is dropped: RL at **L in {8, 10, 12}** (3 tasks x 2 seeds x 2M steps each), then
**refine at best +-1** (2 more lengths). 13-15 only if 12 is clearly best. Same reward weights (normalised), DR, action mapping, seeds, episode lengths, step budget; 100-episode gates. Tasks:
(i) flat goal reach any bearing 1.5 m (median time-to-goal, energy/m, mean/peak bus current); (ii) terrain reach at `terrain_scale 1.0`;
(iii) hug-from-contact (sec 4.2 D1 definition). Decision rule: the shortest L that is **statically comfortable** (>= 6 contiguous hugging modules with
<= 0.75 rad per joint and <= 4 A holding) **and** within 20% of the fastest L on flat and terrain time-to-goal **within the current budget**. Budget:
5 lengths x 3 tasks x 2 seeds x 2M = 60M steps (~8-12 h compute). Fable proposes L; the owner approves before Stage A.

## 4. Stages, frozen gates, budgets

### 4.1 Gate upgrades (tooling, before any training)
- `Gate.episodes` default 100; success with **Wilson 95% CI**; pass iff **CI lower bound >= threshold**.
- `Gate.ablations`: list of `(name, task_param overrides)` run on the same seeds; paired drop with bootstrap CI; a sensing claim passes only if the
  **lower CI bound of the drop >= the stated minimum**. Ablation knobs: `touch_off` (motor reply), `imu_off` (gravity + gyro), `capacitive_off`, `delta_bias_m`.
- `Gate.blind_baseline`: for obstacle gates the straight-walker (Stage A policy) must score below a ceiling on the gate layout, else the gate is void.
- **Current budget in every gate**: median episode mean bus current <= 4 A **and** fraction of episodes with any 50 ms-window peak > 10 A <= 0.05
  (`bus_mean_a`, `bus_peak_a` metrics, computed from the physical model in sec 5.5, independent of the reward's soft penalty).
- Second-measurement rule: every pass metric has a ground-truth twin (hug: sensor-model coverage vs ground-truth contacts; plus total normal force on
  the post and the diagnostic wrap angle, reported only).
- Glitch rate 0 (existing) with launch/penetration guards (sec 5.2).
- Videos: 4 per gate (2 best, 2 worst by time), captioned (existing `record_video`), plus a **contact sheet PNG** (16 frames) per video for Fable's review.

### 4.2 Stage chain (one policy, warm-started down the chain; terms in sec 5-6). Thresholds FROZEN (D8).
Budgets are policy steps; ~2.7k steps/s flat, ~0.5-1k steps/s with layouts/terrain, one job at a time on 8 envs. Robust condition DR 0.6, n = 100, CI lower bound.

| # | Stage | Trains | Frozen gate (all gates also require the current budget and glitch 0) | Budget |
|---|---|---|---|---|
| S0 | body-length study | sec 3 | decision, not a gate | ~60M |
| A1 | `flat_nav` | goal 0.8-2.0 m, any bearing, random start pose, occasional retarget; speed reward | success (head within 5 cm) >= 0.80; median t_goal reported | 3M |
| A2 | `flat_omni` | short goals 0.3-0.8 m at any bearing with a time limit that forces lateral/reverse moves | success >= 0.70; lateral displacement >= 0.4 m in 8 s for +-90 deg goals | 2M |
| B1 | `terrain_easy` | A2 + heightfield `terrain_scale` 0.3-0.6, friction tiles | success >= 0.75 at scale 0.6 | 3M |
| B2 | `terrain_full` | scale 0.6-1.0 (bumps 6 cm, slopes 15 deg, loose patches), goals 1.5-2.5 m | success >= 0.70 at scale 1.0; **imu_off drop >= 0.15** | 4M |
| C1 | `barrier_touch` | flat; barrier +-1.1-1.6 m wide, one 0.45-0.6 m gap (`goal_obstacles.py:131-144`); path-shaped reward; asymmetric critic | success >= 0.60 (stall 45 s); blind baseline <= 0.30; **touch_off drop >= 0.20**; median contact_frac >= 0.02 | 6M |
| C2 | `field_touch` | layered field (18-28 obstacles, radius 3 m) + barrier mixture | success >= 0.65; touch_off drop >= 0.15; blind baseline <= 0.45 | 4M |
| D1 | `hug_from_contact` | start with a random body part 0-0.25 m from the post, flat; post triggers the capacitive block | **hug held** (sec 5.3) in >= 0.75; penetration-free; **capacitive_off drop >= 0.15**; touch_off drop reported; delta bias 8 cm: success >= 0.60 | 4M |
| D2 | `approach_hug` | post 0.5-1.5 m away, flat; nav then coil | hug held >= 0.70; median time-to-first-contact and time-to-hug reported | 4M |
| E | `full_course` | terrain scale 0.5-1.0 + field + post | hug held >= 0.60; t_contact median reported; touch_off, imu_off and capacitive_off drops >= 0.15 each | 8M |

Order: locomotion (A) -> terrain/IMU robustness (B) -> contact navigation (C, historically hardest; benefits from terrain-hardened gaits) -> hug (D)
-> integration (E). If S0 shows the hug dominates the length decision, D1 is run early on candidate lengths (task iii of sec 3). Training distributions
are always a superset of the gate's. Each stage: at most 3 knob iterations, then a design change by Fable, then owner escalation (sec 7).

### 4.3 Independent evaluation suite (on every passing policy)
- **Leak test**: obstacles/terrain moved away after reset -> trajectories must diverge after first contact. For the hug: post moved 1 m after first
  contact -> the policy must stop coiling (proves it reacts to face contact, not to a memorised motion).
- **Delta-bias perturbation**: goal reported 0, 4, 8 cm off its true position.
- **Blind baseline**, **touch_off**, **imu_off**, **capacitive_off** on identical seeds; **pressure_off** (touch kept) reported for the hug.
- **Speed and power table**: median/IQR time-to-contact and time-to-hug, energy per metre, mean and peak bus current per condition.

## 5. Environment work: terrain, obstacles, post, hug metric, sensors, power

### 5.1 Terrain (MuJoCo heightfield)
- One `<hfield>` per episode, 8 x 8 m, 2.5 cm cells (321 x 321), data written to `model.hfield_data` after compile. Heights = smooth Gaussian bumps
  (amplitude <= 0.06 m x `terrain_scale`, radius 0.08-0.30 m) + 1-3 planar slope patches (<= 15 deg x scale, tanh-blended) + roughness <= 1 cm in
  "loose" patches; a flat disk (r 0.45 m) at the start; `z0` from the terrain under the body.
- **Friction patches**: 3 x 3 grid of hfield geoms sharing edge heights, each with its own friction (0.3-1.5; loose tiles 0.2-0.4 + roughness).
  Granular media are not modelled (R5).
- Obstacles and the post sit on the terrain; the distance-field planner ignores terrain. Curriculum knob `terrain_scale` ranges; gates fix 1.0.
- Tests: hfield build, flat start disk, obstacle z placement, friction per tile, seed determinism.

### 5.2 Obstacles, post, guards
- Reuse `GoalObstacles` layouts and `DistanceField` (`goal_obstacles.py:48-219`) with a **layout cache** (2000 layouts per stage on disk, drawn by seed).
- **Post**: cylinder geom welded to the world, radius 0.05 m, height 0.4 m, friction as floor; goal point = post axis on the floor; no obstacle within
  0.5 m of it. The policy sees it only through the delta, the motor reply and the capacitive block.
- **Glitch/launch guard** (episode counts against the gate): CoM speed > 3 m/s (existing, `goal_reach.py:26`), any module z > 0.5 m above local terrain,
  any contact penetration > 1 cm.

### 5.3 Hug: success metric and reward (face PROXIMITY contact; no angles)
**Re-calibration after M1 (2026-11-13, `docs/plans/M1_HUG_FEASIBILITY.md`): demonstrated mis-calibration of the metric, not of a threshold.** With rigid
convex hulls a flat 6 cm face can touch a 5 cm-radius cylinder only along a line, and a fixed-pitch chain is over-constrained, so geom-contact on
>= 5 contiguous modules is geometrically unreachable for any length (M1: 2-3 modules held, 9 transient at n=11 with middle modules 1-10 mm off).
Real capacitive pads respond to **proximity** (conductive object within ~5-20 mm) and the real body has compliant covers, so the physical model is:
- **Ground truth (privileged, eval + reward):** `face_prox_gt[k,f]` = 1 if the minimum distance between face f's polygon of module k and the post surface
  is **<= d_prox = 15 mm** (fixed in the ground truth; the sensor model randomises 8-20 mm). Implementation hint: `margin = gap = 0.015` on the post geom
  makes MuJoCo *detect* contacts up to 15 mm without generating force - a proximity sensor with no physics change (Sonnet verifies margin/gap combine as max).
  `module_hug_gt[k]` = any face within d_prox in >= 3 of the last 5 ticks. **Hug tick** = longest chain-contiguous run of hugging modules
  **>= H = max(5, ceil(0.4 * n_modules))** (a straight body alongside the post gets <= 3 modules within 15 mm as measured in M1b, so wrapping is still forced).
  **Hug held** = hug tick true in >= 90% of the ticks of a sliding **3 s** window (dynamic hold with torque is allowed; brief dropouts tolerated).
  Episode success = held at least once; `t_hug` recorded. **No squeeze/pressure condition** (rigid hulls cannot squeeze; pressure stays an optional obs).
  Cross-checks (reported only): post axis inside the polygon of hugging module centres (encirclement), total normal force on the post, wrap angle.
- **Second measurements (reported, not criteria):** coverage from the *sensor model* (should agree with ground truth within the modelled noise);
  total normal force on the post from hugging modules (a hug squeezes; expected >= ~0.3 N per module); diagnostic floor-plane wrap angle and helix height.
- **Reward (D1/D2/E, on top of the nav reward, which keeps paying until first post contact):**
  `approach`: reduction of the mean module-to-post surface distance (privileged, telescoping);
  `coverage`: ratchet on the running peak of the number of hugging (proximity) modules in the longest contiguous run (pays growth only; S7a lesson);
  (no squeeze term - dropped after M1; contact forces are an artefact of rigid hulls);
  `hold`: per tick while the hug tick holds, growing with consecutive hold ticks; terminal `hug_bonus x (1 + speed_gain x remaining-time fraction)` at 3 s;
  `validity`: penalty if any penetration > 5 mm; energy, action-rate and current terms unchanged.
- **Exploit guards**: lying alongside the post (<= 2 modules touching) earns only the saturating `approach`; flinging the body round the post earns nothing
  because hugging requires debounced, chain-contiguous, held contact; clipping through the post hits `validity` and the glitch guard; pushing the post is
  impossible (welded); touching with end caps (not faces) does not count; stacking modules on the post (helical) counts if faces touch (D5);
  the sensor model is never used in the reward, so sensor noise cannot be gamed.

### 5.4 Capacitive face-sensor model in sim (new; post only)
- **Face geometry per module**: the three lateral prism faces are identified from the module hull as the three largest planar facets whose normals are
  roughly perpendicular to the chain axis; labelled in the **body frame already corrected as `RobotBackend` presents it**: `middle` = the face whose
  normal is up on even modules / down on odd modules (owner's "top = even-module middle-pad side"), `left`/`right` egocentric looking from tail to head.
  The odd-module L/R swap and pad-order reversals are hardware-side corrections and are **not** re-applied in sim. Tests: settled flat robot - even
  modules' middle normals point +z, odd -z; left normals have +y components for all modules.
- **Signal**: `touch[k,f]` = 1 if any post contact maps to face f (35 deg cone); `pressure[k,f]` = clipped, quantised (8-bit) sum of normal forces on
  that face / 2 N. **Only the post geom triggers** the block; floor, obstacles and self-contacts never do (assumption A1 below).
- **Realism/DR**: sensor sample rate 10-20 Hz (hold between samples), latency 1-3 control ticks, false-negative probability 2-5% per sample, false
  positive 0.5%, pressure noise 10%, pressure block zeroed in 30% of episodes. Owner may replace these with measured values (sec 8, open item O1).
- Pads are not modelled individually (a face is one touch bit + one pressure value); pad-level layout is below the policy's needs and would overfit the sim.

### 5.5 Power model: per-robot current budget (mean <= 4 A, spikes <= 10 A)
- **Model** (from the hanging measurements, `config.py:14`): `bus = I_idle(n) + 13.3 * sum_j tau_j^2`, `I_idle(n) = 0.25 + 0.05 * n_joints` A (0.6 A at 7
  joints as today; the split between electronics and per-motor idle is an assumption, A2). Scaling: for n motors at equal RMS torque, the 4 A mean
  allows `tau_rms = sqrt((4 - I_idle)/(13.3 n))` = 0.19 N.m (7 joints), 0.15 (11), 0.13 (14); the 10 A spike allows `sum tau^2 <= ~0.7 N.m^2`
  regardless of n (so a long body can still saturate 2-3 joints at once, but not more).
- **Enforcement in sim (sim-to-real: the real power manager will clip)**: a per-tick **spike clamp** - if the model predicts > 10 A, all actuator
  `forcerange`s are scaled down for that tick so the prediction equals 10 A (mirrors the demand allocator) - and a **mean budget** enforced by the reward:
  `current` penalty on a 2 s running mean above 4 A (replaces the instantaneous `max(0, bus-4)` of `locomote.py:65`), plus the gate metrics of sec 4.1.
  The policy observes nothing new (the clamp is part of the plant, as on hardware).
- Consequence: speed on terrain and hug squeeze are current-limited; the length study (sec 3) reports mean/peak current per length so the trade-off
  is explicit.

## 6. Algorithm choices (decisive)
- **PPO (SB3) stays**; recipe per stage fixed: `n_steps 1024`, batch 2048, 8 epochs, `gamma 0.997` (horizon ~17 s: barrier search and the 3 s hold),
  `gae_lambda 0.97`, `ent_coef 0.005` from scratch / 0.002 on warm starts, lr 3e-4 linear decay, `log_std_init -1`. Continuations never change ent/lr.
- **Asymmetric critic** (new): Dict observation `{"policy", "privileged"}`; custom `ActorCriticPolicy` whose actor reads only `policy`; the critic reads
  both. `export_policy` exports the actor only. Main lever for the touch problem (the critic sees the wall and the gap).
- **Memory**: EMAs 0.5/3/10/30 s first (deployable, worked). **LSTM via `sb3-contrib RecurrentPPO` is pre-approved (D3)** and is used if C1 fails its
  3-iteration ladder; the deployable export then carries the recurrent state.
- **Flat policy, no selector/CPG**; rolling allowed, not rewarded.
- **Curriculum** by per-episode sampling ranges; warm start down the chain with zero-padded new obs columns (`train.py:35-70`).
- **DR**: plant DR 0.6 train / 0.6 gate (1.0 reported); terrain scale and friction tiles; delta noise/bias/delay; reply-frame noise x0.5-2, delay,
  dropped frames; capacitive latency/dropouts/pressure-off. **D9: no hardware capture.** A stationary telemetry read (robot limp, no motion) would give
  the reply noise floor without moving the robot, but still means connecting to the robot; it is an optional later ask (O2), not a dependency.
- **Sim-to-real hygiene**: obs/action identical to `GaitPolicy`; no privileged entries in `policy.npz`; the spike clamp mirrors the power manager;
  longer bodies are sim-only until built.

## 7. Orchestration (Fable directs, Sonnet executes)

**Fable keeps:** stage and gate definitions; reward terms/weights; obs/action design; sensor- and power-model assumptions; the body-length proposal;
diagnosis of every failed iteration; exploit hunting; declaring passes (never from training curves); escalation; plan updates; owner questions.

**Sonnet executes** (directives with acceptance criteria): code + unit tests (`n_modules`, hfield, post, face-sensor model, power clamp, hug metric,
asymmetric critic, gate CI/ablations/blind baseline/current metrics, layout cache, contact sheets); launching/monitoring runs; gates, ablations, leak
tests, perturbation suites; captioned videos + contact sheets; factual log entries in `CURRICULUM_LOG.md`; regression tests before every launch
(`pytest tests/test_rl_env.py tests/test_rl_tasks.py tests/test_curriculum.py tests/test_model_builder.py`); throughput benchmarks.

**Directive template (code):**
```
Task: <one sentence>. Files: <paths>. Do NOT change: stages.py gates, reward weights, obs order (append only), petctl/config.py.
Spec: <bullets with exact names/defaults>. Tests to add: <list>. Acceptance: pytest green; 8-module MJCF hash unchanged; <specific numbers>.
Report (<=200 words + JSON): files changed, tests, MJCF hash before/after, open doubts. No design decisions: ask.
```
**Directive template (run):**
```
Launch stage <name> iter_<NN> from <parent> with overrides <dict> (validated against Stage.knobs). One job; 8 envs; PETCTL_GAMMA=0.997.
On finish/early-stop: gate + ablations (touch_off, imu_off, capacitive_off, delta_bias) + leak test + blind baseline; 4 captioned videos + contact sheets.
Report JSON: run dir, steps, wall_s, gate json path, success nominal/robust with CI, each ablation drop with CI, blind baseline, median t_goal/t_hug,
contact_frac, bus_mean_a/bus_peak_a, glitch rate, video paths, 3-line anomaly note. Do not interpret or retune.
```
**Log entry template:** date, stage/iter, change + reason, gate numbers with CIs, ablations, current metrics, verdict (PASS/FAIL/VOID + why), next step.

**Assessment per iteration (Fable, ~15 min):** JSON -> CI lower bound vs threshold -> ablation drops, blind baseline, current budget -> 2 contact sheets
(one success, one failure) -> exploit checklist (what else earns this reward? is the gate metric independent? does the sensor model leak?) -> decide:
pass / one named knob / design change / escalate.

**Stop and escalation rules:** 3 knob iterations without the CI moving -> design change; 2 design changes -> owner escalation with options; exploitable
gate -> VOID, fix metric (logged as mis-calibration with data), re-gate; glitch > 0 -> stop, fix physics; success falling > 0.2 within a run -> stop,
restart from best checkpoint with identical hyper-parameters; current budget violated by a passing policy -> FAIL (not a knob).

**Cost controls (owner directive 2026-11-14: compute is a fanless M4 MacBook Air, 4P+6E cores, thermally throttled):**
- **Every run has a wall budget of 60 min including its gate.** Executors measure steps/s over the first 2 min, set `steps = 0.75 x budget x steps/s`
  (cap 2.5M flat; hug stages: 0.75 x wall x steps/s, cap 4M), and if the projected wall exceeds the budget they cut steps or episode length, never evidence (seeds, held-out n, ablations, captions).
- **Pilot first**: 10-15 min (<= 600k steps), DR 0.3, with an explicit go/no-go signal (flat: rolling success >= 0.15 and median progress >= 0.3 m;
  hug: >= 5% of episodes with run >= 3 and span >= 90 deg and at least one wrap tick; holding: hold_duty > 0.2 by the second hour-run). No long run without a pilot.
- **Hug-run stop checks are TREND rules, not fixed thresholds on a level the policy has not reached** (2026-11-14, hug_curr_02): at 20 min stop ONLY if ALL of: curved-family (cylinders + rounded
  boxes) success < 0.03 over its last 150 episodes, it has not risen vs the 150 before, and the cylinder peak-span p75 is not >= 5 deg above its value at the start (106 deg); no 40-min stop; a
  separate safety stop if the box success falls by more than 0.2 from its level at the start. At the end: n=30 screen >= 0.55 -> n=100 verdict; else a family advanced a level or curved success
  rising -> one more hour; else design change. Every check records per family: level, success, wrap-90 rate, contact rate, span p50/p75, joint bend p50/p75.
- **Screening gates n = 30** (robust DR 0.6) during iteration: go if the point estimate >= threshold - 0.15 (Wilson 95% half-width ~0.17 at n = 30);
  **n = 100 held-out only for stage verdicts.**
- Long stages are chained as 60-min continuations with identical hyper-parameters, each with its own screen; one job at a time; stage budgets in sec 4.2
  are ceilings, not targets. Episodes as short as the task allows (flat 20 s, hug 20 s with stall/success termination).
- Sonnet at default effort; Fable reads JSON + 2 contact sheets per run; no new stage designed until the previous ablation is in.

## 8. Risks, assumptions, open items
Assumptions (A) - stated so they can be falsified later:
- **A1** The floor/carpet, obstacles and the robot's own body do **not** trigger the capacitive pads; only the post does (owner: post acts as a human).
- **A2** Idle current splits as 0.25 A electronics + 0.05 A per motor; the 13.3 A/(N.m)^2 coefficient (static hanging) holds while moving.
- **A3** Reply-frame noise 0.02 N.m / 12-bit, 47.5 Hz bursty feedback approximated by 1-2 tick delay + 5% drops (no capture, D9).
- **A4** Capacitive sample rate 10-20 Hz, latency 50-150 ms, dropout 2-5%.
Open items for the owner (not blocking):
- **O1** Capacitive sensor specs if known (rate, latency, whether pressure pads stay); otherwise A4 stands.
- **O2** Optional later: a stationary (limp, no-motion) telemetry read for the reply noise floor; a moving capture would need motion approval. Not required.
Risks (R):
- **R1** 8-10 modules cannot place 5 contiguous faces on a 10 cm post within the current budget (sec 1.3) - resolved by the static feasibility search;
  a longer body must be built for hardware.
- **R2** Touch use still undemonstrated on the barrier after asymmetric critic + memory (the historically unsolved part). Fallbacks in order: LSTM
  (pre-approved); deployable two-level scheme using the existing linear contact decoder (AUC 0.905) as a hand-made feature.
- **R3** Terrain contact glitches with convex hulls - smooth bumps, 2.5 cm cells, glitch gate.
- **R4** The 4 A mean budget, not the 0.5 N.m cap, limits speed on slopes and hug squeeze, and tightens with length - measured in S0; may force L down.
- **R5** Loose terrain approximated by friction + roughness.
- **R6** Localiser realism (delta DR levels are guesses) - bounded by the bias-perturbation eval.
- **R7** Compute ~100M steps on 10 cores; layout cache mitigates.
- **R8** Face-sensor model realism (A1, A4; face identification from hulls) - unit tests on geometry, DR on timing; a `capacitive_off` policy
  (motor-reply-only hug) is trained as a fallback if hardware sensing disappoints.
- **R9** Power-model error: if the real allocator clips harder than the sim clamp, gaits trained here will be slower on hardware; the gate's
  current metrics make the margin visible.

## 9. First milestones (concrete)
| M | Deliverable | Success criterion | Est. |
|---|---|---|---|
| M0 | Infra: `n_modules` (8..15), hfield terrain + friction tiles, post, **face-sensor model + `capacitive_off`**, **per-robot current model + spike clamp + gate metrics**, hug metric (contiguous faces, hold), asymmetric critic, gate CI/ablations/blind baseline, layout cache, contact sheets, stale cap default fixed, head-frame delta + near-field channel + delta DR | all tests green; 8-module MJCF identical; face labels verified on the settled robot; hug metric unit tests (5 contiguous modules touching -> hug tick; straight body alongside -> no); current metrics match the model on a saturated-joint test; throughput logged | 3-4 days Sonnet |
| M1 | Static hug feasibility for L = 8..15 (DONE, geom-contact def.: 2-3 modules for all L -> metric re-calibrated, sec 5.3) | table: max contiguous hugging modules, joint angles, holding current | done |
| M1b | Re-score M1 best poses and the scripted coils under the proximity definition (no new search unless < H at n=8); **dynamic hold test**: scripted coil held by torque for 3 s under the 90%-duty rule at n = 8, 10, 12 | >= H modules held for the candidate lengths, or a documented failure before M2 | DONE 2026-11-13: H held at n=8, 9 (poses) and n=8, 10 (constant coil); n=12 only at the penetration/current limit (M1_HUG_FEASIBILITY.md) |
| M2 | RL sweep {8, 10, 12} then refine +-1 (flat, terrain, hug-from-contact; 2 seeds; 100-episode tables with CIs and current metrics) | L proposed with evidence; owner approves | 1.5-2 days compute |
| M3 | A1 + A2 passed at L | gates with CI; lateral metric; current budget met | 1 day |
| M4 | B1 + B2 passed; imu_off drop shown | gates + ablation | 1-2 days |
| M5 | C1 barrier passed **with touch_off drop >= 0.20 and blind baseline <= 0.30**; C2 | the key navigation result | 3-5 days (R2) |
| M6 | D1 + D2 hug passed; capacitive_off drop; delta-bias table; post-leak test | hug held >= 0.70 | 2-3 days |
| M7 | E full course passed; final eval suite; videos; plan + log updated; skills banked in `models/skills/` | hug held >= 0.60 on terrain + field with all three ablations | 2-3 days |
Total: ~3.5 weeks wall-clock, one job at a time; Fable effort concentrated at M1/M2 decisions, every gate verdict, and failed iterations.


## Owner standing rule: videos (revised 2026-11-14)
Supersedes the earlier video rule. Every 30 minutes, one captioned progress video of the best run so far for each stage under way, until that stage completes (several stages under way = several videos per 30 min), plus one best video at stage end. No per-hour cap. Captions state only facts measured on the recorded episode. Implemented by `scripts/rl/progress_videos.py` and a periodic `progress_policy.npz` export in the training callback.
