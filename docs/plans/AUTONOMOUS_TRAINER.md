# AUTONOMOUS_TRAINER — brief for PET's autonomous trainer

Saved verbatim from the owner's brief (2026-11-26). It overrides older RL plans where they conflict.
`CURRICULUM_STATE.md` stays the short, current "where we are" file.

## Mission
1. Find the best body length for the rebuilt robot from evidence. Candidates: 10, 11, 12, 13
   modules. Owner leans to odd lengths for aesthetic reasons, but function decides.
2. Train PET in simulation until it uses its body robustly and flexibly: locomotion in all
   directions, steering, self-righting, terrain, obstacles, climbing where physically possible,
   smooth transitions and blends between gaits and skills, and the poses of a Rubik's snake.
Work autonomously. The owner reviews progress at any time through the review index (see Reporting).

## Fixed facts and limits (do not re-litigate)
- Sim only. Never move the real robot. The owner is rebuilding the robot; its length is not fixed.
- Body: a chain of triangular-prism modules with Rubik's-snake joint geometry (see
  docs/plans/URDF_GENERATOR.md): n modules = n-1 single-axis joints, axes at 45 degrees to the
  chain, neighbour axes at 90 degrees. Joints are multi-turn, no angle limits.
- Use the URDF generator for every candidate length. Existing skills, gates and tuning were made
  on the previous body: use them as reference and as seeds, not as baselines to beat.
- Torque cap 0.5 N.m is correct. Current: 10 A peak / 4 A mean, on a shared bus. Model the shared
  bus realistically for every length.
- Hardware: Apple Silicon Mac, CPU only, no GPU. Native MuJoCo + SB3. Do not use MJX locally.
  Measured throughput is about 2k env steps/s with 10 envs: budget RL runs in steps, not hours.
- Keep the std-runaway fix (ent_coef 0, log_std_init -1, re-applied after warm starts).
- Flipping is NOT a failure. Never end an episode because the body flipped.
- Import hardware limits from petctl/config.py.

## Approvals (ask the owner first)
1. Any change to the frozen evaluation suites or a gate threshold.
2. Any change to physical limits (torque, current, module geometry/mass, default friction).
3. Anything that touches the real robot.
4. Destructive git (reset --hard, force push, branch -D) and deleting data.
5. Installing packages (for example pip install cma). Ask once, then continue.
6. GitHub issues (see CLAUDE.md).
7. The length-study criteria and weights, before the study runs.
8. Any change to .claude/ settings or hook scripts. The trainer may not edit them itself.
All other changes are allowed, including redesigns of the action space, task, reward and
curriculum. Log each redesign and its reason in docs/plans/METHOD_REVIEWS.md.

## Models and subagents
- Main loop runs on Sonnet. Define these subagents in .claude/agents/ (show the owner the files first):
  - reviewer.md (model: opus): adversarial pre-launch review (Phase 1).
  - method-review.md (model: opus): method reviews, length recommendation, reward redesign after
    a stage fails twice.
  - explore.md (model: haiku): file search and log summaries.
- Do not start a subagent for a task under ~10 min; do it in the main session.
- Never use Haiku for contact-sheet judgments, reward design or gate decisions.

## Phase 0 - Housekeeping
- Commit the uncommitted skill-gating work to branch exp/skill-gating-wip
  ("WIP: skill gating experiment, failed: gates unused"). Tag attempt-1-end at f450903.
  Create feat/autonomous-trainer from f450903 and work there.
- Move the unattended ladder scripts (dunes_ladder*.sh) from the Claude scratchpad into
  scripts/rl/. Archive dead stage-specific drivers (m2v*, hug_curr*, etc.) into an archive folder
  so one curriculum runner remains. Moving is fine; ask before deleting.
- Fix the note in docs/BEST_RUNS.md: 0.5 N.m is the correct torque cap.
- Record git hash, body length and seed with every checkpoint, video, table and gate result.

## Phase 1 - Safeguards (before new training)
- Two frozen, length-generic evaluation sets per skill: a DECISION set (used for gates and
  choices) and a TEST set (never used for decisions; opened only at milestones). Fixed seeds,
  start poses (including upside-down, side and twisted starts), terrains and commands. Store in
  data/rl/eval_suite/ with a SHA-256 manifest and a check script that fails if the hash changes.
- Report results with confidence intervals over >= 3 training seeds (Wilson or bootstrap).
- Log every reward term separately (mean and share of total) to TensorBoard and the run summary.
- Watchdog (extend the existing health callback): check every 2-5 min, with a minimum window
  before any verdict; stop the run and write STOP_watchdog with the reason when:
  - reward rises but the decision-set task metric is flat or falling (reward-hack signature);
  - a reward term's share of the total changes sharply from its pilot baseline;
  - no task progress after N steps, or episode length collapses;
  - the flip rate rises while the task metric falls, or flips raise reward without progress;
  - actions saturate, NaN appears, or the policy std leaves bounds set per action unit.
- Log orientation time, flips per episode and recovery time as diagnostics.
- Pilot rule: every change to reward, task, env or action space gets a short pilot (about
  0.5-1M steps) with a contact sheet (frame grid of all eval scenarios). Start a full run only
  after the pilot passes the watchdog AND the contact sheet has been looked at.
- Hooks (PreToolUse, exit code 2 blocks), stored outside the repo where the trainer cannot edit them:
  - block a full-run launch unless the pilot passed and review.md exists for the previous run;
  - block git reset --hard, git push --force, git branch -D;
  - block writes to .claude/ and to the hook scripts.
  Write the hook config and scripts as a proposal file and show them to the owner; the owner installs them.
- Adversarial reviewer: before each full run, a fresh-context subagent gets the reward diff, the
  reward-term numbers and the pilot contact sheet, and must find how the policy could get this
  reward without doing the task. Fix what it finds before launch.
- Token rule 4 change: one contact-sheet review per pilot and per full-run end is REQUIRED.
  Videos stay optional and event-driven. Keep the zero-token event watcher.

## Phase 2 - Model checks and speed (short task, not a long phase)
- Generate URDFs for 10, 11, 12 and 13 modules. Flag every geometric assumption that is not
  verified (for example odd-length tail orientation) in a list for the owner to confirm in CAD.
- Check that the observation lets the policy see multi-turn joint state (observed angles are
  wrapped to +-pi; the last target or a turn count must be visible).
- Friction: randomize friction and include direction-dependent (anisotropic) friction, because
  the real skin friction is unmeasured. Expect rolling and sidewinding to beat slithering.
- Benchmark steps/s against n_envs (4-12) and use the best value. Use mujoco.rollout or a process
  pool for parallel black-box evaluation.

## Phase 3 - CPG basis and feasibility statics
- CPG in joint-PAIR coordinates: for each neighbour pair (q_e, q_o), a yaw wave (q_e + q_o) and a
  twist wave (q_e - q_o), plus a per-pair static twist bias that selects the bending plane. With
  an odd number of joints, define how the unpaired joint is driven. Parameters: amplitudes,
  frequency, travelling phase lag, yaw-twist phase offset, biases (about 10-40 in total,
  independent of length). Phase is integrated (dphi/dt = 2*pi*f); amplitudes and offsets follow
  2nd-order dynamics so parameter changes never cause joint jumps. Test with parameter sweeps.
- Validate the basis against scripts/rl/serpenoid_baseline.py before any search.
- Static / quasi-static feasibility per skill and per length at 0.5 N.m: head lift, step-up
  height, gap bridge, self-righting, cylinder wrap range. Skills with no feasible pose are dropped
  or redefined (for example as dynamic, momentum-based moves) and logged.

## Phase 4 - Gait search tools
- CMA-ES gait search, seeded from the existing repertoire (archive.json, archive_height.json,
  hoop-roll search), mapped into the new CPG basis.
- MAP-Elites over the CPG space with behavior descriptors: travel direction, body rotation,
  contact pattern, head height. Also sample interpolations between pairs of library gaits (about
  10 points per pair). A new descriptor cell = a new gait type; keep it.
- Target gaits: forward, reverse, turning, sidewinding/lateral, rolling, concertina/inchworm, plus
  any new or blended gait that is useful.
- Score each gait: speed (m/s and body lengths/s), cost of transport, current (peak and mean,
  total and per module), smoothness (joint jerk), robustness under domain randomization.
- Validate the tools on one candidate length (11) first.

## Phase 5 - Rubik's-snake pose tools
- Goal: PET can take and hold the Rubik's-snake poses that fit its length, and move between them.
- Enumerate joint configurations at 90-degree steps (4^(n-1)); remove self-collisions with a fast
  kinematic check; then test each in MuJoCo: settle on the ground and hold for 3 s within
  0.5 N.m and the bus budget. Start with planar (flat) configurations.
- Descriptors per pose: height, compactness, symmetry, closed loop or open, contact faces.
  Include the known Rubik's-snake figures that fit the length.
- Transitions: a graph of feasible poses; edges = joint moves that stay feasible along the whole
  path (quasi-static sim). Plan with graph search; mark pairs with no quasi-static path for an RL
  transition skill later.
- Validate the tools on length 11 first.

## Phase 6 - Body length study (10, 11, 12, 13)
- Before running, write the criteria and weights to METHOD_REVIEWS.md and show them to the owner
  (approval 7). Criteria: speed (m/s and body lengths/s), cost of transport, peak/mean bus
  current (total and per module, shared-bus limit), gait-type count, feasibility statics
  (head lift, step-up, gap bridge, self-righting, wrap range), number of feasible held poses,
  closed-loop poses, mass.
- For each length: gait library (Phase 4), pose library (Phase 5) and statics (Phase 3), with
  >= 3 seeds, an equal pre-registered compute budget and fixed hyperparameters. No RL here.
- Output: models/gaits/L<n>/, models/poses/L<n>/, one comparison table with absolute and
  per-body-length metrics, ranking stability under +-30% weight changes, and contact sheets.
- Report a recommendation with the evidence. Results stay provisional until the owner confirms the CAD
  assumptions from Phase 2. The final length choice is the owner's.

## Phase 7 - RL locomotion on the chosen length
- Action = continuous modulation of CPG parameters + a small residual joint action. Put
  multi-turn behaviour in the integrated bias/twist parameters, not in raw targets. The policy
  works in CPG parameter space (not gait gates), so it can blend gaits and find in-between gaits.
- Task = command tracking of gait-cycle-averaged (vx, vy, yaw rate) in a principal-axis heading
  frame, dense reward with an exp(-err^2/sigma) kernel. Change commands mid-episode.
- Per-environment terrain curriculum (Rudin et al. 2021): promote on success, demote on failure;
  keep 10-20% of envs on easy terrain. Domain randomization from the start, increasing with
  terrain level. Randomize start orientation.
- Record CPG parameters during evaluation. A parameter cluster far from all library gaits is an
  emergent gait: classify it, record a video, add it to the library.

## Phase 8 - Skills and selector (chosen length)
- Skills, in this order: self-righting / fall recovery; turning in place and reorientation;
  pose transitions where the quasi-static planner fails; obstacle negotiation; hug (LSTM policy,
  as planned); climbing only if Phase 3 statics show it is feasible.
- High-level policy: selects a skill or a target pose and sends commands. Goal-reaching and
  obstacle fields are trained here. Use forced-skill exploration so every skill gets tried.
- Smooth hand-overs between non-CPG skills: crossfade joint targets over 0.3-1 s, or train each
  skill to start from the end states of the others. Switch skills at random times mid-episode.
- Optional: distill all skills into one student policy for onboard use.

## Real-robot check (when the rebuilt robot is available, with the owner's approval)
- Run 2-3 open-loop CPG gaits and a few held poses to check joint signs, speed ratios and holding
  torque against the sim. Recalibrate the sim before further training.

## Operating loop (every iteration)
1. Write a hypothesis (change, reason, expected result) in the run's review.md.
2. Pilot -> watchdog -> contact sheet -> adversarial reviewer.
3. Full run (step budget set in advance) -> gate on the decision set.
4. Complete review.md (actual result, cause, next step). Log <= 5 lines in CURRICULUM_LOG.md.
5. Stop rules: max 3 knob iterations per stage, then a design change, then a method review.

## Method review
Every 10 runs, and after a stage fails twice with different causes: stop tuning and write a
review in METHOD_REVIEWS.md. Is the action space right? The task? The curriculum? What does
current literature do (search the web)? What is the biggest bottleneck: speed, reward or method?
Then act on it.

## Reporting (owner reviews at will)
- Keep docs/media/index.html current: newest run first; per run: body length, contact sheet,
  showcase videos, key metrics with CIs, reward-term breakdown, hypothesis and result, git hash.
  Mark the best-so-far per skill and per length. Pages per length for gaits and poses, and the
  length comparison table.
- Showcase videos: for every full run (and every new gait or pose in the libraries), render one
  landscape (16:9) and one portrait (9:16) video with a cinematic camera:
  - a slow orbit around the robot, combined with zoom: start wide (whole body and terrain),
    push in to a close-up of an interesting moment (ground contact, a wave passing through the
    joints, a pose forming), then pull back out. Pick the close-up moment automatically (for
    example the largest joint motion or the most ground contact);
  - smooth eased camera motion (no jumps or sudden speed changes); the camera follows the body's
    centre so the robot stays in frame; never cut through the body or the terrain;
  - length about 15-30 s, so the portrait version fits Reels/TikTok/Shorts.
- Render each showcase video twice: one with labels (for review) and one clean, without
  labels (for social media).
- Labels: two lines in a semi-transparent band at the bottom of the frame (about 3-4% of the
  frame height):
  Line 1 (run): run id, skill, body length, git hash, step count, speed, success rate, and a
  one-line description of the change.
  Line 2 (environment): terrain type and level, friction (mean, and anisotropy if used),
  domain-randomization level, start orientation, and the active command (vx, vy, yaw rate) or
  target pose. For pose and gait-library videos, line 2 shows terrain, friction and peak/mean
  bus current.
- Comparison media: contact sheets and side-by-side videos (new policy next to the previous
  best) use a FIXED camera per eval scenario, so runs are directly comparable. Clips are
  10-20 s per scenario.
- Render videos after a run ends, not during training. Rendering is cheap; viewing costs tokens,
  so only contact sheets are reviewed by default.
- Videos don't get added to commits, ignore them.
- Keep CURRICULUM_STATE.md to about one page and current.

## References
- Eureka: https://arxiv.org/abs/2310.12931
- DrEureka: https://arxiv.org/abs/2406.01967
- Learning to Walk in Minutes: https://arxiv.org/abs/2109.11978
- CPG review (Ijspeert 2008): https://doi.org/10.1016/j.neunet.2008.03.014
- CPG-RL (Bellegarda & Ijspeert): https://arxiv.org/abs/2211.00458
- Salamander robot gait transitions: https://doi.org/10.1126/science.1138353
- Slithering and friction (Hu et al. 2009): https://doi.org/10.1073/pnas.0812533106
- MAP-Elites: https://arxiv.org/abs/1504.04909
- CMA-ES tutorial: https://arxiv.org/abs/1604.00772
- rliable (RL statistics): https://arxiv.org/abs/2108.13264
- Magic Snake 3D conformations: https://pmc.ncbi.nlm.nih.gov/articles/PMC8208313/
- Rubik's Snakes on a plane: https://math.uni.lu/eml/assets/reports/2021/snakes_WS21.pdf
- Claude Code hooks: https://code.claude.com/docs/en/hooks
