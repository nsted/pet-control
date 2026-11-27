# Edge avoidance (owner goal, 2026-11-27): locomote on a raised platform without falling off

**Goal.** While locomoting, when motor feedback says the floor is gone (a part of the body overhangs the edge), find the floor again. Bad: falling. Equally bad: refusing to move. Belongs to Phase 8 obstacle skills. Sim only, n = 11, platform 25 cm high (a fall cannot be recovered: a 12 cm climb already failed).

**Sensing = proprioception only** (what the robot has): joint torques (MIT feedback), joint position / velocity tracking error, head IMU (gravity direction, gyro). No position, no edge map. (Earlier skill demos used privileged position; this one must not.)

**Pieces.**
1. `edge_sim.py`: the scene (finite platform 1.5 x 1.5 m, height 0.25 m), the library selector as the locomotion layer (`CPGEnv.prior`, scale_f), a wander command generator, per-tick labels (overhang fraction = modules whose centre is outside the platform footprint; fallen = COM below 0.8 H), episode metrics.
2. Detector: supervised on rollouts (features: joint torques, tracking errors, IMU tilt, gyro, short history; label: overhang >= 1 module within the next 0.5 s). Judge: AUC and the LEAD TIME before overhang reaches 3 modules.
3. Reflex: LOCOMOTE -> (detector fires) -> RETREAT (reverse gait) -> TURN (away; random sign) -> LOCOMOTE.
4. Evaluation: 60 s episodes, random start position / heading / friction x1.0-1.3. Report per policy: fall rate, mean path length (m), area coverage (0.25 m cells), fraction of time moving. Baselines: wander without reflex (falls), stand still (never falls, never moves), reflex with an oracle edge detector (privileged, upper bound).
5. Tuning of reflex parameters (threshold, retreat time, turn angle) with CMA-ES on the weighted objective. RL only if the classical + CMA version leaves a gap.

**Pass.** Fall rate <= 5% over >= 100 randomised episodes AND mean path length >= 60% of the unprotected wanderer's AND coverage >= 50% of the oracle-reflex coverage. Stand-still must fail the activity test; the unprotected wanderer must fail the fall test (otherwise the task is too easy).

## Status 2026-11-27 (stopped by the owner)
Done: scene (`edge_sim.py`), data collection, proprioceptive detector (AUC 0.84 for "overhanging now"; the gait's own torques mask the ~0.02 N.m overhang signature), reflex controllers and renderer, baselines (wander falls in 98% of 60 s episodes, stand never falls / never moves), baseline video. NOT done: the oracle-reflex probe (`edge_oracle_probe.py`, 6 variants: instant parameter snap, halt-then-retreat, distance-triggered) was killed before printing results; the first oracle run (trigger at overhang >= 1, slow retreat) still fell 100% of episodes (the body falls a median 1.0 s after the first module overhangs; reversing the gait takes longer). The learned-detector reflex therefore has not been shown to work.
Direction under discussion with the owner: restart around RL that integrates proprioception (and touch) with a base layer of pose HOLDING under disturbance, possibly preceded by a self-model / compliance layer. See CURRICULUM_STATE.md.

## Reflex videos (2026-11-27, seeds 90000-90002, 40 s)
Wander (no reflex): falls at 11.5 / 19.2 / 6.0 s. Oracle trigger at first overhang with the slow retreat: 11.2 / 18.1 / 6.8 s (no help). Oracle trigger at COM within 30 cm of the edge + instant parameter snap + 1 s halt + retreat + turn: 31.7 / 36.5 / 35.8 s (about 3x longer survival, still falls). Learned detector reflex: 11.5 / 29.9 / 6.8 s. So: (1) a reflex needs >= 1-2 s of warning before overhang, (2) perfect early warning triples survival but does not stop falls yet (the turn / retreat gaits themselves drift toward the edge), (3) the proprioceptive detector does not provide that warning. Videos: edge_reflex_*.mp4.
