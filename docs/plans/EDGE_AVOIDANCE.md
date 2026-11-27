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
