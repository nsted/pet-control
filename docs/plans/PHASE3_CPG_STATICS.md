# Phase 3 — CPG basis and feasibility statics (2026-11-26)

Code: `petctl/rl/cpg.py` (basis), `petctl/rl/gait_eval.py` (open-loop evaluator, reused by Phase 4), `scripts/rl/cpg_sweep.py`, `scripts/rl/statics.py`. Tests: `tests/test_cpg.py` (67).
Conditions: sim only, 0.5 N.m cap, isotropic friction 1.0, GIM3505-8 current PROXY (unvalidated), default joint gains (kp 0.8, kd 0.035; dynamic gains come in Phase 7).

## 1. The CPG basis (joint-pair coordinates)
Joints 1..J (J = n_modules - 1) are paired (2m-1, 2m); yaw_m = q_e + q_o, twist_m = q_e - q_o (so q_e = (yaw + twist)/2, q_o = (yaw - twist)/2):

    yaw_m   = A_yaw(m)   sin(Phi - (m-1) dphi)        + B_yaw(m)
    twist_m = A_twist(m) sin(Phi - (m-1) dphi + psi)  + B_twist(m)      dPhi/dt = 2 pi f

- **11 parameters, independent of length:** `f, dphi, psi, a_yaw, a_twist, taper_yaw, taper_twist, b_yaw0, b_yaw1, b_twist0, b_twist1` (amplitude envelope and bias profile are linear along the
  body; the twist bias selects the bending plane, the yaw bias a steady curvature = turning; biases are unbounded, so multi-turn behaviour lives in them, not in raw joint targets).
- **Odd joint count** (n_modules even: 10, 12): the last pair has only its odd member, driven as the `o` member of a virtual pair with the same parameters (no extra parameter).
- **No jumps:** the phase is integrated; every parameter, including f, follows an exact critically damped 2nd-order filter (`smooth_hz`, 1 Hz default: ~0.65 s settling). Test: the per-tick
  joint-target change scales with dt through a huge parameter step (a discontinuity would not), and the filter never overshoots.
- **Validation against `serpenoid_baseline.py` (required before any search): PASSED.** The baseline q_k = s_k A sin(2 pi f t - k phi), plain or alternating signs, is an exact special case
  (`from_serpenoid`: dphi = 2 phi, yaw/twist amplitudes and psi from the pair phasors). Targets agree to 1e-9 for J = 7, 9, 10, 11, 12 and 6 lags x 2 sign patterns (60 cases); simulated bodies end in
  identical states (n = 10, 11). Evaluator cross-check against the baseline script (n = 10, 40 configs): forward displacement correlation 1.00, best 20 cm vs 22 cm in 10 s (window / COM vs head difference).

## 2. Parameter sweeps (400 configurations per length, 12 s rollouts, `data/rl/cpg_sweep/`, `scripts/rl/cpg_sweep.py`)
Share of configurations that move the body (> 1 cm/s), by group: yaw wave alone 0-4 %, twist wave alone 2-5 %, **both waves (psi, unequal amplitudes) 31-38 %**, static-bias group 0-67 %
(the bias numbers are slow drifts of a re-posed body, not locomotion; they vary erratically with length parity and are not used). Best configuration per behaviour (cm/s, rad/s; grid edge: f <= 1 Hz,
amplitudes <= 0.9 rad):

| n | forward | reverse | left | right | turn (max |yaw rate|) | roll (max |rate|) |
|---|---|---|---|---|---|---|
| 10 | +3.7 cm/s (0.080 bl/s) | -4.1 (0.091) | +2.5 | -1.8 | +0.20 / -0.08 rad/s | -0.11 rad/s |
| 11 | +3.6 (0.072) | -4.0 (0.078) | +2.3 | -2.3 | +0.15 / -0.12 | -0.11 |
| 12 | +3.0 (0.056) | -3.4 (0.061) | +1.3 | -1.2 | +0.08 / -0.07 | -0.14 |
| 13 | +3.8 (0.063) | -3.2 (0.053) | +2.6 | -1.4 | +0.11 / -0.14 | -0.13 |
Reading: (1) the pair basis produces all of forward, reverse, sideways, turning and rolling at every length; the best forward / reverse configurations are the same wave with reversed lag,
(2) they use BOTH waves (yaw 0.9 + twist 0.9, f 1 Hz, i.e. at the edge of the grid), a single wave almost never moves the body, (3) peak torque reaches the 0.5 N.m cap in the best
configurations: the gaits are torque-limited, so the search (Phase 4) must go beyond this grid (higher f, other amplitude ratios, tapers), (4) speeds here are only a coarse-grid lower bound; the earlier
36-parameter repertoire reached up to ~7-11 cm/s (8 modules, other conditions), so the Phase 4 search must show whether the 11-parameter basis loses expressiveness; seeding from that repertoire
(mapped into this basis) is the first Phase 4 task. No length ranking is drawn from this table (single coarse grid, no seeds, no robustness).

## 3. Feasibility statics at 0.5 N.m
(Results appended below when the runs finish: `scripts/rl/statics.py`, `data/rl/statics/results.json`.)
