# Phase 4 (in progress): CMA-ES gait search, validation at n = 11

Tool: `scripts/rl/gait_search_cma.py` (11 CPG params, 1500 evals, 0.5 N.m cap, isotropic, GIM current proxy). Check: `scripts/rl/verify_gaits.py` (world-frame COM displacement, independent of the search metrics).

| objective | n=10 | n=11 | n=12 | n=13 | n=11 seed 1 | n=11 fmax 4 Hz |
|---|---|---|---|---|---|---|
| forward cm/s (world) | 17.7 | 19.3 | 13.3 | 15.7 | 10.6 | 11.6 |
| sideways cm/s | - | 17.2 | - | - | 21.9 | 17.8 |
| turn rad/s | - | 1.96 | - | - | 1.77 | - |
| roll rad/s / travel cm/s | 4.1 / 13.6 | 5.6 / 21.8 | 5.8 / 27.0 | 4.9 / 20.8 | 2.1 / 9.5 | - |

Findings
- World-frame displacement matches the search metrics (within a few %), max module height < 16 cm, no net rise: not a launch or metric artefact. Rolling is whole-body (5 modules agree).
- Grid best was 3-4 cm/s; search gives 5x. Speeds are seed-dependent (forward 19 vs 11 at n=11), so one seed per length cannot rank lengths. No length signal yet.
- Relaxing f to 4 Hz did not raise speed: the limit is the 0.5 N.m torque cap, not the 2 Hz bound.
- Open: n=12 roll step 2.8 rad/tick is near the pi aliasing limit; real-robot feasibility (2 Hz, cap-saturated, unvalidated current proxy) untested.
