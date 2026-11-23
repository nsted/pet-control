# Curriculum state (read this first; ~1 page; keep it current)

Updated: 2026-11-23. Full history: `CURRICULUM_LOG.md` (recent) and `archive/` (old, do not read by default).

## Where we are
- Hug line PARKED at n=100 success 0.12 (skill `hug_L8_scale15_h6`), gate 0.70/0.50. Return later with an LSTM policy + hold-break data (half of hold breaks coincide with bus > 4 A).
- Nav chain (`scripts/rl/chain_ab_driver.py`, data `data/rl/chain_ab/`): A1 flat_nav PASS, A2 flat_omni PASS, B1 terrain_easy PASS, **B2 terrain_full STOPPED** (`STOP_terrain_full`): success 0.93 / imu ablation OK / glitch 0, but peak-current budget fails twice (28%, then 24% of episodes > 10 A; limit 5%) even with bus_peak_penalty 10.
- 2026-11-23 fix (owner request): the hard 10 A cutoff in the sim is replaced, for terrain_full, by a SOFT limiter (`bus_soft_knee` 0.7: applied current follows demand up to 7 A, then rolls off smoothly (tanh) toward 10 A and never exceeds it; `petctl/sim/bus.py::soft_limit_current`). Knee 0 = old hard clamp (other stages unchanged). Gate still judges DEMANDED current.
- **B2 terrain_full PASSED (2026-11-23 12:30, attempt 1 under the soft limiter, root `data/rl/chain_ab_soft`, warm from the old hard-clamp a2):** success 0.93 (Wilson lo 0.863 vs 0.70), imu_off drop 0.87 (CI lo 0.80 vs 0.15), t_goal median 20.5 s, bus mean 2.49 A, bus peak median 8.5 A, episodes > 10 A 1% (limit 5%; was 24-28%), glitch 0; 4.0 M steps, 33 min; skill `models/skills/nav_chain/terrain_full`. Chain A1-A2-B1-B2 is DONE.
- Nothing is running. Next: C1/C2 touch navigation (plan sec 4.2 of REGIMEN_V2_PLAN.md), then return to the hug (LSTM policy + hold-break data). Note: stage_summary.py reads only `data/rl/chain_ab` (terrain_full row not in STAGE_SUMMARIES.md; its ROOT needs a `chain_ab_soft` fallback).
- Cosmetic: new sim colors in `petctl/sim/style.py` (opt out with BuildParams.extra["plain_colors"]=1).

## Fixed rules (do not re-litigate)
Sim only, never move the robot. Torque cap 0.5 N.m; current 10 A peak / 4 A mean. Gates frozen except demonstrated mis-calibration. Runs <= ~1 h; pilots first. Fail on current budget = fix, not retune. Stage: max 3 knob iterations, then design change, then owner escalation (a message, never a halt of compute).

## Token-cost rules (binding, owner request 2026-11-23: credits)
1. No polling crons. Idle checking is done by `scripts/rl/event_watch.sh` (plain shell, zero tokens); Claude wakes only on an event line (STOP_/DONE/driver dead/idle machine).
2. Routine decisions follow the written stop rules automatically; the expert (Opus) is consulted only when a rule is ambiguous or a stage failed twice with different causes.
3. Do not read `archive/` or the full log; read this file plus the log tail (`tail -c 3000`).
4. Do not look at progress videos/contact sheets unless an event or a gate verdict needs it. Rendering is free; viewing is not. One best video per stage at its end.
5. Batch work into one executor directive; do small edits in the main session; no cold-start subagent for a < 10 min task.
6. Log entries: result + decision, <= 5 lines.
