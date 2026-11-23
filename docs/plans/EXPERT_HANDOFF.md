# Regimen v2 - expert handoff (Fable -> Opus), 2026-11-14

The directing expert is now an **Opus** agent (owner decision, cost). Fable's last state is summarised here; the plan is `REGIMEN_V2_PLAN.md`, the history is the tail of `CURRICULUM_LOG.md`.

## Role
Reserve effort for: stage/gate design, reward/obs/action design, diagnosing failed iterations, deciding go/no-go after stops, body-length and gate-shape decisions. Sonnet executors do code, runs, logs. Manager (main Claude session) dispatches executors and relays.

## Cost rule (2026-11-23, owner: credits)
Consult the expert only when a written stop rule is ambiguous or a stage has failed twice for different causes; routine go/no-go follows the written rules. Expert reads JSON + at most one contact sheet. See `CURRICULUM_STATE.md` (token-cost rules).

## Owner constraints (binding)
Sim only, never move the robot. Torque cap 0.5 N.m; robot current 10 A spikes / 4 A mean. Obstacles felt through motor feedback only; capacitive sensors are for the target only (acts like a human body part). Target knowledge = delta (head frame, head only). NO hand-coded poses/priors for the hug (target shape unknown: wide/narrow, cylinder/box). Flipping/rolling never a failure. Runs <= ~1 h wall (M4 MacBook Air, ~4-6k steps/s flat, ~1.7k hug); pilots with go/no-go signals first. Videos: every 30 min a progress video of the best run so far per active stage, plus one best video per stage at its end, no cap, factual captions. Owner wants autonomy; escalate only if truly blocking. Gates frozen except demonstrated mis-calibration (log it).

## Decisions so far
- Hug = module faces within 15 mm of ANY object; wrap tick = contiguous run >= 4 AND normal_span >= 180 deg; held = 3 s window >= 90% wrap ticks (gate). Frozen gate: aggregate Wilson lower bound >= 0.70, each held-out shape >= 0.50, capacitive_off drop >= 0.15, current budget, glitch 0. Training object family: cyl r 2-12, boxes 4-16 (aspect <= 3), rounded boxes.
- Length: real robot is 8 modules. L=8 for now (takes off ~2x faster); L=10 needed for cyl r9; L=8 suffices to ~r7. Gate sets are length-dependent (rule: gate_r_max = largest r with >=1 module slack): L=8 = cyl r3, cyl r7, 8x8, 5x14, rounded 8x8 r2; L=10 = r3, r9, 8x8, 5x14, rounded 12x12 r2 (r11 -> r9 and the L=8 set are logged mis-calibrations).
- Actions: absolute EMA targets + a LEARNED HOLD CHANNEL h in [0,1] (ema_eff = 0.3+0.7h; current target in obs). Delta actions failed (no gait discovery). Scripted coil prior REJECTED by owner.
- Flat takeoff: L=8 abs 1.26M steps, with hold channel 1.13M (0.98 success at 2M); L=10 abs 2.75M (0.97 by 4M); L=10 hold channel 0.14 at 2M (marginal).
- Hug pilots (L=8, hold channel, warm from flat): frac_reach 44% -> 51% -> 60%; frac_hold 4.5% -> 7.8% -> 11.2%; h_wrap 0.44 -> 0.66 -> 0.72 for hold_weight 3 -> 6 -> 10. hug_long_01 stopped at its 20-min check; end screen success 0.05, hug_modules_mean 3.85, span 107 deg.
- Hug env speedup attempt (vectorised probe, in-memory buffers, commit 067fa05) did not reach 3k steps/s: hug is ~1.7x costlier per step than flat (physics + Python); stays ~1.7k steps/s.

## In flight
`hug_curr_01`: training-only success curriculum L1 (span>=120, run>=3, hold 1.5 s) -> L2 (150/4/2 s) -> L3 = gate (180/4/3 s), advance at rolling training success >= 0.30, hold_weight 10, constant lr 1.5e-4, continuing hug_long_01, L=8. Checks: 20 min L1 success >= 0.15 & h_wrap >= 0.70; 40 min L2 reached; end n=30 screen at L3 (>=0.55 -> n=100 verdict; 0.30-0.55 -> one more hour; <0.30 stop with per-shape breakdown). Expert expects cylinders to lead and boxes to lag; if boxes stay 0 at L2 change shape mix (phase-1 sizes 70%), not reward.

## Still to do after the hug passes
n=100 verdict with capacitive_off at L=8 -> then terrain (stage B, imu_off ablation) -> obstacle field/barrier with touch_off drop >= 0.20 and blind baseline <= 0.30 (R2: touch use unproven; fallbacks LSTM (sb3-contrib install approved) or the existing linear contact decoder) -> full course E. Replicate the passing recipe at L=10 after D1 passes.
