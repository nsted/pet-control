#!/usr/bin/env python3
"""
Tune reactive PI backstop parameters for the power manager.

Motor 7 must be physically clamped. The script ramps the commanded position
past the clamped position to build a sustained position error, forcing the
motor to draw current trying to close it. The PowerManager should cap
current at budget by smoothly reducing scale.

Trial structure per parameter combination:
  Stabilize (2s) — command at clamped position, PM warms up
  Ramp      (4s) — command ramps away at 2 rad/s (0 → 8 rad error)
  Hold      (9s) — command held at +8 rad error, PM settles — this is scored

Score = std(current) + 2 × mean_overage_above_budget (lower = better).
A low-std, budget-tracking hold phase is the target.

Usage:
    python scripts/tune_power.py
    python scripts/tune_power.py --host pet-robot.local
    python scripts/tune_power.py --delta 8.0   # position error magnitude (rad)
Motor 7 hardware facts (informs default --budget):
  M7 hardware current cap:  ~1.5A
  Robot idle baseline:       ~0.62A
  Max total with M7 pinned: ~2.12A
  Test budget default:        1.5A  (forces PM to cut scale ~50% and hold it)
"""

from __future__ import annotations

import argparse
import asyncio
import datetime
import json
import math
import sys
import time
from typing import NamedTuple

import numpy as np

from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT, RobotBackend
from petctl.config import MOTOR_LIMITS, POWER_BUDGET
from petctl.power_manager import PowerManager
from petctl.types import ServoCommand

MOTOR_ID = 7
TICK_HZ = 30
DT = 1.0 / TICK_HZ

_KP = MOTOR_LIMITS.kp_default    # 0.4
_KD = MOTOR_LIMITS.kd_default    # 0.035

_STABILIZE_S = 2.0   # command at home_pos; PM EMA and wall-detection warm up
_RAMP_S      = 4.0   # ramp from 0 to TARGET_DELTA rad
_HOLD_S      = 9.0   # hold at TARGET_DELTA; this window is scored

# Grid: (ki, ki_decay, recovery_rate)
# First entry is the current config — baseline reference.
# Recovery rate is the primary lever now that predictive scaling is removed.
# Higher recovery = faster response after current drops but more oscillation risk.
# ki_decay controls how long the I-term holds the scale down after a transient.
_GRID: list[tuple[float, float, float]] = [
    (0.5,  3.0, 0.05),   # current config — baseline (slow recovery)
    (0.5,  3.0, 0.20),   # faster recovery, same ki
    (0.5,  3.0, 0.50),
    (0.5,  3.0, 1.00),
    (0.5,  0.3, 0.20),   # low ki_decay (I persists) + faster recovery
    (0.5,  0.3, 0.50),
    (0.5,  0.3, 1.00),
    (2.0,  0.3, 0.20),   # higher ki + low ki_decay + faster recovery
    (2.0,  0.3, 0.50),
    (2.0,  0.3, 1.00),
    (5.0,  0.3, 0.20),
    (5.0,  0.3, 0.50),
    (5.0,  0.3, 1.00),
    (2.0,  0.5, 0.50),
    (5.0,  0.5, 0.50),
]


class TrialResult(NamedTuple):
    ki: float
    ki_decay: float
    recovery_rate: float
    mean_a: float
    std_a: float
    max_a: float
    peak_count: int   # samples above max_peak_current_a
    score: float
    hold_trace: list[float]


def _score(hold: np.ndarray, budget: float, peak_limit: float) -> float:
    std = float(np.std(hold))
    mean_overage = float(np.mean(np.clip(hold - budget, 0.0, None)))
    peak_penalty = float(np.sum(hold > peak_limit)) * 0.5
    return std + 2.0 * mean_overage + peak_penalty


def _cmd(pos: float) -> ServoCommand:
    return ServoCommand(servo_id=MOTOR_ID, position=pos, kp=_KP, kd=_KD, torque_ff=0.0)


def _relax(motor_id: int, pos: float) -> ServoCommand:
    return ServoCommand(servo_id=motor_id, position=pos, kp=0.0, kd=MOTOR_LIMITS.kd_max, torque_ff=0.0)


async def _prompt(msg: str) -> None:
    print(f"\n  >>> {msg} — press Enter to continue... ", end="", flush=True)
    await asyncio.to_thread(sys.stdin.readline)


def _banner(title: str) -> None:
    print(f"\n{'─' * 62}")
    print(f"  {title}")
    print(f"{'─' * 62}")


async def _run_trial(
    backend: RobotBackend,
    home_pos: float,
    target_delta: float,
    relax_cmds: list[ServoCommand],
    ki: float,
    ki_decay: float,
    recovery_rate: float,
    budget: float,
    peak_limit: float,
    trial_num: int,
    total_trials: int,
) -> TrialResult:
    pm = PowerManager(
        reactive_integral_ki=ki,
        reactive_integral_ki_decay=ki_decay,
        reactive_scale_recovery_rate=recovery_rate,
        budget_override=budget,
    )

    n_stab = int(_STABILIZE_S * TICK_HZ)
    n_ramp = int(_RAMP_S      * TICK_HZ)
    n_hold = int(_HOLD_S      * TICK_HZ)
    n_total = n_stab + n_ramp + n_hold

    ramp_rate = target_delta / _RAMP_S   # rad/s
    hold_trace: list[float] = []

    eta_s = int((total_trials - trial_num) * (_STABILIZE_S + _RAMP_S + _HOLD_S))
    print(
        f"  [{trial_num:2d}/{total_trials}]  ki={ki:.1f}  decay={ki_decay:.2f}  "
        f"rcvry={recovery_rate:.3f}  (~{eta_s}s left) ...",
        end="  ", flush=True,
    )

    for tick in range(n_total):
        t0 = time.monotonic()
        state = await backend.get_state()

        # Build commanded position for this tick
        if tick < n_stab:
            cmd_pos = home_pos
        elif tick < n_stab + n_ramp:
            ramp_tick = tick - n_stab
            cmd_pos = home_pos + ramp_rate * (ramp_tick / TICK_HZ)
        else:
            cmd_pos = home_pos + target_delta

        pm.update(state, t0)
        pm.drain_disable_events()
        pm.drain_voltage_cutoff()

        active_cmds, _ = pm.allocate_budget([_cmd(cmd_pos)], state)
        await backend.send_commands(active_cmds + relax_cmds)

        if tick >= n_stab + n_ramp:
            hold_trace.append(state.battery_current_amps)

        await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))

    arr = np.array(hold_trace)
    score = _score(arr, budget, peak_limit)
    peak_count = int(np.sum(arr > peak_limit))

    print(
        f"mean={np.mean(arr):.2f}A  std={np.std(arr):.3f}A  "
        f"max={np.max(arr):.2f}A  score={score:.3f}"
    )

    return TrialResult(
        ki=ki, ki_decay=ki_decay, recovery_rate=recovery_rate,
        mean_a=float(np.mean(arr)), std_a=float(np.std(arr)),
        max_a=float(np.max(arr)), peak_count=peak_count,
        score=score, hold_trace=list(arr),
    )


_IDLE_BASELINE_A = 0.62   # measured robot idle current with all motors relaxed


async def run(host: str, port: int, target_delta: float, budget: float) -> None:
    backend = RobotBackend(host=host, port=port, auto_reconnect=False)
    print(f"Connecting to {host}:{port} ...")
    if not await backend.connect():
        print("Connection failed.")
        return

    motors = backend.discovered_servos
    print(f"Connected. Discovered motors: {sorted(motors)}")
    if MOTOR_ID not in motors:
        print(f"Motor {MOTOR_ID} not found — aborting.")
        await backend.disconnect()
        return

    state0 = await backend.get_state()
    home_pos = state0.servo_positions.get(MOTOR_ID, 0.0)
    v = state0.battery_voltage_v

    # Estimate equilibrium scale: scale where kp×delta torque produces exactly budget-level M7 draw
    b = POWER_BUDGET
    m7_headroom = max(budget - _IDLE_BASELINE_A - b.per_motor_base_a, 0.0)
    tau_budget = math.sqrt(max(m7_headroom, 0.0) / (b.per_motor_torque_coeff * b.bus_voltage_nominal_v / max(v, 8.0)))
    scale_eq = min(tau_budget / (_KP * target_delta), 1.0) if target_delta > 0 else 1.0
    peak_limit = budget * 1.4   # scoring threshold only

    print(f"Motor {MOTOR_ID} home: {math.degrees(home_pos):.1f}°   V_bus: {v:.1f}V")
    print(f"Test budget: {budget:.2f}A   (peak scoring threshold: {peak_limit:.2f}A)")
    print(
        f"Target error: {target_delta:.1f} rad → τ_max = {_KP*target_delta:.2f} Nm"
        f"  |  expected equilibrium scale ≈ {scale_eq:.2f}"
    )
    print(
        f"  Idle baseline ~{_IDLE_BASELINE_A:.2f}A  |  "
        f"M7 headroom {budget - _IDLE_BASELINE_A:.2f}A  |  "
        f"M7 hw cap ~1.5A  |  max total ~{_IDLE_BASELINE_A + 1.5:.2f}A"
    )

    other_ids = [m for m in sorted(motors) if m != MOTOR_ID]
    relax_cmds = [_relax(mid, state0.servo_positions.get(mid, 0.0)) for mid in other_ids]
    if relax_cmds:
        print(f"Relaxing motors {other_ids} (kp=0, kd_max).")

    # 3s warmup: hold motor at home, let firmware settle
    _banner("Warmup (3s)")
    ws = time.monotonic()
    while time.monotonic() - ws < 3.0:
        t0 = time.monotonic()
        await backend.send_commands([_cmd(home_pos)] + relax_cmds)
        await backend.get_state()
        await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))

    trial_s = int(_STABILIZE_S + _RAMP_S + _HOLD_S)
    total_s  = len(_GRID) * trial_s
    print(
        f"\n  {len(_GRID)} trials × {trial_s}s = "
        f"~{total_s // 60}m{total_s % 60}s"
    )
    await _prompt(f"Confirm motor {MOTOR_ID} is clamped and ready")

    results: list[TrialResult] = []
    _banner("Trials")

    try:
        for i, (ki, ki_decay, recovery_rate) in enumerate(_GRID, 1):
            r = await _run_trial(
                backend, home_pos, target_delta, relax_cmds,
                ki, ki_decay, recovery_rate,
                budget, peak_limit,
                trial_num=i, total_trials=len(_GRID),
            )
            results.append(r)
    except KeyboardInterrupt:
        print("\n  Interrupted.")

    if not results:
        await backend.disable_torques()
        await backend.disconnect()
        return

    # Return to home between result display and cleanup
    print("\n  Returning command to home position ...")
    ret_start = time.monotonic()
    while time.monotonic() - ret_start < 1.0:
        t0 = time.monotonic()
        await backend.send_commands([_cmd(home_pos)] + relax_cmds)
        await backend.get_state()
        await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))

    ranked = sorted(results, key=lambda r: r.score)

    _banner("Results (ranked by score — lower = steadier at budget)")
    hdr = (
        f"  {'#':>3}  {'ki':>5}  {'decay':>6}  {'rcvry':>6}"
        f"  {'mean_A':>7}  {'std_A':>7}  {'max_A':>7}  {'peaks':>6}  {'score':>7}"
    )
    print(hdr)
    print("  " + "─" * (len(hdr) - 2))
    for rank, r in enumerate(ranked, 1):
        is_best     = rank == 1
        is_baseline = (r.ki, r.ki_decay, r.recovery_rate) == _GRID[0]
        flag = " ★" if is_best else ("  (baseline)" if is_baseline else "")
        print(
            f"  {rank:>3}  {r.ki:>5.1f}  {r.ki_decay:>6.2f}  {r.recovery_rate:>6.3f}"
            f"  {r.mean_a:>7.3f}  {r.std_a:>7.3f}  {r.max_a:>7.3f}"
            f"  {r.peak_count:>6}  {r.score:>7.3f}{flag}"
        )

    best = ranked[0]
    baseline = next(r for r in results if (r.ki, r.ki_decay, r.recovery_rate) == _GRID[0])
    _banner("Best config — paste into petctl/config.py PowerBudgetConfig")
    print(f"    reactive_integral_ki: float = {best.ki}")
    print(f"    reactive_integral_ki_decay: float = {best.ki_decay}")
    print(f"    reactive_scale_recovery_rate: float = {best.recovery_rate}")
    print(
        f"\n  Score: baseline={baseline.score:.3f}  best={best.score:.3f}"
        f"  improvement={baseline.score - best.score:+.3f}"
    )

    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = f"power_tune_{ts}.jsonl"
    with open(log_path, "w") as f:
        for r in results:
            f.write(json.dumps({
                "ki": r.ki, "ki_decay": r.ki_decay, "recovery_rate": r.recovery_rate,
                "mean_a": r.mean_a, "std_a": r.std_a, "max_a": r.max_a,
                "peak_count": r.peak_count, "score": r.score,
                "hold_trace": r.hold_trace,
            }) + "\n")
    print(f"\n  Raw hold traces → {log_path}")

    # Brake and disconnect
    print("  Braking motor 7 ...")
    brake_start = time.monotonic()
    while time.monotonic() - brake_start < 1.5:
        t0 = time.monotonic()
        await backend.send_commands([ServoCommand(
            servo_id=MOTOR_ID, position=home_pos,
            kp=0.0, kd=MOTOR_LIMITS.kd_max, torque_ff=0.0,
        )])
        await backend.get_state()
        await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))
    await backend.disable_torques()
    await backend.disconnect()
    print("Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tune PowerManager reactive backstop parameters")
    parser.add_argument("--host", default=ROBOT_DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=ROBOT_DEFAULT_PORT)
    parser.add_argument(
        "--delta", type=float, default=8.0,
        help="Position error magnitude in radians (default: 8.0).",
    )
    parser.add_argument(
        "--budget", type=float, default=1.5,
        help="Test budget in amps (default: 1.5). Set below M7 hw cap (~2.12A total) "
             "so the PM must actively regulate. Lower = more headroom for PM to cut.",
    )
    args = parser.parse_args()
    asyncio.run(run(args.host, args.port, args.delta, args.budget))
