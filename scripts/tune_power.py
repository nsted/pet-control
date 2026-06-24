#!/usr/bin/env python3
"""
Adaptive reactive PI backstop tuner.

Each run loads all previous power_tune_*.jsonl results (schema v3) and fits a
quadratic surrogate in log-parameter space.  The next batch of trials is centred
on the surrogate-predicted optimum with progressively tighter perturbations, so
the search converges across sessions.

On the first run (no history): falls back to a default grid that broadly covers
the (ki, recovery_rate) space.

Schema v3: symmetric integrator (no ki_decay, no dead band).

Trial structure per parameter set:
  Stabilize (2s) — command at clamped position; PM warms up
  Ramp      (4s) — command ramps at 2 rad/s to DELTA rad past clamped pos
  Hold      (9s) — command held at DELTA rad; this window is scored

Score = std(current) + 2 × mean_overage_above_budget (lower = better).

Usage:
    python scripts/tune_power.py
    python scripts/tune_power.py --budget 1.5 --n-trials 15
    python scripts/tune_power.py --seed 42
"""

from __future__ import annotations

import argparse
import asyncio
import datetime
import glob
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

_KP = MOTOR_LIMITS.kp_default
_KD = MOTOR_LIMITS.kd_default

_STABILIZE_S = 2.0
_RAMP_S      = 4.0
_HOLD_S      = 9.0

_IDLE_BASELINE_A = 0.62

_SCHEMA = "v3"   # v2: one-sided I + dead band; v3: symmetric integrator

# Parameter search bounds (actual space; converted to log internally)
# Tuning axes: ki (integrator gain) and recovery_rate (scale rise speed)
_BOUNDS = np.array([
    [0.5,  30.0],   # ki
    [0.1,   5.0],   # recovery_rate
])
_LOG_BOUNDS = np.log(_BOUNDS)

# Default grid for first run (no history)
_DEFAULT_GRID: list[tuple[float, float]] = [
    (5.0,  1.0),
    (5.0,  0.5),
    (5.0,  2.0),
    (10.0, 1.0),
    (10.0, 0.5),
    (10.0, 2.0),
    (2.0,  1.0),
    (2.0,  0.5),
    (15.0, 1.0),
    (15.0, 0.5),
    (3.0,  1.5),
    (7.0,  1.5),
    (5.0,  3.0),
    (10.0, 3.0),
    (2.0,  2.0),
]


# ---------------------------------------------------------------------------
# Surrogate model (2D quadratic in log-parameter space)
# ---------------------------------------------------------------------------

def _poly_features(log_x: np.ndarray) -> np.ndarray:
    """6-coefficient polynomial features for 2D quadratic: [1, x1, x2, x1², x2², x1x2]."""
    x1, x2 = log_x
    return np.array([1.0, x1, x2, x1*x1, x2*x2, x1*x2])


def _fit_surrogate(log_X: np.ndarray, y: np.ndarray, lam: float = 0.05) -> np.ndarray:
    """Ridge-regularised least-squares 2D quadratic fit."""
    F = np.stack([_poly_features(x) for x in log_X])
    FtF = F.T @ F
    reg = np.eye(6) * lam
    reg[0, 0] = 0.0
    return np.linalg.solve(FtF + reg, F.T @ y)


def _surrogate_value(coeffs: np.ndarray, log_x: np.ndarray) -> float:
    return float(coeffs @ _poly_features(log_x))


def _surrogate_grad(coeffs: np.ndarray, log_x: np.ndarray) -> np.ndarray:
    c = coeffs
    x1, x2 = log_x
    return np.array([
        c[1] + 2*c[3]*x1 + c[5]*x2,
        c[2] + 2*c[4]*x2 + c[5]*x1,
    ])


def _surrogate_minimum(coeffs: np.ndarray, x_init: np.ndarray) -> np.ndarray:
    """Find surrogate minimum by gradient descent with bounds clipping."""
    lo, hi = _LOG_BOUNDS[:, 0], _LOG_BOUNDS[:, 1]
    x = np.clip(x_init.copy(), lo, hi)
    lr = 0.05
    for step in range(2000):
        grad = _surrogate_grad(coeffs, x)
        x = np.clip(x - lr * grad, lo, hi)
        lr *= 0.9995
    return x


# ---------------------------------------------------------------------------
# History I/O
# ---------------------------------------------------------------------------

class TrialResult(NamedTuple):
    ki: float
    recovery_rate: float
    mean_a: float
    std_a: float
    max_a: float
    peak_count: int
    temp_peak: int
    score: float
    hold_trace: list[float]


def _load_history(budget: float) -> list[TrialResult]:
    """Load all schema-v3 JSONL files; filter to compatible budget."""
    results: list[TrialResult] = []
    for path in sorted(glob.glob("power_tune_*.jsonl")):
        try:
            with open(path) as f:
                for line in f:
                    d = json.loads(line.strip())
                    if d.get("schema") != _SCHEMA:
                        continue
                    if abs(d.get("budget", budget) - budget) > 0.1:
                        continue
                    results.append(TrialResult(
                        ki=d["ki"], recovery_rate=d["recovery_rate"],
                        mean_a=d["mean_a"], std_a=d["std_a"], max_a=d["max_a"],
                        peak_count=d["peak_count"], temp_peak=d.get("temp_peak", 0),
                        score=d["score"], hold_trace=d.get("hold_trace", []),
                    ))
        except Exception:
            pass
    return results


# ---------------------------------------------------------------------------
# Adaptive trial generation
# ---------------------------------------------------------------------------

def _log_to_params(log_x: np.ndarray) -> tuple[float, float]:
    p = np.exp(log_x)
    return (round(float(p[0]), 2), round(float(p[1]), 3))


def _generate_trials(
    history: list[TrialResult],
    n_trials: int,
    rng: np.random.Generator,
) -> tuple[list[tuple[float, float]], np.ndarray | None, np.ndarray | None]:
    lo, hi = _LOG_BOUNDS[:, 0], _LOG_BOUNDS[:, 1]

    if len(history) < 8:
        return _DEFAULT_GRID[:n_trials], None, None

    log_X = np.array([[math.log(r.ki), math.log(r.recovery_rate)] for r in history])
    scores = np.array([r.score for r in history])

    s_min, s_max = scores.min(), scores.max()
    y_norm = (scores - s_min) / max(s_max - s_min, 1e-6)

    coeffs = _fit_surrogate(log_X, y_norm)

    best_idx = int(np.argmin(scores))
    log_best_obs = log_X[best_idx]
    log_pred_opt = _surrogate_minimum(coeffs, log_best_obs)

    gen = len(history) // n_trials
    sigma_tight  = max(0.10, 0.22 - gen * 0.02)
    sigma_medium = max(0.20, 0.45 - gen * 0.04)
    sigma_wide   = max(0.35, 0.70 - gen * 0.05)

    candidates_log: list[np.ndarray] = [log_pred_opt, log_best_obs]

    for _ in range(5):
        candidates_log.append(np.clip(log_pred_opt + rng.standard_normal(2) * sigma_tight, lo, hi))
    for _ in range(4):
        candidates_log.append(np.clip(log_pred_opt + rng.standard_normal(2) * sigma_medium, lo, hi))
    for _ in range(4):
        candidates_log.append(np.clip(log_best_obs + rng.standard_normal(2) * sigma_wide, lo, hi))

    seen: set[tuple[float, float]] = set()
    unique: list[tuple[float, float]] = []
    for lx in candidates_log:
        t = _log_to_params(lx)
        if t not in seen:
            seen.add(t)
            unique.append(t)

    while len(unique) < n_trials:
        lx = np.clip(log_pred_opt + rng.standard_normal(2) * sigma_wide, lo, hi)
        t = _log_to_params(lx)
        if t not in seen:
            seen.add(t)
            unique.append(t)

    anchors = unique[:2]
    rest = unique[2:n_trials]
    rng.shuffle(rest)
    return anchors + rest, coeffs, log_pred_opt


# ---------------------------------------------------------------------------
# Scoring and hardware helpers
# ---------------------------------------------------------------------------

def _score(hold: np.ndarray, budget: float, peak_limit: float) -> float:
    std = float(np.std(hold))
    mean_overage = float(np.mean(np.clip(hold - budget, 0.0, None)))
    peak_penalty = float(np.sum(hold > peak_limit)) * 0.5
    return std + 2.0 * mean_overage + peak_penalty


def _m7_temp(state) -> int:
    return max(
        state.motor_temperatures.get(MOTOR_ID, 0),
        state.motor_winding_temperatures.get(MOTOR_ID, 0),
    )


def _cmd(pos: float) -> ServoCommand:
    return ServoCommand(servo_id=MOTOR_ID, position=pos, kp=_KP, kd=_KD, torque_ff=0.0)


def _relax(motor_id: int, pos: float) -> ServoCommand:
    return ServoCommand(servo_id=motor_id, position=pos, kp=0.0, kd=MOTOR_LIMITS.kd_max, torque_ff=0.0)


async def _cool_if_needed(
    backend: RobotBackend,
    home_pos: float,
    relax_cmds: list[ServoCommand],
    hot_c: int = 60,
    cool_c: int = 55,
) -> None:
    """If M7 is above hot_c °C, relax all motors and wait until it drops to cool_c °C."""
    state = await backend.get_state()
    temp = _m7_temp(state)
    if temp <= hot_c:
        return
    relax_m7 = ServoCommand(servo_id=MOTOR_ID, position=home_pos, kp=0.0, kd=MOTOR_LIMITS.kd_max, torque_ff=0.0)
    print(f"\n  [COOL] M7 {temp}°C > {hot_c}°C — waiting for ≤{cool_c}°C ...", flush=True)
    while temp > cool_c:
        t0 = time.monotonic()
        await backend.send_commands([relax_m7] + relax_cmds)
        state = await backend.get_state()
        temp = _m7_temp(state)
        print(f"\r  [COOL] M7 temp: {temp:3d}°C  (target ≤{cool_c}°C)  ", end="", flush=True)
        await asyncio.sleep(max(0.0, 2.0 - (time.monotonic() - t0)))
    print(f"\r  [COOL] M7 temp: {temp:3d}°C — resuming.               ")


async def _prompt(msg: str) -> None:
    print(f"\n  >>> {msg} — press Enter to continue... ", end="", flush=True)
    await asyncio.to_thread(sys.stdin.readline)


def _banner(title: str) -> None:
    print(f"\n{'─' * 62}")
    print(f"  {title}")
    print(f"{'─' * 62}")


# ---------------------------------------------------------------------------
# Single trial
# ---------------------------------------------------------------------------

async def _run_trial(
    backend: RobotBackend,
    home_pos: float,
    target_delta: float,
    relax_cmds: list[ServoCommand],
    ki: float,
    recovery_rate: float,
    budget: float,
    peak_limit: float,
    trial_num: int,
    total_trials: int,
) -> TrialResult:
    pm = PowerManager(
        reactive_integral_ki=ki,
        reactive_scale_recovery_rate=recovery_rate,
        budget_override=budget,
    )

    n_stab  = int(_STABILIZE_S * TICK_HZ)
    n_ramp  = int(_RAMP_S      * TICK_HZ)
    n_hold  = int(_HOLD_S      * TICK_HZ)
    n_total = n_stab + n_ramp + n_hold

    ramp_rate = target_delta / _RAMP_S
    hold_trace: list[float] = []
    temp_peak: int = 0

    eta_s = int((total_trials - trial_num) * (_STABILIZE_S + _RAMP_S + _HOLD_S))
    print(
        f"  [{trial_num:2d}/{total_trials}]  ki={ki:.2f}  rcvry={recovery_rate:.3f}"
        f"  (~{eta_s}s left) ...",
        end="  ", flush=True,
    )

    for tick in range(n_total):
        t0 = time.monotonic()
        state = await backend.get_state()

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

        temp_peak = max(temp_peak, _m7_temp(state))

        if tick >= n_stab + n_ramp:
            hold_trace.append(state.battery_current_amps)

        await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))

    arr = np.array(hold_trace)
    score = _score(arr, budget, peak_limit)
    peak_count = int(np.sum(arr > peak_limit))

    print(
        f"mean={np.mean(arr):.3f}A  std={np.std(arr):.3f}A  "
        f"max={np.max(arr):.3f}A  temp={temp_peak}°C  score={score:.4f}"
    )

    return TrialResult(
        ki=ki, recovery_rate=recovery_rate,
        mean_a=float(np.mean(arr)), std_a=float(np.std(arr)),
        max_a=float(np.max(arr)), peak_count=peak_count,
        temp_peak=temp_peak, score=score, hold_trace=list(arr),
    )


# ---------------------------------------------------------------------------
# Results display
# ---------------------------------------------------------------------------

def _print_results(
    all_results: list[TrialResult],
    new_results: list[TrialResult],
    coeffs: np.ndarray | None,
    log_pred_opt: np.ndarray | None,
    budget: float,
    n_show: int = 12,
) -> None:
    ranked_all = sorted(all_results, key=lambda r: r.score)
    best = ranked_all[0]

    gen = len(all_results) // max(len(new_results), 1)
    _banner(f"All results so far — {len(all_results)} total  (generation {gen})")
    hdr = (
        f"  {'#':>3}  {'ki':>6}  {'rcvry':>6}"
        f"  {'mean_A':>7}  {'std_A':>7}  {'max_A':>7}  {'temp°C':>7}  {'score':>7}"
    )
    print(hdr)
    print("  " + "─" * (len(hdr) - 2))
    for rank, r in enumerate(ranked_all[:n_show], 1):
        is_new = r in new_results
        flag = " ★" if rank == 1 else ("  (new)" if is_new else "")
        temp_str = f"{r.temp_peak:>7d}" if r.temp_peak else f"  {'--':>5}"
        print(
            f"  {rank:>3}  {r.ki:>6.2f}  {r.recovery_rate:>6.3f}"
            f"  {r.mean_a:>7.3f}  {r.std_a:>7.3f}  {r.max_a:>7.3f}"
            f"  {temp_str}  {r.score:>7.4f}{flag}"
        )

    if coeffs is not None and log_pred_opt is not None:
        ki_opt, rr_opt = np.exp(log_pred_opt)
        print()
        print(f"  Surrogate optimum:  ki={ki_opt:.2f}  recovery={rr_opt:.3f}")
        print(f"  Best observed:      ki={best.ki:.2f}  recovery={best.recovery_rate:.3f}  → score={best.score:.4f}")
        prev = sorted([r for r in all_results if r not in new_results], key=lambda r: r.score)
        if prev:
            delta = prev[0].score - best.score
            print(f"  Improvement this run: {prev[0].score:.4f} → {best.score:.4f}  ({delta:+.4f})")

    _banner("Best config — paste into petctl/config.py PowerBudgetConfig")
    print(f"    reactive_integral_ki: float = {best.ki}")
    print(f"    reactive_scale_recovery_rate: float = {best.recovery_rate}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

async def run(host: str, port: int, target_delta: float, budget: float, n_trials: int, seed: int | None) -> None:
    rng = np.random.default_rng(seed)

    history = _load_history(budget)
    gen = len(history) // n_trials if n_trials else 0
    print(f"Loaded {len(history)} historical results (generation {gen}).")

    trials, coeffs, log_pred_opt = _generate_trials(history, n_trials, rng)

    if coeffs is None:
        print("No sufficient history — using default grid.")
    else:
        ki_opt, rr_opt = np.exp(log_pred_opt)
        print(f"Surrogate optimum: ki={ki_opt:.2f}  recovery={rr_opt:.3f}")

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

    b = POWER_BUDGET
    m7_headroom = max(budget - _IDLE_BASELINE_A - b.per_motor_base_a, 0.0)
    tau_budget = math.sqrt(max(m7_headroom, 0.0) / (b.per_motor_torque_coeff * b.bus_voltage_nominal_v / max(v, 8.0)))
    scale_eq = min(tau_budget / (_KP * target_delta), 1.0) if target_delta > 0 else 1.0
    peak_limit = budget * 1.4

    print(f"Motor {MOTOR_ID} home: {math.degrees(home_pos):.1f}°   V_bus: {v:.1f}V")
    print(f"Budget: {budget:.2f}A   delta: {target_delta:.1f} rad   peak threshold: {peak_limit:.2f}A")
    print(f"Expected equilibrium scale ≈ {scale_eq:.2f}")

    other_ids = [m for m in sorted(motors) if m != MOTOR_ID]
    relax_cmds = [_relax(mid, state0.servo_positions.get(mid, 0.0)) for mid in other_ids]
    if relax_cmds:
        print(f"Relaxing motors {other_ids} (kp=0, kd_max).")

    _banner("Warmup (3s)")
    ws = time.monotonic()
    while time.monotonic() - ws < 3.0:
        t0 = time.monotonic()
        await backend.send_commands([_cmd(home_pos)] + relax_cmds)
        await backend.get_state()
        await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))

    trial_s = int(_STABILIZE_S + _RAMP_S + _HOLD_S)
    total_s = len(trials) * trial_s
    print(f"\n  {len(trials)} trials × {trial_s}s = ~{total_s // 60}m{total_s % 60}s")
    await _prompt(f"Confirm motor {MOTOR_ID} is clamped and ready")

    new_results: list[TrialResult] = []
    _banner("Trials")

    try:
        for i, (ki, recovery_rate) in enumerate(trials, 1):
            await _cool_if_needed(backend, home_pos, relax_cmds)
            r = await _run_trial(
                backend, home_pos, target_delta, relax_cmds,
                ki, recovery_rate,
                budget, peak_limit,
                trial_num=i, total_trials=len(trials),
            )
            new_results.append(r)
    except KeyboardInterrupt:
        print("\n  Interrupted.")

    if not new_results:
        await backend.disable_torques()
        await backend.disconnect()
        return

    print("\n  Returning command to home position ...")
    ret_start = time.monotonic()
    while time.monotonic() - ret_start < 1.0:
        t0 = time.monotonic()
        await backend.send_commands([_cmd(home_pos)] + relax_cmds)
        await backend.get_state()
        await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))

    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = f"power_tune_{ts}.jsonl"
    with open(log_path, "w") as f:
        for r in new_results:
            f.write(json.dumps({
                "schema": _SCHEMA, "budget": budget,
                "ki": r.ki, "recovery_rate": r.recovery_rate,
                "mean_a": r.mean_a, "std_a": r.std_a, "max_a": r.max_a,
                "peak_count": r.peak_count, "temp_peak": r.temp_peak,
                "score": r.score, "hold_trace": r.hold_trace,
            }) + "\n")
    print(f"\n  Raw hold traces → {log_path}")

    all_results = history + new_results
    if len(all_results) >= 8:
        log_X = np.array([[math.log(r.ki), math.log(r.recovery_rate)] for r in all_results])
        scores = np.array([r.score for r in all_results])
        s_min, s_max = scores.min(), scores.max()
        y_norm = (scores - s_min) / max(s_max - s_min, 1e-6)
        coeffs_final = _fit_surrogate(log_X, y_norm)
        best_idx = int(np.argmin(scores))
        log_pred_opt_final = _surrogate_minimum(coeffs_final, log_X[best_idx])
        _print_results(all_results, new_results, coeffs_final, log_pred_opt_final, budget)
    else:
        _print_results(all_results, new_results, None, None, budget)

    print("\n  Run again to continue converging.")

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
    parser = argparse.ArgumentParser(description="Adaptive power manager parameter tuner")
    parser.add_argument("--host", default=ROBOT_DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=ROBOT_DEFAULT_PORT)
    parser.add_argument("--delta", type=float, default=8.0,
                        help="Position error magnitude in radians (default: 8.0).")
    parser.add_argument("--budget", type=float, default=1.5,
                        help="Test budget in amps (default: 1.5).")
    parser.add_argument("--n-trials", type=int, default=15,
                        help="Trials per run (default: 15).")
    parser.add_argument("--seed", type=int, default=None,
                        help="Random seed for reproducible perturbations.")
    args = parser.parse_args()
    asyncio.run(run(args.host, args.port, args.delta, args.budget, args.n_trials, args.seed))
