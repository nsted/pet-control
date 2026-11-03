"""Replay a capture_all.py run through the MuJoCo model and compare simulated vs measured joint angle and torque.

Usage: python scripts/replay_mujoco.py data/capture/capture_robot_YYYYmmdd_HHMMSS [--sweep] [--const-cap]

The logged command (``cmd``, offsets from the hanging pose), delayed by ``--delay``, drives the simulated PD law; the
power manager's logged per-motor torque cap is applied tick by tick (unless --const-cap). Reports per-joint RMS error and
correlation of angle and torque, overall and per phase class. --sweep grids delay x armature x cap mode.
"""
from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path

import numpy as np

from petctl.sim.mujoco_model import N_JOINTS, PEAK_TORQUE_NM, SimParams, Simulator

PHASES = {"baseline": ("A_baseline",), "ramp": ("ramp",), "step": ("step",), "chirp": ("chirp",), "fast": ("fast",),
          "pair": ("pair",), "wave": ("wave",), "pose": ("pose",)}


def replay(d: dict[str, np.ndarray], p: SimParams, const_cap: bool, max_ticks: int | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Return simulated (theta, tau) at every logged tick."""
    t = d["t"] - d["t"][0]
    n = len(t) if max_ticks is None else min(len(t), max_ticks)
    sim = Simulator(p)
    sim.reset(d["theta"][0])
    th, tau = np.zeros((n, N_JOINTS)), np.zeros((n, N_JOINTS))
    cmd = d["cmd"]
    cap = np.where(np.isnan(d["cap"]), PEAK_TORQUE_NM, np.minimum(d["cap"], PEAK_TORQUE_NM))
    for i in range(n):
        td = t[i] - p.delay_s
        c = np.array([np.interp(td, t, cmd[:, j]) for j in range(N_JOINTS)]) if td > 0 else np.zeros(N_JOINTS)
        sim.step_to(c, np.full(N_JOINTS, PEAK_TORQUE_NM) if const_cap else cap[i], (t[i + 1] - t[i]) if i + 1 < n else 0.01)
        th[i], tau[i] = sim.data.qpos, sim.last_tau
    return th, tau


def stats(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """RMS error and correlation of a vs b."""
    if len(a) < 5 or np.std(a) < 1e-9 or np.std(b) < 1e-9:
        return float(np.sqrt(np.mean((a - b) ** 2))) if len(a) else np.nan, np.nan
    return float(np.sqrt(np.mean((a - b) ** 2))), float(np.corrcoef(a, b)[0, 1])


def phase_slices(t: np.ndarray, marks: list[dict], prefixes: tuple[str, ...]) -> np.ndarray:
    idx = np.zeros(len(t), bool)
    for m in marks:
        if "t1" in m and m["name"].startswith(prefixes):
            idx |= (t >= m["t0"]) & (t <= m["t1"])
    return idx


def report(d: dict[str, np.ndarray], marks: list[dict], th: np.ndarray, tau: np.ndarray, label: str) -> float:
    n = len(th)
    print(f"\n== {label}")
    print("| joint | angle RMS rad | angle corr | torque RMS N.m | torque corr | (zero-model: angle RMS if theta=cmd) |")
    print("|---|---|---|---|---|---|")
    tot = 0.0
    for j in range(N_JOINTS):
        ea, ca = stats(th[:, j], d["theta"][:n, j])
        et, ct = stats(tau[:, j], d["tau"][:n, j])
        e0, _ = stats(d["cmd"][:n, j], d["theta"][:n, j])
        tot += ea
        print(f"| {j + 1} | {ea:.3f} | {ca:.2f} | {et:.3f} | {ct:.2f} | {e0:.3f} |")
    print("\nAngle RMS (rad) / torque RMS (N.m) by phase, averaged over joints:")
    for name, pre in PHASES.items():
        s = phase_slices(d["t"][:n], marks, pre)
        if s.sum() > 5:
            print(f"  {name:9s} {np.mean([stats(th[s, j], d['theta'][:n][s, j])[0] for j in range(N_JOINTS)]):.3f} / "
                  f"{np.mean([stats(tau[s, j], d['tau'][:n][s, j])[0] for j in range(N_JOINTS)]):.3f}")
    return tot / N_JOINTS


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("base")
    ap.add_argument("--delay", type=float, default=0.08)
    ap.add_argument("--armature", type=float, default=3e-4)
    ap.add_argument("--const-cap", action="store_true")
    ap.add_argument("--sweep", action="store_true")
    a = ap.parse_args()
    z = np.load(f"{a.base}.npz")
    d = {k: z[k] for k in z.files}
    marks = json.loads(Path(f"{a.base}.json").read_text())["marks"]
    if a.sweep:
        res = []
        for delay, arm, const in itertools.product((0.0, 0.04, 0.08, 0.12, 0.16), (1e-4, 3e-4, 1e-3), (False, True)):
            th, tau = replay(d, SimParams(delay_s=delay, armature=arm), const)
            ea = np.mean([stats(th[:, j], d["theta"][:, j])[0] for j in range(N_JOINTS)])
            et = np.mean([stats(tau[:, j], d["tau"][:, j])[0] for j in range(N_JOINTS)])
            res.append((ea, et, delay, arm, const))
            print(f"delay {delay:.2f} armature {arm:.0e} const_cap {const!s:5}: angle RMS {ea:.3f} rad, torque RMS {et:.4f} N.m", flush=True)
        ea, et, delay, arm, const = min(res)
        print(f"\nbest by angle RMS: delay {delay} armature {arm} const_cap {const}")
        a.delay, a.armature, a.const_cap = delay, arm, const
    th, tau = replay(d, SimParams(delay_s=a.delay, armature=a.armature), a.const_cap)
    report(d, marks, th, tau, f"delay {a.delay}s armature {a.armature} const_cap={a.const_cap}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
