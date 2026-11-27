"""Baseline for the pose-HOLDING task (owner 2026-11-27: hold poses despite dynamic conditions, not assume them): how well does a plain PD controller hold a pose under disturbances, at several stiffnesses?

    python scripts/rl/pose_hold_baseline.py --out data/rl/hold/pd_baseline.json
Poses (n = 11): straight, head lift (statics), the tallest and a compact lattice pose (pose_graph), a mid lattice pose. Entry: ramp (or the stored staged order) then 4 s hold. Disturbances act for 3 s after that, then
are released and the pose is observed 3 s more: external push on a module (xfrc), a 0.1 s shove, steady load, support tilt (gravity rotated), added head mass, torque derating (forcerange x0.7). Reported per
(pose, kp scale, disturbance): peak joint error, mean joint error during the disturbance, whether the pose was LOST (any joint > 0.6 rad off for > 1 s) and the recovery error after release.
PD gains: kp = kp0 x scale, kd = kd0 x sqrt(scale) (damping follows stiffness).
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from petctl.rl.gait_eval import build_sim

DT = 0.05


def poses() -> dict[str, tuple[np.ndarray, list[int] | None]]:
    """name -> (joint target vector, staged joint order or None for an all-at-once ramp)."""
    out: dict[str, tuple[np.ndarray, list[int] | None]] = {"straight": (np.zeros(10), None)}
    hl = next(np.array(r["q"]) for r in json.loads(Path("data/rl/statics/results.json").read_text()) if r["task"] == "head_lift" and r["n_modules"] == 11 and r["seed"] == 0)
    out["head lift"] = (hl, None)
    lat = sorted(json.loads(Path("data/rl/pose_graph_n11.json").read_text()), key=lambda r: -r["height_cm"])
    for name, r in (("lattice tall", lat[0]), ("lattice mid", lat[len(lat) // 2]), ("lattice compact", min(lat, key=lambda r: r["footprint_cm2"]))):
        out[name] = (np.array(r["seq"], float) * math.pi / 2, r["order"])
    return out


def enter(sim, q: np.ndarray, order: list[int] | None) -> None:
    """Reset and move into the pose (staged order one joint per second, or a 3 s all-at-once ramp), then settle 4 s."""
    nj = 10
    sim.reset()
    sim.step(np.zeros(nj), 0.5)
    if order is None:
        for i in range(60):
            sim.step(q * (i + 1) / 60, 0.05)
    else:
        cur = np.zeros(nj)
        for j in order:
            a = cur[j]
            for i in range(20):
                cur[j] = a + (q[j] - a) * (i + 1) / 20
                sim.step(cur, 0.05)
    for _ in range(int(4.0 / DT)):
        sim.step(q, DT)


def disturbance_list(m0: int) -> list[dict]:
    """Disturbances: kind, body index offset, force vector (N) or parameter."""
    return [dict(name="push head 2N", body=0, f=(0, 2.0, 0)), dict(name="push head 5N", body=0, f=(0, 5.0, 0)), dict(name="push mid 5N", body=5, f=(0, 5.0, 0)),
            dict(name="load mid 4N down", body=5, f=(0, 0, -4.0)), dict(name="shove head 12N 0.1s", body=0, f=(0, 12.0, 0), dur=0.1),
            dict(name="tilt 10deg", tilt=10.0), dict(name="tilt 20deg", tilt=20.0), dict(name="head mass x3", mass=3.0), dict(name="torque cap x0.7", cap=0.7), dict(name="none")]


def run(sim, q: np.ndarray, order, kp_scale: float, dist: dict) -> dict:
    """One hold episode under one disturbance; returns error statistics."""
    m = sim.model
    kp0, kd0 = sim.gains()
    sim.set_gains(kp0 * kp_scale, kd0 * math.sqrt(kp_scale))
    g0, fr0, mass0 = m.opt.gravity.copy(), m.actuator_forcerange.copy(), m.body_mass.copy()
    enter(sim, q, order)
    d = sim.data
    body = sim.m0_id + dist.get("body", 0)
    t_on = dist.get("dur", 3.0)
    if "tilt" in dist:
        a = math.radians(dist["tilt"])
        m.opt.gravity[:] = [9.81 * math.sin(a), 0.0, -9.81 * math.cos(a)]
    if "mass" in dist:
        m.body_mass[sim.m0_id] = mass0[sim.m0_id] * dist["mass"]
    if "cap" in dist:
        m.actuator_forcerange[:] = fr0 * dist["cap"]
    errs, lost_t, post = [], 0.0, []
    for i in range(int(6.0 / DT)):
        on = i * DT < t_on if "dur" in dist else i * DT < 3.0
        d.xfrc_applied[:] = 0.0
        if "f" in dist and on:
            d.xfrc_applied[body, :3] = dist["f"]
        sim.step(q, DT)
        e = np.abs(sim.joint_pos() - q)
        (errs if i * DT < 3.0 else post).append(e)
        if e.max() > 0.6:
            lost_t += DT
    d.xfrc_applied[:] = 0.0
    m.opt.gravity[:] = g0
    m.actuator_forcerange[:] = fr0
    m.body_mass[:] = mass0
    sim.set_gains(kp0, kd0)
    E, P = np.array(errs), np.array(post)
    return {"peak": float(E.max()), "mean": float(E.mean()), "lost": bool(lost_t > 1.0), "recover": float(P[-10:].mean())}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    sim = build_sim(11)
    res = []
    P = poses()
    for pname, (q, order) in P.items():
        for ks in (1.0, 2.0, 4.0):
            for dist in disturbance_list(sim.m0_id):
                r = run(sim, q, order, ks, dist)
                res.append({"pose": pname, "kp_scale": ks, "dist": dist["name"], **r})
    a.out.write_text(json.dumps(res, indent=1))
    print("pose            kp x   lost-fraction over 10 disturbances | mean peak joint error (rad) | mean 'none' error")
    for pname in P:
        for ks in (1.0, 2.0, 4.0):
            rr = [r for r in res if r["pose"] == pname and r["kp_scale"] == ks]
            none = next(r for r in rr if r["dist"] == "none")
            print(f"{pname:16s} {ks:3.0f}    lost {np.mean([r['lost'] for r in rr]):.2f}   peak {np.mean([r['peak'] for r in rr]):.2f}   none-error {none['mean']:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
