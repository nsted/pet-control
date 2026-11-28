"""S2 cells: the six poses, four orientations, settled-pose cache and validity (STAGE_2_GATES, Poses / Orientations).

A cell is (pose, orientation). Entry is scripted (nominal PD, staged order or a 3 s ramp, then a 4 s settle) and its result is the cached start state; the policy never does the entry. Sources are
recorded for the suite manifest. Sim only; the plant is the nominal one (no domain randomisation).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from petctl.rl.gait_eval import build_sim

N_MODULES = 11
NJ = N_MODULES - 1
DT = 0.05
CACHE_DIR = Path("data/rl/s2/cache")
ORIENTS = {"upright": 0, "left": 2, "right": 3, "inverted": 1}           # name -> env ``start_orient`` code (env._apply_start_orient)
ALPHAS = (1.0, 0.9, 0.8, 0.7, 0.6, 0.5)
HOLD_TAU_FRAC = 0.8                                                      # holdability: nominal-PD tau_peak <= 0.8 x derated cap at the L2 worst corner
VALID_E_RAD = 0.1                                                        # a cell is valid if nominal PD settles with E < 0.1 rad and no loss
LOST_RAD = 0.6
POSE_NAMES = ("straight", "head lift", "lattice tall", "lattice mid", "lattice compact", "wrap")
LIFTED = ("head lift", "lattice tall", "lattice mid")                    # poses with a lifted part (yield family)
L2_WORST = {"mass": 1.2, "head_mass": 2.0, "derate": 0.8}


@dataclass
class Pose:
    """A target pose q* (n_joints rad), the staged entry order (or None), and where it came from."""

    name: str
    q: np.ndarray
    order: list[int] | None
    source: dict[str, Any] = field(default_factory=dict)


def _lattice() -> list[dict[str, Any]]:
    return sorted(json.loads(Path("data/rl/pose_graph_n11.json").read_text()), key=lambda r: -r["height_cm"])


def load_poses(alpha: float = 1.0, lattice_skip: dict[str, int] | None = None) -> dict[str, Pose]:
    """The six S2 poses. ``alpha`` scales the head-lift vector; ``lattice_skip[name]`` = how many entries of that lattice slot's sort order were skipped (holdability replacement)."""
    skip = lattice_skip or {}
    out: dict[str, Pose] = {"straight": Pose("straight", np.zeros(NJ), None, {"file": "zeros"})}
    res = json.loads(Path("data/rl/statics/results.json").read_text())
    hl = next(r for r in res if r["task"] == "head_lift" and r["n_modules"] == N_MODULES and r["seed"] == 0)
    out["head lift"] = Pose("head lift", alpha * np.array(hl["q"], float), None, {"file": "data/rl/statics/results.json", "task": "head_lift", "seed": 0, "alpha": alpha})
    lat = _lattice()
    idx = {"lattice tall": 0, "lattice mid": len(lat) // 2}
    for name, base in idx.items():
        i = min(base + skip.get(name, 0), len(lat) - 1)
        out[name] = _lat_pose(name, lat, i)
    ic = min(range(len(lat)), key=lambda j: lat[j]["footprint_cm2"])
    out["lattice compact"] = _lat_pose("lattice compact", lat, ic)
    w = json.loads(Path("data/rl/statics/wrap.json").read_text())
    best = max((e for e in w if e["n_modules"] == N_MODULES), key=lambda e: e["score"])
    out["wrap"] = Pose("wrap", np.array(best["x"][:NJ], float), None, {"file": "data/rl/statics/wrap.json", "radius_cm": best["radius_cm"], "note": "held on the floor without the cylinder"})
    return out


def _lat_pose(name: str, lat: list[dict[str, Any]], i: int) -> Pose:
    r = lat[i]
    return Pose(name, np.array(r["seq"], float) * math.pi / 2, list(r["order"]), {"file": "data/rl/pose_graph_n11.json", "sorted_index": i, "height_cm": r["height_cm"], "footprint_cm2": r["footprint_cm2"]})


def _apply_orient(sim: Any, code: int) -> None:
    """Roll the reset body about its long axis (same convention as ``PetEnv._apply_start_orient``)."""
    if code <= 0:
        return
    roll = {1: math.pi, 2: math.pi / 2, 3: -math.pi / 2}[code]
    sim.data.qpos[3:7] = [math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0]
    sim.data.qpos[2] += 0.04
    mujoco.mj_forward(sim.model, sim.data)


def enter(sim: Any, pose: Pose, orient: int, settle_s: float = 4.0) -> list[np.ndarray]:
    """Scripted entry with the nominal PD (the sim's own gains) and a settle; returns the joint-error samples (rad, per control tick) of the settle phase."""
    q = pose.q
    sim.reset()
    _apply_orient(sim, orient)
    sim.step(np.zeros(NJ), 0.5)
    if pose.order is None:
        for i in range(60):
            sim.step(q * (i + 1) / 60, DT)
    else:
        cur = np.zeros(NJ)
        for j in pose.order:
            a = cur[j]
            for i in range(20):
                cur[j] = a + (q[j] - a) * (i + 1) / 20
                sim.step(cur, DT)
    errs = []
    for _ in range(int(settle_s / DT)):
        sim.step(q, DT)
        errs.append(np.abs(sim.joint_pos() - q))
    return errs


def tau_peak(sim: Any) -> float:
    """Peak applied actuator torque (N.m) of the current state."""
    return float(np.abs(sim.joint_torque()).max())


def settle_cell(pose: Pose, orient_name: str, sim: Any | None = None) -> dict[str, Any]:
    """Enter and settle one cell with the nominal PD; returns the cached state and its validity numbers."""
    sim = sim or build_sim(N_MODULES)
    errs = enter(sim, pose, ORIENTS[orient_name])
    e_last = np.array(errs[-20:])                                          # last second
    E = float(e_last.mean())
    lost = bool(np.array(errs).max() > LOST_RAD and (np.array(errs).max(axis=1) > LOST_RAD).sum() * DT > 1.0 and e_last.max() > LOST_RAD)
    return {"qpos": sim.data.qpos.copy(), "qvel": sim.data.qvel.copy(), "E": E, "lost": lost, "valid": bool(E < VALID_E_RAD and not lost), "tau_peak": tau_peak(sim)}


def holdable(pose: Pose, sim: Any | None = None) -> tuple[bool, float]:
    """Holdability at the L2 worst corner (mass x1.2, head mass x2, derate 0.8): nominal PD settled upright must have tau_peak <= 0.8 x derated cap and no loss."""
    sim = sim or build_sim(N_MODULES)
    m = sim.model
    mass0, fr0 = m.body_mass.copy(), m.actuator_forcerange.copy()
    try:
        m.body_mass[:] = mass0 * L2_WORST["mass"]
        m.body_mass[sim.m0_id] = mass0[sim.m0_id] * L2_WORST["mass"] * L2_WORST["head_mass"]
        m.actuator_forcerange[:] = fr0 * L2_WORST["derate"]
        errs = enter(sim, pose, 0)
        cap_eff = float(m.actuator_forcerange[:, 1].min())
        tp = tau_peak(sim)
        ok = tp <= HOLD_TAU_FRAC * cap_eff and float(np.array(errs[-20:]).max()) < LOST_RAD
        return bool(ok), tp
    finally:
        m.body_mass[:] = mass0
        m.actuator_forcerange[:] = fr0


def choose_poses() -> tuple[dict[str, Pose], dict[str, Any]]:
    """Apply the holdability rule (STAGE_2_GATES Poses): head-lift alpha, lattice replacement by the next sort-order entry, DROP with no passing option. Returns (poses, report)."""
    sim = build_sim(N_MODULES)
    report: dict[str, Any] = {"alpha": None, "dropped": [], "tau_peak": {}}
    poses = load_poses()
    kept: dict[str, Pose] = {}
    for a in ALPHAS:
        p = load_poses(alpha=a)["head lift"]
        ok, tp = holdable(p, sim)
        report["tau_peak"][f"head lift alpha {a}"] = tp
        if ok:
            kept["head lift"], report["alpha"] = p, a
            break
    if "head lift" not in kept:
        report["dropped"].append("head lift")
    n_lat = len(_lattice())
    for name in ("lattice tall", "lattice mid", "lattice compact"):
        for skip in range(n_lat):
            p = load_poses(lattice_skip={name: skip})[name] if name != "lattice compact" else poses[name]
            ok, tp = holdable(p, sim)
            report["tau_peak"][f"{name} skip {skip}"] = tp
            if ok:
                kept[name] = p
                break
            if name == "lattice compact":
                break
        if name not in kept:
            report["dropped"].append(name)
    for name in ("straight", "wrap"):
        ok, tp = holdable(poses[name], sim)
        report["tau_peak"][name] = tp
        if ok:
            kept[name] = poses[name]
        else:
            report["dropped"].append(name)
    return {n: kept[n] for n in POSE_NAMES if n in kept}, report
