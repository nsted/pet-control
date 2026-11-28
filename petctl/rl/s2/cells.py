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
REF_KP, REF_ZETA = 3.2, 1.0                                               # rev. 6: settled cache, validity and holdability use this PD (0.8 was the GL40 setting)
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
    kp0, kd0 = sim.gains()
    sim.set_gains(kp0 * 0 + REF_KP, kd0 * 0 + kd0.mean() * math.sqrt(REF_KP / kp0.mean()) * REF_ZETA)       # reference PD (rev. 6): kp 3.2, zeta_m 1, kd = kd_ref sqrt(kp / kp_ref)
    try:
        return _enter(sim, pose, q, orient, settle_s)
    finally:
        sim.set_gains(kp0, kd0)


def _enter(sim: Any, pose: Pose, q: np.ndarray, orient: int, settle_s: float) -> list[np.ndarray]:
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
    n = int(settle_s / DT)
    sim._s2_tau = 0.0                                                       # peak applied torque over the last 2 s of the settle (rev. 7: not the entry ramp)
    for i in range(n):
        sim.step(q, DT)
        errs.append(np.abs(sim.joint_pos() - q))
        if i >= n - int(2.0 / DT):
            sim._s2_tau = max(sim._s2_tau, float(np.abs(sim.joint_torque()).max()))
    return errs


def tau_peak(sim: Any) -> float:
    """Peak applied actuator torque (N.m) over the last 2 s of the latest ``enter`` settle (falls back to the current state)."""
    return float(getattr(sim, "_s2_tau", np.abs(sim.joint_torque()).max()))


def settle_cell(pose: Pose, orient_name: str, sim: Any | None = None) -> dict[str, Any]:
    """Enter and settle one cell with the nominal PD; returns the cached state and its validity numbers."""
    sim = sim or build_sim(N_MODULES)
    errs = enter(sim, pose, ORIENTS[orient_name])
    e_last = np.array(errs[-20:])                                          # last second
    E = float(e_last.mean())
    lost = bool(np.array(errs).max() > LOST_RAD and (np.array(errs).max(axis=1) > LOST_RAD).sum() * DT > 1.0 and e_last.max() > LOST_RAD)
    return {"qpos": sim.data.qpos.copy(), "qvel": sim.data.qvel.copy(), "E": E, "lost": lost, "valid": bool(E < VALID_E_RAD and not lost), "tau_peak": tau_peak(sim)}


def head_z(sim: Any) -> float:
    """Height (m) of the head module's centre."""
    return float(sim.data.xipos[sim.m0_id, 2])


def corner_check(pose: Pose, orient_name: str, sim: Any, head_mass_x: float = 1.0) -> tuple[bool, float, float]:
    """Holdability of one cell at the L2 DR corner (mass x1.2, derate 0.8; rev. 7: head mass x2 is a load-family disturbance, only ``head_mass_x=2`` as a stress) with the reference PD: (ok, tau_peak, E).
    ok = tau_peak (last 2 s of the settle) <= 0.8 x cap_eff and no loss."""
    m = sim.model
    mass0, fr0 = m.body_mass.copy(), m.actuator_forcerange.copy()
    try:
        m.body_mass[:] = mass0 * L2_WORST["mass"]
        m.body_mass[sim.m0_id] = mass0[sim.m0_id] * L2_WORST["mass"] * head_mass_x
        m.actuator_forcerange[:] = fr0 * L2_WORST["derate"]
        errs = enter(sim, pose, ORIENTS[orient_name])
        cap_eff = float(m.actuator_forcerange[:, 1].min())
        tp, E = tau_peak(sim), float(np.array(errs[-20:]).mean())
        return bool(tp <= HOLD_TAU_FRAC * cap_eff and float(np.array(errs[-20:]).max()) < LOST_RAD), tp, E
    finally:
        m.body_mass[:] = mass0
        m.actuator_forcerange[:] = fr0


def cell_report(pose: Pose, orient_name: str, sim: Any) -> dict[str, Any]:
    """Validity (nominal plant: E < 0.1, no loss) and holdability (L2 corner) of one cell."""
    nom = settle_cell(pose, orient_name, sim)
    ok, tp, Ec = corner_check(pose, orient_name, sim)
    _, tps, Es = corner_check(pose, orient_name, sim, head_mass_x=L2_WORST["head_mass"])
    return {"tau_peak_stress": tps, "E_stress": Es, "pose": pose.name, "orient": orient_name, "E_nominal": nom["E"], "lost_nominal": nom["lost"], "tau_peak_nominal": nom["tau_peak"], "tau_peak_corner": tp, "E_corner": Ec,
            "holdable": ok, "head_z": head_z(sim), "valid": bool(nom["valid"] and ok)}


HEAD_LIFT_MIN_GAIN_M = 0.05
WRAP_MIN_ALPHA = 0.6
HEAD_ALPHAS = (1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2)
WRAP_ALPHAS = (1.0, 0.9, 0.8, 0.7, 0.6)
LATTICE_TALL_MIN_CM = 14.0
LATTICE_MID_CM = (9.0, 13.0)
LATTICE_MID_BELOW_TALL_CM = 3.0


def _scaled(base: Pose, alpha: float) -> Pose:
    return Pose(base.name, alpha * base.q, base.order, {**base.source, "alpha": alpha})


def choose_cells() -> tuple[dict[str, Pose], list[dict[str, Any]], dict[str, Any]]:
    """Apply the rev. 6 pose rules. Returns (poses kept, per-cell reports of the kept poses, summary). A pose survives if one of its cells is valid; alpha (head lift, wrap) is the largest that
    leaves one valid cell (head lift also needs >= 5 cm head-height gain over straight, wrap alpha >= 0.6); lattice poses take the first passing entry of their height band."""
    sim = build_sim(N_MODULES)
    base = load_poses()
    kept: dict[str, Pose] = {}
    reports: list[dict[str, Any]] = []
    summary: dict[str, Any] = {"dropped": [], "alpha": {}}
    straight_reports = [cell_report(base["straight"], o, sim) for o in ORIENTS]
    z_straight = next(r["head_z"] for r in straight_reports if r["orient"] == "upright")
    kept["straight"] = base["straight"]
    reports += straight_reports

    def alpha_search(name: str, alphas: tuple[float, ...], min_gain: float | None = None) -> None:
        for a in alphas:
            p = _scaled(base[name], a)
            reps = [cell_report(p, o, sim) for o in ORIENTS]
            up = next(r for r in reps if r["orient"] == "upright")
            gain_ok = min_gain is None or (up["head_z"] - z_straight) >= min_gain
            if any(r["valid"] for r in reps) and gain_ok:
                kept[name] = p
                reports.extend(reps)
                summary["alpha"][name] = a
                return
            if min_gain is not None and not gain_ok:
                break
        summary["dropped"].append(name)

    alpha_search("head lift", HEAD_ALPHAS, HEAD_LIFT_MIN_GAIN_M)
    if "wrap" in base:
        alpha_search("wrap", WRAP_ALPHAS)
    lat = _lattice()

    def band_pick(name: str, order: list[int]) -> float | None:
        for i in order:
            p = _lat_pose(name, lat, i)
            reps = [cell_report(p, o, sim) for o in ORIENTS]
            if any(r["valid"] for r in reps):
                kept[name] = p
                reports.extend(reps)
                return float(lat[i]["height_cm"])
        summary["dropped"].append(name)
        return None

    tall_h = band_pick("lattice tall", [i for i, r in enumerate(lat) if r["height_cm"] >= LATTICE_TALL_MIN_CM])
    lo, hi = LATTICE_MID_CM
    mid_order = [i for i, r in enumerate(lat) if lo <= r["height_cm"] <= hi and (tall_h is None or r["height_cm"] <= tall_h - LATTICE_MID_BELOW_TALL_CM)]
    band_pick("lattice mid", mid_order)
    band_pick("lattice compact", sorted(range(len(lat)), key=lambda j: lat[j]["footprint_cm2"]))
    ordered = {n: kept[n] for n in POSE_NAMES if n in kept}
    summary["poses"] = list(ordered)
    summary["lifted_with_valid_cell"] = [n for n in LIFTED if n in ordered and any(r["valid"] for r in reports if r["pose"] == n)]
    summary["no_go"] = bool(len(ordered) < 4 or "head lift" not in ordered or len(summary["lifted_with_valid_cell"]) < 2)
    return ordered, reports, summary
