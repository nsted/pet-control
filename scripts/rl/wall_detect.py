"""Wall detectability test: can contact with a wall be detected from what the motors report (MIT reply: position, velocity, torque)?

Walks a trained goal-reach policy toward a goal beyond a wall (several wall angles) and with no wall, logs reply-frame signals every control
tick (quantised, noisy) with the ground-truth contact label from MuJoCo, then trains a classifier on a short window of signals and reports
tick-level AUC, detection rate and latency at a fixed false-alarm rate, per signal subset.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import mujoco
import numpy as np

from petctl.rl.env import PetEnv
from petctl.rl.policy import GaitPolicy
from petctl.sim.model_builder import BuildParams

RUN = Path(sys.argv[1] if len(sys.argv) > 1 else "data/rl/curriculum_fast/s5_goal_far_robust/iter_01")
OUT = Path("data/rl/wall_detect")
ANGLES_DEG = [0, 25, -25, 50, -50]
SEEDS = 8
WALL_DIST_M = 0.6
EPISODE_S = 20.0
WINDOW = 10
# Assumed MIT-reply resolution (CubeMars-style ranges; to be checked against the real reply frames).
Q_POS, Q_VEL, Q_TRQ = 25.0 / 65536, 90.0 / 4096, 24.0 / 4096
N_POS, N_VEL, N_TRQ = 0.0, 0.05, 0.02


def quant(x: np.ndarray, q: float) -> np.ndarray:
    return np.round(x / q) * q


def wall_contact(sim) -> bool:
    gid = {sim.model.geom(f"wall{i}").id for i in range(len(sim.p.walls))}
    return any(c.geom1 in gid or c.geom2 in gid for c in sim.data.contact[: sim.data.ncon])


def episode(pol: GaitPolicy, seed: int, angle_deg: float | None) -> dict[str, np.ndarray]:
    cfg = pol.cfg
    cfg.task_params = {"goal_dist_min": 3.0, "goal_dist_max": 3.0, "bearing_deg": 0.0, "n_goals": 0, "start_pose_rad": 0.0}
    cfg.episode_s = EPISODE_S
    cfg.dr.scale = 0.3
    base = BuildParams(urdf_dir=cfg.urdf_dir, dt=cfg.sim_dt)
    if angle_deg is not None:                                   # reset once without the wall to read the head position and heading
        e0 = PetEnv(cfg, base)
        e0.reset(seed=seed)
        h, fwd = e0.task._head(e0.sim), e0.task.heading
        c = h[:2] + WALL_DIST_M * fwd[:2]
        yaw = np.arctan2(fwd[1], fwd[0]) + np.pi / 2 + np.deg2rad(angle_deg)
        base = BuildParams(**{**base.__dict__, "walls": ((float(c[0]), float(c[1]), float(yaw), 1.2),)})
    env = PetEnv(cfg, base)
    obs, _ = env.reset(seed=seed)
    rng = np.random.default_rng(seed + 777)
    rows, contact = [], []
    for _ in range(env.max_steps):
        obs, _, term, trunc, _ = env.step(pol.action(obs))
        s = env.sim
        pos = quant(s.joint_pos() + rng.normal(0, N_POS, 7), Q_POS)
        vel = quant(s.joint_vel() + rng.normal(0, N_VEL, 7), Q_VEL)
        trq = quant(s.joint_torque() + rng.normal(0, N_TRQ, 7), Q_TRQ)
        rows.append(np.concatenate([trq, env._tgt - pos, vel]))
        contact.append(wall_contact(s) if angle_deg is not None else False)
        if term or trunc:
            break
    return {"x": np.array(rows), "y": np.array(contact), "angle": np.full(len(rows), np.nan if angle_deg is None else angle_deg)}


def windows(x: np.ndarray) -> np.ndarray:
    pad = np.vstack([np.repeat(x[:1], WINDOW - 1, 0), x])
    return np.stack([pad[i:i + WINDOW].reshape(-1) for i in range(len(x))])


def main() -> None:
    pol = GaitPolicy(RUN)
    eps = []
    for seed in range(SEEDS):
        for ang in [None] + ANGLES_DEG:
            r = episode(pol, 20000 + seed, ang)
            r["seed"] = seed
            eps.append(r)
            print(f"seed {seed} angle {ang}: {len(r['y'])} ticks, contact ticks {int(r['y'].sum())}", flush=True)
    np.savez(OUT / "episodes.npz", **{f"{i}_{k}": v for i, e in enumerate(eps) for k, v in e.items()})
    from sklearn.ensemble import HistGradientBoostingClassifier
    from sklearn.metrics import roc_auc_score
    chans = {"torque": slice(0, 7), "error": slice(7, 14), "velocity": slice(14, 21)}
    subsets = {"torque only": ["torque"], "tracking error only": ["error"], "velocity only": ["velocity"], "torque+error": ["torque", "error"], "all three": ["torque", "error", "velocity"]}
    train = [e for e in eps if e["seed"] % 2 == 0]
    test = [e for e in eps if e["seed"] % 2 == 1]
    res = {}
    for name, keep in subsets.items():
        idx = np.concatenate([np.arange(21)[chans[k]] for k in keep])
        cols = np.concatenate([idx + 21 * t for t in range(WINDOW)])
        Xtr = np.vstack([windows(e["x"])[:, cols] for e in train]); ytr = np.concatenate([e["y"] for e in train])
        clf = HistGradientBoostingClassifier(max_iter=200, learning_rate=0.1, random_state=0).fit(Xtr, ytr)
        P = [clf.predict_proba(windows(e["x"])[:, cols])[:, 1] for e in test]
        yte = np.concatenate([e["y"] for e in test]); pte = np.concatenate(P)
        auc = roc_auc_score(yte, pte)
        nowall = np.concatenate([p for e, p in zip(test, P) if np.isnan(e["angle"][0])])
        thr = float(np.quantile(nowall, 0.99))                     # 1% false-alarm rate per tick on wall-free episodes
        lat, det = [], 0
        for e, p in zip(test, P):
            if np.isnan(e["angle"][0]) or not e["y"].any():
                continue
            t0 = int(np.argmax(e["y"])); hit = np.where(p[t0:] > thr)[0]
            if len(hit) and hit[0] <= 20:
                det += 1; lat.append(hit[0] / 20.0)
        n_wall = sum(1 for e in test if not np.isnan(e["angle"][0]) and e["y"].any())
        res[name] = {"auc": round(float(auc), 3), "episodes_with_contact": n_wall, "detected_within_1s": det, "median_latency_s": round(float(np.median(lat)), 2) if lat else None, "tick_false_alarm": 0.01}
        print(name, res[name], flush=True)
    res["contact_tick_fraction_wall_eps"] = float(np.mean(np.concatenate([e["y"] for e in eps if not np.isnan(e["angle"][0])])))
    (OUT / "result.json").write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
