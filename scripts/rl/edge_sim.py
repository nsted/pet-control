"""Edge-avoidance scene (docs/plans/EDGE_AVOIDANCE.md): a finite raised platform, the robust-library selector as the locomotion layer, a controller interface, per-tick labels and episode metrics.

    python scripts/rl/edge_sim.py --episodes 40 --policy wander          # baselines: wander | stand
A controller is an object with ``reset(rng)`` and ``step(obs, priv) -> np.ndarray(3)`` returning the commanded body velocity (vx, vy, yaw rate). ``obs`` holds PROPRIOCEPTION ONLY (joint torques, joint
tracking errors, joint velocities, head-IMU gravity direction and gyro, time); ``priv`` (overhang count, position) exists for oracle baselines and labels, a deployable controller must not read it.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

import numpy as np

from petctl.rl.cpg_env import CPGEnv
from petctl.sim.model_builder import BuildParams, PetSim

CAP = 0.5


@dataclass
class EpisodeLog:
    """Per-episode record: metrics plus per-tick arrays for detector training."""

    fell: bool = False
    t_fall: float = float("nan")
    seconds: float = 0.0
    path_m: float = 0.0
    cells: int = 0
    moving_frac: float = 0.0
    ticks: dict[str, list] = field(default_factory=lambda: {k: [] for k in ("tau", "err", "qd", "up", "gyro", "overhang", "dist_edge", "phase", "params")})


class Scene:
    """Platform + selector + CPG; ``run`` plays one episode with a controller."""

    def __init__(self, n: int = 11, height: float = 0.25, half: float = 0.75) -> None:
        self.n, self.nj, self.H, self.half = n, n - 1, height, half
        self.env = CPGEnv(n_modules=n, randomise=False, use_prior=True, scale_f=True, episode_s=1e4, cmd_s=1e6, mask_s=0.0)
        obs = (("box", 0.0, 0.0, 0.0, half, half),)
        self.sim = PetSim(BuildParams(n_modules=n, cap_nm=(CAP,) * (n - 1), dt=0.004, obstacles=obs, extra={"obstacle_half_h": height / 2}))
        e = self.env
        e.sim = self.sim
        e.floor_geoms = [g for g in range(self.sim.model.ngeom) if self.sim.model.geom_bodyid[g] == 0 and self.sim.model.geom_type[g] == 0]
        e.floor_mu0 = self.sim.model.geom_friction[e.floor_geoms, 0].copy()
        e.kp0, e.kd0 = (g.copy() for g in self.sim.gains())
        e.last_tgt = np.zeros(self.nj)
        orig = e.cpg.step

        def stepper(dt, tgt):
            e.last_tgt = orig(dt, tgt)                                                              # remember the joint targets so the tracking error is observable
            return e.last_tgt
        e.cpg.step = stepper

    def positions(self) -> np.ndarray:
        s = self.sim
        return s.data.xipos[s.m0_id:s.m0_id + self.n]

    def run(self, controller, seed: int, seconds: float = 60.0, record: bool = False, frame_cb=None, start: tuple | None = None, orient: float = 0.0) -> EpisodeLog:
        """One episode. ``start`` = (x, y, yaw) overrides the random start; ``frame_cb(scene, t, mode)`` is called every tick (video)."""
        e, sim, d = self.env, self.sim, self.sim.data
        rng = np.random.default_rng(seed)
        e.reset(seed=seed)
        mu = float(rng.uniform(1.0, 1.3))
        sim.model.geom_friction[e.floor_geoms, 0] = e.floor_mu0 * mu
        x, y, yaw = start if start is not None else (rng.uniform(-0.35, 0.35), rng.uniform(-0.35, 0.35), rng.uniform(-math.pi, math.pi))
        d.qpos[0], d.qpos[1], d.qpos[2] = x, y, d.qpos[2] + self.H + 0.004
        qh = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
        if orient:                                                                                  # roll about the long axis, then yaw
            qr = [math.cos(orient / 2), math.sin(orient / 2), 0.0, 0.0]
            qh = [qh[0] * qr[0] - qh[3] * qr[3], qh[0] * qr[1] + qh[3] * qr[2], qh[0] * qr[2] - qh[3] * qr[1], qh[0] * qr[3] + qh[3] * qr[0]]
        d.qpos[3:7] = qh
        sim.step(np.zeros(self.nj), 0.8)
        e.hist.clear()
        e._push()
        controller.reset(rng)
        log = EpisodeLog()
        cmd_prev, seen, moved = None, set(), []
        c_prev = self.positions().mean(axis=0)[:2].copy()
        z_ref = self.H
        for i in range(int(seconds / e.dt)):
            t = i * e.dt
            tau, q, qd = sim.joint_torque().copy(), sim.joint_pos().copy(), sim.joint_vel().copy()
            up = d.site_xmat[sim.imu_id].reshape(3, 3)[2, :].copy()
            gyro = d.sensor("gyro").data.copy()
            p = self.positions()
            outside = (np.abs(p[:, 0]) > self.half) | (np.abs(p[:, 1]) > self.half)
            com = p.mean(axis=0)
            dist_edge = float(self.half - max(abs(com[0]), abs(com[1])))
            priv = {"overhang": int(outside.sum()), "pos": com.copy(), "dist_edge": dist_edge}
            obs = {"tau": tau, "q": q, "qd": qd, "err": e.last_tgt - q, "up": up, "gyro": gyro, "t": t, "phase": np.array([math.sin(e.cpg.phase), math.cos(e.cpg.phase)]), "params": e.cpg.x.copy()}
            cmd = np.asarray(controller.step(obs, priv), float)
            if cmd_prev is None or not np.allclose(cmd, cmd_prev):
                e.cmd, e.base = cmd, e.prior(cmd)
                e.target = e.base.copy()
                if getattr(controller, "snap", False):                                                  # reflex: jump the gait parameters instantly (no 1 Hz smoothing)
                    e.cpg.x = e.target.copy()
                    e.cpg.v = np.zeros_like(e.cpg.x)
                cmd_prev = cmd.copy()
            if record:
                for k, v in (("tau", tau), ("err", e.last_tgt - q), ("qd", qd), ("up", up), ("gyro", gyro), ("overhang", priv["overhang"]), ("dist_edge", dist_edge), ("phase", np.array([math.sin(e.cpg.phase), math.cos(e.cpg.phase)])), ("params", e.cpg.x.copy())):
                    log.ticks[k].append(v)
            _o, _r, term, _tr, _inf = e.step(np.zeros(13))
            if frame_cb is not None:
                frame_cb(self, t, getattr(controller, "mode", "LOCOMOTE"))
            p = self.positions()
            c = p.mean(axis=0)
            log.path_m += float(np.linalg.norm(c[:2] - c_prev))
            moved.append(float(np.linalg.norm(c[:2] - c_prev)) / e.dt)
            c_prev = c[:2].copy()
            seen.add((int(math.floor(c[0] / 0.25)), int(math.floor(c[1] / 0.25))))
            log.seconds = t + e.dt
            if p[:, 2].min() < self.H - 0.08 or term:
                log.fell, log.t_fall = True, t
                break
        log.cells = len(seen)
        w = int(2.0 / e.dt)
        mv = np.array(moved)
        log.moving_frac = float(np.mean([mv[i:i + w].mean() > 0.03 for i in range(0, max(len(mv) - w, 1), w)])) if len(mv) > w else 0.0
        return log


class Wander:
    """Unprotected wanderer: forward most of the time, with turns / sidesteps / reversing; no edge sensing."""

    PLAN = [((0.14, 0.0, 0.0), 0.5), ((0.0, 0.0, 0.5), 0.15), ((0.0, 0.0, -0.5), 0.15), ((-0.14, 0.0, 0.0), 0.08), ((0.0, 0.14, 0.0), 0.06), ((0.0, -0.14, 0.0), 0.06)]
    mode = "LOCOMOTE"

    def reset(self, rng) -> None:
        self.rng, self.t_next, self.cmd = rng, 0.0, np.zeros(3)

    def step(self, obs: dict, priv: dict) -> np.ndarray:
        if obs["t"] >= self.t_next:
            probs = np.array([p for _, p in self.PLAN])
            self.cmd = np.array(self.PLAN[int(self.rng.choice(len(self.PLAN), p=probs / probs.sum()))][0], float)
            self.t_next = obs["t"] + float(self.rng.uniform(3.0, 6.0))
        return self.cmd


class Stand:
    """Never moves."""

    mode = "STAND"

    def reset(self, rng) -> None:
        pass

    def step(self, obs: dict, priv: dict) -> np.ndarray:
        return np.zeros(3)


def summarize(name: str, logs: list[EpisodeLog]) -> dict:
    """Aggregate metrics."""
    out = {"policy": name, "episodes": len(logs), "fall_rate": float(np.mean([l.fell for l in logs])), "path_m": float(np.mean([l.path_m / max(l.seconds, 1e-6) * 60.0 for l in logs])),
           "cells": float(np.mean([l.cells for l in logs])), "moving_frac": float(np.mean([l.moving_frac for l in logs]))}
    print(f"{name:14s} episodes {out['episodes']}  fall rate {out['fall_rate']:.2f}  path {out['path_m']:.1f} m/min  cells {out['cells']:.1f}  moving {out['moving_frac']:.2f}", flush=True)
    return out


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=40)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--policy", choices=("wander", "stand"), default="wander")
    a = ap.parse_args()
    sc = Scene()
    ctl = Wander() if a.policy == "wander" else Stand()
    summarize(a.policy, [sc.run(ctl, 1000 + s, a.seconds) for s in range(a.episodes)])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
