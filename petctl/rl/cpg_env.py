"""Phase 7 environment: the policy acts in CPG-parameter space (plus motor gains) to track a commanded body-frame velocity.

Action (13, in [-1, 1]): 11 increments of the CPG parameters (``PARAM_NAMES``; each tick the target parameter moves by ``a * step_frac * range`` and is clipped to ``PARAM_BOUNDS``; the CPG's exact
critically damped filter smooths what reaches the joints) and 2 multiplicative gain scales (kp, kd: x ``gain_range`` ** a around the nominal values, set through ``PetSim.set_gains``; kp is
capped at the motor's MIT range). The policy therefore blends and morphs gaits instead of choosing among them, and can vary the gains dynamically (owner decision 2026-11-26).

Observation (deployable on the robot: head IMU + joint encoders only): command (vx, vy, yaw), gravity direction in the head frame, gyro, joint positions / velocities, the current normalised CPG
parameters, phase (sin, cos), the previous action.
Task: command tracking of the 1 s window-averaged COM velocity in the body (mid-chord) heading frame, exp kernel, + yaw-rate tracking; commands change every ``cmd_s`` seconds (stand / forward /
back / left / right / turn / forward+turn). Randomised per episode: start orientation (upright / inverted / side), floor friction x0.7..1.3, the start gait (a random MAP-Elites elite).
Flipping is not a failure. Reward terms are reported in ``info['metrics']`` as ``term_*`` for the watchdog; ``metrics['success']`` is the mean velocity-tracking kernel over the episode.
"""

from __future__ import annotations

import json
import math
from collections import deque
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np


from petctl.config import GIM3505_MIT_KD_MAX, GIM3505_MIT_KP_MAX
from petctl.rl.cpg import CPG, PARAM_BOUNDS, PARAM_NAMES
from petctl.rl.gait_eval import build_sim

LO = np.array([PARAM_BOUNDS[k][0] for k in PARAM_NAMES])
HI = np.array([PARAM_BOUNDS[k][1] for k in PARAM_NAMES])
NP = len(PARAM_NAMES)
ARCHIVES = ("data/rl/map_elites/n{n}_s0_v2.json", "data/rl/map_elites/n{n}_s1_v2.json")


class CPGEnv(gym.Env):  # type: ignore[type-arg]
    """Command-tracking environment in CPG-parameter space (see module docstring)."""

    metadata = {"render_modes": []}

    def __init__(self, n_modules: int = 11, episode_s: float = 24.0, cmd_s: float = 6.0, dt: float = 0.05, step_frac: float = 0.12, res_frac: float = 0.25, gain_range: float = 2.0, smooth_hz: float = 1.0,
                 max_speed: float = 0.18, max_yaw: float = 1.0, randomise: bool = True, use_prior: bool = True, w_vel: float = 1.0, w_yaw: float = 0.5, w_rate: float = 0.02, w_sat: float = 0.25, w_cur: float = 0.02) -> None:
        super().__init__()
        self.n, self.nj, self.dt = n_modules, n_modules - 1, dt
        self.res_frac = res_frac
        self.episode_ticks, self.cmd_ticks, self.step_frac, self.gain_range = int(episode_s / dt), int(cmd_s / dt), step_frac, gain_range
        self.max_speed, self.max_yaw, self.randomise = max_speed, max_yaw, randomise
        self.w = {"vel": w_vel, "yaw": w_yaw, "rate": w_rate, "sat": w_sat, "cur": w_cur}
        self.sim = build_sim(n_modules)
        self.kp0, self.kd0 = (g.copy() for g in self.sim.gains())
        floor = [g for g in range(self.sim.model.ngeom) if self.sim.model.geom_type[g] == 0]
        self.floor_geoms, self.floor_mu0 = floor, self.sim.model.geom_friction[floor, 0].copy()
        self.cpg = CPG(self.nj, smooth_hz=smooth_hz)
        self.use_prior = use_prior
        self.elites, self.elite_v = self._load_elites()
        self.action_space = gym.spaces.Box(-1.0, 1.0, (NP + 2,), np.float32)
        d = 3 + 3 + 3 + 2 * self.nj + NP + 2 + NP + 2
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (d,), np.float32)
        self.chords = [(0, self.n - 1), (self.n // 4, (3 * self.n) // 4), (self.n // 3, (2 * self.n) // 3)]
        self.rng = np.random.default_rng(0)

    def _load_elites(self) -> tuple[list[np.ndarray], np.ndarray]:
        """Library gaits (MAP-Elites archives of this length): parameters and their (vx, vy, yaw rate) so a command can be matched to a gait."""
        out: list[np.ndarray] = []
        vel: list[list[float]] = []
        for pat in ARCHIVES:
            f = Path(pat.format(n=self.n))
            if f.exists():
                for c in json.loads(f.read_text())["cells"]:
                    if c["metrics"]["speed"] > 0.0:
                        out.append(np.array(c["params"]))
                        vel.append([c["metrics"]["vx"], c["metrics"]["vy"], c["metrics"]["yaw_rate"]])
        if not out:
            return [np.array([1.0, 1.0, 1.57, 0.6, 0.6, 0, 0, 0, 0, 0, 0])], np.zeros((1, 3))
        return out, np.array(vel)

    def prior(self, cmd: np.ndarray) -> np.ndarray:
        """Library selector: the elite whose measured (vx, vy, yaw) is closest to the command (a near-zero command -> the quietest elite, amplitudes at their minimum)."""
        if float(np.linalg.norm(cmd)) < 1e-6:
            t = self.elites[0].copy()
            t[[PARAM_NAMES.index("a_yaw"), PARAM_NAMES.index("a_twist"), PARAM_NAMES.index("f")]] = 0.0
            return t
        sc = np.array([self.max_speed, self.max_speed, self.max_yaw])
        i = int(np.argmin(np.linalg.norm((self.elite_v - cmd) / sc, axis=1)))
        return self.elites[i].copy()

    # ---- helpers -------------------------------------------------------------------------------------------------
    def _chord_angles(self) -> np.ndarray:
        p = self.sim.data.xipos[self.sim.m0_id:self.sim.m0_id + self.n]
        return np.array([math.atan2(p[a, 1] - p[b, 1], p[a, 0] - p[b, 0]) for a, b in self.chords])

    def _sample_cmd(self) -> np.ndarray:
        r, mode = self.rng, self.rng.integers(0, 8)
        v, w = self.rng.uniform(0.4, 1.0) * self.max_speed, self.rng.uniform(0.3, 1.0) * self.max_yaw
        return np.array([[0, 0, 0], [v, 0, 0], [-v, 0, 0], [0, v, 0], [0, -v, 0], [0, 0, w], [0, 0, -w], [v, 0, w * r.choice([-1, 1])]][mode], float)

    def _obs(self) -> np.ndarray:
        d, sim = self.sim.data, self.sim
        up = d.site_xmat[sim.imu_id].reshape(3, 3)[2, :]
        pn = 2 * (self.target - LO) / (HI - LO) - 1
        return np.concatenate([self.cmd / np.array([self.max_speed, self.max_speed, self.max_yaw]), up, d.sensor("gyro").data * 0.3, sim.joint_pos(), sim.joint_vel() * 0.1, pn,
                               [math.sin(self.cpg.phase), math.cos(self.cpg.phase)], self.prev_a]).astype(np.float32)

    # ---- gym API -------------------------------------------------------------------------------------------------
    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[np.ndarray, dict[str, Any]]:
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        sim, m = self.sim, self.sim.model
        sim.reset()
        if self.randomise:
            m.geom_friction[self.floor_geoms, 0] = self.floor_mu0 * self.rng.uniform(0.7, 1.3)
            orient = self.rng.choice([0.0, math.pi, math.pi / 2], p=[0.7, 0.15, 0.15])
        else:
            orient = 0.0
        if orient:
            sim.data.qpos[3:7] = [math.cos(orient / 2), math.sin(orient / 2), 0.0, 0.0]
            sim.data.qpos[2] += 0.04
        sim.set_gains(self.kp0, self.kd0)
        sim.step(np.zeros(self.nj), 0.8)
        self.prev_a, self.k, self.cmd = np.zeros(NP + 2, np.float32), 0, self._sample_cmd()
        self.target = self.prior(self.cmd) if (self.use_prior and self.rng.random() < 0.7) else self.elites[self.rng.integers(len(self.elites))].copy()
        self.base = self.target.copy()
        self.cpg.reset(self.target, 0.0)
        self.hist: deque = deque(maxlen=int(1.0 / self.dt) + 1)
        self._push()
        self.h0, self.mask = float(np.median(self._chord_angles())), int(1.0 / self.dt) + 1             # heading frame fixed per command segment; first second masked
        self.sums = {"vel": 0.0, "yaw": 0.0, "rate": 0.0, "sat": 0.0, "cur": 0.0}
        self.n_scored = 0
        self.kernel_sum, self.flips, self.was_up = 0.0, 0, float(sim.data.site_xmat[sim.imu_id].reshape(3, 3)[2, 2]) > 0
        return self._obs(), {}

    def _push(self) -> None:
        d = self.sim.data
        self.hist.append((d.xipos[self.sim.m0_id:self.sim.m0_id + self.n].mean(axis=0).copy(), self._chord_angles()))

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        a = np.clip(np.asarray(action, float), -1.0, 1.0)
        if self.use_prior:                                                                               # residual: target = selector gait + bounded offset (zero action = the library baseline)
            self.target = np.clip(self.base + a[:NP] * self.res_frac * (HI - LO), LO, HI)
        else:
            self.target = np.clip(self.target + a[:NP] * self.step_frac * (HI - LO), LO, HI)
        sc = self.gain_range ** a[NP:]
        self.sim.set_gains(np.minimum(self.kp0 * sc[0], GIM3505_MIT_KP_MAX), np.minimum(self.kd0 * sc[1], GIM3505_MIT_KD_MAX))
        self.sim.step(self.cpg.step(self.dt, self.target), self.dt)
        if not np.all(np.isfinite(self.sim.data.qpos)):
            raise FloatingPointError("simulation diverged (NaN)")
        self._push()
        self.k += 1
        if self.k % self.cmd_ticks == 0:
            self.cmd, self.mask, self.h0 = self._sample_cmd(), int(1.0 / self.dt) + 1, float(np.median(self._chord_angles()))
            if self.use_prior:
                self.base = self.prior(self.cmd)                                                      # selector sets the gait of each segment; the policy adds a bounded residual
        c0, c1 = self.hist[0][0], self.hist[-1][0]
        t = (len(self.hist) - 1) * self.dt
        dv = (c1 - c0)[:2] / t if t > 0 else np.zeros(2)
        h = self.h0
        vb = np.array([math.cos(h) * dv[0] + math.sin(h) * dv[1], -math.sin(h) * dv[0] + math.cos(h) * dv[1]])
        ang = np.unwrap(np.stack([x[1] for x in self.hist]), axis=0)                                   # per-chord unwrapped angle along the whole window
        dchord = (ang[-1] - ang[0]) / t if t > 0 else np.zeros(len(self.chords))
        yaw = float(np.median(dchord)) if float(np.ptp(dchord)) * t < 0.3 else 0.0                       # chords must agree (a shape change is not a turn)
        tau = self.sim.joint_torque()
        terms = {"rate": -self.w["rate"] * float(np.mean((a - self.prev_a) ** 2)), "sat": -self.w["sat"] * float(np.mean(np.abs(tau) >= 0.98 * 0.5)),
                 "cur": -self.w["cur"] * max(0.0, 0.02 * self.nj + 0.25 + 4.6 * float(np.sum(tau ** 2)) - 4.0), "vel": 0.0, "yaw": 0.0}
        score = None
        if self.mask > 0:
            self.mask -= 1
        else:
            sp, sw = np.linalg.norm(self.cmd[:2]), abs(self.cmd[2])
            if sp > 1e-6:                                                                                # dense progress along the command + accuracy bonus (improvement over standing still)
                ch = self.cmd[:2] / sp
                along, side = float(vb @ ch), float(np.linalg.norm(vb - (vb @ ch) * ch))
                sig = max(0.03, 0.4 * sp)
                k0 = math.exp(-sp ** 2 / (2 * sig ** 2))
                acc = (math.exp(-float(np.linalg.norm(vb - self.cmd[:2])) ** 2 / (2 * sig ** 2)) - k0) / (1 - k0)
                terms["vel"] = self.w["vel"] * (0.5 * (min(1.2, max(-1.0, along / sp)) - 0.5 * min(1.0, side / sp)) + 0.5 * acc)
            else:                                                                                        # commanded to stay put: moving is penalised, standing earns 0
                terms["vel"] = -self.w["vel"] * 0.3 * min(1.0, float(np.linalg.norm(vb)) / 0.1)
            if sw > 1e-6:
                sg = max(0.15, 0.4 * sw)
                k0 = math.exp(-sw ** 2 / (2 * sg ** 2))
                accw = (math.exp(-(yaw - self.cmd[2]) ** 2 / (2 * sg ** 2)) - k0) / (1 - k0)
                terms["yaw"] = self.w["yaw"] * (0.5 * min(1.2, max(-1.0, yaw * np.sign(self.cmd[2]) / sw)) + 0.5 * accw)
            else:
                terms["yaw"] = -self.w["yaw"] * 0.3 * min(1.0, abs(yaw) / 0.5)
            self.n_scored += 1
            score = (terms["vel"] / self.w["vel"] * (sp > 1e-6) + terms["yaw"] / self.w["yaw"] * (sw > 1e-6)) / max(1, int(sp > 1e-6) + int(sw > 1e-6))
        for k_, v_ in terms.items():
            self.sums[k_] += v_
        if score is not None:
            self.kernel_sum += score
        up = float(self.sim.data.site_xmat[self.sim.imu_id].reshape(3, 3)[2, 2]) > 0
        self.flips += int(up != self.was_up)
        self.was_up = up
        self.prev_a = a.astype(np.float32)
        done = self.k >= self.episode_ticks
        info: dict[str, Any] = {}
        if done:
            info["metrics"] = {"success": self.kernel_sum / max(self.n_scored, 1), "orient_flips": float(self.flips), **{f"term_{k_}": v_ / self.k for k_, v_ in self.sums.items()}}
        return self._obs(), float(sum(terms.values())), False, done, info
