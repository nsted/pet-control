"""Gymnasium environment for PET on the MuJoCo ground model. The action mapping and observation match deployment (see obs.py)."""

from __future__ import annotations

from collections import deque
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from petctl.config import REGIMEN_BUS_MEAN_A, REGIMEN_BUS_PEAK_A
from petctl.rl.capacitive import CapacitiveSensor, PostProbe, sample_cap_params
from petctl.rl.config import BUS_A_PER_NM2, EnvConfig
from petctl.rl.domain_rand import _u, sample_params
from petctl.rl.layout_cache import LayoutCache, uses_cache
from petctl.rl.obs import ObsBuilder, Snapshot
from petctl.rl.tasks import TASKS
from petctl.sim.bus import BusClamp
from petctl.sim.model_builder import POST_RADIUS_M, BuildParams, PetSim
from petctl.sim.terrain import START_RADIUS_M, generate_terrain

SETTLE_S = 0.75
BUS_MEAN_WINDOW_S = 2.0                # running-mean window of the bus-current penalty (plan sec 5.5)


def _qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


REPLY_TORQUE_NOISE_NM = 0.02          # assumed noise on the torque in the motor reply (to be checked against real frames)
REPLY_TORQUE_Q_NM = 24.0 / 4096        # assumed 12-bit torque field over +-12 N.m


def quantise(x: np.ndarray, q: float) -> np.ndarray:
    """Round ``x`` to the nearest multiple of ``q``."""
    return np.round(x / q) * q


class PetEnv(gym.Env):  # type: ignore[type-arg]
    """Action: ``n_joints`` values (7 for the real 8-module robot) in [-1, 1] -> joint-angle targets (rad) about the zero pose, EMA-filtered and slew-limited like the deployed path."""

    metadata = {"render_modes": []}

    def __init__(self, cfg: EnvConfig | None = None, base: BuildParams | None = None) -> None:
        """Create the env; ``base`` overrides nominal plant parameters (e.g. weighed masses)."""
        self.cfg = cfg or EnvConfig()
        if base is None:
            base = BuildParams(urdf_dir=self.cfg.urdf_dir, dt=self.cfg.sim_dt, n_modules=self.cfg.n_modules)
        elif base.n_modules != self.cfg.n_modules:
            raise ValueError(f"base BuildParams has {base.n_modules} modules but EnvConfig.n_modules is {self.cfg.n_modules}")
        self.base = base
        self.dt = 1.0 / self.cfg.control_hz
        self.obs_builder = ObsBuilder(self.cfg)
        self.observation_space = spaces.Box(-np.inf, np.inf, (self.obs_builder.size,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (self.cfg.n_joints,), np.float32)
        self.max_steps = int(round(self.cfg.episode_s * self.cfg.control_hz))
        self.sim: PetSim | None = None
        self._cap_out: tuple[np.ndarray, np.ndarray] | None = None
        self.cap_sensor: CapacitiveSensor | None = None
        self._reply_mult, self._reply_buf = 1.0, None
        self.bus: BusClamp | None = None

    # ------------------------------------------------------------------ helpers
    def _snapshot(self) -> Snapshot:
        assert self.sim is not None
        d, rng = self.sim.data, self._rng
        dr = self.cfg.dr
        pos = self.sim.joint_pos() + self._joint_off + rng.normal(0, dr.pos_noise_rad * dr.scale, self.cfg.n_joints)
        quat = _qmul(self._tilt, d.sensor("quat").data.copy())
        gyro = d.sensor("gyro").data + rng.normal(0, dr.gyro_noise * dr.scale, 3)
        trq = quantise(self.sim.joint_torque() + rng.normal(0, REPLY_TORQUE_NOISE_NM * self._reply_mult, self.cfg.n_joints), REPLY_TORQUE_Q_NM)       # MIT reply frame: noisy, coarsely quantised
        if self._reply_buf is not None:                                               # reply-frame DR (knob reply_dr): delay of 1-2 ticks, dropped frames hold the last delivered value
            if not self._reply_buf:
                self._reply_buf.extend([trq] * self._reply_buf.maxlen)               # type: ignore[arg-type]
                self._reply_last = trq
            self._reply_buf.append(trq)
            out = self._reply_buf[0]
            trq = self._reply_last if self._reply_rng.random() < self._reply_drop else out
            self._reply_last = trq
        return Snapshot(pos, quat, gyro, self.sim.model.actuator_forcerange[:, 1].copy(), self.task.extra_obs(self.sim), trq, self._tgt.copy(),
                        None if self._cap_out is None else np.concatenate([self._cap_out[0].reshape(-1), self._cap_out[1].reshape(-1)]))

    def _init_reply_dr(self) -> None:
        """Per-episode reply-frame DR (task knob ``reply_dr``, assumption A3): torque-noise multiplier x0.5-2, 1-2 tick delay, 5% dropped frames, all scaled by ``dr.scale``."""
        self._reply_mult, self._reply_buf = 1.0, None
        if self.task.param("reply_dr", 0.0) <= 0.0:
            return
        dr, s = self.cfg.dr, self.cfg.dr.scale
        r = self._reply_rng = np.random.default_rng(int(self._rng.integers(2 ** 31 - 1)))        # separate stream: knob off leaves the env rng untouched
        lo, hi = dr.reply_noise_mult
        self._reply_mult = float(np.exp(s * r.uniform(np.log(lo), np.log(hi))))
        delay = int(round(_u(r, *dr.reply_delay_ticks, s, mid=dr.reply_delay_ticks[0])))
        self._reply_drop = dr.reply_drop * s
        self._reply_buf = deque(maxlen=delay + 1)
        self._reply_last = np.zeros(self.cfg.n_joints)

    def _nominal_head(self) -> tuple[np.ndarray, np.ndarray]:
        """Head position and forward direction (floor plane) of the robot lying at the zero pose; where tasks lay out world geometry."""
        if getattr(self, "_nom", None) is None:
            s = PetSim(self.base)
            fwd = s.data.site_xmat[s.model.site("head").id].reshape(3, 3)[:, 0].copy()
            fwd[2] = 0.0
            self._nom = (s.head_pos().copy(), fwd / max(np.linalg.norm(fwd), 1e-9))
            self._foot = s._foot
        return self._nom

    def _layout(self, seed: int | None) -> tuple[tuple, np.ndarray | None]:
        """Static geometry for this episode: from the layout cache for training resets when ``layout_cache`` > 0, else sampled from the env rng (always for gate seeds)."""
        head, heading = self._nominal_head()
        if self.task.cacheable and self.task.param("layout_cache", 0.0) > 0 and uses_cache(seed):
            cache = getattr(self, "_cache", None)
            if cache is None:
                cache = self._cache = LayoutCache(self.cfg, head, heading)

            def make(rng: np.random.Generator) -> tuple[dict[str, object], np.ndarray]:
                t = TASKS[self.cfg.task](self.cfg)
                _, goal = t.layout(head, heading, rng)
                return t.layout_state(), goal  # type: ignore[attr-defined, return-value]

            state, goal = cache.get(int(self._rng.integers(cache.n)), make)
            return self.task.restore_layout(state, goal)  # type: ignore[attr-defined, no-any-return]
        return self.task.layout(head, heading, self._rng)

    def privileged_dim(self) -> int:
        """Length of ``privileged_vector()`` for this env's module count: ``17 + 7 n_modules``."""
        return 17 + 7 * self.cfg.n_modules

    def privileged_vector(self) -> np.ndarray:
        """Flat privileged state for an asymmetric critic (never observed by the actor; plan sec 2 "Privileged" list; blocks absent in this episode read zero).

        Layout: plant (kp, delay_s, armature, coulomb_scale, cap_nm, floor_friction) 6 | terrain level, height under head, under CoM 3 | post present, head-to-post
        dx, dy 3 | per-module distance to the post surface n | ground-truth face touch 3n | face normal force 3n | distance-field value at head, its gradient (2),
        value at CoM 4 | last-tick bus current 1.
        """
        assert self.sim is not None
        n, sim = self.cfg.n_modules, self.sim
        pv = self.privileged
        head, com = sim.head_pos(), sim.data.subtree_com[1].copy()
        ter = sim.p.terrain
        th = (0.0, 0.0, 0.0) if ter is None else (float(pv.get("terrain_level", 0.0)), float(ter.height_at(head[0], head[1])), float(ter.height_at(com[0], com[1])))
        post = np.zeros(3 + n + 6 * n)
        probe = PostProbe.of(sim)
        if sim.p.post is not None and probe is not None:
            px, py = sim.p.post
            post[:3] = (1.0, px - head[0], py - head[1])
            post[3:3 + n] = np.maximum(np.linalg.norm(sim.data.xipos[1:n + 1, :2] - np.array([px, py]), axis=1) - POST_RADIUS_M, 0.0)
            r = probe.read()
            post[3 + n:3 + 4 * n], post[3 + 4 * n:] = r.touch.reshape(-1).astype(float), r.force.reshape(-1)
        fld = np.zeros(4)
        field = getattr(self.task, "_field", None)
        if field is not None:
            g = field.direction(head)
            fld[:] = (field.value(head), g[0], g[1], field.value(com))
        bus = float(self._bus_means[-1]) if self.bus is not None and self._bus_means else 0.0
        plant = [float(pv.get(k, 0.0)) for k in ("kp", "delay_s", "armature", "coulomb_scale", "cap_nm", "floor_friction")]
        return np.concatenate([plant, th, post, fld, [bus]]).astype(np.float32)

    def _up_dot(self) -> float:
        assert self.sim is not None
        return float(self.sim.data.site_xmat[self.sim.model.site("imu").id].reshape(3, 3)[2, 2])

    # ------------------------------------------------------------------ gym API
    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[np.ndarray, dict[str, Any]]:
        """Randomise the plant, settle the robot on the floor, and return the first observation."""
        super().reset(seed=seed)
        self._rng = self.np_random
        params, self.privileged = sample_params(self.cfg.dr, self._rng, self.base)
        self.task = TASKS[self.cfg.task](self.cfg)
        walls, goal_xy = self._layout(seed)                                           # static geometry (walls) must exist before the sim is built
        obstacles = getattr(self.task, "obstacles", ())
        if walls or obstacles:
            params = BuildParams(**{**params.__dict__, "walls": walls, "obstacles": tuple(obstacles)})
        if self.task.param("post", 0.0) > 0.0:                                         # heavy immovable target post on the goal point
            if goal_xy is None:
                raise ValueError(f"task '{self.cfg.task}' lays out no target, so it cannot have a post")
            params = BuildParams(**{**params.__dict__, "post": (float(goal_xy[0]), float(goal_xy[1]))})
        level = self.task.param("terrain_level", 0.0)
        if level > 0.0:                                                                 # per-episode heightfield terrain (flat start disk covers the body)
            tseed = int(self._rng.integers(2 ** 31 - 1))
            terrain = generate_terrain(level, tseed, max(START_RADIUS_M, self._foot))
            params = BuildParams(**{**params.__dict__, "terrain": terrain})
            self.privileged["terrain_level"], self.privileged["terrain_seed"] = float(level), float(tseed)
        self.sim = PetSim(params)
        dr = self.cfg.dr
        self._joint_off = self._rng.uniform(-dr.joint_offset_rad, dr.joint_offset_rad, self.cfg.n_joints) * dr.scale
        ax = self._rng.normal(size=3)
        ang = self._rng.uniform(0, dr.imu_tilt_rad * dr.scale)
        self._tilt = np.concatenate([[np.cos(ang / 2)], np.sin(ang / 2) * ax / np.linalg.norm(ax)])
        amp = float(self.cfg.task_params.get("start_pose_rad", 0.0))          # random start pose: each joint uniform in +-amp (rad)
        q0 = np.zeros(self.cfg.n_joints)
        for _ in range(8 if amp > 0 else 1):                                  # resample start poses that topple while settling (episode would end at once)
            if amp > 0:
                q0 = self._rng.uniform(-amp, amp, self.cfg.n_joints)
                self.sim.reset(q0)
            self.sim.step(q0, SETTLE_S)                                       # hold the start pose while the body settles on the floor
            if self._up_dot() >= self.cfg.flip_up_dot or amp <= 0:
                break
        else:
            q0 = np.zeros(self.cfg.n_joints)
            self.sim.reset()
            self.sim.step(q0, SETTLE_S)
        self.task.preset_goal = goal_xy
        self.task.reset(self.sim, self._rng)
        post = self.task.post_pose(self.sim)
        if post is not None:
            self.privileged["post"] = post
        self.cap_sensor, self._cap_out = None, None
        if self.obs_builder.capacitive:                                                # post-only capacitive face sensors (randomised per episode, off in the obs when there is no post)
            training = uses_cache(seed)                                                # same split as the layout cache: gate/eval seeds never zero the pressure
            n = self.cfg.n_modules
            self.cap_sensor = CapacitiveSensor(n, self.dt, sample_cap_params(self.cfg.dr, self._rng, training), np.random.default_rng(int(self._rng.integers(2 ** 31 - 1))),
                                               enabled=PostProbe.of(self.sim) is not None)
            self._cap_out = (np.zeros((n, 3)), np.zeros((n, 3)))
        self._init_reply_dr()
        self.bus = None
        if self.task.param("bus_model", 0.0) > 0.0 or self.task.param("bus_penalty", 0.0) > 0.0:       # per-robot current model, spike clamp and metrics (plan sec 5.5); off by default
            self.bus = self.sim.bus = BusClamp(self.sim.model, BUS_A_PER_NM2, REGIMEN_BUS_PEAK_A, clamp=self.task.param("bus_clamp", 1.0) > 0.0)
            self._bus_win: deque[float] = deque(maxlen=max(1, int(round(BUS_MEAN_WINDOW_S * self.cfg.control_hz))))
            self._bus_means: list[float] = []
            self._bus_peak = self._bus_peak_applied = 0.0
            self._bus_over = 0
        # random start poses: begin from where the body actually settled (curled modules can block a joint); else from the zero pose (as GaitPolicy.reset)
        self._tgt = self.sim.joint_pos().copy() if amp > 0 else np.zeros(self.cfg.n_joints)
        self._prev_action = np.zeros(self.cfg.n_joints)
        self._t = 0
        snap = self._snapshot()
        self.obs_builder.reset(snap)
        return self.obs_builder.build(snap), {"privileged": self.privileged}

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        """Apply one policy step (1/control_hz seconds of physics)."""
        assert self.sim is not None
        a = np.clip(np.asarray(action, float), -1.0, 1.0)
        c = self.cfg
        raw = c.action_scale_rad * a
        smoothed = c.action_ema * self._tgt + (1 - c.action_ema) * raw
        self._tgt = self._tgt + np.clip(smoothed - self._tgt, -c.max_target_step_rad, c.max_target_step_rad)
        self.sim.step(self._tgt, self.dt)
        if not np.all(np.isfinite(self.sim.data.qpos)):
            raise FloatingPointError("simulation diverged (NaN)")
        self.obs_builder.push_action(a)
        reward, terms = self.task.reward(self.sim, a, self._prev_action)
        if self.bus is not None:
            tick = self.bus.end_tick()
            self._bus_win.append(tick["mean"])
            self._bus_means.append(tick["mean"])
            self._bus_peak, self._bus_peak_applied = max(self._bus_peak, tick["peak"]), max(self._bus_peak_applied, tick["applied_peak"])
            self._bus_over += int(tick["over"])
            pen = self.task.param("bus_penalty", 0.0)
            if pen > 0.0:                                                                 # penalty on the 2 s running mean above the 4 A budget
                terms["bus"] = -pen * max(0.0, float(np.mean(self._bus_win)) - REGIMEN_BUS_MEAN_A)
                reward += terms["bus"]
        self._prev_action = a
        self._t += 1
        if self.cap_sensor is not None:
            probe = PostProbe.of(self.sim)
            if probe is not None:
                gt = probe.read()
                self._cap_out = self.cap_sensor.step(gt.touch, gt.force, gt.face_dist)
        terminated = (self.task.terminate_on_flip and self._up_dot() < c.flip_up_dot) or self.task.terminated(self.sim)
        truncated = self._t >= self.max_steps
        info = {"terms": terms, "distance": getattr(self.task, "distance", lambda s: 0.0)(self.sim)}
        if hasattr(self.task, "metrics"):
            info["metrics"] = self.task.metrics(self.sim)
        if self.bus is not None:
            info.setdefault("metrics", {}).update({"bus_mean_a": float(np.mean(self._bus_means)), "bus_peak_a": self._bus_peak, "bus_peak_applied_a": self._bus_peak_applied,
                                                   "frac_ticks_over_10a": self._bus_over / len(self._bus_means)})
        if (terminated or truncated) and hasattr(self.task, "pose"):
            info["pose"] = self.task.pose()
        return self.obs_builder.build(self._snapshot()), reward, terminated, truncated, info
