"""Gymnasium environment for PET on the MuJoCo ground model. The action mapping and observation match deployment (see obs.py)."""

from __future__ import annotations

import os
from collections import deque
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from petctl.config import (GIM3505_HOLD_A_PER_NM2, GIM3505_IDLE_PER_MOTOR_A, GIM3505_MOVE_A_PER_NM2, GIM3505_MOVE_OMEGA0_RAD_S, GIM3505_V_TEST, REGIMEN_BUS_MEAN_A,
                          REGIMEN_BUS_PEAK_A)
from petctl.rl.capacitive import CapacitiveSensor, PostProbe, sample_cap_params
from petctl.rl.config import BUS_A_PER_NM2, MIT_TORQUE_Q_NM, MIT_TORQUE_RANGE_NM, MIT_VEL_Q_RAD_S, MIT_VEL_RANGE_RAD_S, EnvConfig
from petctl.rl.domain_rand import _u, sample_params
from petctl.rl.hold_diag import TORQUE_FRAC, HoldBreakTracker
from petctl.rl.hug_buffer import BUFFER_ENV, HugStartBuffer
from petctl.rl.layout_cache import LayoutCache, uses_cache
from petctl.rl.obs import ObsBuilder, Snapshot, clip_action, hold_value, next_target, unbounded
from petctl.rl.target_shapes import V2_SAMPLER, heldout_index, heldout_shape, sample_training_shape
from petctl.rl.tasks import TASKS
from petctl.sim.bus import IDLE_ELECTRONICS_A, BusClamp
from petctl.sim.model_builder import BuildParams, PetSim
from petctl.sim.shapes import shape_of
from petctl.sim.terrain import START_RADIUS_M, generate_terrain

SETTLE_S = 0.75
RANGE_RUN, SAT_ACTION = 3, 0.95         # action-range diagnostic: wrap-attempt = hug run >= 3 modules; a joint action is saturated at |a| >= 0.95
STATE_RUN, STATE_SPAN_DEG = 3, 90.0     # hold-mode diagnostics: the hug pilot state (run >= 3 modules and normal span >= 90 deg)
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
        bound = np.inf if unbounded(self.cfg) else 1.0                                   # unbounded: joint targets carry no angle range (the user's rule: no angle limits, ever)
        self.action_space = spaces.Box(-bound, bound, (self.cfg.n_actions,), np.float32)
        self.max_steps = int(round(self.cfg.episode_s * self.cfg.control_hz))
        self.sim: PetSim | None = None
        self._cap_out: tuple[np.ndarray, np.ndarray] | None = None
        self.cap_sensor: CapacitiveSensor | None = None
        self._reply_mult, self._reply_buf = 1.0, None
        self._mit_frame = False
        self.bus: BusClamp | None = None
        self._start_buf: HugStartBuffer | None = None
        self._start_buf_path: str | None = None
        self._hug_fam_levels: dict[str, int] = {}                           # per-family ladder indices (hug_curriculum 2; empty = every family at its top level = the gate definition)
        self._hug_level = int(os.environ.get("PETCTL_HUG_LEVEL", "3"))       # hug success-curriculum level (training only; the task knob ``hug_curriculum`` switches it on; default 3 = the gate definition)

    # ------------------------------------------------------------------ hug start buffer (in memory; merged by the training callback once per rollout)
    def start_buffer(self) -> HugStartBuffer | None:
        """This env's in-memory hug start buffer on the file named by ``$PETCTL_HUG_BUFFER`` (created on first use; None when the variable is unset)."""
        p = os.environ.get(BUFFER_ENV) or None
        if p != self._start_buf_path:
            self._start_buf_path, self._start_buf = p, (HugStartBuffer(p) if p else None)
        return self._start_buf

    def drain_start_buffer(self) -> list[dict[str, Any]]:
        """New start states stored by this env since the last call (for the callback's merge)."""
        b = self.start_buffer()
        return [] if b is None else b.drain()

    def set_start_buffer(self, items: list[dict[str, Any]]) -> None:
        """Install the merged start buffer broadcast by the callback."""
        b = self.start_buffer()
        if b is not None:
            b.replace(items)

    def set_dr_scale(self, scale: float) -> None:
        """Domain-randomisation scale for the episodes that start after this call (broadcast by ``DrRampCallback``)."""
        self.cfg.dr.scale = float(scale)

    def set_hug_level(self, level: int) -> None:
        """Success-curriculum level (1-3) for the episodes that start after this call (broadcast by ``HugLevelCallback`` once per rollout)."""
        self._hug_level = min(3, max(1, int(level)))

    def set_hug_family_levels(self, levels: dict[str, int]) -> None:
        """Per-family success-curriculum ladder indices (stage hug_curr_02) for the episodes that start after this call (broadcast by ``HugFamilyLevelCallback``)."""
        self._hug_fam_levels = {str(k): int(v) for k, v in levels.items()}

    def get_hug_family_levels(self) -> dict[str, int]:
        """The per-family ladder indices that the next episode starts at."""
        return dict(self._hug_fam_levels)

    def get_hug_level(self) -> int:
        """The success-curriculum level that the next episode starts at."""
        return self._hug_level

    # ------------------------------------------------------------------ helpers
    def _snapshot(self) -> Snapshot:
        assert self.sim is not None
        d, rng = self.sim.data, self._rng
        dr = self.cfg.dr
        pos = self.sim.joint_pos() + self._joint_off + rng.normal(0, dr.pos_noise_rad * dr.scale, self.cfg.n_joints)
        quat = _qmul(self._tilt, self.sim.sensor("quat").copy())
        gyro = self.sim.sensor("gyro") + rng.normal(0, dr.gyro_noise * dr.scale, 3)
        if self._mit_frame:                                                           # GIM3505-8 reply frame (knob mit_frame): position, velocity and torque arrive together, one delay / drop for all three
            nj = self.cfg.n_joints
            vel = quantise(np.clip(self.sim.joint_vel() + rng.normal(0, dr.vel_noise_rad_s * dr.scale, nj), -MIT_VEL_RANGE_RAD_S, MIT_VEL_RANGE_RAD_S), MIT_VEL_Q_RAD_S)
            trq = quantise(np.clip(self.sim.joint_torque() + rng.normal(0, REPLY_TORQUE_NOISE_NM * self._reply_mult, nj), -MIT_TORQUE_RANGE_NM, MIT_TORQUE_RANGE_NM), MIT_TORQUE_Q_NM)
            frame = np.concatenate([pos, vel, trq])
            if self._reply_buf is not None:
                if not self._reply_buf:
                    self._reply_buf.extend([frame] * self._reply_buf.maxlen)         # type: ignore[arg-type]
                    self._reply_last = frame
                self._reply_buf.append(frame)
                out = self._reply_buf[0]
                frame = self._reply_last if self._reply_rng.random() < self._reply_drop else out
                self._reply_last = frame
            pos, vel, trq = frame[:nj], frame[nj:2 * nj], frame[2 * nj:]
            return Snapshot(pos, quat, gyro, self.sim.model.actuator_forcerange[:, 1].copy(), self.task.extra_obs(self.sim), trq, self._tgt.copy(),
                            None if self._cap_out is None else np.concatenate([self._cap_out[0].reshape(-1), self._cap_out[1].reshape(-1)]), joint_vel=vel)
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
        self._mit_frame = self.task.param("mit_frame", 0.0) > 0.0                       # GIM3505-8 reply frame: direct quantised velocity, +-2 N.m torque field, one delay for the whole frame
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
            post[3:3 + n] = shape_of(sim.p.post_shape).dist_xy(sim.data.xipos[1:n + 1, :2], np.array([px, py]))
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

    def _apply_start_orient(self, orient: int) -> None:
        """Roll the freshly reset body about its long (x) axis: 1 = 180 deg (upside-down), 2 = +90, 3 = -90, 4 = uniform random roll; lifts the root so no module starts in the floor."""
        assert self.sim is not None
        roll = {1: np.pi, 2: np.pi / 2, 3: -np.pi / 2}.get(orient, float(self._rng.uniform(-np.pi, np.pi)))
        self.sim.data.qpos[3:7] = [np.cos(roll / 2), np.sin(roll / 2), 0.0, 0.0]
        self.sim.data.qpos[2] += 0.04
        import mujoco  # type: ignore[import-untyped]
        mujoco.mj_forward(self.sim.model, self.sim.data)

    def _orient_step(self) -> None:
        """Orientation diagnostics (flips, time inverted, recovery time); never affects the reward or termination."""
        o = self._orient
        inverted = self._up_dot() < self.cfg.flip_up_dot
        o["inv_s"] += self.dt if inverted else 0.0
        if inverted and not o["inv"]:
            o["inv"], o["spell_s"] = True, 0.0
            o["flips"] += o["armed"]
        elif o["inv"] and self._up_dot() > 0.5:                                 # recovered (hysteresis: back above 0.5)
            o["inv"] = False
            o["rec_sum"] += o["spell_s"]
            o["rec_n"] += 1
        if o["inv"]:
            o["spell_s"] += self.dt
        o["armed"] = 1 if not o["inv"] else o["armed"]

    def _orient_metrics(self) -> dict[str, float]:
        o = self._orient
        return {"orient_inverted_s": o["inv_s"], "orient_flips": float(o["flips"]), "orient_recoveries": float(o["rec_n"]),
                "orient_recovery_s": o["rec_sum"] / o["rec_n"] if o["rec_n"] else 0.0, "orient_end_inverted": float(o["inv"])}

    def _up_dot(self) -> float:
        assert self.sim is not None
        return float(self.sim.data.site_xmat[self.sim.imu_id].reshape(3, 3)[2, 2])

    # ------------------------------------------------------------------ gym API
    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[np.ndarray, dict[str, Any]]:
        """Randomise the plant, settle the robot on the floor, and return the first observation."""
        super().reset(seed=seed)
        self._rng = self.np_random
        params, self.privileged = sample_params(self.cfg.dr, self._rng, self.base)
        tp = self.cfg.task_params
        if "floor_friction" in tp or "friction_aniso" in tp:                          # eval-suite scenarios fix the friction (overrides the randomised draw)
            params = BuildParams(**{**params.__dict__, "floor_friction": float(tp.get("floor_friction", params.floor_friction)), "skin_aniso": float(tp.get("friction_aniso", params.skin_aniso))})
            self.privileged.update({"floor_friction": params.floor_friction, "skin_aniso": params.skin_aniso})
        self.task = TASKS[self.cfg.task](self.cfg)
        if hasattr(self.task, "hug_level"):
            self.task.hug_level = self._hug_level                                   # type: ignore[attr-defined]
        if hasattr(self.task, "hug_family_levels"):
            self.task.hug_family_levels = dict(self._hug_fam_levels)                # type: ignore[attr-defined]
        shape = None
        pick = getattr(self.task, "pick_buffer_start", None)
        if pick is not None:
            self.task.start_buffer = self.start_buffer()  # type: ignore[attr-defined]
        bstart = pick(self._rng) if pick is not None else None                        # hug start buffer (task knob hug_buffer): a stored state fixes the object's shape and place
        if bstart is not None:
            from petctl.sim.shapes import TargetShape
            shape = TargetShape(*bstart["shape"])
            self.task.target_shape = shape  # type: ignore[attr-defined]
        elif self.task.param("target_family", 0.0) > 0.0:                                 # shape-agnostic target (petctl.rl.target_shapes); drawn before the layout, which sizes the start gaps by it
            hi = heldout_index(seed) if (seed is not None and self.task.param("target_heldout", 0.0) > 0.0) else None
            shape = (heldout_shape(hi, int(seed), self.cfg.n_modules) if hi is not None
                     else sample_training_shape(self._rng, float(self.task.param("shape_phase", 2.0)), self.cfg.n_modules,
                                                **(V2_SAMPLER if self.task.param("shape_mix", 0.0) > 0.0 else {})))  # type: ignore[arg-type]
            self.task.target_shape = shape  # type: ignore[attr-defined]
        self.target_shape = shape
        walls, goal_xy = self._layout(seed)                                           # static geometry (walls) must exist before the sim is built
        if bstart is not None and goal_xy is not None:
            goal_xy = np.array([bstart["post"][0], bstart["post"][1], goal_xy[2]])
        obstacles = getattr(self.task, "obstacles", ())
        if walls or obstacles:
            params = BuildParams(**{**params.__dict__, "walls": walls, "obstacles": tuple(obstacles)})
        if self.task.param("post", 0.0) > 0.0:                                         # heavy immovable target post on the goal point
            if goal_xy is None:
                raise ValueError(f"task '{self.cfg.task}' lays out no target, so it cannot have a post")
            params = BuildParams(**{**params.__dict__, "post": (float(goal_xy[0]), float(goal_xy[1])), "post_shape": shape})
        level = self.task.param("terrain_level", 0.0)
        lmin = self.task.param("terrain_level_min", -1.0)
        if level > 0.0 and 0.0 <= lmin < level:                                         # training: level drawn per episode in [min, level] (gates fix the level)
            level = float(np.random.default_rng(int(self._rng.integers(2 ** 31 - 1))).uniform(lmin, level))
        if level > 0.0:                                                                 # per-episode heightfield terrain (flat start disk covers the body)
            tseed = int(self._rng.integers(2 ** 31 - 1))
            terrain = generate_terrain(level, tseed, max(START_RADIUS_M, self._foot), rolling=self.task.param("terrain_rolling", 0.0) > 0.0, ridges=self.task.param("terrain_ridges", 0.0))
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
        arranged = False
        arrange = getattr(self.task, "arrange_start", None)
        restored: np.ndarray | None = None
        if bstart is not None:
            restored, arranged = self.task.restore_buffer_start(self.sim, self._rng), True      # type: ignore[attr-defined]
        elif arrange is not None:                                                      # task-arranged start (goal_hug coil starts): joints posed and the post placed by the task
            new_goal = arrange(self.sim, self._rng)
            if new_goal is not None:
                goal_xy, arranged = new_goal, True
        orient = int(self.cfg.task_params.get("start_orient", 0.0))           # eval-suite start orientation: 0 upright, 1 upside-down, 2 on the left side, 3 on the right side, 4 random roll
        for _ in range(8 if amp > 0 and orient <= 0 else 1):                  # resample start poses that topple while settling (episode would end at once)
            if arranged:
                break
            if amp > 0 or orient > 0:
                if amp > 0:
                    q0 = self._rng.uniform(-amp, amp, self.cfg.n_joints)
                self.sim.reset(q0)
                if orient > 0:
                    self._apply_start_orient(orient)
                self.sim.lift_clear_of_terrain()                              # terrain only: a curled start pose may begin below the heightfield
            self.sim.step(q0, SETTLE_S)                                       # hold the start pose while the body settles on the floor
            if self._up_dot() >= self.cfg.flip_up_dot or amp <= 0 or orient > 0:
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
            gim = self.task.param("bus_motor", 0.0) > 0.0                                  # 1: SteadyWin GIM3505-8 model (owner measurements, MOTOR_GIM3505.md); 0: legacy GL40 II model
            vscale = GIM3505_V_TEST / max(self.task.param("bus_voltage", GIM3505_V_TEST), 1.0)
            kw = ({"move_coeff": GIM3505_MOVE_A_PER_NM2 * vscale, "move_omega0": GIM3505_MOVE_OMEGA0_RAD_S, "idle_a": IDLE_ELECTRONICS_A + GIM3505_IDLE_PER_MOTOR_A * self.cfg.n_joints}
                  if gim else {})
            self.bus = self.sim.bus = BusClamp(self.sim.model, GIM3505_HOLD_A_PER_NM2 * vscale if gim else BUS_A_PER_NM2, REGIMEN_BUS_PEAK_A, clamp=self.task.param("bus_clamp", 1.0) > 0.0,
                                         soft_knee=self.task.param("bus_soft_knee", 0.0), **kw)
            self._bus_win: deque[float] = deque(maxlen=max(1, int(round(BUS_MEAN_WINDOW_S * self.cfg.control_hz))))
            self._bus_means: list[float] = []
            self._bus_peak = self._bus_peak_applied = self._bus_peak_win = 0.0
            self._bus_over_win = 0
            self._bus_over = 0
        # random start poses: begin from where the body actually settled (curled modules can block a joint); else from the zero pose (as GaitPolicy.reset)
        self._tgt = restored if restored is not None else self.sim.joint_pos().copy() if amp > 0 or arranged else np.zeros(self.cfg.n_joints)
        self._term_sums: dict[str, float] = {}
        self._orient = {"inv": self._up_dot() < self.cfg.flip_up_dot, "inv_s": 0.0, "spell_s": 0.0, "flips": 0, "armed": 0, "rec_sum": 0.0, "rec_n": 0}
        self._orient["armed"] = 0 if self._orient["inv"] else 1
        self._h_sum = self._h_wrap_sum = self._state_run = self._state_best = 0
        self._h_wrap_n = 0
        self._h_wrap1_sum, self._h_wrap1_n = 0.0, 0
        self._range_on = hasattr(self.task, "_hug") and self.task.param("range_diag", 0.0) > 0.0         # task knob ``range_diag``: log the action-range metrics (off: older stages' logs unchanged)
        self._hd = HoldBreakTracker(self.cfg.control_hz) if (hasattr(self.task, "_hug") and self.task.param("hold_diag", 0.0) > 0.0) else None      # task knob ``hold_diag``: hold-break diagnostic (default off)
        self._range = {"n": 0.0, "any": 0.0, "all": 0.0, "frac": 0.0, "gap": 0.0, "tmax": 0.0}      # action-range diagnostic sums over wrap-attempt ticks (hug run >= 3)
        self._prev_action = np.zeros(self.cfg.n_joints)
        self._t = 0
        snap = self._snapshot()
        self.obs_builder.reset(snap)
        return self.obs_builder.build(snap), {"privileged": self.privileged}

    def _hold_step(self, h: float) -> None:
        """Hold-mode bookkeeping for this tick: the hold value ``h``, whether this is a wrap tick (the hug metric's ``hug_tick``) and the streak of ticks in the pilot state (run >= 3 and normal span >= 90 deg)."""
        self._h_sum += h
        last = getattr(getattr(self.task, "_hug", None), "last", None)
        if last is not None and last.hug_tick:
            self._h_wrap_sum += h
            self._h_wrap_n += 1
        l1 = getattr(self.task, "_hug_levels", {}).get(1)                               # success curriculum: wrap ticks by the level-1 definition as well
        if l1 is not None and l1.last is not None and l1.last.hug_tick:
            self._h_wrap1_sum += h
            self._h_wrap1_n += 1
        in_state = last is not None and last.hug_modules >= STATE_RUN and last.span_deg >= STATE_SPAN_DEG
        self._state_run = self._state_run + 1 if in_state else 0
        self._state_best = max(self._state_best, self._state_run)

    def _range_step(self, a: np.ndarray) -> None:
        """Action-range bookkeeping for this tick (hug tasks): on a wrap-attempt tick (hug run >= ``RANGE_RUN`` modules) look at the joints INSIDE the run: whether their action is saturated
        (|a| >= ``SAT_ACTION``), the gap between each one's commanded target and its measured angle, and the largest |target|."""
        last = getattr(getattr(self.task, "_hug", None), "last", None)
        if last is None or last.hug_modules < RANGE_RUN:
            return
        nj = self.cfg.n_joints
        idx = [k - 1 for k in range(last.run_start + 1, last.run_start + last.hug_modules) if 1 <= k <= nj]
        if not idx:
            return
        sat = np.abs(np.asarray(a, float)[idx]) >= SAT_ACTION
        r = self._range
        r["n"] += 1.0
        r["any"] += float(sat.any())
        r["all"] += float(sat.all())
        r["frac"] += float(sat.mean())
        r["gap"] += float(np.mean(np.abs(self._tgt[idx] - self.sim.joint_pos()[idx])))
        r["tmax"] = max(r["tmax"], float(np.max(np.abs(self._tgt[idx]))))

    def _hold_diag_step(self, bus_a: float) -> None:
        """Hold-break diagnostic bookkeeping for this tick (``hold_diag`` knob): feeds the hug tracker's state and the joint torques / bus current to ``HoldBreakTracker``."""
        st = getattr(getattr(self.task, "_hug", None), "last", None)
        if st is None or self._hd is None:
            return
        hug = self.task._hug
        cap = np.asarray(self.sim.model.actuator_forcerange[:, 1], float)
        at_cap = np.abs(self.sim.joint_torque()) >= TORQUE_FRAC * cap
        self._hd.update(hug_tick=bool(st.hug_tick), just_held=bool(st.just_held), held=bool(st.held), run=int(st.hug_modules), run_start=int(st.run_start), h=int(hug.h), span_deg=float(st.span_deg),
                        span_min=hug.span_min, at_cap=at_cap, bus_a=bus_a, n_joints=self.cfg.n_joints)

    def _range_metrics(self) -> dict[str, float]:
        """Episode metrics of the action range: ``wrap_attempt_ticks`` and, when there were any, ``saturated_frac`` (mean over those ticks of the fraction of in-run joints with |action| >= 0.95), ``sat_any_frac`` (ticks with at least one
        saturated in-run joint), ``sat_all_frac`` (ticks with all of them saturated), ``target_gap_run`` (mean |target - angle| of the in-run joints); ``max_abs_target_run`` (rad, 0 without wrap-attempt ticks)."""
        r = self._range
        out = {"wrap_attempt_ticks": r["n"], "max_abs_target_run": r["tmax"]}
        if r["n"] > 0:
            out.update({"saturated_frac": r["frac"] / r["n"], "sat_any_frac": r["any"] / r["n"], "sat_all_frac": r["all"] / r["n"], "target_gap_run": r["gap"] / r["n"]})
        return out

    def _hold_metrics(self) -> dict[str, float]:
        """Episode metrics of the hold channel: ``h_mean`` (all ticks), ``h_wrap_mean`` / ``h_wrap_n`` (wrap ticks; the mean is absent without wrap ticks), ``state_hold_s`` (longest streak in the pilot state)."""
        out = {"h_mean": self._h_sum / max(self._t, 1), "h_wrap_n": float(self._h_wrap_n), "state_hold_s": self._state_best / self.cfg.control_hz}
        if self._h_wrap_n:
            out["h_wrap_mean"] = self._h_wrap_sum / self._h_wrap_n
        if getattr(self.task, "_hug_levels", None):
            out["h_wrap1_n"] = float(self._h_wrap1_n)
            if self._h_wrap1_n:
                out["h_wrap1_mean"] = self._h_wrap1_sum / self._h_wrap1_n
        return out

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        """Apply one policy step (1/control_hz seconds of physics)."""
        assert self.sim is not None
        c = self.cfg
        a_full = clip_action(c, action)
        self._tgt = next_target(c, self._tgt, a_full)
        a = a_full[:c.n_joints]                                                          # joint outputs; in ``hold`` mode the last entry is the hold channel (history / reward see the joints only)
        self.sim.step(self._tgt, self.dt)
        if not np.all(np.isfinite(self.sim.data.qpos)):
            raise FloatingPointError("simulation diverged (NaN)")
        self.obs_builder.push_action(a * c.action_obs_scale)
        reward, terms = self.task.reward(self.sim, a, self._prev_action)
        if self.bus is not None:
            tick = self.bus.end_tick()
            self._bus_win.append(tick["mean"])
            self._bus_means.append(tick["mean"])
            self._bus_peak, self._bus_peak_applied = max(self._bus_peak, tick["peak"]), max(self._bus_peak_applied, tick["applied_peak"])
            self._bus_over += int(tick["over"])
            self._bus_peak_win = max(self._bus_peak_win, tick["demand_mean"])
            self._bus_over_win += int(tick["demand_mean"] > REGIMEN_BUS_PEAK_A)
            pen = self.task.param("bus_penalty", 0.0)
            if pen > 0.0:                                                                 # penalty on the 2 s running mean above the 4 A budget
                terms["bus"] = -pen * max(0.0, float(np.mean(self._bus_win)) - REGIMEN_BUS_MEAN_A)
                reward += terms["bus"]
            bpen = self.task.param("bus_peak_penalty", 0.0)
            if bpen > 0.0:                                                                # v3: per-tick penalty on the DEMANDED peak current above the 10 A ceiling (before the clamp)
                terms["bus_peak"] = -bpen * max(0.0, float(tick["peak"]) - REGIMEN_BUS_PEAK_A)
                reward += terms["bus_peak"]
        self._prev_action = a
        if self._range_on:
            self._range_step(a)
        if self._hd is not None:
            self._hold_diag_step(float(tick["mean"]) if self.bus is not None else 0.0)
        if c.action_mode == "hold":
            self._hold_step(hold_value(a_full))
        self._t += 1
        self._orient_step()
        store = getattr(self.task, "maybe_store", None)
        if store is not None:
            store(self.sim, self._tgt)
        for k, v in terms.items():                                                    # per-term episode sums (logged as metrics term_<name> at the end of goal-task episodes)
            self._term_sums[k] = self._term_sums.get(k, 0.0) + float(v)
        if self.cap_sensor is not None:
            probe = PostProbe.of(self.sim)
            if probe is not None:
                gt = probe.read()
                self._cap_out = self.cap_sensor.step(gt.touch, gt.force, gt.face_dist)
        terminated = (self.task.terminate_on_flip and c.task_params.get("allow_flip", 0.0) <= 0.0 and self._up_dot() < c.flip_up_dot) or self.task.terminated(self.sim)
        truncated = self._t >= self.max_steps
        info = {"terms": terms, "distance": getattr(self.task, "distance", lambda s: 0.0)(self.sim)}
        if hasattr(self.task, "metrics"):
            info["metrics"] = self.task.metrics(self.sim)
        if terminated or truncated:                                                   # every task: per-term episode sums + orientation diagnostics (watchdog / TensorBoard read them)
            info.setdefault("metrics", {}).update({f"term_{k}": v for k, v in self._term_sums.items()})
            info["metrics"].update(self._orient_metrics())
        if self.bus is not None:
            info.setdefault("metrics", {}).update({"bus_mean_a": float(np.mean(self._bus_means)), "bus_peak_a": self._bus_peak, "bus_peak_applied_a": self._bus_peak_applied,
                                                   "frac_ticks_over_10a": self._bus_over / len(self._bus_means),
                                                   "bus_peak_win_a": self._bus_peak_win, "frac_ticks_over_win": self._bus_over_win / len(self._bus_means)})
        if c.action_mode == "hold":
            info.setdefault("metrics", {}).update(self._hold_metrics())
        if self._range_on:
            info.setdefault("metrics", {}).update(self._range_metrics())
        if self._hd is not None:
            info.setdefault("metrics", {}).update(self._hd.metrics())
        if (terminated or truncated) and hasattr(self.task, "pose"):
            info["pose"] = self.task.pose()
        return self.obs_builder.build(self._snapshot()), reward, terminated, truncated, info
