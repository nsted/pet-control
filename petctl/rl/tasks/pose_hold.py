"""S2 hold-and-comply task (docs/plans/STAGE_2_GATES.md). The episode starts in a cached settled pose; the policy holds the known pose q* under hidden disturbances. Cost-only reward, NO termination.

The scenario comes from ``reset(options={"scenario": S2Scenario dict})`` (a frozen scenario) or, without it, a quiet default. Plant draws of a scenario (friction, delay, noise, zeta, start T_coil) are applied by the
evaluator through task knobs; this task applies mass scale, derate, the disturbance window and the reward. The reward starting weights are pre-registered in STAGE_2_GATES; the current-reference and
yield-load terms need the T14 tables and are off until those exist (knob ``s2_full_reward``).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from petctl.rl.config import EnvConfig
from petctl.rl.s2.cells import CACHE_DIR, ORIENTS, cache_path
from petctl.rl.s2.ladder import LEVELS
from petctl.rl.s2.yieldload import load_of
from petctl.rl.tasks.base import Task
from petctl.sim.model_builder import PetSim

W_TRACK, W_BUS_MEAN, W_BUS_PEAK, W_TCOIL, W_RATE, W_YIELD = 1.0, 0.5, 0.5, 0.2, 0.05, 1.0           # pre-registered starting weights (STAGE_2_GATES, Reward)
TRACK_DEAD_RAD, TRACK_SCALE_RAD = 0.02, 0.05
BUS_MEAN_A, BUS_PEAK_A = 4.0, 10.0
T_KNEE_C = 80.0
AXES = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1)}


class PoseHold(Task):
    """Hold the cached pose q* under the scenario's disturbance. Never terminates (flip / lost are metrics)."""

    name = "pose_hold"
    score_label = "negative tracking cost"
    terminate_on_flip = False
    extra_obs_dim = 10            # q* (the commanded pose is known to the policy), n_joints = 10 at n = 11

    @classmethod
    def extra_dim(cls, params: dict[str, float]) -> int:
        """q* has one entry per joint; the S2 task is n = 11 only."""
        return cls.extra_obs_dim

    def __init__(self, cfg: EnvConfig) -> None:
        """Defaults: no options."""
        super().__init__(cfg)
        self.options: dict[str, Any] = {}
        self.q_star = np.zeros(cfg.n_joints)
        self._prev_full: np.ndarray | None = None

    # ------------------------------------------------------------------ start
    def _scenario(self) -> dict[str, Any]:
        sc = {"pose": "straight", "orient": "upright", "family": "none", "level": "L2", "onset_s": 5.0, "duration_s": 3.0, "body": "head", "axis": "+y", "mag": 1.0, "tilt_axis_deg": 0.0,
              "load_kind": "force", "derate_x": 1.0, "yield_mult": 0.0, "press_n": 0.0, "plant": {"mass": 1.0, "derate": 1.0}}
        sc.update(self.options.get("scenario", {}))
        return sc

    def arrange_start(self, sim: PetSim, rng: np.random.Generator) -> np.ndarray:
        """Restore the cached settled state of the scenario's cell (the policy never does the entry); returns a dummy goal so the env treats the start as arranged."""
        sc = self._scenario()
        root = Path(self.options.get("cache_dir", CACHE_DIR))
        c = np.load(cache_path(sc["pose"], sc["orient"], root))
        sim.reset()
        sim.data.qpos[:] = c["qpos"]
        sim.data.qvel[:] = c["qvel"]
        mujoco.mj_forward(sim.model, sim.data)
        sim._buf.clear()
        sim._last_target = sim.joint_pos().copy()
        self.q_star = np.asarray(c["qstar"], float)
        return np.zeros(3)

    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """Apply the scenario's persistent plant factors (mass scale, torque derate) and prepare the disturbance window."""
        sc = self._scenario()
        m = sim.model
        self._g0, self._mass0, self._fr0 = m.opt.gravity.copy(), m.body_mass.copy(), m.actuator_forcerange.copy()
        plant = sc.get("plant", {})
        m.body_mass[:] = self._mass0 * float(plant.get("mass", 1.0))
        m.actuator_forcerange[:] = self._fr0 * float(plant.get("derate", 1.0))
        self._sc = sc
        self._lv = LEVELS[sc["level"]]
        n = sim.p.n_modules
        self._body = {"head": sim.m0_id, "mid": sim.m0_id + n // 2, "tail-1": sim.m0_id + n - 2}[sc["body"]]
        self._on = False
        self._prev_full = None
        self.onset_s, self.end_s = float(sc["onset_s"]), float(sc["onset_s"]) + float(sc["duration_s"])
        if self.q_star is None or not np.any(self.q_star) and sc["pose"] != "straight":
            raise RuntimeError("q* not loaded")

    def extra_obs(self, sim: PetSim) -> np.ndarray:
        """The commanded pose q* (known to the policy)."""
        return self.q_star.astype(np.float32)

    # ------------------------------------------------------------------ disturbances
    def force_n(self) -> float:
        """Magnitude (N) of the scenario's force disturbance (push / shove / load / yield)."""
        sc, lv = self._sc, self._lv
        fam = sc["family"]
        if fam == "push":
            return sc["mag"] * lv.push_max_n
        if fam == "shove":
            return sc["mag"] * lv.shove_max_n
        if fam == "load" and sc["load_kind"] == "force":
            return sc["mag"] * lv.load_max_n
        if fam == "yield":
            return float(sc["yield_mult"]) * float(sc.get("press_n", 0.0))
        return 0.0

    def pre_step(self, sim: PetSim, t: float) -> None:
        """Switch the disturbance on at the onset and off after its duration (forces, tilted gravity, head mass, torque derate)."""
        sc, d, m = self._sc, sim.data, sim.model
        on = self.onset_s <= t < self.end_s
        fam = sc["family"]
        if fam in ("push", "shove", "yield") or (fam == "load" and sc["load_kind"] == "force"):
            d.xfrc_applied[:] = 0.0
            if on:
                v = np.array(AXES[sc["axis"]], float) * self.force_n()
                d.xfrc_applied[self._body, :3] = d.xmat[self._body].reshape(3, 3) @ v if sc["family"] != "load" and sc["family"] != "yield" else v
        if on != self._on:
            if fam == "tilt":
                a, b = math.radians(sc["mag"] * self._lv.tilt_max_deg), math.radians(sc["tilt_axis_deg"])
                m.opt.gravity[:] = (9.81 * math.sin(a) * math.cos(b), 9.81 * math.sin(a) * math.sin(b), -9.81 * math.cos(a)) if on else self._g0
            elif fam == "load" and sc["load_kind"] == "head_mass":
                m.body_mass[self._body] = self._mass0[self._body] * float(sc["plant"].get("mass", 1.0)) * (1 + sc["mag"] * (self._lv.head_mass[1] - 1)) if on else self._mass0[self._body] * float(sc["plant"].get("mass", 1.0))
            elif fam == "derate":
                m.actuator_forcerange[:] = self._fr0 * float(sc["plant"].get("derate", 1.0)) * (sc["derate_x"] if on else 1.0)
            self._on = on

    # ------------------------------------------------------------------ reward
    def overload(self) -> bool:
        """Privileged flag: the yield overload is acting."""
        return self._sc["family"] == "yield" and self._on

    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """Tracking cost on the wrapped joint error (every tick, settle included)."""
        e = np.abs((sim.joint_pos() - self.q_star + np.pi) % (2 * np.pi) - np.pi)
        track = -W_TRACK * float(np.mean((np.maximum(e - TRACK_DEAD_RAD, 0.0) / TRACK_SCALE_RAD) ** 2))
        return track, {"track": track}

    def reward_extra(self, sim: PetSim, a_full: np.ndarray, prev_action: np.ndarray, st: dict[str, Any]) -> dict[str, float]:
        """Bus mean / peak, T_coil and action-rate costs (need this tick's bus and thermal state)."""
        out = {"bus_mean": -W_BUS_MEAN * max(st["bus_mean_2s"] - BUS_MEAN_A, 0.0) ** 2, "bus_peak": -W_BUS_PEAK * max(st["bus_tick"] - BUS_PEAK_A, 0.0) ** 2}
        if st["temp"] is not None:
            out["tcoil"] = -W_TCOIL * (max(float(np.max(st["temp"])) - T_KNEE_C, 0.0) / 10.0) ** 2
        prev = self._prev_full if self._prev_full is not None else np.zeros_like(a_full)
        out["action_rate"] = -W_RATE * float(np.mean((np.asarray(a_full) - prev) ** 2))
        self._prev_full = np.asarray(a_full, float).copy()
        if self.overload():                                                       # yield load on OVERLOAD ticks only (privileged flag); true actuator force vs the derated cap
            out["yield_load"] = -W_YIELD * load_of(sim.joint_torque(), sim.model.actuator_forcerange[:, 1])
        return out
