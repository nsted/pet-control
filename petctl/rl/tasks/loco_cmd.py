"""S3 task: track a body-frame velocity command from any start state (docs/plans/STAGE_3_GATES_PROPOSAL.md). DEV version until the owner approves the S3 suites.

Two 10 s command segments per 20 s episode; the segment frame (tail-to-head chord heading) is fixed at the start of each segment; no termination. Reward per tick = T of the last 1 s window (0 during the 1 s mask after
a switch), so standing still earns 0 and moving the wrong way is negative; on stop segments only a penalty for speed (never a reward for stillness). Homeostasis terms reuse the S2 definitions (bus, T_coil, action rate).
The scenario comes from ``reset(options={"scenario": {...}})`` (frozen evaluation) or is sampled from the families (training).
"""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np

from petctl.rl.config import EnvConfig
from petctl.rl.s3 import metrics as M
from petctl.rl.tasks.base import Task
from petctl.rl.tasks.pose_hold import BUS_MEAN_A, BUS_PEAK_A, T_KNEE_C, W_BUS_MEAN, W_BUS_PEAK, W_RATE, W_TCOIL
from petctl.sim.model_builder import PetSim

SEG_S = 10.0
MASK_S = 1.0
WINDOW_S = 1.0
STOP_PENALTY_W = 0.5
FAMILIES = ("forward", "back", "left", "right", "turn_left", "turn_right", "combined", "stop")


def command_of(family: str, rng: np.random.Generator) -> np.ndarray:
    """A command (vx, vy, wz) in the segment frame: speeds 5-15 cm/s, turns 0.3-1.0 rad/s, combined = a translation with a turn."""
    v, w = float(rng.uniform(0.05, 0.15)), float(rng.uniform(0.3, 1.0))
    sgn = float(rng.choice([-1.0, 1.0]))
    return {"forward": np.array([v, 0, 0]), "back": np.array([-v, 0, 0]), "left": np.array([0, v, 0]), "right": np.array([0, -v, 0]), "turn_left": np.array([0, 0, w]),
            "turn_right": np.array([0, 0, -w]), "combined": np.array([v * rng.choice([-1, 1]), 0, sgn * w]), "stop": np.zeros(3)}[family]


class LocoCmd(Task):
    """Velocity-command tracking from any start (no termination)."""

    name = "loco_cmd"
    score_label = "mean T"
    terminate_on_flip = False
    extra_obs_dim = 3             # the command (vx / 0.15, vy / 0.15, wz / 1.0) in the segment frame

    @classmethod
    def extra_dim(cls, params: dict[str, float]) -> int:
        """The command block."""
        return cls.extra_obs_dim

    def __init__(self, cfg: EnvConfig) -> None:
        """Options come from ``PetEnv.reset``."""
        super().__init__(cfg)
        self.options: dict[str, Any] = {}
        self._prev_full: np.ndarray | None = None

    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """Pick (or read) the two command segments and start the window history."""
        sc = self.options.get("scenario")
        if sc is None:
            fams = [str(rng.choice(FAMILIES[:-1] if rng.random() > 0.15 else FAMILIES)) for _ in range(2)]
            sc = {"segments": [{"family": f, "cmd": command_of(f, rng).tolist()} for f in fams]}
        self.scenario = sc
        self.segments = [np.asarray(s["cmd"], float) for s in sc["segments"]]
        self.seg_s = float(sc.get("seg_s", SEG_S))
        self.n = sim.p.n_modules
        self.ch = M.chords(self.n)
        self.hist: deque[tuple[np.ndarray, np.ndarray]] = deque(maxlen=int(round(WINDOW_S * self.cfg.control_hz)) + 1)
        self.t = 0
        self.seg = 0
        self.h0 = self._heading(sim)
        self.mask = int(round(MASK_S * self.cfg.control_hz))
        self.last_T: float | None = None
        self.last_stop: float | None = None
        self.log: list[dict[str, Any]] = []
        self._prev_full = None
        self._push(sim)
        self.push = sc.get("push")

    # ------------------------------------------------------------------ helpers
    def _com_ang(self, sim: PetSim) -> tuple[np.ndarray, np.ndarray]:
        pos = sim.data.xipos[sim.m0_id:sim.m0_id + self.n]
        return pos.mean(axis=0).copy(), M.chord_angles(pos, self.ch)

    def _heading(self, sim: PetSim) -> float:
        return float(np.median(self._com_ang(sim)[1]))

    def _push(self, sim: PetSim) -> None:
        self.hist.append(self._com_ang(sim))

    @property
    def cmd(self) -> np.ndarray:
        """The current segment's command."""
        return self.segments[min(self.seg, len(self.segments) - 1)]

    def extra_obs(self, sim: PetSim) -> np.ndarray:
        """The command in the segment frame, scaled to about [-1, 1]."""
        c = self.cmd
        return np.array([c[0] / 0.15, c[1] / 0.15, c[2] / 1.0], np.float32)

    # ------------------------------------------------------------------ dynamics hooks
    def pre_step(self, sim: PetSim, t_s: float) -> None:
        """Optional push during the episode (scenario ``push``: start s, duration s, body, force vector N in the world frame)."""
        p = self.push
        if p is None:
            return
        d = sim.data
        d.xfrc_applied[:] = 0.0
        if p["t"] <= t_s < p["t"] + p["dur"]:
            body = {"head": sim.m0_id, "mid": sim.m0_id + self.n // 2, "tail": sim.m0_id + self.n - 1}[p.get("body", "mid")]
            d.xfrc_applied[body, :3] = np.asarray(p["f"], float)

    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """T of the last 1 s window in the segment frame (0 while masked); stop segments: a speed penalty only."""
        self.t += 1
        self._push(sim)
        if self.t * self.dt >= (self.seg + 1) * self.seg_s and self.seg + 1 < len(self.segments):        # segment switch: new frame, mask the first second
            self.seg += 1
            self.h0 = self._heading(sim)
            self.hist.clear()
            self._push(sim)
            self.mask = int(round(MASK_S * self.cfg.control_hz))
        self.mask = max(self.mask - 1, 0)
        r, terms = 0.0, {"T": 0.0, "stop": 0.0}
        self.last_T = self.last_stop = None
        if self.mask == 0 and len(self.hist) >= 2:
            com = np.stack([h[0] for h in self.hist])
            ang = np.stack([h[1] for h in self.hist])
            v, yaw = M.window_velocity(com, ang, (len(self.hist) - 1) * self.dt, self.h0)
            T = M.t_score(v, yaw, self.cmd)
            if T is None:
                self.last_stop = M.stop_score(v, yaw)
                terms["stop"] = -STOP_PENALTY_W * min(float(np.hypot(v[0], v[1])) / M.STOP_SPEED_M_S, 1.0)
            else:
                self.last_T = T
                terms["T"] = T
            r = terms["T"] + terms["stop"]
        self.log.append({"seg": self.seg, "T": self.last_T, "stop": self.last_stop})
        return r, terms

    @property
    def dt(self) -> float:
        """Control tick (s)."""
        return 1.0 / self.cfg.control_hz

    def reward_extra(self, sim: PetSim, a_full: np.ndarray, prev_action: np.ndarray, st: dict[str, Any]) -> dict[str, float]:
        """Homeostasis: bus mean / peak, T_coil, action rate (same definitions and weights as S2)."""
        out = {"bus_mean": -W_BUS_MEAN * max(st["bus_mean_2s"] - BUS_MEAN_A, 0.0) ** 2, "bus_peak": -W_BUS_PEAK * max(st["bus_tick"] - BUS_PEAK_A, 0.0) ** 2}
        if st["temp"] is not None:
            out["tcoil"] = -W_TCOIL * (max(float(np.max(st["temp"])) - T_KNEE_C, 0.0) / 10.0) ** 2
        prev = self._prev_full if self._prev_full is not None else np.zeros_like(a_full)
        out["action_rate"] = -W_RATE * float(np.mean((np.asarray(a_full) - prev) ** 2))
        self._prev_full = np.asarray(a_full, float).copy()
        return out
