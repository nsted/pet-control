"""Start distribution of the v2 hug stage (``hug_start_mix``): three start modes sampled per episode, plus the scripted-coil geometry from M1b.

``straight``  the robot lies straight with its head 5-20 cm (surface gap) from the post at a bearing of +-60 deg (the ordinary goal layout);
``coil``      the body starts in the scripted constant-angle coil round the post (every joint at the M1b angle, joints perturbed +-0.2 rad);
``half``      only the first half of the joints (head end) is at the coil angle, the post sits inside that coil and the rest of the body lies straight.

The coil is laid down exactly as in the M1b protocol (``petctl.rl.hug_search``): loose (start looseness alpha) with the post far away, then the post is put at the centre of
the least-squares circle through the curled modules. The mix is 30/30/40 (coil/half/straight) until the rolling training success crosses a threshold, then 10/20/70; the
switch is a flag FILE (``PETCTL_HUG_MIX_FLAG``) written by ``StartMixCallback`` so the vectorised worker processes see it.
"""

from __future__ import annotations

import os
from collections import deque
from pathlib import Path

import mujoco  # type: ignore[import-untyped]
import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

from petctl.rl.hug_search import FREE_S, LIFT_M, START_PEN_M, circle_centre
from petctl.sim.model_builder import PetSim, post_geom_ids, set_post_xy

MODES = ("straight", "coil", "half")
MIX_EARLY = (0.3, 0.3)          # (coil, half) probabilities; the rest is straight
MIX_LATE = (0.1, 0.2)
MIX_V3_EARLY, MIX_V3_LATE = (0.7, 0.3), (0.3, 0.3)       # hug v3 (coil, half); phase 1 has NO straight starts, phase 2 40 % straight
PERTURB_V3_EARLY, PERTURB_V3_LATE = 0.5, 1.0             # x PERTURB_RAD: +-0.10 rad in phase 1, +-0.20 rad in phase 2
V3_PHASE1_STEPS, V3_STOP_DUTY, V3_SWITCH_DUTY, V3_WINDOW = 1_000_000, 0.3, 0.5, 200
SWITCH_SUCCESS = 0.5            # rolling training success (last SWITCH_WINDOW episodes) above which the mix switches to MIX_LATE
SWITCH_WINDOW = 200
PERTURB_RAD = 0.2               # coil mode: each joint perturbed uniformly by +-this
SETTLE_AFTER_S = 0.5            # hold the loose coil round the placed post before the policy takes over
FLAG_ENV = "PETCTL_HUG_MIX_FLAG"
# M1b constant-angle coil per body length (angle rad, start looseness alpha): data/rl/m1_hug_feasibility/m1b_sweep*.json, best feasible point (duty 1.0, bus <= 4 A)
COIL_TABLE = {8: (0.80, 0.95), 10: (0.90, 0.95), 12: (1.025, 0.96)}


def coil_params(n_modules: int) -> tuple[float, float]:
    """``(angle, alpha)`` of the scripted coil for a body of ``n_modules`` (nearest tabulated length; ties go to the shorter)."""
    k = min(COIL_TABLE, key=lambda m: (abs(m - n_modules), m))
    return COIL_TABLE[k]


def half_joints(n_modules: int) -> int:
    """Number of head-end joints curled in ``half`` mode: ``ceil((n_modules - 1) / 2)``."""
    return (n_modules - 1 + 1) // 2


def mix_probs(switched: bool) -> tuple[float, float]:
    """``(p_coil, p_half)`` before / after the switch."""
    return MIX_LATE if switched else MIX_EARLY


def sample_start_mode(rng: np.random.Generator, p_coil: float, p_half: float) -> str:
    """Draw one of ``MODES`` (straight gets the remaining probability)."""
    u = float(rng.random())
    return "coil" if u < p_coil else "half" if u < p_coil + p_half else "straight"


def mix_switched(flag: str | None = None) -> bool:
    """True once the switch flag file exists (``flag`` or ``$PETCTL_HUG_MIX_FLAG``)."""
    path = flag or os.environ.get(FLAG_ENV)
    return bool(path) and Path(path).exists()


def coil_angles(n_modules: int, mode: str, rng: np.random.Generator, scale: float = 1.0) -> np.ndarray:
    """Target joint angles of the start coil: ``alpha * angle`` on every joint (``coil``, plus uniform +-``PERTURB_RAD * scale``) or on the first ``half_joints`` only (``half``)."""
    ang, alpha = coil_params(n_modules)
    q = np.zeros(n_modules - 1)
    q[:half_joints(n_modules) if mode == "half" else n_modules - 1] = alpha * ang
    if mode == "coil":
        q = q + PERTURB_RAD * scale * rng.uniform(-1.0, 1.0, n_modules - 1)
    return q


def _lay_down(sim: PetSim, q: np.ndarray) -> np.ndarray | None:
    """Lay the body down in pose ``q`` with the post far away, put the post at the centre of the curled modules, hold; the post xy, or None if it would start inside the body."""
    from petctl.rl.capacitive import PostProbe

    n = int(sim.p.n_modules)
    set_post_xy(sim.model, (5.0, 5.0))
    sim.reset(q)
    sim.data.qpos[2] += LIFT_M
    mujoco.mj_forward(sim.model, sim.data)
    sim.step(q, FREE_S)
    big = np.abs(q) > 0.4 * max(float(np.max(np.abs(q))), 1e-9)               # the joints that actually curl; the post goes inside the coil they form
    sel = np.zeros(n, bool)
    for j in np.flatnonzero(big):
        sel[j:j + 2] = True
    post = circle_centre(sim.data.xipos[1:n + 1][sel, :2].copy())
    set_post_xy(sim.model, post)
    mujoco.mj_forward(sim.model, sim.data)
    probe = PostProbe.of(sim)
    if probe is None or probe.read().penetration > START_PEN_M or not np.all(np.isfinite(sim.data.qpos)):
        return None
    sim.step(q, SETTLE_AFTER_S)
    return post


def arrange_coil(sim: PetSim, rng: np.random.Generator, mode: str, perturb: float = 1.0) -> np.ndarray | None:
    """Put the sim into the start pose of ``mode`` (``coil`` / ``half``) round the post and move the post (geom and ``sim.p.post``) to the coil centre.

    Returns the new post position as a goal vector ``(x, y, head z)``, or None if no coil could be placed (post would start inside the body even unperturbed; the post is
    restored to where the layout put it and the caller uses the ordinary straight start). Retries with the perturbation halved each time; ``perturb`` scales the +-0.2 rad (0 = exact coil).
    """
    n = int(sim.p.n_modules)
    orig = sim.model.geom_pos[post_geom_ids(sim.model)[0], :2].copy()
    for scale in (1.0, 0.5, 0.25, 0.0):
        post = _lay_down(sim, coil_angles(n, mode, rng, scale * perturb))
        if post is not None:
            sim.p.post = (float(post[0]), float(post[1]))
            mujoco.mj_forward(sim.model, sim.data)
            return np.array([post[0], post[1], float(sim.head_pos()[2])])
        if mode != "coil":
            break
    set_post_xy(sim.model, orig)
    sim.reset()
    mujoco.mj_forward(sim.model, sim.data)
    return None


class StartMixCallback(BaseCallback):
    """Write the switch flag file once the rolling training success (last ``window`` episodes) exceeds ``threshold`` (the worker processes read it at each reset)."""

    def __init__(self, flag: Path, window: int = SWITCH_WINDOW, threshold: float = SWITCH_SUCCESS) -> None:
        """``flag``: the file to create (the driver exports it as ``$PETCTL_HUG_MIX_FLAG`` before the workers start)."""
        super().__init__()
        self.flag, self.threshold = Path(flag), threshold
        self._succ: deque[float] = deque(maxlen=window)
        self.switched_at: int | None = None

    def _on_step(self) -> bool:
        for done, info in zip(self.locals["dones"], self.locals["infos"]):
            if done and "metrics" in info:
                self._succ.append(float(info["metrics"].get("success", 0.0)))
        if self.switched_at is None and len(self._succ) == self._succ.maxlen and sum(self._succ) / len(self._succ) > self.threshold:
            self.switched_at = int(self.num_timesteps)
            self.flag.write_text(str(self.switched_at))
            print(f"[hug-mix] rolling success {sum(self._succ) / len(self._succ):.2f} > {self.threshold}: mix {MIX_EARLY}+straight -> {MIX_LATE}+straight at {self.switched_at} steps", flush=True)
        return True


class ScriptedCoilEvaluator:
    """Gate evaluator (``run_gate2(evaluator=...)``) that drives the env with a SCRIPTED policy instead of an exported network: the M1b constant-angle coil.

    Same rollout loop and row format as ``PolicyEvaluator``, so the episode, the hug metric and the gate2 aggregation are exactly those of a trained policy. The action is the
    coil target ``angle`` on every joint (``action = angle / action_scale``), ramped linearly from the start looseness ``alpha * angle`` over ``ramp_s`` seconds as in the M1b protocol.
    """

    def __init__(self, n_modules: int, task_params: dict[str, float], dr_scale: float, episode_s: float | None = None, ramp_s: float = 2.0,
                 angle: float | None = None) -> None:
        """``task_params``: the gate's (``hug_start_mix`` and ``hug_force_mode`` are added so the episode starts in the coil)."""
        self.n, self.dr_scale, self.episode_s, self.ramp_s = n_modules, dr_scale, episode_s, ramp_s
        self.angle, self.alpha = coil_params(n_modules)
        if angle is not None:
            self.angle = angle
        self.task_params = {**task_params, "hug_start_mix": 1.0, "hug_force_mode": 1.0}

    def __call__(self, overrides: dict[str, float], seeds: list[int], leak: bool = False, dr_scale: float | None = None) -> list[dict]:
        """One scripted episode per seed."""
        from petctl.rl.config import DEFAULT_URDF_DIR, EnvConfig
        from petctl.rl.env import PetEnv

        cfg = EnvConfig(task="goal_hug", urdf_dir=DEFAULT_URDF_DIR, n_modules=self.n)
        cfg.task_params = {**self.task_params, **overrides}
        cfg.dr.scale = self.dr_scale if dr_scale is None else dr_scale
        cfg.dr.cap_nm = (0.5, 0.5)
        cfg.action_scale_rad, cfg.action_ema = 1.0, 0.3
        if self.episode_s is not None:
            cfg.episode_s = self.episode_s
        env = PetEnv(cfg)
        rows: list[dict] = []
        for s in seeds:
            env.reset(seed=s)
            done, info, ret, steps = False, {}, 0.0, 0
            while not done:
                frac = self.alpha + (1.0 - self.alpha) * min(1.0, steps / max(1.0, self.ramp_s * cfg.control_hz))
                _, r, te, tr, info = env.step(np.full(self.n - 1, frac * self.angle / cfg.action_scale_rad, np.float32))
                ret, steps, done = ret + float(r), steps + 1, te or tr
            rows.append({"seed": int(s), "ret": ret, "steps": steps, "metrics": {k: float(v) for k, v in info.get("metrics", {}).items()}, "gt": {}, "leak_moved_at_step": -1})
        return rows


def v3_phase_decision(duties: list[float] | deque[float], steps: int, switched: bool, phase1_steps: int = V3_PHASE1_STEPS, window: int = V3_WINDOW,
                      switch_duty: float = V3_SWITCH_DUTY, stop_duty: float = V3_STOP_DUTY) -> str:
    """Hug v3 phase logic from the rolling mean ``hold_duty`` of the last ``window`` episodes: ``"none"``, ``"switch"`` (to phase 2) or ``"stop"``.

    Early switch when the full window's mean exceeds ``switch_duty``; at ``phase1_steps`` (still in phase 1) the run stops if the mean (no episodes = 0) is below ``stop_duty``, else it switches.
    """
    if switched:
        return "none"
    last = list(duties)[-window:]
    mean = sum(last) / len(last) if last else 0.0
    if len(last) >= window and mean > switch_duty:
        return "switch"
    if steps >= phase1_steps:
        return "stop" if mean < stop_duty else "switch"
    return "none"


class HugV3PhaseCallback(BaseCallback):
    """Hug v3 start-mix phases: writes the switch flag on ``v3_phase_decision == "switch"``; on ``"stop"`` ends training (``stopped`` / ``rate`` / ``checked_at`` record it)."""

    def __init__(self, flag: Path, phase1_steps: int = V3_PHASE1_STEPS, window: int = V3_WINDOW) -> None:
        """``flag``: the switch flag file (exported by the driver as ``$PETCTL_HUG_MIX_FLAG`` before the workers start)."""
        super().__init__()
        self.flag, self.phase1_steps, self.window = Path(flag), phase1_steps, window
        self._duty: deque[float] = deque(maxlen=window)
        self.switched_at: int | None = None
        self.stopped = False
        self.rate: float | None = None
        self.checked_at: int | None = None

    def _on_step(self) -> bool:
        for done, info in zip(self.locals["dones"], self.locals["infos"]):
            if done and "metrics" in info:
                self._duty.append(float(info["metrics"].get("hold_duty", 0.0)))
        d = v3_phase_decision(self._duty, int(self.num_timesteps), self.switched_at is not None, self.phase1_steps, self.window)
        mean = sum(self._duty) / len(self._duty) if self._duty else 0.0
        if d == "switch":
            self.switched_at = int(self.num_timesteps)
            self.flag.write_text(str(self.switched_at))
            print(f"[hug-v3] rolling hold_duty {mean:.3f}: phase 2 (coil 30 / half 30 / straight 40, perturb 0.2) at {self.switched_at} steps", flush=True)
        elif d == "stop":
            self.stopped, self.rate, self.checked_at = True, mean, int(self.num_timesteps)
            print(f"[hug-v3] rolling hold_duty {mean:.3f} < {V3_STOP_DUTY} at {self.checked_at} steps: STOP RULE", flush=True)
            return False
        return True
