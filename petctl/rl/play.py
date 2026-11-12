"""Run a trained ``GaitPolicy`` in the sim (headless or in the MuJoCo viewer)."""

from __future__ import annotations

import time
from pathlib import Path

from petctl.rl.env import PetEnv
from petctl.rl.markers import draw_markers
from petctl.rl.policy import GaitPolicy


def play(run: Path, seconds: float = 15.0, seed: int = 0, dr_scale: float = 0.0, view: bool = False, policy_file: str = "policy.npz") -> dict[str, float]:
    """Drive the sim with the exported policy only (no SB3/torch); returns the task metrics at the end."""
    pol = GaitPolicy(run, policy_file)
    cfg = pol.cfg
    cfg.dr.scale = dr_scale
    cfg.episode_s = seconds
    env = PetEnv(cfg)
    obs, _ = env.reset(seed=seed)
    viewer = None
    if view:
        import mujoco.viewer  # type: ignore[import-untyped]
        viewer = mujoco.viewer.launch_passive(env.sim.model, env.sim.data)
    done = False
    info: dict = {}
    while not done:
        obs, _, term, trunc, info = env.step(pol.action(obs))
        done = term or trunc
        if viewer is not None:
            draw_markers(viewer.user_scn, env.task.markers(env.sim), reset=True)
            viewer.sync()
            time.sleep(env.dt)
            done = done or not viewer.is_running()
    m = info.get("metrics", {"distance": info["distance"]})
    return {k: float(v) for k, v in m.items()}
