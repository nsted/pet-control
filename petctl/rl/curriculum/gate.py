"""Frozen stage gate: replays an exported policy on fixed held-out seeds and decides pass/fail from the stage's ``Gate``.

Uses only ``policy.npz`` (what would ship), the stage's evaluation task_params and fixed seeds, so it does not depend on the training reward
or on whatever the tuning loop changed. Glitch episodes (physics blow-ups) count against the policy and fail the gate if above ``max_glitch``.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from petctl.rl.curriculum.stages import Stage
from petctl.rl.env import PetEnv
from petctl.rl.policy import GaitPolicy


def _condition(run: Path, stage: Stage, policy_file: str, dr_scale: float) -> dict[str, float]:
    pol = GaitPolicy(run, policy_file)
    cfg = pol.cfg
    cfg.task_params = dict(stage.gate.task_params)
    cfg.dr.scale = dr_scale
    if stage.gate.episode_s is not None:
        cfg.episode_s = stage.gate.episode_s
    env = PetEnv(cfg)
    if env.observation_space.shape[0] != pol._mean.shape[0]:
        raise ValueError(f"policy expects {pol._mean.shape[0]} observation entries but task '{cfg.task}' provides {env.observation_space.shape[0]}")
    rows: list[dict[str, float]] = []
    for i in range(stage.gate.episodes):
        obs, _ = env.reset(seed=stage.gate.seed0 + i)
        done, info = False, {}
        while not done:
            obs, _, term, trunc, info = env.step(pol.action(obs))
            done = term or trunc
        rows.append(info["metrics"])
    succ = np.array([m.get("success", 0.0) for m in rows])
    dist = np.array([m["distance"] for m in rows])
    ttg = [m["time_to_goal_s"] for m in rows if math.isfinite(m.get("time_to_goal_s", float("nan")))]
    return {"dr_scale": dr_scale, "episodes": len(rows), "success_rate": float(succ.mean()), "distance_median": float(np.median(dist)),
            "distance_q1": float(np.percentile(dist, 25)), "glitch_rate": float(np.mean([m.get("glitch", 0.0) for m in rows])),
            "time_to_goal_median_s": float(np.median(ttg)) if ttg else float("nan")}


def run_gate(run: Path, stage: Stage, policy_file: str = "policy.npz") -> dict[str, Any]:
    """Evaluate ``run/policy_file`` against ``stage.gate``; writes ``gate_<stage>.json`` into the run and returns the result dict."""
    nominal = _condition(run, stage, policy_file, 0.0)
    robust = _condition(run, stage, policy_file, stage.gate.dr_scale)
    reasons = []
    if robust["success_rate"] < stage.gate.min_success:
        reasons.append(f"robust success {robust['success_rate']:.2f} < {stage.gate.min_success:.2f}")
    if robust["glitch_rate"] > stage.gate.max_glitch or nominal["glitch_rate"] > stage.gate.max_glitch:
        reasons.append("glitch episodes present")
    res = {"stage": stage.name, "run": str(run), "policy_file": policy_file, "passed": not reasons, "fail_reasons": reasons,
           "nominal": nominal, "robust": robust, "gate": {"min_success": stage.gate.min_success, "task_params": stage.gate.task_params}}
    (run / f"gate_{stage.name}.json").write_text(json.dumps(res, indent=1))
    return res
