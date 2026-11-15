"""Inference-only trained gait: no stable-baselines3, torch or training code needed at run time.

``export_policy`` converts a trained SB3 PPO run into ``policy.npz`` (actor weights + observation normalisation). ``GaitPolicy``
loads that file and maps a ``Snapshot`` to joint-angle targets using the same observation builder and action mapping
(EMA + slew limit) as ``PetEnv``, so it can drive the sim or, later, a control scheme.
"""

from __future__ import annotations

import dataclasses
import json
import os
from pathlib import Path
from typing import Any

import numpy as np

from petctl.rl.config import N_JOINTS, EnvConfig
from petctl.rl.obs import ObsBuilder, Snapshot, next_target

POLICY_FILE = "policy.npz"


def policy_arrays(model: Any, vn: Any, run: Path) -> dict[str, np.ndarray]:
    """Inference arrays (weights, observation normalisation, joint count) of a live PPO ``model`` + ``VecNormalize`` ``vn`` for the config in ``run``."""
    pol = model.policy
    layers = [m for m in pol.mlp_extractor.policy_net if hasattr(m, "weight")] + [pol.action_net]
    n_pol = getattr(pol.mlp_extractor, "n_policy", None)       # asymmetric critic (petctl.rl.asym): the actor reads only the leading deployable observation entries
    n_obs = vn.obs_rms.mean.shape[0] if n_pol is None else int(n_pol)
    arrays: dict[str, np.ndarray] = {"n_layers": np.array(len(layers)), "obs_mean": np.array(vn.obs_rms.mean[:n_obs]), "obs_var": np.array(vn.obs_rms.var[:n_obs]),
                                     "clip_obs": np.array(vn.clip_obs), "eps": np.array(vn.epsilon)}
    for i, lin in enumerate(layers):
        arrays[f"w{i}"] = lin.weight.detach().cpu().numpy().astype(np.float64)
        arrays[f"b{i}"] = lin.bias.detach().cpu().numpy().astype(np.float64)
    cfg_file = run / "config.json"
    arrays["n_joints"] = np.array(EnvConfig.from_dict(json.loads(cfg_file.read_text())).n_joints if cfg_file.exists() else N_JOINTS)
    return arrays


def export_live_policy(model: Any, vn: Any, run: Path, out_file: str = "progress_policy.npz") -> Path:
    """Atomically write the inference policy of the LIVE training ``model``/``vn`` to ``run/out_file`` (tmp file + rename; safe while training)."""
    out = run / out_file
    tmp = run / (out_file + ".tmp.npz")
    np.savez(tmp, **policy_arrays(model, vn, run))
    os.replace(tmp, out)
    return out


def export_policy(run: Path, model_file: str = "model", norm_file: str = "vecnormalize.pkl", out_file: str = POLICY_FILE) -> Path:
    """Write ``out_file`` for the PPO checkpoint ``model_file``/``norm_file`` in ``run`` (needs torch + SB3)."""
    import stable_baselines3 as sb3
    from stable_baselines3.common.vec_env import VecNormalize
    import pickle

    model = sb3.PPO.load(str(run / model_file), device="cpu")
    with open(run / norm_file, "rb") as f:
        vn: VecNormalize = pickle.load(f)
    out = run / out_file
    np.savez(out, **policy_arrays(model, vn, run))
    return out


class GaitPolicy:
    """Deterministic trained gait. ``reset`` once with the first snapshot, then call ``step`` every 1/control_hz seconds."""

    def __init__(self, run: Path, policy_file: str = POLICY_FILE) -> None:
        """Load ``policy_file`` (default ``policy.npz``) and ``config.json`` from a run directory."""
        self.cfg = EnvConfig.from_dict(json.loads((run / "config.json").read_text()))
        z = np.load(run / policy_file)
        n_file = int(z["n_joints"]) if "n_joints" in z.files else N_JOINTS          # runs exported before configurable length have 7 joints
        if n_file != self.cfg.n_joints:
            raise ValueError(f"{run / policy_file} was exported for {n_file} joints but config.json says n_modules={self.cfg.n_modules} ({self.cfg.n_joints} joints)")
        n = int(z["n_layers"])
        self._w = [z[f"w{i}"] for i in range(n)]
        self._b = [z[f"b{i}"] for i in range(n)]
        self._mean, self._var = z["obs_mean"], z["obs_var"]
        self._clip, self._eps = float(z["clip_obs"]), float(z["eps"])
        self.obs = ObsBuilder(self.cfg)
        self._tgt = np.zeros(self.cfg.n_joints)

    def reset(self, first: Snapshot) -> None:
        """Start a new run: clear histories and targets."""
        self.obs.reset(first)
        random_start = float(self.cfg.task_params.get("start_pose_rad", 0.0)) > 0.0 or float(self.cfg.task_params.get("hug_start_mix", 0.0)) > 0.0      # mirrors PetEnv.reset (random / coil starts)
        self._tgt = np.asarray(first.joint_pos, float).copy() if random_start else np.zeros(self.cfg.n_joints)

    def action(self, obs: np.ndarray) -> np.ndarray:
        """Raw policy output in [-1, 1] for an (un-normalised) observation."""
        x = np.clip((obs - self._mean) / np.sqrt(self._var + self._eps), -self._clip, self._clip)
        for i, (w, b) in enumerate(zip(self._w, self._b)):
            x = w @ x + b
            if i < len(self._w) - 1:
                x = np.tanh(x)
        return np.clip(x, -1.0, 1.0)

    def step(self, snap: Snapshot) -> np.ndarray:
        """Joint-angle targets (rad) for this control tick; mirrors ``PetEnv.step``'s EMA and slew limit."""
        if self.cfg.action_mode in ("delta", "hold"):           # the observation carries the policy's own commanded targets
            snap = dataclasses.replace(snap, target=self._tgt.copy())
        a = self.action(self.obs.build(snap))
        self.obs.push_action(a[:self.cfg.n_joints] * self.cfg.action_obs_scale)             # the action history holds the joint outputs only (the hold channel is not part of it)
        self._tgt = next_target(self.cfg, self._tgt, a)
        return self._tgt.copy()
