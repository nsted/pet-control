"""Inference-only trained gait: no stable-baselines3, torch or training code needed at run time.

``export_policy`` converts a trained SB3 PPO run into ``policy.npz`` (actor weights + observation normalisation). ``GaitPolicy``
loads that file and maps a ``Snapshot`` to joint-angle targets using the same observation builder and action mapping
(EMA + slew limit) as ``PetEnv``, so it can drive the sim or, later, a control scheme.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from petctl.rl.config import N_JOINTS, EnvConfig
from petctl.rl.obs import ObsBuilder, Snapshot

POLICY_FILE = "policy.npz"


def export_policy(run: Path) -> Path:
    """Write ``policy.npz`` for the PPO run in ``run`` (needs torch + SB3; run once after training)."""
    import stable_baselines3 as sb3
    from stable_baselines3.common.vec_env import VecNormalize
    import pickle

    model = sb3.PPO.load(str(run / "model"), device="cpu")
    with open(run / "vecnormalize.pkl", "rb") as f:
        vn: VecNormalize = pickle.load(f)
    pol = model.policy
    layers = [m for m in pol.mlp_extractor.policy_net if hasattr(m, "weight")] + [pol.action_net]
    arrays: dict[str, np.ndarray] = {"n_layers": np.array(len(layers)), "obs_mean": vn.obs_rms.mean, "obs_var": vn.obs_rms.var,
                                     "clip_obs": np.array(vn.clip_obs), "eps": np.array(vn.epsilon)}
    for i, lin in enumerate(layers):
        arrays[f"w{i}"] = lin.weight.detach().numpy().astype(np.float64)
        arrays[f"b{i}"] = lin.bias.detach().numpy().astype(np.float64)
    out = run / POLICY_FILE
    np.savez(out, **arrays)
    return out


class GaitPolicy:
    """Deterministic trained gait. ``reset`` once with the first snapshot, then call ``step`` every 1/control_hz seconds."""

    def __init__(self, run: Path) -> None:
        """Load ``policy.npz`` and ``config.json`` from a run directory."""
        self.cfg = EnvConfig.from_dict(json.loads((run / "config.json").read_text()))
        z = np.load(run / POLICY_FILE)
        n = int(z["n_layers"])
        self._w = [z[f"w{i}"] for i in range(n)]
        self._b = [z[f"b{i}"] for i in range(n)]
        self._mean, self._var = z["obs_mean"], z["obs_var"]
        self._clip, self._eps = float(z["clip_obs"]), float(z["eps"])
        self.obs = ObsBuilder(self.cfg)
        self._tgt = np.zeros(N_JOINTS)

    def reset(self, first: Snapshot) -> None:
        """Start a new run: clear histories and targets."""
        self.obs.reset(first)
        self._tgt = np.zeros(N_JOINTS)

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
        a = self.action(self.obs.build(snap))
        self.obs.push_action(a)
        c = self.cfg
        smoothed = c.action_ema * self._tgt + (1 - c.action_ema) * c.action_scale_rad * a
        self._tgt = self._tgt + np.clip(smoothed - self._tgt, -c.max_target_step_rad, c.max_target_step_rad)
        return self._tgt.copy()
