"""PPO training and deterministic evaluation with reproducible artefacts (config, versions, git hash, VecNormalize stats)."""

from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path
from typing import Any, Callable

import numpy as np

from petctl.rl.config import EnvConfig
from petctl.rl.env import PetEnv


def _git_hash() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return "unknown"


def make_env_fn(cfg: EnvConfig, rank: int, seed: int) -> Callable[[], Any]:
    """Factory for vectorised envs: each worker gets its own seed stream."""
    def _f() -> Any:
        from stable_baselines3.common.monitor import Monitor
        env = PetEnv(cfg)
        env.reset(seed=seed + rank)
        return Monitor(env)
    return _f


def warm_start(model: Any, venv: Any, src: Path) -> dict[str, int]:
    """Initialise ``model``/``venv`` from the PPO run ``src`` (``model.zip`` + ``vecnormalize.pkl``), tolerating a larger observation.

    Tensors whose shape matches are copied as-is. For a first-layer weight with more input columns than the source, the overlapping
    leading columns are copied and the new ones start at zero (new observation entries are appended at the end, see ``ObsBuilder``), so the
    policy initially behaves exactly like the source and learns to use the new inputs. Observation normalisation is copied likewise.
    Returns counts of copied / adapted / skipped tensors.
    """
    import pickle

    import stable_baselines3 as sb3
    import torch

    old = sb3.PPO.load(str(src / "model"), device="cpu").policy.state_dict()
    new = model.policy.state_dict()
    n = {"copied": 0, "adapted": 0, "skipped": 0}
    for k, v in new.items():
        o = old.get(k)
        if o is None:
            n["skipped"] += 1
        elif o.shape == v.shape:
            new[k] = o.clone()
            n["copied"] += 1
        elif o.ndim == 2 and o.shape[0] == v.shape[0] and o.shape[1] < v.shape[1]:
            z = torch.zeros_like(v)
            z[:, :o.shape[1]] = o
            new[k] = z
            n["adapted"] += 1
        else:
            n["skipped"] += 1
    model.policy.load_state_dict(new)
    with open(src / "vecnormalize.pkl", "rb") as f:
        vn = pickle.load(f)
    m = min(len(vn.obs_rms.mean), len(venv.obs_rms.mean))
    venv.obs_rms.mean[:m], venv.obs_rms.var[:m], venv.obs_rms.count = vn.obs_rms.mean[:m], vn.obs_rms.var[:m], vn.obs_rms.count
    return n


def train(cfg: EnvConfig, steps: int, n_envs: int, out: Path, seed: int = 0, algo: str = "ppo", hours: float | None = None,
          ent_coef: float = 0.0, init_from: Path | None = None, video: bool = True, stop_when_gate_passes: Any = None) -> Path:
    """Train and save ``model.zip``, ``vecnormalize.pkl``, ``config.json`` and ``meta.json`` under ``out``.

    ``init_from``: a PPO run directory to warm-start from (see ``warm_start``). ``video=False`` skips the end-of-run video in docs/media. ``stop_when_gate_passes``: a curriculum ``Stage``; training stops as soon as its gate passes.
    """
    # Worker processes inherit this: unpinned BLAS/OpenMP threads oversubscribe the cores and cost ~3.7x throughput.
    for var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "MKL_NUM_THREADS"):
        os.environ.setdefault(var, "1")
    import mujoco  # type: ignore[import-untyped]
    import stable_baselines3 as sb3
    import torch
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize

    torch.set_num_threads(1)
    out.mkdir(parents=True, exist_ok=True)
    (out / "config.json").write_text(json.dumps(cfg.to_dict(), indent=1))   # written first so best/checkpoint policies are replayable mid-run
    fns = [make_env_fn(cfg, i, seed) for i in range(n_envs)]
    venv: Any = SubprocVecEnv(fns) if n_envs > 1 else DummyVecEnv(fns)
    venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)
    model: Any
    if algo == "ppo":
        model = sb3.PPO("MlpPolicy", venv, n_steps=int(os.environ.get("PETCTL_NSTEPS", "256")), batch_size=2048 if n_envs >= 8 else 256, n_epochs=8, gamma=float(os.environ.get("PETCTL_GAMMA", "0.99")), gae_lambda=0.95,
                        clip_range=0.2, ent_coef=ent_coef, learning_rate=lambda f: float(os.environ.get("PETCTL_LR", "3e-4")) * f, seed=seed, verbose=1,
                        policy_kwargs={"net_arch": [256, 256], "log_std_init": -1.0}, tensorboard_log=str(out / "tb"))
    else:
        model = sb3.SAC("MlpPolicy", venv, seed=seed, verbose=1, tensorboard_log=str(out / "tb"))
    if init_from is not None:
        print(f"[warm-start] from {init_from}: {warm_start(model, venv, init_from)}", flush=True)
    t0 = time.time()
    callback: Any
    if cfg.task in ("reach_still", "jump"):
        from petctl.rl.records import RecordCallback
        callback = RecordCallback(out, hours)
    else:
        from petctl.rl.records import EpisodeLogCallback
        callback = EpisodeLogCallback(out, hours)
        if stop_when_gate_passes is not None:
            from stable_baselines3.common.callbacks import CallbackList

            from petctl.rl.curriculum.early_stop import GateStopCallback
            callback = CallbackList([callback, GateStopCallback(out, stop_when_gate_passes)])
    model.learn(total_timesteps=steps, progress_bar=False, callback=callback)
    model.save(str(out / "model"))
    venv.save(str(out / "vecnormalize.pkl"))
    (out / "config.json").write_text(json.dumps(cfg.to_dict(), indent=1))
    (out / "meta.json").write_text(json.dumps({"git": _git_hash(), "mujoco": mujoco.__version__, "sb3": sb3.__version__, "torch": torch.__version__,
                                               "steps": steps, "n_envs": n_envs, "seed": seed, "algo": algo, "wall_s": time.time() - t0}, indent=1))
    venv.close()
    if algo == "ppo":
        from petctl.rl.policy import export_policy
        export_policy(out)                                  # inference-only artefact: policy.npz
    if video and cfg.task not in ("reach_still", "jump"):
        from petctl.rl.video import record_best_video
        record_best_video(out)                              # record runs write a whole session of videos instead (see records.py)
    return out


def evaluate(out: Path, episodes: int = 20, dr_scale: float | None = None) -> dict[str, float]:
    """Deterministic rollouts over fixed seeds. Returns median/IQR of distance and mean saturation fraction."""
    import stable_baselines3 as sb3
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    cfg = EnvConfig.from_dict(json.loads((out / "config.json").read_text()))
    if dr_scale is not None:
        cfg.dr.scale = dr_scale
    env = PetEnv(cfg)
    venv: Any = VecNormalize.load(str(out / "vecnormalize.pkl"), DummyVecEnv([lambda: env]))
    venv.training, venv.norm_reward = False, False
    model = sb3.PPO.load(str(out / "model"))
    dist, sat, mets = [], [], []
    for ep in range(episodes):
        first, _ = env.reset(seed=10_000 + ep)  # fixed held-out seeds
        obs = venv.normalize_obs(first)[None]
        done, d, s, n = False, 0.0, 0.0, 0
        while not done:
            act, _ = model.predict(obs, deterministic=True)
            o, _, term, trunc, info = env.step(act[0])
            obs = venv.normalize_obs(o)[None]
            done = term or trunc
            d, s, n = info["distance"], s + -info["terms"].get("saturation", 0.0), n + 1
        dist.append(d)
        if "metrics" in info:
            mets.append(info["metrics"])
        sat.append(s / max(n, 1))
    q = np.percentile(dist, [25, 50, 75])
    res = {"distance_median": float(q[1]), "distance_q1": float(q[0]), "distance_q3": float(q[2]), "saturation_cost_mean": float(np.mean(sat))}
    if mets:
        for k in mets[0]:
            vals = [m[k] for m in mets if np.isfinite(m[k])]
            res[f"{k}_median"] = float(np.median(vals)) if vals else float("nan")
    return res
