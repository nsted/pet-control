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


def make_env_fn(cfg: EnvConfig, rank: int, seed: int, asymmetric: bool = False) -> Callable[[], Any]:
    """Factory for vectorised envs: each worker gets its own seed stream. ``asymmetric``: append the privileged vector to the observation (see ``petctl.rl.asym``)."""
    def _f() -> Any:
        from stable_baselines3.common.monitor import Monitor
        env: Any = PetEnv(cfg)
        if asymmetric:
            from petctl.rl.asym import PrivilegedObsWrapper
            env = PrivilegedObsWrapper(env)
        env.reset(seed=seed + rank)
        return Monitor(env)
    return _f


def column_maps(old: EnvConfig, new: EnvConfig, old_asym: bool, new_asym: bool) -> dict[int, tuple[int, np.ndarray]]:
    """Map the input columns of a source run's first layers onto the new env's, by observation block name (``ObsBuilder.blocks``).

    Returns ``{old input width: (new input width, index)}`` for the actor (deployable observation) and, when both runs use the asymmetric critic, the critic (deployable +
    privileged vector, which stays the last block); ``index[i]`` is the new column of old column ``i`` or -1 (block absent or of a different size in the new layout).
    """
    from petctl.rl.obs import ObsBuilder

    out: dict[int, tuple[int, np.ndarray]] = {}
    for with_priv in (False, True):
        if with_priv and not (old_asym and new_asym):
            continue
        ob, nb = ObsBuilder(old).blocks(), ObsBuilder(new).blocks()
        if with_priv:
            ob, nb = ob + [("privileged", 17 + 7 * old.n_modules)], nb + [("privileged", 17 + 7 * new.n_modules)]
        starts_new, c = {}, 0
        for name, size in nb:
            starts_new[name] = (c, size)
            c += size
        idx = []
        for name, size in ob:
            if name in starts_new and starts_new[name][1] == size:
                idx.extend(range(starts_new[name][0], starts_new[name][0] + size))
            else:
                idx.extend([-1] * size)
        out[sum(s for _, s in ob)] = (sum(s for _, s in nb), np.array(idx, int))
    return out


def init_hold_output(model: Any, n_joints: int, h0: float = 0.5) -> None:
    """``hold`` action mode: start the hold channel (policy output ``n_joints``) at ``h = h0`` (default 0.5: EMA 0.65) by setting its last-layer bias to ``2 h0 - 1`` and zeroing its weights row."""
    import torch

    net = model.policy.action_net
    with torch.no_grad():
        net.weight[n_joints].zero_()
        net.bias[n_joints] = 2.0 * h0 - 1.0


def rescale_joint_outputs(model: Any, n_joints: int, factor: float) -> None:
    """Multiply the actor's mean-output weights and bias of the ``n_joints`` JOINT channels by ``factor`` (the hold channel and the value network are untouched).

    Used when ``action_scale_rad`` is widened from ``s0`` to ``s1``: with ``factor = s0 / s1`` (and ``action_obs_scale = s1 / s0`` so the action history keeps its units) the commanded
    targets of actions within +-1 are unchanged and only the reachable range grows.
    """
    import torch

    net = model.policy.action_net
    with torch.no_grad():
        net.weight[:n_joints] *= float(factor)
        net.bias[:n_joints] *= float(factor)


def warm_start(model: Any, venv: Any, src: Path, new_cfg: EnvConfig | None = None, new_asym: bool = False, copy_ret: bool = False) -> dict[str, int]:
    """Initialise ``model``/``venv`` from the PPO run ``src`` (``model.zip`` + ``vecnormalize.pkl``), tolerating a larger observation.

    Tensors whose shape matches are copied as-is. For a first-layer weight with more input columns than the source, the overlapping
    leading columns are copied and the new ones start at zero (new observation entries are appended at the end, see ``ObsBuilder``), so the
    policy initially behaves exactly like the source and learns to use the new inputs. Observation normalisation is copied likewise.
    ``new_cfg`` (the new env's config; the source's is read from ``src/config.json``): when the observation LAYOUTS differ (a block such as the touch block sits in the
    middle of the new vector) the first-layer columns and the normalisation statistics are mapped block by block (``column_maps``) instead of by position; new columns start at zero.
    ``copy_ret``: also copy the reward-normalisation statistics (a continuation of the same task; leave False when the reward changes).
    Returns counts of copied / adapted / skipped tensors.
    """
    import pickle

    import stable_baselines3 as sb3
    import torch

    old = sb3.PPO.load(str(src / "model"), device="cpu").policy.state_dict()
    new = model.policy.state_dict()
    maps: dict[int, tuple[int, np.ndarray]] = {}
    if new_cfg is not None and (src / "config.json").exists():
        old_cfg = EnvConfig.from_dict(json.loads((src / "config.json").read_text()))
        old_meta = json.loads((src / "meta.json").read_text()) if (src / "meta.json").exists() else {}
        from petctl.rl.obs import ObsBuilder
        if ObsBuilder(old_cfg).blocks() != ObsBuilder(new_cfg).blocks():
            maps = column_maps(old_cfg, new_cfg, bool(old_meta.get("asymmetric")), new_asym)
    n = {"copied": 0, "adapted": 0, "skipped": 0}
    for k, v in new.items():
        o = old.get(k)
        if o is None:
            n["skipped"] += 1
        elif o.shape == v.shape and not (maps and o.ndim == 2 and o.shape[1] in maps and k.endswith(".0.weight")):
            new[k] = o.clone()
            n["copied"] += 1
        elif maps and o.ndim == 2 and o.shape[0] == v.shape[0] and o.shape[1] in maps and maps[o.shape[1]][0] == v.shape[1]:
            z = torch.zeros_like(v)
            idx = maps[o.shape[1]][1]
            ok = idx >= 0
            z[:, torch.as_tensor(idx[ok])] = o[:, torch.as_tensor(np.flatnonzero(ok))]
            new[k] = z
            n["adapted"] += 1
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
    old_w, new_w = len(vn.obs_rms.mean), len(venv.obs_rms.mean)
    if maps and old_w in maps and maps[old_w][0] == new_w:
        idx = maps[old_w][1]
        ok = idx >= 0
        venv.obs_rms.mean[idx[ok]], venv.obs_rms.var[idx[ok]] = vn.obs_rms.mean[ok], vn.obs_rms.var[ok]
        venv.obs_rms.count = vn.obs_rms.count
    else:
        m = min(old_w, new_w)
        venv.obs_rms.mean[:m], venv.obs_rms.var[:m], venv.obs_rms.count = vn.obs_rms.mean[:m], vn.obs_rms.var[:m], vn.obs_rms.count
    if copy_ret:
        venv.ret_rms = vn.ret_rms
    return n


def lr_schedule(lr_const: bool = False) -> Any:
    """PPO learning-rate schedule (f = remaining progress 1 -> 0): ``PETCTL_LR`` (default 3e-4) x f, or CONSTANT ``PETCTL_LR`` when ``lr_const`` or ``PETCTL_LR_CONST`` is 1 / true / yes (read per call)."""
    def schedule(f: float) -> float:
        base = float(os.environ.get("PETCTL_LR", "3e-4"))
        return base if lr_const or os.environ.get("PETCTL_LR_CONST", "0").strip().lower() in ("1", "true", "yes") else base * f
    return schedule


def train(cfg: EnvConfig, steps: int, n_envs: int, out: Path, seed: int = 0, algo: str = "ppo", hours: float | None = None,
          ent_coef: float = 0.0, init_from: Path | None = None, video: bool = True, stop_when_gate_passes: Any = None, asymmetric: bool = False,
          extra_callbacks: list[Any] | None = None, warm_copy_ret: bool = False, log_std_init: float | None = None, lr_const: bool = False, rescale_joints: float | None = None) -> Path:
    """Train and save ``model.zip``, ``vecnormalize.pkl``, ``config.json`` and ``meta.json`` under ``out``.

    ``init_from``: a PPO run directory to warm-start from (see ``warm_start``). ``video=False`` skips the end-of-run video in docs/media. ``stop_when_gate_passes``: a curriculum ``Stage``; training stops as soon as its gate passes.
    ``warm_copy_ret``: with ``init_from``, also copy the reward-normalisation statistics (continuing the same task). ``log_std_init``: initial policy log-std (default -1.0; with ``init_from`` it is re-applied after the warm start, which would otherwise copy the source's; None with ``init_from`` keeps the source's trained value). ``lr_const`` (or env ``PETCTL_LR_CONST`` = 1): constant learning rate ``PETCTL_LR`` (default 3e-4) instead of the linear decay to 0, so a continuation does not restart the decay. ``rescale_joints``: with ``init_from``, multiply the joint-channel output weights / bias of the warm-started actor by this factor (``rescale_joint_outputs``). ``asymmetric`` (PPO only, default off): the critic also sees the privileged vector of ``PetEnv.privileged_vector()`` (``petctl.rl.asym``); the actor and the exported ``policy.npz`` use the deployable observation only.
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
    fns = [make_env_fn(cfg, i, seed, asymmetric) for i in range(n_envs)]
    venv: Any = SubprocVecEnv(fns) if n_envs > 1 else DummyVecEnv(fns)
    venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)
    model: Any
    if asymmetric and algo != "ppo":
        raise ValueError("asymmetric critic is implemented for PPO only")
    policy: Any = "MlpPolicy"
    pkw: dict[str, Any] = {"net_arch": [256, 256], "log_std_init": -1.0 if log_std_init is None else float(log_std_init)}
    if asymmetric:
        from petctl.rl.asym import AsymActorCriticPolicy
        policy, pkw = AsymActorCriticPolicy, {**pkw, "n_policy": PetEnv(cfg).observation_space.shape[0]}
    learning_rate = lr_schedule(lr_const)
    if algo == "ppo":
        model = sb3.PPO(policy, venv, n_steps=int(os.environ.get("PETCTL_NSTEPS", "256")), batch_size=2048 if n_envs >= 8 else 256, n_epochs=8, gamma=float(os.environ.get("PETCTL_GAMMA", "0.99")), gae_lambda=float(os.environ.get("PETCTL_GAE", "0.95")),
                        clip_range=0.2, ent_coef=ent_coef, learning_rate=learning_rate, seed=seed, verbose=1,
                        policy_kwargs=pkw, tensorboard_log=str(out / "tb"))
    else:
        model = sb3.SAC("MlpPolicy", venv, seed=seed, verbose=1, tensorboard_log=str(out / "tb"))
    if cfg.action_mode == "hold" and init_from is None:
        init_hold_output(model, cfg.n_joints)
    if init_from is not None:
        print(f"[warm-start] from {init_from}: {warm_start(model, venv, init_from, cfg, asymmetric, warm_copy_ret)}", flush=True)
        src_cfg = init_from / "config.json"
        if cfg.action_mode == "hold" and (not src_cfg.exists() or json.loads(src_cfg.read_text()).get("action_mode") != "hold"):      # the source has no hold output: start it at h = 0.5
            init_hold_output(model, cfg.n_joints)
        if log_std_init is not None:
            model.policy.log_std.data.fill_(float(log_std_init))
        if rescale_joints is not None:                      # wider action range: keep the commanded targets of the source policy (see ``rescale_joint_outputs``)
            rescale_joint_outputs(model, cfg.n_joints, rescale_joints)
            print(f"[warm-start] joint output channels rescaled by {rescale_joints:.4f}", flush=True)
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
    from stable_baselines3.common.callbacks import CallbackList
    from petctl.rl.records import ProgressExportCallback
    from petctl.rl.hug_buffer import BUFFER_ENV
    from petctl.rl.records import HugBufferCallback
    buf_cbs = [HugBufferCallback(os.environ[BUFFER_ENV])] if os.environ.get(BUFFER_ENV) else []     # hug start buffer: merged / persisted / broadcast once per rollout
    callback = CallbackList([callback, ProgressExportCallback(out), *buf_cbs, *(extra_callbacks or [])])      # progress_policy.npz / progress.json every 20 min (PETCTL_PROGRESS_EXPORT_S)
    model.learn(total_timesteps=steps, progress_bar=False, callback=callback)
    model.save(str(out / "model"))
    venv.save(str(out / "vecnormalize.pkl"))
    (out / "config.json").write_text(json.dumps(cfg.to_dict(), indent=1))
    (out / "meta.json").write_text(json.dumps({"git": _git_hash(), "mujoco": mujoco.__version__, "sb3": sb3.__version__, "torch": torch.__version__,
                                               "steps": steps, "n_envs": n_envs, "seed": seed, "algo": algo, "wall_s": time.time() - t0,
                                               **({"asymmetric": True} if asymmetric else {})}, indent=1))
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
    env: Any = PetEnv(cfg)
    meta = out / "meta.json"
    if meta.exists() and json.loads(meta.read_text()).get("asymmetric"):          # asymmetric critic: the model's observation carries the privileged tail
        from petctl.rl.asym import PrivilegedObsWrapper
        env = PrivilegedObsWrapper(env)
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
