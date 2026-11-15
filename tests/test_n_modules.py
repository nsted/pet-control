"""Configurable body length (8..15 modules): 8-module model unchanged, longer chains settle and move, RL stack follows ``n_joints``."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

from petctl.rl.config import DEFAULT_URDF_DIR, EnvConfig  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.obs import ObsBuilder, Snapshot  # noqa: E402
from petctl.rl.policy import POLICY_FILE, GaitPolicy  # noqa: E402
from petctl.sim.model_builder import BuildParams, PetSim, build_mjcf  # noqa: E402

_URDF = DEFAULT_URDF_DIR
_HAS_URDF = os.path.isdir(_URDF)
_S8 = Path(__file__).resolve().parents[1] / "data" / "rl" / "curriculum_fast" / "s8_obstacle_field" / "iter_10"

# sha256 of build_mjcf(...) taken at the commit before n_modules existed (8 modules must stay byte-identical)
_HASH_8 = {
    "box": ("c137a334c5d2a9fe0cec9e09b3e8a3b48437a802e19a08271c3e50c1c14c1f89", lambda: BuildParams()),
    "hanging": ("1120a8493f1aa0296afe35faca02bb94ce26797f174abf80fb4a4984fecae0c3", lambda: BuildParams(hanging=True)),
}
_HASH_8_URDF = {
    "urdf": ("9ec012c8a9f81a9718a30816f553e11dab653ffa53bc2b4c4d5d11b3d55e09aa", lambda: BuildParams(urdf_dir=_URDF)),
    "urdf_dt": ("91ec50ca8439a312c2ef50bcfdbdffa81d464d4731a6b6408e40082777a567a8", lambda: BuildParams(urdf_dir=_URDF, dt=0.004, kp=1.3, delay_s=0.1)),
    "obst": ("4371b68a040cd0e59e2d14a1f31272012899440f0b733bf9ebd53826c6d84f0c", lambda: BuildParams(
        urdf_dir=_URDF, walls=((1.0, 0.2, 0.3, 1.2),), obstacles=(("box", 0.5, 0.5, 0.1, 0.1, 0.2), ("cyl", 1, 1, 0, 0.1, 0.1)))),
}


def _sha(p: BuildParams) -> str:
    return hashlib.sha256(build_mjcf(p)[0].encode()).hexdigest()


@pytest.mark.parametrize("name", list(_HASH_8))
def test_eight_module_mjcf_bit_identical(name: str) -> None:
    want, make = _HASH_8[name]
    assert _sha(make()) == want


@pytest.mark.skipif(not _HAS_URDF, reason="Fusion URDF export not present")
@pytest.mark.parametrize("name", list(_HASH_8_URDF))
def test_eight_module_urdf_mjcf_bit_identical(name: str) -> None:
    want, make = _HASH_8_URDF[name]
    assert _sha(make()) == want


def test_config_validation_and_backcompat() -> None:
    with pytest.raises(ValueError):
        EnvConfig(n_modules=7)
    with pytest.raises(ValueError):
        EnvConfig(n_modules=16)
    with pytest.raises(ValueError):
        BuildParams(n_modules=16)
    assert EnvConfig().n_joints == 7 and EnvConfig(n_modules=15).n_joints == 14
    d = EnvConfig().to_dict()
    d.pop("n_modules")
    assert EnvConfig.from_dict(d).n_modules == 8                     # configs saved before the field existed
    assert EnvConfig(n_modules=11).len_norm == pytest.approx(10 / 7) and EnvConfig().len_norm == 1.0
    p = BuildParams(n_modules=11)
    assert len(p.masses) == 11 and len(p.cap_nm) == len(p.coulomb) == 10 and p.coulomb[-1] == BuildParams().coulomb[-1]


def _serpenoid_fwd_cm(sim: PetSim, n: int, seconds: float = 10.0) -> float:
    """Plain travelling wave (amp 0.5 rad, 1 Hz, phi +1): head displacement along the initial heading (cm)."""
    nj = n - 1
    sim.step(np.zeros(nj), 1.0)
    p0 = sim.head_pos().copy()
    mat = sim.data.site_xmat[sim.model.site("head").id].reshape(3, 3)
    fwd = mat[:, 0].copy()
    fwd[2] = 0.0
    fwd /= np.linalg.norm(fwd)
    for i in range(int(seconds / 0.05)):
        sim.step(0.5 * np.sin(2 * np.pi * 1.0 * i * 0.05 - 1.0 * np.arange(1, nj + 1)), 0.05)
    return 100 * float((sim.head_pos() - p0) @ fwd)


@pytest.mark.skipif(not _HAS_URDF, reason="Fusion URDF export not present")
@pytest.mark.parametrize("n", [9, 10, 11, 15])
@pytest.mark.parametrize("urdf", [False, True])
def test_long_model_settles(n: int, urdf: bool) -> None:
    """n != 8 always uses the generated URDF variant (``urdf_dir`` None = the owner's default folder as the source)."""
    sim = PetSim(BuildParams(n_modules=n, urdf_dir=_URDF if urdf else None, delay_s=0.0, dt=0.004))
    assert sim.model.nu == n - 1 and sim.model.nbody == n + 1 and len(sim.joint_pos()) == n - 1
    sim.step(np.zeros(n - 1), 3.0)
    assert np.all(np.isfinite(sim.data.qpos))
    assert np.abs(sim.joint_pos()).max() < 0.05 and np.abs(sim.data.qvel).max() < 0.1
    up = sim.data.site_xmat[sim.model.site("imu").id].reshape(3, 3)[2, 2]
    assert up > 0.98


@pytest.mark.skipif(not _HAS_URDF, reason="Fusion URDF export not present")
@pytest.mark.parametrize("n", [11, 15])
def test_long_model_serpenoid_moves(n: int) -> None:
    sim = PetSim(BuildParams(n_modules=n, urdf_dir=_URDF, cap_nm=(0.164,) * (n - 1)))
    assert abs(_serpenoid_fwd_cm(sim, n)) > 5.0


def test_env_n_modules_11() -> None:
    cfg = EnvConfig(n_modules=11, task="goal_reach", episode_s=2.0)
    env = PetEnv(cfg)
    assert env.action_space.shape == (10,)
    obs, _ = env.reset(seed=0)
    assert obs.shape == (ObsBuilder(cfg).size,) == env.observation_space.shape
    for _ in range(3):
        obs, r, *_ = env.step(env.action_space.sample())
    assert np.isfinite(obs).all() and np.isfinite(r)
    env7 = PetEnv(EnvConfig(task="goal_reach"))
    assert env.observation_space.shape[0] - env7.observation_space.shape[0] == 3 * (3 + 4)   # 3 joint blocks + 4 history; touch off for this task


def test_gait_policy_roundtrip_and_mismatch(tmp_path: Path) -> None:
    cfg = EnvConfig(n_modules=11, task="goal_reach", episode_s=1.0)
    env = PetEnv(cfg)
    n, rng = env.observation_space.shape[0], np.random.default_rng(0)
    np.savez(tmp_path / POLICY_FILE, n_layers=np.array(2), obs_mean=np.zeros(n), obs_var=np.ones(n), clip_obs=np.array(10.0), eps=np.array(1e-8),
             n_joints=np.array(10), w0=rng.normal(0, 0.1, (16, n)), b0=np.zeros(16), w1=rng.normal(0, 0.1, (10, 16)), b1=np.zeros(10))
    (tmp_path / "config.json").write_text(json.dumps(cfg.to_dict()))
    pol = GaitPolicy(tmp_path)
    obs, _ = env.reset(seed=1)
    a = pol.action(obs)
    assert a.shape == (10,)
    snap = Snapshot(np.zeros(10), np.array([1.0, 0, 0, 0]), np.zeros(3), np.full(10, 0.164), np.zeros(pol.obs.extra_dim, np.float32))
    pol.reset(snap)
    assert pol.step(snap).shape == (10,) and snap.torque.shape == (10,)
    cfg8 = tmp_path / "bad"
    cfg8.mkdir()
    np.savez(cfg8 / POLICY_FILE, **{k: v for k, v in np.load(tmp_path / POLICY_FILE).items()})
    (cfg8 / "config.json").write_text(json.dumps(EnvConfig(task="goal_reach").to_dict()))
    with pytest.raises(ValueError, match="10 joints"):
        GaitPolicy(cfg8)
    legacy = tmp_path / "legacy"                                                       # exported before the field existed: 7 joints
    legacy.mkdir()
    np.savez(legacy / POLICY_FILE, **{k: v for k, v in np.load(tmp_path / POLICY_FILE).items() if k != "n_joints"})
    (legacy / "config.json").write_text(json.dumps(cfg.to_dict()))
    with pytest.raises(ValueError, match="7 joints"):
        GaitPolicy(legacy)


# 3-episode gate-style rollout of the S8 policy (seeds 10000..10002, nominal DR) recorded before n_modules existed
_S8_REF = [(-108.861156, 665, 0.690518), (-917.142686, 2272, 1.980354), (686.315318, 1784, 2.698437)]


@pytest.mark.skipif(not _S8.is_dir(), reason="S8 iter_10 run not present")
def test_s8_policy_rollout_unchanged() -> None:
    from petctl.rl.curriculum.stages import get_stage

    stage = get_stage("s8_obstacle_field")
    pol = GaitPolicy(_S8)
    assert pol.cfg.n_modules == 8
    cfg = pol.cfg
    cfg.task_params = dict(stage.gate.task_params)
    cfg.dr.scale = 0.0
    if stage.gate.episode_s is not None:
        cfg.episode_s = stage.gate.episode_s
    env = PetEnv(cfg)
    for i, (r_ref, steps_ref, dist_ref) in enumerate(_S8_REF):
        obs, _ = env.reset(seed=stage.gate.seed0 + i)
        done, total, steps, info = False, 0.0, 0, {}
        while not done:
            obs, r, te, tr, info = env.step(pol.action(obs))
            total, steps, done = total + r, steps + 1, te or tr
        assert steps == steps_ref and total == pytest.approx(r_ref, abs=1e-4) and info["metrics"]["distance"] == pytest.approx(dist_ref, abs=1e-5)
