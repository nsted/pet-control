"""Bit-identity guard for env speed work: fixed-seed 500-step rollouts of the Regimen v2 task configs must reproduce the stored observations, rewards, flags, info and privileged vectors exactly.

The golden file ``tests/golden/perf_identity.npz`` was generated from the pre-optimisation code (``python -m tests.test_perf_identity --gen``); regenerate only on an intentional behaviour change.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

from petctl.rl.config import DEFAULT_URDF_DIR  # noqa: E402
from petctl.rl.curriculum.runner import build_config  # noqa: E402
from petctl.rl.curriculum.stages import get_stage  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402

GOLDEN = Path(__file__).parent / "golden" / "perf_identity.npz"
STEPS = 500
needs_urdf = pytest.mark.skipif(not os.path.isdir(DEFAULT_URDF_DIR), reason="Fusion URDF export not present")


def _cfg(name: str):  # type: ignore[no-untyped-def]
    if name == "hug10_terrain":
        cfg = build_config(get_stage("m2_hug3"), {"n_modules": 10.0})
        cfg.task_params.update({"terrain_level": 0.5})
    elif name == "hug8_coil":
        cfg = build_config(get_stage("m2_hug3"), {"n_modules": 8.0})
        cfg.task_params.update({"hug_force_mode": 1.0, "hug_perturb_scale": 0.5})
    elif name == "flat8":
        cfg = build_config(get_stage("m2_flat"), {})
    elif name == "terrain10":
        cfg = build_config(get_stage("m2_terrain"), {"n_modules": 10.0})
    else:
        raise KeyError(name)
    return cfg


CASES = ("hug10_terrain", "hug8_coil", "flat8", "terrain10")


def _flat(prefix: str, v: object, out: dict[str, object]) -> None:
    if isinstance(v, dict):
        for k in sorted(v):
            _flat(f"{prefix}/{k}", v[k], out)
    elif isinstance(v, (np.ndarray, list, tuple)):
        out[prefix] = np.asarray(v, float).tolist()
    elif isinstance(v, (bool, np.bool_)):
        out[prefix] = bool(v)
    elif isinstance(v, (int, float, np.floating, np.integer)):
        out[prefix] = float(v)
    else:
        out[prefix] = repr(v)


def rollout(name: str, steps: int = STEPS) -> dict[str, np.ndarray]:
    """Run ``steps`` env steps of case ``name`` (seeded random actions, reset on episode end) and return the recorded arrays."""
    cfg = _cfg(name)
    env = PetEnv(cfg)
    rng = np.random.default_rng(7)
    obs, info = env.reset(seed=100)
    o, r, f, p, infos = [obs.copy()], [], [], [env.privileged_vector().copy()], []
    ep = 0
    for _ in range(steps):
        a = rng.uniform(-1, 1, cfg.n_joints).astype(np.float32)
        obs, rew, te, tr, inf = env.step(a)
        flat: dict[str, object] = {}
        _flat("i", inf, flat)
        infos.append(flat)
        r.append(rew)
        f.append((te, tr))
        if te or tr:
            ep += 1
            obs, _ = env.reset(seed=100 + ep)
        o.append(obs.copy())
        p.append(env.privileged_vector().copy())
    return {"obs": np.array(o), "rew": np.array(r), "flags": np.array(f), "priv": np.array(p), "info": np.array(json.dumps(infos))}


def generate() -> None:
    """Write the golden file from the current code."""
    GOLDEN.parent.mkdir(exist_ok=True)
    out: dict[str, np.ndarray] = {}
    for c in CASES:
        for k, v in rollout(c).items():
            out[f"{c}.{k}"] = v
    np.savez_compressed(GOLDEN, **out)


@needs_urdf
@pytest.mark.skipif(not GOLDEN.exists(), reason="golden file missing")
@pytest.mark.parametrize("case", CASES)
def test_rollout_bit_identical(case: str) -> None:
    g = np.load(GOLDEN)
    got = rollout(case)
    for k, v in got.items():
        ref = g[f"{case}.{k}"]
        if k == "info":
            assert json.loads(str(v)) == json.loads(str(ref)), "info differs"
        else:
            assert v.shape == ref.shape and np.array_equal(v, ref, equal_nan=True), f"{k} differs"


if __name__ == "__main__":
    if "--gen" in sys.argv:
        generate()
        print("wrote", GOLDEN)
