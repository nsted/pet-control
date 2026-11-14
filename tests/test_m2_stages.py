"""M2 length-study stages: they build, accept n_modules, and the env's observation size matches the policy's expectations."""

from __future__ import annotations

import numpy as np
import pytest

from petctl.rl.curriculum.runner import build_config, validate_overrides
from petctl.rl.curriculum.stages import M2_TASKS, STAGES, get_stage
from petctl.rl.env import PetEnv


@pytest.mark.parametrize("task", M2_TASKS)
@pytest.mark.parametrize("n", [8, 10])
def test_stage_instantiates(task: str, n: int) -> None:
    """Each stage builds an env at length n, resets, steps, and the obs/action sizes follow n."""
    st = get_stage(task)
    cfg = build_config(st, {"n_modules": float(n)})
    assert cfg.n_modules == n and "n_modules" not in cfg.task_params
    env = PetEnv(cfg)
    obs, _ = env.reset(seed=3)
    assert obs.shape == env.observation_space.shape == (env.obs_builder.size,)
    assert env.action_space.shape == (n - 1,)
    assert env.privileged_vector().shape == (env.privileged_dim(),)
    obs, r, *_ = env.step(np.zeros(n - 1, np.float32))
    assert obs.shape == env.observation_space.shape and np.isfinite(r)
    # 5 head-frame goal entries (delta channel) and the 9 n capacitive entries are part of the observation
    base = build_config(st, {"n_modules": float(n)})
    base.task_params["capacitive_obs"] = 0.0
    assert PetEnv(base).obs_builder.size + 9 * n == env.obs_builder.size


def test_gate_is_study_gate() -> None:
    """n=100, dr 0.6, no pass threshold; training-only terms are not in the gate distribution."""
    for t in M2_TASKS:
        g = get_stage(t).gate
        assert g.episodes == 100 and g.dr_scale == 0.6 and g.min_success == 0.0
        assert "bus_penalty" not in g.task_params and "terrain_level_min" not in g.task_params
    assert get_stage("m2_terrain").gate.task_params["terrain_level"] == 1.0
    assert get_stage("m2_hug").task == "goal_hug" and get_stage("m2_hug").task_params["squeeze_weight"] == 0.0


def test_knobs_and_old_stages_untouched() -> None:
    """n_modules / imu_off are valid overrides; out-of-range n is rejected; the legacy stages are still there."""
    st = get_stage("m2_terrain")
    validate_overrides(st, {"n_modules": 12.0, "imu_off": 1.0})
    with pytest.raises(ValueError):
        validate_overrides(st, {"n_modules": 20.0})
    assert "s8c_flat_v2" in STAGES and "n_modules" not in STAGES["s8c_flat_v2"].knobs


def test_terrain_level_sampled_per_episode() -> None:
    """Training samples a terrain level in [min, level]; the gate (no min) fixes it."""
    env = PetEnv(build_config(get_stage("m2_terrain"), {}))
    levels = set()
    for s in range(3):
        env.reset(seed=s)
        levels.add(round(env.privileged["terrain_level"], 3))
    assert len(levels) == 3 and all(0.3 <= v <= 1.0 for v in levels)
    gate_cfg = build_config(get_stage("m2_terrain"), {})
    gate_cfg.task_params = dict(get_stage("m2_terrain").gate.task_params)
    g = PetEnv(gate_cfg)
    g.reset(seed=1)
    assert g.privileged["terrain_level"] == 1.0
