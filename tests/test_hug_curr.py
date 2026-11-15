"""Hug success curriculum (stage hug_curr_01): level state machine, level-keyed hold / bonus, shared level across workers, per-shape breakdown, driver checks."""
from __future__ import annotations

import json
import os

import numpy as np
import pytest

pytest.importorskip("mujoco")

from petctl.rl.capacitive import PostRead  # noqa: E402
from petctl.rl.config import DEFAULT_URDF_DIR  # noqa: E402
from petctl.rl.curriculum import hug_curr as hc  # noqa: E402
from petctl.rl.curriculum.runner import build_config  # noqa: E402
from petctl.rl.curriculum.stages import get_stage  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.hug import HUG_LEVELS  # noqa: E402
from petctl.rl.hug_levels import LEVEL_FILE, LevelState, read_level  # noqa: E402
from petctl.rl.records import HugLevelCallback  # noqa: E402

needs = pytest.mark.skipif(not os.path.isdir(DEFAULT_URDF_DIR), reason="URDF missing")


def test_level_state_advances_at_030_and_never_regresses(tmp_path) -> None:
    st = LevelState(1)
    for i in range(99):
        assert not st.add(1, 1.0)                                   # window not full
    assert st.add(1, 1.0) and st.level == 2 and st.history[0]["level"] == 2
    assert not st.add(1, 1.0) and st.level == 2                      # episodes that started at the old level do not count
    st2 = LevelState(1)
    for i in range(100):
        assert not st2.add(1, 1.0 if i < 29 else 0.0)               # 0.29 < 0.30
    for _ in range(100):                                              # the window rolls: 0.30 exactly advances
        st2.add(1, 0.0)
    assert st2.level == 1
    st3 = LevelState(1)
    for i in range(100):
        st3.add(1, 1.0 if i % 10 < 3 else 0.0)
    assert st3.level == 2
    for _ in range(300):
        st3.add(2, 0.0)                                              # poor success at L2: no regression
    assert st3.level == 2
    for _ in range(100):
        st3.add(2, 1.0)
    assert st3.level == 3
    for _ in range(500):
        st3.add(3, 1.0)
    assert st3.level == 3                                            # capped
    st3.save(tmp_path / LEVEL_FILE)
    assert read_level(tmp_path) == 3 and LevelState.load(tmp_path / LEVEL_FILE).history == st3.history and read_level(tmp_path / "none") == 1


def _stub_read(span_deg: float, run: int = 4, n: int = 8) -> PostRead:
    touch = np.zeros((n, 3), bool)
    touch[:run, 0] = True
    ang = np.full(n, np.nan)
    ang[:run] = np.deg2rad(np.linspace(0.0, span_deg, run))
    return PostRead(touch, touch * 0.5, touch.any(axis=1), 0.0, None, ang)


@needs
def test_level_keyed_hold_bonus_and_gate_scores_l3() -> None:
    stage = get_stage("hug_curr_01")
    assert (stage.task_params["hug_curriculum"], "hug_curriculum" in stage.gate.task_params) == (1.0, False)
    out = {}
    for level, span in ((1, 130.0), (2, 130.0), (2, 160.0), (3, 160.0), (3, 190.0)):
        env = PetEnv(build_config(stage, {"n_modules": 8}))
        env.set_hug_level(level)
        env.reset(seed=3)
        task = env.task
        assert task._hug_level == level and sorted(task._hug_levels) == [1, 2, 3]
        assert (task._hug.span_min, task._hug.h, task._hug.hold_need) == (HUG_LEVELS[level][0], HUG_LEVELS[level][1], int(round(HUG_LEVELS[level][2] * 20)))
        assert task._hug_levels[3].span_min == 180.0 and task._hug_levels[3].h == 4 and task._hug_levels[3].hold_need == 60 and task._cov_h == 4
        task._hug_probe.read = lambda sp=span: _stub_read(sp)      # type: ignore[method-assign]
        hold = bonus = 0.0
        first = None
        for t in range(70):
            _, terms = task.hug_step(env.sim)
            hold += terms["hold"]
            bonus += terms["hug_bonus"]
            if terms["hug_bonus"] > 0 and first is None:
                first = t + 1
        out[(level, span)] = (hold, bonus, first, task.hug_success)
        m = task.hug_metrics()
        assert m["hug_level"] == level and m["success_l1"] == float(span >= 120) and m["success_l3"] == float(span >= 180 and t + 1 >= 60)
    assert out[(1, 130.0)][3] and out[(1, 130.0)][2] is not None and out[(1, 130.0)][2] < 40 and out[(1, 130.0)][1] > 0       # L1: held after 1.5 s of run-4 span-130 ticks
    assert not out[(2, 130.0)][3] and out[(2, 130.0)][0] == 0.0                                                          # span 130 is no wrap tick at L2 (needs 150)
    assert out[(2, 160.0)][3] and out[(2, 160.0)][2] is not None and 40 <= out[(2, 160.0)][2] < 60                      # L2 holds after 2 s
    assert not out[(3, 160.0)][3] and out[(3, 160.0)][0] == 0.0 and out[(3, 190.0)][3] and out[(3, 190.0)][2] >= 60     # L3 = gate definition (3 s)


@needs
def test_curriculum_off_is_the_gate_definition() -> None:
    stage = get_stage("hug_long_01")
    env = PetEnv(build_config(stage, {"n_modules": 8}))
    env.set_hug_level(1)                                                                     # ignored without the task knob
    env.reset(seed=3)
    t = env.task
    assert t._hug_levels == {} and (t._hug.span_min, t._hug.h, t._hug.hold_need) == (180.0, 4, 60) and "hug_level" not in t.hug_metrics() and "shape_kind" not in t.metrics(env.sim)
    gate = build_config(stage, {"n_modules": 8})
    gate.task_params.update(get_stage("hug_curr_01").gate.task_params)
    env2 = PetEnv(gate)
    env2.set_hug_level(1)
    env2.reset(seed=10000)
    assert env2.task._hug_levels == {} and env2.task._hug.span_min == 180.0


@needs
def test_level_shared_across_workers_by_callback(tmp_path) -> None:
    from stable_baselines3.common.vec_env import SubprocVecEnv

    from petctl.rl.train import make_env_fn

    cfg = build_config(get_stage("hug_curr_01"), {"n_modules": 8})
    venv = SubprocVecEnv([make_env_fn(cfg, i, 0) for i in range(2)])
    try:
        cb = HugLevelCallback(tmp_path, 1)
        cb.model = type("M", (), {"get_env": lambda self: venv})()                                    # type: ignore[assignment]
        cb._on_training_start()
        assert venv.env_method("get_hug_level") == [1, 1]
        for i in range(100):
            cb.locals = {"dones": [True, False], "infos": [{"metrics": {"hug_level": 1.0, "success": 1.0}}, {}]}
            cb.num_timesteps = 10 * i
            cb._on_step()
        assert cb.state.level == 2 and read_level(tmp_path) == 2
        assert venv.env_method("get_hug_level") == [1, 1]                    # broadcast only at the rollout end
        cb._on_rollout_end()
        assert venv.env_method("get_hug_level") == [2, 2]
        resumed = HugLevelCallback(tmp_path, 1)
        assert resumed.state.level == 2 and resumed.state.history[0]["level"] == 2             # resumable from the run dir
    finally:
        venv.close()


def _row(kind: int, succ: float, reach: bool = True, s1: float = 0.0, h1: float = 0.8, n1: float = 40.0) -> dict:
    return {"shape_kind": kind, "success": succ, "success_l1": s1, "hug_modules": 4.0 if reach else 1.0, "normal_span_deg": 130.0 if reach else 5.0, "h_wrap1_mean": h1, "h_wrap1_n": n1}


def test_per_shape_breakdown_and_checks(tmp_path) -> None:
    rows = [_row(0, 1.0)] * 10 + [_row(1, 0.0)] * 10 + [_row(2, 0.0, reach=False)] * 5
    bd = hc.per_shape_breakdown(rows)
    assert bd["cylinders"] == {"n": 10.0, "wrap90_rate": 1.0, "reach_rate": 1.0, "contact_rate": 0.0, "level_success": 1.0} and bd["boxes"]["level_success"] == 0.0     # reach_rate = old-name alias
    assert bd["rounded"] == {"n": 5.0, "wrap90_rate": 0.0, "reach_rate": 0.0, "contact_rate": 0.0, "level_success": 0.0}
    good = [_row(i % 3, 0.2, s1=1.0 if i % 5 == 0 else 0.0) for i in range(300)]                  # success_l1 0.20
    ok, sig = hc.check1(good, 1)
    assert ok and sig["success_l1"] == pytest.approx(0.2) and sig["h_wrap"] == pytest.approx(0.8) and set(sig["per_shape"]) == {"cylinders", "boxes", "rounded"}
    assert not hc.check1([_row(0, 0, s1=1.0 if i % 10 == 0 else 0.0) for i in range(300)], 1)[0]  # success 0.10 < 0.15 at level 1 ...
    assert hc.check1([_row(0, 0, h1=0.8)] * 300, 2)[0]                                              # ... but level 2 reached passes the success part
    assert not hc.check1([_row(0, 0, s1=1.0, h1=0.69)] * 300, 1)[0] and not hc.check1([], 1)[0]   # h_wrap below 0.70
    assert hc.check2(good, 2)[0] and not hc.check2(good, 1)[0]


def test_driver_checks_callback_with_synthetic_episodes(tmp_path) -> None:
    (tmp_path / "episodes.jsonl").write_text("\n".join(json.dumps(_row(i % 3, 0.0, s1=1.0 if i % 4 == 0 else 0.0)) for i in range(300)) + '\n{"hug_mo')
    LevelState(1).save(tmp_path / LEVEL_FILE)
    cb = hc.CurrChecks(tmp_path, 1.0, 2.0)
    assert cb.run_check1(steps=11) and cb.results["check1"]["timesteps"] == 11 and (tmp_path / "check1.json").exists() and "per_shape" in cb.results["check1"]
    assert not cb.run_check2(at_end=True, steps=22) and cb.results["check2"]["at_end"] and cb.results["check2"]["level"] == 1
    LevelState(2).save(tmp_path / LEVEL_FILE)
    assert cb.run_check2() and cb.results["check2"]["ok"]


@needs
def test_training_episode_metrics_carry_level_and_shape() -> None:
    env = PetEnv(build_config(get_stage("hug_curr_01"), {"n_modules": 8}))
    env.set_hug_level(2)
    env.reset(seed=5)
    _, _, _, _, info = env.step(np.zeros(env.action_space.shape))
    m = info["metrics"]
    assert m["hug_level"] == 2.0 and m["shape_kind"] in (0.0, 1.0, 2.0) and {"success_l1", "success_l2", "success_l3", "h_wrap1_n"} <= set(m)
