"""Per-family hug success curriculum (stage hug_curr_02): family level state machine, level-keyed hold / bonus per family, shape mix / range / exclusion margin, logging keys, driver check rules."""
from __future__ import annotations

import json
import os

import numpy as np
import pytest

pytest.importorskip("mujoco")

from petctl.rl.capacitive import PostRead  # noqa: E402
from petctl.rl.config import DEFAULT_URDF_DIR  # noqa: E402
from petctl.rl.curriculum import hug_curr2 as hc2  # noqa: E402
from petctl.rl.curriculum.runner import build_config  # noqa: E402
from petctl.rl.curriculum.stages import get_stage  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.hug_levels import FAMILIES, LADDERS, LEVEL_FILE, LEVEL_SPECS, FamilyLevelState, level_caption, read_family_levels  # noqa: E402
from petctl.rl.records import HugFamilyLevelCallback  # noqa: E402
from petctl.rl.target_shapes import V2_SAMPLER, held_out_set, near_heldout, sample_training_shape  # noqa: E402

needs = pytest.mark.skipif(not os.path.isdir(DEFAULT_URDF_DIR), reason="Fusion URDF export not present")


def test_ladders_and_specs() -> None:
    assert LADDERS["box"] == ("l1", "l2", "l3") and LADDERS["cyl"] == LADDERS["rbox"] == ("c0", "c1", "l2", "l3")
    assert LEVEL_SPECS["c0"] == (100.0, 4, 1.0) and LEVEL_SPECS["c1"] == (125.0, 4, 1.5) and LEVEL_SPECS["l1"] == (120.0, 3, 1.5)
    assert LEVEL_SPECS["l2"] == (150.0, 4, 2.0) and LEVEL_SPECS["l3"] == (180.0, 4, 3.0)


def test_family_levels_advance_independently_and_never_demote(tmp_path) -> None:
    st = FamilyLevelState()
    for i in range(99):
        assert not st.add("cyl", 0, 1.0)
    assert st.add("cyl", 0, 1.0) and st.levels == {"cyl": 1, "box": 0, "rbox": 0} and st.history[0]["family"] == "cyl" and st.history[0]["level"] == "c1"
    assert not st.add("cyl", 0, 1.0)                                          # episodes that started at the old level do not count
    assert not st.add("box", 1, 1.0) and st.rolling("box") is None             # box is at its own level 0
    for i in range(100):
        st.add("rbox", 0, 1.0 if i % 10 < 3 else 0.0)                          # exactly 0.30 advances
    assert st.levels["rbox"] == 1
    for _ in range(500):
        st.add("rbox", 1, 0.0)
    assert st.levels["rbox"] == 1                                              # no demotion
    for lvl in range(1, 5):
        for _ in range(100):
            st.add("cyl", lvl, 1.0)
    assert st.levels["cyl"] == 3                                               # capped at L3
    st.save(tmp_path / LEVEL_FILE)
    assert read_family_levels(tmp_path) == st.levels and level_caption(tmp_path) == "levels cyl L3 / box L1 / rbox C1" and read_family_levels(tmp_path / "x") == {"cyl": 0, "box": 0, "rbox": 0}


def test_shape_mix_range_and_margin() -> None:
    rng = np.random.default_rng(0)
    shapes = [sample_training_shape(rng, 1.5, 8, **V2_SAMPLER) for _ in range(6000)]
    kinds = [s.kind for s in shapes]
    assert abs(kinds.count("cyl") / 6000 - 0.5) < 0.03 and abs(kinds.count("box") / 6000 - 0.2) < 0.03 and abs(kinds.count("rbox") / 6000 - 0.3) < 0.03
    radii = np.array([s.a for s in shapes if s.kind == "cyl"])
    assert radii.min() >= 0.015 and radii.max() <= 0.09 and (radii < 0.04).mean() > 0.2                       # thin cylinders (r < 4 cm) are trained now
    assert not any(near_heldout(s, 8, 0.005) for s in shapes)
    assert ((radii > 0.025) & (radii < 0.035)).sum() == 0 and ((radii > 0.065) & (radii < 0.075)).sum() == 0     # +-0.5 cm around gate r3 / r7 excluded
    assert ((radii > 0.0355) & (radii < 0.04)).sum() > 0 and ((radii > 0.0205) & (radii < 0.0245)).sum() > 0   # ... and nothing more (the old margin was 1 cm)
    r_old = np.random.default_rng(1)
    old = [sample_training_shape(r_old, 1.5, 8) for _ in range(2000)]
    assert min(s.a for s in old if s.kind == "cyl") >= 0.04 - 1e-9                                       # old behaviour (1 cm margin): r3 was outside the trained range
    assert [sample_training_shape(np.random.default_rng(1), 1.5, 8) for _ in range(2)] == [sample_training_shape(np.random.default_rng(1), 1.5, 8) for _ in range(2)]
    assert {n for n, *_ in held_out_set(8)} >= {"cyl_r3", "cyl_r7"}


def _stub_read(span_deg: float, run: int = 4, n: int = 8) -> PostRead:
    touch = np.zeros((n, 3), bool)
    touch[:run, 0] = True
    ang = np.full(n, np.nan)
    ang[:run] = np.deg2rad(np.linspace(0.0, span_deg, run))
    return PostRead(touch, touch * 0.5, touch.any(axis=1), 0.0, None, ang)


def _seed_of_kind(stage_name: str, kind: str) -> int:
    env = PetEnv(build_config(get_stage(stage_name), {"n_modules": 8}))
    for s in range(1, 60):
        env.reset(seed=s)
        if env.target_shape.kind == kind and env._start_buf is None:
            return s
    raise AssertionError(kind)


@needs
def test_level_keyed_hold_bonus_per_family_and_logging_keys() -> None:
    stage = get_stage("hug_curr_02")
    assert (stage.task_params["hug_curriculum"], stage.task_params["shape_mix"], "hug_curriculum" in stage.gate.task_params, "shape_mix" in stage.gate.task_params) == (2.0, 1.0, False, False)
    got = {}
    for fam in FAMILIES:
        seed = _seed_of_kind("hug_curr_02", fam)
        for idx in range(len(LADDERS[fam])):
            env = PetEnv(build_config(stage, {"n_modules": 8}))
            env.set_hug_family_levels({fam: idx})
            env.reset(seed=seed)
            t = env.task
            name = LADDERS[fam][idx]
            assert t.target_shape.kind == fam and sorted(t._hug_levels) == sorted(LADDERS[fam]) and t._hug_level == idx
            span, run, hold = LEVEL_SPECS[name]
            assert (t._hug.span_min, t._hug.h, t._hug.hold_need) == (span, run, int(round(hold * 20))) and t._cov_h == 4
            t._hug_probe.read = lambda sp=span + 5.0: _stub_read(sp)       # type: ignore[method-assign]
            bonus_t = None
            for k in range(70):
                _, terms = t.hug_step(env.sim)
                if terms["hug_bonus"] > 0 and bonus_t is None:
                    bonus_t = k + 1
            got[(fam, name)] = bonus_t
            m = t.metrics(env.sim)
            assert m["hug_level"] == idx and m[f"success_{name}"] == 1.0 and m["success"] == 1.0
            assert {"shape_kind", "shape_a", "shape_b", "shape_rc", "contact", "bend_run_deg"} <= set(m) and m["shape_kind"] == FAMILIES.index(fam)
    for (fam, name), t_hold in got.items():                                            # the bonus comes when the hold window (hold s x 20 Hz) is full
        assert t_hold == int(round(LEVEL_SPECS[name][2] * 20)), (fam, name, t_hold)


@needs
def test_unset_levels_score_the_gate_definition_and_curriculum_off_has_no_new_keys() -> None:
    env = PetEnv(build_config(get_stage("hug_curr_02"), {"n_modules": 8}))
    env.reset(seed=3)                                                       # no levels broadcast: every family at its top level = the gate definition
    assert env.task._hug.span_min == 180.0 and env.task._hug.hold_need == 60
    env2 = PetEnv(build_config(get_stage("hug_long_01"), {"n_modules": 8}))
    env2.reset(seed=3)
    m = env2.task.metrics(env2.sim)
    assert env2.task._hug_levels == {} and not ({"shape_kind", "shape_a", "contact", "bend_run_deg", "hug_level"} & set(m)) and "bend_run_deg" not in env2.task.hug_metrics()


@needs
def test_family_levels_shared_across_workers_by_callback(tmp_path) -> None:
    from stable_baselines3.common.vec_env import SubprocVecEnv

    from petctl.rl.train import make_env_fn

    cfg = build_config(get_stage("hug_curr_02"), {"n_modules": 8})
    venv = SubprocVecEnv([make_env_fn(cfg, i, 0) for i in range(2)])
    try:
        cb = HugFamilyLevelCallback(tmp_path)
        cb.model = type("M", (), {"get_env": lambda self: venv})()                                    # type: ignore[assignment]
        cb._on_training_start()
        assert venv.env_method("get_hug_family_levels") == [{"cyl": 0, "box": 0, "rbox": 0}] * 2
        for i in range(100):
            cb.locals = {"dones": [True, False], "infos": [{"metrics": {"hug_level": 0.0, "shape_kind": 1.0, "success": 1.0}}, {}]}
            cb.num_timesteps = 10 * i
            cb._on_step()
        assert cb.state.levels["box"] == 1 and read_family_levels(tmp_path)["box"] == 1
        assert venv.env_method("get_hug_family_levels")[0]["box"] == 0                                  # broadcast only at the rollout end
        cb._on_rollout_end()
        assert venv.env_method("get_hug_family_levels") == [{"cyl": 0, "box": 1, "rbox": 0}] * 2
        assert HugFamilyLevelCallback(tmp_path).state.levels["box"] == 1                                 # resumable
    finally:
        venv.close()


def _row(kind: int, succ: float, span: float = 90.0, s1: float = 0.0, bend: float = 60.0) -> dict:
    return {"shape_kind": kind, "success": succ, "success_l1": s1, "hug_modules": 4.0, "normal_span_deg": span, "contact": 1.0, "bend_run_deg": bend}


def test_wrap90_alias_and_contact_rate() -> None:
    from petctl.rl.curriculum import hug_curr as hc, m2v7

    bd = hc.per_shape_breakdown([_row(0, 0.0, span=95.0)] * 4 + [_row(1, 0.0, span=50.0)] * 4)
    assert bd["cylinders"]["wrap90_rate"] == bd["cylinders"]["reach_rate"] == 1.0 and bd["boxes"]["wrap90_rate"] == 0.0 and bd["boxes"]["contact_rate"] == 1.0
    assert m2v7.wrap90 is m2v7.reached_state


def test_curved_trend_rule() -> None:
    flat = [_row(i % 3 // 2 * 2, 0.0, span=95.0) for i in range(600)]                    # kinds 0 and 2 (curved), no success, span p75 95
    stop, sig = hc2.curved_stop(flat)
    assert stop and sig["curved_last150"] == 0.0 and sig["cyl_span_p75"] == 95.0
    rows = flat[:300] + [_row(0, 1.0 if i >= 290 else 0.0, span=95.0) for i in range(300)]       # success 0.067 >= 0.03 -> keep going
    assert not hc2.curved_stop(rows)[0]
    rows = [_row(0, 1.0 if i < 3 else 0.0, 95.0) for i in range(150)] + [_row(0, 1.0 if i < 2 else 0.0, 95.0) for i in range(150)]   # 0.013 < 0.03 but rose? 0.02 -> 0.013: not risen -> stop
    assert hc2.curved_stop(rows)[0]
    rows = [_row(0, 1.0 if i < 1 else 0.0, 95.0) for i in range(150)] + [_row(0, 1.0 if i < 3 else 0.0, 95.0) for i in range(150)]   # 0.007 -> 0.02: risen -> no stop
    assert not hc2.curved_stop(rows)[0]
    rows = [_row(0, 0.0, 95.0) for i in range(150)] + [_row(0, 0.0, 112.0 if i % 2 else 95.0) for i in range(150)]                # span p75 111+ -> progress -> no stop
    assert not hc2.curved_stop(rows)[0]
    rows = [_row(0, 0.0, 95.0) for i in range(150)] + [_row(0, 0.0, 110.0 if i % 2 else 95.0) for i in range(150)]                # p75 110 < 111 -> stop
    assert hc2.curved_stop(rows)[0]
    assert not hc2.curved_stop([_row(0, 0.0)] * 100)[0]                                    # fewer than 150 curved episodes: no stop


def test_box_safety_rule_and_end_decision() -> None:
    base = [_row(1, 0.0, s1=1.0 if i % 2 == 0 else 0.0) for i in range(100)]              # baseline 0.5
    assert not hc2.box_safety_stop(base)[0]                                               # fewer than 200 box episodes
    ok = base + [_row(1, 0.0, s1=1.0 if i % 10 < 4 else 0.0) for i in range(100)]          # 0.4: drop 0.1
    bad = base + [_row(1, 0.0, s1=1.0 if i % 10 < 2 else 0.0) for i in range(100)]         # 0.2: drop 0.3 > 0.2
    assert not hc2.box_safety_stop(ok)[0] and hc2.box_safety_stop(bad)[0] and hc2.box_safety_stop(bad)[1]["box_baseline"] == 0.5
    rows = [_row(0, 0.0)] * 600
    assert hc2.end_decision(0.6, False, rows) == "final" and hc2.end_decision(0.3, True, rows) == "continue" and hc2.end_decision(0.3, False, rows) == "stop"
    rising = [_row(0, 0.0)] * 300 + [_row(0, 1.0 if i < 10 else 0.0) for i in range(150)]
    assert hc2.end_decision(0.1, False, rising) == "continue"
    t = hc2.family_table([_row(0, 1.0, span=100.0, bend=50.0)] * 5 + [_row(1, 0.0)] * 3, {"cyl": 1, "box": 0, "rbox": 0})
    assert t["cyl"]["level"] == "c1" and t["cyl"]["success"] == 1.0 and t["cyl"]["span_p75"] == 100.0 and t["cyl"]["bend_p50"] == 50.0 and t["rbox"]["n"] == 0 and t["box"]["level"] == "l1"


def test_callback_records_table_and_stops(tmp_path) -> None:
    (tmp_path / "episodes.jsonl").write_text("\n".join(json.dumps(_row(0 if i % 2 else 2, 0.0)) for i in range(400)) + '\n{"shape')
    FamilyLevelState().save(tmp_path / LEVEL_FILE)
    cb = hc2.Curr2Checks(tmp_path, 1.0)
    assert not cb.run_check1() and cb.results["check1"]["table"]["cyl"]["n"] == 100 and (tmp_path / "check1.json").exists() and (tmp_path / "checks.jsonl").exists()
    assert cb.run_safety() and cb.results["safety"]["n_box"] == 0


@needs
def test_hold_ramp_value_cap_default_off_and_level_keyed() -> None:
    stage = get_stage("hug_curr_02")
    assert stage.task_params["hold_ramp"] == 0.0

    def holds(ramp: float | None, fam: str, idx: int, ticks: int) -> list[float]:
        cfg = build_config(stage, {"n_modules": 8, **({} if ramp is None else {"hold_ramp": ramp})})
        env = PetEnv(cfg)
        env.set_hug_family_levels({fam: idx})
        env.reset(seed=_seed_of_kind("hug_curr_02", fam))
        t = env.task
        t._hug_probe.read = lambda: _stub_read(190.0)           # type: ignore[method-assign]
        return [t.hug_step(env.sim)[1]["hold"] for _ in range(ticks)], t._hug.hold_need, t.param("hold_weight", 1.0)

    for fam, idx in (("cyl", 0), ("box", 1)):
        off, need, w = holds(0.0, fam, idx, 3 * 30)
        on, need2, _ = holds(1.0, fam, idx, 3 * 30)
        dflt, _, _ = holds(None, fam, idx, 3 * 30)
        assert need == need2 and off == dflt                                    # off by default, bit-identical
        i0 = next(k for k, v in enumerate(off) if v > 0)                        # first paying hug tick (the tracker debounces the start)
        assert abs(on[i0] - w * (1 + 1 / need)) < 1e-9                        # first hug tick: hold_ticks = 1 (hold_ticks 0 would pay hold_weight)
        assert on[i0 + 1] > on[i0] and abs(on[i0 + 5] - w * (1 + 6 / need)) < 1e-9         # the ramp grows by hold_weight / hold_need per tick
        assert abs(on[i0 + need - 1] - w * 2.0) < 1e-9                          # at hold_need hold ticks: 2x
        assert max(on) <= w * 2.0 + 1e-12 and abs(on[-1] - w * 2.0) < 1e-9 and max(off) == w   # capped at 2x; off = the flat v3 hold
    assert holds(1.0, "cyl", 0, 1)[1] != holds(1.0, "cyl", 2, 1)[1]            # hold_need follows the family's current level
