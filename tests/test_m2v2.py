"""M2 v2: stage definitions, hug_start_mix sampling and switch, per-term logging, takeoff rule, driver bookkeeping, warm-start column map, scripted-coil gate sanity."""
from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

from petctl.rl.config import DEFAULT_URDF_DIR, EnvConfig  # noqa: E402
from petctl.rl.curriculum.gate2 import Gate2Spec, run_gate2  # noqa: E402
from petctl.rl.curriculum.m2v2 import TakeoffCallback, best_seed, next_seed, run_note, takeoff_failed  # noqa: E402
from petctl.rl.curriculum.runner import build_config, validate_overrides  # noqa: E402
from petctl.rl.curriculum.stages import FLAT2_PHASE1, M2_TASKS, M2V2_TASKS, STAGES, get_stage  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.hug_start import (COIL_TABLE, FLAG_ENV, MIX_EARLY, MIX_LATE, ScriptedCoilEvaluator, StartMixCallback, coil_angles, coil_params, half_joints, mix_probs,  # noqa: E402
                                 mix_switched, sample_start_mode)
from petctl.rl.train import column_maps  # noqa: E402

needs_urdf = pytest.mark.skipif(not os.path.isdir(DEFAULT_URDF_DIR), reason="Fusion URDF export not present")


# ---------------------------------------------------------------- stages
def test_stage_definitions() -> None:
    """m2_hug2 / m2_flat2 carry exactly the ordered parameters; the v1 stages are untouched."""
    assert M2_TASKS == ("m2_flat", "m2_terrain", "m2_hug") and M2V2_TASKS == ("m2_hug2", "m2_flat2")
    h = get_stage("m2_hug2")
    tp = h.task_params
    assert (tp["approach_weight"], tp["coverage_weight"], tp["hold_weight"], tp["hug_bonus"], tp["speed_gain"], tp["time_penalty"]) == (200.0, 30.0, 5.0, 300.0, 1.0, 0.2)
    assert (tp["progress_weight"], tp["joint_acc_weight"], tp["action_rate_weight"], tp["tilt_weight"], tp["yaw_weight"]) == (0.0, 0.0, 0.02, 0.0, 0.0)
    assert tp["bus_penalty"] == 1.0 and tp["hug_start_mix"] == 1.0 and tp["capacitive_obs"] == 1.0
    assert (h.task, h.episode_s, h.steps, h.ent_coef) == ("goal_hug", 30.0, 3_000_000, 0.005)
    assert "hug_start_mix" not in h.gate.task_params and "bus_penalty" not in h.gate.task_params         # the gate is the v1 straight-start hug gate
    assert h.gate.task_params["goal_dist_min"] == 0.2 and h.gate.episodes == 100 and h.gate.min_success == 0.0
    f = get_stage("m2_flat2")
    assert (f.task, f.episode_s, f.steps, f.ent_coef, f.dr_scale) == ("goal_reach", 120.0, 2_000_000, 0.005, 0.6)
    assert (f.task_params["bearing_deg"], f.task_params["goal_dist_min"], f.task_params["goal_dist_max"]) == (180.0, 0.8, 2.0)
    assert f.gate == get_stage("m2_flat").gate
    assert FLAT2_PHASE1 == {"bearing_deg": 45.0, "goal_dist_min": 0.6, "goal_dist_max": 1.2, "dr_scale": 0.3}
    validate_overrides(f, {"n_modules": 10.0, **FLAT2_PHASE1})
    c1 = build_config(f, {"n_modules": 10.0, **FLAT2_PHASE1})
    assert c1.dr.scale == 0.3 and c1.task_params["bearing_deg"] == 45.0 and c1.task_params["goal_dist_max"] == 1.2 and c1.n_modules == 10
    assert build_config(f, {}).dr.scale == 0.6
    assert STAGES["m2_hug"].task_params["approach_weight"] if "approach_weight" in STAGES["m2_hug"].task_params else True
    assert "hug_start_mix" not in STAGES["m2_hug"].task_params


def test_hug2_obs_layout_differs_from_flat_and_columns_map() -> None:
    """goal_hug has the touch block in the middle of the vector; the warm start from m2_flat maps blocks by name (cap block moves, touch is zero-padded), actor and critic widths."""
    flat, hug = build_config(get_stage("m2_flat"), {"n_modules": 8.0}), build_config(get_stage("m2_hug2"), {"n_modules": 8.0})
    nf, nh = PetEnv(flat).observation_space.shape[0], PetEnv(hug).observation_space.shape[0]
    assert (nf, nh) == (132, 216)
    maps = column_maps(flat, hug, True, True)
    new_w, idx = maps[nf]
    assert new_w == nh and np.array_equal(idx[:60], np.arange(60)) and np.array_equal(idx[60:], np.arange(144, 216))
    new_w2, idx2 = maps[nf + 73]
    assert new_w2 == nh + 73 and np.array_equal(idx2[:nf], idx) and np.array_equal(idx2[nf:], np.arange(nh, nh + 73))
    assert set(column_maps(flat, hug, False, True)) == {nf}                      # a symmetric source has no critic map


# ---------------------------------------------------------------- start mix
def test_mix_probs_and_sampling() -> None:
    """30/30/40 early, 10/20/70 after the switch; sampling follows them."""
    assert mix_probs(False) == MIX_EARLY == (0.3, 0.3) and mix_probs(True) == MIX_LATE == (0.1, 0.2)
    rng = np.random.default_rng(0)
    for switched, want in ((False, (0.3, 0.3, 0.4)), (True, (0.1, 0.2, 0.7))):
        draws = [sample_start_mode(rng, *mix_probs(switched)) for _ in range(4000)]
        got = tuple(draws.count(m) / len(draws) for m in ("coil", "half", "straight"))
        assert all(abs(g - w) < 0.03 for g, w in zip(got, want)), got


def test_switch_flag_file(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """The mix switches when the flag file named by $PETCTL_HUG_MIX_FLAG exists."""
    flag = tmp_path / "flag"
    monkeypatch.delenv(FLAG_ENV, raising=False)
    assert not mix_switched()
    monkeypatch.setenv(FLAG_ENV, str(flag))
    assert not mix_switched()
    flag.write_text("1")
    assert mix_switched() and mix_switched(str(flag))


def _fake_locals(successes: list[float]) -> dict:
    return {"dones": np.ones(len(successes), bool), "infos": [{"metrics": {"success": s}} for s in successes]}


def test_start_mix_callback_switches_above_half(tmp_path: Path) -> None:
    """The flag is written only once 200 episodes are in the window and their mean success exceeds 0.5; never below."""
    cb = StartMixCallback(tmp_path / "flag")
    cb.num_timesteps = 1234
    cb.locals = _fake_locals([0.0] * 100)
    cb._on_step()
    assert not (tmp_path / "flag").exists()                                      # window not full yet
    cb.locals = _fake_locals([1.0] * 100)
    cb._on_step()
    assert not (tmp_path / "flag").exists()                                      # exactly 0.5 is not above 0.5
    cb.locals = _fake_locals([1.0] * 20)
    cb._on_step()
    assert (tmp_path / "flag").exists() and cb.switched_at == 1234
    assert mix_switched(str(tmp_path / "flag"))


def test_coil_geometry_table() -> None:
    """Per-length coil angle/alpha from M1b; nearest length for the others; half mode curls ceil((n-1)/2) joints; perturbation is +-0.2."""
    assert set(COIL_TABLE) == {8, 10, 12} and coil_params(8) == (0.80, 0.95) and coil_params(9) == (0.80, 0.95) and coil_params(11) == (0.90, 0.95) and coil_params(15) == COIL_TABLE[12]
    assert half_joints(8) == 4 and half_joints(10) == 5
    rng = np.random.default_rng(1)
    q0 = coil_angles(8, "coil", rng, 0.0)
    assert np.allclose(q0, 0.8 * 0.95)
    q = coil_angles(8, "coil", rng, 1.0)
    assert np.all(np.abs(q - q0) <= 0.2) and np.any(np.abs(q - q0) > 0.01)
    h = coil_angles(8, "half", rng)
    assert np.allclose(h[:4], 0.76) and np.allclose(h[4:], 0.0)


@needs_urdf
@pytest.mark.parametrize("mode,forced", [("coil", 1.0), ("half", 2.0)])
def test_coil_and_half_start_in_env(mode: str, forced: float) -> None:
    """Forced coil / half episode: the post is moved to the coil, the goal is the post, the body already touches it, start_mode is logged, and the step loop runs."""
    cfg = build_config(get_stage("m2_hug2"), {"n_modules": 8.0})
    cfg.task_params.update({"hug_force_mode": forced, "hug_perturb_scale": 0.0})
    env = PetEnv(cfg)
    env.reset(seed=3)
    t = env.task
    post = np.array(env.sim.p.post)
    assert t._start_placed and np.allclose(t.goal[:2], post) and np.allclose(env.sim.model.geom_pos[env.sim.model.geom("post").id, :2], post)
    dist = np.linalg.norm(env.sim.data.xipos[1:9, :2] - post, axis=1)
    assert dist.min() < 0.13 and env.privileged["post"]["x"] == pytest.approx(post[0])
    _, r, te, tr, info = env.step(np.zeros(7, np.float32))
    assert np.isfinite(r) and info["metrics"]["start_mode"] == (1.0 if mode == "coil" else 2.0) and info["metrics"]["start_failed"] == 0.0
    if mode == "half":
        assert abs(env.sim.joint_pos()[5:]).max() < 0.35                          # the far half of the joints was left straight


@needs_urdf
def test_straight_start_geometry_and_rng_unchanged_without_knob() -> None:
    """Straight mode: head 5-20 cm (surface) from the post, bearing within +-60 deg. Without hug_start_mix the layout draws exactly what the v1 task drew."""
    cfg = build_config(get_stage("m2_hug2"), {"n_modules": 8.0})
    cfg.task_params["hug_force_mode"] = 0.0
    cfg.task_params["hug_mix_switched"] = 1.0                                   # 70 % straight
    env = PetEnv(cfg)
    seen = 0
    for s in range(12):
        env.reset(seed=s)
        if env.task._start_mode != "straight":
            continue
        seen += 1
        head = env.sim.head_pos()[:2]
        gap = np.linalg.norm(np.array(env.sim.p.post) - head) - 0.05
        assert 0.05 - 1e-6 <= gap <= 0.20 + 1e-6, gap
    assert seen >= 5
    v1 = build_config(get_stage("m2_hug"), {"n_modules": 8.0})
    a, b = PetEnv(v1), PetEnv(v1)
    a.reset(seed=5)
    b.reset(seed=5)
    assert a.task._start_mode == "straight" and np.allclose(a.task.goal, b.task.goal) and 0.2 <= np.linalg.norm((a.task.goal - a.sim.head_pos())[:2]) <= 0.35 + 1e-6


# ---------------------------------------------------------------- per-term logging
@needs_urdf
def test_term_sums_logged_for_goal_tasks_only_at_episode_end() -> None:
    """Goal-task episodes end with term_<name> sums of every reward term in the metrics (equal to the sum of the per-step terms); non-goal tasks are untouched."""
    cfg = build_config(get_stage("m2_hug"), {"n_modules": 8.0})
    cfg.episode_s = 2.0
    env = PetEnv(cfg)
    env.reset(seed=1)
    tot: dict[str, float] = {}
    ret, done, info = 0.0, False, {}
    while not done:
        _, r, te, tr, info = env.step(np.zeros(7, np.float32))
        ret += r
        done = te or tr
        for k, v in info["terms"].items():
            tot[k] = tot.get(k, 0.0) + v
        if not done:
            assert not any(k.startswith("term_") for k in info["metrics"])
    m = info["metrics"]
    assert {"term_progress", "term_energy", "term_time", "term_approach", "term_hug_bonus"} <= set(m)
    for k, v in tot.items():
        assert m[f"term_{k}"] == pytest.approx(v)
    assert sum(m[k] for k in m if k.startswith("term_")) == pytest.approx(ret, rel=1e-6, abs=1e-6)
    loc = PetEnv(EnvConfig(task="locomote", urdf_dir=DEFAULT_URDF_DIR, episode_s=1.0))
    loc.reset(seed=0)
    _, _, te, tr, info = loc.step(np.zeros(7, np.float32))
    assert not any(k.startswith("term_") for k in info.get("metrics", {}))


@needs_urdf
def test_reward_weight_knobs_default_unchanged_and_override() -> None:
    """joint_acc_weight / action_rate_weight default to the config weights (v1 unchanged) and override them."""
    def run(extra: dict[str, float]) -> dict[str, float]:
        cfg = EnvConfig(task="goal_reach", urdf_dir=DEFAULT_URDF_DIR, task_params={"goal_dist_min": 1.0, "goal_dist_max": 1.0, **extra})
        env = PetEnv(cfg)
        env.reset(seed=0)
        env.step(np.ones(7, np.float32))
        return env.step(-np.ones(7, np.float32))[4]["terms"]
    base, off = run({}), run({"joint_acc_weight": 0.0, "action_rate_weight": 0.0})
    assert base["joint_acc"] < 0.0 and base["action_rate"] < 0.0 and off["joint_acc"] == 0.0 and off["action_rate"] == 0.0


# ---------------------------------------------------------------- takeoff rule + driver bookkeeping
def test_takeoff_rule() -> None:
    """None before 0.6M steps; then fail iff the last-200 mean success is below 0.10."""
    assert takeoff_failed([0.0] * 300, 599_999) is None
    assert takeoff_failed([0.0] * 300, 600_000) is True
    assert takeoff_failed([1.0] * 19 + [0.0] * 181, 600_000) is True                 # 19/200 = 0.095
    assert takeoff_failed([1.0] * 20 + [0.0] * 180, 600_000) is False                # 0.10 is not below 0.10
    assert takeoff_failed([1.0] * 300 + [0.0] * 150, 700_000) is False               # only the last 200 count: 50/200
    assert takeoff_failed([0.0] * 800 + [1.0] * 200, 700_000) is False
    assert takeoff_failed([], 700_000) is True


def test_takeoff_callback_stops_once_with_synthetic_episodes() -> None:
    """The callback returns False (stop) at the check if success is low, and keeps going if it is high; the decision is taken once."""
    for succ, stop in (([0.0] * 200, True), ([1.0] * 100 + [0.0] * 100, False)):
        cb = TakeoffCallback(check_steps=1000)
        cb.num_timesteps = 500
        cb.locals = _fake_locals(succ)
        assert cb._on_step() is True and cb.checked_at is None                  # too early
        cb.num_timesteps = 1000
        cb.locals = {"dones": np.zeros(1, bool), "infos": [{}]}
        assert cb._on_step() is (not stop) and cb.failed is stop and cb.checked_at == 1000
        cb.num_timesteps = 5000
        assert cb._on_step() is True                                            # decided once


def _line(L: int, task: str, seed: int, status: str = "ok", success: float | None = 0.5, tg: float | None = 30.0) -> dict:
    return {"L": L, "task": task, "seed": seed, "status": status, "success": success, "t_goal_median_s": tg}


def test_seed_bookkeeping_restarts_with_next_seed() -> None:
    """A takeoff failure does not count towards the quota and the next seed is tried; the quota stops the search; max seed caps it; best seed picks success then speed."""
    lines = [_line(10, "m2_flat2", 0, "takeoff_fail", None)]
    assert next_seed(lines, 10, "m2_flat2", 0, 3) == 1
    lines += [_line(10, "m2_flat2", 1), _line(10, "m2_flat2", 2, success=0.8)]
    assert next_seed(lines, 10, "m2_flat2", 0, 3) == 3
    lines.append(_line(10, "m2_flat2", 3, success=0.8, tg=20.0))
    assert next_seed(lines, 10, "m2_flat2", 0, 3) is None
    assert best_seed(lines, 10, "m2_flat2") == 3
    assert next_seed([_line(8, "m2_flat2", s, "takeoff_fail", None) for s in range(1, 6)], 8, "m2_flat2", 1, 2) is None       # max seed 5 exhausted
    assert best_seed([_line(10, "m2_flat2", 0, "takeoff_fail", None)], 10, "m2_flat2") is None


def test_driver_order(tmp_path: Path) -> None:
    """(a) hug2 L8, (b) flat2 L8 seeds 1-2, (c)/(d) flat2 x3 then hug2 per length; terrain / L=9, 11 never appear; hug2 init is the best flat seed."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("m2v2_driver", Path(__file__).resolve().parents[1] / "scripts" / "rl" / "m2v2_driver.py")
    assert spec and spec.loader
    d = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(d)

    def add(line: dict) -> None:
        with open(tmp_path / "summary.jsonl", "a") as f:
            f.write(json.dumps(line) + "\n")

    seq = []
    while (j := d.next_job(tmp_path)) is not None:
        seq.append((j["task"], j["L"], j["seed"]))
        if j["task"] == "m2_hug2" and j["L"] > 8:
            assert Path(j["init"]).name == "seed2"                              # best flat seed of that length (seed 2 below)
        add(_line(j["L"], j["task"], j["seed"], success=0.9 if j["seed"] == 2 else 0.4))
        assert len(seq) < 30
    assert seq == [("m2_hug2", 8, 0), ("m2_flat2", 8, 1), ("m2_flat2", 8, 2), ("m2_flat2", 10, 0), ("m2_flat2", 10, 1), ("m2_flat2", 10, 2), ("m2_hug2", 10, 0),
                   ("m2_flat2", 12, 0), ("m2_flat2", 12, 1), ("m2_flat2", 12, 2), ("m2_hug2", 12, 0)]


def test_run_note_synthetic(tmp_path: Path) -> None:
    """The note reports a return curve, term means, start-mode success and survives missing tensorboard logs."""
    rows = [{"timesteps": 1000 * (i + 1), "return": -100.0 + i, "success": float(i % 2), "term_approach": 5.0, "start_mode": float(i % 3)} for i in range(60)]
    (tmp_path / "episodes.jsonl").write_text("\n".join(json.dumps(r) for r in rows))
    note = run_note(tmp_path)
    assert "60 training episodes" in note and "return/success per tenth" in note and "approach=5.0" in note and "success by start mode" in note and "no tensorboard" in note


# ---------------------------------------------------------------- scripted-coil gate sanity (step 0c)
@needs_urdf
def test_scripted_coil_scores_hug_success_through_gate2() -> None:
    """The M1b constant-angle coil, run as a policy-like scripted rollout through run_gate2 (same metric, same aggregation, dr 0.6), is a hug success."""
    st = get_stage("m2_hug2")
    params = {**st.gate.task_params, "bus_model": 1.0, "hug_perturb_scale": 0.0}
    res = run_gate2(Path("."), st, Gate2Spec(n=4, dr_scale=0.6, nominal=False, threshold=0.0), evaluator=ScriptedCoilEvaluator(8, params, 0.6, 30.0), write=False)
    assert res["robust"]["rate"] >= 0.75, res["robust"]
    mm = res["robust"]["metric_median"]
    assert mm["hug_modules"] >= 5 and mm["t_hug_s"] == pytest.approx(3.0, abs=0.5) and mm["hold_duty"] >= 0.9
