"""Action-range study: the joint-output rescale keeps the commanded targets, the saturation metric, stage definitions, decision rules on synthetic data and the driver wiring."""
from __future__ import annotations

import dataclasses
import importlib.util
import inspect
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("mujoco")
torch = pytest.importorskip("torch")

from petctl.rl.config import EnvConfig  # noqa: E402
from petctl.rl.curriculum import hug_range as hr  # noqa: E402
from petctl.rl.curriculum.runner import build_config  # noqa: E402
from petctl.rl.curriculum.stages import get_stage  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.policy import GaitPolicy  # noqa: E402
from petctl.rl.train import rescale_joint_outputs  # noqa: E402

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts" / "rl"


def _load(name: str):
    sys.path.insert(0, str(SCRIPTS))
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def _cfg(stage: str, seconds: float = 4.0) -> EnvConfig:
    cfg = build_config(get_stage(stage), {"n_modules": 8.0})
    cfg.episode_s = seconds
    cfg.dr.scale = 0.3
    cfg.task_params = {**cfg.task_params, "range_diag": 1.0}
    return cfg


class _Actor(torch.nn.Module):
    """Stand-in for an SB3 actor: tanh MLP trunk + linear ``action_net`` (n_joints + 1 outputs), ``.policy.action_net`` as in SB3."""

    def __init__(self, n_in: int, n_out: int) -> None:
        super().__init__()
        torch.manual_seed(3)
        self.trunk = torch.nn.Sequential(torch.nn.Linear(n_in, 32), torch.nn.Tanh())
        self.action_net = torch.nn.Linear(32, n_out)
        with torch.no_grad():
            self.action_net.weight.mul_(0.5)
            self.action_net.bias.uniform_(-0.3, 0.3)
        self.policy = SimpleNamespace(action_net=self.action_net)

    def act(self, obs: np.ndarray) -> np.ndarray:
        with torch.no_grad():
            return self.action_net(self.trunk(torch.as_tensor(obs, dtype=torch.float32))).numpy().astype(np.float64)


def _rollout(cfg: EnvConfig, actor: _Actor, steps: int, seed: int) -> tuple[np.ndarray, np.ndarray, float]:
    env = PetEnv(cfg)
    obs, _ = env.reset(seed=seed)
    tgts, acts, max_a = [], [], 0.0
    for _ in range(steps):
        a = actor.act(obs)
        max_a = max(max_a, float(np.abs(a[:cfg.n_joints]).max()))
        obs, _, te, tr, _ = env.step(a)
        tgts.append(env._tgt.copy())
        acts.append(a)
        if te or tr:
            break
    return np.array(tgts), np.array(acts), max_a


def test_rescale_keeps_the_commanded_targets_for_actions_within_one() -> None:
    old, new = _cfg("hug_curr_02"), _cfg("hug_scale15")
    assert (old.action_scale_rad, old.action_obs_scale, new.action_scale_rad, new.action_obs_scale) == (1.0, 1.0, 1.5, 1.5)
    n_obs = PetEnv(old).observation_space.shape[0]
    a_old, a_new = _Actor(n_obs, old.n_actions), _Actor(n_obs, new.n_actions)
    a_new.load_state_dict(a_old.state_dict())
    hold_w, hold_b = a_new.action_net.weight[7].clone(), a_new.action_net.bias[7].clone()
    rescale_joint_outputs(a_new, old.n_joints, 1.0 / 1.5)
    assert torch.equal(a_new.action_net.weight[7], hold_w) and torch.equal(a_new.action_net.bias[7], hold_b)       # hold channel untouched
    assert torch.allclose(a_new.action_net.weight[:7], a_old.action_net.weight[:7] / 1.5)
    t_old, acts_old, max_a = _rollout(old, a_old, 80, seed=11)
    t_new, acts_new, _ = _rollout(new, a_new, 80, seed=11)
    assert max_a < 1.0 and len(t_old) == len(t_new) == 80                                                          # the test actions stay within +-1
    assert np.abs(t_old - t_new).max() < 1e-4
    assert np.abs(acts_old[:, :7] / 1.5 - acts_new[:, :7]).max() < 1e-4 and np.abs(acts_old[:, 7] - acts_new[:, 7]).max() < 1e-4
    assert np.abs(t_old).max() > 0.05                                                                              # the targets actually moved


def test_wider_range_is_reachable() -> None:
    cfg = _cfg("hug_scale15")
    env = PetEnv(cfg)
    env.reset(seed=1)
    for _ in range(40):
        env.step(np.concatenate([np.ones(7), [-1.0]]))
    assert np.abs(env._tgt).max() > 1.2 and np.abs(env._tgt).max() <= 1.5 + 1e-9                                  # old cap was 1.0


def test_gait_policy_uses_the_config_scale(tmp_path: Path) -> None:
    cfg = _cfg("hug_scale15")
    (tmp_path / "config.json").write_text(json.dumps(cfg.to_dict()))
    n_obs = PetEnv(cfg).observation_space.shape[0]
    arrays = {"n_layers": np.array(1), "w0": np.zeros((cfg.n_actions, n_obs)), "b0": np.concatenate([np.ones(7), [-1.0]]), "obs_mean": np.zeros(n_obs), "obs_var": np.ones(n_obs),
              "clip_obs": np.array(10.0), "eps": np.array(1e-8), "n_joints": np.array(7)}
    np.savez(tmp_path / "policy.npz", **arrays)
    pol = GaitPolicy(tmp_path)
    assert pol.cfg.action_scale_rad == 1.5 and pol.cfg.action_obs_scale == 1.5
    env = PetEnv(cfg)
    obs, _ = env.reset(seed=2)
    snap = env._snapshot()
    pol.reset(snap)
    tgt = np.zeros(7)
    for _ in range(40):
        tgt = pol.step(env._snapshot())
    assert tgt.max() > 1.45 and tgt.max() <= 1.5 + 1e-9                                                            # action 1.0 -> target 1.5 rad
    assert np.allclose(pol.obs._act[-1], 1.5)                                                                      # the history keeps the old units (action x obs scale)


def test_saturation_metric_on_wrap_attempt_ticks() -> None:
    env = PetEnv(_cfg("hug_curr_02"))
    env.reset(seed=3)
    env._tgt = np.array([0.1, 0.5, -1.0, 0.9, 0.2, 0.0, 0.3])
    q = env.sim.joint_pos()
    run = lambda n, s: SimpleNamespace(hug_modules=n, run_start=s)                                                  # noqa: E731
    env.task._hug.last = run(2, 1)                                                                                  # run < 3: not a wrap-attempt tick
    env._range_step(np.ones(7))
    assert env._range_metrics() == {"wrap_attempt_ticks": 0.0, "max_abs_target_run": 0.0}
    env.task._hug.last = run(4, 2)                                                                                  # run = modules 2..5 -> joints 3, 4, 5 (0-based indices 2, 3, 4)
    a = np.array([0.0, 0.0, -1.0, 0.95, 0.5, 0.0, 0.0])
    env._range_step(a)                                                                                              # in-run actions -1.0 (sat), 0.95 (sat), 0.5 (not)
    m = env._range_metrics()
    assert m["wrap_attempt_ticks"] == 1.0 and m["saturated_frac"] == pytest.approx(2 / 3) and m["sat_any_frac"] == 1.0 and m["sat_all_frac"] == 0.0
    assert m["max_abs_target_run"] == pytest.approx(1.0)                                                            # joint index 2 target -1.0 is inside the run
    assert m["target_gap_run"] == pytest.approx(float(np.mean(np.abs(env._tgt[2:5] - q[2:5]))))
    env._range_step(np.array([0.0, 0.0, 0.94, 0.0, 0.0, 0.0, 0.0]))
    m = env._range_metrics()
    assert m["wrap_attempt_ticks"] == 2.0 and m["saturated_frac"] == pytest.approx((2 / 3 + 0.0) / 2) and m["sat_any_frac"] == 0.5


def test_episode_metrics_carry_the_range_fields() -> None:
    env = PetEnv(_cfg("hug_curr_02", 2.0))
    env.reset(seed=4)
    off = _cfg("hug_curr_02", 2.0)
    off.task_params = {k: v for k, v in off.task_params.items() if k != "range_diag"}
    e2 = PetEnv(off)
    e2.reset(seed=4)
    assert "wrap_attempt_ticks" not in e2.step(np.zeros(8))[4]["metrics"]                                           # off by default: older stages unchanged
    _, _, _, _, info = env.step(np.zeros(8))
    assert {"wrap_attempt_ticks", "max_abs_target_run"} <= set(info["metrics"])


def test_stage_definitions() -> None:
    base, s15, lep = get_stage("hug_curr_02"), get_stage("hug_scale15"), get_stage("hug_long_ep")
    assert dataclasses.replace(s15, name=base.name, description=base.description, action_scale=base.action_scale, action_obs_scale=base.action_obs_scale, task_params=base.task_params) == base
    assert (s15.action_scale, s15.action_obs_scale) == (1.5, 1.5) and s15.task_params == {**base.task_params, 'range_diag': 1.0} and s15.gate == base.gate
    assert s15.ent_coef == 0.002 and s15.dr_scale == 0.3 and s15.episode_s == 20.0 and s15.task_params["hold_weight"] == 10.0 and s15.task_params["shape_phase"] == 1.5 and "hold_ramp" in s15.knobs
    assert (lep.action_scale, lep.action_obs_scale, lep.episode_s) == (1.0, 1.0, 30.0) and lep.gate.episode_s == 20.0 and lep.gate == base.gate
    assert lep.task_params["hug_buffer_p"] == 0.5 and lep.task_params["hug_buffer_min_span"] == 120.0
    assert {k: v for k, v in lep.task_params.items() if k not in ("hug_buffer_p", "hug_buffer_min_span", "range_diag")} == base.task_params
    assert build_config(get_stage("hug_curr_02"), {}).action_obs_scale == 1.0
    assert build_config(s15, {"hold_ramp": 1.0, "n_modules": 8.0}).task_params["hold_ramp"] == 1.0
    assert EnvConfig.from_dict({"action_scale_rad": 1.0}).action_obs_scale == 1.0                                   # old config.json files


def test_range_decision_thresholds() -> None:
    assert hr.range_decision(0.20)[0] == "B" and hr.range_decision(0.35)[0] == "B"
    assert hr.range_decision(0.0999)[0] == "skip_B" and hr.range_decision(0.0)[0] == "skip_B"
    d, why = hr.range_decision(0.15)
    assert d == "B" and "ambiguous" in why and hr.range_decision(0.10)[0] == "B" and hr.range_decision(0.1999)[0] == "B"
    assert hr.range_decision(None)[0] == "B"


def test_pool_range_weights_by_ticks() -> None:
    rows = [{"metrics": {"wrap_attempt_ticks": 100.0, "saturated_frac": 0.5, "sat_any_frac": 1.0, "sat_all_frac": 0.0, "target_gap_run": 0.1, "max_abs_target_run": 0.9}},
            {"metrics": {"wrap_attempt_ticks": 300.0, "saturated_frac": 0.1, "sat_any_frac": 0.5, "sat_all_frac": 0.0, "target_gap_run": 0.3, "max_abs_target_run": 1.0}},
            {"metrics": {"wrap_attempt_ticks": 0.0, "max_abs_target_run": 0.0}}]
    p = hr.pool_range(rows)
    assert p["saturated_frac"] == pytest.approx((50 + 30) / 400) and p["sat_any_frac"] == pytest.approx(0.625) and p["target_gap_run"] == pytest.approx(0.25)
    assert p["max_abs_target_run"] == 1.0 and p["episodes_with_wrap_attempt"] == 2 and p["wrap_attempt_ticks"] == 400.0
    assert hr.pool_range([{"metrics": {"wrap_attempt_ticks": 0.0}}])["saturated_frac"] is None


def test_go_rule() -> None:
    z = {"cyl_r3": 0.0, "cyl_r7": 0.0, "square_8x8": 0.0, "rect_5x14": 0.0, "rounded_8x8_r2": 0.0}
    assert hr.go_rule(0.15, z, 100.0)[0] and not hr.go_rule(0.1333, z, 100.0)[0]
    assert hr.go_rule(0.03, {**z, "cyl_r7": 1 / 6}, 100.0)[0] and hr.go_rule(0.03, {**z, "rounded_8x8_r2": 1 / 6}, None)[0]
    assert not hr.go_rule(0.07, {**z, "square_8x8": 2 / 6}, 140.0)[0]                                                # boxes do not count as curved
    assert hr.go_rule(0.0, z, 165.0)[0] and not hr.go_rule(0.0, z, 164.9)[0]


def _cyl_rows(span: float, n: int = 160) -> list[dict]:
    return [{"shape_kind": 0, "normal_span_deg": span, "success": 0.0} for _ in range(n)]


def test_span_fall_rule() -> None:
    assert hr.span_fall_stop(_cyl_rows(129.0), 140.0)[0]                                                            # fell by 11 deg
    assert not hr.span_fall_stop(_cyl_rows(131.0), 140.0)[0] and not hr.span_fall_stop(_cyl_rows(100.0, 100), 140.0)[0]   # fell by 9 deg / fewer than 150 cylinder episodes
    assert not hr.span_fall_stop(_cyl_rows(100.0), None)[0] and not hr.span_fall_stop(_cyl_rows(170.0), 140.0)[0]


def test_filter_buffer() -> None:
    items = [{"span": 90.0}, {"span": 119.9}, {"span": 120.0}, {"span": 180.0}, {}]
    assert [e["span"] for e in hr.filter_buffer(items, 120.0)] == [120.0, 180.0]


def test_driver_wiring(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    d = _load("hug_range_driver")
    assert (d.L, d.BUDGET_MIN, d.CAP_STEPS, d.MAX_HOURS) == (8, 45.0, 4_000_000, 3) and str(d.SRC) == "data/rl/hug_curr4/hug_curr_02" and d.SERIES == {"scale15": "hug_scale15", "long_ep": "hug_long_ep", "long15": "hug_long15"}
    src = inspect.getsource(d)
    assert "gate2_spec_hug5(L)" in src and "ESCALATE_OWNER" in src and "rescale_joints=rescale" in src and '{"hold_ramp": 1.0}' in src
    assert "summary.jsonl" in inspect.getsource(d6_record()) and hr.SCALE15_REF / hr.SCALE15 == pytest.approx(1 / 1.5)
    # buffer preparation: scale15 copies, long_ep keeps span >= 120
    buf = tmp_path / "src.json"
    buf.write_text(json.dumps([{"span": 100.0}, {"span": 125.0}, {"span": 180.0}]))
    monkeypatch.setattr(d, "SRC_BUFFER", buf)
    d.prepare_buffer("scale15", tmp_path / "a.json")
    d.prepare_buffer("long_ep", tmp_path / "b.json")
    assert len(json.loads((tmp_path / "a.json").read_text())) == 3 and [e["span"] for e in json.loads((tmp_path / "b.json").read_text())] == [125.0, 180.0]
    # halt writes a summary line and a marker
    d.halt(tmp_path, "x", "why")
    assert (tmp_path / "STOP_x").exists() and d.phase_line(tmp_path, "stop_x")["why"] == "why"
    monkeypatch.setattr(d, "log_entry", lambda t, b: None)                                                          # never write the real curriculum log from a test
    d.escalate(tmp_path, "both failed")
    assert (tmp_path / "STOP_ESCALATE_OWNER").exists() and (tmp_path / "ESCALATE").exists() and "L=8 screens" in (tmp_path / "ESCALATE").read_text()


def d6_record():
    import m2v6_driver
    return m2v6_driver.record


def test_progress_videos_cover_the_new_root() -> None:
    pv = _load("progress_videos")
    assert "hug_range*" in inspect.getsource(pv.run_dirs)


Z = {"cyl_r3": 0.0, "cyl_r7": 0.0, "square_8x8": 0.0, "rect_5x14": 0.0, "rounded_8x8_r2": 0.0}


def _succ_rows(p: float, n: int = 200) -> list[dict]:
    return [{"shape_kind": 0, "success": float(int((i + 1) * p + 1e-9) - int(i * p + 1e-9))} for i in range(n)]


def test_success_fall_rule() -> None:
    assert not hr.success_fall_stop(_succ_rows(0.13))[0] and hr.success_fall_stop(_succ_rows(0.12))[0]       # 0.177 - 0.05 = 0.127
    assert not hr.success_fall_stop(_succ_rows(0.30))[0] and not hr.success_fall_stop(_succ_rows(0.0, 100))[0]  # fewer than 150 episodes: no stop
    box = [{"shape_kind": 1, "success_l1": 0.9 if i < 100 else 0.5, "success": 0.2} for i in range(200)]
    assert hr.success_fall_stop(box)[0]                                                                      # box success fell > 0.2


def test_success_trend() -> None:
    rows = [{"success": 0.0}] * 60 + [{"success": 0.0}] * 120 + [{"success": 1.0 if i >= 56 else 0.0} for i in range(60)]
    assert hr.success_trend(rows)["rising"] and not hr.success_trend([{"success": 0.1}] * 120)["rising"] and not hr.success_trend([{"success": 1.0}] * 10)["rising"]


def test_post_hour_decisions() -> None:
    d = hr.post_hour_decision
    assert d("scale15", 2, 0.55, Z, False) == "final" and d("long15", 1, 0.6, Z, False) == "final" and d("scale15", 4, 0.55, Z, False) == "final"
    assert d("scale15", 2, 0.15, Z, False) == "next" and d("scale15", 2, 0.0, {**Z, "cyl_r3": 1 / 6}, False) == "next" and d("scale15", 2, 0.0, Z, True) == "next"
    assert d("scale15", 2, 0.1, {**Z, "square_8x8": 0.5}, False) == "C15"                                    # boxes do not count as curved
    assert d("scale15", 3, 0.29, {**Z, "cyl_r7": 1 / 6}, True) == "escalate" and d("scale15", 3, 0.30, Z, False) == "extra"
    assert d("scale15", 3, 0.2, {**Z, "rounded_8x8_r2": 2 / 6}, False) == "extra"
    assert d("long15", 1, 0.2, Z, True) == "escalate" and d("long15", 1, 0.3, Z, False) == "extra"
    assert d("scale15", 4, 0.54, Z, True) == "escalate" and d("long15", 2, 0.4, Z, True) == "escalate"       # after the extra hour only 0.55 passes


def _drive(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, screens: list[tuple[float, dict, bool]], reasons: dict | None = None):
    """Run ``run_post`` from hour 2 with synthetic hours; returns (result, trained (series, hour) list, markers, summary lines)."""
    d = _load("hug_range_driver")
    trained, it = [], iter(screens)
    monkeypatch.setattr(d, "train_hour", lambda root, series, hour, init, n, sc, f: (trained.append((series, hour, init.name)) or tmp_path / f"{series}_h{hour}", (reasons or {}).get((series, hour))))
    monkeypatch.setattr(d.hd, "family_dump", lambda out: {"cyl_span_p75": 150.0})
    monkeypatch.setattr(d.hl, "read_rows_tolerant", lambda p: [])

    def fake_screen(out, stage, n, w):
        s, per, rising = next(it)
        fake_screen.rising = rising
        return {"success": s, "per_shape": {**Z, **per}}
    monkeypatch.setattr(d, "screen", fake_screen)
    monkeypatch.setattr(d.hr, "success_trend", lambda rows: {"rising": fake_screen.rising})
    monkeypatch.setattr(d, "final_verdict", lambda root, out, series, hour, n, w, f: (root / "DONE").write_text(f"{series} {hour}"))
    monkeypatch.setattr(d, "log_entry", lambda t, b: None)
    a = SimpleNamespace(force_go=False, screen_n=30, final_n=100, eval_workers=1)
    res = d.run_post(tmp_path, 2, tmp_path / "scale15_h1", 1, 1.0, a)
    return res, trained, sorted(p.name for p in tmp_path.iterdir() if p.name.isupper() or p.name.startswith("STOP")), d.lines(tmp_path)


def test_run_post_branches(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # h2 screen 0.60 -> verdict
    (tmp_path / "a").mkdir()
    r, tr, mk, ln = _drive(tmp_path / "a", monkeypatch, [(0.60, {}, False)])
    assert r == "done" and tr == [("scale15", 2, "scale15_h1")] and "DONE" in mk and any(x["phase"] == "decision_scale15_h2" and x["decision"] == "final" for x in ln)
    # h2 0.10, nothing -> C' warm from h2 on scale 1.5 (series long15 hour 1), then 0.20/no curved -> escalate
    (tmp_path / "b").mkdir()
    r, tr, mk, ln = _drive(tmp_path / "b", monkeypatch, [(0.10, {}, False), (0.20, {}, False)])
    assert r == "stopped" and tr == [("scale15", 2, "scale15_h1"), ("long15", 1, "scale15_h2")] and "STOP_ESCALATE_OWNER" in mk and "ESCALATE" in mk
    # h2 rising -> h3 (0.35) -> extra h4 (0.56) -> verdict
    (tmp_path / "c").mkdir()
    r, tr, mk, ln = _drive(tmp_path / "c", monkeypatch, [(0.10, {}, True), (0.35, {}, False), (0.56, {}, False)])
    assert r == "done" and [t[:2] for t in tr] == [("scale15", 2), ("scale15", 3), ("scale15", 4)]
    # h2 0.2 -> h3 0.2 (<0.30, no curved 2/6) -> escalate, no h4
    (tmp_path / "d").mkdir()
    r, tr, mk, ln = _drive(tmp_path / "d", monkeypatch, [(0.20, {}, False), (0.20, {"cyl_r3": 1 / 6}, False)])
    assert r == "stopped" and len(tr) == 2 and "STOP_ESCALATE_OWNER" in mk
    # 20-min stop in hour 2 -> halt, no further hour
    (tmp_path / "e").mkdir()
    r, tr, mk, ln = _drive(tmp_path / "e", monkeypatch, [(0.20, {}, False)], {("scale15", 2): "curved_trend_20min"})
    assert r == "stopped" and len(tr) == 1 and "STOP_curved_trend_20min" in mk


def test_start_success_checks_base_is_first_150_episodes(tmp_path: Path) -> None:
    """The --force-c15 rule uses the run's own first 150 episodes as the base."""
    chk = hr.StartSuccessChecks(tmp_path)
    rows = [{"success": 0.4, "family": "cyl"}] * 150 + [{"success": 0.0, "family": "cyl"}] * 150
    chk._rows = lambda: rows  # type: ignore[method-assign]
    chk._record = lambda *a, **k: None  # type: ignore[method-assign]
    chk.run_check1()
    assert abs(chk.base_success - 0.4) < 1e-9
