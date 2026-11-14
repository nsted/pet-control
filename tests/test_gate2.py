"""Regimen v2 gate tooling (gate2), asymmetric critic, contact sheet (M0-E)."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

from petctl.rl.config import DEFAULT_URDF_DIR  # noqa: E402
from petctl.rl.curriculum import gate2  # noqa: E402
from petctl.rl.curriculum.gate2 import Ablation, Gate2Spec, mcnemar_exact_p, paired_drop, run_gate2, wilson_ci  # noqa: E402
from petctl.rl.curriculum.stages import get_stage  # noqa: E402

needs_urdf = pytest.mark.skipif(not os.path.isdir(DEFAULT_URDF_DIR), reason="Fusion URDF export not present")
_S8 = Path(__file__).resolve().parents[1] / "data" / "rl" / "curriculum_fast" / "s8_obstacle_field" / "iter_10"


# ---------------------------------------------------------------- statistics
def test_wilson_known_values() -> None:
    lo, hi = wilson_ci(80, 100)
    assert lo == pytest.approx(0.7112, abs=5e-4) and hi == pytest.approx(0.8666, abs=5e-4)
    lo, hi = wilson_ci(0, 10)
    assert lo == pytest.approx(0.0, abs=1e-12) and hi == pytest.approx(0.2775, abs=5e-4)
    lo, hi = wilson_ci(10, 10)
    assert hi == pytest.approx(1.0) and lo == pytest.approx(0.7225, abs=5e-4)
    assert wilson_ci(0, 0) == (0.0, 1.0)
    lo, hi = wilson_ci(50, 100)
    assert lo == pytest.approx(1 - hi) and lo < 0.5 < hi


def test_mcnemar_exact() -> None:
    assert mcnemar_exact_p(0, 0) == 1.0
    assert mcnemar_exact_p(10, 0) == pytest.approx(2 / 1024)
    assert mcnemar_exact_p(5, 5) == 1.0
    assert mcnemar_exact_p(3, 9) == mcnemar_exact_p(9, 3)


def test_paired_drop_deterministic_and_ci() -> None:
    base = np.ones(100)
    abl = np.ones(100)
    abl[:30] = 0.0                                                   # exactly 30 seeds lose the success
    d = paired_drop(base, abl)
    assert d["drop"] == pytest.approx(0.30) and d["base_only"] == 30 and d["abl_only"] == 0
    assert d["ci_lo"] < 0.30 < d["ci_hi"] and d["ci_lo"] > 0.15 and d["mcnemar_p"] < 1e-6
    assert paired_drop(base, abl) == d                               # fixed bootstrap seed: reproducible
    same = paired_drop(base, base)
    assert same["drop"] == 0.0 and same["ci_lo"] == 0.0 and same["ci_hi"] == 0.0
    with pytest.raises(ValueError):
        paired_drop(np.ones(3), np.ones(4))


# ---------------------------------------------------------------- toy deterministic evaluator
def _toy_eval(n_fail_touch: int = 40, blind_rate: float = 0.1):
    """success(seed) deterministic: base succeeds on all seeds; touch_off fails on the first ``n_fail_touch`` seeds; blind (all three off) succeeds on a fraction."""
    def ev(overrides: dict, seeds: list[int], leak: bool = False, dr_scale: float | None = None) -> list[dict]:
        rows = []
        for i, s in enumerate(seeds):
            ok = 1.0
            if overrides.get("touch_off"):
                ok = 0.0 if i < n_fail_touch else 1.0
            if overrides.get("capacitive_off") and overrides.get("imu_off"):
                ok = 1.0 if i < blind_rate * len(seeds) else 0.0
            if leak:
                ok = 0.0
            rows.append({"seed": s, "ret": float(s % 7) + 100.0 * ok, "steps": 10, "gt": {"gtm": ok}, "leak_moved_at_step": 3 if leak else -1,
                          "metrics": {"success": ok, "glitch": 0.0, "bus_mean_a": 2.0, "frac_ticks_over_10a": 0.0}})
        return rows
    return ev


def test_gate2_toy_pass_fail_void(tmp_path: Path) -> None:
    stage = get_stage("s1_goal_ahead")
    spec = Gate2Spec(n=100, ablations=(Ablation("touch_off", {"touch_off": 1.0}, min_drop=0.2),), blind_overrides=gate2.BLIND_OVERRIDES, blind_ceiling=0.3,
                     leak_test=True, leak_max_success=0.2, second_metrics={"gtm": lambda e, i: 0.0})
    r = run_gate2(tmp_path, stage, spec, evaluator=_toy_eval(40), write=True)
    assert r["status"] == "PASS" and r["passed"], r["fail_reasons"]
    assert r["robust"]["k"] == 100 and r["robust"]["wilson_lo"] > 0.96
    assert r["ablations"]["touch_off"]["drop"]["drop"] == pytest.approx(0.4) and r["ablations"]["touch_off"]["passed"]
    assert r["blind"]["rate"] == pytest.approx(0.1) and r["blind"]["within_ceiling"]
    assert r["leak"]["rate"] == 0.0 and r["leak"]["moved_fraction"] == 1.0
    assert r["best_episode"]["seed"] % 7 == 6 and r["current"]["ok"]
    saved = json.loads((tmp_path / f"gate2_{stage.name}.json").read_text())
    assert saved["status"] == "PASS" and saved["nominal"]["k"] == 100
    # ablation drop too small -> FAIL
    r = run_gate2(tmp_path, stage, spec, evaluator=_toy_eval(10), write=False)
    assert r["status"] == "FAIL" and any("ablation touch_off" in x for x in r["fail_reasons"])
    # blind baseline above ceiling -> VOID
    r = run_gate2(tmp_path, stage, spec, evaluator=_toy_eval(40, blind_rate=0.6), write=False)
    assert r["status"] == "VOID" and r["void"]


def test_gate2_hard_fails(tmp_path: Path) -> None:
    stage = get_stage("s1_goal_ahead")
    base = _toy_eval(0)

    def hot(o, s, leak=False, dr_scale=None):
        rows = base(o, s, leak, dr_scale)
        for r in rows:
            r["metrics"]["bus_mean_a"] = 4.5
        return rows

    def spiky(o, s, leak=False, dr_scale=None):
        rows = base(o, s, leak, dr_scale)
        for r in rows[:10]:
            r["metrics"]["frac_ticks_over_10a"] = 0.01
        return rows

    def glitchy(o, s, leak=False, dr_scale=None):
        rows = base(o, s, leak, dr_scale)
        rows[0]["metrics"]["glitch"] = 1.0
        return rows

    def unmeasured(o, s, leak=False, dr_scale=None):
        rows = base(o, s, leak, dr_scale)
        for r in rows:
            del r["metrics"]["bus_mean_a"]
        return rows

    spec = Gate2Spec(n=50)
    for ev, text in ((hot, "current budget"), (spiky, "current budget"), (glitchy, "glitch"), (unmeasured, "not measured")):
        r = run_gate2(tmp_path, stage, spec, evaluator=ev, write=False)
        assert r["status"] == "FAIL" and any(text in x for x in r["fail_reasons"]), (text, r["fail_reasons"])
    # success below the threshold on the CI lower bound, even though the point estimate clears it
    ev = lambda o, s, leak=False, dr_scale=None: [dict(r, metrics={**r["metrics"], "success": 1.0 if i < 24 else 0.0}) for i, r in enumerate(base(o, s))]  # noqa: E731
    r = run_gate2(tmp_path, stage, Gate2Spec(n=30, threshold=0.7), evaluator=ev, write=False)     # 24/30 = 0.8 >= 0.7 but Wilson lo ~0.63
    assert r["robust"]["rate"] == pytest.approx(0.8) and r["status"] == "FAIL"


def test_second_metric_defines_success(tmp_path: Path) -> None:
    stage = get_stage("s1_goal_ahead")
    ev = _toy_eval(0)
    rows_fn = lambda o, s, leak=False, dr_scale=None: [dict(r, gt={"gtm": 0.0}) for r in ev(o, s)]  # noqa: E731
    r = run_gate2(tmp_path, stage, Gate2Spec(n=20, success_key="gtm"), evaluator=rows_fn, write=False)
    assert r["robust"]["k"] == 0 and r["status"] == "FAIL"


# ---------------------------------------------------------------- end to end on the real S8 policy
@needs_urdf
@pytest.mark.skipif(not _S8.is_dir(), reason="S8 iter_10 run not present")
def test_gate2_end_to_end_s8(tmp_path: Path) -> None:
    import shutil

    run = tmp_path / "iter_10"
    run.mkdir()
    for f in ("config.json", "policy.npz"):
        shutil.copy(_S8 / f, run / f)
    stage = get_stage("s8_obstacle_field")
    spec = Gate2Spec(n=6, nominal=True, ablations=(Ablation("touch_off", {"touch_off": 1.0}, min_drop=0.0), Ablation("imu_off", {"imu_off": 1.0})),
                     blind_overrides=gate2.BLIND_OVERRIDES, blind_ceiling=1.0)
    r = run_gate2(run, stage, spec)
    js = json.loads((run / f"gate2_{stage.name}.json").read_text())
    for k in ("status", "robust", "nominal", "ablations", "blind", "current", "best_episode", "per_seed", "glitch_rate", "fail_reasons", "threshold"):
        assert k in js
    assert js["robust"]["n"] == 6 and 0.0 <= js["robust"]["wilson_lo"] <= js["robust"]["wilson_hi"] <= 1.0
    assert js["current"]["measured"] and set(js["ablations"]) == {"touch_off", "imu_off"}
    assert len(js["per_seed"]) == 6 and js["ablations"]["touch_off"]["drop"]["method"].startswith("paired")
    assert r["status"] in ("PASS", "FAIL", "VOID")
    if js["best_episode"] is not None:                                    # contact sheet of that episode, no video
        out = gate2.contact_sheet(run, stage.name, n_frames=6)
        assert out.exists() and out.suffix == ".png"


@needs_urdf
@pytest.mark.skipif(not _S8.is_dir(), reason="S8 iter_10 run not present")
def test_contact_sheet_writes_png(tmp_path: Path) -> None:
    import shutil

    from PIL import Image

    run = tmp_path / "iter_10"
    run.mkdir()
    for f in ("config.json", "policy.npz"):
        shutil.copy(_S8 / f, run / f)
    stage = get_stage("s8_obstacle_field")
    out = gate2.contact_sheet_episode(run, tmp_path / "sheet.png", 10_000, dict(stage.gate.task_params), 0.0, 3.0, size=(160, 90))
    im = Image.open(out)
    assert im.size == (3 * 160, 2 * 90)


def test_render_best_uses_record_video_and_budget(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    from petctl.rl import video

    stage = get_stage("s8_obstacle_field")
    (tmp_path / f"gate2_{stage.name}.json").write_text(json.dumps({"best_episode": {"seed": 10_042}, "dr_scale": 0.6, "episode_s": 30.0, "eval_task_params": {"a": 1.0}}))
    calls = []
    monkeypatch.setattr(video, "record_video", lambda run, out, pf, **kw: calls.append((out, kw)) or {})
    p = gate2.render_best(tmp_path, stage.name)
    assert len(calls) == 1 and calls[0][1]["seed"] == 10_042 and calls[0][1]["replace_task_params"] and stage.name in calls[0][1]["label"] and p == calls[0][0]
    monkeypatch.undo()
    monkeypatch.setattr(video, "video_budget_ok", lambda now=None: False)
    with pytest.raises(RuntimeError, match="budget"):
        gate2.render_best(tmp_path, stage.name)


# ---------------------------------------------------------------- env: imu_off knob, privileged vector
@needs_urdf
def test_imu_off_zeroes_imu_channels_only() -> None:
    from petctl.rl.config import EnvConfig
    from petctl.rl.env import PetEnv

    def first(params: dict) -> np.ndarray:
        env = PetEnv(EnvConfig(task="goal_reach", task_params=params, episode_s=2.0))
        o, _ = env.reset(seed=3)
        return o
    a, b = first({}), first({"imu_off": 1.0})
    assert np.all(b[:6] == 0.0) and np.any(a[:6] != 0.0) and np.array_equal(a[6:], b[6:])


@needs_urdf
def test_privileged_vector_layout() -> None:
    from petctl.rl.config import EnvConfig
    from petctl.rl.env import PetEnv

    env = PetEnv(EnvConfig(task="goal_hug", task_params={"capacitive_obs": 1.0, "bus_model": 1.0}, episode_s=2.0))
    env.reset(seed=1)
    pv = env.privileged_vector()
    assert pv.shape == (env.privileged_dim(),) == (17 + 7 * 8,) and np.all(np.isfinite(pv))
    assert pv[9] == 1.0                                                  # post present flag
    env.step(np.zeros(7))
    assert env.privileged_vector().shape == pv.shape


# ---------------------------------------------------------------- asymmetric critic
@needs_urdf
def test_asymmetric_critic_smoke(tmp_path: Path) -> None:
    pytest.importorskip("stable_baselines3")
    from petctl.rl.config import EnvConfig
    from petctl.rl.env import PetEnv
    from petctl.rl.policy import GaitPolicy
    from petctl.rl.train import train

    cfg = EnvConfig(task="goal_reach", task_params={"goal_dist_min": 0.8, "goal_dist_max": 1.2}, episode_s=4.0)
    out = train(cfg, 2048, 2, tmp_path / "asym", video=False, asymmetric=True)
    meta = json.loads((out / "meta.json").read_text())
    assert meta["asymmetric"] is True
    pol = GaitPolicy(out)
    n_obs = PetEnv(cfg).observation_space.shape[0]
    assert pol._mean.shape == (n_obs,)                                                   # actor-only normalisation: no privileged columns
    import pickle

    import stable_baselines3 as sb3
    import torch

    model = sb3.PPO.load(str(out / "model"), device="cpu")
    vn = pickle.load(open(out / "vecnormalize.pkl", "rb"))
    assert model.policy.mlp_extractor.policy_net[0].in_features == n_obs
    assert model.policy.mlp_extractor.value_net[0].in_features == n_obs + PetEnv(cfg).privileged_dim()
    env = PetEnv(cfg)
    o, _ = env.reset(seed=5)
    for _ in range(3):
        full = np.concatenate([o, env.privileged_vector()])
        # the actor output must not depend on the privileged tail
        a_exp = []
        for tail in (full[n_obs:], full[n_obs:] * 3 + 1):
            x = np.clip((np.concatenate([o, tail]) - vn.obs_rms.mean) / np.sqrt(vn.obs_rms.var + vn.epsilon), -vn.clip_obs, vn.clip_obs)
            a, _ = model.predict(x[None].astype(np.float32), deterministic=True)
            a_exp.append(np.clip(a[0], -1, 1))
        assert np.allclose(a_exp[0], a_exp[1])
        assert np.allclose(pol.action(o), a_exp[0], atol=1e-4)                       # exported policy.npz == the actor
        o, *_ = env.step(a_exp[0])
    from petctl.rl.train import evaluate
    assert "distance_median" in evaluate(out, episodes=1)


@needs_urdf
def test_default_training_path_unchanged(tmp_path: Path) -> None:
    pytest.importorskip("stable_baselines3")
    import stable_baselines3 as sb3

    from petctl.rl.config import EnvConfig
    from petctl.rl.train import train

    out = train(EnvConfig(task="goal_reach", episode_s=3.0), 512, 1, tmp_path / "sym", video=False)
    assert "asymmetric" not in json.loads((out / "meta.json").read_text())
    assert not hasattr(sb3.PPO.load(str(out / "model"), device="cpu").policy.mlp_extractor, "n_policy")
