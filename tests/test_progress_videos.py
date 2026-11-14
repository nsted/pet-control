"""Progress export callback, progress-video watcher discovery / best-run choice, caption-number consistency, no video budget."""
from __future__ import annotations

import importlib.util
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

from petctl.rl import video  # noqa: E402
from petctl.rl.config import DEFAULT_URDF_DIR, EnvConfig  # noqa: E402
from petctl.rl.policy import POLICY_FILE, GaitPolicy  # noqa: E402

needs_urdf = pytest.mark.skipif(not os.path.isdir(DEFAULT_URDF_DIR), reason="Fusion URDF export not present")


def _watcher():
    spec = importlib.util.spec_from_file_location("progress_videos", Path(__file__).resolve().parents[1] / "scripts" / "rl" / "progress_videos.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["progress_videos"] = mod
    spec.loader.exec_module(mod)
    return mod


def test_no_video_budget(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(video, "_VIDEO_LOG", Path("/nonexistent/x.jsonl"))
    assert video.video_budget_ok() is True and video.video_budget_ok(now=0.0) is True
    assert not hasattr(video, "VIDEO_BUDGET_PER_HOUR")


@needs_urdf
@pytest.mark.parametrize("asym", [False, True])
def test_progress_export_loadable_by_gaitpolicy(tmp_path: Path, asym: bool) -> None:
    from petctl.rl.config import EnvConfig
    from petctl.rl.records import ProgressExportCallback
    from petctl.rl.train import train

    cfg = EnvConfig(task="goal_reach", n_modules=8, episode_s=2.0, action_mode="hold" if asym else "abs")
    cfg.dr.scale = 0.0
    os.environ["PETCTL_PROGRESS_EXPORT_S"] = "0.5"
    try:
        out = train(cfg, steps=1024, n_envs=1, out=tmp_path / "r", asymmetric=asym)
    finally:
        os.environ.pop("PETCTL_PROGRESS_EXPORT_S")
    prog = json.loads((out / "progress.json").read_text())
    assert {"timesteps", "wall_s", "rolling_success_100", "time"} <= set(prog) and prog["timesteps"] > 0
    assert prog["rolling_success_100"] is None or 0.0 <= prog["rolling_success_100"] <= 1.0
    assert not list(out.glob("*.tmp*"))
    a = GaitPolicy(out, "progress_policy.npz")
    b = GaitPolicy(out, POLICY_FILE)
    assert a._w[0].shape == b._w[0].shape and a.cfg.n_joints == b.cfg.n_joints
    assert ProgressExportCallback(out).period_s == 1200.0


def _fake_runs(data: Path, now: float) -> dict[str, Path]:
    def mk(root: str, name: str, task: str, succ: float, steps: int, age_s: float, start_age_s: float = 3600.0) -> Path:
        d = data / root / "L8" / name
        d.mkdir(parents=True)
        (d / "config.json").write_text(json.dumps({"task": task, "n_modules": 8}))
        rows = [{"timesteps": steps - 100 + i, "wall_s": 60.0 + i, "success": float(i < succ * 100)} for i in range(100)]
        (d / "episodes.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
        (d / "progress_policy.npz").write_bytes(b"x")
        for f in d.iterdir():
            os.utime(f, (now - age_s, now - age_s))
        os.utime(d / "config.json", (now - start_age_s, now - start_age_s))
        return d
    return {"a": mk("m2v7", "A", "goal_reach", 0.3, 1000, 60), "b": mk("m2v7", "B", "goal_reach", 0.6, 900, 120), "c": mk("m2v7", "C", "goal_reach", 0.9, 5000, 4000),
            "h": mk("m2v7", "H", "goal_hug", 0.1, 800, 30), "old": mk("m2v6", "O", "goal_reach", 0.99, 9000, 7200), "old_day": mk("m2v7", "D", "goal_reach", 0.95, 9000, 10)}


def test_watcher_discovery_and_best_run(tmp_path: Path) -> None:
    w = _watcher()
    now = time.time()
    runs = _fake_runs(tmp_path, now)
    os.utime(runs["old_day"] / "config.json", (now - 90_000, now - 90_000))             # started > 24 h ago: not eligible as best
    st = w.discover(tmp_path, now, alive=False)
    assert set(st) == {("m2v7", "flat"), ("m2v7", "hug")}                              # C and O are inactive (> 15 min), v6 root has nothing active
    assert st[("m2v7", "flat")]["best"]["run"] == runs["b"] and st[("m2v7", "hug")]["best"]["run"] == runs["h"]
    assert st[("m2v7", "flat")]["best"]["rolling_success"] == pytest.approx(0.6) and st[("m2v7", "flat")]["best"]["policy_file"] == "progress_policy.npz"
    (tmp_path / "m2v7" / "DONE_P2").write_text("")                                      # complete root + no live driver -> nothing active there
    assert w.discover(tmp_path, now, alive=False) == {}
    assert set(w.discover(tmp_path, now, alive=True)) == {("m2v7", "flat"), ("m2v7", "hug")}
    st = w.discover(tmp_path, now, alive=False, forced=[runs["c"]])                    # forced run counts as active despite markers/age
    assert st[("m2v7", "flat")]["best"]["run"] == runs["c"]


def test_watcher_ticks_are_idempotent(tmp_path: Path) -> None:
    w = _watcher()
    now = time.time()
    runs = _fake_runs(tmp_path, now)
    os.utime(runs["old_day"] / "config.json", (now - 90_000, now - 90_000))
    st = w.discover(tmp_path, now, alive=False)
    flat = st[("m2v7", "flat")]
    assert w.due(flat, "flat", now, 1800.0) is True and w.due(flat, "flat", now, 7200.0) is False       # stage active ~1 h: first tick due; not with a 2 h period
    (tmp_path / "m2v7" / "progress_ticks").mkdir()
    (tmp_path / "m2v7" / "progress_ticks" / "flat_x.json").write_text("{}")                          # a tick was just rendered -> idempotent until the period passes
    assert w.due(flat, "flat", time.time() + 100, 1800.0) is False and w.due(flat, "flat", time.time() + 4000, 1800.0) is True


@needs_urdf
def test_caption_numbers_equal_recorded_metrics(tmp_path: Path) -> None:
    from tests.test_m2v7 import _cfg, _fake_run

    run = _fake_run(tmp_path, _cfg("abs"))
    seen: list[tuple[dict[str, float], bool]] = []
    m = video.record_video(run, tmp_path / "v.mp4", seconds=1.0, seed=10_000, size=(320, 180), fps=10, distance=1.6,
                           final_note=lambda met, reached: seen.append((met, reached)) or "note")
    assert (tmp_path / "v.mp4").stat().st_size > 0 and len(seen) == 1 and seen[0][0] == m
    probe = video.rollout_metrics(run, "policy.npz", seed=10_000, seconds=1.0)         # the render-free probe reproduces the recorded episode
    assert probe.keys() == m.keys() and all(np.isclose(probe[k], m[k], equal_nan=True) for k in m)
    w = _watcher()
    note = w.episode_note({"success": 1.0, "final_dist": 0.123, "hug_modules": 5.0, "wrap_deg": 211.0}, True, 10_003, True, "hug")
    assert "SUCCESS" in note and "0.12 m" in note and "hug modules 5" in note and "wrap 211 deg" in note and "no successful" not in note
    assert "no successful episode yet" in w.episode_note({"success": 0.0}, False, 10_000, False, "flat")
