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
        out = train(cfg, steps=1024, n_envs=1, out=tmp_path / "r", asymmetric=asym, video=False)   # never render into docs/media
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


def test_watcher_discovery_per_run(tmp_path: Path) -> None:
    w = _watcher()
    now = time.time()
    runs = _fake_runs(tmp_path, now)
    found = {i["run"]: i for i in w.discover_runs(tmp_path, now)}
    assert {runs["a"], runs["b"], runs["h"], runs["old_day"]} <= set(found) and all(found[runs[k]]["alive"] for k in ("a", "b", "h", "old_day"))      # EVERY alive run, not only the best of a stage
    assert not found[runs["c"]]["alive"] and runs["c"] in found and runs["old"] in found            # ended (stale > 15 min, < 2 h or 2 h edge) runs without a final video get one
    assert found[runs["b"]]["rolling_success"] == pytest.approx(0.6) and found[runs["b"]]["policy_file"] == "progress_policy.npz" and found[runs["h"]]["family"] == "hug"
    (runs["c"] / "progress_ticks").mkdir()
    (runs["c"] / "progress_ticks" / "final_x.json").write_text("{}")
    assert runs["c"] not in {i["run"] for i in w.discover_runs(tmp_path, now)}                       # final video already rendered
    (runs["a"] / "meta.json").write_text("{}")                                                       # a finished run is no longer alive
    assert not {i["run"]: i for i in w.discover_runs(tmp_path, now)}[runs["a"]]["alive"]
    assert w.discover_runs(tmp_path, now, forced=[runs["c"]]) and {i["run"]: i for i in w.discover_runs(tmp_path, now, forced=[runs["c"]])}[runs["c"]]["alive"]
    aborted = tmp_path / "m2v7" / "L8" / "P1.aborted_1"
    aborted.mkdir(parents=True)
    (aborted / "episodes.jsonl").write_text("{}\n")
    assert aborted not in {r for _, r in w.run_dirs(tmp_path)}


def test_watcher_schedule(tmp_path: Path) -> None:
    w = _watcher()
    now = time.time()
    first, period = 720.0, 1500.0
    mk = lambda **k: {"alive": True, "ticks": [], "start": now - 600.0, **k}                       # noqa: E731
    assert w.next_kind(mk(), now, first, period) is None                                             # 10 min in: not yet
    assert w.next_kind(mk(start=now - 800.0), now, first, period) == "progress"                      # first video 12 min after the start (<= 15 min)
    assert w.next_kind(mk(start=now - 4000.0, ticks=[(now - 1000.0, "progress")]), now, first, period) is None
    assert w.next_kind(mk(start=now - 4000.0, ticks=[(now - 1500.0, "progress")]), now, first, period) == "progress"      # 25 min period: 1-2 per 30 min, never > 3
    assert w.next_kind(mk(alive=False, ticks=[(now - 100.0, "progress")]), now, first, period) == "final"
    assert w.next_kind(mk(alive=False, ticks=[(now - 100.0, "final")]), now, first, period) is None
    # a 20-min run: one video at ~12 min (midpoint) and the final one at its end
    t = 0.0
    seen = []
    for minute in range(0, 21):
        t = now + 60.0 * minute
        info = mk(start=now, ticks=list(seen), alive=minute < 20)
        k = w.next_kind(info, t, first, period)
        if k:
            seen.append((t, k))
    assert [k for _, k in seen] == ["progress", "final"]


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
