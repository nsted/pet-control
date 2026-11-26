"""Frozen eval suites: manifest integrity, split disjointness, TEST-set gating, length independence, and that every scenario builds in the env."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from petctl.rl import evalsuite as es
from petctl.rl.config import EnvConfig
from petctl.rl.env import PetEnv


def test_committed_suite_matches_manifest() -> None:
    assert es.check_manifest() == []


def test_generator_reproduces_committed_files() -> None:
    for split in es.SPLITS:
        for skill in es.SKILLS:
            on_disk = json.loads(es._file(split, skill).read_text())
            gen = json.loads(json.dumps([es.asdict(s) for s in es.build_suite(split, skill)]))
            assert on_disk == gen, (split, skill)


def test_splits_disjoint_and_covering() -> None:
    seeds = {sp: {s.seed for k in es.SKILLS for s in es.build_suite(sp, k)} for sp in es.SPLITS}
    assert not seeds["decision"] & seeds["test"] and min(min(v) for v in seeds.values()) >= 10_000
    for sp in es.SPLITS:
        rl = es.build_suite(sp, "self_righting")
        assert {s.start_orient for s in rl} == {1, 2, 3, 4}
        assert {s.terrain for s in es.build_suite(sp, "terrain")} >= {"rolling", "ridges", "dunes"}


def test_tamper_detected_and_write_refused(tmp_path: Path) -> None:
    es.write_suite(tmp_path)
    assert es.check_manifest(tmp_path) == []
    with pytest.raises(FileExistsError):
        es.write_suite(tmp_path)
    f = es._file("decision", "locomotion", tmp_path)
    f.write_text(f.read_text().replace('"seed"', '"seed" ', 1))
    assert es.check_manifest(tmp_path)
    with pytest.raises(RuntimeError):
        es.load_suite("decision", "locomotion", tmp_path)


def test_test_set_needs_milestone(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    es.write_suite(tmp_path)
    monkeypatch.delenv("PETCTL_OPEN_TEST", raising=False)
    with pytest.raises(PermissionError):
        es.load_suite("test", "terrain", tmp_path, reason="x")
    monkeypatch.setenv("PETCTL_OPEN_TEST", "1")
    with pytest.raises(PermissionError):
        es.load_suite("test", "terrain", tmp_path)
    assert len(es.load_suite("test", "terrain", tmp_path, reason="milestone")) == es.N_PER_SET
    assert "milestone" in (tmp_path / "test_access.log").read_text()


@pytest.mark.parametrize("n_modules", [10, 13])
def test_scenarios_build_at_any_length(n_modules: int) -> None:
    for skill in es.SKILLS:
        sc = es.build_suite("decision", skill)[0]
        cfg = EnvConfig(task="goal_reach", n_modules=n_modules, episode_s=0.5, action_mode="hold", task_params={"goal_dist_min": 1.0, "goal_dist_max": 2.0, **es.task_params(sc)})
        env = PetEnv(cfg)
        env.reset(seed=sc.seed)
        env.step(__import__("numpy").zeros(cfg.n_actions, "float32"))
