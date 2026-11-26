"""Frozen, length-generic evaluation suites (docs/plans/AUTONOMOUS_TRAINER.md, Phase 1).

Per skill there are two disjoint sets: DECISION (gates, choices, watchdog metric) and TEST (never used for decisions; opened only at milestones, every open is
logged to ``data/rl/eval_suite/test_access.log``). A scenario is a pure spec (seed, start orientation / twist, terrain, friction, domain-randomisation level,
command or goal): nothing refers to the body length, so every candidate length replays the same scenarios. ``data/rl/eval_suite/manifest.json`` stores the SHA-256
of every file; ``check_manifest`` (also ``scripts/rl/eval_suite.py check``) fails when anything changed. Changing a suite needs the owner's approval.

Fields consumed by the current env: ``start_orient`` / ``start_pose_rad`` / ``terrain`` / ``dr_level`` / ``goal`` (see ``task_params``). ``command``, ``friction_aniso``,
``obstacles`` and ``pose_family`` are part of the frozen spec and become live with the Phase 2 / 7 / 8 features.
"""

from __future__ import annotations

import hashlib
import json
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

SUITE_DIR = Path("data/rl/eval_suite")
SPLITS = ("decision", "test")
SKILLS = ("locomotion", "turning", "self_righting", "terrain", "obstacle", "pose_hold")
N_PER_SET = 24
SEED_BASE = {"decision": 2_000_000, "test": 7_000_000}          # both >= GATE_SEED_MIN (no layout cache), far from any training seed
MASTER_SEED = 20261126
ORIENT_NAMES = {0: "upright", 1: "upside_down", 2: "side_left", 3: "side_right", 4: "random_roll"}
TERRAINS = ("flat", "rolling", "ridges", "dunes")
POSE_FAMILIES = ("flat_zigzag", "arch", "loop", "spiral", "tripod", "wave", "coil")


@dataclass
class Scenario:
    """One evaluation scenario (JSON-serialisable; no body-length dependence)."""

    id: str
    skill: str
    split: str
    seed: int
    start_orient: int = 0                 # 0 upright, 1 upside-down, 2 left side, 3 right side, 4 random roll (env task knob ``start_orient``)
    start_pose_rad: float = 0.0           # random joint twist at the start, uniform +-this (env task knob ``start_pose_rad``)
    terrain: str = "flat"
    terrain_level: float = 0.0
    friction_mean: float = 1.0
    friction_aniso: float = 1.0           # forward/lateral friction ratio (1 = isotropic); live from Phase 2
    dr_level: float = 0.3                 # domain-randomisation scale in [0, 1]
    command: tuple[float, float, float] | None = None     # (vx m/s, vy m/s, yaw rad/s) in the principal-axis heading frame; live from Phase 7
    goal: tuple[float, float] | None = None               # (bearing deg relative to the start heading, distance m)
    obstacle_density: float = 0.0
    pose_family: str = ""
    episode_s: float = 30.0
    notes: str = ""
    extra: dict[str, Any] = field(default_factory=dict)


def _scenario(skill: str, split: str, i: int, rng: np.random.Generator) -> Scenario:
    seed = SEED_BASE[split] + SKILLS.index(skill) * 1000 + i
    # stratified cycles guarantee coverage of every start orientation / terrain regardless of the random draws
    orient_cycle = (0, 1, 2, 3, 4, 0)
    sc = Scenario(id=f"{skill}-{split[0]}{i:02d}", skill=skill, split=split, seed=seed,
                  friction_mean=float(rng.choice([0.6, 0.8, 1.0, 1.2])), friction_aniso=float(rng.choice([1.0, 1.0, 1.5, 2.5])),
                  dr_level=float(rng.choice([0.0, 0.3, 0.6])), start_pose_rad=float(rng.choice([0.0, 0.0, 0.6, 1.5])))
    bearing = float(rng.choice([0, 45, 90, 135, 180, -45, -90, -135]))
    if skill == "locomotion":
        sc.start_orient = orient_cycle[i % 6] if i % 3 == 0 else 0
        sc.terrain, sc.terrain_level = ("flat", 0.0) if i % 2 == 0 else ("rolling", float(rng.choice([0.3, 0.6])))
        sc.command = (float(rng.choice([-0.04, 0.0, 0.04, 0.08])), float(rng.choice([-0.04, 0.0, 0.04])), float(rng.choice([-0.5, 0.0, 0.5])))
        if sc.command == (0.0, 0.0, 0.0):
            sc.command = (0.06, 0.0, 0.0)
        sc.goal = (bearing, float(rng.uniform(1.0, 2.5)))
    elif skill == "turning":
        sc.start_orient = 0 if i % 4 else orient_cycle[(i // 4) % 6]
        sc.command = (0.0, 0.0, float(rng.choice([-1.0, -0.5, 0.5, 1.0])))
        sc.goal = (float(rng.choice([45, 90, 135, 180, -45, -90, -135, -180])), 0.0)
        sc.episode_s = 20.0
    elif skill == "self_righting":
        sc.start_orient = (1, 2, 3, 4)[i % 4]
        sc.terrain, sc.terrain_level = ("flat", 0.0) if i % 3 else ("rolling", 0.4)
        sc.start_pose_rad = float(rng.choice([0.0, 0.6, 1.5]))
        sc.command = (0.0, 0.0, 0.0)
        sc.episode_s = 20.0
    elif skill == "terrain":
        sc.terrain = TERRAINS[1 + i % 3]
        sc.terrain_level = float(rng.choice([0.2, 0.4, 0.6, 0.8, 1.0]))
        sc.start_orient = 0 if i % 4 else 1 + (i // 4) % 4
        sc.goal = (bearing, float(rng.uniform(1.5, 2.5)))
        sc.episode_s = 45.0
    elif skill == "obstacle":
        sc.terrain, sc.terrain_level = ("flat", 0.0) if i % 2 == 0 else ("rolling", 0.3)
        sc.obstacle_density = float(rng.choice([0.2, 0.4, 0.6]))
        sc.goal = (bearing, float(rng.uniform(1.5, 2.5)))
        sc.episode_s = 45.0
    elif skill == "pose_hold":
        sc.pose_family = POSE_FAMILIES[i % len(POSE_FAMILIES)]
        sc.start_orient = 0 if i % 3 else 1 + (i // 3) % 4
        sc.dr_level = float(rng.choice([0.0, 0.3]))
        sc.episode_s = 15.0
    return sc


def build_suite(split: str, skill: str) -> list[Scenario]:
    """Deterministically (re)generate one set (independent of any other set and of the body length)."""
    rng = np.random.default_rng([MASTER_SEED, SPLITS.index(split), SKILLS.index(skill)])
    return [_scenario(skill, split, i, rng) for i in range(N_PER_SET)]


def _file(split: str, skill: str, root: Path = SUITE_DIR) -> Path:
    return root / split / f"{skill}.json"


def _sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def write_suite(root: Path = SUITE_DIR, overwrite: bool = False) -> dict[str, Any]:
    """Write all sets and the manifest. Refuses to touch an existing manifest unless ``overwrite`` (= owner-approved change)."""
    man = root / "manifest.json"
    if man.exists() and not overwrite:
        raise FileExistsError(f"{man} exists: the suites are frozen (changing them needs the owner's approval; pass overwrite=True)")
    files: dict[str, str] = {}
    for split in SPLITS:
        for skill in SKILLS:
            p = _file(split, skill, root)
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(json.dumps([asdict(s) for s in build_suite(split, skill)], indent=1, sort_keys=True) + "\n")
            files[str(p.relative_to(root))] = _sha(p)
    seeds = {split: {s.seed for skill in SKILLS for s in build_suite(split, skill)} for split in SPLITS}
    assert not seeds["decision"] & seeds["test"], "decision and test seeds must be disjoint"
    manifest = {"files": files, "master_seed": MASTER_SEED, "n_per_set": N_PER_SET, "created": time.strftime("%Y-%m-%d"),
                "suite_sha256": hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()}
    man.write_text(json.dumps(manifest, indent=1, sort_keys=True) + "\n")
    return manifest


def check_manifest(root: Path = SUITE_DIR) -> list[str]:
    """Return a list of problems (empty = every file matches its recorded SHA-256 and the sets still equal their generator)."""
    man = root / "manifest.json"
    if not man.exists():
        return [f"missing {man}"]
    manifest = json.loads(man.read_text())
    bad = []
    for rel, sha in manifest["files"].items():
        p = root / rel
        if not p.exists():
            bad.append(f"missing {rel}")
        elif _sha(p) != sha:
            bad.append(f"hash mismatch {rel}")
    expected = {f"{s}/{k}.json" for s in SPLITS for k in SKILLS}
    if set(manifest["files"]) != expected:
        bad.append("manifest file list differs from the expected set of files")
    return bad


def load_suite(split: str, skill: str, root: Path = SUITE_DIR, reason: str = "") -> list[Scenario]:
    """Load one set after verifying the manifest. The TEST split needs ``PETCTL_OPEN_TEST=1`` and a non-empty ``reason`` (milestone); every open is logged."""
    problems = check_manifest(root)
    if problems:
        raise RuntimeError("eval suite integrity check failed: " + "; ".join(problems))
    if split == "test":
        if os.environ.get("PETCTL_OPEN_TEST") != "1" or not reason:
            raise PermissionError("the TEST set is opened only at milestones: set PETCTL_OPEN_TEST=1 and pass reason=...")
        with open(root / "test_access.log", "a") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {skill} {reason}\n")
    rows = json.loads(_file(split, skill, root).read_text())
    for r in rows:
        for k in ("command", "goal"):
            if r.get(k) is not None:
                r[k] = tuple(r[k])
    return [Scenario(**r) for r in rows]


def task_params(sc: Scenario) -> dict[str, float]:
    """Env ``task_params`` that realise the parts of ``sc`` the current env understands (goal_reach task); the rest waits for Phase 2 / 7 / 8."""
    p: dict[str, float] = {"start_orient": float(sc.start_orient), "start_pose_rad": float(sc.start_pose_rad), "allow_flip": 1.0}
    if sc.terrain != "flat" and sc.terrain_level > 0.0:
        p["terrain_level"] = float(sc.terrain_level)
        p["terrain_rolling"] = 1.0 if sc.terrain == "rolling" else 0.0
        p["terrain_ridges"] = {"rolling": 0.0, "ridges": 1.0, "dunes": 3.0}.get(sc.terrain, 0.0)
    if sc.goal is not None and sc.goal[1] > 0:
        p["goal_dist_min"] = p["goal_dist_max"] = float(sc.goal[1])
        p["bearing_deg"] = p["bearing_min_deg"] = abs(float(sc.goal[0]))           # goal at exactly this |bearing|, random sign drawn from the scenario seed
    return p
