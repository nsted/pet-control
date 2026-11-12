"""Skill library: verified, named, committed policies (``models/skills/<task>/<name>/``) that can be listed and replayed by name.

Each skill folder holds ``policy.npz`` (inference-only weights), ``config.json`` (the EnvConfig it was trained with), ``skill.json``
(task, verified score, conditions, provenance, caveats) and ``demo.mp4``. Listing scans the folders, so there is no index to go stale.
The stored score comes from a fresh deterministic replay on fixed seeds, never from the training record.
"""

from __future__ import annotations

import json
import shutil
import statistics
import subprocess
import time
from pathlib import Path
from typing import Any

from petctl.rl.config import DomainRand

REAL_CAP_NM = 0.5                     # the real per-joint torque cap (N.m), per Nick 2026-11-11; older skills trained at 0.164 were conservative
SKILLS_ROOT = Path("models/skills")
POLICY_FILE = "policy.npz"
VERIFY_SEEDS = tuple(range(5))        # fixed replay seeds
VERIFY_DR_SCALE = 0.3                 # robustness replay uses the training-time randomisation


def skills_root() -> Path:
    """Root folder of the library (relative to the working directory, like data/ and docs/)."""
    return SKILLS_ROOT


def _all() -> list[Path]:
    root = skills_root()
    return sorted(p.parent for p in root.glob("*/*/skill.json")) if root.exists() else []


def resolve(name_or_path: str | Path) -> Path:
    """Folder for a skill id (``jump/jump_20m_v1``), a bare unique skill name, or a plain run directory containing ``config.json``."""
    p = Path(name_or_path)
    if p.is_dir() and (p / "config.json").exists():
        return p
    s = str(name_or_path).strip("/")
    direct = skills_root() / s
    if (direct / "skill.json").exists():
        return direct
    matches = [d for d in _all() if d.name == s]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise FileNotFoundError(f"no skill or run directory '{name_or_path}' (see: petctl rl skills)")
    raise ValueError(f"'{name_or_path}' is ambiguous: " + ", ".join(f"{d.parent.name}/{d.name}" for d in matches))


def _score(run: Path, policy_file: str, seeds: tuple[int, ...], dr_scale: float) -> list[float]:
    from petctl.rl.config import EnvConfig
    from petctl.rl.play import play

    cfg = EnvConfig.from_dict(json.loads((run / "config.json").read_text()))
    return [play(run, cfg.episode_s, seed, dr_scale, False, policy_file)["distance"] for seed in seeds]


def _git_sha() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, timeout=10).stdout.strip()
    except Exception:  # noqa: BLE001 - provenance is best-effort
        return ""


def add_skill(run: Path, name: str, policy_file: str = POLICY_FILE, note: str = "", render: bool = True) -> Path:
    """Verify ``run/policy_file`` by replay, copy it into the library as ``<task>/<name>``, write ``skill.json`` and a demo video."""
    from petctl.rl.config import EnvConfig
    from petctl.rl.tasks import TASKS

    cfg = EnvConfig.from_dict(json.loads((run / "config.json").read_text()))
    dest = skills_root() / cfg.task / name
    if dest.exists():
        raise FileExistsError(f"{dest} already exists; pick another name (skills are never overwritten)")
    nominal = _score(run, policy_file, VERIFY_SEEDS, 0.0)
    robust = _score(run, policy_file, VERIFY_SEEDS, VERIFY_DR_SCALE)
    dest.mkdir(parents=True)
    shutil.copy2(run / policy_file, dest / POLICY_FILE)
    shutil.copy2(run / "config.json", dest / "config.json")
    cap = cfg.dr.cap_nm
    caveats = []
    if max(cap) > REAL_CAP_NM:
        caveats.append(f"trained at a {max(cap):g} N.m torque cap, above the real {REAL_CAP_NM:g} N.m: sim only until retrained at the real cap")
    meta = json.loads((run / "meta.json").read_text()) if (run / "meta.json").exists() else {}
    info: dict[str, Any] = {
        "id": f"{cfg.task}/{name}", "task": cfg.task, "name": name, "note": note, "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "score_label": TASKS[cfg.task].score_label,
        "score": {"nominal_median": statistics.median(nominal), "nominal_best": max(nominal), "robust_median": statistics.median(robust),
                  "seeds": list(VERIFY_SEEDS), "robust_dr_scale": VERIFY_DR_SCALE},
        "conditions": {"torque_cap_nm": list(cap), "action_scale_rad": cfg.action_scale_rad, "action_ema": cfg.action_ema,
                       "max_target_step_rad": cfg.max_target_step_rad, "episode_s": cfg.episode_s, "control_hz": cfg.control_hz},
        "source": {"run": str(run), "policy_file": policy_file, "steps": meta.get("steps"), "wall_s": meta.get("wall_s"), "git": _git_sha()},
        "caveats": caveats,
    }
    (dest / "skill.json").write_text(json.dumps(info, indent=1))
    if render:
        render_demo(dest)
    return dest


def render_demo(skill: Path, seed: int = 0) -> Path:
    """(Re)record ``demo.mp4`` for a skill folder."""
    from petctl.rl.video import record_video

    out = skill / "demo.mp4"
    record_video(skill, out, POLICY_FILE, seed=seed)
    return out


def list_skills(task: str | None = None) -> list[dict[str, Any]]:
    """All skills (optionally one task), best first within each task; ``best`` marks the top verified nominal median per task."""
    rows = [json.loads((d / "skill.json").read_text()) for d in _all()]
    rows = [r for r in rows if task is None or r["task"] == task]
    rows.sort(key=lambda r: (r["task"], -r["score"]["nominal_median"]))
    seen: set[str] = set()
    for r in rows:
        r["best"] = r["task"] not in seen
        seen.add(r["task"])
    return rows
