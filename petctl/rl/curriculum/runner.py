"""Guard-railed stage launcher: the tuning loop changes training only through ``overrides`` that are validated against ``Stage.knobs``."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from petctl.rl.config import DEFAULT_URDF_DIR, EnvConfig
from petctl.rl.curriculum.stages import Stage

ROOT = Path("data/rl/curriculum_fast")    # 0.5 N.m cap, success judged at the HEAD; older data/rl/curriculum*/ runs used other definitions and are not warm-start parents


def stage_dir(stage: Stage) -> Path:
    """Folder holding every iteration of a stage."""
    return ROOT / stage.name


def next_iteration_dir(stage: Stage) -> Path:
    """``data/rl/curriculum/<stage>/iter_NN`` with the next free NN."""
    d = stage_dir(stage)
    n = len(list(d.glob("iter_*"))) + 1 if d.exists() else 1
    return d / f"iter_{n:02d}"


def passing_run(stage_name: str) -> Path | None:
    """Most recent iteration of ``stage_name`` whose gate passed (used to warm-start child stages)."""
    hits = sorted((ROOT / stage_name).glob(f"iter_*/gate_{stage_name}.json")) if (ROOT / stage_name).exists() else []
    for g in reversed(hits):
        if json.loads(g.read_text()).get("passed"):
            return g.parent
    return None


def validate_overrides(stage: Stage, overrides: dict[str, float]) -> None:
    """Reject any override that is not a declared knob of the stage or lies outside its allowed range."""
    for k, v in overrides.items():
        if k not in stage.knobs:
            raise ValueError(f"'{k}' is not a tunable knob of {stage.name} (allowed: {', '.join(stage.knobs)})")
        lo, hi = stage.knobs[k]
        if not lo <= v <= hi:
            raise ValueError(f"{k}={v} outside the allowed range [{lo}, {hi}] for {stage.name}")


def build_config(stage: Stage, overrides: dict[str, float]) -> EnvConfig:
    """EnvConfig for a stage iteration (stage defaults + validated overrides)."""
    validate_overrides(stage, overrides)
    cfg = EnvConfig(task=stage.task, urdf_dir=DEFAULT_URDF_DIR)
    cfg.dr.scale = overrides.get("dr_scale", stage.dr_scale)
    cfg.dr.cap_nm = (stage.cap_nm, stage.cap_nm)
    cfg.episode_s = stage.episode_s
    if stage.action_scale is not None:
        cfg.action_scale_rad = stage.action_scale
    if stage.action_ema is not None:
        cfg.action_ema = stage.action_ema
    cfg.task_params = {**stage.task_params, **{k: v for k, v in overrides.items() if k in stage.task_params or k not in ("ent_coef", "dr_scale", "steps")}}
    return cfg


def launch(stage: Stage, overrides: dict[str, float] | None = None, out: Path | None = None, init_from: Path | None = None, seed: int = 0,
           note: str = "") -> Path:
    """Train one iteration of ``stage`` (blocking) and record ``iteration.json``; returns the run directory."""
    from petctl.rl.train import _git_hash, train

    ov = dict(overrides or {})
    cfg = build_config(stage, ov)
    out = out or next_iteration_dir(stage)
    parent = init_from or (passing_run(stage.parent) if stage.parent else None)
    out.mkdir(parents=True, exist_ok=True)
    info: dict[str, Any] = {"stage": stage.name, "overrides": ov, "init_from": str(parent) if parent else None, "seed": seed, "note": note,
                            "git": _git_hash(), "started": time.strftime("%Y-%m-%d %H:%M:%S")}
    (out / "iteration.json").write_text(json.dumps(info, indent=1))
    train(cfg, int(ov.get("steps", stage.steps)), stage.n_envs, out, seed, "ppo", None, ov.get("ent_coef", stage.ent_coef), parent,
          stop_when_gate_passes=stage)
    return out
