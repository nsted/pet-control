"""On-disk cache of obstacle layouts (Regimen v2 plan, section 5.2): a training reset draws a stored layout instead of re-running the sampler and the planner.

A cache is keyed by the task, the layout-relevant task parameters, the body length and the nominal head pose, plus a ``layout_block`` id (task knob).
Layout ``i`` of a cache is generated once, from a generator seeded by ``(key, block, i)``, and stored as ``<i>.npz`` under ``data/rl/layout_cache/<name>/``
(git-ignored; override the root with ``PETCTL_LAYOUT_CACHE``). Generation is lazy and atomic, so parallel environments can share a cache.

Enabled by the task knob ``layout_cache = N`` (N layouts; 0 = off, the default). Gate / evaluation seeds (>= ``GATE_SEED_MIN``) never use the cache:
their layout is always drawn from the seed, so gate numbers do not depend on what is on disk.
"""

from __future__ import annotations

import hashlib
import json
import os
import tempfile
from pathlib import Path
from typing import Callable

import numpy as np

from petctl.rl.config import EnvConfig

GATE_SEED_MIN = 10000
CACHE_VERSION = 1
LAYOUT_KEYS = ("field_radius", "goal_dist_min", "goal_dist_max", "bearing_deg", "n_obs_min", "n_obs_max", "spread", "n_block", "barrier_halfwidth",
               "barrier_gap", "n_extra", "min_gap", "n_corridor", "min_on_line", "line_band", "max_detour", "post")     # the task knobs that layout() reads


def cache_root() -> Path:
    """Cache directory root: ``$PETCTL_LAYOUT_CACHE`` or ``<repo>/data/rl/layout_cache``."""
    env = os.environ.get("PETCTL_LAYOUT_CACHE")
    return Path(env) if env else Path(__file__).resolve().parents[2] / "data" / "rl" / "layout_cache"


def uses_cache(seed: int | None) -> bool:
    """False for gate/evaluation seeds (never cached); True for training resets (no seed or a seed below ``GATE_SEED_MIN``)."""
    return seed is None or int(seed) < GATE_SEED_MIN


class LayoutCache:
    """The cache of ``n`` layouts for one (task, layout parameters, body, head pose, block)."""

    def __init__(self, cfg: EnvConfig, head: np.ndarray, heading: np.ndarray) -> None:
        """Key the cache from the config and the nominal head pose."""
        tp = cfg.task_params
        self.n = int(tp.get("layout_cache", 0))
        self.block = int(tp.get("layout_block", 0))
        ident = {"v": CACHE_VERSION, "task": cfg.task, "n_modules": int(cfg.n_modules), "head": [round(float(v), 4) for v in head],
                 "heading": [round(float(v), 4) for v in heading], "params": {k: float(tp[k]) for k in LAYOUT_KEYS if k in tp}}
        self.key = hashlib.sha1(json.dumps(ident, sort_keys=True).encode()).hexdigest()[:12]
        self.dir = cache_root() / f"{cfg.task}_{self.key}_b{self.block}"

    def rng(self, idx: int) -> np.random.Generator:
        """The generator that produces layout ``idx`` (deterministic in key, block and index)."""
        return np.random.default_rng([int(self.key, 16) % (2 ** 32), self.block, int(idx)])

    def path(self, idx: int) -> Path:
        """File of layout ``idx``."""
        return self.dir / f"{idx}.npz"

    def get(self, idx: int, make: Callable[[np.random.Generator], tuple[dict[str, object], np.ndarray]]) -> tuple[dict[str, object], np.ndarray]:
        """Return ``(state, goal)`` of layout ``idx``, generating it with ``make(rng)`` (and storing it) on a miss."""
        f = self.path(idx)
        if f.exists():
            try:
                with np.load(f, allow_pickle=False) as z:
                    st: dict[str, object] = {k: z[k] for k in z.files if k not in ("obstacles", "goal")}
                    st["obstacles"] = json.loads(str(z["obstacles"]))
                    return st, z["goal"]
            except (OSError, ValueError, KeyError):
                pass                                                  # partial / corrupt file: regenerate below
        state, goal = make(self.rng(idx))
        self.dir.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=self.dir, suffix=".npz")
        os.close(fd)
        arrays = {k: np.asarray(v) for k, v in state.items() if k != "obstacles"}
        np.savez(tmp, obstacles=np.array(json.dumps(state["obstacles"])), goal=np.asarray(goal, float), **arrays)
        os.replace(tmp, f)
        return state, np.asarray(goal, float)

    def prebuild(self, make: Callable[[np.random.Generator], tuple[dict[str, object], np.ndarray]]) -> int:
        """Generate every missing layout; returns how many were created."""
        made = 0
        for i in range(self.n):
            if not self.path(i).exists():
                self.get(i, make)
                made += 1
        return made
