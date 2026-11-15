"""Training-only success curriculum of the hug task (stage ``hug_curr_01``): the level state machine and its persistence.

Levels (``petctl.rl.hug.HUG_LEVELS``): L1 = normal span >= 120 deg, run >= 3, hold 1.5 s; L2 = 150 deg, run >= 4, hold 2 s; L3 = the gate definition (180 deg, run >= 4, hold 3 s, 90 % duty).
The per-tick ``hold`` reward and the ``hug_bonus`` follow the CURRENT level's wrap tick / held definition; the level advances when the rolling success (last ``WINDOW`` finished episodes
that STARTED at the current level, success defined at that level) reaches ``ADVANCE_AT``; it never regresses. Gates and screens always score L3 (their task params do not switch the curriculum on).
The state lives in ``<run dir>/hug_level.json`` (resumable) and is broadcast to the env workers once per rollout by ``petctl.rl.records.HugLevelCallback``.
"""

from __future__ import annotations

import collections
import json
import os
import time
from pathlib import Path
from typing import Any

WINDOW = 100
ADVANCE_AT = 0.30
MAX_LEVEL = 3
LEVEL_FILE = "hug_level.json"
LEVEL_ENV = "PETCTL_HUG_LEVEL"


class LevelState:
    """Current curriculum level plus the rolling success window of that level."""

    def __init__(self, level: int = 1, history: list[dict[str, Any]] | None = None, window: int = WINDOW, advance_at: float = ADVANCE_AT) -> None:
        """``level``: starting level (1-3); ``history``: earlier advances (``{level, timesteps, wall_s, rolling}``)."""
        self.level = min(MAX_LEVEL, max(1, int(level)))
        self.history: list[dict[str, Any]] = list(history or [])
        self.window, self.advance_at = window, advance_at
        self._win: collections.deque[float] = collections.deque(maxlen=window)

    def rolling(self) -> float | None:
        """Mean success of the window (None until it is full)."""
        return float(sum(self._win) / len(self._win)) if len(self._win) == self.window else None

    def add(self, episode_level: int, success: float, timesteps: int = 0, wall_s: float = 0.0) -> bool:
        """Record a finished episode; only episodes that started at the current level count. Returns True if the level advanced (the window restarts)."""
        if int(episode_level) != self.level:
            return False
        self._win.append(float(success))
        r = self.rolling()
        if r is not None and r >= self.advance_at and self.level < MAX_LEVEL:
            self.history.append({"level": self.level + 1, "timesteps": int(timesteps), "wall_s": float(wall_s), "rolling": r})
            self.level += 1
            self._win.clear()
            return True
        return False

    def to_json(self) -> dict[str, Any]:
        """Persisted form."""
        return {"level": self.level, "history": self.history, "rolling": self.rolling(), "n_window": len(self._win), "time": time.strftime("%Y-%m-%d %H:%M:%S")}

    def save(self, path: Path) -> None:
        """Atomic write to ``path``."""
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.to_json(), indent=1))
        os.replace(tmp, path)

    @classmethod
    def load(cls, path: Path, default_level: int = 1) -> LevelState:
        """State from ``path`` (``default_level`` and no history if absent / unreadable)."""
        try:
            d = json.loads(path.read_text())
            return cls(int(d["level"]), d.get("history", []))
        except (OSError, ValueError, KeyError):
            return cls(default_level)


def read_level(run_dir: Path) -> int:
    """The persisted level of a run directory (1 if none)."""
    return LevelState.load(run_dir / LEVEL_FILE).level


# ---------------------------------------------------------------- per-family curriculum (stage ``hug_curr_02``, task knob ``hug_curriculum`` = 2)
# Level specs (normal span deg, contiguous run, hold seconds). ``l1``/``l2``/``l3`` are the hug_curr_01 levels (l3 = the frozen gate definition); ``c0``/``c1`` are the extra curved-family steps.
LEVEL_SPECS: dict[str, tuple[float, int, float]] = {"c0": (100.0, 4, 1.0), "c1": (125.0, 4, 1.5), "l1": (120.0, 3, 1.5), "l2": (150.0, 4, 2.0), "l3": (180.0, 4, 3.0)}
FAMILIES = ("cyl", "box", "rbox")                                  # order of the ``shape_kind`` metric (``petctl.rl.tasks.goal_hug.SHAPE_KINDS``)
LADDERS: dict[str, tuple[str, ...]] = {"cyl": ("c0", "c1", "l2", "l3"), "box": ("l1", "l2", "l3"), "rbox": ("c0", "c1", "l2", "l3")}


def family_level_name(family: str, idx: int) -> str:
    """Name (``c0`` .. ``l3``) of ladder step ``idx`` (clamped) of ``family``."""
    lad = LADDERS[family]
    return lad[min(len(lad) - 1, max(0, int(idx)))]


class FamilyLevelState:
    """Per shape-family curriculum levels (index into ``LADDERS[family]``) with one rolling-success window per family. A family advances when the success of its last ``window`` episodes
    that STARTED at its current level reaches ``advance_at``; it never regresses and families are independent."""

    def __init__(self, levels: dict[str, int] | None = None, history: list[dict[str, Any]] | None = None, window: int = WINDOW, advance_at: float = ADVANCE_AT) -> None:
        """``levels``: starting ladder index per family (default 0); ``history``: earlier advances."""
        self.levels = {f: min(len(LADDERS[f]) - 1, max(0, int((levels or {}).get(f, 0)))) for f in FAMILIES}
        self.history: list[dict[str, Any]] = list(history or [])
        self.window, self.advance_at = window, advance_at
        self._win: dict[str, collections.deque[float]] = {f: collections.deque(maxlen=window) for f in FAMILIES}

    def rolling(self, family: str) -> float | None:
        """Mean success of the family's window (None until it is full)."""
        w = self._win[family]
        return float(sum(w) / len(w)) if len(w) == self.window else None

    def add(self, family: str, episode_level: int, success: float, timesteps: int = 0, wall_s: float = 0.0) -> bool:
        """Record a finished episode of ``family``; only episodes that started at the family's current level count. Returns True if the family advanced (its window restarts)."""
        if family not in self.levels or int(episode_level) != self.levels[family]:
            return False
        self._win[family].append(float(success))
        r = self.rolling(family)
        if r is not None and r >= self.advance_at and self.levels[family] < len(LADDERS[family]) - 1:
            self.levels[family] += 1
            self.history.append({"family": family, "level": LADDERS[family][self.levels[family]], "timesteps": int(timesteps), "wall_s": float(wall_s), "rolling": r})
            self._win[family].clear()
            return True
        return False

    def to_json(self) -> dict[str, Any]:
        """Persisted form."""
        return {"levels": self.levels, "names": {f: family_level_name(f, i) for f, i in self.levels.items()}, "history": self.history, "rolling": {f: self.rolling(f) for f in FAMILIES},
                "n_window": {f: len(self._win[f]) for f in FAMILIES}, "time": time.strftime("%Y-%m-%d %H:%M:%S")}

    def save(self, path: Path) -> None:
        """Atomic write to ``path``."""
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.to_json(), indent=1))
        os.replace(tmp, path)

    @classmethod
    def load(cls, path: Path) -> FamilyLevelState:
        """State from ``path`` (all families at ladder step 0 if absent / unreadable)."""
        try:
            d = json.loads(path.read_text())
            return cls(d["levels"], d.get("history", []))
        except (OSError, ValueError, KeyError, TypeError):
            return cls()


def read_family_levels(run_dir: Path) -> dict[str, int]:
    """The persisted per-family ladder indices of a run directory (all 0 if none)."""
    return FamilyLevelState.load(run_dir / LEVEL_FILE).levels


def level_caption(run_dir: Path) -> str:
    """Factual per-family level text of a run directory for captions (``""`` when the run has no per-family curriculum state)."""
    try:
        d = json.loads((run_dir / LEVEL_FILE).read_text())
        lv = d["levels"]
        return "levels " + " / ".join(f"{f} {family_level_name(f, lv[f]).upper()}" for f in FAMILIES)
    except (OSError, ValueError, KeyError, TypeError):
        return ""
