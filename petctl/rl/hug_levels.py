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
