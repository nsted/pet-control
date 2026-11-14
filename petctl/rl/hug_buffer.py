"""Self-bootstrapping start buffer of the m2_hug6 stage (no scripted start poses).

During training every episode that reaches a state with a hug run >= ``MIN_RUN`` and a normal span >= ``MIN_SPAN_DEG`` stores that state (joint angles, joint targets, the robot's free-body pose, the
object's shape and world position); a bounded FIFO shared by all vectorised workers through a file (atomic ``os.replace`` writes under an ``fcntl`` lock). Once the buffer holds ``MIN_STATES``
states, a fraction ``P_BUFFER`` of the episode starts is drawn from it (joints and targets perturbed by +-``PERTURB_RAD``). Nothing here is scripted: the poses are ones the policy itself found.

The file is named by ``$PETCTL_HUG_BUFFER`` (set by the driver before the workers start); without it the feature is off whatever the task knobs say.
"""

from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path
from typing import Any

import numpy as np

BUFFER_ENV = "PETCTL_HUG_BUFFER"
CAPACITY = 500
MIN_STATES = 20
P_BUFFER = 0.30
MIN_RUN = 3
MIN_SPAN_DEG = 90.0
PERTURB_RAD = 0.10


class HugStartBuffer:
    """Bounded FIFO of start states in a JSON file (list of entries). Safe for concurrent writers (lock + atomic replace) and lock-free readers."""

    def __init__(self, path: Path | str, capacity: int = CAPACITY) -> None:
        """``path``: the buffer file (created on the first ``add``)."""
        self.path, self.capacity = Path(path), int(capacity)
        self._cache: tuple[tuple[int, int], list[dict[str, Any]]] | None = None

    @classmethod
    def from_env(cls, capacity: int = CAPACITY) -> HugStartBuffer | None:
        """The buffer named by ``$PETCTL_HUG_BUFFER`` or None when unset."""
        p = os.environ.get(BUFFER_ENV)
        return cls(p, capacity) if p else None

    def load(self) -> list[dict[str, Any]]:
        """All stored entries, oldest first (cached by file mtime and size; [] when there is no file)."""
        try:
            st = self.path.stat()
        except FileNotFoundError:
            return []
        key = (st.st_mtime_ns, st.st_size)
        if self._cache is None or self._cache[0] != key:
            try:
                self._cache = (key, json.loads(self.path.read_text()))
            except (json.JSONDecodeError, FileNotFoundError):
                return self._cache[1] if self._cache else []
        return self._cache[1]

    def __len__(self) -> int:
        """Number of stored states."""
        return len(self.load())

    def add(self, entry: dict[str, Any]) -> None:
        """Append ``entry``; the oldest entries beyond ``capacity`` are dropped (FIFO)."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with open(str(self.path) + ".lock", "w") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            try:
                try:
                    cur = json.loads(self.path.read_text())
                except (FileNotFoundError, json.JSONDecodeError):
                    cur = []
                cur = (cur + [entry])[-self.capacity:]
                tmp = self.path.with_name(self.path.name + f".tmp{os.getpid()}")
                tmp.write_text(json.dumps(cur))
                os.replace(tmp, self.path)
            finally:
                fcntl.flock(lock, fcntl.LOCK_UN)

    def sample(self, rng: np.random.Generator, p: float = P_BUFFER, min_states: int = MIN_STATES) -> dict[str, Any] | None:
        """With probability ``p`` (only once the buffer holds >= ``min_states`` states) a uniformly drawn entry, else None. One uniform and one index are always drawn so the stream does not depend on the fill."""
        u, k = float(rng.random()), float(rng.random())
        items = self.load()
        if len(items) < min_states or u >= p:
            return None
        return items[min(int(k * len(items)), len(items) - 1)]


def make_entry(q: np.ndarray, tgt: np.ndarray, qpos7: np.ndarray, post_xy: tuple[float, float], shape: tuple[Any, ...], run: int, span_deg: float) -> dict[str, Any]:
    """A buffer entry: joint angles / targets (rad), free-body pose (x y z qw qx qy qz), the object's world xy and ``(kind, a, b, rc, yaw)``, and the run / span it had."""
    return {"q": [float(x) for x in q], "tgt": [float(x) for x in tgt], "qpos7": [float(x) for x in qpos7], "post": [float(post_xy[0]), float(post_xy[1])],
            "shape": [shape[0], *(float(x) for x in shape[1:])], "run": int(run), "span": float(span_deg)}
