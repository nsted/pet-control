"""Self-bootstrapping start buffer of the m2_hug6 stage (no scripted start poses).

During training every episode that reaches a state with a hug run >= ``MIN_RUN`` and a normal span >= ``MIN_SPAN_DEG`` stores that state (joint angles, joint targets, the robot's free-body pose, the
object's shape and world position); a bounded FIFO. Each vectorised worker keeps it in memory; the training callback merges the workers' new states once per rollout, persists the file and broadcasts the merged copy (no file IO in the step path). Once the buffer holds ``MIN_STATES``
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
    """Bounded FIFO of start states, held IN MEMORY by each env (no file IO on the step / reset path).

    Each env stores its own new states in its copy (``add``; they are also queued as ``pending``). The training callback (``HugBufferCallback``) collects every env's pending
    states once per rollout (``drain``), merges them into one 500-state FIFO, persists that to the JSON file (``write_file``, atomic) and broadcasts it back (``replace``), so all envs
    sample from the merged copy. The file is read once when the buffer is created (resume). ``flush`` (merge this copy's pending states into the file under a lock) is for single
    process use and tests.
    """

    def __init__(self, path: Path | str, capacity: int = CAPACITY) -> None:
        """``path``: the buffer file (read once, lazily; created by ``write_file`` / ``flush``)."""
        self.path, self.capacity = Path(path), int(capacity)
        self._items: list[dict[str, Any]] | None = None
        self.pending: list[dict[str, Any]] = []

    @classmethod
    def from_env(cls, capacity: int = CAPACITY) -> HugStartBuffer | None:
        """A fresh buffer on the file named by ``$PETCTL_HUG_BUFFER`` or None when unset."""
        p = os.environ.get(BUFFER_ENV)
        return cls(p, capacity) if p else None

    @staticmethod
    def read_file(path: Path) -> list[dict[str, Any]]:
        """The entries stored in ``path`` ([] when missing or unreadable)."""
        try:
            return list(json.loads(path.read_text()))
        except (FileNotFoundError, json.JSONDecodeError):
            return []

    def load(self) -> list[dict[str, Any]]:
        """All entries of this copy, oldest first (the file is read on first use only)."""
        if self._items is None:
            self._items = self.read_file(self.path)[-self.capacity:]
        return self._items

    def __len__(self) -> int:
        """Number of stored states."""
        return len(self.load())

    def add(self, entry: dict[str, Any]) -> None:
        """Append ``entry`` to this copy (the oldest beyond ``capacity`` are dropped, FIFO) and queue it for the callback's merge. No file IO."""
        items = self.load()
        items.append(entry)
        del items[:-self.capacity]
        self.pending.append(entry)

    def drain(self) -> list[dict[str, Any]]:
        """The states added since the last drain (oldest first); clears the queue."""
        out, self.pending = self.pending, []
        return out

    def replace(self, items: list[dict[str, Any]]) -> None:
        """Install the merged buffer broadcast by the callback (everything pending has been drained into it first)."""
        self._items = list(items)[-self.capacity:]
        self.pending = []

    def write_file(self, items: list[dict[str, Any]] | None = None) -> None:
        """Persist ``items`` (default: this copy) to the buffer file atomically (callback / resume; no lock: one writer)."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_name(self.path.name + f".tmp{os.getpid()}")
        tmp.write_text(json.dumps(self.load() if items is None else items))
        os.replace(tmp, self.path)

    def flush(self) -> None:
        """Merge this copy's pending states into the file (lock + atomic replace; single-process use and tests)."""
        pend = self.drain()
        if not pend:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with open(str(self.path) + ".lock", "w") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            try:
                self.write_file((self.read_file(self.path) + pend)[-self.capacity:])
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
