"""Cheap latency / jitter statistics for the driver ↔ brain bus (PROCESS_SPLIT Stage 6)."""

from __future__ import annotations

from collections import deque
from typing import Optional

import numpy as np


class SampleStats:
    """Rolling sample buffer with percentile summaries. Values are seconds; reported in ms."""

    def __init__(self, maxlen: int = 50_000) -> None:
        self._v: deque[float] = deque(maxlen=maxlen)
        self._last_arrival: Optional[float] = None

    def add(self, seconds: float) -> None:
        """Record one value (e.g. a one-way latency)."""
        self._v.append(seconds)

    def add_arrival(self, now: float) -> None:
        """Record the gap since the previous arrival (for inter-arrival jitter)."""
        if self._last_arrival is not None:
            self._v.append(now - self._last_arrival)
        self._last_arrival = now

    def __len__(self) -> int:
        return len(self._v)

    def summary_ms(self) -> Optional[dict[str, float]]:
        """n, mean, std, p50, p95, p99, max in milliseconds; None if empty."""
        if not self._v:
            return None
        a = np.fromiter(self._v, dtype=float) * 1000.0
        return {
            "n": float(len(a)),
            "mean": float(a.mean()),
            "std": float(a.std()),
            "p50": float(np.percentile(a, 50)),
            "p95": float(np.percentile(a, 95)),
            "p99": float(np.percentile(a, 99)),
            "max": float(a.max()),
        }

    def format(self, label: str) -> str:
        s = self.summary_ms()
        if s is None:
            return f"{label}: no samples"
        return (
            f"{label}: n={int(s['n'])} mean={s['mean']:.2f}ms std={s['std']:.2f}ms "
            f"p50={s['p50']:.2f} p95={s['p95']:.2f} p99={s['p99']:.2f} max={s['max']:.2f}ms"
        )
