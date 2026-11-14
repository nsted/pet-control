"""Helpers of the M2 v2 driver (``scripts/rl/m2v2_driver.py``): takeoff rule, seed bookkeeping over the summary lines, and the per-run note (return curve, value-loss summary).

Takeoff rule (expert order): a flat run whose rolling training success (last 200 episodes) is below 0.10 at 0.6M steps is killed and logged as a takeoff failure
(summary status ``takeoff_fail``); the driver then restarts with the next seed.
"""

from __future__ import annotations

import json
import math
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

TAKEOFF_STEPS = 600_000
TAKEOFF_WINDOW = 200
TAKEOFF_MIN_RATE = 0.10
MAX_SEED = 5                      # a (L, task) never tries seeds beyond this, whatever the takeoff failures


def takeoff_failed(successes: list[float] | deque[float], steps: int, check_steps: int = TAKEOFF_STEPS, window: int = TAKEOFF_WINDOW, min_rate: float = TAKEOFF_MIN_RATE) -> bool | None:
    """None before ``check_steps``; then True iff the mean of the last ``window`` training successes (all of them if fewer; no episodes = 0) is below ``min_rate``."""
    if steps < check_steps:
        return None
    last = list(successes)[-window:]
    return (sum(last) / len(last) if last else 0.0) < min_rate


class TakeoffCallback(BaseCallback):
    """Stops training once, at ``check_steps``, if the rolling training success is below ``min_rate``; ``failed`` / ``rate`` / ``checked_at`` record the decision."""

    def __init__(self, check_steps: int = TAKEOFF_STEPS, window: int = TAKEOFF_WINDOW, min_rate: float = TAKEOFF_MIN_RATE) -> None:
        """Defaults are the expert's rule (0.10 at 0.6M steps over 200 episodes)."""
        super().__init__()
        self.check_steps, self.window, self.min_rate = check_steps, window, min_rate
        self._succ: deque[float] = deque(maxlen=window)
        self.failed = False
        self.rate: float | None = None
        self.checked_at: int | None = None

    def _on_step(self) -> bool:
        for done, info in zip(self.locals["dones"], self.locals["infos"]):
            if done and "metrics" in info:
                self._succ.append(float(info["metrics"].get("success", 0.0)))
        if self.checked_at is None:
            verdict = takeoff_failed(self._succ, int(self.num_timesteps), self.check_steps, self.window, self.min_rate)
            if verdict is not None:
                self.checked_at = int(self.num_timesteps)
                self.rate = float(np.mean(list(self._succ))) if self._succ else 0.0
                self.failed = bool(verdict)
                print(f"[takeoff] rolling success {self.rate:.3f} over {len(self._succ)} episodes at {self.checked_at} steps: {'FAIL (stop)' if self.failed else 'ok'}", flush=True)
                return not self.failed
        return True


def summary_lines(root: Path) -> list[dict[str, Any]]:
    """Every line of ``root/summary.jsonl``."""
    f = root / "summary.jsonl"
    return [json.loads(x) for x in f.read_text().splitlines() if x.strip()] if f.exists() else []


def counted(lines: list[dict[str, Any]], L: int, task: str) -> list[dict[str, Any]]:
    """The runs of (L, task) that count towards the seed quota: everything except ``takeoff_fail`` (a failed takeoff is retried with the next seed)."""
    return [d for d in lines if d["L"] == L and d["task"] == task and d.get("status") != "takeoff_fail"]


def next_seed(lines: list[dict[str, Any]], L: int, task: str, first_seed: int, need: int, max_seed: int = MAX_SEED) -> int | None:
    """The next seed to run for (L, task) so that ``need`` runs that did not fail takeoff exist, or None when the quota is met (or ``max_seed`` is exhausted).

    Seeds already in the summary (any status) are skipped; seeds are tried in increasing order from ``first_seed``.
    """
    if len(counted(lines, L, task)) >= need:
        return None
    used = {int(d["seed"]) for d in lines if d["L"] == L and d["task"] == task}
    for s in range(first_seed, max_seed + 1):
        if s not in used:
            return s
    return None


def best_seed(lines: list[dict[str, Any]], L: int, task: str) -> int | None:
    """Seed of the run with the highest gate success among the finished (gated) runs of (L, task); ties go to the lower median time to goal. None if there is none."""
    runs = [d for d in counted(lines, L, task) if d.get("success") is not None]
    if not runs:
        return None
    return int(max(runs, key=lambda d: (d["success"], -(d.get("t_goal_median_s") or 1e9)))["seed"])


def _tb_scalars(run: Path, tags: tuple[str, ...]) -> dict[str, list[tuple[int, float]]]:
    """Scalar series of the run's tensorboard events (empty if tensorboard is unavailable or there are none)."""
    out: dict[str, list[tuple[int, float]]] = {t: [] for t in tags}
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    except Exception:  # noqa: BLE001
        return out
    for d in sorted((run / "tb").glob("*")) if (run / "tb").exists() else []:
        try:
            acc = EventAccumulator(str(d), size_guidance={"scalars": 0})
            acc.Reload()
            for t in tags:
                if t in acc.Tags().get("scalars", []):
                    out[t].extend((int(e.step), float(e.value)) for e in acc.Scalars(t))
        except Exception:  # noqa: BLE001
            continue
    return out


def run_note(run: Path, phase_dirs: list[Path] | None = None, buckets: int = 10) -> str:
    """A short text note of a finished run: the training-episode return curve (mean per bucket of ``buckets`` equal step ranges), rolling success, per-term means of the last
    tenth of the run (``term_*`` metrics), success by start mode (hug), and the value-loss / explained-variance summary from the tensorboard logs, to rule out a critic bug.

    ``phase_dirs``: the directories whose ``episodes.jsonl`` / ``tb`` make up the run (default: ``run`` itself); steps are offset by the preceding phases.
    """
    dirs = phase_dirs or [run]
    eps: list[dict[str, Any]] = []
    vl: list[tuple[int, float]] = []
    ev: list[tuple[int, float]] = []
    off = 0
    for d in dirs:
        f = d / "episodes.jsonl"
        rows = [json.loads(x) for x in f.read_text().splitlines() if x.strip()] if f.exists() else []
        for r in rows:
            r["_t"] = int(r["timesteps"]) + off
        eps.extend(rows)
        sc = _tb_scalars(d, ("train/value_loss", "train/explained_variance"))
        vl.extend((s + off, v) for s, v in sc["train/value_loss"])
        ev.extend((s + off, v) for s, v in sc["train/explained_variance"])
        off = max([r["_t"] for r in rows] + [off])
    lines = [f"note {run}: {len(eps)} training episodes"]
    if eps:
        t = np.array([r["_t"] for r in eps], float)
        ret = np.array([r.get("return", float("nan")) for r in eps], float)
        suc = np.array([r.get("success", 0.0) for r in eps], float)
        edges = np.linspace(0, t.max() + 1, buckets + 1)
        curve = []
        for a, b in zip(edges[:-1], edges[1:]):
            m = (t >= a) & (t < b)
            curve.append("n/a" if not m.any() else f"{np.nanmean(ret[m]):.0f}/{suc[m].mean():.2f}")
        lines.append("return/success per tenth of the run: " + " ".join(curve))
        tail = t >= edges[-2]
        terms = sorted({k for r in eps for k in r if k.startswith("term_")})
        if terms:
            lines.append("mean per-episode term sums, last tenth: " + " ".join(f"{k[5:]}={np.mean([r.get(k, 0.0) for r, m in zip(eps, tail) if m]):.1f}" for k in terms if abs(np.mean([r.get(k, 0.0) for r, m in zip(eps, tail) if m])) >= 0.05))
        if any("start_mode" in r for r in eps):
            names = ("straight", "coil", "half")
            lines.append("success by start mode, last tenth: " + " ".join(f"{names[k]}={np.mean([r['success'] for r, m in zip(eps, tail) if m and int(r.get('start_mode', 0)) == k] or [float('nan')]):.2f}" for k in range(3)))
    if vl:
        v = np.array([x for _, x in vl])
        q = max(1, len(v) // 10)
        lines.append(f"value_loss: first {np.mean(v[:q]):.2f}, median {np.median(v):.2f}, max {v.max():.2f}, last tenth {np.mean(v[-q:]):.2f} ({len(v)} updates)")
    if ev:
        e = np.array([x for _, x in ev])
        q = max(1, len(e) // 10)
        lines.append(f"explained_variance (critic): first {np.mean(e[:q]):.2f}, median {np.median(e):.2f}, last tenth {np.mean(e[-q:]):.2f}" + ("  <- near 0 / negative: critic not learning" if np.mean(e[-q:]) < 0.1 else ""))
    if not vl and not ev:
        lines.append("no tensorboard scalars found")
    return "\n".join(lines)


def finite(x: Any) -> bool:
    """True for a finite real number."""
    return isinstance(x, (int, float)) and math.isfinite(x)
