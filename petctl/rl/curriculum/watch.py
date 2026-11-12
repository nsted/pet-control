"""Deterministic run monitor: reads ``episodes.jsonl``/``status.json`` and returns a verdict the tuning loop acts on. No LLM involved.

States: ``warming_up`` (too few episodes), ``improving`` (leave it running), ``passing`` (rolling score at the gate level -> run the gate),
``plateau`` (no gain over several windows), ``collapse`` (score fell well below its peak), ``glitch`` (physics blow-ups), ``stalled`` (no
fresh status while unfinished), ``finished`` (training ended). Everything except ``warming_up``/``improving`` is actionable.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

ACTIONABLE = {"passing", "plateau", "collapse", "glitch", "stalled", "finished"}


@dataclass
class Verdict:
    """Monitor output (also written to ``<run>/verdict.json``)."""

    state: str
    timesteps: int = 0
    episodes: int = 0
    window: int = 0
    score_label: str = ""
    scores: list[float] = field(default_factory=list)      # rolling-window score series, oldest first
    last_score: float = float("nan")
    peak_score: float = float("nan")
    glitch_rate: float = 0.0
    reward_return: float = float("nan")
    note: str = ""


def _read_episodes(run: Path) -> list[dict[str, Any]]:
    f = run / "episodes.jsonl"
    rows: list[dict[str, Any]] = []
    if f.exists():
        for line in f.read_text().splitlines():
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                pass                                         # partially written last line
    return rows


def analyze(run: Path, window: int = 100, pass_score: float | None = None, plateau_windows: int = 4, plateau_gain: float = 0.02,
            stall_s: float = 900.0, glitch_tol: float = 0.02) -> Verdict:
    """Classify the run. Score = success rate when episodes carry ``success``, else median ``distance`` (m)."""
    rows = _read_episodes(run)
    status: dict[str, Any] = {}
    if (run / "status.json").exists():
        try:
            status = json.loads((run / "status.json").read_text())
        except json.JSONDecodeError:
            pass
    v = Verdict("warming_up", int(status.get("timesteps", rows[-1]["timesteps"] if rows else 0)), len(rows), window)
    if len(rows) < window:
        v.state = "finished" if status.get("final") else "warming_up"
        v.note = f"{len(rows)}/{window} episodes"
        return v
    use_success = "success" in rows[0]
    v.score_label = "success_rate" if use_success else "distance_median_m"
    chunks = [rows[i:i + window] for i in range(len(rows) - window, -1, -window)][::-1][-8:]      # last <=8 full windows, oldest first
    key = "success" if use_success else "distance"
    v.scores = [float(np.mean([r[key] for r in c]) if use_success else np.median([r[key] for r in c])) for c in chunks]
    v.last_score, v.peak_score = v.scores[-1], max(v.scores)
    last = chunks[-1]
    v.glitch_rate = float(np.mean([r.get("glitch", 0.0) for r in last]))
    rets = [r["return"] for r in last if "return" in r and math.isfinite(r["return"])]
    v.reward_return = float(np.mean(rets)) if rets else float("nan")
    if v.glitch_rate > glitch_tol:
        v.state, v.note = "glitch", f"{100 * v.glitch_rate:.1f}% of recent episodes tripped the glitch guard"
    elif pass_score is not None and v.last_score >= pass_score:
        v.state, v.note = "passing", f"rolling {v.score_label} {v.last_score:.2f} >= {pass_score:.2f}: run the gate"
    elif status.get("final"):
        v.state = "finished"
    elif v.peak_score > 0.1 and v.last_score < 0.5 * v.peak_score and len(v.scores) >= 3:
        v.state, v.note = "collapse", f"score {v.last_score:.2f} fell below half of peak {v.peak_score:.2f}"
    elif len(v.scores) > plateau_windows and max(v.scores[-plateau_windows:]) - max(v.scores[:-plateau_windows]) < plateau_gain * max(abs(v.peak_score), 0.1):
        v.state, v.note = "plateau", f"no gain over the last {plateau_windows} windows"
    elif status.get("updated") and time.time() - float(status["updated"]) > stall_s and not status.get("final"):
        v.state, v.note = "stalled", "status.json not refreshed"
    else:
        v.state = "improving"
    return v


def watch(run: Path, pass_score: float | None = None, window: int = 100, until_actionable: bool = False, poll_s: float = 60.0,
          max_wait_s: float = 6 * 3600) -> Verdict:
    """Analyse once, or poll until the verdict is actionable (or ``max_wait_s``); always writes ``verdict.json``."""
    t0 = time.time()
    while True:
        v = analyze(run, window, pass_score)
        (run / "verdict.json").write_text(json.dumps(asdict(v), indent=1))
        if not until_actionable or v.state in ACTIONABLE or time.time() - t0 > max_wait_s:
            return v
        time.sleep(poll_s)
