"""Training watchdog (docs/plans/AUTONOMOUS_TRAINER.md, Phase 1): reward-term logging plus automatic stop rules.

``WatchdogCallback`` records, per rollout, every reward term's mean and share of the total to TensorBoard (``terms/<name>_mean``, ``terms/<name>_share``)
and the orientation diagnostics (``diag/*``). Every ``check_s`` seconds it summarises the last ``window`` episodes into a snapshot and calls ``judge``; a
verdict stops the run and writes ``STOP_watchdog`` (with the reason) into the run directory. No verdict is issued before ``min_steps`` / ``min_episodes``.

The task metric is the training-episode ``success`` (or ``distance``) unless ``decision_metric.jsonl`` (rows ``{"timesteps", "value"}``, written by a
decision-set evaluator) exists, which then takes precedence: the decision set is the authority, the training metric a proxy.
"""

from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback


@dataclass
class WatchdogConfig:
    """Thresholds (all per-run overridable; defaults are deliberately conservative: false stops cost a pilot, missed hacks cost a full run)."""

    window: int = 200                    # episodes per snapshot
    check_s: float = 180.0               # wall seconds between checks (2-5 min)
    min_steps: int = 300_000             # no verdict before this many steps ...
    min_episodes: int = 400              # ... and this many finished episodes
    trend_snaps: int = 4                 # snapshots compared for the reward / metric trend rules
    return_gain: float = 0.10            # "reward rises": relative gain of the mean return over the trend window
    metric_gain: float = 0.02            # "task metric flat": absolute gain below this (success in [0,1]; distance in m scaled by ``metric_scale``)
    metric_scale: float = 1.0            # scale of the metric (1.0 for success; set to a typical distance for distance metrics)
    share_jump: float = 0.30             # a term's share of total |reward| may move at most this from its baseline share
    share_floor: float = 0.05            # ... and a term must hold at least this share (now or at baseline) to count
    no_progress_steps: int = 1_500_000   # after this many steps the metric must exceed ``min_metric``
    min_metric: float = 0.02
    len_collapse: float = 0.5            # mean episode length below this fraction of its best snapshot
    flip_rise_abs: float = 0.2           # flips per episode rise ...
    flip_rise_rel: float = 0.5           # ... and relative rise, together with a falling metric
    sat_frac: float = 0.60               # fraction of action entries at |a| >= sat_level (bounded action spaces only)
    sat_level: float = 0.98
    std_min: float = 0.05                # policy std bounds, per action unit (std is the action's own unit: rad of target for unbounded, [-1,1] units for clipped)
    std_max: float = 2.0


@dataclass
class Snapshot:
    """Summary of the last ``window`` episodes at one check."""

    steps: int
    episodes: int
    ret: float
    metric: float
    ep_len: float
    flips: float
    inverted_s: float
    shares: dict[str, float] = field(default_factory=dict)
    means: dict[str, float] = field(default_factory=dict)
    sat: float = 0.0
    std_max: float = 0.0
    std_min: float = 0.0
    nan: bool = False


def term_shares(sums: dict[str, float]) -> dict[str, float]:
    """Share of each reward term in the total absolute reward (shares sum to 1; empty dict if all zero)."""
    tot = sum(abs(v) for v in sums.values())
    return {k: abs(v) / tot for k, v in sums.items()} if tot > 0 else {}


def judge(snaps: list[Snapshot], base_shares: dict[str, float], cfg: WatchdogConfig) -> str | None:
    """Return the reason to stop the run, or None. ``snaps`` oldest first; ``base_shares`` the pilot (or first-window) term shares."""
    if not snaps:
        return None
    s = snaps[-1]
    if s.nan or not all(math.isfinite(x) for x in (s.ret, s.metric)):
        return "NaN / non-finite return or metric"
    if s.std_max > cfg.std_max or (0.0 < s.std_min < cfg.std_min):
        return f"policy std left bounds [{cfg.std_min}, {cfg.std_max}]: min {s.std_min:.3f} max {s.std_max:.3f}"
    if s.sat > cfg.sat_frac:
        return f"actions saturate: {100 * s.sat:.0f}% of entries at |a| >= {cfg.sat_level}"
    if s.steps < cfg.min_steps or s.episodes < cfg.min_episodes:
        return None                                                      # minimum window before any other verdict
    if s.steps >= cfg.no_progress_steps and s.metric < cfg.min_metric * cfg.metric_scale and max(x.metric for x in snaps) < cfg.min_metric * cfg.metric_scale:
        return f"no task progress after {s.steps} steps (metric {s.metric:.3f})"
    best_len = max(x.ep_len for x in snaps)
    if len(snaps) >= 3 and best_len > 0 and s.ep_len < cfg.len_collapse * best_len:
        return f"episode length collapsed: {s.ep_len:.0f} vs best {best_len:.0f}"
    if base_shares:
        for k, base in base_shares.items():
            now = s.shares.get(k, 0.0)
            if abs(now - base) > cfg.share_jump and max(now, base) >= cfg.share_floor:
                return f"reward term '{k}' share moved {base:.2f} -> {now:.2f} (limit +-{cfg.share_jump})"
        for k, now in s.shares.items():
            if k not in base_shares and now > cfg.share_jump:
                return f"new reward term '{k}' holds share {now:.2f}"
    if len(snaps) >= cfg.trend_snaps:
        w = snaps[-cfg.trend_snaps:]
        r0, r1 = w[0].ret, w[-1].ret
        ret_up = (r1 - r0) > cfg.return_gain * max(abs(r0), 1e-6) and r1 > r0
        m_gain = w[-1].metric - w[0].metric
        metric_flat = m_gain < cfg.metric_gain * cfg.metric_scale
        if ret_up and metric_flat:
            return f"reward-hack signature: return {r0:.1f} -> {r1:.1f} while task metric {w[0].metric:.3f} -> {w[-1].metric:.3f}"
        f0, f1 = w[0].flips, w[-1].flips
        if f1 - f0 > cfg.flip_rise_abs and f1 > (1 + cfg.flip_rise_rel) * max(f0, 1e-6):
            if m_gain < 0:
                return f"flip rate rose {f0:.2f} -> {f1:.2f} while the task metric fell {w[0].metric:.3f} -> {w[-1].metric:.3f}"
            if ret_up and metric_flat:
                return f"flips raise reward without progress: flips {f0:.2f} -> {f1:.2f}"
    return None


class WatchdogCallback(BaseCallback):
    """Log reward terms / diagnostics and stop the run on a ``judge`` verdict (see module docstring)."""

    def __init__(self, out: Path, cfg: WatchdogConfig | None = None, baseline: Path | None = None) -> None:
        """``out``: run directory; ``baseline``: a pilot run directory whose ``reward_terms.json`` supplies the baseline term shares (else the first post-window snapshot)."""
        super().__init__()
        self.out, self.cfg = out, cfg or WatchdogConfig()
        self.base_shares: dict[str, float] = {}
        if baseline is not None and (baseline / "reward_terms.json").exists():
            self.base_shares = json.loads((baseline / "reward_terms.json").read_text()).get("shares", {})
        self.eps: deque[dict[str, float]] = deque(maxlen=self.cfg.window)
        self.rollout_eps: list[dict[str, float]] = []
        self.snaps: list[Snapshot] = []
        self.n_eps = 0
        self.reason: str | None = None
        self._sat_n = self._sat_k = 0

    def _on_training_start(self) -> None:
        self._last = time.time()
        self.out.mkdir(parents=True, exist_ok=True)

    def _std(self) -> tuple[float, float]:
        ls = getattr(self.model.policy, "log_std", None)
        if ls is None:
            return 0.0, 0.0
        std = np.exp(ls.detach().cpu().numpy())
        return float(std.min()), float(std.max())

    def _decision_metric(self) -> float | None:
        f = self.out / "decision_metric.jsonl"
        if not f.exists():
            return None
        rows = [json.loads(x) for x in f.read_text().splitlines() if x.strip()]
        return float(rows[-1]["value"]) if rows else None

    def _on_step(self) -> bool:
        act = self.locals.get("actions")
        if act is not None and np.isfinite(self.model.action_space.high).all():
            a = np.abs(np.asarray(act))
            self._sat_k += int((a >= self.cfg.sat_level).sum())
            self._sat_n += a.size
        for done, info in zip(self.locals["dones"], self.locals["infos"]):
            m = info.get("metrics")
            if not done or m is None:
                continue
            row = {"ret": float(info.get("episode", {}).get("r", float("nan"))), "len": float(info.get("episode", {}).get("l", 0)),
                   "metric": float(m.get("success", m.get("distance", info.get("distance", 0.0)))),
                   "flips": float(m.get("orient_flips", 0.0)), "inv": float(m.get("orient_inverted_s", 0.0))}
            row.update({k: float(v) for k, v in m.items() if k.startswith("term_")})
            self.eps.append(row)
            self.rollout_eps.append(row)
            self.n_eps += 1
        if time.time() - self._last >= self.cfg.check_s:
            self._last = time.time()
            return self._check()
        return True

    def _on_rollout_end(self) -> None:
        if not self.rollout_eps:
            return
        sums = self._term_means(self.rollout_eps)
        for k, v in sums.items():
            self.logger.record(f"terms/{k}_mean", v)
        for k, v in term_shares(sums).items():
            self.logger.record(f"terms/{k}_share", v)
        for key, name in (("flips", "diag/flips_per_episode"), ("inv", "diag/inverted_s_per_episode")):
            self.logger.record(name, float(np.mean([e[key] for e in self.rollout_eps])))
        self.rollout_eps = []

    @staticmethod
    def _term_means(eps: Any) -> dict[str, float]:
        keys = sorted({k[5:] for e in eps for k in e if k.startswith("term_")})
        return {k: float(np.mean([e.get(f"term_{k}", 0.0) for e in eps])) for k in keys}

    def snapshot(self, partial: bool = False) -> Snapshot | None:
        """Summarise the last ``window`` episodes now (None until a full window exists; ``partial``: accept >= 20 episodes, for the end-of-run summary)."""
        if len(self.eps) < (20 if partial else self.cfg.window):
            return None
        means = self._term_means(self.eps)
        lo, hi = self._std()
        dec = self._decision_metric()
        ret = float(np.mean([e["ret"] for e in self.eps]))
        return Snapshot(steps=int(self.num_timesteps), episodes=self.n_eps, ret=ret, metric=dec if dec is not None else float(np.mean([e["metric"] for e in self.eps])),
                        ep_len=float(np.mean([e["len"] for e in self.eps])), flips=float(np.mean([e["flips"] for e in self.eps])),
                        inverted_s=float(np.mean([e["inv"] for e in self.eps])), shares=term_shares(means), means=means,
                        sat=self._sat_k / self._sat_n if self._sat_n else 0.0, std_max=hi, std_min=lo, nan=not math.isfinite(ret))

    def _check(self) -> bool:
        snap = self.snapshot()
        self._sat_n = self._sat_k = 0
        if snap is None:
            return True
        self.snaps.append(snap)
        if not self.base_shares and snap.steps >= self.cfg.min_steps and snap.episodes >= self.cfg.min_episodes:
            self.base_shares = dict(snap.shares)                       # no pilot baseline: the first post-minimum window defines it
        self.reason = judge(self.snaps, self.base_shares, self.cfg)
        self.write_summary()
        if self.reason:
            (self.out / "STOP_watchdog").write_text(f"{self.reason}\nsteps={snap.steps}\n")
            print(f"[watchdog] STOP at {snap.steps} steps: {self.reason}", flush=True)
            return False
        return True

    def write_summary(self) -> None:
        """Write ``reward_terms.json`` (mean and share per term, latest snapshot, history of the headline numbers, thresholds)."""
        if not self.snaps:
            return
        s = self.snaps[-1]
        (self.out / "reward_terms.json").write_text(json.dumps({"steps": s.steps, "means": s.means, "shares": s.shares, "baseline_shares": self.base_shares,
                                                                "history": [{"steps": x.steps, "ret": x.ret, "metric": x.metric, "ep_len": x.ep_len, "flips": x.flips} for x in self.snaps[-50:]],
                                                                "stop_reason": self.reason, "watchdog_cfg": asdict(self.cfg)}, indent=1))

    def _on_training_end(self) -> None:
        snap = self.snapshot(partial=True)
        if snap is not None:
            self.snaps.append(snap)
        self.write_summary()
