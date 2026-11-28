"""Yield load (STAGE_2_GATES, Metrics): how hard the joints are pushed against the torque cap during an overload. Shared by the yield score Y, the reward term and the hack tests so they cannot drift apart."""

from __future__ import annotations

import numpy as np


def f_joint(tau: np.ndarray, cap_eff: np.ndarray | float) -> np.ndarray:
    """Per-joint graded load f(tau) = clip((|tau| / cap_eff - 0.5) / 0.4, 0, 1): 0 up to half the cap, 1 at 0.9 x cap (no cliff)."""
    return np.clip((np.abs(np.asarray(tau, float)) / np.asarray(cap_eff, float) - 0.5) / 0.4, 0.0, 1.0)


def load_of(tau: np.ndarray, cap_eff: np.ndarray | float) -> float:
    """Tick load = 0.5 x max_j f + 0.5 x (mean of the top 3 f over all joints). One joint pinned at the cap costs at least 0.667."""
    f = f_joint(tau, cap_eff)
    top3 = np.sort(f)[-3:]
    return float(0.5 * f.max() + 0.5 * top3.mean())


def yield_score(load_overload_ticks: np.ndarray, e_rec: float) -> float:
    """Y = 0.5 (1 - mean load over the overload ticks) + 0.5 clip(1 - E_rec / 0.1, 0, 1)."""
    ld = float(np.mean(load_overload_ticks)) if len(load_overload_ticks) else 0.0
    return 0.5 * (1.0 - ld) + 0.5 * float(np.clip(1.0 - e_rec / 0.1, 0.0, 1.0))
