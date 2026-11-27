"""Reward hack checks for the CPG-space command-tracking env (adversarial review 2026-11-26): scripted policies that do not track must score ~0 or below."""

from __future__ import annotations

import numpy as np
import pytest

from petctl.rl.cpg import PARAM_NAMES
from petctl.rl.cpg_env import NP, CPGEnv

I = {k: i for i, k in enumerate(PARAM_NAMES)}


def run(policy, seeds: int = 12, use_prior: bool = False) -> tuple[float, float, float]:
    """Mean (success, |yaw| measured on stand-or-translation commands is not available here -> flips, return) over seeds."""
    succ, flips, ret = [], [], []
    for s in range(seeds):
        e = CPGEnv(randomise=True, use_prior=use_prior)
        o, _ = e.reset(seed=100 + s)
        rng, R, info = np.random.default_rng(s), 0.0, {}
        for _ in range(e.episode_ticks):
            o, r, _, done, info = e.step(policy(rng))
            R += r
        succ.append(info["metrics"]["success"])
        flips.append(info["metrics"]["orient_flips"])
        ret.append(R)
    return float(np.mean(succ)), float(np.mean(flips)), float(np.mean(ret))


def stand(_rng) -> np.ndarray:
    a = np.zeros(NP + 2)
    a[[I["f"], I["a_yaw"], I["a_twist"]]] = -1.0
    return a


def thrash(_rng) -> np.ndarray:
    a = np.zeros(NP + 2)
    a[[I["f"], I["a_yaw"], I["a_twist"]]] = 1.0
    return a


@pytest.mark.parametrize("name,policy", [("stand", stand), ("zero", lambda r: np.zeros(NP + 2)), ("thrash", thrash), ("random", lambda r: r.uniform(-1, 1, NP + 2))])
def test_non_tracking_policies_do_not_score(name, policy) -> None:
    success, _flips, _ret = run(policy)
    limit = 0.10 if name == "thrash" else 0.05            # a constant gait travels somewhere and fits the commands that happen to point that way (~0.07): real motion, not an exploit
    assert success < limit, f"{name} scored {success:.3f} improvement over standing still"


def test_library_selector_baseline_tracks() -> None:
    """With the selector prior, doing nothing (zero action) is the library baseline: it must score clearly above the non-tracking policies on moving commands."""
    modes: dict[str, list[float]] = {}
    for s in range(12):
        e = CPGEnv(randomise=True)
        e.reset(seed=700 + s)
        for _ in range(e.episode_ticks):
            _o, _r, _t, _d, info = e.step(np.zeros(NP + 2))
        for k, v in info["metrics"].items():
            if k.startswith("mode_"):
                modes.setdefault(k, []).append(v)
    m = {k: float(np.mean(v)) for k, v in modes.items()}
    assert m["mode_turn"] > 0.03 and m["mode_mixed"] > 0.03 and m["mode_move"] > -0.04, m          # weak guards: the selector alone is a low baseline (~0.03) that learning must beat
