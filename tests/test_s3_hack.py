"""S3 hack tests (T24, DEV suite): stand-still, limp, thrash and random must never score (M <= 0.05, normalised return <= 0.05), from every start type: upright, left, right, inverted, a banked collapsed
state and a banked post-gait state. Also: the banked start really is used."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("gymnasium")
pytest.importorskip("mujoco")

from petctl.rl.s3.evaluator import dev_suite, run_scenario  # noqa: E402

BANK = Path("data/rl/s3/bank")
pytestmark = pytest.mark.skipif(not (BANK / "manifest.json").exists(), reason="start-state bank not built (scripts/rl/s3_build_bank.py)")
NJ = 10


def _starts() -> list[dict]:
    man = json.loads((BANK / "manifest.json").read_text())
    col = next(k for k in man if k.startswith("collapsed"))
    pg = next(k for k in man if k.startswith("postgait"))
    out = []
    for st in ("upright", "left", "right", "inverted"):
        out.append({"start": st})
    out.append({"start": "upright", "start_state": str(BANK / col), "tag": "collapsed"})
    out.append({"start": "upright", "start_state": str(BANK / pg), "tag": "postgait"})
    return out


def _scenarios() -> list[dict]:
    base = dev_suite(8)
    sel = [base[0], base[2], base[4], base[6], base[7]]          # forward, left, turn, combined and a stop segment
    out = []
    for si, st in enumerate(_starts()):
        for b in sel:
            sc = {**b, "id": f"{b['id']}-{st.get('tag', st['start'])}", "start": st["start"], "seed": b["seed"] + 10 * si}
            if "start_state" in st:
                sc["start_state"] = st["start_state"]
            out.append(sc)
    return out


def _policies() -> dict:
    rng = np.random.default_rng(0)
    return {"stand": (lambda o, e, t: np.zeros(2 * NJ), {}),
            "limp": (lambda o, e, t: np.zeros(2 * NJ), {"fixed_kp": 1e-4}),
            "thrash": (lambda o, e, t: np.concatenate([np.full(NJ, 1.0 if t % 2 else -1.0), np.zeros(NJ)]), {}),
            "random": (lambda o, e, t: np.concatenate([rng.uniform(-1, 1, NJ), rng.uniform(-1, 1, NJ)]), {})}


@pytest.mark.parametrize("name", ["stand", "limp", "thrash", "random"])
def test_hack_policies_never_score(name: str) -> None:
    pol, extra = _policies()[name]
    Ms, rets = [], []
    for sc in _scenarios():
        r = run_scenario(sc, pol, extra=extra)
        if r["M"] is not None:
            Ms.append(r["M"])
        rets.append(r["return"] / 400)
    assert np.mean(Ms) <= 0.05, (name, np.mean(Ms))
    assert np.mean(rets) <= 0.05, (name, np.mean(rets))


def test_banked_start_is_applied() -> None:
    from petctl.rl.s3.evaluator import env_for
    sc = _scenarios()[-1]                                          # post-gait bank state
    env = env_for(sc)
    env.reset(seed=1, options={"scenario": {"segments": sc["segments"], "start_state": sc["start_state"]}})
    c = np.load(sc["start_state"])
    assert np.allclose(env.sim.data.qpos, c["qpos"])
