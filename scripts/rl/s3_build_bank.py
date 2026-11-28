"""T23: S3 start-state bank (DEV until the S3 suites are frozen). Collapsed states = end states of S2 yield / press draws run with best PD (no yielding: the body is left displaced), post-gait states =
end states after 10 s of a library gait. Upright / side / inverted starts need no bank (``start_orient``). Writes data/rl/s3/bank/*.npz and manifest.json (SHA per file).

    PYTHONPATH=scripts/rl:. python scripts/rl/s3_build_bank.py
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from petctl.rl.s2 import suite
from petctl.rl.s2.evaluator import env_for as s2_env, pd_policy
from petctl.rl.s3.evaluator import env_for as s3_env
from petctl.rl.s3.selector import Selector

BANK = Path("data/rl/s3/bank")


def collapsed(n: int = 8) -> list[tuple[str, dict]]:
    """End states of yield draws under best PD (kp 5, zeta 2) with the L2 presses (needs data/rl/s2/press_table.json)."""
    tab = json.loads(Path("data/rl/s2/press_table.json").read_text())["table"]
    out = []
    draws = [s for s in suite.load("decision", "L2", "yield_draws") if f"{s.pose}|{s.orient}" in tab]
    for s in draws[:n]:
        d = s.to_dict() | {"press_n": tab[f"{s.pose}|{s.orient}"]["press_n"]}
        env = s2_env(d, fixed_kp=5.0, zeta=2.0)
        env.reset(seed=d["seed"], options={"scenario": d, "cache_dir": "data/rl/s2/cache"})
        for t in range(env.max_steps):
            env.step(pd_policy(None, env, t))
        E = float(np.abs(env.sim.joint_pos() - env.task.q_star).mean())
        out.append((f"collapsed_{s.id}", {"qpos": env.sim.data.qpos.copy(), "qvel": env.sim.data.qvel.copy(), "E_from_pose": E}))
    return out


def post_gait(n: int = 12, seed: int = 0) -> list[tuple[str, dict]]:
    """End states after 10 s of library gaits (deployable slew, random elite)."""
    rng = np.random.default_rng(seed)
    sel0 = Selector(11, kp_action=-0.139)
    out = []
    for i in range(n):
        idx = int(rng.integers(0, len(sel0.elites)))
        sel = Selector(11, kp_action=-0.139)
        sc = {"id": f"bank-{i}", "start": "upright", "seed": 500 + i, "segments": [{"family": "forward", "cmd": [0.1, 0, 0]}, {"family": "forward", "cmd": [0.1, 0, 0]}]}
        env = s3_env(sc)
        env.reset(seed=sc["seed"], options={"scenario": {"segments": sc["segments"]}})
        sel.params_for = lambda cmd, e=sel.elites[idx]: e                      # noqa: E731  (this elite for the whole run)
        for t in range(int(10 / env.dt)):
            env.step(sel(None, env, t))
        out.append((f"postgait_{i}_elite{idx}", {"qpos": env.sim.data.qpos.copy(), "qvel": env.sim.data.qvel.copy(), "up_dot": float(env._up_dot())}))
    return out


def main() -> int:
    """Build the bank and its manifest."""
    BANK.mkdir(parents=True, exist_ok=True)
    man = {}
    for name, d in collapsed() + post_gait():
        p = BANK / f"{name}.npz"
        np.savez(p, **d)
        man[p.name] = {"sha": hashlib.sha256(p.read_bytes()).hexdigest(), **{k: float(v) for k, v in d.items() if k not in ("qpos", "qvel")}}
    (BANK / "manifest.json").write_text(json.dumps(man, indent=1))
    print(len(man), "states", {k: round(v.get("E_from_pose", v.get("up_dot", 0)), 2) for k, v in man.items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
