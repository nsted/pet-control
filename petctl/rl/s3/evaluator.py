"""S3 evaluator (DEV until the suites are frozen): replays one command scenario through PetEnv (task ``loco_cmd``) with a policy and returns M, per-segment T / S_stop, current, return and a read-back of what was
actually applied (command per segment, start orientation). ``dev_suite`` builds an UNFROZEN development suite; nothing measured on it is a gate number."""

from __future__ import annotations

from collections.abc import Callable
from multiprocessing import Pool
from typing import Any

import numpy as np

from petctl.rl.config import EnvConfig
from petctl.rl.env import PetEnv
from petctl.rl.tasks.loco_cmd import FAMILIES, command_of

START_ORIENT = {"upright": 0, "inverted": 1, "left": 2, "right": 3}
EPISODE_S = 20.0
NJ = 10
TP = {"mit_frame": 1.0, "thermal": 1.0, "action_unbounded": 1.0, "bus_model": 1.0, "bus_motor": 1.0, "bus_clamp": 0.0, "reply_dr": 1.0, "plant_nominal": 1.0}
Policy = Callable[[np.ndarray, PetEnv, int], np.ndarray]


def dev_suite(n_pairs: int = 8, starts: tuple[str, ...] = ("upright",), seed: int = 0) -> list[dict[str, Any]]:
    """Unfrozen DEV scenarios: ``n_pairs`` command pairs per start type (every non-stop family appears; the last pairs include a stop segment)."""
    rng = np.random.default_rng(seed)
    pairs: list[list[str]] = []
    moving = list(FAMILIES[:-1])
    for i in range(n_pairs):
        a, b = moving[i % len(moving)], moving[(i * 3 + 1) % len(moving)]
        pairs.append([a, "stop" if i == n_pairs - 1 else b])
    out = []
    for st in starts:
        for i, (fa, fb) in enumerate(pairs):
            segs = [{"family": f, "cmd": command_of(f, rng).tolist()} for f in (fa, fb)]
            out.append({"id": f"dev-{st}-{i}", "start": st, "seed": int(seed * 1000 + i), "segments": segs})
    return out


def env_for(sc: dict[str, Any], extra: dict[str, float] | None = None, slew_rad: float | None = None) -> PetEnv:
    """The S3 evaluation env for one scenario (n = 11, delta_kp actions, MIT frame, thermal, bus demand model, nominal plant, the scenario's start orientation)."""
    tp = {**TP, "start_orient": float(START_ORIENT[sc["start"]]), **(extra or {})}
    cfg = EnvConfig(task="loco_cmd", n_modules=11, episode_s=EPISODE_S, action_mode="delta_kp", task_params=tp)
    if slew_rad is not None:                                            # bridge runs only: the deployed protective ramp is 0.35 rad/tick (7 rad/s at 20 Hz); the old CPG env had no ramp
        cfg.max_target_step_rad = slew_rad
    return PetEnv(cfg)


def run_scenario(sc: dict[str, Any], policy: Policy, extra: dict[str, float] | None = None, slew_rad: float | None = None) -> dict[str, Any]:
    """One 20 s episode. M = mean of the per-segment T over the non-stop segments (None when there are none); S_stop over stop segments (reported apart)."""
    env = env_for(sc, extra, slew_rad)
    obs, _ = env.reset(seed=int(sc["seed"]), options={"scenario": {"segments": sc["segments"], **({"push": sc["push"]} if "push" in sc else {})}})
    task = env.task
    applied = {"start_orient": float(env.cfg.task_params["start_orient"]), "segments": [s.tolist() for s in task.segments], "up_dot_start": float(env._up_dot())}
    Ts: list[list[float]] = [[] for _ in task.segments]
    Ss: list[list[float]] = [[] for _ in task.segments]
    ret, cur = 0.0, []
    for t in range(env.max_steps):
        obs, r, term, trunc, info = env.step(np.asarray(policy(obs, env, t), float))
        assert not term, "S3 never terminates"
        ret += float(r)
        L = task.log[-1]
        if L["T"] is not None:
            Ts[L["seg"]].append(L["T"])
        if L["stop"] is not None:
            Ss[L["seg"]].append(L["stop"])
        cur.append(env._bus_means[-1])
    assert trunc
    seg_T = [float(np.mean(x)) if x else None for x in Ts]
    seg_S = [float(np.mean(x)) if x else None for x in Ss]
    moving = [x for x in seg_T if x is not None]
    stops = [x for x in seg_S if x is not None]
    return {"id": sc["id"], "start": sc["start"], "M": float(np.mean(moving)) if moving else None, "seg_T": seg_T, "seg_S": seg_S, "S_stop": float(np.mean(stops)) if stops else None,
            "I_mean": float(np.mean(cur)), "I_peak_tick": float(np.max(cur)), "return": ret, "up_dot_end": float(env._up_dot()), "applied": applied,
            "families": [s["family"] for s in sc["segments"]]}


def _selector_job(args: tuple[dict[str, Any], float | None]) -> dict[str, Any]:
    from petctl.rl.s3.selector import Selector
    sc, slew = args
    return run_scenario(sc, Selector(11, kp_action=-0.139, unclipped=slew is not None), slew_rad=slew)


def run_selector_suite(scs: list[dict[str, Any]], procs: int = 8, slew_rad: float | None = None) -> list[dict[str, Any]]:
    """The library selector over a list of scenarios (process pool); ``slew_rad`` relaxes the target slew limit for the bridge run."""
    with Pool(procs) as p:
        return p.map(_selector_job, [(s, slew_rad) for s in scs])
