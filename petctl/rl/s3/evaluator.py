"""S3 evaluator (DEV until the suites are frozen): replays one command scenario through PetEnv (task ``loco_cmd``) with a policy and returns M, per-segment T / S_stop, current, return and a read-back of what was
actually applied (command per segment, start orientation). ``dev_suite`` builds an UNFROZEN development suite; nothing measured on it is a gate number."""

from __future__ import annotations

from collections.abc import Callable
from multiprocessing import Pool
from typing import Any

import numpy as np

from petctl.rl.config import EnvConfig
from petctl.rl.env import PetEnv
from petctl.rl.s3 import ladder
from petctl.rl.tasks.loco_cmd import FAMILIES, command_of

START_ORIENT = {"upright": 0, "inverted": 1, "left": 2, "right": 3}
EPISODE_S = 20.0
NJ = 10
TP = {"mit_frame": 1.0, "thermal": 1.0, "action_unbounded": 1.0, "bus_model": 1.0, "bus_motor": 1.0, "bus_clamp": 0.0, "reply_dr": 1.0, "plant_nominal": 1.0}
Policy = Callable[[np.ndarray, PetEnv, int], np.ndarray]


def dev_suite(n_pairs: int = 8, starts: tuple[str, ...] = ("upright",), seed: int = 0, level: str | None = None) -> list[dict[str, Any]]:
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
            sc = {"id": f"dev-{st}-{i}", "start": st, "seed": int(seed * 1000 + i), "segments": segs}
            if level is not None:                                           # ladder draw: plant + slope + terrain + a push during locomotion
                pl = ladder.sample(level, rng)
                az = float(rng.uniform(0, 2 * np.pi))
                sc.update({"level": level, "plant": pl, "push": {"t": float(rng.uniform(3, 15)), "dur": float(rng.uniform(0.5, 2.0)), "body": "mid", "f": [pl["push_n"] * np.cos(az), pl["push_n"] * np.sin(az), 0.0]}})
            out.append(sc)
    return out


def env_for(sc: dict[str, Any], extra: dict[str, float] | None = None, slew_rad: float | None = None) -> PetEnv:
    """The S3 evaluation env for one scenario (n = 11, delta_kp actions, MIT frame, thermal, bus demand model, nominal plant, the scenario's start orientation)."""
    tp = {**TP, "start_orient": float(START_ORIENT[sc["start"]]), **(extra or {})}
    pl = sc.get("plant")
    if pl:                                                                  # ladder draws that go through env knobs
        tp.update({"floor_friction": float(pl["friction"]), "reply_delay_ticks": float(pl["delay_ticks"]), "reply_noise_mult": float(pl["noise_mult"]), "zeta_m": float(pl["zeta_m"]),
                   "thermal_start_c": float(pl["start_t_coil"])})
        if pl["ridge_level"] > 0.0:
            tp.update({"terrain_level": float(pl["ridge_level"]), "terrain_ridges": 1.0})
    cfg = EnvConfig(task="loco_cmd", n_modules=11, episode_s=EPISODE_S, action_mode="delta_kp", task_params=tp)
    if slew_rad is not None:                                            # bridge runs only: the deployed protective ramp is 0.35 rad/tick (7 rad/s at 20 Hz); the old CPG env had no ramp
        cfg.max_target_step_rad = slew_rad
    return PetEnv(cfg)


def applied_config(env: PetEnv) -> dict[str, float]:
    """What the live sim is actually doing (read back, not echoed): floor friction, mass, head mass, cap, joint friction, armature, floor tilt, terrain, reply delay / noise, zeta, start temperature."""
    sim, m = env.sim, env.sim.model
    gid = int(np.flatnonzero(m.geom_type == 0)[0])                           # mjGEOM_PLANE
    import mujoco  # type: ignore[import-untyped]
    mujoco.mj_forward(m, sim.data)
    nz = float(sim.data.geom_xmat[gid].reshape(3, 3)[2, 2])
    return {"floor_geom_friction": float(m.geom_friction[gid, 0]), "mass_total": float(m.body_mass.sum()), "head_mass": float(m.body_mass[sim.m0_id]), "cap_nm": float(m.actuator_forcerange[:, 1].mean()),
            "joint_friction": float(m.dof_frictionloss[6:16].mean()), "armature": float(m.dof_armature[6:16].mean()), "slope_deg": float(np.degrees(np.arccos(np.clip(nz, -1, 1)))),
            "terrain": float(sim.p.terrain is not None), "reply_delay_ticks": float(0 if env._reply_buf is None else env._reply_buf.maxlen - 1), "reply_noise_mult": float(env._reply_mult),
            "zeta_m": float(env._zeta_m), "start_t_coil": float(env.thermal.temp.mean()) if env.thermal is not None else float("nan")}


def run_scenario(sc: dict[str, Any], policy: Policy, extra: dict[str, float] | None = None, slew_rad: float | None = None) -> dict[str, Any]:
    """One 20 s episode. M = mean of the per-segment T over the non-stop segments (None when there are none); S_stop over stop segments (reported apart)."""
    env = env_for(sc, extra, slew_rad)
    obs, _ = env.reset(seed=int(sc["seed"]), options={"scenario": {"segments": sc["segments"], **({"plant": sc["plant"]} if "plant" in sc else {}), **({"push": sc["push"]} if "push" in sc else {}), **({"start_state": sc["start_state"]} if "start_state" in sc else {})}})
    task = env.task
    applied = {"start_orient": float(env.cfg.task_params["start_orient"]), "segments": [s.tolist() for s in task.segments], "up_dot_start": float(env._up_dot()), "config": applied_config(env)}
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
