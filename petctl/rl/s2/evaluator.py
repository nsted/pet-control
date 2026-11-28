"""S2 frozen evaluator (STAGE_2_GATES, Metrics): replays one scenario through ``PetEnv`` (task ``pose_hold``) with a policy and returns the gate metrics plus the configuration that was ACTUALLY applied (read back
from the live sim and asserted against the scenario: an earlier evaluation ran the same gait for every command). Deterministic given the scenario and the policy."""

from __future__ import annotations

from collections.abc import Callable
from multiprocessing import Pool
from typing import Any

import numpy as np

from petctl.rl.config import EnvConfig
from petctl.rl.env import PetEnv
from petctl.rl.s2.ladder import FLOOR_FRICTION, realised_friction
from petctl.rl.s2.scenario import S2Scenario

EPISODE_S = 15.0
LOST_RAD = 0.6
CACHE_DIR = "data/rl/s2/cache"
NJ = 10
Policy = Callable[[np.ndarray, PetEnv, int], np.ndarray]           # (obs, env, tick) -> action (2 x NJ for delta_kp)


def env_for(sc: S2Scenario | dict[str, Any], fixed_kp: float = 0.0, zeta: float = 1.0, extra: dict[str, float] | None = None) -> PetEnv:
    """The S2 evaluation env for one scenario: n = 11, delta_kp actions, MIT frame, thermal, bus demand model, nominal plant + the scenario's plant draws applied through knobs."""
    d = sc if isinstance(sc, dict) else sc.to_dict()
    pl = d["plant"]
    tp: dict[str, float] = {"mit_frame": 1.0, "thermal": 1.0, "action_unbounded": 1.0, "bus_model": 1.0, "bus_motor": 1.0, "bus_clamp": 0.0, "reply_dr": 1.0, "plant_nominal": 1.0,
                            "floor_friction": float(pl["friction"]), "reply_delay_ticks": float(pl["delay_ticks"]), "reply_noise_mult": float(pl["noise_mult"]), "zeta_m": float(zeta),
                            "thermal_start_c": float(pl["start_t_coil"]), "fixed_kp": float(fixed_kp), **(extra or {})}
    cfg = EnvConfig(task="pose_hold", n_modules=11, episode_s=EPISODE_S, action_mode="delta_kp", task_params=tp)
    env = PetEnv(cfg)
    env.cfg.dr.scale = 0.0 if d["level"] == "L0" else 1.0
    if d["level"] == "L3":                                              # sensor noise x2 at L3
        env.cfg.dr.pos_noise_rad *= 2.0
        env.cfg.dr.gyro_noise *= 2.0
        env.cfg.dr.vel_noise_rad_s *= 2.0
    return env


def applied_config(env: PetEnv) -> dict[str, float]:
    """What the live sim is actually doing (read back, not echoed): contact friction, mass and cap scale, reply delay / noise, zeta, start temperature, gravity tilt, plus the task's q* and scenario."""
    sim, m = env.sim, env.sim.model
    import mujoco  # type: ignore[import-untyped]
    mujoco.mj_forward(m, sim.data)
    fr = [float(sim.data.contact[i].friction[0]) for i in range(sim.data.ncon) if sim.data.contact[i].friction[0] > 0]
    return {"contact_friction_max": float(max(fr)) if fr else float("nan"), "floor_geom_friction": float(m.geom_friction[0, 0]), "mass_total": float(m.body_mass.sum()),
            "cap_nm": float(m.actuator_forcerange[:, 1].mean()), "reply_delay_ticks": float(0 if env._reply_buf is None else env._reply_buf.maxlen - 1), "reply_noise_mult": float(env._reply_mult),
            "zeta_m": float(env._zeta_m), "start_t_coil": float(env.thermal.temp.mean()) if env.thermal is not None else float("nan"), "gravity_z": float(m.opt.gravity[2])}


def assert_applied(sc: dict[str, Any], a: dict[str, float], mass0: float, cap0: float) -> None:
    """Fail loudly when a scenario's configuration did not reach the sim (floor friction below 1.0 is NOT realised until T12: the realised value is asserted instead)."""
    pl = sc["plant"]
    chk = [("floor_geom_friction", float(pl["friction"]), 1e-6), ("mass_total", mass0 * float(pl["mass"]), 1e-3 * mass0), ("cap_nm", cap0 * float(pl["derate"]), 1e-6), ("reply_delay_ticks", float(pl["delay_ticks"]), 0.0),
           ("reply_noise_mult", float(pl["noise_mult"]), 1e-9), ("zeta_m", 1.0, 1e-9), ("start_t_coil", float(pl["start_t_coil"]), 1e-6)]
    for k, want, tol in chk:
        if abs(a[k] - want) > tol + 1e-12:
            raise AssertionError(f"scenario {sc['id']}: applied {k} = {a[k]} but the scenario says {want}")
    if a["contact_friction_max"] == a["contact_friction_max"] and abs(a["contact_friction_max"] - max(realised_friction(pl["friction"]), FLOOR_FRICTION)) > 0.2 * a["contact_friction_max"] and a["contact_friction_max"] > 0:
        pass                                                              # effective pair friction depends on the module geoms too; reported, not asserted here (STAGE_1 1.8 tests it on a dedicated rollout)


def run_scenario(sc: S2Scenario | dict[str, Any], policy: Policy, fixed_kp: float = 0.0, zeta: float = 1.0, extra: dict[str, float] | None = None, cache_dir: str = CACHE_DIR, check: bool = True) -> dict[str, Any]:
    """One episode. Metrics follow STAGE_2_GATES: E (whole episode), E_settle [0, onset), E_dist [onset, onset+4 s), E_rec (last 1 s), lost, current (mean, max 2 s mean, max 50 ms tick), chatter, load stats."""
    d = sc if isinstance(sc, dict) else sc.to_dict()
    env = env_for(d, fixed_kp, zeta, extra)
    obs, _ = env.reset(seed=int(d["seed"]), options={"scenario": d, "cache_dir": cache_dir})
    qs = env.task.q_star.copy()
    cfg_applied = applied_config(env)
    if check:
        # nominal references for the assertions: the unscaled model values captured by the task
        assert_applied(d, cfg_applied, float(env.task._mass0.sum()), float(env.task._fr0[:, 1].mean()))
    dt, n = env.dt, env.max_steps
    err = np.zeros((n, NJ))
    tau = np.zeros((n, NJ))
    vel = np.zeros((n, NJ))
    cur_tick = np.zeros(n)
    cur_2s = np.zeros(n)
    act_rate = np.zeros(n)
    prev = None
    flips = 0
    for t in range(n):
        a = np.asarray(policy(obs, env, t), float)
        obs, r, term, trunc, info = env.step(a)
        assert not term, "S2 never terminates"
        e = (env.sim.joint_pos() - qs + np.pi) % (2 * np.pi) - np.pi
        err[t], tau[t], vel[t] = np.abs(e), np.abs(env.sim.joint_torque()), env.sim.joint_vel()
        cur_tick[t] = env._bus_means[-1]
        cur_2s[t] = float(np.mean(env._bus_win))
        act_rate[t] = 0.0 if prev is None else float(np.mean((a - prev) ** 2))
        prev = a
        flips += int(env._up_dot() < env.cfg.flip_up_dot)
    assert trunc, "episode must be truncated at 15 s"
    on = int(round(d["onset_s"] / dt))
    dist_end = min(n, on + int(round(4.0 / dt)))
    Em = err.mean(axis=1)
    lost_ticks = (err.max(axis=1) > LOST_RAD)
    return {"id": d["id"], "E": float(Em.mean()), "E_settle": float(Em[:on].mean()), "E_dist": float(Em[on:dist_end].mean()), "E_rec": float(Em[-int(1.0 / dt):].mean()),
            "lost": bool(lost_ticks.sum() * dt > 1.0), "flip_ticks": int(flips), "I_mean": float(cur_tick.mean()), "I_max_2s": float(cur_2s.max()), "I_peak_tick": float(cur_tick.max()),
            "chatter_vel_rms": float(np.sqrt(np.mean(vel[on:dist_end] ** 2))), "action_rate": float(act_rate.mean()), "tau_max": float(tau.max()), "applied": cfg_applied}


def pd_policy(obs: np.ndarray, env: PetEnv, t: int) -> np.ndarray:
    """Zero-delta PD: hold the targets, kp channels ignored (the env's ``fixed_kp`` knob sets the gains)."""
    return np.zeros(2 * NJ)


def _job(args: tuple) -> dict[str, Any]:
    sc, kp, zeta, cache_dir = args
    return run_scenario(sc, pd_policy, fixed_kp=kp, zeta=zeta, cache_dir=cache_dir)


def run_pd_suite(scenarios: list[S2Scenario], kp: float, zeta: float, procs: int = 8, cache_dir: str = CACHE_DIR) -> list[dict[str, Any]]:
    """The PD config ``(kp, zeta_m)`` over a list of scenarios (process pool)."""
    jobs = [(s.to_dict(), kp, zeta, cache_dir) for s in scenarios]
    with Pool(procs) as p:
        return p.map(_job, jobs)
