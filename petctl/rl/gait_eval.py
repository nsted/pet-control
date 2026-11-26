"""Open-loop gait evaluator (shared by the Phase 3 sweeps and the Phase 4 searches): roll a CPG parameter vector out on the lying body and measure what it does.

``evaluate_gait`` returns, over the window after ``warmup_s``: gait-cycle-averaged velocity of the centre of mass in the INITIAL principal-axis heading frame (forward = tail -> head
horizontal direction, left = up x forward), heading-rotation (yaw) rate, rotation rate about the long axis (roll), speed in body lengths / s, cost of transport, current proxy
(GIM3505-8 model, UNVALIDATED), joint torque saturation, a smoothness (jerk) figure and diagnostics (flip, NaN). No domain randomisation here: friction / terrain are arguments.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from petctl.config import GIM3505_HOLD_A_PER_NM2, GIM3505_IDLE_PER_MOTOR_A, GIM3505_MOVE_A_PER_NM2, GIM3505_MOVE_OMEGA0_RAD_S
from petctl.rl.cpg import CPG, CPGParams
from petctl.sim.bus import IDLE_ELECTRONICS_A, BusClamp
from petctl.sim.model_builder import BuildParams, PetSim

CONTROL_DT = 0.05            # 20 Hz control tick, as the env
G = 9.81


@dataclass
class GaitResult:
    """Metrics of one rollout (SI units)."""

    vx: float = 0.0              # m/s along the initial heading (positive = head first)
    vy: float = 0.0              # m/s to the left
    yaw_rate: float = 0.0        # rad/s, rotation of the tail -> head direction about the vertical axis (left positive)
    roll_rate: float = 0.0       # rad/s, rotation about the long axis (net, unwrapped)
    speed: float = 0.0           # |(vx, vy)| m/s
    speed_bl: float = 0.0        # body lengths / s
    cot: float = float("nan")    # mechanical cost of transport: integral |tau * qdot| dt / (m g d)
    current_mean_a: float = 0.0  # proxy: mean total bus current (A), GIM3505-8 model, UNVALIDATED
    current_peak_a: float = 0.0  # proxy: peak over control ticks of the 50 ms mean (A)
    current_per_joint_peak_a: float = 0.0
    tau_peak: float = 0.0        # N.m
    sat_frac: float = 0.0        # fraction of (joint, tick) samples at >= 98 % of the cap
    jerk: float = 0.0            # mean |d3 q / dt3| of the joint positions (rad/s^3)
    up_min: float = 1.0          # lowest tail-IMU up.z over the window (negative = upside-down at some time)
    flipped: bool = False
    ok: bool = True              # False if the simulation produced non-finite numbers
    descriptors: tuple[float, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        """Plain dict (JSON)."""
        d = {k: v for k, v in self.__dict__.items() if k != "descriptors"}
        d["descriptors"] = list(self.descriptors)
        return d


def build_sim(n_modules: int, cap_nm: float = 0.5, friction: float = 1.0, dt: float = 0.004, terrain: Any = None) -> PetSim:
    """Lying body of ``n_modules`` with the GIM3505-8 current model attached (clamp off: the proxy only measures)."""
    sim = PetSim(BuildParams(n_modules=n_modules, cap_nm=(cap_nm,) * (n_modules - 1), floor_friction=friction, dt=dt, terrain=terrain))
    sim.bus = BusClamp(sim.model, GIM3505_HOLD_A_PER_NM2, 1e9, clamp=False, move_coeff=GIM3505_MOVE_A_PER_NM2, move_omega0=GIM3505_MOVE_OMEGA0_RAD_S,
                       idle_a=IDLE_ELECTRONICS_A + GIM3505_IDLE_PER_MOTOR_A * (n_modules - 1))
    return sim


def _heading(sim: PetSim) -> tuple[np.ndarray, np.ndarray, float]:
    """(forward unit vector, left unit vector, body length proxy) from the tail -> head direction at the current pose."""
    d = sim.data
    head, tail = d.xipos[sim.m0_id].copy(), d.xipos[sim.m0_id + sim.p.n_modules - 1].copy()
    fwd = head - tail
    fwd[2] = 0.0
    length = float(np.linalg.norm(fwd)) or 1e-6
    fwd /= length
    return fwd, np.array([-fwd[1], fwd[0], 0.0]), length


def evaluate_gait(params: CPGParams | np.ndarray, n_modules: int, seconds: float = 12.0, warmup_s: float = 3.0, cap_nm: float = 0.5, friction: float = 1.0,
                  sim: PetSim | None = None, phase0: float = 0.0, smooth_hz: float = 0.0, start_orient: float | None = None) -> GaitResult:
    """Roll ``params`` out for ``seconds`` (metrics over the window after ``warmup_s``). ``sim``: reuse a built sim (reset here); ``smooth_hz`` 0 = parameters applied instantly."""
    nj = n_modules - 1
    sim = sim or build_sim(n_modules, cap_nm, friction)
    sim.reset()
    if start_orient is not None:
        roll = float(start_orient)
        sim.data.qpos[3:7] = [math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0]
        sim.data.qpos[2] += 0.04
    sim.step(np.zeros(nj), 0.8)                                       # settle on the floor
    cpg = CPG(nj, smooth_hz=smooth_hz)
    cpg.reset(params, phase0)
    fwd0, left0, length = _heading(sim)
    d, m = sim.data, sim.model
    mass = float(m.body_mass.sum())
    bus = sim.bus
    n = int(round(seconds / CONTROL_DT))
    n_warm = int(round(warmup_s / CONTROL_DT))
    com, yaw, roll, qs, taus, sat, cur_mean, cur_peak, mech = [], [], [], [], [], [], [], [], 0.0
    up_min, ok = 1.0, True
    bus.begin_tick()
    for k in range(n):
        tgt = cpg.step(CONTROL_DT, params)
        sim.step(tgt, CONTROL_DT)
        tick = bus.end_tick()
        if not np.all(np.isfinite(d.qpos)):
            ok = False
            break
        tau, qd = sim.joint_torque(), sim.joint_vel()
        fwd, _, _ = _heading(sim)
        up = float(d.site_xmat[sim.imu_id].reshape(3, 3)[2, 2])
        # roll about the long axis: angle of a mid-body module's up vector in the (left0, vertical) plane of the initial heading frame
        mid = sim.p.n_modules // 2
        u = d.xmat[sim.m0_id + mid].reshape(3, 3)[:, 2]
        roll_a = math.atan2(float(u @ left0), float(u[2]))
        if k >= n_warm:
            com.append(d.xipos[sim.m0_id:sim.m0_id + n_modules].mean(axis=0))
            yaw.append(math.atan2(float(fwd @ left0), float(fwd @ fwd0)))
            roll.append(roll_a)
            qs.append(sim.joint_pos().copy())
            taus.append(np.abs(tau).max())
            sat.append(float(np.mean(np.abs(tau) >= 0.98 * cap_nm)))
            cur_mean.append(tick["mean"])
            cur_peak.append(tick["peak"])
            mech += float(np.sum(np.abs(tau * qd))) * CONTROL_DT
            up_min = min(up_min, up)
    res = GaitResult(ok=ok)
    if not ok or len(com) < 4:
        res.ok = False
        return res
    t_win = (len(com) - 1) * CONTROL_DT
    disp = np.array(com[-1]) - np.array(com[0])
    res.vx, res.vy = float(disp @ fwd0) / t_win, float(disp @ left0) / t_win
    res.speed = math.hypot(res.vx, res.vy)
    res.speed_bl = res.speed / length
    res.yaw_rate = float(np.unwrap(yaw)[-1] - np.unwrap(yaw)[0]) / t_win
    res.roll_rate = float(np.unwrap(roll)[-1] - np.unwrap(roll)[0]) / t_win
    dist = res.speed * t_win
    res.cot = mech / (mass * G * dist) if dist > 0.02 else float("nan")
    res.current_mean_a, res.current_peak_a = float(np.mean(cur_mean)), float(np.max(cur_peak))
    res.current_per_joint_peak_a = (res.current_peak_a - bus.idle) / nj
    res.tau_peak, res.sat_frac = float(np.max(taus)), float(np.mean(sat))
    q = np.array(qs)
    res.jerk = float(np.abs(np.diff(q, n=3, axis=0)).mean() / CONTROL_DT ** 3) if len(q) > 5 else 0.0
    res.up_min, res.flipped = up_min, up_min < 0.2
    res.descriptors = (math.atan2(res.vy, res.vx) if res.speed > 0.005 else 0.0, res.roll_rate, res.yaw_rate, res.speed_bl)
    return res
