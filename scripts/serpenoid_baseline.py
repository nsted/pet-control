"""Serpenoid (travelling-wave) gait sweep on the lying PET model: does ANY open-loop gait move the body under the torque cap?

q_k(t) = s_k * A * sin(2 pi f t - k * phi),  s_k = 1 ("plain") or (-1)^(k+1) ("alt", matching the alternating joint-axis signs).
Reports displacement of the head along its initial heading and sideways, peak torque and the saturated fraction. This is the
baseline the RL policy must beat, and the Phase 1.5 go/no-go gate (single-joint commands cannot move the body: the axes are at
45 deg to the chain, so one joint must lift everything downstream; adjacent joints in opposition do not).

Usage: python scripts/serpenoid_baseline.py --urdf PATH/PET_description [--caps 0.164 0.05] [--seconds 10]
"""
from __future__ import annotations

import argparse
import itertools

import numpy as np

from petctl.sim.model_builder import BuildParams, PetSim


def run(urdf: str | None, cap: float, amp: float, freq: float, phi: float, alt: bool, seconds: float, friction: float) -> dict[str, float]:
    sim = PetSim(BuildParams(cap_nm=(cap,) * 7, urdf_dir=urdf, floor_friction=friction))
    sim.step(np.zeros(7), 1.0)
    p0, q0 = sim.head_pos().copy(), sim.data.qpos[3:7].copy()
    fwd = np.zeros(3)
    mat = np.zeros(9)
    import mujoco  # type: ignore[import-untyped]
    mujoco.mju_quat2Mat(mat, q0)
    fwd = mat.reshape(3, 3)[:, 0]                                    # body x at t0 = robot "forward" in the world
    fwd[2] = 0.0
    fwd /= np.linalg.norm(fwd)
    lat = np.array([-fwd[1], fwd[0], 0.0])
    dt, taus, sat = 0.05, [], []
    sign = np.array([(-1.0) ** k if alt else 1.0 for k in range(7)])   # joint k+1 -> index k
    for i in range(int(seconds / dt)):
        t = i * dt
        tgt = sign * amp * np.sin(2 * np.pi * freq * t - phi * np.arange(1, 8))
        sim.step(tgt, dt)
        tau = np.abs(sim.joint_torque())
        taus.append(tau.max())
        sat.append(float(np.mean(tau >= 0.98 * cap)))
    d = sim.head_pos() - p0
    return {"fwd_cm": 100 * float(d @ fwd), "lat_cm": 100 * float(d @ lat), "tau_max": float(max(taus)), "sat": float(np.mean(sat)),
            "up_dot": float(sim.data.sensor("quat").data[0])}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--caps", type=float, nargs="+", default=[0.164])
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--friction", type=float, default=1.0)
    ap.add_argument("--amps", type=float, nargs="+", default=[0.3, 0.5])
    ap.add_argument("--freqs", type=float, nargs="+", default=[0.5, 1.0])
    ap.add_argument("--phis", type=float, nargs="+", default=[-2.1, -1.0, 1.0, 2.1, 3.14])
    a = ap.parse_args()
    print("| cap | amp | f Hz | phi | signs | fwd cm | lat cm | peak tau | sat frac |")
    print("|---|---|---|---|---|---|---|---|---|")
    for cap, amp, f, phi, alt in itertools.product(a.caps, a.amps, a.freqs, a.phis, (False, True)):
        r = run(a.urdf, cap, amp, f, phi, alt, a.seconds, a.friction)
        print(f"| {cap} | {amp} | {f} | {phi:+.2f} | {'alt' if alt else 'plain'} | {r['fwd_cm']:+.1f} | {r['lat_cm']:+.1f} | "
              f"{r['tau_max']:.3f} | {r['sat']:.2f} |", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
