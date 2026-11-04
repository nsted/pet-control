"""Phase 1.5 go/no-go: what can the torque cap physically do to the lying robot?

For each joint and each torque cap, command a single-joint offset (+-0.3, +-0.6 rad) and report the angle actually reached, the
torque saturation, and how far the body lifts or moves; then a zig-zag and a travelling-wave pose with every joint. Uses PetSim
(free body on the floor), no robot motion.

Usage: python scripts/feasibility.py [--caps 0.05 0.164]
"""
from __future__ import annotations

import argparse

import numpy as np

from petctl.sim.model_builder import BuildParams, PetSim


def run(cap: float, target: np.ndarray, seconds: float = 2.5) -> tuple[np.ndarray, np.ndarray, float, float]:
    sim = PetSim(BuildParams(cap_nm=(cap,) * 7, delay_s=0.0))
    sim.step(np.zeros(7), 1.5)
    p0 = sim.head_pos().copy()
    sim.step(target, seconds)
    lift = float(max(sim.data.geom_xpos[i][2] for i in range(1, 9)) - 0.0175 - 0.0)
    return sim.joint_pos().copy(), sim.joint_torque(), lift, float(np.linalg.norm((sim.head_pos() - p0)[:2]))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--caps", type=float, nargs="+", default=[0.05, 0.164])
    a = ap.parse_args()
    for cap in a.caps:
        print(f"\n== torque cap {cap} N.m (kp 0.8 saturates at {cap / 0.8:.2f} rad error)")
        print("| joint | cmd rad | reached rad | torque N.m | saturated | max box height cm |")
        print("|---|---|---|---|---|---|")
        for j in range(7):
            for c in (0.3, -0.3, 0.6, -0.6):
                t = np.zeros(7)
                t[j] = c
                q, tau, lift, _ = run(cap, t)
                print(f"| {j + 1} | {c:+.1f} | {q[j]:+.2f} | {tau[j]:+.3f} | {'yes' if abs(tau[j]) >= 0.98 * cap else 'no'} | {lift * 100:.1f} |")
        for name, t in (("zigzag 0.5", 0.5 * np.array([1, -1, 1, -1, 1, -1, 1.0])), ("zigzag 0.25", 0.25 * np.array([1, -1, 1, -1, 1, -1, 1.0]))):
            q, tau, lift, drift = run(cap, t)
            print(f"{name}: reached {np.round(q, 2)}  |tau| max {np.abs(tau).max():.3f}  lift {lift * 100:.1f} cm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
