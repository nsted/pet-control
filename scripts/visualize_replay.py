"""Show a capture_all.py run in Rerun: the real robot and its MuJoCo replay as two 3D chains, plus angle/torque curves.

Usage: python scripts/visualize_replay.py data/capture/capture_robot_YYYYmmdd_HHMMSS [--start S] [--end S] [--fps 20]
Blue chain/lines = measured, orange = simulated (same kinematics, ``gravity_model.positions``). Gravity points down.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import rerun as rr

from petctl.gravity_model import ghat, positions
from petctl.sim.mujoco_model import SimParams

from replay_mujoco import replay  # noqa: E402  (sibling script)

BLUE, ORANGE = [60, 140, 255], [255, 140, 40]


def _to_z_up() -> np.ndarray:
    """Rotation taking the chain's gravity direction to -Z."""
    z = np.array([0.0, 0.0, -1.0])
    v = np.cross(ghat, z)
    c = float(ghat @ z)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K / (1 + c)


def chain(phi: np.ndarray, R: np.ndarray, dx: float) -> np.ndarray:
    pos = positions(phi)
    return np.array([R @ pos[i] + np.array([dx, 0, 0]) for i in range(1, 8)])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("base")
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--end", type=float, default=1e9)
    ap.add_argument("--fps", type=float, default=20.0)
    a = ap.parse_args()
    z = np.load(f"{a.base}.npz")
    d = {k: z[k] for k in z.files}
    meta = json.loads(Path(f"{a.base}.json").read_text())
    t = d["t"] - d["t"][0]
    print("simulating...", flush=True)
    th, tau = replay(d, SimParams(delay_s=0.08, armature=1e-4), const_cap=True)
    R = _to_z_up()
    rr.init("pet_replay")
    rr.spawn(memory_limit="1GiB")
    rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    for k in range(1, 8):
        for kind, col in (("real", BLUE), ("sim", ORANGE)):
            rr.log(f"angle/joint_{k}/{kind}", rr.SeriesLines(colors=[col], names=f"joint {k} {kind}"), static=True)
            rr.log(f"torque/joint_{k}/{kind}", rr.SeriesLines(colors=[col], names=f"joint {k} {kind}"), static=True)
    marks = sorted((m for m in meta["marks"] if "t1" in m), key=lambda m: m["t0"])
    mi, last = 0, -1e9
    for i in range(len(t)):
        if t[i] < a.start or t[i] > a.end or t[i] - last < 1.0 / a.fps:
            continue
        last = t[i]
        rr.set_time("time", duration=float(t[i]))
        while mi < len(marks) and marks[mi]["t0"] - d["t"][0] <= t[i]:
            rr.log("phase", rr.TextLog(marks[mi]["name"]))
            mi += 1
        for dx, phi, kind, col in ((-0.12, d["theta"][i], "real", BLUE), (0.12, th[i], "sim", ORANGE)):
            pts = chain(phi, R, dx)
            rr.log(f"robot/{kind}/modules", rr.Points3D(pts, radii=0.012, colors=[col]))
            rr.log(f"robot/{kind}/links", rr.LineStrips3D([pts], radii=0.004, colors=[col]))
        for k in range(7):
            rr.log(f"angle/joint_{k + 1}/real", rr.Scalars(float(d["theta"][i, k])))
            rr.log(f"angle/joint_{k + 1}/sim", rr.Scalars(float(th[i, k])))
            rr.log(f"torque/joint_{k + 1}/real", rr.Scalars(float(d["tau"][i, k])))
            rr.log(f"torque/joint_{k + 1}/sim", rr.Scalars(float(tau[i, k])))
    print("done - the Rerun viewer has the run (scrub the 'time' timeline).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
