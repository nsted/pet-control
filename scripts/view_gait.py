"""Interactive MuJoCo viewer of the lying PET model running a serpenoid gait. Run with `mjpython scripts/view_gait.py --urdf PATH`."""
from __future__ import annotations

import argparse
import time

import mujoco.viewer  # type: ignore[import-untyped]
import numpy as np

from petctl.sim.model_builder import BuildParams, PetSim


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--amp", type=float, default=0.5)
    ap.add_argument("--freq", type=float, default=0.5)
    ap.add_argument("--phi", type=float, default=2.1)
    ap.add_argument("--cap", type=float, default=0.164)
    a = ap.parse_args()
    sim = PetSim(BuildParams(urdf_dir=a.urdf, cap_nm=(a.cap,) * 7, delay_s=0.0))
    sim.step(np.zeros(7), 1.0)
    t0, dt = time.time(), 0.02
    with mujoco.viewer.launch_passive(sim.model, sim.data) as v:
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 0.8, -35, 130
        while v.is_running():
            t = time.time() - t0
            sim.step(a.amp * np.sin(2 * np.pi * a.freq * t - a.phi * np.arange(1, 8)), dt)
            v.cam.lookat[:] = sim.data.xpos[1:].mean(0)
            v.sync()
            time.sleep(max(0.0, dt - 0.002))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
