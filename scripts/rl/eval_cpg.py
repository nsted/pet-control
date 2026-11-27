"""Evaluate a CPG-space policy on fixed commands, write a captioned video (every clip shows the command, the measured window velocity and the CPG parameters) and a contact sheet (4 frames per command).

    python scripts/rl/eval_cpg.py data/rl/cpg_pilot1 --video docs/media/cpg_pilot1.mp4 --sheet data/rl/cpg_pilot1/contact_sheet.png [--zero]
``--zero``: no policy (zero action = the start gait keeps running unmodified), the baseline to compare against. Records the mean tracking kernel per command and the final CPG parameters (distance to the
nearest library elite is printed: a far-away parameter set is a candidate emergent gait).
"""

from __future__ import annotations

import argparse
import json
import math
import pickle
import subprocess
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from petctl.rl.cpg_env import HI, LO, CPGEnv
from petctl.rl.video import burn_caption

W, H, FPS = 1280, 720, 20
CMDS = {"stand": (0, 0, 0), "forward": (0.14, 0, 0), "back": (-0.14, 0, 0), "left": (0, 0.14, 0), "right": (0, -0.14, 0), "turn left": (0, 0, 0.45), "turn right": (0, 0, -0.45), "forward + turn left": (0.12, 0, 0.3)}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path)
    ap.add_argument("--video", type=Path, required=True)
    ap.add_argument("--sheet", type=Path, default=None)
    ap.add_argument("--zero", action="store_true")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--n", type=int, default=11)
    a = ap.parse_args()
    env = CPGEnv(n_modules=a.n, randomise=False, episode_s=a.seconds + 1.0, cmd_s=1e6)
    model = norm = None
    if not a.zero:
        from stable_baselines3 import PPO
        model = PPO.load(str(a.run / "model"), device="cpu")
        norm = pickle.load(open(a.run / "vecnormalize.pkl", "rb"))
    sim = env.sim
    sim.model.vis.global_.offwidth, sim.model.vis.global_.offheight = W, H
    rend = mujoco.Renderer(sim.model, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 1.3, -32.0, 120.0
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(a.video)],
                          stdin=subprocess.PIPE)
    assert ff.stdin is not None
    elites = np.array(env.elites)
    rows, sheet = {}, []
    for name, cmd in CMDS.items():
        o, _ = env.reset(seed=3)
        env.cmd = np.array(cmd, float)
        env.base = env.prior(env.cmd)                                                                   # the selector gait for THIS command (was left from the randomly sampled one: invalid evals before 2026-11-27)
        env.target = env.base.copy()
        env.cpg.reset(env.target, 0.0)
        env.mask = env.mask_ticks
        ks, frames = [], []
        for i in range(int(a.seconds / env.dt)):
            if model is None:
                act = np.zeros(13)
            else:
                ob = np.clip((o - norm.obs_rms.mean) / np.sqrt(norm.obs_rms.var + norm.epsilon), -10, 10)
                act = model.predict(ob, deterministic=True)[0]
            o, r, _, _, info = env.step(act)
            ks.append(env.sums["vel"] / max(env.k, 1))
            if i % int(FPS * env.dt) == 0 or True:
                cam.lookat[:] = sim.data.xipos[sim.m0_id:sim.m0_id + env.n].mean(axis=0)
                rend.update_scene(sim.data, camera=cam)
                img = rend.render()
                (c0, _), (c1, _) = env.hist[0], env.hist[-1]
                v = np.linalg.norm((c1 - c0)[:2]) / max((len(env.hist) - 1) * env.dt, 1e-6)
                cap = f"{env.n} modules | {'NO POLICY (start gait)' if model is None else 'CPG-space policy'} | command '{name}' v=({cmd[0]:+.2f},{cmd[1]:+.2f}) m/s yaw {cmd[2]:+.1f} rad/s | window speed {v:.2f} m/s | f {env.target[0]:.2f} Hz | t={i * env.dt:4.1f}s"
                ff.stdin.write(burn_caption(img, cap).tobytes())
                if i in (0, int(a.seconds / env.dt / 3), int(2 * a.seconds / env.dt / 3), int(a.seconds / env.dt) - 1):
                    frames.append(img)
        rows[name] = {"tracking_kernel_mean": float(np.mean(env.sums["vel"] / max(env.k, 1))), "final_params": env.target.tolist(),
                      "dist_to_nearest_elite": float(np.min(np.linalg.norm((elites - env.target) / (HI - LO), axis=1)))}
        print(f"{name:20s} tracking kernel {rows[name]['tracking_kernel_mean']:.2f}  nearest-elite distance {rows[name]['dist_to_nearest_elite']:.2f}", flush=True)
        sheet.append(np.concatenate([np.array(Image.fromarray(f).resize((320, 180))) for f in frames], axis=1))
    ff.stdin.close()
    ff.wait()
    if a.sheet:
        Image.fromarray(np.concatenate(sheet, axis=0)).save(a.sheet)
    (a.run / ("eval_zero.json" if a.zero else "eval.json")).write_text(json.dumps(rows, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
