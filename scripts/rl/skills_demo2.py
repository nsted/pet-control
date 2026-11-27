"""Phase 8 skill chain on a platform scene: SELF-RIGHT -> ALIGN -> CROSS the gap -> LOCOMOTE, one captioned video.

    PETCTL_PLATFORM_H=0.12 python scripts/rl/skills_demo2.py --gap 10 --out docs/media/skills_chain_n11.mp4
Start INVERTED on the left platform. SELF-RIGHT: the two-stage skill (data/rl/self_right/n11_v2.json). ALIGN: turn with the robust library's turning gaits until the body heading (head-tail chord; PRIVILEGED here: a robot would
need a heading estimate) is within 0.12 rad of the heading the crossing was searched from (yaw0 of data/rl/gap_cross/robust_n11_g<gap>.json). CROSS: the robust open-loop crossing gait until every module is on the far
platform. LOCOMOTE: forward command for a few seconds. Every skill hand-over is printed with its time; failures are shown, not hidden.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path

import mujoco
import numpy as np

import gap_cross as gc
from petctl.rl.cpg import CPG
from petctl.rl.cpg_env import NP, CPGEnv
from petctl.rl.video import burn_caption

W, Hh, FPS = 1280, 720, 20


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--gap", type=int, default=10)
    ap.add_argument("--cross", type=Path, default=None)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    cross = json.loads((a.cross or Path(f"data/rl/gap_cross/robust_n11_g{a.gap}.json")).read_text())
    sr = np.array(json.loads(Path("data/rl/self_right/n11_v2.json").read_text())["params"])
    cp, yaw0 = np.array(cross["params"]), float(cross["yaw0"])
    e = CPGEnv(randomise=False, use_prior=True, episode_s=300.0, cmd_s=1e6, mask_s=0.0)
    from petctl.sim.model_builder import BuildParams, PetSim
    g0 = a.gap / 100.0
    obs = (("box", -(g0 / 2 + 1.5), 0.0, 0.0, 1.5, 1.5), ("box", g0 / 2 + 0.5, 0.0, 0.0, 0.5, 0.6))        # wide, long start platform; the far platform as in the crossing search
    sim = PetSim(BuildParams(n_modules=11, cap_nm=(gc.CAP,) * 10, dt=0.004, obstacles=obs, extra={"obstacle_half_h": gc.H / 2}))
    e.sim, e.floor_geoms = sim, [g for g in range(sim.model.ngeom) if sim.model.geom_bodyid[g] == 0 and sim.model.geom_type[g] == 0]
    e.kp0, e.kd0 = (g.copy() for g in sim.gains())
    e.reset(seed=3)
    d, n, nj, g = sim.data, 11, 10, a.gap / 100.0
    d.qpos[0], d.qpos[1], d.qpos[2] = -(g / 2 + 1.6), 0.4, d.qpos[2] + gc.H + 0.004
    d.qpos[3:7] = [0.0, 1.0, 0.0, 0.0]                                                                    # inverted: roll pi about the long axis
    sim.step(np.zeros(nj), 0.8)
    sim.model.vis.global_.offwidth, sim.model.vis.global_.offheight = W, Hh
    rend = mujoco.Renderer(sim.model, height=Hh, width=W)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 1.6, -30.0, 90.0
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{Hh}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(a.out)],
                          stdin=subprocess.PIPE)
    assert ff.stdin is not None
    dt = e.dt
    cpg = CPG(nj, smooth_hz=0.0)

    def ups() -> np.ndarray:
        return np.array([d.xmat[sim.m0_id + k].reshape(3, 3)[2, 2] for k in range(n)])

    def heading() -> float:
        p = d.xipos[sim.m0_id:sim.m0_id + n]
        return float(np.median([math.atan2(p[i, 1] - p[j, 1], p[i, 0] - p[j, 0]) for i, j in e.chords]))

    def frame(cap: str) -> None:
        cam.lookat[:] = [float(d.xipos[sim.m0_id:sim.m0_id + n, 0].mean()), 0.0, gc.H]
        rend.update_scene(d, camera=cam)
        ff.stdin.write(burn_caption(rend.render(), cap).tobytes())

    log, t = [], 0.0
    # 1. SELF-RIGHT
    cpg.reset(sr, 0.0)
    ok, fade = 0, 1.0
    for _ in range(int(40 / dt)):
        med = float(np.median(ups()))
        if fade == 1.0:
            sim.step(cpg.step(dt, sr), dt)
            ok = ok + 1 if med > 0.8 else 0
            if ok >= 6:
                fade = 0.999
        else:
            fade = max(0.0, fade - dt / 1.5)
            sim.step(cpg.step(dt, sr) * fade, dt)
            if fade == 0.0:
                break
        t += dt
        frame(f"11 modules | SKILL CHAIN on {a.gap} cm gap, {100 * gc.H:.0f} cm platforms | SELF-RIGHT | up median {med:+.2f} | t={t:5.1f}s")
    for _ in range(int(1.5 / dt)):
        sim.step(np.zeros(nj), dt)
        t += dt
        frame(f"11 modules | SKILL CHAIN | SELF-RIGHT done, settling | t={t:5.1f}s")
    log.append((round(t, 1), "self-righted"))
    # 2. APPROACH: heading, then x (distance to the edge), then y, with the library's turn / forward / back / lateral gaits (PRIVILEGED position + heading)
    tx, ty = -(g / 2 + 0.55), 0.0
    t_app = 0.0

    def state() -> tuple[float, float, float]:
        c = d.xipos[sim.m0_id:sim.m0_id + n].mean(axis=0)
        return float(c[0]), float(c[1]), math.atan2(math.sin(heading() - yaw0), math.cos(heading() - yaw0))

    while t_app < 90.0:
        x, y, err = state()
        dx, dy = tx - x, ty - y
        if abs(err) < 0.1 and abs(dx) < 0.05 and abs(dy) < 0.06:
            break
        # command in the body frame (heading frame of the crossing start = yaw0): rotate the world error into it
        c, sn = math.cos(yaw0), math.sin(yaw0)
        bx, by = c * dx + sn * dy, -sn * dx + c * dy
        if abs(err) >= 0.1:
            cmd, what = np.array([0.0, 0.0, -0.5 * np.sign(err)]), f"turn (heading error {err:+.2f} rad)"
        elif abs(dx) >= 0.05 and abs(bx) > abs(by):
            cmd, what = np.array([0.14 * np.sign(bx), 0.0, 0.0]), f"{'forward' if bx > 0 else 'back'} (x error {dx:+.2f} m)"
        else:
            cmd, what = np.array([0.0, 0.14 * np.sign(by), 0.0]), f"sideways (y error {dy:+.2f} m)"
        e.cmd, e.base = cmd, e.prior(cmd)
        e.target = e.base.copy()
        e.cpg.reset(e.target, 0.0)
        burst = min(6.0, max(3.5, 8.0 * min(1.0, max(abs(err) / 0.6 if abs(err) >= 0.1 else 0.0, abs(dx) / 0.5 if abs(dx) >= 0.05 else 0.0, abs(dy) / 0.5))))     # longer bursts (3.5-6 s): the library velocities are steady-state values
        for _ in range(int(burst / dt)):
            sim.step(e.cpg.step(dt, e.target), dt)
            t, t_app = t + dt, t_app + dt
            frame(f"11 modules | SKILL CHAIN | APPROACH: {what} | position + heading are PRIVILEGED | t={t:5.1f}s")
        for _ in range(int(1.5 / dt)):                                                                   # pause, settle
            sim.step(np.zeros(nj), dt)
            t, t_app = t + dt, t_app + dt
            frame(f"11 modules | SKILL CHAIN | APPROACH pause | t={t:5.1f}s")
    x, y, err = state()
    log.append((round(t, 1), f"approach done: x error {tx - x:+.2f} m, y {ty - y:+.2f} m, heading {err:+.2f} rad"))
    # 3. CROSS
    cpg.reset(cp, 0.0)
    done_at = None
    for i in range(int(25 / dt)):
        sim.step(cpg.step(dt, cp), dt)
        t += dt
        p = d.xipos[sim.m0_id:sim.m0_id + n]
        if done_at is None and (p[:, 2] > gc.TOP).all() and p[:, 0].min() > g / 2 + 0.01 and p[:, 0].max() < g / 2 + 1.0:
            done_at = t
        state = "CROSSED" if done_at else "crossing"
        frame(f"11 modules | SKILL CHAIN | CROSS the {a.gap} cm gap: {state} | t={t:5.1f}s")
        if done_at and t > done_at + 1.5:
            break
    log.append((round(t, 1), "crossed" if done_at else "crossing FAILED"))
    # 4. LOCOMOTE
    if done_at:
        cmd = np.array([0.14, 0.0, 0.0])
        e.cmd, e.base = cmd, e.prior(cmd)
        e.target = e.base.copy()
        e.cpg.reset(e.target, 0.0)
        for _ in range(int(4.0 / dt)):
            sim.step(e.cpg.step(dt, e.target), dt)
            t += dt
            frame(f"11 modules | SKILL CHAIN | LOCOMOTE forward | t={t:5.1f}s")
        log.append((round(t, 1), "locomotion"))
    ff.stdin.close()
    ff.wait()
    print(log)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
