"""Phase 8 prototype: a skill selector over the n = 11 skills found so far, rendered as one captioned video.

    python scripts/rl/skills_demo.py --out docs/media/skills_demo_n11.mp4
Start INVERTED. Selector rules (inputs a real robot has: head IMU + joint angles -> every module's up-vector): (1) if the whole-body median up-component < 0.3 and no recovery is running -> SELF-RIGHT (the two-stage
skill of data/rl/self_right/n11_v2.json: rolling CPG gait until the median up > 0.8 for 0.3 s, then ease to straight and hold 1.5 s); (2) otherwise LOCOMOTE: the commanded velocity is served by the robust library selector
(CPGEnv.prior); a command change eases the amplitudes down and back up (steer_classic.Steer). Command script: forward, turn left, left, back. If the body is flipped again at any time, rule (1) fires.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path

import mujoco
import numpy as np

from petctl.rl.cpg import CPG
from petctl.rl.cpg_env import NP, CPGEnv
from petctl.rl.video import burn_caption
from steer_classic import Steer

W, H, FPS = 1280, 720, 20
SCRIPT = [("forward", (0.14, 0.0, 0.0), 9.0), ("turn left", (0.0, 0.0, 0.4), 6.0), ("left", (0.0, 0.14, 0.0), 7.0), ("back", (-0.14, 0.0, 0.0), 7.0)]


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--orient", type=float, default=math.pi, help="start roll about the long axis (rad); pi = inverted")
    a = ap.parse_args()
    sr = np.array(json.loads(Path("data/rl/self_right/n11_v2.json").read_text())["params"])
    e = CPGEnv(randomise=False, use_prior=True, episode_s=200.0, cmd_s=1e6, mask_s=0.0)
    e.reset(seed=2)
    sim, d, n, nj = e.sim, e.sim.data, e.n, e.nj
    d.qpos[3:7] = [math.cos(a.orient / 2), math.sin(a.orient / 2), 0.0, 0.0]
    d.qpos[2] += 0.04
    sim.step(np.zeros(nj), 0.8)
    sim.model.vis.global_.offwidth, sim.model.vis.global_.offheight = W, H
    rend = mujoco.Renderer(sim.model, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 1.5, -32.0, 120.0
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(a.out)],
                          stdin=subprocess.PIPE)
    assert ff.stdin is not None

    def up_all() -> np.ndarray:
        return np.array([d.xmat[sim.m0_id + k].reshape(3, 3)[2, 2] for k in range(n)])

    state, t, ok_ticks, fade, hold = "SELF-RIGHT", 0.0, 0, 1.0, 0.0
    cpg = CPG(nj, smooth_hz=0.0)
    cpg.reset(sr, 0.0)
    seg, seg_t, steer = 0, 0.0, None
    log = []
    total = 4.0 + sum(s[2] for s in SCRIPT) + 25.0
    for i in range(int(total / e.dt)):
        up = up_all()
        med = float(np.median(up))
        if state == "SELF-RIGHT":
            if fade == 1.0:
                sim.step(cpg.step(e.dt, sr), e.dt)
                ok_ticks = ok_ticks + 1 if med > 0.8 else 0
                if ok_ticks >= 6:
                    fade = 0.999
            else:
                fade = max(0.0, fade - e.dt / 1.5)
                sim.step(cpg.step(e.dt, sr) * fade, e.dt)
                if fade == 0.0:
                    hold += e.dt
                    if hold >= 1.5:                                                                          # recovery done: hand over to LOCOMOTE
                        state = "LOCOMOTE"
                        e.hist.clear()
                        e._push()
                        e.h0 = float(np.median(e._chord_angles()))
                        e.cmd = np.array(SCRIPT[0][1])
                        e.base = e.prior(e.cmd)
                        e.target = e.base.copy()
                        e.cpg.reset(e.target, 0.0)
                        steer, seg, seg_t = Steer(e, 1.0, 1.0), 0, 0.0
                        log.append((round(i * e.dt, 1), "self-righted, start locomotion"))
        else:
            if med < 0.3:                                                                                     # flipped again: recover
                state, fade, hold, ok_ticks = "SELF-RIGHT", 1.0, 0.0, 0
                cpg.reset(sr, 0.0)
                log.append((round(i * e.dt, 1), "flipped, self-right"))
            else:
                seg_t += e.dt
                if seg < len(SCRIPT) - 1 and seg_t >= SCRIPT[seg][2]:
                    seg, seg_t = seg + 1, 0.0
                    e.cmd = np.array(SCRIPT[seg][1])
                steer.step()
                e.step(np.zeros(NP + 2))
        cam.lookat[:] = d.xipos[sim.m0_id:sim.m0_id + n].mean(axis=0)
        rend.update_scene(d, camera=cam)
        cmd_txt = f"command '{SCRIPT[seg][0]}'" if state == "LOCOMOTE" else "no command yet"
        cap = f"{n} modules | SKILL SELECTOR | skill: {state} | whole-body up median {med:+.2f} | {cmd_txt} | robust library + transitions, sim only | t={i * e.dt:5.1f}s"
        ff.stdin.write(burn_caption(rend.render(), cap).tobytes())
    ff.stdin.close()
    ff.wait()
    print(log)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
