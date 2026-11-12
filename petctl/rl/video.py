"""Offscreen MP4 rendering of a trained policy (inference only) and the end-of-training "video of the best" helper."""
from __future__ import annotations

import json
import subprocess
import time
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from petctl.rl.env import PetEnv
from petctl.rl.markers import draw_markers
from petctl.rl.policy import GaitPolicy

MEDIA_DIR = Path("docs/media")
SESSIONS_DIR = MEDIA_DIR / "sessions"


def record_video(run: Path, out: Path, policy_file: str = "policy.npz", seconds: float | None = None, seed: int = 0,
                 dr_scale: float = 0.0, fps: int = 30, size: tuple[int, int] = (960, 540), task_params: dict[str, float] | None = None, base: Any = None,
                 distance: float = 0.9, azimuth: float = 130.0) -> dict[str, float]:
    """Roll out ``run/policy_file`` deterministically for one episode, write an MP4 to ``out``, return the episode metrics.

    ``task_params`` overrides entries of the run's ``EnvConfig.task_params`` (e.g. force a mid-route goal change for a demo).
    """
    pol = GaitPolicy(run, policy_file)
    pol.cfg.dr.scale = dr_scale
    pol.cfg.task_params = {**pol.cfg.task_params, **(task_params or {})}
    if seconds is not None:
        pol.cfg.episode_s = seconds
    env = PetEnv(pol.cfg, base)
    obs, _ = env.reset(seed=seed)
    w, h = size
    env.sim.model.vis.global_.offwidth, env.sim.model.vis.global_.offheight = max(w, 640), max(h, 480)
    rend = mujoco.Renderer(env.sim.model, height=h, width=w)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = distance, -35, azimuth
    out.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(fps),
                           "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(out)], stdin=subprocess.PIPE)
    assert ff.stdin is not None
    frame_dt, next_frame, info = 1.0 / fps, 0.0, {}
    done = False
    while not done:
        obs, _, term, trunc, info = env.step(pol.action(obs))
        done = term or trunc
        t = env._t * env.dt
        while next_frame <= t:                                   # real-time playback at the video frame rate
            cam.lookat[:] = env.sim.data.subtree_com[1]
            rend.update_scene(env.sim.data, camera=cam)
            draw_markers(rend.scene, env.task.markers(env.sim))
            ff.stdin.write(np.ascontiguousarray(rend.render()).tobytes())
            next_frame += frame_dt
    ff.stdin.close()
    ff.wait()
    return {k: float(x) for k, x in info.get("metrics", {}).items()}


def record_best_video(run: Path, media_dir: Path = MEDIA_DIR) -> Path | None:
    """Write ``<media_dir>/<run>_best_ep<N>_<timestamp>.mp4`` for the run's best policy (``best_policy.npz``, else ``policy.npz``).

    Never raises: a missing ffmpeg or render error is reported and training results are unaffected.
    """
    try:
        best = run / "best.json"
        policy_file = "best_policy.npz" if (run / "best_policy.npz").exists() else "policy.npz"
        ep = json.loads(best.read_text()).get("episode", "x") if best.exists() else "final"
        out = media_dir / f"{run.name}_best_ep{ep}_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
        m = record_video(run, out, policy_file)
        print(f"[video] {out}  " + "  ".join(f"{k}={v:.3f}" for k, v in m.items()), flush=True)
        return out
    except Exception as e:  # noqa: BLE001 - video is a convenience, never fail the run for it
        print(f"[video] could not record best video: {e}", flush=True)
        return None


def finalize_session(session: Path) -> Path | None:
    """Mark the best video of a training session and write ``index.md``/``index.json``. Returns the best video path.

    "Best" = the highest replayed score (``metrics["distance"]``) among the session's videos; its file is renamed with a ``BEST_`` prefix.
    Videos are listed in the order the policy found them, i.e. the sequence of local maxima it climbed through.
    """
    idx = session / "index.json"
    if not idx.exists():
        return None
    entries = json.loads(idx.read_text())
    for e in entries:                                          # fill in the replayed numbers written by the render subprocess
        side = session / (e["video"] + ".json")
        if side.exists():
            e["replay"] = json.loads(side.read_text())
    done = [e for e in entries if (session / e["video"]).exists() and "replay" in e]
    best = max(done, key=lambda e: e["replay"].get("distance", 0.0)) if done else None
    if best is not None and not best["video"].startswith("BEST_"):
        new = "BEST_" + best["video"]
        (session / best["video"]).rename(session / new)
        if (session / (best["video"] + ".json")).exists():
            (session / (best["video"] + ".json")).rename(session / (new + ".json"))
        best["video"] = new
    for e in entries:
        e["best"] = e is best
    idx.write_text(json.dumps(entries, indent=1))
    rows = ["| # | video | reward (episode return) | height at record (cm) | replayed height (cm) | training steps | minutes |", "|---|---|---|---|---|---|---|"]
    for e in entries:
        r = e.get("replay", {}).get("distance")
        rows.append(f"| {e['n']} | {'**' if e['best'] else ''}{e['video']}{'** (BEST)' if e['best'] else ''} | {e['episode_return']:.0f} | "
                    f"{e['record_cm']:.1f} | {'-' if r is None else f'{100 * r:.1f}'} | {e['timesteps']} | {e['wall_min']:.0f} |")
    (session / "index.md").write_text(f"# Training session {session.name}\n\nOne video per new local maximum, in the order found. "
                                      f"Reward = raw episode return of the record episode (1 point = 1 mm of final height, minus penalties).\n\n"
                                      + "\n".join(rows) + "\n")
    return session / best["video"] if best else None


def main() -> int:
    """CLI used by the training callback to render a snapshot policy in a background process: ``RUN POLICY_FILE OUT [--seconds S]``."""
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path)
    ap.add_argument("policy_file")
    ap.add_argument("out", type=Path)
    ap.add_argument("--seconds", type=float, default=None)
    a = ap.parse_args()
    m = record_video(a.run, a.out, a.policy_file, a.seconds)
    Path(str(a.out) + ".json").write_text(json.dumps(m))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
