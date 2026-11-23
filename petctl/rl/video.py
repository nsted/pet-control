"""Offscreen MP4 rendering of a trained policy (inference only) and the end-of-training "video of the best" helper."""
from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path
from typing import Any, Callable

import mujoco
import numpy as np

from petctl.rl.env import PetEnv
from petctl.rl.markers import draw_markers
from petctl.rl.policy import GaitPolicy

MEDIA_DIR = Path("docs/media")
SESSIONS_DIR = MEDIA_DIR / "sessions"


def training_label(run: Path, task: str) -> str:
    """Human-readable "what was being trained" for a run directory: stage name + description when the run belongs to a curriculum stage."""
    stage = run.parent.name
    try:
        from petctl.rl.curriculum.stages import STAGES

        st = STAGES[stage] if isinstance(STAGES, dict) else next(x for x in STAGES if x.name == stage)
        return f"{st.name} ({run.name}): {st.description}"
    except Exception:  # noqa: BLE001 - not a curriculum run
        return f"{run.parent.name}/{run.name}: task={task}"


def burn_caption(frame: np.ndarray, text: str) -> np.ndarray:
    """Return ``frame`` (H,W,3 uint8) with a dark banner and ``text`` along the top."""
    from PIL import Image, ImageDraw

    img = Image.fromarray(np.ascontiguousarray(frame))
    d = ImageDraw.Draw(img, "RGBA")
    if img.width <= 1000:                                        # original small banner (bitmap font)
        d.rectangle([0, 0, img.width, 22], fill=(0, 0, 0, 170))
        d.text((6, 5), text[:int(img.width / 6.2)], fill=(255, 255, 255, 255))
        return np.asarray(img)
    from PIL import ImageFont

    size = int(img.width / 80)                                   # larger frames: scalable font so the caption keeps its relative size
    try:
        font = ImageFont.load_default(size=size)
    except Exception:  # noqa: BLE001
        font = None
    d.rectangle([0, 0, img.width, size + 14], fill=(0, 0, 0, 170))
    d.text((10, 6), text[:int(img.width / (size * 0.52))], fill=(255, 255, 255, 255), font=font)
    return np.asarray(img)


PORTRAIT_SIZE = (1080, 1920)    # 9:16 output (Instagram reels / stories)
SUPERSAMPLE = int(os.environ.get("PETCTL_VIDEO_SS", "2"))   # frames are rendered at SUPERSAMPLE x the output size and box-filtered down: removes the jagged edges of the offscreen GL renderer


def downscale(frame: np.ndarray, size: tuple[int, int]) -> np.ndarray:
    """Antialiased resize of a (H,W,3) uint8 frame to ``size`` = (w, h); identity when already that size."""
    if frame.shape[1] == size[0] and frame.shape[0] == size[1]:
        return frame
    from PIL import Image

    return np.asarray(Image.fromarray(np.ascontiguousarray(frame)).resize(size, Image.Resampling.LANCZOS))


def _font(width: int):
    from PIL import ImageFont

    try:
        return ImageFont.load_default(size=max(11, int(width / 54)))
    except Exception:  # noqa: BLE001 - old Pillow: fixed-size bitmap font
        return None


def burn_caption_wrapped(frame: np.ndarray, text: str) -> np.ndarray:
    """Like ``burn_caption`` but wraps the text over as many banner lines as it needs (narrow portrait frames)."""
    import textwrap

    from PIL import Image, ImageDraw

    img = Image.fromarray(np.ascontiguousarray(frame))
    d = ImageDraw.Draw(img, "RGBA")
    font = _font(img.width)
    lh = max(13, int(img.width / 42))
    rows = textwrap.wrap(text, max(10, int(img.width / (lh * 0.5))))[:6]
    d.rectangle([0, 0, img.width, 10 + lh * len(rows)], fill=(0, 0, 0, 170))
    for i, r in enumerate(rows):
        d.text((8, 5 + lh * i), r, fill=(255, 255, 255, 255), font=font)
    return np.asarray(img)


_VIDEO_LOG = Path(__file__).resolve().parents[2] / "docs" / "media" / ".video_log.jsonl"      # bookkeeping only: there is NO render cap (owner rule: 30-min progress videos per active stage)


def video_budget_ok(now: float | None = None) -> bool:
    """Always True: the old 10-videos/hour cap was removed (renders are still logged to docs/media/.video_log.jsonl). ``PETCTL_VIDEO_FORCE`` is accepted and ignored."""
    return True


def _log_video(out: Path) -> None:
    import time as _t
    _VIDEO_LOG.parent.mkdir(parents=True, exist_ok=True)
    with _VIDEO_LOG.open("a") as f:
        f.write(json.dumps({"t": _t.time(), "file": str(out)}) + "\n")


def record_video(run: Path, out: Path, policy_file: str = "policy.npz", seconds: float | None = None, seed: int = 0,
                 dr_scale: float = 0.0, fps: int = 30, size: tuple[int, int] = (1920, 1080), task_params: dict[str, float] | None = None, base: Any = None,
                 distance: float = 0.9, azimuth: float = 130.0, label: str | None = None, replace_task_params: bool = False,
                 final_note: Callable[[dict[str, float], bool], str] | None = None, portrait: bool | None = None) -> dict[str, float]:
    """Roll out ``run/policy_file`` deterministically for one episode, write an MP4 to ``out``, return the episode metrics.

    ``task_params`` overrides entries of the run's ``EnvConfig.task_params`` (e.g. force a mid-route goal change for a demo).
    ``replace_task_params=True`` uses ``task_params`` instead of merging it over the run's (replays a gate episode exactly).
    Every frame carries a caption saying what was being trained (``label``, default derived from the stage/task of the run).
    ``final_note(metrics, reached)`` (optional) returns the caption of the 2 s final hold frames, built from the metrics of THE recorded episode itself
    (so captions can state measured numbers); the returned metrics are exactly those passed to it.
    """
    if portrait is None:
        portrait = os.environ.get("PETCTL_PORTRAIT", "1") != "0"
    pol = GaitPolicy(run, policy_file)
    pol.cfg.dr.scale = dr_scale
    pol.cfg.task_params = dict(task_params or {}) if replace_task_params else {**pol.cfg.task_params, **(task_params or {})}
    if seconds is not None:
        pol.cfg.episode_s = seconds
    env = PetEnv(pol.cfg, base)
    obs, _ = env.reset(seed=seed)
    w, h = size
    ss = max(1, SUPERSAMPLE)
    env.sim.model.vis.global_.offwidth, env.sim.model.vis.global_.offheight = max(w * ss, 640), max(h * ss, 480)
    env.sim.model.vis.quality.offsamples = 8
    env.sim.model.vis.quality.shadowsize = 8192               # sharper shadow edges (the default 4096 map shows stair-steps)
    rend = mujoco.Renderer(env.sim.model, height=h * ss, width=w * ss)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = distance, -35, azimuth
    out.parent.mkdir(parents=True, exist_ok=True)
    caption = label or training_label(run, pol.cfg.task)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(fps),
                           "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(out)], stdin=subprocess.PIPE)
    assert ff.stdin is not None
    pw, ph = PORTRAIT_SIZE
    pout = out.with_name(out.stem + "_portrait" + out.suffix)
    prend = pff = None
    if portrait:                                                # second, 9:16 version of the same episode (own camera framing, wrapped caption), for Instagram
        env.sim.model.vis.global_.offwidth, env.sim.model.vis.global_.offheight = max(w * ss, pw * ss, 640), max(h * ss, ph * ss, 480)
        prend = mujoco.Renderer(env.sim.model, height=ph * ss, width=pw * ss)
        pff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{pw}x{ph}", "-r", str(fps), "-i", "-", "-an",
                                "-c:v", "libx264", "-pix_fmt", "yuv420p", str(pout)], stdin=subprocess.PIPE)

    pcam = mujoco.MjvCamera()                                   # portrait is narrower: pull the camera back so the whole body fits
    pcam.distance, pcam.elevation, pcam.azimuth = distance * 1.6, -40, azimuth

    def write_portrait(text: str) -> None:
        if prend is not None and pff is not None and pff.stdin is not None:
            pcam.lookat[:] = env.sim.data.subtree_com[1]
            prend.update_scene(env.sim.data, camera=pcam)
            draw_markers(prend.scene, env.task.markers(env.sim))
            pff.stdin.write(burn_caption_wrapped(downscale(prend.render(), (pw, ph)), text).tobytes())

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
            ff.stdin.write(burn_caption(downscale(rend.render(), (w, h)), f"{caption[:118]}  t={t:4.1f}s").tobytes())
            write_portrait(f"{caption}  t={t:4.1f}s")
            next_frame += frame_dt
    reached = bool(getattr(env.task, "reached", False))
    metrics = {k: float(x) for k, x in info.get("metrics", {}).items()}
    hold_caption = final_note(metrics, reached) if final_note is not None else None
    tag = "TARGET REACHED" if reached else "episode ended (target not reached)"
    for _ in range(2 * fps):                                    # hold the final state for 2 s: the episode ends the moment the target is reached, so the green marker would otherwise flash by
        cam.lookat[:] = env.sim.data.subtree_com[1]
        rend.update_scene(env.sim.data, camera=cam)
        draw_markers(rend.scene, env.task.markers(env.sim))
        ff.stdin.write(burn_caption(downscale(rend.render(), (w, h)), hold_caption or f"{caption[:70]}  {tag}  t={t:4.1f}s").tobytes())
        write_portrait(hold_caption or f"{caption}  {tag}  t={t:4.1f}s")
    ff.stdin.close()
    ff.wait()
    if pff is not None and pff.stdin is not None:
        pff.stdin.close()
        pff.wait()
    _log_video(out)
    return metrics


def rollout_metrics(run: Path, policy_file: str = "policy.npz", seed: int = 0, dr_scale: float = 0.0, seconds: float | None = None,
                    task_params: dict[str, float] | None = None, base: Any = None) -> dict[str, float]:
    """Render-free deterministic episode with exactly ``record_video``'s setup (same config handling and ``reset(seed)``): returns the episode metrics.

    Used to pick a showcase seed cheaply; ``record_video`` with the same arguments reproduces these metrics.
    """
    pol = GaitPolicy(run, policy_file)
    pol.cfg.dr.scale = dr_scale
    pol.cfg.task_params = {**pol.cfg.task_params, **(task_params or {})}
    if seconds is not None:
        pol.cfg.episode_s = seconds
    env = PetEnv(pol.cfg, base)
    obs, _ = env.reset(seed=seed)
    done, info = False, {}
    while not done:
        obs, _, term, trunc, info = env.step(pol.action(obs))
        done = term or trunc
    return {k: float(x) for k, x in info.get("metrics", {}).items()}


def record_best_video(run: Path, media_dir: Path = MEDIA_DIR) -> Path | None:
    """Write ``<media_dir>/<run>_best_ep<N>_<timestamp>.mp4`` for the run's best policy (``best_policy.npz``, else ``policy.npz``).

    Never raises: a missing ffmpeg or render error is reported and training results are unaffected.
    """
    try:
        best = run / "best.json"
        policy_file = "best_policy.npz" if (run / "best_policy.npz").exists() else "policy.npz"
        ep = json.loads(best.read_text()).get("episode", "x") if best.exists() else "final"
        out = media_dir / f"{run.parent.name}_{run.name}_best_ep{ep}_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
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
