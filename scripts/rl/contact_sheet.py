"""Contact sheet: a frame grid of every scenario of one eval set (default: DECISION) for a run or a baseline policy, FIXED camera per scenario.

    python scripts/rl/contact_sheet.py --run data/rl/<run> --skill terrain --out data/rl/<run>/contact_terrain.png
    python scripts/rl/contact_sheet.py --zero --n-modules 11 --skill self_righting --out /tmp/sheet.png      # no policy: zero action (passive body)

Each row cell shows 4 frames (start, 1/3, 2/3, end) with the scenario id, start orientation, terrain and the episode metrics. The sheet is for the owner / reviewer
to LOOK at (one image per pilot and per full-run end); it never influences a gate. The TEST split is refused unless PETCTL_OPEN_TEST=1 (see evalsuite.load_suite).
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from petctl.rl.config import EnvConfig
from petctl.rl.env import PetEnv
from petctl.rl.evalsuite import ORIENT_NAMES, Scenario, load_suite, task_params
from petctl.rl.policy import GaitPolicy

TILE = (200, 150)
FRAMES = 4


def run_scenario(sc: Scenario, cfg: EnvConfig, pol: GaitPolicy | None, seconds: float) -> tuple[list[np.ndarray], dict[str, float]]:
    """Roll out one scenario with a fixed camera; returns FRAMES images and the episode metrics."""
    c = EnvConfig.from_dict(cfg.to_dict())
    c.task_params = {**c.task_params, **task_params(sc)}
    c.dr.scale = sc.dr_level
    c.episode_s = min(sc.episode_s, seconds)
    env = PetEnv(c)
    obs, _ = env.reset(seed=sc.seed)
    if pol is not None:
        pol.cfg = c
        pol.reset  # noqa: B018 - the exported policy is stateless apart from the env's own observation history
    rend = mujoco.Renderer(env.sim.model, height=TILE[1] * 2, width=TILE[0] * 2)
    cam = mujoco.MjvCamera()
    cam.lookat[:], cam.distance, cam.elevation, cam.azimuth = [0.0, 0.0, 0.0], 2.3, -62.0, 90.0       # fixed per scenario: same for every run
    marks = {int(round(i * (env.max_steps - 1) / (FRAMES - 1))) for i in range(FRAMES)}
    frames: list[np.ndarray] = []
    info: dict = {}
    for t in range(env.max_steps):
        if t in marks:
            rend.update_scene(env.sim.data, camera=cam)
            frames.append(np.asarray(Image.fromarray(rend.render()).resize(TILE, Image.Resampling.LANCZOS)))
        act = pol.action(obs) if pol is not None else np.zeros(c.n_actions, np.float32)
        obs, _, term, trunc, info = env.step(act)
        if term or trunc:
            break
    while len(frames) < FRAMES:
        rend.update_scene(env.sim.data, camera=cam)
        frames.append(np.asarray(Image.fromarray(rend.render()).resize(TILE, Image.Resampling.LANCZOS)))
    rend.close()
    return frames, {k: float(v) for k, v in info.get("metrics", {}).items() if not k.startswith("term_")}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path)
    ap.add_argument("--zero", action="store_true", help="no policy: zero actions")
    ap.add_argument("--n-modules", type=int, default=11)
    ap.add_argument("--skill", required=True)
    ap.add_argument("--split", default="decision")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--cols", type=int, default=2)
    ap.add_argument("--limit", type=int, default=0, help="first N scenarios only (smoke test)")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    if a.run is not None:
        pol: GaitPolicy | None = GaitPolicy(a.run)
        cfg = pol.cfg
    else:
        pol, cfg = None, EnvConfig(task="goal_reach", n_modules=a.n_modules, action_mode="hold", task_params={"goal_dist_min": 1.5, "goal_dist_max": 2.0, "allow_flip": 1.0})
    scs = load_suite(a.split, a.skill, reason=os.environ.get("PETCTL_OPEN_REASON", "contact sheet"))
    scs = scs[:a.limit] if a.limit else scs
    cell_w, cell_h = TILE[0] * FRAMES, TILE[1] + 28
    rows = (len(scs) + a.cols - 1) // a.cols
    sheet = Image.new("RGB", (cell_w * a.cols, cell_h * rows), (20, 20, 24))
    d = ImageDraw.Draw(sheet)
    summary = []
    for i, sc in enumerate(scs):
        frames, m = run_scenario(sc, cfg, pol, a.seconds)
        x, y = (i % a.cols) * cell_w, (i // a.cols) * cell_h
        sheet.paste(Image.fromarray(np.concatenate(frames, axis=1)), (x, y + 28))
        sel = {k: round(m[k], 2) for k in ("success", "distance", "orient_flips", "orient_inverted_s", "orient_recovery_s") if k in m}
        d.text((x + 4, y + 3), f"{sc.id} seed={sc.seed} {ORIENT_NAMES[sc.start_orient]} twist={sc.start_pose_rad} {sc.terrain}{sc.terrain_level} mu={sc.friction_mean} dr={sc.dr_level}", fill=(255, 255, 255))
        d.text((x + 4, y + 15), json.dumps(sel), fill=(150, 220, 150))
        summary.append({"id": sc.id, **sel})
        print(f"[{i + 1}/{len(scs)}] {sc.id} {sel}", flush=True)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(a.out)
    a.out.with_suffix(".json").write_text(json.dumps({"run": str(a.run) if a.run else "zero", "n_modules": cfg.n_modules, "split": a.split, "skill": a.skill, "scenarios": summary}, indent=1))
    print(f"wrote {a.out} ({sheet.size[0]}x{sheet.size[1]})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
