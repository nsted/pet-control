"""Render the wall-contact demo videos: S5 policy walking into a wall at given angles, camera from the side. Usage: wall_video.py [angle_deg ...]"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

from petctl.rl.env import PetEnv
from petctl.rl.policy import GaitPolicy
from petctl.rl.video import record_video
from petctl.sim.model_builder import BuildParams

R = Path("data/rl/curriculum_fast/s5_goal_far_robust/iter_01")
TP = {"goal_dist_min": 3.0, "goal_dist_max": 3.0, "bearing_deg": 0.0, "n_goals": 0, "start_pose_rad": 0.0}

for ang in [float(a) for a in sys.argv[1:]] or [0.0, 25.0]:
    seed = 20000
    cfg = GaitPolicy(R).cfg
    cfg.task_params, cfg.dr.scale, cfg.episode_s = TP, 0.0, 12
    base = BuildParams(urdf_dir=cfg.urdf_dir, dt=cfg.sim_dt)
    e0 = PetEnv(cfg, base)
    e0.reset(seed=seed)
    h, f = e0.task._head(e0.sim), e0.task.heading
    c = h[:2] + 0.6 * f[:2]
    yaw = np.arctan2(f[1], f[0]) + np.pi / 2 + np.deg2rad(ang)
    b = BuildParams(**{**base.__dict__, "walls": ((float(c[0]), float(c[1]), float(yaw), 1.2),)})
    az = float(np.rad2deg(np.arctan2(f[1], f[0]))) + 35.0
    out = Path(f"docs/media/wall_contact_side_angle{int(ang)}_{time.strftime('%Y%m%d_%H%M%S')}.mp4")
    record_video(R, out, seed=seed, seconds=12, task_params=TP, base=b, distance=1.6, azimuth=az)
    print(out)
