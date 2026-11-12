"""Curriculum stage definitions. Gates are FROZEN: they live in code (reviewed via git diff) and the tuning loop may only change a stage's
training knobs within ``Stage.knobs`` ranges, never the gate. Thresholds below are initial guesses to be calibrated against measured baselines."""

from __future__ import annotations

from dataclasses import dataclass, field

REAL_CAP_NM = 0.5        # the real per-joint torque cap (Nick, 2026-11-11)


@dataclass(frozen=True)
class Gate:
    """Pass/fail test run on a finished policy over fixed held-out seeds (``seed0`` + i), independent of the training reward."""

    task_params: dict[str, float]            # evaluation distribution (may be harder than training's)
    episodes: int = 40
    dr_scale: float = 0.3                    # robustness condition; a nominal (dr 0) condition is also reported
    min_success: float = 0.8                 # success rate required under the robust condition
    max_glitch: float = 0.0                  # tolerated fraction of episodes tripping the physics glitch guard
    seed0: int = 10_000
    episode_s: float | None = None           # override episode length (None: the stage's training length)


@dataclass(frozen=True)
class Stage:
    """One curriculum stage: what to train, how to judge it, and what the tuning loop may vary."""

    name: str
    task: str
    description: str
    task_params: dict[str, float]            # training distribution
    gate: Gate
    steps: int = 1_500_000
    n_envs: int = 8
    dr_scale: float = 0.3
    cap_nm: float = REAL_CAP_NM
    episode_s: float = 30.0                  # training episode length; a successful episode ends early on arrival
    ent_coef: float = 0.0
    action_scale: float | None = 1.0         # joint-target range (rad) for action=+-1; 1.0 with ema 0.3 walked ~7x further than 0.5/0.6 (docs/plans/CURRICULUM_LOG.md)
    action_ema: float | None = 0.3           # action smoothing: target = ema*prev + (1-ema)*new
    parent: str | None = None                # stage whose passing policy warm-starts this one
    knobs: dict[str, tuple[float, float]] = field(default_factory=dict)   # tunable name -> (lo, hi); keys: ent_coef, dr_scale, steps, or any task_param
    max_iterations: int = 8


_COMMON_KNOBS = {"ent_coef": (0.0, 0.02), "dr_scale": (0.0, 1.0), "steps": (500_000, 4_000_000)}

STAGES: dict[str, Stage] = {s.name: s for s in [
    Stage("s1_goal_ahead", "goal_reach", "Reach a point 80-150 cm straight ahead within 30 s at the real 0.5 N.m cap, from a straight start, under mild randomisation.",
          task_params={"goal_dist_min": 0.8, "goal_dist_max": 1.5, "bearing_deg": 0.0},
          gate=Gate(task_params={"goal_dist_min": 1.0, "goal_dist_max": 1.5, "bearing_deg": 0.0}, min_success=0.8),
          knobs={**_COMMON_KNOBS, "goal_dist_max": (1.0, 2.5), "goal_dist_min": (0.4, 1.5)}),
    Stage("s1b_random_start", "goal_reach", "Same goals (within +-60 deg) but every episode starts from a random joint configuration (each joint +-0.6 rad), so the body begins "
          "curled at a random orientation relative to the target.",
          task_params={"goal_dist_min": 0.8, "goal_dist_max": 1.5, "bearing_deg": 60.0, "start_pose_rad": 0.6}, parent="s1_goal_ahead",
          gate=Gate(task_params={"goal_dist_min": 1.0, "goal_dist_max": 1.5, "bearing_deg": 60.0, "start_pose_rad": 0.6}, min_success=0.75),
          knobs={**_COMMON_KNOBS, "start_pose_rad": (0.0, 1.0), "bearing_deg": (0.0, 90.0)}),
    Stage("s2_goal_any_bearing", "goal_reach", "Goal 80-150 cm away at any bearing from a random start pose. Training bearings cover the gate's.",
          task_params={"goal_dist_min": 0.8, "goal_dist_max": 1.5, "bearing_deg": 180.0, "start_pose_rad": 0.6}, parent="s1b_random_start",
          gate=Gate(task_params={"goal_dist_min": 1.0, "goal_dist_max": 1.5, "bearing_deg": 180.0, "start_pose_rad": 0.6}, min_success=0.7),
          knobs={**_COMMON_KNOBS, "bearing_deg": (0.0, 180.0), "start_pose_rad": (0.0, 1.0)}),
    Stage("s3_goal_sequence", "goal_sequence", "A fresh random layout of 4 targets each pass; the head must visit t1, t2, t3, t4 in order (legs 60-100 cm, any turn between "
          "legs). The policy also sees the next target.",
          task_params={"goal_dist_min": 0.6, "goal_dist_max": 1.0, "bearing_deg": 180.0, "seq_bearing_deg": 180.0, "start_pose_rad": 0.6, "n_goals": 4},
          parent="s2_goal_any_bearing", episode_s=90.0, steps=3_000_000,
          gate=Gate(task_params={"goal_dist_min": 0.6, "goal_dist_max": 1.0, "bearing_deg": 180.0, "seq_bearing_deg": 180.0, "start_pose_rad": 0.6, "n_goals": 4},
                    min_success=0.6, episode_s=90.0),
          knobs={**_COMMON_KNOBS, "n_goals": (2, 6), "seq_bearing_deg": (30.0, 180.0), "goal_dist_max": (0.6, 1.4)}),
    Stage("s4_retarget", "goal_sequence", "Automatic retargeting: while the robot is under way the current target jumps to a new random position at random times (on average "
          "every ~10 s, never closer than 5 s apart), with 2 targets per pass. The policy must re-aim and carry on each time.",
          task_params={"goal_dist_min": 0.6, "goal_dist_max": 1.0, "bearing_deg": 180.0, "seq_bearing_deg": 180.0, "start_pose_rad": 0.6, "n_goals": 2,
                       "redirect_every_s": 10.0, "redirect_min_gap_s": 5.0},
          parent="s3_goal_sequence", episode_s=90.0,
          gate=Gate(task_params={"goal_dist_min": 0.6, "goal_dist_max": 1.0, "bearing_deg": 180.0, "seq_bearing_deg": 180.0, "start_pose_rad": 0.6, "n_goals": 2,
                                 "redirect_every_s": 8.0, "redirect_min_gap_s": 5.0}, min_success=0.6, episode_s=90.0),
          knobs={**_COMMON_KNOBS, "redirect_every_s": (6.0, 30.0), "n_goals": (1, 4), "goal_dist_max": (0.6, 1.4)}),
    Stage("s5_goal_far_robust", "goal_sequence", "Longer legs (1.5-2.2 m), 2 targets with occasional automatic moves, heavier randomisation.",
          task_params={"goal_dist_min": 1.5, "goal_dist_max": 2.2, "bearing_deg": 180.0, "seq_bearing_deg": 180.0, "start_pose_rad": 0.6, "n_goals": 2,
                       "redirect_every_s": 15.0, "redirect_min_gap_s": 6.0},
          parent="s4_retarget", dr_scale=0.6, episode_s=120.0,
          gate=Gate(task_params={"goal_dist_min": 1.5, "goal_dist_max": 2.5, "bearing_deg": 180.0, "seq_bearing_deg": 180.0, "start_pose_rad": 0.6, "n_goals": 2,
                                 "redirect_every_s": 15.0, "redirect_min_gap_s": 6.0}, dr_scale=1.0, min_success=0.6, episode_s=120.0),
          knobs={**_COMMON_KNOBS, "goal_dist_max": (1.5, 3.0)}),
    Stage("s6_wall_by_touch", "goal_wall", "Target 1.2-1.8 m ahead on the far side of a wall the policy is not told about (wall 0.6-1.0 m long across the line, random sideways offset). "
          "Only the motor-reply contact signals (torque, tracking error) reveal it, so the robot must find the way round by feel.",
          task_params={"goal_dist_min": 1.2, "goal_dist_max": 1.8, "bearing_deg": 0.0, "wall_len_min": 0.25, "wall_len_max": 0.4, "wall_offset": 0.2,
                       "contact_penalty": 0.2, "flip_penalty": 50.0},
          parent="s2_goal_any_bearing", episode_s=45.0, steps=3_000_000, ent_coef=0.01,
          gate=Gate(task_params={"goal_dist_min": 1.2, "goal_dist_max": 1.8, "bearing_deg": 20.0, "wall_len_min": 0.5, "wall_len_max": 0.9, "wall_offset": 0.3},
                    min_success=0.6, episode_s=45.0),
          knobs={**_COMMON_KNOBS, "wall_len_min": (0.2, 1.0), "wall_len_max": (0.3, 1.2), "wall_offset": (0.0, 0.5), "bearing_deg": (0.0, 40.0), "touch_off": (0.0, 1.0), "contact_penalty": (0.0, 1.0), "flip_penalty": (0.0, 200.0)}),
]}


def get_stage(name: str) -> Stage:
    """Look a stage up by name (raises with the valid names)."""
    if name not in STAGES:
        raise KeyError(f"unknown stage '{name}'; stages: {', '.join(STAGES)}")
    return STAGES[name]
