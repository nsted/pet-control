"""No angle limits on the joint action (owner rule 2026-11-25): with ``action_unbounded`` the policy's joint outputs are raw angle targets of any size; observed angles wrap."""

from __future__ import annotations

import numpy as np

from petctl.rl.config import EnvConfig
from petctl.rl.obs import clip_action, next_target, unbounded, wrap_angle


def _cfg(flag: float) -> EnvConfig:
    c = EnvConfig()
    c.task_params = {"action_unbounded": flag} if flag else {}
    c.action_mode, c.action_scale_rad = "hold", 1.0
    return c


def test_default_action_is_clipped_to_unit_range() -> None:
    """Without the flag every entry is clipped to [-1, 1] (existing behaviour unchanged)."""
    out = clip_action(_cfg(0.0), np.array([3.0, -2.5, 0.2, 0, 0, 0, 0, 5.0]))
    assert out.max() == 1.0 and out.min() == -1.0


def test_unbounded_action_keeps_joint_angles_and_clips_only_the_hold_channel() -> None:
    """With the flag the joint entries are untouched; the hold channel (last) is still in [-1, 1]."""
    c = _cfg(1.0)
    assert unbounded(c)
    out = clip_action(c, np.array([3.0, -2.5, 0.2, 0, 0, 0, 0, 5.0]))
    assert out[0] == 3.0 and out[1] == -2.5 and out[-1] == 1.0


def test_unbounded_target_reaches_multi_turn_angles_limited_only_by_the_slew_rate() -> None:
    """A 7 rad (more than one turn) target is reached; the per-tick slew limit (a speed limit) is the only restriction."""
    c = _cfg(1.0)
    t = np.zeros(7)
    a = np.array([7.0, -6.0, 0, 0, 0, 0, 0, -1.0])                 # hold channel -1: follow
    for _ in range(80):
        t = next_target(c, t, clip_action(c, a))
    assert t[0] > 6.0 and t[1] < -5.0
    assert float(np.max(np.abs(np.diff([next_target(c, t, clip_action(c, a))[0], t[0]])))) <= c.max_target_step_rad + 1e-9


def test_wrap_angle_identifies_full_turns() -> None:
    """A joint turned a full turn looks identical to the policy."""
    assert abs(float(wrap_angle(7.0)) - (7.0 - 2 * np.pi)) < 1e-9
    assert abs(float(wrap_angle(np.pi / 3 + 4 * np.pi)) - np.pi / 3) < 1e-9


def test_environment_drives_a_joint_beyond_one_radian_when_unbounded() -> None:
    """End to end: with the flag a large joint action moves the simulated joint well past the old +-1 rad ceiling; without it the joint stays within it."""
    from petctl.rl.env import PetEnv

    def peak(flag: float) -> float:
        c = EnvConfig()
        c.task = "goal_obstacles"
        c.task_params = {"goal_dist_min": 1.5, "goal_dist_max": 2.5, "bearing_deg": 0.0, "action_unbounded": flag}
        c.action_mode, c.action_scale_rad, c.episode_s = "hold", 1.0, 10.0
        env = PetEnv(c)
        env.reset(seed=1)
        best = 0.0
        for _ in range(100):
            env.step(np.array([2.6] + [0.0] * (c.n_joints - 1) + [-1.0], np.float32))
            best = max(best, abs(float(env.sim.joint_pos()[0])))
        return best

    assert peak(1.0) > 1.3 > 1.05 >= peak(0.0) or (peak(1.0) > 1.3 and peak(0.0) <= 1.05)
