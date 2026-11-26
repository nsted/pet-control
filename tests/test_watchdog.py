"""Watchdog stop rules on synthetic snapshot histories, plus orientation diagnostics and start orientations in the env."""

from __future__ import annotations

import numpy as np

from petctl.rl.config import EnvConfig
from petctl.rl.env import PetEnv
from petctl.rl.watchdog import Snapshot, WatchdogConfig, judge, term_shares

CFG = WatchdogConfig()


def snap(steps: int, ret: float = 100.0, metric: float = 0.3, ep_len: float = 300.0, flips: float = 0.1, shares: dict | None = None, **kw: float) -> Snapshot:
    """Snapshot past the minimum window unless overridden."""
    return Snapshot(steps=steps, episodes=1000, ret=ret, metric=metric, ep_len=ep_len, flips=flips, inverted_s=0.0, shares=shares or {"progress": 0.6, "energy": 0.4}, std_min=0.3, std_max=0.4, **kw)


def test_healthy_run_not_stopped() -> None:
    hist = [snap(400_000 + i * 100_000, ret=100 + 10 * i, metric=0.3 + 0.05 * i) for i in range(6)]
    assert judge(hist, {"progress": 0.6, "energy": 0.4}, CFG) is None


def test_reward_hack_signature() -> None:
    hist = [snap(400_000 + i * 100_000, ret=100 + 30 * i, metric=0.30) for i in range(5)]
    assert "reward-hack" in (judge(hist, {}, CFG) or "")


def test_share_jump_and_new_term() -> None:
    hist = [snap(500_000, shares={"progress": 0.2, "energy": 0.8})]
    assert "share" in (judge(hist, {"progress": 0.6, "energy": 0.4}, CFG) or "")
    hist = [snap(500_000, shares={"progress": 0.3, "energy": 0.3, "flip": 0.4})]
    assert "new reward term" in (judge(hist, {"progress": 0.5, "energy": 0.5}, CFG) or "")


def test_no_progress_and_len_collapse() -> None:
    assert "no task progress" in (judge([snap(1_600_000, metric=0.0)], {}, CFG) or "")
    hist = [snap(500_000, ep_len=300), snap(600_000, ep_len=290), snap(700_000, ep_len=100)]
    assert "episode length" in (judge(hist, {}, CFG) or "")


def test_flip_rules() -> None:
    hist = [snap(400_000 + i * 100_000, metric=0.5 - 0.05 * i, flips=0.1 + 0.2 * i) for i in range(5)]
    assert "flip" in (judge(hist, {}, CFG) or "")


def test_hard_limits_apply_before_min_window() -> None:
    early = Snapshot(steps=1000, episodes=5, ret=1.0, metric=0.0, ep_len=10, flips=0, inverted_s=0, std_min=0.3, std_max=5.0)
    assert "std" in (judge([early], {}, CFG) or "")
    early = Snapshot(steps=1000, episodes=5, ret=float("nan"), metric=0.0, ep_len=10, flips=0, inverted_s=0)
    assert "NaN" in (judge([early], {}, CFG) or "")
    early = Snapshot(steps=1000, episodes=5, ret=1.0, metric=0.0, ep_len=10, flips=0, inverted_s=0, sat=0.9, std_min=0.3, std_max=0.4)
    assert "saturate" in (judge([early], {}, CFG) or "")
    assert judge([Snapshot(steps=1000, episodes=5, ret=1.0, metric=0.0, ep_len=10, flips=0, inverted_s=0, shares={"a": 1.0}, std_min=0.3, std_max=0.4)], {"a": 0.0, "b": 1.0}, CFG) is None


def test_term_shares() -> None:
    assert term_shares({"a": 3.0, "b": -1.0}) == {"a": 0.75, "b": 0.25}
    assert term_shares({"a": 0.0}) == {}


def test_start_orient_and_diagnostics_generic_length() -> None:
    for n in (10, 11):
        for orient, inv in ((0, False), (1, True), (2, True)):
            cfg = EnvConfig(task="goal_reach", n_modules=n, episode_s=1.0, action_mode="hold", task_params={"start_orient": float(orient), "allow_flip": 1.0, "goal_dist_min": 1.0, "goal_dist_max": 2.0})
            env = PetEnv(cfg)
            env.reset(seed=3)
            assert (env._up_dot() < cfg.flip_up_dot) == inv
            done, info = False, {}
            while not done:
                _, _, term, trunc, info = env.step(np.zeros(cfg.n_actions, np.float32))
                done = term or trunc
            m = info["metrics"]
            assert not term and m["orient_end_inverted"] == float(inv) and any(k.startswith("term_") for k in m)


def test_turn_obs_makes_multiturn_state_visible() -> None:
    """Without ``turn_obs`` two joint states 2 pi apart give identical observations; with it they differ (and the size grows by n_joints / 2 n_joints)."""
    from petctl.rl.obs import ObsBuilder, Snapshot, turn_count

    assert list(turn_count(np.array([0.0, 3.2, -3.2, 7.0, 13.0, -20.0]))) == [0, 1, -1, 1, 2, -3]
    for mode, mult in (("abs", 1), ("hold", 2)):
        sizes, obs = {}, {}
        for turn in (0.0, 1.0):
            cfg = EnvConfig(task="goal_reach", n_modules=10, action_mode=mode, task_params={"action_unbounded": 1.0, "turn_obs": turn, "goal_dist_min": 1.0, "goal_dist_max": 2.0})
            ob = ObsBuilder(cfg)
            q = np.full(cfg.n_joints, 0.3)
            for k, qq in enumerate((q, q + 2 * np.pi)):
                snap = Snapshot(qq, np.array([1.0, 0, 0, 0]), np.zeros(3), np.full(cfg.n_joints, 0.5), np.zeros(ob.extra_dim), target=qq.copy())
                ob.reset(snap)
                obs[(turn, k)] = ob.build(snap)
            sizes[turn] = ob.size
        assert sizes[1.0] - sizes[0.0] == mult * 9
        assert np.allclose(obs[(0.0, 0)], obs[(0.0, 1)])
        assert not np.allclose(obs[(1.0, 0)], obs[(1.0, 1)])
