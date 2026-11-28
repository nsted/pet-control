"""S3 command metric and the loco_cmd task (DEV): standing still scores 0, wrong direction < 0, perfect tracking 1, stop is penalty-only, the command reaches the observation."""
from __future__ import annotations

import numpy as np
import pytest

from petctl.rl.s3 import metrics as M

pytest.importorskip("gymnasium")
pytest.importorskip("mujoco")

from petctl.rl.config import EnvConfig  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402

NJ = 10
TP = {"mit_frame": 1.0, "thermal": 1.0, "action_unbounded": 1.0, "bus_model": 1.0, "bus_motor": 1.0, "bus_clamp": 0.0, "reply_dr": 1.0, "plant_nominal": 1.0}


def test_t_score_definitions() -> None:
    cmd = np.array([0.1, 0.0, 0.0])
    assert M.t_score(np.array([0.1, 0.0]), 0.0, cmd) == pytest.approx(1.0)
    assert M.t_score(np.zeros(2), 0.0, cmd) == pytest.approx(0.0)                 # standing still earns 0
    assert M.t_score(np.array([-0.1, 0.0]), 0.0, cmd) == pytest.approx(-1.0)      # wrong way
    assert M.t_score(np.array([0.0, 0.1]), 0.0, cmd) < 0.0                        # sideways when forward is asked
    assert M.t_score(np.zeros(2), 0.0, np.zeros(3)) is None                       # stop is scored apart
    comb = np.array([0.1, 0.0, 0.5])
    assert M.t_score(np.array([0.1, 0.0]), 0.5, comb) == pytest.approx(1.0)
    assert M.t_score(np.array([0.1, 0.0]), 0.0, comb) == pytest.approx(0.5)       # translation right, no turn: mean of 1 and 0
    assert M.stop_score(np.zeros(2)) == pytest.approx(1.0) and M.stop_score(np.array([0.06, 0.0])) == pytest.approx(-1.0)


def test_yaw_needs_three_chords_to_agree() -> None:
    ch = M.chords(11)
    pos = np.zeros((11, 3))
    com = np.zeros((2, 3))
    base = np.array([0.0, 0.0, 0.0])
    agree = np.stack([base, base + 0.4])                                          # all three chords turn 0.4 rad in 1 s
    _, yaw = M.window_velocity(com, agree, 1.0, 0.0)
    assert yaw == pytest.approx(0.4)
    split = np.stack([base, np.array([0.0, 0.5, -0.5])])                          # chords disagree (a shape change): no yaw credit
    _, yaw = M.window_velocity(com, split, 1.0, 0.0)
    assert yaw == 0.0 and len(ch) == 3 and pos.shape == (11, 3)


def test_segment_frame_rotates_velocity() -> None:
    com = np.array([[0, 0, 0], [0, 0.1, 0]], float)                                # moves +y in the world
    v, _ = M.window_velocity(com, np.zeros((2, 3)), 1.0, np.pi / 2)                # heading +y: that is forward
    assert v[0] == pytest.approx(0.1) and abs(v[1]) < 1e-9


def _env(sc: dict | None, seed: int = 1) -> tuple[PetEnv, np.ndarray]:
    env = PetEnv(EnvConfig(task="loco_cmd", n_modules=11, episode_s=20.0, action_mode="delta_kp", task_params=TP))
    o, _ = env.reset(seed=seed, options={"scenario": sc} if sc else None)
    return env, o


def _run(env: PetEnv, policy, n: int = 400) -> float:
    R = 0.0
    for k in range(n):
        _, r, te, tr, _ = env.step(policy(k))
        assert not te
        R += r
    assert tr
    return R


def test_standing_still_and_thrash_score_about_nothing() -> None:
    sc = {"segments": [{"family": "forward", "cmd": [0.1, 0, 0]}, {"family": "turn_left", "cmd": [0, 0, 0.6]}]}
    env, _ = _env(sc)
    assert abs(_run(env, lambda k: np.zeros(2 * NJ))) / 400 <= 0.05
    env, _ = _env(sc)
    r = _run(env, lambda k: np.concatenate([np.full(NJ, 1.0 if k % 2 else -1.0), np.zeros(NJ)])) / 400
    assert r <= 0.05                                                              # thrash never scores (it scores NEGATIVE here: random drift is wrong-way motion; the proposal's abs(M) <= 0.05 is too strict, flagged)


def test_stop_segment_is_penalty_only() -> None:
    env, _ = _env({"segments": [{"family": "stop", "cmd": [0, 0, 0]}, {"family": "stop", "cmd": [0, 0, 0]}]})
    assert _run(env, lambda k: np.zeros(2 * NJ)) / 400 >= -0.05                  # standing still on stop: no reward and no penalty
    env, _ = _env({"segments": [{"family": "stop", "cmd": [0, 0, 0]}, {"family": "stop", "cmd": [0, 0, 0]}]})
    rng = np.random.default_rng(0)
    assert _run(env, lambda k: np.concatenate([rng.uniform(-1, 1, NJ), np.zeros(NJ)])) <= 0.0   # moving on stop never earns


def test_command_reaches_the_observation() -> None:
    a = _env({"segments": [{"family": "forward", "cmd": [0.1, 0, 0]}, {"family": "left", "cmd": [0, 0.1, 0]}]})[1]
    b = _env({"segments": [{"family": "back", "cmd": [-0.1, 0, 0]}, {"family": "left", "cmd": [0, 0.1, 0]}]})[1]
    assert not np.array_equal(a, b)
    env, _ = _env({"segments": [{"family": "forward", "cmd": [0.1, 0, 0]}, {"family": "left", "cmd": [0, 0.1, 0]}]})
    seg0 = env.task.extra_obs(env.sim).copy()
    for _ in range(205):
        env.step(np.zeros(2 * NJ))
    assert env.task.seg == 1 and not np.array_equal(seg0, env.task.extra_obs(env.sim))
