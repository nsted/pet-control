"""STAGE_1_GATES 1.6 / 1.7: per-joint kp action with tied kd, and the log_std clamp (n = 11)."""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("gymnasium")
pytest.importorskip("mujoco")
torch = pytest.importorskip("torch")

from petctl.rl.config import EnvConfig, kp_scale  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.logstd import clamp_log_std  # noqa: E402

NJ = 10


def _env(zeta: float = 1.0, **kw: float) -> PetEnv:
    tp = {"mit_frame": 1.0, "action_unbounded": 1.0, "zeta_m": zeta, **kw}
    return PetEnv(EnvConfig(n_modules=11, episode_s=60.0, action_mode="delta_kp", task_params=tp))


def test_spaces_and_obs_history_cover_the_kp_channels() -> None:
    env = _env()
    assert env.action_space.shape == (2 * NJ,)
    base = PetEnv(EnvConfig(n_modules=11, action_mode="delta", task_params={"mit_frame": 1.0, "action_unbounded": 1.0}))
    assert env.observation_space.shape[0] - base.observation_space.shape[0] == env.cfg.history * NJ        # history holds 2 x nj entries per tick instead of nj
    env.reset(seed=0)
    o, *_ = env.step(np.zeros(2 * NJ))
    assert o.shape == env.observation_space.shape


def test_kp_channel_maps_to_scale_and_kd_is_tied() -> None:
    env = _env(zeta=1.0)
    env.reset(seed=0)
    kp0, kd0 = env.sim.gains()
    a = np.zeros(2 * NJ)
    a[NJ:] = np.linspace(-1, 1, NJ)
    env.step(a)
    kp, kd = env.sim.gains()
    s = kp_scale(np.linspace(-1, 1, NJ))
    assert np.allclose(kp, kp0 * s) and np.allclose(kd, kd0 * np.sqrt(s))
    a[NJ:] = 7.0                                              # kp channel is clipped to [-1, 1]
    env.step(a)
    assert np.allclose(env.sim.gains()[0], kp0 * 6.25)


def test_zeta_is_an_episode_parameter() -> None:
    env = _env(zeta=0.0)                                      # knob unset: drawn from [0.7, 1.4] x scale
    env.cfg.dr.scale = 1.0
    zs = []
    for seed in range(8):
        env.reset(seed=seed)
        zs.append(env._zeta_m)
    assert 0.7 <= min(zs) and max(zs) <= 1.4 and np.ptp(zs) > 0.1
    env.cfg.dr.scale = 0.0
    env.reset(seed=0)
    assert env._zeta_m == 1.0


@pytest.mark.parametrize("s_act,zeta", [(1.0, 0.5), (1.0, 2.0), (-1.0, 0.5), (-1.0, 2.0)])
def test_gain_extremes_are_stable(s_act: float, zeta: float) -> None:
    env = _env(zeta=zeta)
    env.reset(seed=1)
    rng = np.random.default_rng(0)
    a = np.zeros(2 * NJ)
    a[NJ:] = s_act
    goal = np.zeros(NJ)
    vmax = 0.0
    for k in range(int(60 / env.dt)):                          # 60 s, random target jumps every 0.5 s
        if k % int(0.5 / env.dt) == 0:
            goal = rng.uniform(-1.2, 1.2, NJ)                  # random target jumps (the delta action walks the target there; delta_kp has no angle limit, so keep the jumps bounded)
        a[:NJ] = np.clip((goal - env._tgt) / 0.25, -1.0, 1.0)
        o, r, term, trunc, _ = env.step(a)
        assert np.all(np.isfinite(o)) and np.isfinite(r)
        vmax = max(vmax, float(np.abs(env.sim.joint_vel()).max()))
    assert vmax <= 50.0
    assert sum(int(w.number) for w in env.sim.data.warning) == 0          # no MuJoCo warning (QACC / bad contact) over the 60 s


def test_log_std_clamp_bounds() -> None:
    class P:
        log_std = torch.nn.Parameter(torch.tensor([-6.0, -2.0, 0.5]))
    clamp_log_std(P)
    assert P.log_std.tolist() == [-3.0, -2.0, -1.0]
