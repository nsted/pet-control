"""Capacitive face-sensor model, ground-truth hug metric and the goal_hug task (Regimen v2 M0-C)."""
from __future__ import annotations

import os

import numpy as np
import pytest

pytest.importorskip("mujoco")
import mujoco  # noqa: E402

from petctl.rl.capacitive import CapParams, CapacitiveSensor, PostProbe, PostRead, sample_cap_params  # noqa: E402
from petctl.rl.config import DEFAULT_URDF_DIR, DomainRand, EnvConfig  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.hug import HugTracker, hug_threshold, longest_run  # noqa: E402
from petctl.rl.obs import ObsBuilder, Snapshot  # noqa: E402
from petctl.sim.faces import LEFT, MIDDLE, RIGHT, module_faces  # noqa: E402
from petctl.sim.model_builder import BuildParams, PetSim  # noqa: E402

_URDF = DEFAULT_URDF_DIR
needs_urdf = pytest.mark.skipif(not os.path.isdir(_URDF), reason="Fusion URDF export not present")


def _sim(n: int, **kw: object) -> PetSim:
    return PetSim(BuildParams(n_modules=n, urdf_dir=_URDF, dt=0.004, cap_nm=(0.5,) * (n - 1), **kw))  # type: ignore[arg-type]


def _place_post(sim: PetSim, x: float, y: float) -> None:
    """Move the (static) post to world (x, y) and recompute contacts without stepping the dynamics."""
    sim.model.geom_pos[sim.model.geom("post").id, :2] = [x, y]
    mujoco.mj_forward(sim.model, sim.data)


def _read(touch: np.ndarray | None = None, n: int = 11, pen: float = 0.0, any_c: np.ndarray | None = None) -> PostRead:
    t = np.zeros((n, 3), bool) if touch is None else touch
    return PostRead(t, t * 1.0, t.any(1) if any_c is None else any_c, pen)


# ---------------------------------------------------------------- face identification
@needs_urdf
@pytest.mark.parametrize("n", [8, 11])
def test_face_labels_on_settled_robot(n: int) -> None:
    sim = _sim(n)
    sim.step(np.zeros(n - 1), 2.0)
    faces = module_faces(sim.model, n)
    assert faces.shape == (n, 3, 3)
    for k in range(n):
        R = sim.data.xmat[sim.model.body(f"m{k}").id].reshape(3, 3)
        w = (R @ faces[k].T).T                                          # world outward normals (left, right, middle)
        assert np.allclose(np.linalg.norm(w, axis=1), 1.0)
        assert w[LEFT, 1] > 0.95 and w[RIGHT, 1] < -0.95               # left = +y, right = -y for every module (body frame, chain axis x)
        assert abs(w[MIDDLE, 2]) > 0.95 and abs(w[MIDDLE, 0]) < 0.1    # the flat facet, normal along z
        # sim geometry: even modules sit low with the flat facet on the floor (-z), odd modules high with it on top (+z)
        assert np.sign(w[MIDDLE, 2]) == (-1 if k % 2 == 0 else 1)
        assert np.all(np.abs(faces[k][:, 0]) < 0.3)                     # all three faces are perpendicular to the chain axis
        assert np.allclose(faces[k], module_faces(sim.model, n)[k])      # cached call is identical


# ---------------------------------------------------------------- post-only trigger
@needs_urdf
def test_only_post_triggers_not_floor_or_obstacles() -> None:
    n = 11
    obs = (("box", -0.05, 0.035 + 0.03, 0.0, 0.2, 0.03),)           # a box pressed against the robot's left side
    sim = _sim(n, post=(1.5, 1.5), obstacles=obs)
    sim.step(np.zeros(n - 1), 1.5)
    d, m = sim.data, sim.model
    oid = m.geom("obs0").id
    gids = {m.geom(f"g{k}").id for k in range(n)}
    assert any(int(c.geom1) == m.geom("floor").id or int(c.geom2) == m.geom("floor").id for c in d.contact[:d.ncon])          # floor contacts exist
    assert any({int(c.geom1), int(c.geom2)} & {oid} and ({int(c.geom1), int(c.geom2)} & gids) for c in d.contact[:d.ncon])    # so do obstacle contacts
    r = PostProbe.of(sim).read()
    assert not r.touch.any() and not r.any_contact.any() and not r.force.any() and r.penetration == 0.0
    s = CapacitiveSensor(n, 0.05, CapParams(p_fp=0.0), np.random.default_rng(0))
    for _ in range(40):
        t, p = s.step(r.touch, r.force)
        assert not t.any() and not p.any()


@needs_urdf
def test_post_contact_maps_to_a_face_and_force() -> None:
    n = 11
    sim = _sim(n, post=(1.5, 1.5))
    sim.step(np.zeros(n - 1), 1.5)
    y_cap = 0.035
    for side, face in ((1.0, LEFT), (-1.0, RIGHT)):
        _place_post(sim, -0.15, side * (y_cap + 0.05 - 0.003))              # 3 mm into the side cap
        r = PostProbe.of(sim).read()
        assert r.touch[:, face].any() and r.touch.sum() == r.touch[:, face].sum() and r.force.sum() > 0.0 and r.penetration > 0.0
        ks = np.flatnonzero(r.touch[:, face])
        assert all(abs(sim.data.xipos[1 + k, 0] + 0.15) < 0.08 for k in ks)
    _place_post(sim, -0.15, 0.30)
    assert not PostProbe.of(sim).read().any_contact.any()                  # moved away again: no contact, cache refreshed


# ---------------------------------------------------------------- sensor timing statistics
def test_sensor_latency_and_hold() -> None:
    n, dt = 4, 0.05
    p = CapParams(rate_hz=20.0, latency_s=0.10, p_drop=0.0, p_fn=0.0, p_fp=0.0, pressure_noise=0.0)
    s = CapacitiveSensor(n, dt, p, np.random.default_rng(1))
    on = np.zeros((n, 3), bool)
    on[1, 2] = True
    seen = []
    for t in range(1, 30):
        out, pr = s.step(on if t >= 10 else ~on & False, on * 1.0)
        seen.append(float(out[1, 2]))
    assert seen.index(1.0) + 1 == 10 + 2                           # 100 ms latency = 2 ticks at 20 Hz
    s = CapacitiveSensor(n, dt, CapParams(rate_hz=10.0, latency_s=0.0, p_drop=0.0, p_fn=0.0, p_fp=0.0), np.random.default_rng(2))
    outs = [float(s.step(on if t % 4 < 2 else on & False, on * 1.0)[0][1, 2]) for t in range(1, 41)]
    ch = [i for i, (a, b) in enumerate(zip(outs, outs[1:])) if a != b]
    assert len(set(outs)) == 2 and all(j - i >= 2 for i, j in zip(ch, ch[1:]))                   # 10 Hz sampling: the output only changes every other tick (held in between)


def test_sensor_dropout_rate_and_hold_last() -> None:
    n = 3
    s = CapacitiveSensor(n, 0.05, CapParams(rate_hz=20.0, latency_s=0.0, p_drop=0.05, p_fn=0.0, p_fp=0.0), np.random.default_rng(3))
    on = np.ones((n, 3), bool)
    for _ in range(20000):
        s.step(on, on * 1.0)
    assert s.n_dropped / s.n_samples == pytest.approx(0.05, abs=0.01)
    s = CapacitiveSensor(n, 0.05, CapParams(rate_hz=20.0, latency_s=0.0, p_drop=0.0, p_fn=0.0, p_fp=0.0), np.random.default_rng(3))
    s.step(on, on * 1.0)
    s.params.p_drop = 1.0                                          # every later sample lost: the last value is held
    for _ in range(10):
        t, _ = s.step(on & False, on * 0.0)
        assert t.all()
    s.params.p_drop, s.params.p_fn = 0.0, 0.035                     # false negatives per face
    miss = sum(int((1 - s.step(on, on * 1.0)[0]).sum()) for _ in range(4000))
    assert miss / (4000 * n * 3) == pytest.approx(0.035, abs=0.008)


def test_sampled_params_in_range_scaled_and_pressure_zeroing() -> None:
    dr = DomainRand()
    rng = np.random.default_rng(0)
    ps = [sample_cap_params(dr, rng, True) for _ in range(2000)]
    assert all(10.0 <= p.rate_hz <= 20.0 and 0.05 <= p.latency_s <= 0.15 and 0.02 <= p.p_drop <= 0.05 and 0.02 <= p.p_fn <= 0.05 for p in ps)
    assert np.mean([p.pressure_zero for p in ps]) == pytest.approx(0.3, abs=0.04)
    assert not any(sample_cap_params(dr, rng, False).pressure_zero for _ in range(500))          # gate / eval: never
    dr0 = DomainRand(scale=0.0)
    p0 = sample_cap_params(dr0, rng, True)
    assert p0.rate_hz == pytest.approx(15.0) and p0.latency_s == pytest.approx(0.10) and not p0.pressure_zero


# ---------------------------------------------------------------- observation block
def _cfg(**tp: float) -> EnvConfig:
    cfg = EnvConfig(task="goal_hug", n_modules=8)
    cfg.task_params = {"field_radius": 2.0, "goal_dist_min": 1.2, "goal_dist_max": 2.0, "n_obs_min": 8, "n_obs_max": 10, **tp}
    return cfg


def test_obs_block_size_order_and_off() -> None:
    base, cap = ObsBuilder(_cfg()), ObsBuilder(_cfg(capacitive_obs=1.0))
    assert cap.size - base.size == 72 and not base.capacitive
    n = 8
    snap = Snapshot(np.zeros(7), np.array([1.0, 0, 0, 0]), np.zeros(3), np.full(7, 0.164), np.zeros(4, np.float32), capacitive=np.concatenate([np.ones(24), np.full(24, 0.5)]))
    for b in (base, cap):
        b.reset(snap)
    a, c = base.build(snap), cap.build(snap)
    assert np.array_equal(c[:base.size], a)                       # existing entries unchanged, block appended last
    blk = c[base.size:]
    assert np.all(blk[:24] == 1.0) and np.all(blk[24:48] == 0.5) and np.all(blk[48:] > 0.0)       # touch, pressure, EMA
    off = ObsBuilder(_cfg(capacitive_obs=1.0, capacitive_off=1.0))
    off.reset(snap)
    for _ in range(5):
        o = off.build(snap)
    assert o.shape == c.shape and not o[base.size:].any() and np.array_equal(o[:base.size - 0][:10], o[:10])
    none = ObsBuilder(_cfg(capacitive_obs=1.0))
    none.reset(Snapshot(np.zeros(7), np.array([1.0, 0, 0, 0]), np.zeros(3), np.full(7, 0.164), np.zeros(4, np.float32)))
    assert not none.build(Snapshot(np.zeros(7), np.array([1.0, 0, 0, 0]), np.zeros(3), np.full(7, 0.164), np.zeros(4, np.float32)))[base.size:].any()    # no sensor data = zeros
    assert n == 8


@needs_urdf
def test_env_capacitive_block_zero_far_from_post_and_sizes_stable() -> None:
    plain = PetEnv(_cfg())
    cap = PetEnv(_cfg(capacitive_obs=1.0))
    assert cap.observation_space.shape[0] - plain.observation_space.shape[0] == 72
    cfg0 = EnvConfig(task="goal_obstacles")
    assert PetEnv(cfg0).observation_space.shape == (ObsBuilder(cfg0).size,)
    obs, _ = cap.reset(seed=3)
    cap.cap_sensor.params.p_fp = 0.0
    for _ in range(10):
        obs, *_ = cap.step(np.zeros(7))
    assert not obs[plain.observation_space.shape[0]:].any()      # robot far from the post, floor contact everywhere: zeros


# ---------------------------------------------------------------- hug metric
def test_hug_threshold() -> None:
    assert [hug_threshold(n) for n in (8, 11, 12, 13, 14, 15)] == [5, 5, 5, 6, 6, 6]
    assert longest_run(np.array([1, 1, 0, 1, 1, 1, 0], bool)) == (3, 3) and longest_run(np.zeros(4, bool)) == (0, 0)


def _faces(ks: list[int], n: int = 11) -> np.ndarray:
    t = np.zeros((n, 3), bool)
    t[ks, 0] = True
    return t


def test_five_contiguous_modules_make_a_hug_tick_two_do_not() -> None:
    xy, post = np.zeros((11, 2)), np.zeros(2)
    h = HugTracker(11, 20.0)
    st = [h.update(_read(_faces([2, 3, 4, 5, 6])), xy, post) for _ in range(4)]
    assert not st[0].hug_tick and not st[1].hug_tick and st[2].hug_tick                 # debounce: 3 of the last 5 ticks
    assert st[2].hug_modules == 5 and st[2].run_start == 2
    h = HugTracker(11, 20.0)
    for _ in range(30):
        s = h.update(_read(_faces([4, 5])), xy, post)
    assert s.hug_modules == 2 and not s.hug_tick
    h = HugTracker(11, 20.0)
    for _ in range(30):                                                                  # 6 touching modules but a gap: longest contiguous run is 3
        s = h.update(_read(_faces([1, 2, 3, 5, 6, 7][:3] + [5, 6]) if False else _faces([1, 2, 3, 5, 6])), xy, post)
    assert s.hug_modules == 3 and not s.hug_tick
    h = HugTracker(11, 20.0)
    for _ in range(30):                                                                  # end-cap / edge contact only (no face): never counts
        s = h.update(_read(np.zeros((11, 3), bool), any_c=np.ones(11, bool)), xy, post)
    assert s.hug_modules == 0 and not np.isnan(h.t_contact)


@pytest.mark.parametrize("hold_s", [3.0, 10.0])
def test_hold_timing(hold_s: float) -> None:
    xy, post = np.zeros((11, 2)), np.zeros(2)
    h = HugTracker(11, 20.0, hold_s=hold_s)
    need = int(round(hold_s * 20))
    for t in range(1, need + 40):
        s = h.update(_read(_faces([3, 4, 5, 6, 7])), xy, post)
        assert s.held == (s.hold_ticks >= need and t >= need + 2)    # debounce delays the first hug tick by 2 ticks
        if s.just_held:
            assert s.hold_ticks == need and t == need + 2 and h.t_hug == pytest.approx(t / 20.0)
    assert h.held
    h = HugTracker(11, 20.0, hold_s=hold_s)
    for t in range(need - 4):
        h.update(_read(_faces([3, 4, 5, 6, 7])), xy, post)
    for _ in range(5):                                                                    # a break (modules lose contact) resets the hold counter
        h.update(_read(), xy, post)
    assert not h.held and h.hold_ticks == 0


def test_wrap_angle_diagnostic() -> None:
    ang = np.deg2rad(np.linspace(0, 180, 11))
    xy = 0.085 * np.c_[np.cos(ang), np.sin(ang)]
    h = HugTracker(11, 20.0)
    for _ in range(5):
        s = h.update(_read(_faces(list(range(11)))), xy, np.zeros(2))
    assert s.wrap_deg == pytest.approx(180.0, abs=1.0) and s.hug_modules == 11


# ---------------------------------------------------------------- goal_hug task
@needs_urdf
def test_straight_body_alongside_post_is_no_hug() -> None:
    n = 11
    sim = _sim(n, post=(1.5, 1.5))
    sim.step(np.zeros(n - 1), 1.5)
    pr = PostProbe.of(sim)
    worst = 0
    for x in np.linspace(0.05, -0.38, 18):
        for side in (1.0, -1.0):
            _place_post(sim, x, side * (0.035 + 0.05 - 0.003))
            worst = max(worst, int(pr.read().touch.any(1).sum()))
            h = HugTracker(n, 20.0)
            for _ in range(10):
                s = h.update(pr.read(), sim.data.xipos[1:, :2], np.array([x, side * 0.082]))
            assert s.hug_modules <= 2 and not s.hug_tick
    assert 1 <= worst <= 2                                                  # a straight line touches a round post at one place (two modules at a joint)


@needs_urdf
def test_goal_hug_task_runs_reports_metrics_and_penetration_guard() -> None:
    env = PetEnv(_cfg(capacitive_obs=1.0))
    obs, info = env.reset(seed=4)
    assert "post" in info["privileged"] and env.task.param("post", 0.0) == 1.0
    for _ in range(3):
        obs, r, te, tr, info = env.step(np.zeros(7))
    m = info["metrics"]
    assert {"hug_success", "hug_modules", "wrap_deg", "post_force_n", "coverage_frac", "t_contact_s", "t_hug_s"} <= set(m) and m["success"] == 0.0
    assert {"approach", "coverage", "squeeze", "hold", "hug_bonus", "hug_validity", "progress"} <= set(info["terms"])
    assert np.isfinite(obs).all()
    probe = env.task._hug_probe
    probe.read = lambda: _read(n=8, pen=0.02)                                           # a clip through the post
    _, r, te, tr, info = env.step(np.zeros(7))
    assert env.task.glitch and te and info["terms"]["glitch"] < 0 and info["terms"]["hug_validity"] < 0
    env2 = PetEnv(_cfg())
    env2.reset(seed=4)
    env2.task._hug_probe.read = lambda: _read(n=8, pen=0.007)                          # 7 mm: charged by the validity term only
    _, r, te, tr, info = env2.step(np.zeros(7))
    assert info["terms"]["hug_validity"] == pytest.approx(-env2.cfg.reward.validity * 0.002) and not env2.task.glitch


@needs_urdf
def test_goal_hug_reward_terms_and_success_termination() -> None:
    env = PetEnv(_cfg(hug_hold_s=0.2, hug_h=2.0))
    env.reset(seed=4)
    t = env.task
    t._hug_probe.read = lambda: _read(_faces([3, 4, 5], 8) , n=8)
    seen: dict[str, float] = {}
    done = False
    for i in range(30):
        _, r, done, tr, info = env.step(np.zeros(7))
        for k, v in info["terms"].items():
            seen[k] = seen.get(k, 0.0) + v
        if done:
            break
    assert done and t.hug_success and info["metrics"]["success"] == 1.0 and info["metrics"]["hug_modules"] == 3.0
    assert seen["coverage"] == pytest.approx(2.0 * 3) and seen["hold"] > 0 and seen["squeeze"] > 0 and seen["hug_bonus"] > 100.0
    assert info["terms"]["hug_bonus"] > 100.0                                           # terminal bonus, speed scaled
    assert t._hug.t_contact == pytest.approx(0.05)                                      # progress stops paying from first contact


@needs_urdf
def test_scripted_pose_wraps_11_module_body_around_post() -> None:
    """M1 feasibility seed: every joint hand-set to +0.88 rad (same sign = planar curl), post at the fitted ring centre."""
    n, q = 11, np.full(10, 0.88)
    s0 = _sim(n)
    s0.reset(q)
    mujoco.mj_forward(s0.model, s0.data)
    xy = s0.data.xipos[1:, :2]
    A, b = np.c_[2 * xy, np.ones(n)], (xy ** 2).sum(1)
    c = np.linalg.lstsq(A, b, rcond=None)[0][:2]
    sim = _sim(n, post=(float(c[0]), float(c[1])))
    sim.reset(q)
    mujoco.mj_forward(sim.model, sim.data)
    h, pr, runs = HugTracker(n, 20.0), PostProbe.of(sim), []
    for _ in range(30):
        sim.step(q, 0.05)
        runs.append(h.update(pr.read(), sim.data.xipos[1:, :2], c).hug_modules)
    assert max(runs) >= 5 and sum(r >= 5 for r in runs) >= 10 and h.peak_wrap_deg > 180.0 and h.peak_force_n > 0.0
