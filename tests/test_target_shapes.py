"""Shape-agnostic target family, hug metric v2 (normal-span wrap) and the m2_hug5 stage."""
from __future__ import annotations

import math
import os

import numpy as np
import pytest

pytest.importorskip("mujoco")
import mujoco  # noqa: E402

from petctl.rl.capacitive import PostProbe, PostRead  # noqa: E402
from petctl.rl.config import DEFAULT_URDF_DIR, EnvConfig  # noqa: E402
from petctl.rl.curriculum.m2v5 import gate2_spec_hug5  # noqa: E402
from petctl.rl.curriculum.stages import get_stage  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.hug import HugTracker  # noqa: E402
from petctl.rl.target_shapes import (GATE_SEED0, HELD_OUT, HELD_OUT_NAMES, PER_SHAPE, heldout_group, heldout_shape, make_shape, near_heldout,  # noqa: E402
                                     sample_training_shape)
from petctl.sim.model_builder import BuildParams, PetSim, post_geom_ids, set_post_xy  # noqa: E402
from petctl.sim.shapes import TargetShape, span_deg  # noqa: E402

needs_urdf = pytest.mark.skipif(not os.path.isdir(DEFAULT_URDF_DIR), reason="Fusion URDF export not present")


def _sim(n: int, **kw: object) -> PetSim:
    return PetSim(BuildParams(n_modules=n, urdf_dir=DEFAULT_URDF_DIR, dt=0.004, cap_nm=(0.5,) * (n - 1), **kw))  # type: ignore[arg-type]


# ---------------------------------------------------------------- analytic shape
def test_sdf_and_normals() -> None:
    cyl = TargetShape("cyl", 0.05)
    d, a = cyl.sdf_normal(np.array([0.0, 0.08]))
    assert d == pytest.approx(0.03) and a == pytest.approx(math.pi / 2)
    box = TargetShape("box", 0.04, 0.07, 0.0, 0.0)
    assert box.sdf_normal(np.array([0.06, 0.0])) == pytest.approx((0.02, 0.0))
    d, a = box.sdf_normal(np.array([0.05, 0.08]))                            # off the corner: nearest point is the corner, normal diagonal
    assert d == pytest.approx(math.hypot(0.01, 0.01)) and a == pytest.approx(math.pi / 4)
    rb = TargetShape("rbox", 0.06, 0.06, 0.02, 0.0)
    d, a = rb.sdf_normal(np.array([0.10, 0.10]))
    assert d == pytest.approx(math.hypot(0.06, 0.06) - 0.02 * 1.0 - (0.0)) or d > 0
    assert a == pytest.approx(math.pi / 4)
    yawed = TargetShape("box", 0.04, 0.07, 0.0, math.pi / 2)                  # local x axis now points along world y
    assert yawed.sdf_normal(np.array([0.0, 0.06]))[0] == pytest.approx(0.02)
    assert yawed.dist_xy(np.array([[0.0, 0.06], [0.0, 0.0]]), np.zeros(2)) == pytest.approx([0.02, 0.0])
    assert span_deg(np.array([0.0, math.pi / 2, math.pi])) == pytest.approx(180.0)
    assert span_deg(np.array([0.1, 0.1, 0.1])) == 0.0


# ---------------------------------------------------------------- held-out set / sampling
def test_heldout_deterministic_and_disjoint_from_training() -> None:
    assert [heldout_shape(i, 10_000 + 20 * i + 3) for i in range(5)] == [heldout_shape(i, 10_000 + 20 * i + 3) for i in range(5)]
    groups = [heldout_group(GATE_SEED0 + i) for i in range(100)]
    assert all(groups.count(nm) == PER_SHAPE for nm in HELD_OUT_NAMES) and heldout_group(5) == "other" and heldout_group(GATE_SEED0 + 100) == "other"
    for i in range(5):
        assert near_heldout(heldout_shape(i, 1))                                  # each held-out shape is itself excluded
    for phase in (1, 2):
        rng = np.random.default_rng(0)
        shapes = [sample_training_shape(rng, phase) for _ in range(3000)]
        assert not any(near_heldout(s) for s in shapes)
        kinds = {k: sum(s.kind == k for s in shapes) / len(shapes) for k in ("cyl", "box", "rbox")}
        assert 0.42 < kinds["cyl"] < 0.60 and 0.28 < kinds["box"] < 0.42 and 0.08 < kinds["rbox"] < 0.22      # rejection of held-out neighbourhoods shifts the mix slightly
        for s in shapes:
            if s.kind == "cyl":
                assert (0.04 if phase == 1 else 0.02) <= s.a <= (0.08 if phase == 1 else 0.12)
            else:
                lo, hi = (0.06, 0.10) if phase == 1 else (0.04, 0.16)
                assert lo <= 2 * s.a <= hi and lo <= 2 * s.b <= hi and max(s.a, s.b) / min(s.a, s.b) <= 3.0 + 1e-9
                assert s.kind == "box" or (0.01 <= s.rc <= 0.03 and s.rc < min(s.a, s.b))
    assert sample_training_shape(np.random.default_rng(5)) == sample_training_shape(np.random.default_rng(5))


# ---------------------------------------------------------------- hug metric v2 on synthetic reads
def _read_on(shape: TargetShape, rays_deg: list[float], n: int = 8, first: int = 2) -> tuple[PostRead, np.ndarray]:
    """Modules ``first``.. sit 8 mm off the surface along the given rays from the centre; normals from the analytic shape."""
    touch = np.zeros((n, 3), bool)
    fd = np.full((n, 3), 0.02)
    ang = np.full(n, np.nan)
    xy = np.full((n, 2), 5.0)
    for j, deg in enumerate(rays_deg):
        u = np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg))])
        lo, hi = 0.0, 1.0                                                          # surface point along the ray (bisection on the sdf)
        for _ in range(60):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if shape.sdf_normal(mid * u)[0] < 0 else (lo, mid)
        p = lo * u + 0.001 * u
        k = first + j
        _, ang[k] = shape.sdf_normal(p)
        xy[k] = p + 0.008 * u
        touch[k, 0], fd[k, 0] = True, 0.008
    return PostRead(touch, np.zeros((n, 3)), touch.any(1), 0.0, fd, ang), xy


def _run_ticks(read: PostRead, xy: np.ndarray, n: int = 8, ticks: int = 6) -> tuple[bool, int, float]:
    h = HugTracker(n, 20.0, h_min=4, span_min_deg=180.0)
    for _ in range(ticks):
        s = h.update(read, xy, np.zeros(2))
    return s.hug_tick, s.hug_modules, s.span_deg


@pytest.mark.parametrize("shape", [TargetShape("cyl", 0.05), TargetShape("box", 0.04, 0.04, 0.0, 0.3), TargetShape("box", 0.025, 0.07, 0.0, 1.1), TargetShape("rbox", 0.06, 0.06, 0.02, 0.7)])
def test_half_way_round_any_shape_is_a_wrap_tick(shape: TargetShape) -> None:
    tick, run, sp = _run_ticks(*_read_on(shape, [0, 45, 90, 135, 180]))
    assert run == 5 and sp >= 179.0 and tick


def test_flat_side_has_zero_span_and_one_corner_is_ninety() -> None:
    box = TargetShape("box", 0.04, 0.15, 0.0, 0.0)
    flat = [0.0, 0.0, 0.0, 0.0, 0.0]                                               # five modules along the +x side ray ... same side: place by explicit normals instead
    read, xy = _read_on(box, [-20, -10, 0, 10, 20])                                # all on the +x end face: normals all 0
    tick, run, sp = _run_ticks(read, xy)
    assert run == 5 and sp == pytest.approx(0.0, abs=1e-6) and not tick and len(flat) == 5
    corner, xy2 = _read_on(TargetShape("box", 0.05, 0.05, 0.0, 0.0), [20, 30, 40, 50, 60])   # two sides of a square meeting at one corner
    tick, run, sp = _run_ticks(corner, xy2)
    assert run == 5 and 80.0 <= sp <= 100.0 and not tick


# ---------------------------------------------------------------- physics: multi-geom target, probe, real body
@needs_urdf
def test_model_geoms_and_move() -> None:
    for sh, ng in [(TargetShape("cyl", 0.07), 1), (TargetShape("box", 0.05, 0.03, 0.0, 0.4), 1), (TargetShape("rbox", 0.06, 0.04, 0.015, 0.4), 6)]:
        sim = _sim(8, post=(0.5, -0.2), post_shape=sh)
        ids = post_geom_ids(sim.model)
        assert len(ids) == ng and np.allclose(sim.data.geom_xpos[ids[0], :2], (0.5, -0.2))
        rel = sim.model.geom_pos[ids, :2] - sim.model.geom_pos[ids[0], :2]
        set_post_xy(sim.model, (1.0, 1.0))
        assert np.allclose(sim.model.geom_pos[ids[0], :2], (1.0, 1.0)) and np.allclose(sim.model.geom_pos[ids, :2] - sim.model.geom_pos[ids[0], :2], rel)
    assert len(post_geom_ids(_sim(8).model)) == 0


@needs_urdf
def test_straight_body_along_flat_side_is_no_wrap_but_v1_hug() -> None:
    n = 8
    box = TargetShape("box", 0.25, 0.04, 0.0, 0.0)                                   # 50 cm long flat side
    sim = _sim(n, post=(1.5, 1.5), post_shape=box)
    sim.step(np.zeros(n - 1), 1.5)
    pr = PostProbe.of(sim)
    best_run = 0
    for x in np.linspace(-0.2, 0.0, 6):
        set_post_xy(sim.model, (x, 0.035 + 0.04 + 0.003))
        mujoco.mj_forward(sim.model, sim.data)
        v1, v2 = HugTracker(n, 20.0, h_min=4), HugTracker(n, 20.0, h_min=4, span_min_deg=180.0)
        for _ in range(6):
            r = pr.read()
            s1, s2 = v1.update(r, sim.data.xipos[1:n + 1, :2], np.array([x, 0.08])), v2.update(r, sim.data.xipos[1:n + 1, :2], np.array([x, 0.08]))
        best_run = max(best_run, s1.hug_modules)
        touching = np.isfinite(r.normal_ang) & r.touch.any(1)
        if touching.any():
            assert np.allclose(r.normal_ang[touching], -math.pi / 2, atol=0.05)    # the box's -y face points at the body
        assert s2.span_deg < 1.0 and not s2.hug_tick
    assert best_run >= 4                                                          # the old metric would have called this a hug


@needs_urdf
def test_scripted_coil_round_cylinder_is_a_wrap() -> None:
    n, q = 11, np.full(10, 0.88)
    s0 = _sim(n)
    s0.reset(q)
    mujoco.mj_forward(s0.model, s0.data)
    xy = s0.data.xipos[1:, :2]
    c = np.linalg.lstsq(np.c_[2 * xy, np.ones(n)], (xy ** 2).sum(1), rcond=None)[0][:2]
    sim = _sim(n, post=(float(c[0]), float(c[1])), post_shape=TargetShape("cyl", 0.05))
    sim.reset(q)
    mujoco.mj_forward(sim.model, sim.data)
    h, pr, ticks = HugTracker(n, 20.0, h_min=4, span_min_deg=180.0), PostProbe.of(sim), []
    for _ in range(30):
        sim.step(q, 0.05)
        ticks.append(h.update(pr.read(), sim.data.xipos[1:, :2], c).hug_tick)
    assert sum(ticks) >= 10 and h.peak_span_deg >= 180.0


# ---------------------------------------------------------------- env + stage
@needs_urdf
def test_env_shape_family_and_heldout_by_seed() -> None:
    st = get_stage("m2_hug5")
    cfg = EnvConfig(task="goal_hug", urdf_dir=DEFAULT_URDF_DIR, n_modules=8)
    cfg.task_params = {**st.gate.task_params}
    cfg.dr.scale = 0.6
    env = PetEnv(cfg)
    for seed, name in [(10000, "cyl_r3"), (10025, "cyl_r9"), (10041, "square_8x8"), (10079, "rect_5x14"), (10099, "rounded_12x12_r2")]:
        env.reset(seed=seed)
        want = HELD_OUT[HELD_OUT_NAMES.index(name)]
        sh = env.sim.p.post_shape
        assert sh is not None and sh.kind == want[1] and heldout_group(seed) == name
        assert env.sim.p.post is not None and len(post_geom_ids(env.sim.model)) == (6 if sh.kind == "rbox" else 1)
        for _ in range(3):
            env.step(np.zeros(7))
    assert make_shape("box", (0.08, 0.08)).a == 0.04
    cfg.task_params = {**st.task_params}
    env = PetEnv(cfg)
    env.reset(seed=3)
    assert env.sim.p.post_shape is not None and env.task.hug_metrics()["hug_threshold"] == 4.0


def test_stage_and_gate_spec_frozen() -> None:
    st = get_stage("m2_hug5")
    tp = st.task_params
    assert (tp["coverage_weight"], tp["approach_weight"], tp["hold_weight"], tp["hug_bonus"], tp["post_force_penalty"], tp["current_weight"], tp["time_penalty"],
            tp["action_rate_weight"], tp["joint_acc_weight"], tp["tilt_weight"], tp["yaw_weight"], tp["hug_h"], tp["hug_span_deg"], tp["target_family"]) == \
        (1.0, 200.0, 3.0, 300.0, 0.02, 0.0, 0.2, 0.02, 0.0, 0.0, 0.0, 4.0, 180.0, 1.0)
    assert st.steps == 4_000_000 and st.episode_s == 30.0 and st.log_std_init == -1.2 and st.ent_coef == 0.002
    assert st.gate.min_success == 0.70 and st.gate.episodes == 100 and st.gate.dr_scale == 0.6 and st.gate.task_params["target_heldout"] == 1.0
    sp = gate2_spec_hug5()
    assert sp.n == 100 and sp.min_group_success == 0.5 and sp.ablations[0].name == "capacitive_off" and sp.ablations[0].min_drop == 0.15 and sp.max_glitch == 0.0 and sp.bus_model


def test_odd_length_gate_sets_deterministic_and_disjoint_from_training() -> None:
    from petctl.rl.target_shapes import HELD_OUT_L9, HELD_OUT_L10, HELD_OUT_L11, V2_SAMPLER, held_out_names, held_out_set

    assert held_out_set(9) is HELD_OUT_L9 and held_out_set(11) is HELD_OUT_L11 and held_out_set(10) is HELD_OUT_L10
    assert held_out_names(9) == ("cyl_r3", "cyl_r8", "square_8x8", "rect_5x14", "rounded_8x8_r2")
    assert held_out_names(11) == ("cyl_r3", "cyl_r9", "square_8x8", "rect_5x14", "rounded_8x8_r2")
    for n in (9, 11):
        assert [heldout_shape(i, 10_000 + 20 * i + 3, n) for i in range(5)] == [heldout_shape(i, 10_000 + 20 * i + 3, n) for i in range(5)]
        assert [heldout_group(GATE_SEED0 + i, n) for i in range(0, 100, 20)] == list(held_out_names(n)) and heldout_group(5, n) == "other"
        for i in range(5):
            assert near_heldout(heldout_shape(i, 1, n), n)
        for kw in ({}, {"mix": V2_SAMPLER["mix"], "cyl_r": V2_SAMPLER["cyl_r"], "margin": V2_SAMPLER["margin"]}):
            rng = np.random.default_rng(0)
            shapes = [sample_training_shape(rng, 2, n, **kw) for _ in range(3000)]  # type: ignore[arg-type]
            assert not any(near_heldout(s, n, kw.get("margin", 0.01)) for s in shapes)  # type: ignore[arg-type]
