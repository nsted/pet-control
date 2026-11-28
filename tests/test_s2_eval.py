"""S2 evaluator trust tests (STAGE_1_GATES 1.8 / 1.10): an analytic deflection check that does not use the evaluator, read-back assertions that can fail, and no state leakage between scenarios."""
from __future__ import annotations

import copy
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("gymnasium")
mujoco = pytest.importorskip("mujoco")

from petctl.rl.gait_eval import build_sim  # noqa: E402
from petctl.rl.s2.cells import Pose, write_cache  # noqa: E402
from petctl.rl.s2.evaluator import assert_applied, pd_policy, run_scenario  # noqa: E402
from petctl.rl.s2.scenario import S2Scenario  # noqa: E402

NJ = 10


def test_static_push_deflection_matches_jacobian_over_kp() -> None:
    """Gravity 0, contacts off, head (the free-joint root) held by a huge armature, a small force on the tail module: q = J^T F / kp within 5% (independent of the evaluator)."""
    sim = build_sim(11)
    m, d = sim.model, sim.data
    m.opt.gravity[:] = 0.0
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    m.dof_armature[:6] = 1e6                                         # root (free joint) effectively fixed
    sim.reset()
    sim.set_gains(3.2, 0.07)
    body = sim.m0_id + 10
    force = np.array([0.0, 0.2, 0.0])
    for _ in range(100):
        d.xfrc_applied[:] = 0.0
        d.xfrc_applied[body, :3] = force
        sim.step(np.zeros(NJ), 0.05)
    jacp = np.zeros((3, m.nv))
    mujoco.mj_jacBodyCom(m, d, jacp, None, body)
    tau = jacp[:, 6:6 + NJ].T @ force
    pred = tau / 3.2
    meas = sim.joint_pos()
    big = np.abs(pred) > 0.2 * np.abs(pred).max()
    assert np.abs(pred).max() > 0.01
    assert np.allclose(meas[big], pred[big], rtol=0.05, atol=2e-3), (meas, pred)


@pytest.fixture(scope="module")
def cache(tmp_path_factory: pytest.TempPathFactory) -> str:
    root = tmp_path_factory.mktemp("s2cache")
    write_cache({"straight": Pose("straight", np.zeros(NJ), None)}, [("straight", "upright")], root=Path(root))
    return str(root)


def _sc(**kw: object) -> dict:
    base = S2Scenario("t-0", "decision", "L2", "straight", "upright", "none", 123, 4.0, 3.0, plant={"friction": 1.3, "mass": 1.1, "derate": 0.9, "delay_ticks": 2, "noise_mult": 1.0, "zeta_m": 1.0,
                                                                                                    "start_t_coil": 40.0, "head_mass": 1.0}).to_dict()
    base.update(kw)
    return base


def test_applied_config_matches_the_scenario_and_a_tampered_one_fails(cache: str) -> None:
    sc = _sc()
    r = run_scenario(sc, pd_policy, fixed_kp=1.6, cache_dir=cache)
    a = r["applied"]
    assert a["floor_geom_friction"] == pytest.approx(1.3) and a["reply_delay_ticks"] == 2 and a["start_t_coil"] == pytest.approx(40.0) and a["cap_nm"] == pytest.approx(0.5 * 0.9)
    bad = copy.deepcopy(sc)
    bad["plant"]["friction"] = 0.7
    with pytest.raises(AssertionError):
        assert_applied(bad, a, a["mass_total"] / 1.1, 0.5)


def test_no_state_leaks_between_scenarios(cache: str) -> None:
    quiet = _sc(plant={"friction": 1.0, "mass": 1.0, "derate": 1.0, "delay_ticks": 1, "noise_mult": 1.0, "zeta_m": 1.0, "start_t_coil": 30.0, "head_mass": 1.0}, id="q-0")
    harsh = _sc(family="tilt", mag=1.0, tilt_axis_deg=40.0, id="h-0")
    first = run_scenario(quiet, pd_policy, fixed_kp=1.6, cache_dir=cache)
    run_scenario(harsh, pd_policy, fixed_kp=1.6, cache_dir=cache)
    again = run_scenario(quiet, pd_policy, fixed_kp=1.6, cache_dir=cache)
    for k in ("E", "E_settle", "E_dist", "I_mean", "tau_max"):
        assert first[k] == again[k]


def test_disturbance_actually_changes_the_result(cache: str) -> None:
    """A scenario family must reach the physics: a 20 deg tilt moves the pose, 'none' does not."""
    calm = run_scenario(_sc(id="c-0"), pd_policy, fixed_kp=0.4, cache_dir=cache)
    tilt = run_scenario(_sc(family="tilt", mag=1.0, tilt_axis_deg=0.0, id="t-0"), pd_policy, fixed_kp=0.4, cache_dir=cache)
    assert tilt["tau_max"] != calm["tau_max"] or tilt["E_dist"] != calm["E_dist"]
