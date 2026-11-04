"""Free-body MuJoCo model: settles on the floor in the bench posture; hanging mode matches the analytic gravity model."""
from __future__ import annotations

import os

import numpy as np
import pytest

pytest.importorskip("mujoco")

from petctl.gravity_model import equilibrium  # noqa: E402
from petctl.sim.model_builder import BOX_HALF, BuildParams, PetSim  # noqa: E402


def test_hanging_mode_matches_gravity_model() -> None:
    p = BuildParams(hanging=True, coulomb=(0.0,) * 7, masses=(0.275,) * 7 + (1e-6,), cap_nm=(5.0,) * 7, dt=0.001, delay_s=0.0)
    sim = PetSim(p)
    cmd = np.zeros(7)
    cmd[3] = 0.3
    sim.step(cmd, 6.0)
    phi, tau = equilibrium(cmd, np.full(7, 0.275), 0.8)
    assert np.allclose(sim.joint_pos(), phi, atol=0.01)
    assert np.allclose(sim.joint_torque(), tau, atol=0.01)


def test_free_body_settles_in_bench_posture() -> None:
    sim = PetSim(BuildParams())
    sim.step(np.zeros(7), 3.0)
    d = sim.data
    assert np.all(np.isfinite(d.qpos)) and np.abs(d.qvel).max() < 0.01
    bottoms = np.array([d.geom_xpos[i][2] - BOX_HALF[2] for i in range(1, 9)])
    assert np.all(np.abs(bottoms[0::2]) < 2e-3)            # modules 0,2,4,6 on the floor
    assert np.all(bottoms[1::2] > 5e-3)                    # modules 1,3,5,7 raised
    assert np.abs(sim.joint_pos()).max() < 0.05            # no joint sagged


def test_torque_cap_limits_actuator_force() -> None:
    sim = PetSim(BuildParams(cap_nm=(0.05,) * 7))
    sim.step(np.full(7, 1.0), 0.5)
    assert np.abs(sim.joint_torque()).max() <= 0.05 + 1e-9


_URDF = "/Users/nsted/Documents/PET/body/URDF/PET_description"


@pytest.mark.skipif(not os.path.isdir(_URDF), reason="Fusion URDF export not present")
def test_mesh_model_settles_in_bench_posture_and_tail_moves() -> None:
    sim = PetSim(BuildParams(urdf_dir=_URDF, delay_s=0.0))
    sim.step(np.zeros(7), 1.5)
    low = sorted({int(c.geom2) - 1 for c in sim.data.contact[: sim.data.ncon]})
    assert low == [0, 2, 4, 6]                                   # modules 0,2,4,6 touch the floor
    assert np.all(np.isfinite(sim.data.qpos))
    t = np.zeros(7)
    t[6] = 0.4
    sim.step(t, 2.0)
    assert sim.joint_pos()[6] > 0.3                              # the tail swings freely
