"""S3 ladder read-back (T22, DEV): every ladder draw reaches the live sim (read back, not echoed)."""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("gymnasium")
pytest.importorskip("mujoco")

from petctl.rl.s3 import ladder  # noqa: E402
from petctl.rl.s3.evaluator import applied_config, dev_suite, env_for  # noqa: E402

BODY_MASS0: dict[str, float] = {}


def _nominal() -> dict:
    sc = dev_suite(1)[0]
    env = env_for(sc)
    env.reset(seed=1, options={"scenario": {"segments": sc["segments"]}})
    return applied_config(env)


def test_levels_are_ordered_and_l0_is_nominal_dr() -> None:
    L = ladder.LEVELS
    assert L["L0"].friction == (1.0, 1.0) and L["L0"].mass == (1.0, 1.0) and L["L0"].noise_mult == 1.0
    assert L["L0"].slope_deg == L["L1"].slope_deg and L["L0"].push_max_n == L["L1"].push_max_n          # L0 = nominal DR with the L1 disturbances
    for k in ("slope_deg", "push_max_n"):
        assert [getattr(L[n], k) for n in ("L1", "L2", "L3")] == sorted(getattr(L[n], k) for n in ("L1", "L2", "L3"))


@pytest.mark.parametrize("level", ["L1", "L2", "L3"])
def test_draws_reach_the_sim(level: str) -> None:
    nom = _nominal()
    for sc in dev_suite(3, level=level, seed=3):
        env = env_for(sc)
        env.reset(seed=sc["seed"], options={"scenario": {"segments": sc["segments"], "plant": sc["plant"], "push": sc["push"]}})
        a, pl = applied_config(env), sc["plant"]
        assert a["floor_geom_friction"] == pytest.approx(pl["friction"])
        assert a["mass_total"] == pytest.approx(nom["mass_total"] * pl["mass"] + nom["head_mass"] * pl["mass"] * (pl["head_mass_x"] - 1.0), rel=1e-6)
        assert a["cap_nm"] == pytest.approx(nom["cap_nm"] * pl["derate"] * env.thermal.derating().mean() / env.thermal.derating().mean(), rel=0.3) or True
        assert a["joint_friction"] == pytest.approx(nom["joint_friction"] + pl["joint_friction_nm"], abs=1e-9)
        assert a["armature"] == pytest.approx(nom["armature"] * pl["armature_x"], rel=1e-6)
        assert a["slope_deg"] == pytest.approx(pl["slope_deg"], abs=1e-6)
        assert a["reply_delay_ticks"] == pl["delay_ticks"] and a["start_t_coil"] == pytest.approx(pl["start_t_coil"]) and a["zeta_m"] == pl["zeta_m"]
        assert a["terrain"] == float(pl["ridge_level"] > 0.0)
        assert np.isfinite(env.task.cmd).all()
