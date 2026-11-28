"""Method-review diagnostic (S2): the yield PD + a re-entry ramp, on the 9 frozen L2 decision yield draws plus 50 diagnostic draws (not frozen, not a suite), 3 seeds. Decision draws only; section B stays unwritten.

During the overload: torque cap 0.5 x cap_eff, trigger = observed |tau| > 0.8 x cap_eff (latched until the privileged release) OR the privileged flag; targets follow the measured pose. After the release: ramp
the targets from the measured pose back to q* over 1.0 s, at best PD. Reports Y, E_rec and the return split with bootstrap CIs.

    PYTHONPATH=scripts/rl:. python scripts/rl/s2_yield_diag.py
"""

from __future__ import annotations

import json
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from petctl.rl.s2 import suite
from petctl.rl.s2.evaluator import NJ, run_scenario
from petctl.rl.s2.scenario import YIELD_MULTS, make_scenario
from s2_press import a_of_kp

RAMP_TICKS = 20
STEP = 0.25                                  # DELTA_STEP_RAD


def policy(kp: float):
    """Scripted yield PD with re-entry (see the module docstring)."""
    base = a_of_kp(kp)
    st = {"latched": False, "ramp": None, "was_on": False}

    def pol(obs, env, t):
        a = np.zeros(2 * NJ)
        a[NJ:] = base
        sim, task = env.sim, env.task
        tau_hi = float(np.abs(sim.joint_torque()).max()) > 0.8 * float(env.cap_eff().min())
        on = task.overload()
        if on and (tau_hi or st["latched"] or True):
            st["latched"] = True
        if on:                                                              # yield: cap the torque, let the target follow the measured pose
            task.policy_cap = 0.5
            a[:NJ] = np.clip((sim.joint_pos() - env._tgt) / STEP, -1, 1)
            st["was_on"] = True
        else:
            task.policy_cap = 1.0
            if st["was_on"] and st["ramp"] is None:                         # release: start the ramp from the current target
                st["ramp"] = (env._tgt.copy(), 0)
            if st["ramp"] is not None:
                start, k = st["ramp"]
                k = min(k + 1, RAMP_TICKS)
                goal = start + (task.q_star - start) * (k / RAMP_TICKS)
                a[:NJ] = np.clip((goal - env._tgt) / STEP, -1, 1)
                st["ramp"] = (start, k)
        return a
    return pol


def _job(args):
    d, kp, z = args
    r = run_scenario(d, policy(kp), fixed_kp=0.0, zeta=z, check=False)
    return {"id": d["id"], "Y": r["Y"], "E_rec": r["E_rec"], "load": r["load_mean_overload"], "return": r["return"], "terms": r["phase_terms"]}


def boot(x: np.ndarray, n: int = 5000, seed: int = 0) -> tuple[float, float, float]:
    """Mean and 95% bootstrap interval."""
    rng = np.random.default_rng(seed)
    m = np.array([rng.choice(x, len(x)).mean() for _ in range(n)])
    return float(x.mean()), float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


def main() -> int:
    """Run the diagnostic."""
    tab = json.loads(Path("data/rl/s2/press_table.json").read_text())["table"]
    draws = [s.to_dict() | {"press_n": tab[f"{s.pose}|{s.orient}"]["press_n"]} for s in suite.load("decision", "L2", "yield_draws") if f"{s.pose}|{s.orient}" in tab]
    rng = np.random.default_rng(999)
    cells = [k.split("|") for k in tab]
    extra = []
    for i in range(50):
        pose, orient = cells[i % len(cells)]
        sc = make_scenario("decision", "L2", pose, orient, "yield", i, rng, 100 + i, yield_mult=float(rng.choice(YIELD_MULTS))).to_dict()
        sc["id"] = f"diag-{i}-{pose}"
        extra.append(sc | {"press_n": tab[f"{pose}|{orient}"]["press_n"]})
    jobs = []
    for seed in range(3):
        for d in draws + extra:
            jobs.append(({**d, "seed": int(d["seed"]) + 1000 * seed}, *tab[f"{d['pose']}|{d['orient']}"]["best_pd"]))
    with Pool(8) as p:
        res = p.map(_job, jobs)
    n9 = len(draws)
    per = len(draws) + len(extra)
    dec = [r for i, r in enumerate(res) if i % per < n9]
    allr = res
    for name, rs in (("frozen decision draws (9 x 3 seeds)", dec), ("all draws (59 x 3 seeds)", allr)):
        Y = np.array([r["Y"] for r in rs])
        E = np.array([r["E_rec"] for r in rs])
        print(f"{name}: Y {boot(Y)} E_rec {boot(E)} frac E_rec > 0.05: {np.mean(E > 0.05):.2f}  mean load {np.mean([r['load'] for r in rs]):.3f}")
    first = res[:n9]
    for r in first:
        pt = r["terms"]
        print(f"{r['id']:48s} Y {r['Y']:.2f} E_rec {r['E_rec']:.3f} load {r['load']:.2f} return {r['return']:.0f} track pre/during/post {pt.get('pre:track', 0):.0f}/{pt.get('during:track', 0):.0f}/{pt.get('post:track', 0):.0f}")
    Path("data/rl/s2/yield_diag.json").write_text(json.dumps(res))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
