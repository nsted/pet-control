"""
Closed-loop system identification of one joint by simulation (output-error fit).

A joint under the motor's MIT-mode PD is modelled as::

    target(t)  = cmd(t - delay)                      (command offset from the hanging pose, rad)
    tau        = kp * (target - x) - kd * v          (kp, kd are the EFFECTIVE gains: units not assumed)
    J x''      = tau - B v - C sign(v) - K x         (K of either sign: gravity, springs, unstable equilibria)

The whole loop is simulated at 1 kHz and ``J, B, C, K, kp, kd, delay`` are fitted so that the simulated position **and
motor torque** match what the robot reported at its 50 Hz telemetry times. Fitting the torque as well pins the controller's
effective gains (so the units of kp/kd are measured, not assumed); the command is known exactly, so unlike an
integral-regression fit nothing has to be integrated between the coarse telemetry samples. Pure numpy (a small
Nelder-Mead optimiser; scipy is not a dependency). No I/O.

Used by ``scripts/excite_tail.py``.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import Callable, Optional, Sequence

import numpy as np

PARAM_NAMES = ("J", "B", "C", "K", "kp", "kd", "delay")
SIM_DT = 0.001
MAX_DELAY_S = 0.12


@dataclass
class Trial:
    """One recording that starts at rest at the hanging pose."""

    t: np.ndarray            # telemetry arrival times, s (any origin)
    theta: np.ndarray        # joint angle relative to the hanging pose, rad
    tau: np.ndarray          # measured motor torque, N·m
    cmd_t: np.ndarray        # times at which each command was sent, s (same clock as t)
    cmd: np.ndarray          # commanded offset as sent, rad
    label: str = ""


@dataclass
class Params:
    J: float
    B: float
    C: float
    K: float
    kp: float
    kd: float
    delay: float

    def as_vector(self) -> np.ndarray:
        return np.array([math.log(self.J), math.log(max(self.B, 1e-9)), math.log(max(self.C, 1e-9)), self.K,
                         math.log(self.kp), math.log(self.kd), _delay_to_free(self.delay)])

    @staticmethod
    def from_vector(v: np.ndarray) -> "Params":
        return Params(J=math.exp(v[0]), B=math.exp(v[1]), C=math.exp(v[2]), K=float(v[3]), kp=math.exp(v[4]),
                      kd=math.exp(v[5]), delay=_free_to_delay(v[6]))


def _free_to_delay(x: float) -> float:
    return MAX_DELAY_S / (1.0 + math.exp(-float(np.clip(x, -30, 30))))


def _delay_to_free(d: float) -> float:
    d = min(max(d, 1e-4), MAX_DELAY_S - 1e-4)
    return math.log(d / (MAX_DELAY_S - d))


def simulate(p: Params, trial: Trial) -> tuple[np.ndarray, np.ndarray]:
    """Simulate the loop and return (theta, tau) at the trial's telemetry times (linear interpolation)."""
    t0 = float(trial.t[0])
    n = int((float(trial.t[-1]) - t0) / SIM_DT) + 2
    ct = (trial.cmd_t - t0).tolist()
    cv = trial.cmd.tolist()
    x = float(trial.theta[0])
    v = 0.0
    xs = np.empty(n)
    taus = np.empty(n)
    dt = SIM_DT
    j_inv = 1.0 / p.J
    k = bisect.bisect_right
    n_cmd = len(ct)
    for i in range(n):
        t = i * dt - p.delay
        idx = k(ct, t) - 1
        target = cv[idx] if idx >= 0 else cv[0] if n_cmd else 0.0
        tau = p.kp * (target - x) - p.kd * v
        net = tau - p.K * x
        if v == 0.0 and abs(net) <= p.C:
            acc = 0.0
        else:
            s = math.copysign(1.0, v) if v != 0.0 else math.copysign(1.0, net)
            acc = (net - p.B * v - p.C * s) * j_inv
        v_new = v + acc * dt
        if v != 0.0 and v_new * v < 0.0 and abs(net) <= p.C:      # Coulomb friction stops the joint, it does not reverse it
            v_new = 0.0
        v = v_new
        x += v * dt
        xs[i] = x
        taus[i] = tau
    tt = trial.t - t0
    return np.interp(tt, np.arange(n) * dt, xs), np.interp(tt, np.arange(n) * dt, taus)


def _loss(vec: np.ndarray, trials: Sequence[Trial], sig_theta: float, sig_tau: float) -> float:
    p = Params.from_vector(vec)
    total = 0.0
    with np.errstate(over="ignore", invalid="ignore"):          # the optimiser probes absurd parameters; treat as a bad fit
        for tr in trials:
            th, ta = simulate(p, tr)
            total += float(np.sum(((th - tr.theta) / sig_theta) ** 2) + np.sum(((ta - tr.tau) / sig_tau) ** 2))
    return total / sum(len(tr.t) for tr in trials) if math.isfinite(total) else 1e12


def nelder_mead(f: Callable[[np.ndarray], float], x0: np.ndarray, step: np.ndarray, *, max_iter: int = 700, tol: float = 1e-5) -> tuple[np.ndarray, float]:
    """Plain Nelder-Mead minimiser."""
    n = len(x0)
    sim = [x0.copy()] + [x0 + np.eye(n)[i] * step[i] for i in range(n)]
    fs = [f(s) for s in sim]
    for _ in range(max_iter):
        order = np.argsort(fs)
        sim = [sim[i] for i in order]
        fs = [fs[i] for i in order]
        if abs(fs[-1] - fs[0]) < tol * (abs(fs[0]) + tol):
            break
        c = np.mean(sim[:-1], axis=0)
        xr = c + (c - sim[-1])
        fr = f(xr)
        if fr < fs[0]:
            xe = c + 2.0 * (c - sim[-1])
            fe = f(xe)
            sim[-1], fs[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < fs[-2]:
            sim[-1], fs[-1] = xr, fr
        else:
            xc = c + 0.5 * ((xr if fr < fs[-1] else sim[-1]) - c)
            fc = f(xc)
            if fc < min(fr, fs[-1]):
                sim[-1], fs[-1] = xc, fc
            else:
                sim = [sim[0]] + [sim[0] + 0.5 * (s - sim[0]) for s in sim[1:]]
                fs = [fs[0]] + [f(s) for s in sim[1:]]
    i = int(np.argmin(fs))
    return sim[i], float(fs[i])


@dataclass
class ClosedLoopFit:
    params: Params
    loss: float
    theta_rmse: float
    tau_rmse: float
    per_trial_theta_nrmse: list[float]
    spread: dict[str, float]            # relative spread of each parameter across the multi-start fits
    n_trials: int
    n_samples: int
    notes: list[str] = field(default_factory=list)


def fit_closed_loop(
    trials: Sequence[Trial], guess: Params, *, sig_theta: float = 0.01, sig_tau: float = 0.01,
    gain_scales: Sequence[float] = (1.0, 2.0, 0.5), max_iter: int = 600, polish_iter: int = 400, seed: int = 0,
) -> Optional[ClosedLoopFit]:
    """Fit all seven parameters from several starting gain scales, then polish the best.

    The effective controller gains can differ from the commanded ones by a large factor, so the search starts from
    ``guess`` with ``kp`` and ``kd`` multiplied by each of ``gain_scales``. How much the converged fits differ between
    starts is reported per parameter (``spread``): a large spread means the data do not pin that parameter down.
    """
    trials = [t for t in trials if len(t.t) >= 10]
    if not trials:
        return None
    step = np.array([0.4, 0.5, 0.5, 0.05, 0.3, 0.3, 0.6])
    f = lambda v: _loss(v, trials, sig_theta, sig_tau)  # noqa: E731
    results = []
    for scale in gain_scales:
        x0 = guess.as_vector() + np.array([0, 0, 0, 0, math.log(scale), math.log(scale), 0.0])
        results.append(nelder_mead(f, x0, step, max_iter=max_iter))
    best_v, best_loss = min(results, key=lambda r: r[1])
    polished_v, polished_loss = nelder_mead(f, best_v, step * 0.3, max_iter=polish_iter)     # escape premature stalls
    if polished_loss < best_loss:
        best_v, best_loss = polished_v, polished_loss
        results.append((polished_v, polished_loss))
    best = Params.from_vector(best_v)
    th_err, ta_err, per = [], [], []
    for tr in trials:
        th, ta = simulate(best, tr)
        th_err.append(th - tr.theta)
        ta_err.append(ta - tr.tau)
        rng_th = float(np.ptp(tr.theta))
        per.append(float(np.sqrt(np.mean((th - tr.theta) ** 2)) / rng_th) if rng_th > 1e-6 else float("nan"))
    spread = {}
    near = [Params.from_vector(v) for v, l in results if l < best_loss * 1.5 + 1e-9]
    for name in PARAM_NAMES:
        vals = np.array([getattr(q, name) for q in near])
        scale_ = max(abs(np.mean(vals)), 1e-6) if name != "K" else 0.05
        spread[name] = float((vals.max() - vals.min()) / scale_)
    notes = []
    if len(near) < len(results):
        notes.append(f"{len(results) - len(near)} of {len(results)} starts ended in a clearly worse optimum")
    return ClosedLoopFit(
        params=best, loss=best_loss,
        theta_rmse=float(np.sqrt(np.mean(np.concatenate(th_err) ** 2))),
        tau_rmse=float(np.sqrt(np.mean(np.concatenate(ta_err) ** 2))),
        per_trial_theta_nrmse=per, spread=spread, n_trials=len(trials),
        n_samples=int(sum(len(t.t) for t in trials)), notes=notes,
    )


def format_closed_loop(fit: ClosedLoopFit, nominal_kp: Optional[float] = None, nominal_kd: Optional[float] = None) -> str:
    p = fit.params
    lines = [
        f"  inertia J           = {p.J:.5f} kg·m²        (spread across starts {100 * fit.spread['J']:.0f}%)",
        f"  viscous friction B  = {p.B:.5f} N·m·s/rad    (spread {100 * fit.spread['B']:.0f}%)",
        f"  Coulomb friction C  = {p.C:.5f} N·m          (spread {100 * fit.spread['C']:.0f}%)",
        f"  stiffness K         = {p.K:+.4f} N·m/rad     (spread {100 * fit.spread['K']:.0f}% of 0.05; "
        + ("NEGATIVE = unstable equilibrium)" if p.K < 0 else "positive = restoring)"),
        f"  effective kp        = {p.kp:.3f} N·m/rad" + (f"  (commanded {nominal_kp}; ratio {p.kp / nominal_kp:.2f})" if nominal_kp else "")
        + f"  (spread {100 * fit.spread['kp']:.0f}%)",
        f"  effective kd        = {p.kd:.4f} N·m·s/rad" + (f"  (commanded {nominal_kd}; ratio {p.kd / nominal_kd:.2f})" if nominal_kd else "")
        + f"  (spread {100 * fit.spread['kd']:.0f}%)",
        f"  command delay       = {1000 * p.delay:.0f} ms        (spread {100 * fit.spread['delay']:.0f}%)",
        f"  fit quality: position rmse {fit.theta_rmse:.4f} rad, torque rmse {fit.tau_rmse:.4f} N·m over {fit.n_trials} trials, {fit.n_samples} samples",
    ]
    lines += [f"  note: {n}" for n in fit.notes]
    return "\n".join(lines)
