"""Offline analysis of a ``capture_all.py`` run: one results table per joint (and a static-pose current table).

Usage: python scripts/analyze_capture.py data/capture/capture_robot_YYYYmmdd_HHMMSS [--md out.md]

Per joint: reach at the torque threshold, Coulomb friction (half the torque gap between the up and down branches of the
slow ramp at equal angle), peak speed, tracking delay (chirp cross-correlation + step 10 % delay), effective gains
(least-squares  tau = kp*(cmd-theta) - kd*dtheta/dt + Fc*sign(dtheta/dt) + c  over the step/chirp phases) and bus current
vs the power manager's model (fitted slope of I against sum(tau^2) in the joint's own ramp phases).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from petctl.config import POWER_BUDGET

N = 7


def model_current(tau: np.ndarray, vel: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Power manager's bus-current model (no worst-case floor), summed over joints. tau/vel shape (T, 7), v shape (T,)."""
    b = POWER_BUDGET
    vv = np.maximum(v, 8.0)[:, None]
    per = (b.per_motor_base_a + b.per_motor_torque_coeff * tau ** 2 * (b.bus_voltage_nominal_v / vv)
           + b.per_motor_mech_coeff * np.abs(tau * vel) / vv)
    return per.sum(axis=1)


class Run:
    def __init__(self, base: Path) -> None:
        d = np.load(f"{base}.npz")
        self.d = {k: d[k] for k in d.files}
        self.meta = json.loads(Path(f"{base}.json").read_text())
        self.t = self.d["t"]
        self.marks = {m["name"]: m for m in self.meta["marks"] if "t1" in m}
        self.dt = float(np.median(np.diff(self.t)))

    def sl(self, name: str) -> slice | None:
        m = self.marks.get(name)
        if m is None:
            return None
        i0, i1 = np.searchsorted(self.t, [m["t0"], m["t1"]])
        return slice(int(i0), int(i1)) if i1 - i0 > 5 else None

    def names(self, prefix: str) -> list[str]:
        return [n for n in self.marks if n.startswith(prefix)]


def smooth_diff(x: np.ndarray, dt: float, w: int = 7) -> np.ndarray:
    k = np.ones(w) / w
    return np.gradient(np.convolve(x, k, mode="same"), dt)


def friction(r: Run, j: int) -> tuple[float, float]:
    """Coulomb friction (N.m) from up/down ramp branches at equal angle; and mean tau slope vs angle (N.m/rad)."""
    th, tau = r.d["theta"][:, j - 1], r.d["tau"][:, j - 1]
    fc, slopes = [], []
    for lab in ("pos", "neg"):
        up, dn = r.sl(f"ramp{j}_{lab}_up"), r.sl(f"ramp{j}_{lab}_down")
        if up is None or dn is None:
            continue
        a_up, t_up, a_dn, t_dn = th[up], tau[up], th[dn], tau[dn]
        lo, hi = max(a_up.min(), a_dn.min()), min(a_up.max(), a_dn.max())
        if hi - lo < 0.02:
            continue
        g = np.linspace(lo, hi, 50)
        o_up, o_dn = np.argsort(a_up), np.argsort(a_dn)
        tu = np.interp(g, a_up[o_up], t_up[o_up])
        td = np.interp(g, a_dn[o_dn], t_dn[o_dn])
        fc.append(float(np.mean(np.abs(tu - td)) / 2))
        slopes.append(float(np.polyfit(g, 0.5 * (tu + td), 1)[0]))
    return (float(np.mean(fc)) if fc else np.nan, float(np.mean(np.abs(slopes))) if slopes else np.nan)


def chirp_delay(r: Run, j: int) -> float:
    s = r.sl(f"chirp{j}")
    if s is None:
        return np.nan
    c, th = r.d["cmd"][s, j - 1], r.d["theta"][s, j - 1]
    c, th = c - c.mean(), th - th.mean()
    lags = np.arange(0, 31)
    xc = [float(np.dot(c[: len(c) - L], th[L:])) for L in lags]
    return float(lags[int(np.argmax(xc))] * r.dt)


def step_delay(r: Run, j: int) -> float:
    out = []
    for n in r.names(f"step{j}_"):
        s = r.sl(n)
        if s is None:
            continue
        c, th = r.d["cmd"][s, j - 1], r.d["theta"][s, j - 1]
        peak = c[np.argmax(np.abs(c))]
        if abs(peak) < 0.05:
            continue
        i_c = np.argmax(np.abs(c) > 0.1 * abs(peak))
        resp = th * np.sign(peak)
        i_t = np.argmax(resp > 0.1 * abs(peak))
        if resp.max() > 0.1 * abs(peak):
            out.append((i_t - i_c) * r.dt)
    return float(np.mean(out)) if out else np.nan


def gains(r: Run, j: int) -> tuple[float, float, float, float]:
    rows = []
    for n in r.names(f"step{j}_") + r.names(f"chirp{j}"):
        s = r.sl(n)
        if s is None:
            continue
        th, c, tau = r.d["theta"][s, j - 1], r.d["cmd"][s, j - 1], r.d["tau"][s, j - 1]
        v = smooth_diff(th, r.dt)
        rows.append(np.column_stack([c - th, -v, np.tanh(v / 0.05), np.ones_like(v), tau]))
    if not rows:
        return (np.nan,) * 4
    A = np.vstack(rows)
    x, *_ = np.linalg.lstsq(A[:, :4], A[:, 4], rcond=None)
    return float(x[0]), float(x[1]), float(x[2]), float(x[3])


def peak_speed(r: Run, j: int) -> float:
    best = 0.0
    for n in r.names(f"fast{j}_") + r.names(f"step{j}_"):
        s = r.sl(n)
        if s is None:
            continue
        best = max(best, float(np.max(np.abs(smooth_diff(r.d["theta"][s, j - 1], r.dt)))))
    return best


def bus_fit(r: Run, j: int) -> tuple[float, float, float]:
    """(offset A, fitted A/(N.m)^2 slope, model/measured ratio) from the joint's own slow ramps."""
    idx = []
    for n in r.names(f"ramp{j}_"):
        s = r.sl(n)
        if s is not None:
            idx.append(np.arange(s.start, s.stop))
    if not idx:
        return (np.nan,) * 3
    i = np.concatenate(idx)
    x = np.sum(r.d["tau"][i] ** 2, axis=1)
    y = r.d["bus_a"][i]
    if np.ptp(x) < 1e-5:
        return (np.nan,) * 3
    slope, off = np.polyfit(x, y, 1)
    mod = model_current(r.d["tau"][i], r.d["vel"][i], r.d["bus_v"][i])
    return float(off), float(slope), float(np.mean(mod - mod.min()) / max(1e-9, np.mean(y - y.min())))


def static_poses(r: Run) -> list[dict[str, Any]]:
    out = []
    for n in r.names("pose_") + ["A_baseline"]:
        s = r.sl(n)
        if s is None:
            continue
        half = slice(s.start + (s.stop - s.start) * 2 // 3, s.stop)       # last third: settled
        tau, vel, v = r.d["tau"][half], r.d["vel"][half], r.d["bus_v"][half]
        out.append({"pose": n, "meas_a": float(np.mean(r.d["bus_a"][half])), "model_a": float(np.mean(model_current(tau, vel, v))),
                    "sum_tau2": float(np.mean(np.sum(tau ** 2, axis=1))), "max_tau": float(np.max(np.abs(tau.mean(axis=0))))})
    return out


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("base")
    p.add_argument("--md", default=None)
    a = p.parse_args()
    r = Run(Path(a.base))
    L: list[str] = []
    L.append(f"Run {a.base}: abort='{r.meta['abort']}', max bus {r.meta['max_bus']:.2f} A, max temp {r.meta['max_temp']:.0f} C, "
             f"{len(r.t)} ticks at {1 / r.dt:.0f} Hz\n")
    b = POWER_BUDGET
    L.append(f"Power model: base {b.per_motor_base_a} A + {b.per_motor_torque_coeff} A/(N.m)^2 * tau^2 + mech {b.per_motor_mech_coeff}\n")
    L.append("| servo | reach+ rad | reach- rad | Coulomb N.m | tau slope N.m/rad | peak speed rad/s | delay chirp ms | delay step ms "
             "| kp_eff | kd_eff | Fc_fit N.m | bias N.m | bus off A | A/(N.m)^2 fit | model/meas |")
    L.append("|" + "---|" * 15)
    reach = r.meta.get("reach_measured") or [None] * N
    for j in range(1, N + 1):
        fc, sl = friction(r, j)
        kp, kd, fcf, c0 = gains(r, j)
        off, slope, ratio = bus_fit(r, j)
        rp = [float(np.max(r.d["cmd"][r.sl(f"ramp{j}_{lab}_up"), j - 1] * s_)) if r.sl(f"ramp{j}_{lab}_up") is not None else np.nan
              for lab, s_ in (("pos", 1), ("neg", -1))]
        L.append(f"| {j} | {rp[0]:.2f} | {rp[1]:.2f} | {fc:.3f} | {sl:.2f} | {peak_speed(r, j):.1f} | {chirp_delay(r, j) * 1e3:.0f} | "
                 f"{step_delay(r, j) * 1e3:.0f} | {kp:.2f} | {kd:.3f} | {fcf:.3f} | {c0:+.3f} | {off:.2f} | {slope:.1f} | {ratio:.2f} |")
    L.append("\nStatic whole-body poses (last third of each hold): bus current measured vs power-manager model\n")
    L.append("| pose | measured A | model A | model/measured (above idle) | mean sum tau^2 | max |tau| N.m |")
    L.append("|---|---|---|---|---|---|")
    sp = static_poses(r)
    idle = next((x for x in sp if x["pose"] == "A_baseline"), None)
    for x in sp:
        ratio = (x["model_a"] - (idle["model_a"] if idle else 0)) / max(1e-9, x["meas_a"] - (idle["meas_a"] if idle else 0)) \
            if x is not idle else float("nan")
        L.append(f"| {x['pose']} | {x['meas_a']:.2f} | {x['model_a']:.2f} | {ratio:.2f} | {x['sum_tau2']:.4f} | {x['max_tau']:.3f} |")
    txt = "\n".join(L)
    print(txt)
    if a.md:
        Path(a.md).write_text(txt + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
