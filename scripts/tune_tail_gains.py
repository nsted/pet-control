#!/usr/bin/env python3
"""Find stable (kp, kd) for ONE joint (default servo 7, the tail) and measure what the motor really applies.

Replaces the old ``sweep_kp_kd.py`` (raw backend, pre-encoding-fix gain ranges, no power-manager protection).
Runs through ``SafetyBackend`` and sweeps a small grid, soft to stiff, starting low. For each (kp, kd):

  1. hold still        -> angle / torque jitter (a stable joint is ~0.0005 rad / ~0.002 N·m) and chatter frequency;
  2. small step        -> overshoot, zero crossings, settle time (a real step: the command jumps by --step-rad);
  3. regression of the measured torque on (command - angle) and (-velocity) -> the *effective* kp and kd the motor
     applies, to check the gain units (the power manager's comments say they do not map linearly to N·m/rad).

A pair is "stable" if hold jitter is small and the step rings at most twice. Prints a table and the stable window per
kp; the recommendation is the stiffest stable pair with a settle time under 0.6 s. Nothing is written to the profile.

Best done with the robot hanging from the head (no floor friction to hide chatter). Stops a pair early, returns the joint
to a soft safe gain and moves to the next kp if it runs away (|angle| > --window, jitter std > --bail-std). Also stops on
bus current / temperature limits. Torques are disabled on exit.

    python scripts/tune_tail_gains.py                                         # mock dry run
    python scripts/tune_tail_gains.py --backend robot --host 192.168.18.33 --confirm-motion
"""

from __future__ import annotations

import argparse
import asyncio
import datetime as dt
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np

from petctl.backends.mock import MockBackend
from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT, RobotBackend
from petctl.backends.safety import SafetyBackend
from petctl.config import MOTOR_LIMITS
from petctl.types import RobotState, ServoCommand

TICK_S = 0.01
SAFE_KP, SAFE_KD = 0.15, 0.02          # soft gains used to park the joint between pairs


def _parse() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--backend", choices=["mock", "robot"], default="mock")
    p.add_argument("--host", default=ROBOT_DEFAULT_HOST)
    p.add_argument("--port", type=int, default=ROBOT_DEFAULT_PORT)
    p.add_argument("--servo", type=int, default=7)
    p.add_argument("--kp", type=float, nargs="+", default=[0.1, 0.2, 0.3, 0.45, 0.6, 0.8])
    p.add_argument("--kd", type=float, nargs="+", default=[0.005, 0.01, 0.02, 0.035])
    p.add_argument("--step-rad", type=float, default=0.1)
    p.add_argument("--still-s", type=float, default=2.0)
    p.add_argument("--step-s", type=float, default=1.5)
    p.add_argument("--window", type=float, default=0.35, help="Abort a pair if the angle leaves the start by this many rad")
    p.add_argument("--bail-std", type=float, default=0.05, help="Skip stiffer kd/kp once hold jitter std exceeds this (rad)")
    p.add_argument("--max-bus-a", type=float, default=4.5)
    p.add_argument("--max-temp-c", type=float, default=52.0)
    p.add_argument("--out", default=None)
    p.add_argument("--confirm-motion", action="store_true")
    return p.parse_args()


def _clip_gain(kp: float, kd: float) -> tuple[float, float]:
    return min(kp, MOTOR_LIMITS.kp_max), min(kd, MOTOR_LIMITS.kd_max)


class Tuner:
    def __init__(self, args: argparse.Namespace, backend: SafetyBackend, out: Path) -> None:
        self.a, self.backend = args, backend
        self.state: RobotState = RobotState.empty()
        self.ids: list[int] = []
        self.p0: dict[int, float] = {}
        self.rows: list[dict[str, Any]] = []
        self.fh = out.open("a")
        self.abort = ""
        self.max_bus = 0.0
        self.max_temp = 0.0

    def _check(self) -> str:
        s, a = self.state, self.a
        if not s.connected:
            return "disconnected"
        self.max_bus = max(self.max_bus, s.battery_current_amps)
        if s.battery_current_amps > a.max_bus_a:
            return f"bus current {s.battery_current_amps:.2f} A"
        temps = list(s.motor_temperatures.values())
        self.max_temp = max(self.max_temp, max(temps) if temps else 0.0)
        if temps and max(temps) > a.max_temp_c:
            return f"temperature {max(temps)} C"
        pt = s.power_telemetry
        if pt is not None and (pt.system_state != "RUNNING" or pt.motor_states.get(a.servo, "NORMAL") != "NORMAL"):
            return "power manager intervened"
        return ""

    async def tick(self, kp: float, kd: float, offset: float, tag: str) -> str:
        a, t0 = self.a, time.monotonic()
        self.state = await self.backend.get_state()
        reason = self._check()
        s = self.state
        theta = s.servo_positions.get(a.servo, 0.0) - self.p0[a.servo]
        if not reason and abs(theta) > a.window:
            reason = f"angle {theta:+.2f} rad left the window"
        if reason and not self.abort:
            self.abort = reason
        if self.abort:
            kp, kd, offset = SAFE_KP, SAFE_KD, 0.0
        kp, kd = _clip_gain(kp, kd)
        cmds = [ServoCommand(servo_id=i, position=self.p0[i]) for i in self.ids if i != a.servo]
        cmds.append(ServoCommand(servo_id=a.servo, position=self.p0[a.servo] + offset, kp=kp, kd=kd))
        await self.backend.send_commands(cmds)
        row = {"t": t0, "tag": tag, "kp": kp, "kd": kd, "cmd": offset, "theta": theta,
               "tau": s.motor_torques.get(a.servo, 0.0), "vel": s.motor_velocities.get(a.servo, 0.0),
               "bus_a": s.battery_current_amps}
        self.rows.append(row)
        self.fh.write(json.dumps(row) + "\n")
        await asyncio.sleep(max(0.0, TICK_S - (time.monotonic() - t0)))
        return self.abort

    async def phase(self, kp: float, kd: float, offset: float, seconds: float, tag: str) -> str:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if await self.tick(kp, kd, offset, tag):
                return self.abort
        return ""

    def _distinct(self, tag: str) -> list[dict[str, Any]]:
        out, last = [], None
        for r in self.rows:
            if r["tag"] != tag:
                continue
            key = (r["theta"], r["tau"], r["vel"])
            if key != last:
                out.append(r)
                last = key
        return out

    def measure(self, kp: float, kd: float) -> dict[str, Any]:
        a = self.a
        still = self._distinct(f"still_{kp}_{kd}")
        step = self._distinct(f"step_{kp}_{kd}")
        res: dict[str, Any] = {"kp": kp, "kd": kd}
        if len(still) > 10:
            th = np.array([r["theta"] for r in still]); tau = np.array([r["tau"] for r in still])
            t = np.array([r["t"] for r in still])
            d = th - th.mean(); zc = int(np.sum(np.diff(np.sign(d)) != 0))
            res.update(still_theta_std=float(th.std()), still_tau_std=float(tau.std()),
                       chatter_hz=float(zc / 2 / (t[-1] - t[0])) if t[-1] > t[0] else 0.0)
        if len(step) > 10:
            th = np.array([r["theta"] for r in step]); t = np.array([r["t"] for r in step]) - step[0]["t"]
            final = float(np.mean(th[-max(5, len(th) // 5):]))
            peak = float(np.max(th)) if a.step_rad > 0 else float(np.min(th))
            overshoot = (peak - final) / a.step_rad * 100 if a.step_rad else 0.0
            err = th - final
            cross = int(np.sum(np.diff(np.sign(err[np.abs(err) > 0.004])) != 0)) if np.sum(np.abs(err) > 0.004) > 2 else 0
            out_of_band = np.nonzero(np.abs(err) > max(0.01, 0.1 * a.step_rad))[0]
            settle = float(t[out_of_band[-1]]) if len(out_of_band) else 0.0
            e = np.array([r["cmd"] - r["theta"] for r in step]); tq = np.array([r["tau"] for r in step]); v = np.array([r["vel"] for r in step])
            M = np.column_stack([e, -v, np.ones_like(e)])
            sol = np.linalg.lstsq(M, tq, rcond=None)[0]
            res.update(final=final, overshoot_pct=float(overshoot), crossings=cross, settle_s=settle,
                       kp_eff=float(sol[0]), kd_eff=float(sol[1]))
        res["stable"] = bool(res.get("still_theta_std", 1.0) < 0.004 and res.get("still_tau_std", 1.0) < 0.012
                             and res.get("crossings", 9) <= 2)
        return res


async def main_async(args: argparse.Namespace) -> int:
    if args.backend == "robot" and not args.confirm_motion:
        print("This moves the tail with a range of gains. Re-run with --confirm-motion (robot hanging from the head, "
              "a hand ready).", file=sys.stderr)
        return 2
    inner: Any = RobotBackend(host=args.host, port=args.port, auto_reconnect=False) if args.backend == "robot" else MockBackend()
    backend = SafetyBackend(inner)
    out = Path(args.out or f"data/tune/tune_{args.backend}_{dt.datetime.now():%Y%m%d_%H%M%S}.jsonl")
    out.parent.mkdir(parents=True, exist_ok=True)
    if not await backend.connect():
        print("Connection failed.", file=sys.stderr)
        return 1
    tn = Tuner(args, backend, out)
    results: list[dict[str, Any]] = []
    rc = 0
    try:
        tn.state = await backend.get_state()
        tn.ids = sorted(tn.state.active_servo_ids)
        tn.p0 = dict(tn.state.servo_positions)
        if args.servo not in tn.ids:
            print(f"servo {args.servo} not active (active: {tn.ids})", file=sys.stderr)
            return 1
        deadline, calm = time.monotonic() + 15, None
        while time.monotonic() < deadline:                       # let the power manager stage every motor in
            await tn.tick(SAFE_KP, SAFE_KD, 0.0, "warmup")
            pt = tn.state.power_telemetry
            if pt is None or not pt.pending_motor_ids:
                calm = calm or time.monotonic()
                if time.monotonic() - calm > 1.0:
                    break
            else:
                calm = None
        print(f"Connected. Servo {args.servo}. Temps {tn.state.motor_temperatures}. Writing {out}\n")
        print("   kp     kd | still: theta std  tau std  Hz | step: overshoot  cross  settle | kp_eff  kd_eff | stable")
        for kp in args.kp:
            for kd in args.kd:
                if tn.abort:
                    break
                await tn.phase(SAFE_KP, SAFE_KD, 0.0, 0.7, "park")
                await tn.phase(kp, kd, 0.0, 0.8, "settle")
                if await tn.phase(kp, kd, 0.0, args.still_s, f"still_{kp}_{kd}"):
                    break
                if await tn.phase(kp, kd, args.step_rad, args.step_s, f"step_{kp}_{kd}"):
                    break
                await tn.phase(kp, kd, 0.0, 1.0, f"back_{kp}_{kd}")
                r = tn.measure(kp, kd)
                results.append(r)
                print(f"{kp:5.2f} {kd:6.3f} |   {r.get('still_theta_std', float('nan')):.4f}   {r.get('still_tau_std', float('nan')):.4f}  "
                      f"{r.get('chatter_hz', 0):4.1f} |   {r.get('overshoot_pct', float('nan')):5.0f}%   {r.get('crossings', -1):3d}   "
                      f"{r.get('settle_s', float('nan')):.2f}s |  {r.get('kp_eff', float('nan')):5.2f}  {r.get('kd_eff', float('nan')):.4f} | "
                      f"{'STABLE' if r['stable'] else '-'}", flush=True)
                if r.get("still_theta_std", 0) > args.bail_std:
                    print(f"       jitter above {args.bail_std} rad: skipping the rest of this kp row")
                    break
            if tn.abort:
                print(f"STOPPED: {tn.abort}")
                rc = 1
                break
        stable = [r for r in results if r["stable"] and r.get("settle_s", 9) < 0.6]
        if stable:
            best = max(stable, key=lambda r: (r["kp"], -r["kd"]))
            print(f"\nStiffest stable pair with settle < 0.6 s: kp {best['kp']}, kd {best['kd']} "
                  f"(effective kp {best['kp_eff']:.2f}, kd {best['kd_eff']:.4f}; overshoot {best['overshoot_pct']:.0f}%)")
        else:
            print("\nNo pair met the stable + settle < 0.6 s criteria; see the table.")
        ratios = [r["kp_eff"] / r["kp"] for r in results if r.get("kp_eff") and r["kp"] > 0]
        if ratios:
            print(f"Effective/nominal kp across the sweep: median {np.median(ratios):.2f} (1.0 = the units are N·m/rad as assumed).")
        print(f"Max bus current {tn.max_bus:.2f} A, max motor temperature {tn.max_temp:.0f} C.")
        out.with_suffix(".summary.json").write_text(json.dumps(results, indent=2))
        await tn.phase(SAFE_KP, SAFE_KD, 0.0, 1.0, "end")
    finally:
        tn.fh.close()
        await backend.disable_torques()
        await backend.disconnect()
        print(f"Data: {out}")
    return rc


def main() -> int:
    logging.basicConfig(level=logging.WARNING, format="%(asctime)s.%(msecs)03d  %(message)s", datefmt="%H:%M:%S")
    try:
        return asyncio.run(main_async(_parse()))
    except KeyboardInterrupt:
        print("\nInterrupted - torques disabled.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
