#!/usr/bin/env python3
"""Slow-excitation run on ONE joint (default servo 7, the tail) and a closed-loop dynamics fit.

Drives the joint with smooth, slow commands the 50 Hz telemetry can resolve (rate-limited steps and a gentle frequency
sweep), logging every motor feedback frame at its true arrival time together with the commands as sent. Then fits, by
simulating the whole loop (``petctl/tail_fit.py``), the inertia J, viscous B and Coulomb C friction, a stiffness K of
either sign, the *effective* kp and kd the motor applies (so the units of kp/kd are measured, not assumed) and the
command delay, against both the measured position and the measured torque.

Best done with the robot hanging from the head (nothing else loads the joint). Stops a trial early, parks the joint at soft
gains and goes on if the angle leaves --window or hold jitter explodes; stops everything on the bus-current / temperature
limits or any power-manager intervention. Torques are disabled on exit.

    python scripts/excite_tail.py                                              # mock dry run + fit
    python scripts/excite_tail.py --backend robot --host 192.168.18.33 --confirm-motion
    python scripts/excite_tail.py --fit-only data/excite/excite_s7_a.json      # re-fit saved data
"""

from __future__ import annotations

import argparse
import asyncio
import datetime as dt
import json
import logging
import math
import sys
import time
from pathlib import Path
from typing import Any, Callable, Optional

import numpy as np

from petctl.backends.mock import MockBackend
from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT, RobotBackend
from petctl.backends.safety import SafetyBackend
from petctl.tail_fit import Params, Trial, fit_closed_loop, format_closed_loop
from petctl.types import RobotState, ServoCommand

TICK_S = 0.01
SAFE_KP, SAFE_KD = 0.15, 0.02
STEP_RATE_RAD_S = 3.0          # well under the backend's 6 rad/s ramp, so the motor sees the command as sent


def _parse() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--backend", choices=["mock", "robot"], default="mock")
    p.add_argument("--host", default=ROBOT_DEFAULT_HOST)
    p.add_argument("--port", type=int, default=ROBOT_DEFAULT_PORT)
    p.add_argument("--servo", type=int, default=7)
    p.add_argument("--kp", type=float, default=0.3, help="Commanded kp for the run (the fit measures the effective one)")
    p.add_argument("--kd", type=float, default=0.02)
    p.add_argument("--steps", type=float, nargs="+", default=[0.3, -0.3, 0.4, -0.4], help="Step amplitudes (rad)")
    p.add_argument("--chirps", type=float, nargs="+", default=[0.2, 1.0, 0.4, 1.4],
                   help="Pairs f0 f1 (Hz) for gentle sweeps (keep <= ~2 Hz at 50 Hz telemetry)")
    p.add_argument("--chirp-amp", type=float, default=0.15)
    p.add_argument("--hold-s", type=float, default=2.0)
    p.add_argument("--window", type=float, default=0.6)
    p.add_argument("--max-bus-a", type=float, default=4.5)
    p.add_argument("--max-temp-c", type=float, default=52.0)
    p.add_argument("--out", default=None)
    p.add_argument("--fit-only", default=None, help="Skip the run; re-fit a saved data file")
    p.add_argument("--confirm-motion", action="store_true")
    return p.parse_args()


class Tap:
    """Collects (t, theta_raw, tau, vel) feedback frames for one servo at their arrival times."""

    def __init__(self, servo: int) -> None:
        self.servo, self.rows = servo, []

    def on_frame(self, t: float, fb: Any) -> None:
        if fb.motor_id == self.servo:
            self.rows.append((t, fb.pos, fb.torque, fb.vel))

    def on_state(self, t: float, s: RobotState) -> None:
        if self.servo in s.servo_positions:
            self.rows.append((t, s.servo_positions[self.servo], s.motor_torques.get(self.servo, 0.0),
                              s.motor_velocities.get(self.servo, 0.0)))


def step_profile(amp: float, hold_s: float) -> Callable[[float], float]:
    """Rate-limited step out, hold, rate-limited return, hold."""
    ramp = abs(amp) / STEP_RATE_RAD_S
    t1, t2, t3 = 0.3, 0.3 + ramp + hold_s, 0.3 + ramp + hold_s + ramp

    def f(t: float) -> float:
        if t < 0.3:
            return 0.0
        if t < 0.3 + ramp:
            return amp * (t - 0.3) / ramp
        if t < t2:
            return amp
        if t < t3:
            return amp * (1.0 - (t - t2) / ramp)
        return 0.0
    f.duration = t3 + hold_s  # type: ignore[attr-defined]
    return f


def chirp_profile(amp: float, f0: float, f1: float, T: float = 5.0) -> Callable[[float], float]:
    def f(t: float) -> float:
        if t < 0.0 or t > T:
            return 0.0
        return amp * math.sin(2 * math.pi * (f0 * t + (f1 - f0) * t * t / (2.0 * T))) * min(1.0, t / 0.5) * min(1.0, (T - t) / 0.5)
    f.duration = T + 1.0  # type: ignore[attr-defined]
    return f


class Run:
    def __init__(self, args: argparse.Namespace, backend: Any, inner: Any, tap: Tap, polling: bool) -> None:
        self.a, self.backend, self.tap, self.polling = args, backend, tap, polling
        self.state: RobotState = RobotState.empty()
        self.ids: list[int] = []
        self.p0: dict[int, float] = {}
        self.abort = ""
        self.runaway = False
        self.max_bus = 0.0
        self.max_temp = 0.0
        self.cmd_log: list[tuple[float, float]] = []

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

    async def tick(self, kp: float, kd: float, offset: float) -> None:
        a, t0 = self.a, time.monotonic()
        self.state = await self.backend.get_state()
        if self.polling:
            self.tap.on_state(time.monotonic(), self.state)
        reason = self._check()
        if reason and not self.abort:
            self.abort = reason
        theta = self.state.servo_positions.get(a.servo, self.p0[a.servo]) - self.p0[a.servo]
        if not self.abort and abs(theta) > a.window:
            self.runaway = True
        if self.abort or self.runaway:
            kp, kd, offset = SAFE_KP, SAFE_KD, 0.0
        cmds = [ServoCommand(servo_id=i, position=self.p0[i]) for i in self.ids if i != a.servo]
        cmds.append(ServoCommand(servo_id=a.servo, position=self.p0[a.servo] + offset, kp=kp, kd=kd))
        t_send = time.monotonic()
        await self.backend.send_commands(cmds)
        self.cmd_log.append((t_send, offset))
        await asyncio.sleep(max(0.0, TICK_S - (time.monotonic() - t0)))

    async def run_for(self, seconds: float, kp: float, kd: float, profile: Optional[Callable[[float], float]] = None) -> None:
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            if self.abort or self.runaway:
                return
            await self.tick(kp, kd, profile(time.monotonic() - start) if profile else 0.0)


async def main_async(args: argparse.Namespace) -> int:
    if args.backend == "robot" and not args.confirm_motion:
        print("This drives the tail through steps and sweeps. Re-run with --confirm-motion (robot hanging from the head, "
              "a hand ready).", file=sys.stderr)
        return 2
    inner: Any = RobotBackend(host=args.host, port=args.port, auto_reconnect=False) if args.backend == "robot" else MockBackend()
    backend = SafetyBackend(inner)
    tap = Tap(args.servo)
    polling = not hasattr(inner, "set_feedback_listener")
    if not polling:
        inner.set_feedback_listener(tap.on_frame)
    out = Path(args.out or f"data/excite/excite_s{args.servo}_{dt.datetime.now():%Y%m%d_%H%M%S}.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    if not await backend.connect():
        print("Connection failed.", file=sys.stderr)
        return 1
    run = Run(args, backend, inner, tap, polling)
    trials_raw: list[dict[str, Any]] = []
    rc = 0
    try:
        run.state = await backend.get_state()
        run.ids = sorted(run.state.active_servo_ids)
        run.p0 = dict(run.state.servo_positions)
        if args.servo not in run.ids:
            print(f"servo {args.servo} not active (active: {run.ids})", file=sys.stderr)
            return 1
        deadline, calm = time.monotonic() + 15, None
        while time.monotonic() < deadline:
            await run.tick(SAFE_KP, SAFE_KD, 0.0)
            pt = run.state.power_telemetry
            if pt is None or not pt.pending_motor_ids:
                calm = calm or time.monotonic()
                if time.monotonic() - calm > 1.0:
                    break
            else:
                calm = None
        plans: list[tuple[str, Callable[[float], float]]] = [(f"step{a:+.2f}", step_profile(a, args.hold_s)) for a in args.steps]
        c = args.chirps
        plans += [(f"chirp{c[i]}-{c[i + 1]}", chirp_profile(args.chirp_amp, c[i], c[i + 1])) for i in range(0, len(c) - 1, 2)]
        print(f"Connected. Servo {args.servo}; gains kp {args.kp} kd {args.kd}; temps {run.state.motor_temperatures}. "
              f"{len(plans)} trials. Writing {out}\n", flush=True)
        for label, prof in plans:
            if run.abort:
                break
            run.runaway = False
            await run.run_for(1.2, SAFE_KP, SAFE_KD)                       # park
            await run.run_for(0.8, args.kp, args.kd)                       # settle at the run gains
            ref = [r for r in tap.rows if r[0] > time.monotonic() - 0.6]
            t_start = time.monotonic()
            run.cmd_log.clear()
            n_before = len(tap.rows)
            await run.run_for(prof.duration, args.kp, args.kd, prof)       # type: ignore[attr-defined]
            if run.abort:
                print(f"  {label}: STOPPED ({run.abort})")
                break
            if run.runaway:
                print(f"  {label}: RUNAWAY (angle left the window); parked at soft gains, skipping this trial", flush=True)
                await run.run_for(2.5, SAFE_KP, SAFE_KD)
                run.runaway = False
                continue
            theta0 = float(np.mean([r[1] for r in ref])) if ref else tap.rows[n_before][1]
            rows = [r for r in tap.rows[n_before:] if r[0] >= t_start]
            cmds = [c for c in run.cmd_log if c[0] >= t_start]
            tr = {"label": label, "t": [r[0] for r in rows], "theta": [r[1] - theta0 for r in rows],
                  "tau": [r[2] for r in rows], "vel": [r[3] for r in rows],
                  "cmd_t": [c[0] for c in cmds], "cmd": [c[1] for c in cmds]}
            trials_raw.append(tr)
            th = np.array(tr["theta"])
            print(f"  {label}: {len(rows)} frames, angle {th.min():+.3f}..{th.max():+.3f} rad, "
                  f"|tau| max {np.max(np.abs(tr['tau'])):.3f} N·m", flush=True)
        out.write_text(json.dumps({"servo": args.servo, "kp": args.kp, "kd": args.kd, "backend": args.backend,
                                   "max_bus": run.max_bus, "max_temp": run.max_temp, "trials": trials_raw}))
        await run.run_for(1.0, SAFE_KP, SAFE_KD)
        print(f"\nMax bus current {run.max_bus:.2f} A, max motor temperature {run.max_temp:.0f} C. Data: {out}")
        if run.abort:
            rc = 1
    finally:
        await backend.disable_torques()
        await backend.disconnect()
    if trials_raw:
        _fit_and_print(trials_raw, args.kp, args.kd)
    return rc


def _fit_and_print(trials_raw: list[dict[str, Any]], kp: float, kd: float) -> None:
    trials = [Trial(np.array(t["t"]), np.array(t["theta"]), np.array(t["tau"]), np.array(t["cmd_t"]), np.array(t["cmd"]), t["label"])
              for t in trials_raw]
    print("\nFitting the closed loop (this takes a minute) ...", flush=True)
    fit = fit_closed_loop(trials, Params(J=6e-4, B=0.005, C=0.004, K=0.0, kp=kp, kd=kd, delay=0.02))
    if fit is None:
        print("Not enough data to fit.")
        return
    print(format_closed_loop(fit, kp, kd))
    for tr, e in zip(trials, fit.per_trial_theta_nrmse):
        print(f"    trial {tr.label:14s} position error {100 * e:5.1f}% of its travel")


def main() -> int:
    logging.basicConfig(level=logging.WARNING, format="%(asctime)s.%(msecs)03d  %(message)s", datefmt="%H:%M:%S")
    args = _parse()
    if args.fit_only:
        d = json.loads(Path(args.fit_only).read_text())
        _fit_and_print(d["trials"], d["kp"], d["kd"])
        return 0
    try:
        return asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\nInterrupted - torques disabled.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
