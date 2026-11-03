#!/usr/bin/env python3
"""Hanging-robot test: gravity stiffness, joint friction and a static check of the bus-current model.

Hold the robot up by ONE module (``--held-module``, default the head, module 0) so the rest hangs
straight down. For each requested servo the script steps that joint's command through small angles,
approached from both directions, while every other joint holds its start pose at the default gains.
At each hold it logs motor torque, joint angle, bus current and temperatures. Then it fits

  * gravity stiffness K (N·m/rad)  -> compared with the prediction from the robot's mass and geometry,
  * Coulomb friction c from the up/down hysteresis,
  * and delta_bus = a * tau^2 over the static holds (the current model the power manager uses).

Because the robot hangs from your hand, nothing is in contact with the floor, so friction is the joint's own.

SAFETY. Refuses to start if any joint's static torque is above --start-torque (the robot is not hanging
straight) or any motor is hotter than --start-temp-c. Stops, and does not go further, if: total bus current
exceeds --max-bus-a, any motor passes --max-temp-c, the joint moves faster than 0.45 rad/s or leaves
--window rad, the power manager touches the servo, or the robot disconnects. Each joint's torque is limited
to --stop-torque (default 0.10 N·m, inside the power manager's cap). Torques are disabled on exit, which just
lets the chain hang from your hand.

    python scripts/hang_test.py --servo 7                                        # mock dry run (no gravity in the mock)
    python scripts/hang_test.py --backend robot --servo 7 --confirm-motion       # tail first (lightest)
    python scripts/hang_test.py --backend robot --servo 7 6 5 --confirm-motion
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
from petctl.config import POWER_BUDGET
from petctl.hang_analysis import (
    HoldPoint,
    fit_current_coefficient,
    fit_gravity_friction,
    modules_below,
    predicted_stiffness,
    predicted_stiffness_point_masses,
)
from petctl.types import RobotState, ServoCommand

TICK_S = 0.01
MAX_SPEED_RAD_S = 0.45          # below the 0.5 rad/s velocity-feedback clip


def _parse() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--backend", choices=["mock", "robot"], default="mock")
    p.add_argument("--host", default=ROBOT_DEFAULT_HOST)
    p.add_argument("--port", type=int, default=ROBOT_DEFAULT_PORT)
    p.add_argument("--servo", type=int, nargs="+", default=[7], help="Servos to test, in order (lightest first)")
    p.add_argument("--held-module", type=int, default=0, help="Module you hold the robot by (0 = head)")
    p.add_argument("--mass-kg", type=float, default=2.2, help="Total robot mass")
    p.add_argument("--pitch-cm", type=float, default=5.1,
                   help="Distance between adjacent motor/joint axes (UNCONFIRMED: 5.1 from the viewer model, ~11.6 from the CAD)")
    p.add_argument("--com-model", choices=["motor-axis", "uniform"], default="motor-axis",
                   help="motor-axis: each module's mass sits at its motor axis (Nick); uniform: mass spread along each module")
    p.add_argument("--module-masses", type=float, nargs=8, default=None, metavar="KG",
                   help="Mass of modules 0..7 in kg (default: total mass split equally)")
    p.add_argument("--tilt-deg", type=float, default=45.0, help="Joint axis tilt from vertical-plane geometry")
    p.add_argument("--step-rad", type=float, default=0.04, help="Command increment between holds")
    p.add_argument("--max-cmd-rad", type=float, default=0.30, help="Largest command offset from the hanging pose")
    p.add_argument("--stop-torque", type=float, default=0.10, help="Stop increasing a branch once |torque| reaches this (N·m)")
    p.add_argument("--hold-s", type=float, default=1.5)
    p.add_argument("--ramp-rad-s", type=float, default=0.2,
                   help="Slew rate of the commanded angle between holds (keeps joint speed under the 0.45 rad/s guard)")
    p.add_argument("--rest-s", type=float, default=5.0, help="Rest between joints (motors cool)")
    p.add_argument("--window", type=float, default=0.6, help="Abort if the joint leaves its start angle by this many rad")
    p.add_argument("--max-bus-a", type=float, default=4.5)
    p.add_argument("--max-temp-c", type=float, default=52.0)
    p.add_argument("--start-temp-c", type=float, default=50.0, help="Refuse to start if any motor is hotter")
    p.add_argument("--start-torque", type=float, default=0.05, help="Refuse to start if any static torque exceeds this (N·m)")
    p.add_argument("--out", default=None)
    p.add_argument("--confirm-motion", action="store_true",
                   help="Required with --backend robot: acknowledges the joints will move and hold against gravity")
    return p.parse_args()


class Rig:
    def __init__(self, args: argparse.Namespace, backend: SafetyBackend, out: Path) -> None:
        self.args, self.backend = args, backend
        self.state: RobotState = RobotState.empty()
        self.ids: list[int] = []
        self.p0: dict[int, float] = {}
        self.fh = out.open("a")
        self.rows: list[dict[str, Any]] = []
        self.abort = ""
        self.max_bus = 0.0
        self.max_temp = 0.0
        self._over_since: Optional[float] = None
        self._cmd_now = 0.0          # commanded offset of the joint under test, slewed towards its target

    def _check(self, servo: Optional[int]) -> str:
        a, s = self.args, self.state
        if not s.connected:
            return "disconnected"
        cur = s.battery_current_amps
        self.max_bus = max(self.max_bus, cur)
        if cur > a.max_bus_a:
            self._over_since = self._over_since or time.monotonic()
            if time.monotonic() - self._over_since > 0.1:
                return f"bus current {cur:.2f} A > {a.max_bus_a} A"
        else:
            self._over_since = None
        temps = list(s.motor_temperatures.values())
        tmax = max(temps) if temps else 0.0
        self.max_temp = max(self.max_temp, tmax)
        if tmax > a.max_temp_c:
            return f"motor temperature {tmax:.0f} C > {a.max_temp_c} C"
        pt = s.power_telemetry
        if pt is not None and (pt.system_state != "RUNNING" or pt.voltage_cutoff_active):
            return f"power manager state {pt.system_state}"
        if servo is not None:
            if abs(s.motor_velocities.get(servo, 0.0)) > MAX_SPEED_RAD_S:
                return f"servo {servo} speed {s.motor_velocities[servo]:+.2f} rad/s"
            if abs(s.servo_positions.get(servo, self.p0[servo]) - self.p0[servo]) > a.window:
                return f"servo {servo} left its window"
            if pt is not None and (pt.motor_states.get(servo, "NORMAL") != "NORMAL"
                                   or pt.motor_compliance_scales.get(servo, 1.0) < 1.0):
                return f"power manager touched servo {servo}"
        return ""

    async def tick(self, servo: Optional[int], cmd_offset: float, tag: str, check: bool = True) -> str:
        t0 = time.monotonic()
        self.state = await self.backend.get_state()
        reason = self._check(servo) if check else ""
        if reason and not self.abort:
            self.abort = reason
        target = 0.0 if self.abort else cmd_offset
        max_step = self.args.ramp_rad_s * TICK_S
        self._cmd_now += max(-max_step, min(max_step, target - self._cmd_now))
        offset = self._cmd_now if servo is not None else 0.0
        cmds = [ServoCommand(servo_id=i, position=self.p0[i] + (offset if i == servo else 0.0)) for i in self.ids]
        await self.backend.send_commands(cmds)
        s = self.state
        row = {"t": t0, "tag": tag, "servo": servo, "cmd": offset,
               "theta": (s.servo_positions.get(servo, 0.0) - self.p0.get(servo, 0.0)) if servo else 0.0,
               "tau": s.motor_torques.get(servo, 0.0) if servo else 0.0,
               "vel": s.motor_velocities.get(servo, 0.0) if servo else 0.0,
               "bus_a": s.battery_current_amps, "bus_v": s.battery_voltage_v,
               "temp": s.motor_temperatures.get(servo, 0) if servo else 0,
               "tau_all": {str(k): v for k, v in s.motor_torques.items()}}
        self.rows.append(row)
        self.fh.write(json.dumps(row) + "\n")
        await asyncio.sleep(max(0.0, TICK_S - (time.monotonic() - t0)))
        return self.abort

    async def hold(self, servo: Optional[int], cmd_offset: float, seconds: float, tag: str, check: bool = True) -> str:
        if servo is None:
            self._cmd_now = 0.0
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if await self.tick(servo, cmd_offset, tag, check):
                return self.abort
        return ""

    def settled(self, tag: str) -> dict[str, float]:
        rows = [r for r in self.rows if r["tag"] == tag]
        tail = rows[int(len(rows) * 0.45):]
        if not tail:
            return {}
        m = lambda k: float(np.mean([r[k] for r in tail]))  # noqa: E731
        return {"theta": m("theta"), "tau": m("tau"), "absvel": float(np.mean([abs(r["vel"]) for r in tail])),
                "bus_a": m("bus_a"), "bus_v": m("bus_v"), "temp": m("temp")}


async def sweep_joint(rig: Rig, servo: int) -> tuple[list[HoldPoint], dict[str, float]]:
    a = rig.args
    await rig.hold(servo, 0.0, 2.0, f"s{servo}_base")
    base = rig.settled(f"s{servo}_base")
    base_bus = base.get("bus_a", 0.0)
    points: list[HoldPoint] = []
    n = 0
    stats = {"max_tau": 0.0, "temp_start": base.get("temp", 0.0)}

    async def one(offset: float, direction: float) -> bool:
        nonlocal n
        tag = f"s{servo}_h{n}"
        n += 1
        travel = abs(offset - rig._cmd_now)
        if await rig.hold(servo, offset, travel / a.ramp_rad_s + a.hold_s, tag):
            return False
        s = rig.settled(tag)
        points.append(HoldPoint(theta_rad=s["theta"], tau_nm=s["tau"], direction=direction,
                                delta_bus_a=s["bus_a"] - base_bus, abs_vel_rad_s=s["absvel"]))
        stats["max_tau"] = max(stats["max_tau"], abs(s["tau"]))
        return True

    for sign in (1.0, -1.0):
        offsets: list[float] = []
        off = a.step_rad
        while off <= a.max_cmd_rad + 1e-9:
            if not await one(sign * off, sign):
                return points, stats
            offsets.append(off)
            if abs(points[-1].tau_nm) >= a.stop_torque:
                break
            off += a.step_rad
        for back in list(reversed(offsets[:-1])) + [0.0]:
            if not await one(sign * back, -sign):
                return points, stats
    stats["temp_end"] = rig.settled(f"s{servo}_h{n - 1}").get("temp", 0.0)
    return points, stats


async def main_async(args: argparse.Namespace) -> int:
    if args.backend == "robot" and not args.confirm_motion:
        print("The joints will move and hold against gravity. Re-run with --confirm-motion once the robot is hanging "
              f"from your hand by module {args.held_module}, straight, and you are ready.", file=sys.stderr)
        return 2
    inner: Any = RobotBackend(host=args.host, port=args.port, auto_reconnect=False) if args.backend == "robot" else MockBackend()
    backend = SafetyBackend(inner)
    out = Path(args.out or f"data/hang/hang_{args.backend}_{dt.datetime.now():%Y%m%d_%H%M%S}.jsonl")
    out.parent.mkdir(parents=True, exist_ok=True)
    if not await backend.connect():
        print("Connection failed.", file=sys.stderr)
        return 1
    rig = Rig(args, backend, out)
    rc = 0
    all_points: list[HoldPoint] = []
    try:
        rig.state = await backend.get_state()
        rig.ids = sorted(rig.state.active_servo_ids)
        rig.p0 = dict(rig.state.servo_positions)
        missing = [s for s in args.servo if s not in rig.ids]
        if missing:
            print(f"servo(s) {missing} not active (active: {rig.ids})", file=sys.stderr)
            return 1
        # Warm up: let the power manager stage every motor in, then check the robot is really hanging straight.
        deadline = time.monotonic() + 15
        calm: Optional[float] = None
        while time.monotonic() < deadline:
            await rig.tick(None, 0.0, "warmup", check=False)
            pt = rig.state.power_telemetry
            if pt is None or not pt.pending_motor_ids:
                calm = calm or time.monotonic()
                if time.monotonic() - calm > 1.0:
                    break
            else:
                calm = None
        await rig.hold(None, 0.0, 2.0, "static")
        static_rows = [r for r in rig.rows if r["tag"] == "static"]
        static = {i: float(np.mean([r["tau_all"].get(str(i), 0.0) for r in static_rows] or [0.0])) for i in rig.ids}
        temps = rig.state.motor_temperatures
        print(f"Connected. Servos {rig.ids}. Held by module {args.held_module}. {rig.state.battery_voltage_v:.2f} V, "
              f"bus {rig.state.battery_current_amps:.2f} A, temps {temps}")
        print("Static torque at the hanging pose (should be ~0): " + ", ".join(f"{i}:{static[i]:+.3f}" for i in rig.ids))
        if rig.ids and max(abs(v) for v in static.values()) > args.start_torque:
            print(f"REFUSING TO START: a static torque exceeds {args.start_torque} N·m - is the robot hanging straight from "
                  f"module {args.held_module}?", file=sys.stderr)
            return 1
        if temps and max(temps.values()) > args.start_temp_c:
            print(f"REFUSING TO START: motors are at {max(temps.values())} C (limit {args.start_temp_c}); let them cool.", file=sys.stderr)
            return 1
        print(f"Limits: bus {args.max_bus_a} A, temp {args.max_temp_c} C, torque stop {args.stop_torque} N·m. Writing {out}\n")

        rows_out = []
        for servo in args.servo:
            print(f"=== servo {servo} ===", flush=True)
            pts, stats = await sweep_joint(rig, servo)
            all_points.extend(pts)
            n_below = modules_below(servo, args.held_module)
            masses = args.module_masses or [args.mass_kg / 8.0] * 8
            if args.com_model == "motor-axis":
                k_pred = predicted_stiffness_point_masses(servo, args.held_module, masses, args.pitch_cm / 100.0,
                                                          tilt_deg=args.tilt_deg)
            else:
                k_pred = predicted_stiffness(n_below, sum(masses), args.pitch_cm / 100.0, tilt_deg=args.tilt_deg)
            fit = fit_gravity_friction(pts)
            if fit is None:
                print(f"  not enough data ({len(pts)} holds)" + (f"; stopped: {rig.abort}" if rig.abort else ""))
            else:
                ratio_txt = f"{fit.stiffness_nm_per_rad / k_pred:.2f}" if k_pred > 1e-6 else "n/a (no gravity lever predicted)"
                print(f"  {len(pts)} holds, {n_below} modules below: K = {fit.stiffness_nm_per_rad:.3f} ± {fit.stiffness_se:.3f} N·m/rad "
                      f"(predicted {k_pred:.3f}, measured/predicted {ratio_txt}); Coulomb friction c = {fit.coulomb_nm:.4f} ± {fit.coulomb_se:.4f} N·m; "
                      f"offset {fit.offset_nm:+.4f}; residual {fit.resid_rms_nm:.4f}; max |tau| {stats['max_tau']:.3f}; "
                      f"temp {stats.get('temp_start', 0):.0f}->{stats.get('temp_end', 0):.0f} C")
                rows_out.append({"servo": servo, "n_below": n_below, "K": fit.stiffness_nm_per_rad, "K_se": fit.stiffness_se,
                                 "K_pred": k_pred, "c": fit.coulomb_nm, "c_se": fit.coulomb_se, "offset": fit.offset_nm,
                                 "resid_rms": fit.resid_rms_nm, "n": fit.n_points, "max_tau": stats["max_tau"]})
            if rig.abort:
                print(f"  STOPPED: {rig.abort}")
                rc = 1
                break
            await rig.hold(None, 0.0, args.rest_s, f"s{servo}_rest", check=False)

        cur = fit_current_coefficient(all_points)
        if cur is not None:
            V = float(np.mean([r["bus_v"] for r in rig.rows])) or 12.0
            model = POWER_BUDGET.per_motor_torque_coeff * POWER_BUDGET.bus_voltage_nominal_v / max(V, 8.0)
            print(f"\nStatic bus-current fit: delta_bus = {cur[0]:.1f} A/(N·m)² · tau²  (±{100 * cur[1]:.0f}%); "
                  f"the model uses {model:.1f} at {V:.1f} V -> measured/model = {cur[0] / model:.2f}")
        else:
            print("\nNo static holds with |tau| ≥ 0.02 N·m: current model not checked.")
        print(f"Max bus current {rig.max_bus:.2f} A (limit {args.max_bus_a}); max motor temperature {rig.max_temp:.0f} C (limit {args.max_temp_c}).")
        out.with_suffix(".summary.json").write_text(json.dumps({"joints": rows_out, "args": vars(args)}, indent=2))
        await rig.hold(None, 0.0, 1.0, "end", check=False)
    finally:
        rig.fh.close()
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
