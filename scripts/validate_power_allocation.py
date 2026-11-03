#!/usr/bin/env python3
"""Validate the power manager's per-motor torque allocation against measured bus current.

Steps open-loop torque (``kp = kd = 0``, ``torque_ff``) on ONE servo while the others hold their
start pose, and for each level compares

  * the torque requested vs the torque the power manager actually forwarded (the cap),
  * the measured torque, and
  * the measured rise in total bus current vs the model's prediction
    (``coeff * tau^2 * 12/V  +  mech * |tau * omega| / V``, the same model the cap is built on).

Optionally (``--sustained``) holds one high request for several seconds to watch the sustained
(< 1 A running average) limit ease the allowance down.

Pick a servo that is clamped by friction or the floor so the torque is held and the joint barely moves
(on the bench servo 4 did not move up to 0.1 N·m). Stops immediately, and does not go to higher
levels, if: total bus current exceeds --max-bus-a, any motor passes --max-temp-c, the joint leaves
--window rad of its start pose, the power manager touches the servo, or the robot disconnects.
Torques are disabled on exit.

    python scripts/validate_power_allocation.py --servo 4                       # mock dry run
    python scripts/validate_power_allocation.py --backend robot --servo 4 --confirm-motion
    python scripts/validate_power_allocation.py --backend robot --servo 4 --confirm-motion --sustained
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
from petctl.config import POWER_BUDGET, MOTOR_LIMITS
from petctl.types import RobotState, ServoCommand

DEFAULT_LEVELS = (0.04, 0.08, 0.10, 0.12, 0.14, 0.164, 0.20)
TICK_S = 0.01


def _parse() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--backend", choices=["mock", "robot"], default="mock")
    p.add_argument("--host", default=ROBOT_DEFAULT_HOST)
    p.add_argument("--port", type=int, default=ROBOT_DEFAULT_PORT)
    p.add_argument("--servo", type=int, default=4)
    p.add_argument("--levels", type=float, nargs="+", default=list(DEFAULT_LEVELS),
                   help="|torque| steps in N·m, ascending; the sign alternates")
    p.add_argument("--hold-s", type=float, default=1.5)
    p.add_argument("--rest-s", type=float, default=3.0)
    p.add_argument("--window", type=float, default=0.4, help="Abort if the joint leaves its start pose by this many rad")
    p.add_argument("--max-bus-a", type=float, default=4.5, help="Abort above this total bus current (A)")
    p.add_argument("--max-temp-c", type=float, default=52.0, help="Abort if any motor temperature exceeds this")
    p.add_argument("--sustained", action="store_true", help="After the steps, hold one request for --sustained-s")
    p.add_argument("--sustained-torque", type=float, default=0.164)
    p.add_argument("--sustained-s", type=float, default=8.0)
    p.add_argument("--out", default=None)
    p.add_argument("--confirm-motion", action="store_true",
                   help="Required with --backend robot: acknowledges that the motor WILL be loaded")
    return p.parse_args()


class Validator:
    def __init__(self, args: argparse.Namespace, backend: SafetyBackend, inner: Any, out: Path) -> None:
        self.args, self.backend, self.out = args, backend, out
        self.forwarded: dict[int, ServoCommand] = {}
        orig = inner.send_commands

        async def spy(cmds: list[ServoCommand]) -> None:
            for c in cmds:
                self.forwarded[c.servo_id] = c
            await orig(cmds)

        inner.send_commands = spy
        self.state: RobotState = RobotState.empty()
        self.ids: list[int] = []
        self.p0: dict[int, float] = {}
        self.rows: list[dict[str, Any]] = []
        self.fh = out.open("a")
        self._over_current_since: Optional[float] = None
        self.abort: str = ""
        self.max_bus_seen = 0.0
        self.max_temp_seen = 0.0

    # ------------------------------------------------------------------
    def _check(self, state: RobotState) -> str:
        a = self.args
        if not state.connected:
            return "disconnected"
        cur = state.battery_current_amps
        self.max_bus_seen = max(self.max_bus_seen, cur)
        if cur > a.max_bus_a:
            self._over_current_since = self._over_current_since or time.monotonic()
            if time.monotonic() - self._over_current_since > 0.1:
                return f"bus current {cur:.2f} A > {a.max_bus_a} A"
        else:
            self._over_current_since = None
        temps = list(state.motor_temperatures.values())
        tmax = max(temps) if temps else 0.0
        self.max_temp_seen = max(self.max_temp_seen, tmax)
        if tmax > a.max_temp_c:
            return f"motor temperature {tmax:.0f} C > {a.max_temp_c} C"
        pos = state.servo_positions.get(a.servo, self.p0.get(a.servo, 0.0))
        if abs(pos - self.p0[a.servo]) > a.window:
            return f"joint left its window ({pos - self.p0[a.servo]:+.2f} rad)"
        pt = state.power_telemetry
        if pt is not None:
            if pt.system_state != "RUNNING" or pt.voltage_cutoff_active:
                return f"power manager state {pt.system_state}"
            if pt.motor_states.get(a.servo, "NORMAL") != "NORMAL" or pt.motor_compliance_scales.get(a.servo, 1.0) < 1.0:
                return "power manager touched the servo (thermal state/compliance)"
        return ""

    async def tick(self, request: Optional[float], tag: str, check: bool = True) -> str:
        """One control tick. request=None holds the servo at its start pose; else open-loop torque."""
        a = self.args
        t0 = time.monotonic()
        state = self.state = await self.backend.get_state()
        reason = self._check(state) if check else ""
        if reason and not self.abort:
            self.abort = reason
        if self.abort:
            request = None
        cmds = [ServoCommand(servo_id=i, position=self.p0[i]) for i in self.ids if i != a.servo]
        if request is None:
            cmds.append(ServoCommand(servo_id=a.servo, position=self.p0[a.servo]))
        else:
            cmds.append(ServoCommand(servo_id=a.servo, position=self.p0[a.servo], kp=0.0, kd=0.0, torque_ff=request))
        await self.backend.send_commands(cmds)
        pt = state.power_telemetry
        fwd = self.forwarded.get(a.servo)
        temps = list(state.motor_temperatures.values())
        row = {
            "t": t0, "tag": tag, "req": request if request is not None else 0.0,
            "fwd": fwd.torque_ff if fwd is not None else 0.0,
            "tau": state.motor_torques.get(a.servo, 0.0), "vel": state.motor_velocities.get(a.servo, 0.0),
            "pos": state.servo_positions.get(a.servo, 0.0) - self.p0[a.servo],
            "bus_a": state.battery_current_amps, "bus_v": state.battery_voltage_v,
            "cap": pt.per_motor_torque_cap_nm.get(a.servo) if pt else None,
            "ema_a": pt.per_motor_sustained_current_a.get(a.servo) if pt else None,
            "temp": state.motor_temperatures.get(a.servo, 0), "temp_max": max(temps) if temps else 0,
        }
        self.rows.append(row)
        self.fh.write(json.dumps(row) + "\n")
        await asyncio.sleep(max(0.0, TICK_S - (time.monotonic() - t0)))
        return self.abort

    async def phase(self, seconds: float, request: Optional[float], tag: str, check: bool = True) -> str:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if await self.tick(request, tag, check):
                return self.abort
        return ""

    # ------------------------------------------------------------------
    async def warm_up(self) -> None:
        """Hold until the power manager has staged every motor in (cold start), then log a baseline."""
        deadline = time.monotonic() + 15
        calm = None
        while time.monotonic() < deadline:
            await self.tick(None, "warmup", check=False)
            pt = self.state.power_telemetry
            if pt is not None and not pt.pending_motor_ids:
                calm = calm or time.monotonic()
                if time.monotonic() - calm > 1.0:
                    return
            else:
                calm = None
        raise RuntimeError("power manager did not finish staging motors in")

    def _stats(self, tag: str, frac_from: float = 0.4) -> dict[str, float]:
        rows = [r for r in self.rows if r["tag"] == tag]
        if not rows:
            return {}
        tail = rows[int(len(rows) * frac_from):]
        g = lambda k: float(np.mean([r[k] for r in tail if r[k] is not None]))  # noqa: E731
        return {"n": len(tail), "req": g("req"), "fwd": g("fwd"), "tau": g("tau"), "absvel": float(np.mean([abs(r["vel"]) for r in tail])),
                "bus_a": g("bus_a"), "bus_v": g("bus_v"), "cap": g("cap") if tail[0]["cap"] is not None else float("nan"),
                "travel": max(abs(r["pos"]) for r in rows)}

    async def step_test(self) -> list[dict[str, float]]:
        a = self.args
        b = POWER_BUDGET
        results: list[dict[str, float]] = []
        sign = 1.0
        for k, level in enumerate(a.levels):
            req = sign * level
            sign = -sign
            base_tag, hold_tag = f"rest{k}", f"hold{k}"
            if await self.phase(a.rest_s, None, base_tag):
                break
            if await self.phase(a.hold_s, req, hold_tag):
                results.append(self._summarize(k, base_tag, hold_tag, aborted=True))
                break
            results.append(self._summarize(k, base_tag, hold_tag, aborted=False))
        return results

    def _summarize(self, k: int, base_tag: str, hold_tag: str, aborted: bool) -> dict[str, float]:
        b = POWER_BUDGET
        base = self._stats(base_tag, 0.5)
        hold = self._stats(hold_tag, 0.4)
        out = dict(hold)
        out["aborted"] = float(aborted)
        out["level"] = abs(hold.get("req", 0.0))
        if hold and base:
            V = max(hold["bus_v"], 8.0)
            out["d_bus"] = hold["bus_a"] - base["bus_a"]
            out["model_d"] = (b.per_motor_torque_coeff * hold["tau"] ** 2 * (b.bus_voltage_nominal_v / V)
                              + b.per_motor_mech_coeff * abs(hold["tau"]) * hold["absvel"] / V)
            out["model_from_fwd"] = b.per_motor_torque_coeff * hold["fwd"] ** 2 * (b.bus_voltage_nominal_v / V)
        return out

    async def sustained_test(self) -> dict[str, Any]:
        a = self.args
        await self.phase(a.rest_s, None, "srest")
        reason = await self.phase(a.sustained_s, a.sustained_torque, "sustain")
        rows = [r for r in self.rows if r["tag"] == "sustain"]
        marks = {}
        if rows:
            t0 = rows[0]["t"]
            for sec in (0.3, 1.0, 2.0, 3.0, 4.0, 6.0, 8.0):
                near = [r for r in rows if r["t"] - t0 <= sec]
                if near and (near[-1]["t"] - t0) >= sec - 0.05:
                    r = near[-1]
                    marks[sec] = (r["fwd"], r["tau"], r["ema_a"], r["bus_a"], r["temp"])
        return {"aborted": reason, "marks": marks}


def _print_steps(results: list[dict[str, float]], baseline: float) -> None:
    print("\nlevel  | forwarded | cap    | measured tau | |vel|  | travel | bus A  | d bus  | model d | meas/model")
    print("-------+-----------+--------+--------------+--------+--------+--------+--------+---------+-----------")
    for r in results:
        if "d_bus" not in r:
            continue
        ratio = r["d_bus"] / r["model_d"] if r["model_d"] > 1e-3 else float("nan")
        flag = "  ABORTED" if r["aborted"] else ""
        print(f"{r['level']:.3f}  | {abs(r['fwd']):.4f}    | {r['cap']:.4f} | {abs(r['tau']):.4f}       | {r['absvel']:.3f}  | {r['travel']:.3f}  | "
              f"{r['bus_a']:.2f}   | {r['d_bus']:+.3f} | {r['model_d']:.3f}   | {ratio:5.2f}{flag}")
    print(f"(idle bus baseline {baseline:.2f} A; d bus = rise over the preceding rest; model d = coeff·tau²·12/V + mech term)")


async def main_async(args: argparse.Namespace) -> int:
    if args.backend == "robot" and not args.confirm_motion:
        print("This loads a motor with open-loop torque. Re-run with --confirm-motion once the robot is positioned "
              "and a hand is near.", file=sys.stderr)
        return 2
    if args.backend == "robot":
        inner: Any = RobotBackend(host=args.host, port=args.port, auto_reconnect=False)
    else:
        inner = MockBackend()
    backend = SafetyBackend(inner)
    out = Path(args.out or f"data/power_validation/val_{args.backend}_{dt.datetime.now():%Y%m%d_%H%M%S}.jsonl")
    out.parent.mkdir(parents=True, exist_ok=True)
    if not await backend.connect():
        print("Connection failed.", file=sys.stderr)
        return 1
    v = Validator(args, backend, inner, out)
    rc = 0
    try:
        v.state = await backend.get_state()
        v.ids = sorted(v.state.active_servo_ids)
        v.p0 = dict(v.state.servo_positions)
        if args.servo not in v.ids:
            print(f"servo {args.servo} not active (active: {v.ids})", file=sys.stderr)
            return 1
        pt = v.state.power_telemetry
        print(f"Connected. Servos {v.ids}. Mode: {POWER_BUDGET.per_motor_cap_mode}; peak {POWER_BUDGET.per_motor_peak_current_a} A, "
              f"sustained {POWER_BUDGET.per_motor_sustained_current_a} A, budget (this tick) {pt.power_source if pt else '?'}.")
        await v.warm_up()
        s = v.state
        temps = s.motor_temperatures
        print(f"Start: {s.battery_voltage_v:.2f} V, bus {s.battery_current_amps:.2f} A, temps {temps}. "
              f"Limits: bus {args.max_bus_a} A, temp {args.max_temp_c} C, window {args.window} rad. Writing {out}")
        await v.phase(3.0, None, "baseline")
        baseline = v._stats("baseline", 0.3).get("bus_a", 0.0)
        results = await v.step_test()
        if not v.abort:
            if args.sustained:
                sus = await v.sustained_test()
            else:
                sus = None
        else:
            sus = None
        _print_steps(results, baseline)
        if sus is not None:
            print(f"\nSustained hold of {args.sustained_torque:.3f} N·m requested for {args.sustained_s:.0f} s"
                  + (f" (stopped: {sus['aborted']})" if sus["aborted"] else ""))
            print("  t [s] | forwarded | measured tau | running avg I [A] | bus A | temp C")
            for sec, (fwd, tau, ema, bus, tmp) in sus["marks"].items():
                print(f"  {sec:5.1f} | {abs(fwd):.4f}    | {abs(tau):.4f}       | {ema if ema is not None else float('nan'):.3f}             | {bus:.2f}  | {tmp}")
        print(f"\nMax bus current seen {v.max_bus_seen:.2f} A (limit {args.max_bus_a}); max motor temp {v.max_temp_seen:.0f} C (limit {args.max_temp_c}).")
        if v.abort:
            print(f"STOPPED EARLY: {v.abort}")
            rc = 1
        await v.phase(1.0, None, "end", check=False)
    finally:
        v.fh.close()
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
