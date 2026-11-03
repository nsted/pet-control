#!/usr/bin/env python3
"""One automated run that characterises the whole hanging robot and logs everything raw for offline analysis.

Robot hung from the head (module 0), straight. Phases, in order, each with the other joints holding at their start pose:

  A  baseline        4 s still, all joints (jitter, idle bus current)
  B  sweeps          every joint in succession (7 -> 1): small angle holds approached from both directions,
                     stopping at --stop-torque (hysteresis = friction, slope = coupled gravity stiffness,
                     torque^2 vs bus current = the current model)
  C  excitation      every joint in succession: smooth rate-limited steps and a gentle sweep
                     (closed-loop fit: effective kp/kd, delay, friction)
  D  groups          adjacent pairs moving together; all seven as a travelling wave; static whole-body poses
                     (several motors loaded at once: current superposition, coupled gravity model, power-manager caps)

Logged at every control tick (100 Hz): all seven joints' angle, torque, velocity, the commands, bus current and voltage,
temperatures and the power manager's per-motor caps; plus every motor feedback frame at its true arrival time. Saved as
``<out>.npz`` with ``<out>.json`` describing the phases. Nothing is fitted here.

Stops everything on: bus current > --max-bus-a, any motor > --max-temp-c, any joint > --window rad from its start,
a power-manager intervention, a disconnect. Waits for the motors to cool (holding still) before a phase if they are above
--cool-c. Torques are disabled on exit (the chain just hangs from your hand).

    python scripts/capture_all.py --quick                                   # mock dry run
    python scripts/capture_all.py --backend robot --host 192.168.18.33 --confirm-motion
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
from petctl.config import MOTOR_LIMITS
from petctl.types import RobotState, ServoCommand

TICK_S = 0.01
SLEW_RAD_S = 3.0                       # command slew limit, well under the backend's 6 rad/s ramp
N = 7


def _parse() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--backend", choices=["mock", "robot"], default="mock")
    p.add_argument("--host", default=ROBOT_DEFAULT_HOST)
    p.add_argument("--port", type=int, default=ROBOT_DEFAULT_PORT)
    p.add_argument("--quick", action="store_true", help="Shorten every phase (dry runs)")
    p.add_argument("--skip", nargs="*", default=[], choices=["B", "C", "D"], help="Skip phases")
    p.add_argument("--stop-torque", type=float, default=0.10, help="Stop a sweep branch at this |torque| (N·m)")
    p.add_argument("--sweep-step", type=float, default=0.06)
    p.add_argument("--sweep-max", type=float, default=0.30)
    p.add_argument("--hold-s", type=float, default=1.2)
    p.add_argument("--amp", type=float, default=0.12, help="Amplitude of group motions (rad)")
    p.add_argument("--window", type=float, default=0.6)
    p.add_argument("--max-bus-a", type=float, default=4.5)
    p.add_argument("--max-temp-c", type=float, default=52.0)
    p.add_argument("--cool-c", type=float, default=49.0, help="Wait for motors to cool below this between phases")
    p.add_argument("--out", default=None)
    p.add_argument("--confirm-motion", action="store_true")
    return p.parse_args()


class Capture:
    def __init__(self, args: argparse.Namespace, backend: SafetyBackend, inner: Any) -> None:
        self.a, self.backend = args, backend
        self.scale = 0.2 if args.quick else 1.0
        self.ids: list[int] = []
        self.p0 = np.zeros(N)
        self.state: RobotState = RobotState.empty()
        self.cmd_now = np.zeros(N)
        self.abort = ""
        self.max_bus = 0.0
        self.max_temp = 0.0
        self._over_since: Optional[float] = None
        # raw logs
        self.t: list[float] = []
        self.rows: list[list[float]] = []         # per tick: bus_a, bus_v, temp_max, then pos[7], tau[7], vel[7], cmd[7], cap[7]
        self.frames: list[tuple[float, int, float, float, float]] = []   # t, motor, pos, vel, tau
        self.marks: list[dict[str, Any]] = []
        self.polling = not hasattr(inner, "set_feedback_listener")
        if not self.polling:
            inner.set_feedback_listener(lambda t, fb: self.frames.append((t, fb.motor_id, fb.pos, fb.vel, fb.torque)))

    # ------------------------------------------------------------------ control tick
    def _check(self) -> str:
        s, a = self.state, self.a
        if not s.connected:
            return "disconnected"
        cur = s.battery_current_amps
        self.max_bus = max(self.max_bus, cur)
        if cur > a.max_bus_a:
            self._over_since = self._over_since or time.monotonic()
            if time.monotonic() - self._over_since > 0.1:
                return f"bus current {cur:.2f} A"
        else:
            self._over_since = None
        temps = list(s.motor_temperatures.values())
        tmax = max(temps) if temps else 0.0
        self.max_temp = max(self.max_temp, tmax)
        if tmax > a.max_temp_c:
            return f"temperature {tmax} C"
        pt = s.power_telemetry
        if pt is not None:
            if pt.system_state != "RUNNING" or pt.voltage_cutoff_active:
                return f"power manager state {pt.system_state}"
            bad = [k for k, v in pt.motor_states.items() if v != "NORMAL"]
            if bad:
                return f"power manager touched motors {bad}"
        for j, sid in enumerate(self.ids):
            if abs(s.servo_positions.get(sid, self.p0[j]) - self.p0[j]) > a.window:
                return f"joint {sid} left its window"
        return ""

    async def tick(self, target: np.ndarray) -> None:
        t0 = time.monotonic()
        self.state = await self.backend.get_state()
        s = self.state
        if self.polling:
            for j, sid in enumerate(self.ids):
                self.frames.append((t0, sid, s.servo_positions.get(sid, 0.0), s.motor_velocities.get(sid, 0.0), s.motor_torques.get(sid, 0.0)))
        reason = self._check()
        if reason and not self.abort:
            self.abort = reason
        if self.abort:
            target = np.zeros(N)
        step = SLEW_RAD_S * TICK_S
        self.cmd_now += np.clip(target - self.cmd_now, -step, step)
        cmds = [ServoCommand(servo_id=sid, position=float(self.p0[j] + self.cmd_now[j])) for j, sid in enumerate(self.ids)]
        await self.backend.send_commands(cmds)
        pt = s.power_telemetry
        temps = list(s.motor_temperatures.values())
        self.t.append(t0)
        self.rows.append([s.battery_current_amps, s.battery_voltage_v, max(temps) if temps else 0.0]
                         + [s.servo_positions.get(sid, 0.0) - self.p0[j] for j, sid in enumerate(self.ids)]
                         + [s.motor_torques.get(sid, 0.0) for sid in self.ids]
                         + [s.motor_velocities.get(sid, 0.0) for sid in self.ids]
                         + [float(self.cmd_now[j]) for j in range(N)]
                         + [(pt.per_motor_torque_cap_nm.get(sid, float("nan")) if pt else float("nan")) for sid in self.ids])
        await asyncio.sleep(max(0.0, TICK_S - (time.monotonic() - t0)))

    async def run(self, name: str, seconds: float, fn: Callable[[float], np.ndarray]) -> None:
        start = time.monotonic()
        self.marks.append({"name": name, "t0": start})
        while time.monotonic() - start < seconds * self.scale and not self.abort:
            await self.tick(fn((time.monotonic() - start) / self.scale))
        self.marks[-1]["t1"] = time.monotonic()

    async def settle(self, seconds: float = 1.0, name: str = "settle") -> None:
        await self.run(name, seconds, lambda t: np.zeros(N))

    async def cool(self, phase: str) -> None:
        """Hold still until the hottest motor is below --cool-c (or give up after 150 s)."""
        deadline = time.monotonic() + 150
        self.marks.append({"name": f"cool_before_{phase}", "t0": time.monotonic()})
        while not self.abort and time.monotonic() < deadline:
            temps = list(self.state.motor_temperatures.values())
            if temps and max(temps) >= self.a.cool_c:
                await self.tick(np.zeros(N))
            else:
                break
        self.marks[-1]["t1"] = time.monotonic()

    # ------------------------------------------------------------------ phases
    def _vec(self, joint: int, value: float) -> np.ndarray:
        v = np.zeros(N)
        v[joint - 1] = value
        return v

    async def sweep(self, joint: int) -> None:
        a = self.a
        settled_tau: list[float] = []

        async def hold(offset: float, tag: str) -> float:
            await self.run(f"sweep{joint}_{tag}", abs(offset - self.cmd_now[joint - 1]) / SLEW_RAD_S / self.scale + a.hold_s,
                           lambda t, o=offset: self._vec(joint, o))
            tail = self.rows[-max(5, int(0.4 * a.hold_s * self.scale / TICK_S)):]
            return float(np.mean([r[3 + N + joint - 1] for r in tail]))

        n = 0
        for sign in (1.0, -1.0):
            offs, off = [], a.sweep_step
            while off <= a.sweep_max + 1e-9 and not self.abort:
                tau = await hold(sign * off, f"h{n}")
                n += 1
                offs.append(off)
                if abs(tau) >= a.stop_torque:
                    break
                off += a.sweep_step
            for back in list(reversed(offs[:-1])) + [0.0]:
                if self.abort:
                    return
                await hold(sign * back, f"h{n}")
                n += 1

    async def excite(self, joint: int) -> None:
        for amp in (0.3, -0.3):
            ramp = abs(amp) / SLEW_RAD_S
            await self.run(f"step{joint}_{amp:+.1f}", 0.3 + ramp + 2.0 + ramp + 1.5,
                           lambda t, amp=amp, ramp=ramp: self._vec(joint, amp * float(np.clip((t - 0.3) / ramp, 0, 1)) * (1.0 if t < 0.3 + ramp + 2.0 else 0.0)
                                                                    + (amp * float(np.clip(1 - (t - 0.3 - ramp - 2.0) / ramp, 0, 1)) if t >= 0.3 + ramp + 2.0 else 0.0)))
        T, f0, f1, A = 5.0, 0.2, 1.2, 0.12
        await self.run(f"chirp{joint}", T + 1.0, lambda t: self._vec(joint, A * math.sin(2 * math.pi * (f0 * t + (f1 - f0) * t * t / (2 * T)))
                                                                     * min(1.0, t / 0.5) * min(1.0, max(0.0, (T - t) / 0.5)) if t < T else 0.0))

    def _wave(self, f: float, lag: float, amp: float) -> Callable[[float], np.ndarray]:
        return lambda t: np.array([amp * math.sin(2 * math.pi * f * t - j * lag) * min(1.0, t / 1.0) for j in range(N)])

    async def groups(self) -> None:
        A = self.a.amp
        for pair in ((7, 6), (5, 4), (3, 2)):
            def fn(t: float, pair: tuple[int, int] = pair) -> np.ndarray:
                v = np.zeros(N)
                s = A * math.sin(2 * math.pi * 0.4 * t) * min(1.0, t / 1.0)
                v[pair[0] - 1], v[pair[1] - 1] = s, -s
                return v
            await self.run(f"pair{pair[0]}{pair[1]}", 8.0, fn)
            await self.settle(1.0)
        await self.run("wave_0.3Hz", 12.0, self._wave(0.3, 2 * math.pi / N, A))
        await self.settle(1.5)
        await self.run("wave_0.6Hz", 10.0, self._wave(0.6, 2 * math.pi / N, A * 0.7))
        await self.settle(1.5)
        poses = {
            "pose_zigzag": np.array([A * 1.2 * (1 if j % 2 == 0 else -1) for j in range(N)]),
            "pose_zagzig": np.array([A * 1.2 * (-1 if j % 2 == 0 else 1) for j in range(N)]),
            "pose_all_pos": np.full(N, A),
            "pose_all_neg": np.full(N, -A),
            "pose_ramp_up": np.array([A * (j + 1) / N * 1.5 for j in range(N)]),
            "pose_ramp_down": np.array([-A * (j + 1) / N * 1.5 for j in range(N)]),
        }
        for name, pose in poses.items():
            await self.run(name, 4.0, lambda t, pose=pose: pose)
            await self.settle(1.5)

    # ------------------------------------------------------------------ driver
    async def execute(self) -> None:
        a = self.a
        await self.run("A_baseline", 4.0, lambda t: np.zeros(N))
        if "B" not in a.skip:
            await self.cool("B")
            for j in range(N, 0, -1):
                if self.abort:
                    break
                await self.sweep(j)
                await self.settle(1.0)
        if "C" not in a.skip:
            await self.cool("C")
            for j in range(N, 0, -1):
                if self.abort:
                    break
                await self.excite(j)
                await self.settle(1.0)
        if "D" not in a.skip:
            await self.cool("D")
            await self.groups()
        await self.settle(1.5, "end")


async def main_async(args: argparse.Namespace) -> int:
    if args.backend == "robot" and not args.confirm_motion:
        print("This moves every joint (sweeps, steps, group motions) while holding against gravity. Re-run with "
              "--confirm-motion, robot hanging straight from the head, a hand ready.", file=sys.stderr)
        return 2
    inner: Any = RobotBackend(host=args.host, port=args.port, auto_reconnect=False) if args.backend == "robot" else MockBackend()
    backend = SafetyBackend(inner)
    out = Path(args.out or f"data/capture/capture_{args.backend}_{dt.datetime.now():%Y%m%d_%H%M%S}")
    out.parent.mkdir(parents=True, exist_ok=True)
    if not await backend.connect():
        print("Connection failed.", file=sys.stderr)
        return 1
    cap = Capture(args, backend, inner)
    rc = 0
    try:
        cap.state = await backend.get_state()
        cap.ids = sorted(cap.state.active_servo_ids)
        if cap.ids != list(range(1, N + 1)):
            print(f"expected servos 1-7, found {cap.ids}", file=sys.stderr)
            return 1
        cap.p0 = np.array([cap.state.servo_positions[i] for i in cap.ids])
        deadline, calm = time.monotonic() + 15, None
        while time.monotonic() < deadline:                                    # let the power manager stage every motor in
            await cap.tick(np.zeros(N))
            pt = cap.state.power_telemetry
            if pt is None or not pt.pending_motor_ids:
                calm = calm or time.monotonic()
                if time.monotonic() - calm > 1.0:
                    break
            else:
                calm = None
        cap.t.clear(); cap.rows.clear(); cap.frames.clear(); cap.marks.clear()
        s = cap.state
        print(f"Connected. {s.battery_voltage_v:.2f} V, idle bus {s.battery_current_amps:.2f} A, temps {s.motor_temperatures}. "
              f"Static torques: {[round(s.motor_torques.get(i, 0.0), 3) for i in cap.ids]}. Writing {out}.npz\n", flush=True)
        t_run = time.monotonic()
        await cap.execute()
        rc = 1 if cap.abort else 0
        print(f"\n{'STOPPED: ' + cap.abort if cap.abort else 'Completed all phases'} in {time.monotonic() - t_run:.0f} s. "
              f"Max bus {cap.max_bus:.2f} A (limit {args.max_bus_a}); max motor temperature {cap.max_temp:.0f} C (limit {args.max_temp_c}).")
    finally:
        try:
            await backend.disable_torques()
        finally:
            await backend.disconnect()
        if cap.t:
            arr = np.array(cap.rows)
            fr = np.array(cap.frames) if cap.frames else np.zeros((0, 5))
            np.savez_compressed(f"{out}.npz", t=np.array(cap.t), bus_a=arr[:, 0], bus_v=arr[:, 1], temp_max=arr[:, 2],
                                theta=arr[:, 3:3 + N], tau=arr[:, 3 + N:3 + 2 * N], vel=arr[:, 3 + 2 * N:3 + 3 * N],
                                cmd=arr[:, 3 + 3 * N:3 + 4 * N], cap=arr[:, 3 + 4 * N:3 + 5 * N],
                                fr_t=fr[:, 0], fr_motor=fr[:, 1], fr_pos=fr[:, 2], fr_vel=fr[:, 3], fr_tau=fr[:, 4], p0=cap.p0)
            Path(f"{out}.json").write_text(json.dumps({"args": vars(args), "marks": cap.marks, "abort": cap.abort,
                                                       "max_bus": cap.max_bus, "max_temp": cap.max_temp}, indent=1))
            print(f"Data: {out}.npz ({len(cap.t)} ticks, {len(cap.frames)} feedback frames)")
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
