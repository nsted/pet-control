#!/usr/bin/env python3
"""
Guided sensor survey for PET robot capacitive pads and FSRs.

Walks through each module × face interactively, records raw samples to a
timestamped JSONL file, and prints a summary table flagging anomalies.

Every sample captures all three faces so cross-talk is visible (touching
the LEFT face but seeing RIGHT light up reveals a wiring/assignment error).

Usage:
    python scripts/sensor_survey.py                         # real robot, all modules
    python scripts/sensor_survey.py --modules 0,1,2         # partial hardware
    python scripts/sensor_survey.py --dry-run               # MockBackend (noise)
    python scripts/sensor_survey.py --dry-run --dry-run-mode mock-sensor-sine
"""

from __future__ import annotations

import argparse
import asyncio
import dataclasses
import datetime
import json
import sys
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from petctl.config import LOOP_LIMITS

if TYPE_CHECKING:
    from petctl.protocols import RobotBackend as RobotBackendProto
    from petctl.types import RobotState


# ── Constants ─────────────────────────────────────────────────────────────────

SURVEY_SAMPLE_HZ  = 20
BASELINE_SAMPLES  = 20
ACTIVATED_SAMPLES = 20
RELEASE_SAMPLES   = 10
FACES             = ("left", "right", "middle")
PAD_COUNTS        = {"left": 4, "right": 4, "middle": 6}
BAR_WIDTH         = 20

DEAD_THRESHOLD  = 0.10
STUCK_THRESHOLD = 0.30
LOW_SENSITIVITY = 0.30


# ── Data types ────────────────────────────────────────────────────────────────

@dataclass
class SurveyRecord:
    """One sensor snapshot. `face` is the target face being tested; all three
    faces are always captured so cross-talk is visible in the raw data."""
    ts:          float
    module_id:   int
    face:        str    # target face being tested: "left" | "right" | "middle"
    phase:       str    # "baseline" | "activated" | "release"
    step_index:  int
    retake:      int    # 0 for first attempt, increments on redo
    left_pads:   list[float]   # 4 values
    right_pads:  list[float]   # 4 values
    middle_pads: list[float]   # 6 values
    left_fsr:    float
    right_fsr:   float
    middle_fsr:  float


@dataclass
class FaceStats:
    module_id:          int
    face:               str
    retake:             int
    baseline_mean:      list[float]
    activated_mean:     list[float]
    delta:              list[float]
    fsr_baseline_mean:  float
    fsr_activated_mean: float
    fsr_delta:          float
    anomalies:          list[str] = field(default_factory=list)
    skipped:            bool = False


# ── File I/O ──────────────────────────────────────────────────────────────────

class SurveyRecorder:
    def __init__(self, path: str) -> None:
        self._path = path
        self._file = open(path, "w")

    def write(self, record: SurveyRecord) -> None:
        self._file.write(json.dumps(dataclasses.asdict(record)) + "\n")
        self._file.flush()

    def close(self) -> None:
        self._file.close()

    @property
    def path(self) -> str:
        return self._path


# ── Live display ──────────────────────────────────────────────────────────────

class LiveDisplay:
    def __init__(self, is_tty: bool) -> None:
        self._is_tty = is_tty
        self._lines_printed = 0

    def _bar(self, value: float) -> str:
        filled = round(max(0.0, min(1.0, value)) * BAR_WIDTH)
        return "[" + "#" * filled + "." * (BAR_WIDTH - filled) + f"] {value:.2f}"

    def render(
        self,
        module_id: int,
        target_face: str,
        phase: str,
        record: SurveyRecord,
        sample_idx: int,
        total: int,
    ) -> None:
        lines = [
            f"  Module {module_id} | target: {target_face.upper()} | {phase.upper()}"
            f"  ({sample_idx}/{total})",
        ]
        for face in FACES:
            marker = "▶" if face == target_face else " "
            pads = getattr(record, f"{face}_pads")
            fsr  = getattr(record, f"{face}_fsr")
            lines.append(f"  {marker} {face}")
            for i, v in enumerate(pads):
                lines.append(f"      pad {i}: {self._bar(v)}")
            lines.append(f"      FSR:   {self._bar(fsr)}")

        if self._is_tty and self._lines_printed:
            sys.stdout.write(f"\033[{self._lines_printed}A")
        for line in lines:
            sys.stdout.write(line + "\n")
        sys.stdout.flush()
        self._lines_printed = len(lines)

    def clear(self) -> None:
        if self._is_tty and self._lines_printed:
            sys.stdout.write(f"\033[{self._lines_printed}A")
            for _ in range(self._lines_printed):
                sys.stdout.write("\033[2K\n")
            sys.stdout.write(f"\033[{self._lines_printed}A")
            sys.stdout.flush()
        self._lines_printed = 0


# ── Stats and anomaly detection ───────────────────────────────────────────────

def _target_pads(record: SurveyRecord) -> list[float]:
    return getattr(record, f"{record.face}_pads")


def _target_fsr(record: SurveyRecord) -> float:
    return getattr(record, f"{record.face}_fsr")


def _mean_by_pad(records: list[SurveyRecord]) -> list[float]:
    if not records:
        return []
    pads_list = [_target_pads(r) for r in records]
    n = len(pads_list[0])
    return [sum(p[i] for p in pads_list) / len(pads_list) for i in range(n)]


def _mean_fsr(records: list[SurveyRecord]) -> float:
    if not records:
        return 0.0
    return sum(_target_fsr(r) for r in records) / len(records)


def compute_face_stats(
    module_id: int,
    face: str,
    retake: int,
    baseline: list[SurveyRecord],
    activated: list[SurveyRecord],
) -> FaceStats:
    base_mean = _mean_by_pad(baseline)
    act_mean  = _mean_by_pad(activated)
    delta     = [a - b for a, b in zip(act_mean, base_mean)]
    fsr_base  = _mean_fsr(baseline)
    fsr_act   = _mean_fsr(activated)
    stats = FaceStats(
        module_id=module_id,
        face=face,
        retake=retake,
        baseline_mean=base_mean,
        activated_mean=act_mean,
        delta=delta,
        fsr_baseline_mean=fsr_base,
        fsr_activated_mean=fsr_act,
        fsr_delta=fsr_act - fsr_base,
    )
    stats.anomalies = detect_anomalies(stats)
    return stats


def detect_anomalies(stats: FaceStats) -> list[str]:
    anomalies: list[str] = []
    for i, (base, act, delta) in enumerate(
        zip(stats.baseline_mean, stats.activated_mean, stats.delta)
    ):
        if act < DEAD_THRESHOLD:
            anomalies.append(f"pad{i}:DEAD(act={act:.2f})")
        elif base > STUCK_THRESHOLD:
            anomalies.append(f"pad{i}:STUCK(base={base:.2f})")
        elif delta < LOW_SENSITIVITY:
            anomalies.append(f"pad{i}:LOW_SENS(Δ={delta:.2f})")
    if stats.fsr_delta < LOW_SENSITIVITY:
        anomalies.append(f"FSR:LOW_SENS(Δ={stats.fsr_delta:.2f})")
    return anomalies


def _format_result(stats: FaceStats) -> str:
    if stats.skipped:
        return f"{stats.face.upper()} M{stats.module_id}: SKIPPED"
    base  = sum(stats.baseline_mean) / len(stats.baseline_mean) if stats.baseline_mean else 0.0
    act   = sum(stats.activated_mean) / len(stats.activated_mean) if stats.activated_mean else 0.0
    delta = act - base
    tag   = ("⚠  " + "  ".join(stats.anomalies)) if stats.anomalies else "OK"
    retake_str = f" [retake {stats.retake}]" if stats.retake else ""
    return (
        f"{stats.face.upper()} M{stats.module_id}{retake_str}: "
        f"base={base:.2f}  act={act:.2f}  Δ={delta:.2f}  FSR-Δ={stats.fsr_delta:.2f}  {tag}"
    )


# ── Summary output ────────────────────────────────────────────────────────────

def print_summary_table(all_stats: list[FaceStats]) -> None:
    def _scalar(vals: list[float]) -> float:
        return sum(vals) / len(vals) if vals else 0.0

    print()
    sep = "═" * 82
    print(sep)
    print("  SENSOR SURVEY SUMMARY")
    print(sep)
    print(f"  {'Mod':>3}  {'Face':<6}  {'#':>2}  {'Base':>6}  {'Act':>6}  {'Δ':>6}  {'FSR-Δ':>6}  {'Retake':>6}  Anomalies")
    print("  " + "─" * 78)

    total_anomalies = 0
    dead_count      = 0
    stuck_count     = 0

    for s in all_stats:
        if s.skipped:
            print(f"  {s.module_id:>3}  {s.face:<6}  {'—':>2}  {'SKIPPED':>6}")
            continue
        n_pads = PAD_COUNTS[s.face]
        base   = _scalar(s.baseline_mean)
        act    = _scalar(s.activated_mean)
        delta  = _scalar(s.delta)
        status = "  ".join(s.anomalies) if s.anomalies else "OK"
        print(
            f"  {s.module_id:>3}  {s.face:<6}  {n_pads:>2}  "
            f"{base:>6.2f}  {act:>6.2f}  {delta:>6.2f}  {s.fsr_delta:>6.2f}  {s.retake:>6}  {status}"
        )
        total_anomalies += len(s.anomalies)
        dead_count  += sum(1 for a in s.anomalies if "DEAD"  in a)
        stuck_count += sum(1 for a in s.anomalies if "STUCK" in a)

    print(sep)
    tested = sum(1 for s in all_stats if not s.skipped)
    print(
        f"  Faces tested: {tested}  |  Anomaly flags: {total_anomalies}"
        f"  |  Dead pads: {dead_count}  |  Stuck pads: {stuck_count}"
    )
    print(sep)
    print()


def write_summary_json(all_stats: list[FaceStats], path: str) -> None:
    data = {
        "generated_at": datetime.datetime.now().isoformat(),
        "faces": [dataclasses.asdict(s) for s in all_stats],
    }
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


# ── Session orchestrator ──────────────────────────────────────────────────────

class SurveySession:
    def __init__(
        self,
        backend: "RobotBackendProto",
        recorder: SurveyRecorder,
        module_ids: list[int],
        display: LiveDisplay,
    ) -> None:
        self._backend    = backend
        self._recorder   = recorder
        self._module_ids = module_ids
        self._display    = display
        self._completed_stats: list[FaceStats] = []
        self._loop = asyncio.get_running_loop()

    async def run(self) -> list[FaceStats]:
        if hasattr(self._backend, "sensor_poll_hz"):
            self._backend.sensor_poll_hz = min(
                SURVEY_SAMPLE_HZ, LOOP_LIMITS.sensor_poll_hz_max
            )

        # Wait for first sensor data to arrive
        await asyncio.sleep(0.5)

        total_faces = len(self._module_ids) * len(FACES)
        step = 0

        for module_id in self._module_ids:
            for face in FACES:
                step += 1
                retake = 0

                while True:
                    print(f"\n{'─' * 50}")
                    retake_str = f"  retake {retake}" if retake else ""
                    print(f"  Module {module_id}  |  Face: {face.upper()}  (step {step}/{total_faces}{retake_str})")
                    print(f"{'─' * 50}")

                    stats = await self._test_face(module_id, face, retake)

                    if stats.skipped:
                        print(f"  [{face.upper()} skipped]")
                        self._completed_stats.append(stats)
                        break

                    print(f"  {_format_result(stats)}")
                    response = await self._prompt(
                        "  Enter=next  r=redo  s=skip  q=quit: "
                    )
                    if response == "q":
                        raise KeyboardInterrupt
                    if response == "r":
                        retake += 1
                        continue
                    if response == "s":
                        self._completed_stats.append(
                            FaceStats(
                                module_id=module_id, face=face, retake=retake,
                                baseline_mean=[], activated_mean=[], delta=[],
                                fsr_baseline_mean=0.0, fsr_activated_mean=0.0,
                                fsr_delta=0.0, skipped=True,
                            )
                        )
                    else:
                        self._completed_stats.append(stats)
                    break

        return self._completed_stats

    async def _test_face(
        self, module_id: int, face: str, retake: int
    ) -> FaceStats:
        response = await self._prompt(
            f"\n  Do NOT touch the robot.\n"
            f"  Press Enter to collect baseline  (s=skip, q=quit): "
        )
        if response == "q":
            raise KeyboardInterrupt
        if response == "s":
            return FaceStats(
                module_id=module_id, face=face, retake=retake,
                baseline_mean=[], activated_mean=[], delta=[],
                fsr_baseline_mean=0.0, fsr_activated_mean=0.0, fsr_delta=0.0,
                skipped=True,
            )

        baseline = await self._collect_phase(module_id, face, "baseline", BASELINE_SAMPLES, retake)

        response = await self._prompt(
            f"\n  Now FIRMLY touch the {face.upper()} face of Module {module_id}.\n"
            f"  Hold steady, then press Enter to collect activation  (s=skip, q=quit): "
        )
        if response == "q":
            raise KeyboardInterrupt
        if response == "s":
            return FaceStats(
                module_id=module_id, face=face, retake=retake,
                baseline_mean=[], activated_mean=[], delta=[],
                fsr_baseline_mean=0.0, fsr_activated_mean=0.0, fsr_delta=0.0,
                skipped=True,
            )

        activated = await self._collect_phase(module_id, face, "activated", ACTIVATED_SAMPLES, retake)

        response = await self._prompt(
            f"\n  RELEASE the sensor (no touch).\n"
            f"  Press Enter to verify release  (q=quit): "
        )
        if response == "q":
            raise KeyboardInterrupt

        await self._collect_phase(module_id, face, "release", RELEASE_SAMPLES, retake)
        self._display.clear()

        return compute_face_stats(module_id, face, retake, baseline, activated)

    async def _collect_phase(
        self,
        module_id: int,
        face: str,
        phase: str,
        n_samples: int,
        retake: int,
    ) -> list[SurveyRecord]:
        records: list[SurveyRecord] = []
        interval = 1.0 / SURVEY_SAMPLE_HZ

        for i in range(n_samples):
            t0 = time.monotonic()
            state: "RobotState" = await self._backend.get_state()
            sensors = state.sensors.get(module_id)

            if sensors is not None:
                left_pads   = list(sensors.touch_left_pads)
                right_pads  = list(sensors.touch_right_pads)
                middle_pads = list(sensors.touch_middle_pads)
                left_fsr    = sensors.pressure_left
                right_fsr   = sensors.pressure_right
                middle_fsr  = sensors.pressure_middle
            else:
                left_pads   = [0.0] * PAD_COUNTS["left"]
                right_pads  = [0.0] * PAD_COUNTS["right"]
                middle_pads = [0.0] * PAD_COUNTS["middle"]
                left_fsr = right_fsr = middle_fsr = 0.0

            record = SurveyRecord(
                ts=time.time(),
                module_id=module_id,
                face=face,
                phase=phase,
                step_index=i,
                retake=retake,
                left_pads=left_pads,
                right_pads=right_pads,
                middle_pads=middle_pads,
                left_fsr=left_fsr,
                right_fsr=right_fsr,
                middle_fsr=middle_fsr,
            )
            self._recorder.write(record)
            records.append(record)

            self._display.render(module_id, face, phase, record, i + 1, n_samples)

            elapsed = time.monotonic() - t0
            remaining = interval - elapsed
            if remaining > 0:
                await asyncio.sleep(remaining)

        return records

    async def _prompt(self, message: str) -> str:
        try:
            response = await self._loop.run_in_executor(None, input, message)
            return response.strip().lower()
        except EOFError:
            return "q"


# ── Entry point ───────────────────────────────────────────────────────────────

async def main(
    host: str,
    port: int,
    modules: list[int],
    dry_run: bool,
    dry_run_mode: str,
) -> None:
    ts_str       = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    jsonl_path   = f"sensor_survey_{ts_str}.jsonl"
    summary_path = f"sensor_survey_{ts_str}_summary.json"

    if dry_run:
        from petctl.backends.mock import MockBackend
        backend: "RobotBackendProto" = MockBackend(
            mode=dry_run_mode, num_modules=max(modules) + 1
        )
        print(f"[sensor_survey] Using MockBackend (mode={dry_run_mode})")
    else:
        from petctl.backends.robot import RobotBackend
        backend = RobotBackend(
            host=host,
            port=port,
            calibrate_on_connect=False,
            auto_reconnect=False,
        )
        print(f"[sensor_survey] Connecting to {host}:{port} ...")

    ok = await backend.connect()
    if not ok:
        print("[sensor_survey] Connection failed.")
        return

    print(f"[sensor_survey] Connected. Modules to test: {modules}")
    print(f"[sensor_survey] Recording to: {jsonl_path}")

    display  = LiveDisplay(is_tty=sys.stdout.isatty())
    recorder = SurveyRecorder(jsonl_path)
    session  = SurveySession(backend, recorder, modules, display)

    all_stats: list[FaceStats] = []
    try:
        all_stats = await session.run()
    except KeyboardInterrupt:
        print("\n[sensor_survey] Interrupted — partial data saved.")
        all_stats = session._completed_stats
    finally:
        recorder.close()
        await backend.disconnect()

    if all_stats:
        print_summary_table(all_stats)
        write_summary_json(all_stats, summary_path)
        print(f"  Raw data : {jsonl_path}")
        print(f"  Summary  : {summary_path}")
    else:
        print(f"[sensor_survey] No completed faces. Raw samples in: {jsonl_path}")


if __name__ == "__main__":
    from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT

    parser = argparse.ArgumentParser(
        description="Interactive sensor survey for PET robot capacitive pads and FSRs"
    )
    parser.add_argument("--host",    default=ROBOT_DEFAULT_HOST)
    parser.add_argument("--port",    type=int, default=ROBOT_DEFAULT_PORT)
    parser.add_argument(
        "--modules",
        default="0,1,2,3,4,5,6,7",
        help="Comma-separated module IDs to test (default: all 8)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Use MockBackend instead of real robot",
    )
    parser.add_argument(
        "--dry-run-mode",
        default="noise",
        choices=["noise", "mock-sensor-sine"],
    )
    args = parser.parse_args()

    module_ids = [int(m.strip()) for m in args.modules.split(",") if m.strip()]
    asyncio.run(main(args.host, args.port, module_ids, args.dry_run, args.dry_run_mode))
