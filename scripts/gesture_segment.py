#!/usr/bin/env python3
"""
Segment gesture events from recorded sensor streams.

Two modes:

  Trim mode (default): reads labeled JSONL from gesture_recorder.py and trims
  baseline frames from the start/end of each session, keeping only the frames
  where sensor contact is actually detected.

  Interactive mode (--interactive): detects events from any JSONL stream
  (labeled or not) and prompts for a label on each one. Useful for processing
  cap_monitor --log output or reviewing unlabeled recordings.

Output is compatible with gesture_analyze.py.

Usage:
    python scripts/gesture_segment.py gesture_data_<ts>.jsonl
    python scripts/gesture_segment.py gesture_data_<ts>.jsonl --interactive
    python scripts/gesture_segment.py cap_monitor_<ts>.jsonl --interactive
    python scripts/gesture_segment.py data.jsonl --out trimmed.jsonl
    python scripts/gesture_segment.py data.jsonl --gap 0.3 --min-dur 0.2
"""

from __future__ import annotations

import argparse
import collections
import json
import math
from collections import deque
from pathlib import Path

from petctl.perception.stroke import (
    CradleDetector,
    HoldDetector,
    MIN_CONFIDENCE,
    PAD_THRESHOLD,
    TOUCH_THRESHOLD,
    VELOCITY_THRESHOLD,
    WINDOW_FRAMES,
    StrokeDetector,
    _find_blobs,
    _find_qualifying_modules,
    _linear_fit,
    _pad_centroid,
)
from petctl.types import ModuleSensors, RobotState

LABEL_KEYS: dict[str, str] = {
    "s": "stroke",
    "h": "hold",
    "p": "poke",
    "q": "squeeze",
    "c": "cradle",
    "b": "baseline",
    "n": "none",
}


# ---------------------------------------------------------------------------
# State reconstruction (same as gesture_analyze.py)
# ---------------------------------------------------------------------------

def _make_state(record: dict) -> RobotState:
    sensors: dict[int, ModuleSensors] = {}
    for mid_str, sd in record["sensors"].items():
        mid = int(mid_str)
        sensors[mid] = ModuleSensors(
            module_id=mid,
            touch_left_pads=tuple(sd["left_pads"]),
            touch_right_pads=tuple(sd["right_pads"]),
            touch_middle_pads=tuple(sd["middle_pads"]),
            pressure_left=sd["left_fsr"],
            pressure_right=sd["right_fsr"],
            pressure_middle=sd["middle_fsr"],
        )
    motor = record.get("motor", {})
    return RobotState(
        timestamp=record["ts"],
        sensors=sensors,
        motor_velocities={int(k): v for k, v in motor.get("vel", {}).items()},
        motor_torques={int(k): v for k, v in motor.get("torque", {}).items()},
        servo_positions={int(k): v for k, v in motor.get("pos", {}).items()},
    )


def _is_contact(record: dict) -> bool:
    """True if the frame has any detectable pad contact."""
    state = _make_state(record)
    centroid, _ = _pad_centroid(state)
    return centroid is not None


# ---------------------------------------------------------------------------
# Event detection
# ---------------------------------------------------------------------------

def detect_events(
    records: list[dict],
    gap_grace_s: float = 0.5,
    min_duration_s: float = 0.15,
    pre_pad: int = 2,
    post_pad: int = 3,
) -> list[list[dict]]:
    """Find contiguous contact spans in a record stream.

    Returns a list of events, each a list of frame records (with pre/post padding).
    Gaps shorter than gap_grace_s are bridged. Events shorter than min_duration_s
    are discarded.
    """
    n = len(records)
    if n == 0:
        return []

    # Mark which frames have contact
    contact = [_is_contact(r) for r in records]

    # Bridge gaps shorter than gap_grace_s
    i = 0
    while i < n:
        if not contact[i]:
            # Find end of this gap
            j = i
            while j < n and not contact[j]:
                j += 1
            # Measure gap duration
            if i > 0 and j < n:
                t_gap = records[j]["ts"] - records[i - 1]["ts"]
                if t_gap <= gap_grace_s:
                    for k in range(i, j):
                        contact[k] = True
            i = j
        else:
            i += 1

    # Collect contiguous active spans
    events: list[list[dict]] = []
    i = 0
    while i < n:
        if contact[i]:
            j = i
            while j < n and contact[j]:
                j += 1
            # Span is [i, j)
            duration = records[j - 1]["ts"] - records[i]["ts"]
            if duration >= min_duration_s:
                start = max(0, i - pre_pad)
                end = min(n, j + post_pad)
                events.append(records[start:end])
            i = j
        else:
            i += 1

    return events


# ---------------------------------------------------------------------------
# Per-event feature summary
# ---------------------------------------------------------------------------

def _event_summary(event_records: list[dict]) -> dict:
    """Compute aggregate features for one event."""
    centroid_window: deque[tuple[float, float]] = deque(maxlen=WINDOW_FRAMES)
    vel_samples: list[float] = []
    fsr_peaks: list[float] = []
    qual_mods_list: list[int] = []

    for rec in event_records:
        state = _make_state(rec)
        centroid, _ = _pad_centroid(state)
        if centroid is not None:
            centroid_window.append((state.timestamp, centroid))
        if len(centroid_window) >= 5:
            vel, _ = _linear_fit(centroid_window)
            vel_samples.append(abs(vel))
        fsr_vals = [
            v
            for s in state.sensors.values()
            for v in (s.pressure_left, s.pressure_right, s.pressure_middle)
        ]
        fsr_peaks.append(max(fsr_vals) if fsr_vals else 0.0)
        qual_mods_list.append(len(_find_qualifying_modules(state, PAD_THRESHOLD)))

    duration = event_records[-1]["ts"] - event_records[0]["ts"] if event_records else 0.0
    mean_vel = sum(vel_samples) / len(vel_samples) if vel_samples else 0.0
    std_vel = math.sqrt(
        sum((v - mean_vel) ** 2 for v in vel_samples) / len(vel_samples)
    ) if vel_samples else 0.0
    peak_fsr = max(fsr_peaks) if fsr_peaks else 0.0
    max_qual = max(qual_mods_list) if qual_mods_list else 0

    return {
        "duration": duration,
        "frames": len(event_records),
        "centroid_vel_mean": mean_vel,
        "centroid_vel_std": std_vel,
        "fsr_peak": peak_fsr,
        "qualifying_mods_max": max_qual,
    }


def _event_detector_str(event_records: list[dict]) -> str:
    """Run live detectors over the event and return the most common classification."""
    stroke_det = StrokeDetector()
    hold_det = HoldDetector()
    cradle_det = CradleDetector()
    counts: dict[str, int] = collections.Counter()

    for rec in event_records:
        state = _make_state(rec)
        if cradle_det.update(state):
            counts["CRADLE"] += 1
        elif stroke_det.update(state):
            counts["STROKE"] += 1
        elif hold_det.update(state):
            counts["HOLD"] += 1
        else:
            counts["none"] += 1

    if not counts:
        return "none"
    top = max(counts, key=counts.__getitem__)
    total = sum(counts.values())
    return f"{top} ({counts[top]}/{total} frames)"


# ---------------------------------------------------------------------------
# Trim mode
# ---------------------------------------------------------------------------

def trim_labeled(
    records: list[dict],
    gap_grace_s: float,
    min_duration_s: float,
    pre_pad: int,
    post_pad: int,
) -> tuple[list[dict], dict[str, tuple[int, int]]]:
    """Trim baseline frames within each labeled session.

    Returns (trimmed_records, stats_per_label) where stats is {label: (before, after)}.
    """
    # Group by session_id
    sessions: dict[int | None, list[dict]] = collections.defaultdict(list)
    for rec in records:
        sessions[rec.get("session_id")].append(rec)

    label_stats: dict[str, list[int]] = collections.defaultdict(lambda: [0, 0])
    out: list[dict] = []

    for session_id, session_recs in sessions.items():
        label = session_recs[0].get("label") if session_recs else None

        # Unlabeled frames (label=None or missing) — pass through without trimming
        if not label:
            out.extend(session_recs)
            continue

        before = len(session_recs)
        events = detect_events(
            session_recs,
            gap_grace_s=gap_grace_s,
            min_duration_s=min_duration_s,
            pre_pad=pre_pad,
            post_pad=post_pad,
        )
        # Re-stamp the label (pre/post pad frames may have come from unlabeled spans)
        for event in events:
            for rec in event:
                rec = dict(rec)
                rec["label"] = label
                rec["session_id"] = session_id
                out.append(rec)

        after = sum(len(e) for e in events)
        label_stats[label][0] += before
        label_stats[label][1] += after

    return out, {lb: (v[0], v[1]) for lb, v in label_stats.items()}


# ---------------------------------------------------------------------------
# Interactive mode
# ---------------------------------------------------------------------------

def interactive_label(
    records: list[dict],
    gap_grace_s: float,
    min_duration_s: float,
    pre_pad: int,
    post_pad: int,
) -> list[dict]:
    """Detect events and prompt the user to label each one."""
    events = detect_events(
        records,
        gap_grace_s=gap_grace_s,
        min_duration_s=min_duration_s,
        pre_pad=pre_pad,
        post_pad=post_pad,
    )
    print(f"Found {len(events)} events.\n")

    key_hint = "  ".join(f"{k}={v}" for k, v in LABEL_KEYS.items())
    print(f"Keys: {key_hint}")
    print("Press Enter to skip an event.\n")

    out: list[dict] = []
    session_id = 0

    for i, event in enumerate(events):
        summary = _event_summary(event)
        detector_str = _event_detector_str(event)

        print(
            f"Event {i + 1}/{len(events)}  |  "
            f"dur={summary['duration']:.2f}s  frames={summary['frames']}  |  "
            f"vel={summary['centroid_vel_mean']:.2f}±{summary['centroid_vel_std']:.2f}  "
            f"fsr_peak={summary['fsr_peak']:.2f}  "
            f"qual_mods={summary['qualifying_mods_max']}"
        )
        print(f"  Detector: {detector_str}")

        while True:
            raw = input("  Label: ").strip().lower()
            if raw == "":
                print("  (skipped)\n")
                label = None
                break
            if raw in LABEL_KEYS:
                label = LABEL_KEYS[raw]
                break
            if raw in LABEL_KEYS.values():
                label = raw
                break
            print(f"  Unknown label '{raw}'. Options: {key_hint}")

        if label is not None:
            session_id += 1
            for rec in event:
                rec = dict(rec)
                rec["label"] = label
                rec["session_id"] = session_id
                out.append(rec)
            print(f"  → labeled as '{label}'\n")

    return out


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Segment and trim gesture events from sensor JSONL streams"
    )
    parser.add_argument("file", metavar="FILE",
                        help="JSONL file from gesture_recorder.py or cap_monitor.py")
    parser.add_argument("--out", default=None, metavar="PATH",
                        help="Output path (default: <stem>_segmented.jsonl)")
    parser.add_argument("--interactive", action="store_true",
                        help="Detect events and label them interactively")
    parser.add_argument("--gap", type=float, default=0.5, metavar="SECONDS",
                        help="Gap grace period in seconds (default: 0.5)")
    parser.add_argument("--min-dur", type=float, default=0.15, metavar="SECONDS",
                        help="Minimum event duration to keep (default: 0.15s)")
    parser.add_argument("--pre-pad", type=int, default=2, metavar="FRAMES",
                        help="Frames to include before contact onset (default: 2)")
    parser.add_argument("--post-pad", type=int, default=3, metavar="FRAMES",
                        help="Frames to include after contact offset (default: 3)")
    args = parser.parse_args()

    in_path = args.file
    out_path = args.out or str(Path(in_path).stem) + "_segmented.jsonl"

    print(f"[gesture_segment] Loading {in_path} ...")
    records: list[dict] = []
    with open(in_path) as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    print(f"[gesture_segment] {len(records)} records loaded.")

    if args.interactive:
        out_records = interactive_label(
            records,
            gap_grace_s=args.gap,
            min_duration_s=args.min_dur,
            pre_pad=args.pre_pad,
            post_pad=args.post_pad,
        )
    else:
        out_records, stats = trim_labeled(
            records,
            gap_grace_s=args.gap,
            min_duration_s=args.min_dur,
            pre_pad=args.pre_pad,
            post_pad=args.post_pad,
        )
        print("\nTrim summary:")
        for label, (before, after) in sorted(stats.items()):
            pct = 100 * after / before if before else 0
            print(f"  {label:<12}  {before:>5} → {after:>5} frames  ({pct:.0f}% kept)")

    with open(out_path, "w") as f:
        for rec in out_records:
            f.write(json.dumps(rec) + "\n")

    print(f"\n[gesture_segment] {len(out_records)} frames written to {out_path}")
    print(f"[gesture_segment] Run: python scripts/gesture_analyze.py {out_path} --tree")


if __name__ == "__main__":
    main()
