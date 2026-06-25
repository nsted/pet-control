#!/usr/bin/env python3
"""
Analyze recorded gesture data from gesture_recorder.py.

Reconstructs perception features from raw sensor records, computes per-class
statistics, and optionally fits a shallow decision tree to derive data-driven
thresholds for the live detectors.

Usage:
    python scripts/gesture_analyze.py gesture_data_*.jsonl
    python scripts/gesture_analyze.py data.jsonl --labels stroke,hold,squeeze
    python scripts/gesture_analyze.py data.jsonl --hist centroid_vel
    python scripts/gesture_analyze.py data.jsonl --tree
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import sys
from collections import deque

from petctl.perception.stroke import (
    MIN_CONFIDENCE,
    PAD_THRESHOLD,
    TOUCH_THRESHOLD,
    VELOCITY_THRESHOLD,
    WINDOW_FRAMES,
    _find_blobs,
    _find_qualifying_modules,
    _linear_fit,
    _pad_centroid,
)
from petctl.types import ModuleSensors, RobotState

FEATURE_NAMES = [
    "centroid_pos",
    "centroid_vel",
    "centroid_r2",
    "active_modules",
    "qualifying_mods",
    "fsr_total",
    "fsr_peak",
    "blob_count",
    "torque_peak",
]

# Threshold annotations printed alongside stats for reference
THRESHOLDS: dict[str, tuple[str, float]] = {
    "centroid_vel": ("VELOCITY_THRESHOLD", VELOCITY_THRESHOLD),
    "centroid_r2": ("MIN_CONFIDENCE", MIN_CONFIDENCE),
}


def _load_records(paths: list[str]) -> list[dict]:
    records = []
    for path in paths:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line:
                    records.append(json.loads(line))
    return records


def _make_state(record: dict) -> RobotState:
    """Reconstruct a minimal RobotState from a JSONL record."""
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


def _compute_features(records: list[dict]) -> list[tuple[str, dict[str, float]]]:
    """Return (label, feature_dict) for every labeled frame."""
    centroid_window: deque[tuple[float, float]] = deque(maxlen=WINDOW_FRAMES)
    last_session_id: int | None = None
    results: list[tuple[str, dict[str, float]]] = []

    for rec in records:
        label = rec.get("label")
        if not label:
            continue

        session_id = rec.get("session_id")
        if session_id != last_session_id:
            centroid_window.clear()
            last_session_id = session_id

        state = _make_state(rec)

        centroid, _ = _pad_centroid(state)
        if centroid is not None:
            centroid_window.append((state.timestamp, centroid))

        centroid_vel = 0.0
        centroid_r2 = 0.0
        if len(centroid_window) >= 5:
            vel, r2 = _linear_fit(centroid_window)
            centroid_vel = abs(vel)
            centroid_r2 = r2

        active_modules = sum(
            1 for s in state.sensors.values() if s.touch_total >= TOUCH_THRESHOLD
        )
        qualifying_mods = len(_find_qualifying_modules(state, PAD_THRESHOLD))

        activations = {mid: s.touch_total for mid, s in state.sensors.items()}
        blob_count = len(_find_blobs(activations))

        fsr_vals = [
            v
            for s in state.sensors.values()
            for v in (s.pressure_left, s.pressure_right, s.pressure_middle)
        ]
        fsr_total = sum(fsr_vals)
        fsr_peak = max(fsr_vals) if fsr_vals else 0.0

        torques = [abs(v) for v in state.motor_torques.values()]
        torque_peak = max(torques) if torques else 0.0

        results.append((label, {
            "centroid_pos": centroid if centroid is not None else 0.0,
            "centroid_vel": centroid_vel,
            "centroid_r2": centroid_r2,
            "active_modules": float(active_modules),
            "qualifying_mods": float(qualifying_mods),
            "fsr_total": fsr_total,
            "fsr_peak": fsr_peak,
            "blob_count": float(blob_count),
            "torque_peak": torque_peak,
        }))

    return results


def _stats(values: list[float]) -> tuple[float, float, float, float]:
    if not values:
        return 0.0, 0.0, 0.0, 0.0
    n = len(values)
    mean = sum(values) / n
    std = math.sqrt(sum((v - mean) ** 2 for v in values) / n)
    return mean, std, min(values), max(values)


def _print_stats_table(
    labeled: dict[str, list[dict[str, float]]],
    labels: list[str],
) -> None:
    col_w = 18
    sep = " | "
    header = f"{'Feature':<18}{sep}" + sep.join(f"{lb:>{col_w}}" for lb in labels)
    print(header)
    print("─" * len(header))

    for feat in FEATURE_NAMES:
        row_cells = []
        for lb in labels:
            vals = [f[feat] for f in labeled.get(lb, [])]
            if not vals:
                row_cells.append(f"{'n/a':>{col_w}}")
            else:
                mean, std, mn, mx = _stats(vals)
                row_cells.append(f"{f'{mean:.3f} ±{std:.3f}':>{col_w}}")
        print(f"{feat:<18}{sep}" + sep.join(row_cells))
        if feat in THRESHOLDS:
            name, val = THRESHOLDS[feat]
            print(f"  {'↑ ' + name + ' = ' + str(val)}")

    print()


def _print_histogram(
    labeled: dict[str, list[dict[str, float]]],
    labels: list[str],
    feature: str,
    width: int = 40,
) -> None:
    all_vals = [f[feature] for lb in labels for f in labeled.get(lb, [])]
    if not all_vals:
        print(f"No data for feature: {feature}")
        return

    lo, hi = min(all_vals), max(all_vals)
    if hi <= lo:
        hi = lo + 1.0
    n_bins = 20

    print(f"\n  Histogram: {feature}  [{lo:.3f} – {hi:.3f}]")
    for lb in labels:
        vals = [f[feature] for f in labeled.get(lb, [])]
        if not vals:
            continue
        bins = [0] * n_bins
        for v in vals:
            idx = min(int((v - lo) / (hi - lo) * n_bins), n_bins - 1)
            bins[idx] += 1
        max_count = max(bins) if any(bins) else 1
        print(f"\n  {lb}  (n={len(vals)})")
        for i, count in enumerate(bins):
            bar_len = round(count / max_count * width) if max_count else 0
            lo_e = lo + i * (hi - lo) / n_bins
            hi_e = lo + (i + 1) * (hi - lo) / n_bins
            bar = "#" * bar_len + "." * (width - bar_len)
            print(f"    [{lo_e:6.3f}-{hi_e:6.3f}] [{bar}] {count}")

    if feature in THRESHOLDS:
        name, val = THRESHOLDS[feature]
        print(f"\n  ↑ current threshold: {name} = {val}")


def _fit_decision_tree(
    labeled: dict[str, list[dict[str, float]]],
    labels: list[str],
) -> None:
    try:
        from sklearn.metrics import classification_report, confusion_matrix
        from sklearn.tree import DecisionTreeClassifier, export_text
    except ImportError:
        print("scikit-learn not installed. Run: pip install scikit-learn")
        return

    X, y = [], []
    for lb in labels:
        for feat_dict in labeled.get(lb, []):
            X.append([feat_dict[f] for f in FEATURE_NAMES])
            y.append(lb)

    if len(set(y)) < 2:
        print("Need at least 2 different labels to fit a tree.")
        return

    clf = DecisionTreeClassifier(max_depth=4, min_samples_leaf=5, random_state=0)
    clf.fit(X, y)

    print("\n── Decision Tree Rules ─────────────────────────────────────────────")
    print(export_text(clf, feature_names=FEATURE_NAMES))

    y_pred = clf.predict(X)
    present = sorted(set(y))

    print("── Confusion Matrix ─────────────────────────────────────────────────")
    cm = confusion_matrix(y, y_pred, labels=present)
    header = f"{'true \\ pred':>14} | " + " | ".join(f"{lb[:10]:>10}" for lb in present)
    print(header)
    print("─" * len(header))
    for i, true_lb in enumerate(present):
        row = f"{true_lb[:14]:>14} | " + " | ".join(f"{cm[i][j]:>10}" for j in range(len(present)))
        print(row)

    print("\n── Classification Report ─────────────────────────────────────────────")
    print(classification_report(y, y_pred))
    accuracy = sum(a == b for a, b in zip(y, y_pred)) / len(y)
    print(f"Training accuracy: {accuracy:.1%}  (overfits — use tree rules as threshold guidance)")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze gesture_recorder.py JSONL for threshold tuning"
    )
    parser.add_argument("files", nargs="+", metavar="FILE")
    parser.add_argument(
        "--labels", default=None, metavar="LABELS",
        help="Comma-separated labels to include (default: all found)",
    )
    parser.add_argument(
        "--hist", default=None, metavar="FEATURE",
        help=f"Print ASCII histogram for a feature. Options: {', '.join(FEATURE_NAMES)}",
    )
    parser.add_argument(
        "--tree", action="store_true",
        help="Fit a shallow decision tree and print rules (requires scikit-learn)",
    )
    args = parser.parse_args()

    print(f"[gesture_analyze] Loading {len(args.files)} file(s)...")
    records = _load_records(args.files)
    print(f"[gesture_analyze] {len(records)} records loaded.")

    labeled_pairs = _compute_features(records)
    if not labeled_pairs:
        print("No labeled records found.")
        return

    labeled: dict[str, list[dict[str, float]]] = collections.defaultdict(list)
    for lb, feats in labeled_pairs:
        labeled[lb].append(feats)

    labels = (
        [l.strip() for l in args.labels.split(",")]
        if args.labels
        else sorted(labeled.keys())
    )

    print(f"[gesture_analyze] Labels: {labels}")
    print(
        "[gesture_analyze] Frames: "
        + ", ".join(f"{lb}={len(labeled[lb])}" for lb in labels)
    )
    print()

    _print_stats_table(labeled, labels)

    if args.hist:
        if args.hist not in FEATURE_NAMES:
            print(f"Unknown feature '{args.hist}'. Options: {', '.join(FEATURE_NAMES)}")
        else:
            _print_histogram(labeled, labels, args.hist)

    if args.tree:
        _fit_decision_tree(labeled, labels)


if __name__ == "__main__":
    main()
