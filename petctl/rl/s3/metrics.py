"""S3 command-tracking metric (STAGE_3_GATES_PROPOSAL, Metric): windowed COM velocity in a segment-fixed body frame, 3-chord yaw, T = clip(1 - |v - v_cmd| / |v_cmd|, -1, 1), stop scored apart.

Pure functions so the training reward, the evaluator and the tests share one definition (the earlier eval ran the same gait for every command; the metric must be checkable offline).
"""

from __future__ import annotations

import math

import numpy as np

STOP_SPEED_M_S = 0.03            # S_stop = clip(1 - speed / 3 cm/s, -1, 1)
YAW_AGREE_RAD = 0.3              # chords must agree within this over the window (a shape change is not a turn)
MIN_CMD_V = 1e-6


def chords(n_modules: int) -> list[tuple[int, int]]:
    """The three module-index chords whose planar angle defines the body heading (tail -> head etc.)."""
    n = n_modules
    return [(0, n - 1), (n // 4, (3 * n) // 4), (n // 3, (2 * n) // 3)]


def chord_angles(pos: np.ndarray, ch: list[tuple[int, int]]) -> np.ndarray:
    """Planar angle (rad) of each chord from module positions ``pos`` (n, 3): atan2 of (a - b) in the floor plane."""
    return np.array([math.atan2(pos[a, 1] - pos[b, 1], pos[a, 0] - pos[b, 0]) for a, b in ch])


def window_velocity(com: np.ndarray, ang: np.ndarray, t_s: float, h0: float) -> tuple[np.ndarray, float]:
    """(v in the segment frame [vx, vy] m/s, yaw rate rad/s) over a window.

    ``com`` (k, 3) module-mean positions, ``ang`` (k, 3) chord angles at the window ticks (first = start, last = end), ``t_s`` the window length, ``h0`` the segment heading. Yaw is the median chord rate only when the
    three unwrapped chord displacements agree within ``YAW_AGREE_RAD``, else 0.
    """
    d = (com[-1] - com[0])[:2] / t_s
    c, s = math.cos(-h0), math.sin(-h0)
    v = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])
    a = np.unwrap(ang, axis=0)
    dch = (a[-1] - a[0]) / t_s
    yaw = float(np.median(dch)) if float(np.ptp(dch)) * t_s < YAW_AGREE_RAD else 0.0
    return v, yaw


def t_score(v: np.ndarray, yaw: float, cmd: np.ndarray) -> float | None:
    """T of one window for the command ``cmd`` = (vx, vy, wz): translation term, yaw term, or their mean for combined commands; None for a stop command."""
    vt = float(np.hypot(cmd[0], cmd[1]))
    wt = abs(float(cmd[2]))
    terms = []
    if vt > MIN_CMD_V:
        terms.append(float(np.clip(1.0 - np.hypot(v[0] - cmd[0], v[1] - cmd[1]) / vt, -1.0, 1.0)))
    if wt > MIN_CMD_V:
        terms.append(float(np.clip(1.0 - abs(yaw - cmd[2]) / wt, -1.0, 1.0)))
    return float(np.mean(terms)) if terms else None


def stop_score(v: np.ndarray, yaw: float = 0.0) -> float:
    """S_stop of one window: clip(1 - speed / 3 cm/s, -1, 1) (reported and gated against the hack policies; never a training reward for standing still)."""
    return float(np.clip(1.0 - float(np.hypot(v[0], v[1])) / STOP_SPEED_M_S, -1.0, 1.0))
