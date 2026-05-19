"""Sweep curl-right joint angles and compute FK head/tail distance."""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

ASSEMBLY = os.path.join(os.path.dirname(__file__), "../petctl/assets/robot_assembly.json")


def hpr_to_mat3(h_deg, p_deg, r_deg):
    h, p, r = math.radians(h_deg), math.radians(p_deg), math.radians(r_deg)
    ch, sh = math.cos(h), math.sin(h)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)
    Rz = np.array([[ch, -sh, 0.], [sh, ch, 0.], [0., 0., 1.]])
    Rx = np.array([[1., 0., 0.], [0., cp, -sp], [0., sp, cp]])
    Ry = np.array([[cr, 0., sr], [0., 1., 0.], [-sr, 0., cr]])
    return Rz @ Rx @ Ry


def axis_angle_mat3(axis, angle_rad):
    ax, ay, az = axis
    c, s, t = math.cos(angle_rad), math.sin(angle_rad), 1. - math.cos(angle_rad)
    return np.array([
        [t*ax*ax + c,    t*ax*ay - s*az, t*ax*az + s*ay],
        [t*ax*ay + s*az, t*ay*ay + c,    t*ay*az - s*ax],
        [t*ax*az - s*ay, t*ay*az + s*ax, t*az*az + c],
    ])


with open(ASSEMBLY) as f:
    data = json.load(f)

modules = data["assembly"]["modules"]
module_map = {int(m["id"]): m for m in modules}

hpr_mats = {int(m["id"]): hpr_to_mat3(*m.get("rotation", [0, 0, 0])) for m in modules}
offsets   = {int(m["id"]): m.get("offset", [0., 0., 0.]) for m in modules}
axes      = {int(m["id"]): tuple(m.get("joint", {}).get("axis", [0, 0, 1])) for m in modules}
parents   = {int(m["id"]): (int(m["parent"]) if m.get("parent") is not None else None) for m in modules}


def ancestor_chain(mod_id):
    chain = []
    cur = mod_id
    while cur is not None:
        chain.append(cur)
        cur = parents.get(cur)
    chain.reverse()
    return chain


# curl-right sign: index in sorted servo IDs [1..7]
# i=0 (sid=1): +1,  i=1 (sid=2): -1,  i=2 (sid=3): +1 ...
def curl_right_sign(servo_id):
    i = servo_id - 1  # sid 1→i=0, sid 2→i=1, ...
    return 1.0 if i % 2 == 0 else -1.0


def fk_pos(mod_id, servo_angles_rad):
    """Return world position of mod_id's joint origin given servo angles dict (rad)."""
    pos = np.zeros(3)
    R = np.eye(3)
    for mid in ancestor_chain(mod_id):
        pos = pos + R @ np.array(offsets[mid])
        # viz negates servo angle
        angle_rad = -servo_angles_rad.get(mid, 0.0)
        R = R @ (hpr_mats[mid] @ axis_angle_mat3(axes[mid], angle_rad))
    return pos


print(f"{'angle_deg':>10}  {'head_pos':>32}  {'tail_pos':>32}  {'dist':>8}")
print("-" * 90)

best_angle = 0.0
best_dist = float("inf")

for angle_deg in range(0, 110, 2):
    angle_rad = math.radians(angle_deg)
    servo_angles = {sid: curl_right_sign(sid) * angle_rad for sid in range(1, 8)}

    head_pos = fk_pos(0, servo_angles)   # head joint origin
    tail_pos = fk_pos(7, servo_angles)   # tail joint origin

    dist = float(np.linalg.norm(head_pos - tail_pos))
    print(f"{angle_deg:>10.1f}  {str(head_pos.round(2)):>32}  {str(tail_pos.round(2)):>32}  {dist:>8.2f}")

    if dist < best_dist:
        best_dist = dist
        best_angle = angle_deg

print()
print(f"Closest approach: {best_angle:.1f}° per joint, distance {best_dist:.2f} cm")
