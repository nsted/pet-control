"""Visual style of the MuJoCo sim: sky, floor, light, robot gradient, terrain/wall/obstacle/target colors.

Purely cosmetic: it never touches masses, contacts, friction or sensors (``build_mjcf`` stays byte-identical; the style is applied to the XML string
and to the compiled model in ``PetSim``). Colors only show in renders (videos, viewer); training does not render.
"""

from __future__ import annotations

import colorsys

import mujoco  # type: ignore[import-untyped]
import numpy as np

# --- palette (r, g, b) in 0..1 ---------------------------------------------------------------------------------------------------------------------
SKY_TOP = (0.40, 0.58, 0.80)
SKY_HORIZON = (0.96, 0.90, 0.86)
FLOOR_A, FLOOR_B, FLOOR_LINE = (0.87, 0.79, 0.67), (0.82, 0.73, 0.61), (0.72, 0.62, 0.51)   # warm sand checker
TERRAIN_FIRM, TERRAIN_LOOSE = (0.55, 0.66, 0.60), (0.72, 0.58, 0.44)                         # sage / clay
WALL = (1.0, 0.97, 0.92)                                                                     # cream
OBSTACLE = (0.52, 0.44, 0.68)                                                                 # dusty violet
TARGET = (0.97, 0.76, 0.24)                                                                   # gold: contrasts with the cool robot
ROBOT_HUE_HEAD, ROBOT_HUE_TAIL = 0.93, 0.50                                                   # pink head -> violet -> blue -> teal tail


def _s(c: tuple[float, ...]) -> str:
    return " ".join(f"{v:.3f}" for v in c)


def module_rgb(k: int, n_modules: int) -> tuple[float, float, float]:
    """Color of module ``k`` (0 = head): a hue gradient head -> tail, with alternate modules slightly darker so segment boundaries read."""
    t = k / max(n_modules - 1, 1)
    hue = ROBOT_HUE_HEAD + (ROBOT_HUE_TAIL - ROBOT_HUE_HEAD) * t
    return colorsys.hsv_to_rgb(hue % 1.0, 0.55, 0.96 if k % 2 == 0 else 0.84)


def style_xml(xml: str, n_modules: int) -> str:
    """Return ``xml`` with the sky, floor colors, light, haze and robot materials injected (unchanged where an anchor string is missing)."""
    floor_old = 'rgb1=".25 .3 .35" rgb2=".18 .22 .27"'
    if floor_old in xml:
        xml = xml.replace(floor_old, f'rgb1="{_s(FLOOR_A)}" rgb2="{_s(FLOOR_B)}" mark="edge" markrgb="{_s(FLOOR_LINE)}"', 1)
    assets = (f'<texture name="sky" type="skybox" builtin="gradient" rgb1="{_s(SKY_TOP)}" rgb2="{_s(SKY_HORIZON)}" width="32" height="512"/>'
              + "".join(f'<material name="pet{k}" rgba="{_s(module_rgb(k, n_modules))} 1" specular="0.5" shininess="0.7"/>' for k in range(n_modules)))
    if "<asset>" in xml:
        xml = xml.replace("<asset>", "<asset>" + assets, 1)
    visual = (f'<visual><headlight ambient="0.16 0.16 0.20" diffuse="0.08 0.08 0.08" specular="0.02 0.02 0.02"/><rgba haze="{_s(SKY_HORIZON)} 1"/>'
              f'<map haze="0.2"/><quality shadowsize="2048"/></visual>')
    light = '<light name="sun" directional="true" pos="0 0 4" dir="-0.85 0.45 -0.45" diffuse="0.85 0.80 0.68" specular="0.25 0.25 0.25" castshadow="true"/>'
    return xml.replace("<worldbody>", visual + "<worldbody>" + light, 1)


def apply_style(model: mujoco.MjModel) -> None:
    """Color the geoms of a compiled model (robot gradient + glossy material, terrain, walls, obstacles, target)."""
    n_modules = model.nu + 1
    for gid in range(model.ngeom):
        name = model.geom(gid).name
        if name.startswith("terr"):
            model.geom_rgba[gid] = (*(TERRAIN_LOOSE if model.geom_rgba[gid][0] > 0.38 else TERRAIN_FIRM), 1.0)    # the builder marks loose tiles with a warmer rgba
        elif name.startswith("wall"):
            model.geom_rgba[gid] = (*WALL, 1.0)
        elif name.startswith("obs"):
            model.geom_rgba[gid] = (*OBSTACLE, 1.0)
        elif name.startswith("post"):
            model.geom_rgba[gid] = (*TARGET, 1.0)
        elif name.startswith("g") and name[1:].isdigit() and int(name[1:]) < n_modules:
            k = int(name[1:])
            mid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, f"pet{k}")
            model.geom_rgba[gid] = (*module_rgb(k, n_modules), 1.0)
            if mid >= 0:
                model.geom_matid[gid] = mid
