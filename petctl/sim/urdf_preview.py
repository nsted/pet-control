"""Visual + numeric validation of generated n-module URDFs: renders (top-down and oblique) and a table of module centres / pitch / mass.

``python -m petctl.sim.urdf_preview --n 8 9 10 11`` writes ``data/urdf_variants/previews/n<N>.png`` and ``n8_9_10_11.png``.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco  # type: ignore[import-untyped]
import numpy as np
from PIL import Image, ImageDraw

from petctl.sim.model_builder import BuildParams, PetSim, load_urdf, resolve_urdf_dir
from petctl.sim.urdf_gen import DEFAULT_SOURCE, VARIANTS_DIR, parse_chain


def _project(scene: mujoco.MjvScene, pts: np.ndarray, w: int, h: int, fovy_deg: float = 45.0) -> np.ndarray:
    """Pixel coordinates of world points ``pts`` (N, 3) for the first camera of ``scene``."""
    cam = scene.camera[0]
    pos = (np.array(cam.pos) + np.array(scene.camera[1].pos)) / 2
    fwd = np.array(cam.forward)
    up = np.array(cam.up)
    right = np.cross(fwd, up)
    d = pts - pos
    z = d @ fwd
    f = 0.5 * h / np.tan(np.radians(fovy_deg) / 2)
    return np.stack([w / 2 + f * (d @ right) / z, h / 2 - f * (d @ up) / z], axis=1)


def render(sim: PetSim, view: str, w: int = 900, h: int = 600) -> Image.Image:
    """Render the settled-at-q=0 robot, whole chain in frame, with module indices and a 10 cm scale bar."""
    m, d = sim.model, sim.data
    mujoco.mj_forward(m, d)
    gids = [m.geom(f"g{k}").id for k in range(m.nbody - 1)] if m.ngeom > 1 and "g0" in [m.geom(i).name for i in range(m.ngeom)] else []
    for k, g in enumerate(gids):                                          # colour by parity; head red, tail green
        m.geom_rgba[g] = (0.85, 0.25, 0.25, 1) if k == 0 else (0.25, 0.75, 0.35, 1) if k == len(gids) - 1 else (0.3, 0.55, 0.9, 1) if k % 2 == 0 else (0.95, 0.65, 0.2, 1)
    ctr = np.array([d.geom_xpos[g] for g in gids])
    lo, hi = ctr.min(0), ctr.max(0)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (lo + hi) / 2                                          # from geom_xpos after mj_forward
    span = float(np.linalg.norm(hi - lo)) + 0.12
    cam.distance = span * (0.85 if view == "top" else 1.1)
    cam.azimuth, cam.elevation = (90.0, -90.0) if view == "top" else (60.0, -38.0)
    m.vis.global_.offwidth, m.vis.global_.offheight = max(w, 640), max(h, 480)
    rend = mujoco.Renderer(m, height=h, width=w)
    rend.update_scene(d, camera=cam)
    img = Image.fromarray(rend.render())
    px = _project(rend.scene, ctr, w, h)
    dr = ImageDraw.Draw(img)
    for k, (x, y) in enumerate(px):
        dr.text((x - 4, y - 6), str(k), fill=(255, 255, 255), stroke_width=2, stroke_fill=(0, 0, 0))
    a = (lo + hi) / 2 - np.array([0.0, span / 3.5, 0.0])
    a[2] = ctr[:, 2].min() - 0.036
    seg = _project(rend.scene, np.array([a, a + [0.1, 0, 0]]), w, h)      # 10 cm along the robot x axis
    dr.line([tuple(seg[0]), tuple(seg[1])], fill=(255, 255, 255), width=3)
    dr.text((seg[0][0], seg[0][1] - 14), "10 cm", fill=(255, 255, 255))
    rend.close()
    return img


def table_row(n: int) -> dict:
    """Numbers for the doc: module centres along the chain (cm), pitch, joint axes (robot frame), URDF and sim total mass."""
    from petctl.sim.model_builder import _urdf_frame

    p = BuildParams(n_modules=n)
    d = resolve_urdf_dir(p) or str(DEFAULT_SOURCE)
    g = load_urdf(d)
    R = _urdf_frame(g)
    c = np.array([R @ x for x in g.centres])
    u = (c[0] - c[-1]) / np.linalg.norm(c[0] - c[-1])
    s = -(c - c[0]) @ u * 100.0                                            # cm from the head, along the chain
    axes = [R @ g.axes[k] for k in range(1, n)]
    mass = sum(lk["mass"] for lk in parse_chain(d)["links"])
    return {"n": n, "centres_cm": s, "pitch_cm": float(np.mean(np.diff(s))), "axes": axes, "mass_urdf": mass, "mass_sim": float(sum(p.masses))}


def main(argv: list[str] | None = None) -> None:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, nargs="+", default=[8, 9, 10, 11])
    ap.add_argument("--out", default=str(VARIANTS_DIR / "previews"))
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    tiles = []
    for n in a.n:
        sim = PetSim(BuildParams(n_modules=n, delay_s=0.0, dt=0.004, urdf_dir=str(DEFAULT_SOURCE)))
        sim.step(np.zeros(n - 1), 1.0)
        top, obl = render(sim, "top"), render(sim, "oblique")
        both = Image.new("RGB", (top.width, top.height * 2 + 30))
        both.paste(top, (0, 30))
        both.paste(obl, (0, top.height + 30))
        ImageDraw.Draw(both).text((8, 8), f"n = {n} modules (module 0 = head, {n - 1} = tail; top: looking down, bottom: oblique)", fill=(255, 255, 255))
        both.save(out / f"n{n}.png")
        tiles.append(both)
    comp = Image.new("RGB", (sum(t.width for t in tiles), tiles[0].height))
    x = 0
    for t in tiles:
        comp.paste(t, (x, 0))
        x += t.width
    comp.save(out / ("n" + "_".join(map(str, a.n)) + ".png"))
    print("wrote", out)


if __name__ == "__main__":
    main()
