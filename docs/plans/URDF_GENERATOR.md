# URDF generator (n-module bodies)

Code: `petctl/sim/urdf_gen.py` (generator), `petctl/sim/urdf_preview.py` (renders + table), builder hook `model_builder.resolve_urdf_dir`.
Owner's folder is only read. Output: `data/urdf_variants/PET_description_n<N>/` (git-ignored with the rest of `data/`).

## Analysis of the owner's 8-module export
- Links: `base_link` (head, m0), `module_1_1..module_6_1` (middle, mesh 14316 tris), `module_7_1` (tail, 13500 tris = same tessellation as the head). 7 `continuous` joints
  (no limits/dynamics), names `Revolute 1..9` in Fusion order (j7 is called `Revolute 9`); the generator names joints `Revolute k` in chain order.
- STL meshes are in the CAD WORLD frame at q=0 (mm, scale 0.001); each link's `visual`/`collision` origin is minus the cumulative joint offset, so
  the mesh lands exactly where the STL says. `model_builder.load_urdf` reads only the meshes (hulls, centroids) and, for n=8, the gravity model for the axes.
- URDF joint origins form a straight chain: j1 = (0,-0.090805,-0.051445), then (0, -/+0.001237, -0.050735) alternating (even joints -y, odd +y).
  URDF axes alternate (0, +/-0.707107, -0.707107) (odd +y). The mesh chain, however, runs along the 45 deg diagonal (0,1,1) of the y-z plane
  (pair shift T = (0, 0.07175, 0.07175) m = 10.15 cm, pitch ~5.07 cm) and the builder's mating-face axes are +y (odd) / -z (even): the URDF joint frames and the meshes
  are NOT mutually consistent (known, ignored by the builder). The generator repeats both the same way, so viewers show what the owner's export shows.
- Inertials: odd middle (1,3,5) and even middle (2,4,6) links: mass 0.27965, com y = +/-0.015207, iyz = -/+2e-6; they are related by a 180 deg turn about the link z axis
  (x,y -> -x,-y; ixz,iyz change sign). Head and tail: mass 0.27607, identical inertia tensor. (The sim builder uses its own uniform `masses`, not these.)
- **Repeating unit = a screw motion S**: 180 deg about the chain axis (0,1,1) + half a pair shift T/2. Verified on the meshes: module_{k+1} = S(module_k) for k=1..5 to
  3.5e-8 m, module_{k+2} = module_k + T, and the tail = head rotated 180 deg about the x axis (end-for-end).
- Parity-dependent: orientation of a middle module about the chain axis (even = odd turned 180 deg about the chain), sign of the joint-origin y offset, sign of the URDF
  axis y component, sign of the inertial com y / iyz. Position along the chain is parity independent (T per two modules).

## Generator rules
- modules 1..6: owner's verbatim (n=8 is therefore bit-identical in meshes, numerically identical in links/joints; test `test_n8_identical_to_owner`).
- middle k>=7: owner's module 5 (odd k) / 6 (even k) moved by S^(k-5) / S^(k-6) (mesh), inertial of 5/6, joint origin (0, -/+0.001237, -0.050735).
- head unchanged; tail = owner's `module_7_1` moved by S^(n-8).
- **`tail_flip` (the assumption to review)**: S^(n-8) is a pure translation for even n; for odd n it also contains the 180 deg rotation about the chain axis. So
  `tail_flip=None` (auto) means: turn the tail 180 deg about the chain axis exactly for odd n, because the module it is mounted on (n-2) then has the other parity
  than module 6 of the 8-module robot. Derivation: tail_n must sit against module n-2 as the owner's tail sits against module 6; S maps (m6, tail8) to (m7, S tail8),
  so the mating patch is preserved for S^s; the unflipped tail would be a different orientation of the same shape against an opposite-parity module. Checks:
  mating vertices coincide exactly with flip and fail without (`test_tail_flip_is_needed_for_odd_n`); the rendered odd-n chains are head/tail symmetric. The inertial
  of an odd-n tail is the owner's tail inertial turned 180 deg about link z (com y, iyz sign flipped). Override: `tail_flip=True/False` (CLI `--tail-flip yes|no|auto`).
  Not verifiable from the CAD alone: whether the physical tail module can be mounted in both orientations (the owner should confirm for odd n).

## Builder integration
`BuildParams.n_modules != 8` always uses `urdf_gen.variant_dir(n)` (generated on first use, atomic, cached with a manifest incl. a hash of the owner's meshes); `_extend_chain` removed.
`load_urdf` takes the mesh count from the folder; for n != 8 axes/anchors come from the mating-face normals with the sign rule (+y, then minus the screw-rotated previous axis);
applied to the owner's 8 meshes this reproduces the gravity-model-based axes/anchors exactly (diff 0.0). n=8 MJCF hashes unchanged (5-case test passes).
Per-module masses (uniform `MODULE_MASS_KG`), `cap_nm`, `coulomb` (median of measured middle joints for new joints, tail joint keeps its own), armature, kp/kd, delay: unchanged
`BuildParams.__post_init__` rules, unaffected by the geometry source. Note: for even n the lying chain is tilted ~5 deg as for n=8 (frame from head->tail centres); odd n is level.

## Numbers (robot frame; centres = hull centroids along head->tail, cm)
Adjacent hulls touch (gap 0.000 mm, no penetration) and skip-one hulls keep 1.76 mm clearance for every n = 8..15, same as the owner's 8; zero-command 3 s settle: finite, |q| < 0.03 rad, |qdot| < 0.02, n = 8..15.
| n | tail module centre (cm) | pitch (cm) | URDF mass (kg) | sim mass (kg) |
|---|---|---|---|---|
| 8 | 35.53 | 5.08 | 2.230 | 2.200 |
| 9 | 40.46 | 5.06 | 2.510 | 2.475 |
| 10 | 45.65 | 5.07 | 2.789 | 2.750 |
| 11 | 50.60 | 5.06 | 3.069 | 3.025 |
| 12 | 55.77 | 5.07 | 3.349 | 3.300 |
| 13 | 60.75 | 5.06 | 3.628 | 3.575 |
| 14 | 65.90 | 5.07 | 3.908 | 3.850 |
| 15 | 70.90 | 5.06 | 4.188 | 4.125 |

Centres, n=9 (cm): 0, 5.01, 10.09, 15.16, 20.24, 25.30, 30.39, 35.45, 40.46. n=10: 0, 5.23, 10.07, 15.36, 20.19, 25.48, 30.31, 35.60, 40.43, 45.65. (Even n: the tail end is shifted ~0.2 cm by its parity.)
Joint axes (robot frame, joint 1..n-1): odd n gives alternating (-0.70,0,0.71), (0.70,0,0.71); even n (and the owner's 8) is the same pattern rotated 5 deg by the head->tail frame choice, e.g. (-0.76,0,0.66), (0.65,0,0.76).
Previews: `data/urdf_variants/previews/n8.png ... n11.png`, comparison `n8_9_10_11.png` (head red, tail green, odd modules orange, even blue, 10 cm bar).

## Use
`python -m petctl.sim.urdf_gen --n 9 --out data/urdf_variants/PET_description_n9`; then `xacro urdf/pet.xacro > pet.urdf` (ROS: package name `pet_description`, e.g.
`roslaunch` display.launch after placing the folder in a catkin workspace), or open `urdf/pet.xacro` in any xacro-aware URDF viewer (e.g. urdf-viewer / Foxglove / RViz).
Regenerate previews: `python -m petctl.sim.urdf_preview --n 8 9 10 11`.
