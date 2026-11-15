# Passed stages (L=8, held-out n=100, randomisation 0.6)

| Stage | Attempts | Success | Wilson 95% lo-hi | Threshold | t_goal median (s) | Bus mean (A, median) | Bus peak (A, median) | Episodes >10 A | Glitch | Train time (min) | Steps (M) | Steps/s | Lateral (m, median / share>=0.4) | Ablations (drop) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| flat_nav | 1 | 0.97 | 0.915-0.990 | 0.80 | 12.2 | 2.55 | 7.3 | 0.05 | 0.00 | 15 | 4.0 | 4462 | - | - |
| flat_omni | 2 (1 failed first) | 0.96 | 0.902-0.984 | 0.70 | 5.1 | 2.57 | 7.0 | 0.03 | 0.00 | 37 | 8.0 | 3581 | 0.68 / 0.96 | - |
| terrain_easy | 2 (1 failed first) | 1.00 | 0.963-1.000 | 0.75 | 4.8 | 2.32 | 6.8 | 0.05 | 0.00 | 70 | 6.6 | 1575 | - | - |

- **flat_nav**: skill `models/skills/nav_chain/flat_nav`; gate JSON `data/rl/chain_ab/flat_nav/a1/gate2_flat_nav.json`; final video `docs/media/final_chain_ab_flat_a1_20261115_155417.mp4`; fail reasons before passing: none
- **flat_omni**: skill `models/skills/nav_chain/flat_omni`; gate JSON `data/rl/chain_ab/flat_omni/a2/gate2_flat_omni.json`; final video `docs/media/final_chain_ab_flat_a2_20261115_163246.mp4`; fail reasons before passing: current budget: median bus_mean_a 2.55 (max 4.0), episodes over peak 0.06 (max 0.05)
- **terrain_easy**: skill `models/skills/nav_chain/terrain_easy`; gate JSON `data/rl/chain_ab/terrain_easy/a2/gate2_terrain_easy.json`; final video `docs/media/final_chain_ab_terrain_easy_a2_20261115_174341.mp4`; fail reasons before passing: current budget: median bus_mean_a 2.46 (max 4.0), episodes over peak 0.06 (max 0.05); glitch rate 0.010 > 0.0
