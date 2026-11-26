#!/bin/bash
cd /Users/nsted/Documents/PET/code/petctl
export PETCTL_PROGRESS_EXPORT_S=900
mkdir -p data/rl/chain_c_terrain_ridges_warm data/rl/chain_c_terrain_ridges_scratch
python scripts/rl/chain_c_driver.py --stage terrain_ridges --root data/rl/chain_c_terrain_ridges_warm --warm data/rl/chain_c_terrain_rolling_d1/terrain_rolling/a1 > data/rl/chain_c_terrain_ridges_warm/driver.log 2>&1
python scripts/rl/chain_c_driver.py --stage terrain_ridges --root data/rl/chain_c_terrain_ridges_scratch --scratch > data/rl/chain_c_terrain_ridges_scratch/driver.log 2>&1
