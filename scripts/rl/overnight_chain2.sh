#!/bin/sh
# Chained jobs so the machine is never idle: bigger gaps between 25 cm platforms (a fall cannot be recovered), then climbing lower steps.
cd "$(dirname "$0")/../.." || exit 1
export PYTHONPATH=scripts/rl
for g in 25 30 40; do
  PETCTL_PLATFORM_H=0.25 python scripts/rl/gap_cross_robust.py --gap $g --evals 600 --library 5 --out data/rl/gap_cross/high_n11_g${g}.json
done
for h in 0.06 0.08 0.10; do
  PETCTL_PLATFORM_H=$h python scripts/rl/climb_robust.py --evals 600 --library 5 --out data/rl/climb/h${h}_n11.json
done
echo done > data/rl/chain2.done
