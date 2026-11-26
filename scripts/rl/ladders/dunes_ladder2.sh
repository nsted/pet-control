#!/bin/bash
cd /Users/nsted/Documents/PET/code/petctl
export PETCTL_PROGRESS_EXPORT_S=900
prev=data/rl/chain_c_dunes_L0.3/terrain_dunes/a1
for L in 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0; do
  R=data/rl/chain_c_dunes2_L${L}; mkdir -p $R
  PETCTL_DUNES_LEVEL=$L python scripts/rl/chain_c_driver.py --stage terrain_dunes --root $R --warm $prev > $R/driver.log 2>&1
  [ -e $R/DONE ] || exit 0
  A=$(python -c "import json;print([json.loads(l)['attempt'] for l in open('$R/summary.jsonl') if json.loads(l).get('phase')=='gate' and json.loads(l).get('passed')][-1])")
  prev=$R/terrain_dunes/a$A
done
