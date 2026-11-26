#!/bin/bash
# dunes ladder with the std-runaway fix (ent_coef 0, log_std_init -1 reapplied at each warm start), from the level 0.25 pass; then C0 sparse_field from the last dune pass.
cd /Users/nsted/Documents/PET/code/petctl
export PETCTL_PROGRESS_EXPORT_S=900
prev=data/rl/chain_c_dunes5_L0.25/terrain_dunes/a1
for L in 0.3 0.35 0.4 0.5 0.6 0.7 0.8 1.0; do
  R=data/rl/chain_c_dunes7_L${L}; mkdir -p $R
  PETCTL_DUNES_LEVEL=$L python scripts/rl/chain_c_driver.py --stage terrain_dunes --root $R --warm $prev > $R/driver.log 2>&1
  [ -e $R/DONE ] || break
  A=$(python -c "import json;print([json.loads(l)['attempt'] for l in open('$R/summary.jsonl') if json.loads(l).get('phase')=='gate' and json.loads(l).get('passed')][-1])")
  prev=$R/terrain_dunes/a$A
done
R=data/rl/chain_c_sparse_field_fixed; mkdir -p $R
python scripts/rl/chain_c_driver.py --stage sparse_field --root $R --warm $prev > $R/driver.log 2>&1
