#!/bin/bash
# Waits for ladder 5. If it stopped at level 0.3: try 0.275 (warm from the L0.25 pass). If 0.275 also fails (or the ladder stopped later), the dune limit is bracketed: move on to C0 sparse_field warm from the best dune pass.
cd /Users/nsted/Documents/PET/code/petctl
export PETCTL_PROGRESS_EXPORT_S=900
while pgrep -f dunes_ladder5.sh > /dev/null; do sleep 60; done
best=""; for L in 0.25 0.3 0.35 0.4 0.45 0.5 0.6 0.7 0.8 0.9 1.0; do
  R=data/rl/chain_c_dunes5_L${L}
  if [ -e $R/DONE ]; then A=$(python -c "import json;print([json.loads(l)['attempt'] for l in open('$R/summary.jsonl') if json.loads(l).get('phase')=='gate' and json.loads(l).get('passed')][-1])"); best=$R/terrain_dunes/a$A; lvl=$L; fi
done
[ -z "$best" ] && exit 0
[ -e data/rl/chain_c_dunes5_L1.0/DONE ] && exit 0                       # full dunes passed: nothing to bracket
if [ "$lvl" = "0.25" ]; then
  R=data/rl/chain_c_dunes6_L0.275; mkdir -p $R
  PETCTL_DUNES_LEVEL=0.275 python scripts/rl/chain_c_driver.py --stage terrain_dunes --root $R --warm $best > $R/driver.log 2>&1
  if [ -e $R/DONE ]; then A=$(python -c "import json;print([json.loads(l)['attempt'] for l in open('$R/summary.jsonl') if json.loads(l).get('phase')=='gate' and json.loads(l).get('passed')][-1])"); best=$R/terrain_dunes/a$A; fi
fi
R=data/rl/chain_c_sparse_field_after_dunes; mkdir -p $R
python scripts/rl/chain_c_driver.py --stage sparse_field --root $R --warm $best > $R/driver.log 2>&1
