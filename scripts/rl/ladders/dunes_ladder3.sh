#!/bin/bash
# Waits for ladder 2; if it stopped at a failing level, re-runs from that level with the original sand friction and a training-only goal bearing of +-135 deg (gate stays +-180).
cd /Users/nsted/Documents/PET/code/petctl
export PETCTL_PROGRESS_EXPORT_S=900
while pgrep -f dunes_ladder2.sh > /dev/null; do sleep 60; done
LEVELS="0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0"
prev=data/rl/chain_c_dunes_L0.2/terrain_dunes/a1
start=""
for L in $LEVELS; do
  R=data/rl/chain_c_dunes2_L${L}
  if [ -e $R/DONE ]; then
    A=$(python -c "import json;print([json.loads(l)['attempt'] for l in open('$R/summary.jsonl') if json.loads(l).get('phase')=='gate' and json.loads(l).get('passed')][-1])")
    prev=$R/terrain_dunes/a$A
  else
    start=$L; break
  fi
done
[ -z "$start" ] && exit 0
[ "$start" = "0.3" ] && prev=data/rl/chain_c_dunes_L0.3/terrain_dunes/a1
go=0
for L in $LEVELS; do
  [ "$L" = "$start" ] && go=1
  [ $go = 1 ] || continue
  R=data/rl/chain_c_dunes3_L${L}; mkdir -p $R
  PETCTL_DUNE_SOFT_FRICTION=0 PETCTL_DUNES_TRAIN_BEARING=135 PETCTL_DUNES_LEVEL=$L python scripts/rl/chain_c_driver.py --stage terrain_dunes --root $R --warm $prev > $R/driver.log 2>&1
  [ -e $R/DONE ] || exit 0
  A=$(python -c "import json;print([json.loads(l)['attempt'] for l in open('$R/summary.jsonl') if json.loads(l).get('phase')=='gate' and json.loads(l).get('passed')][-1])")
  prev=$R/terrain_dunes/a$A
done
