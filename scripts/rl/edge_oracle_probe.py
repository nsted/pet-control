import sys; sys.path.insert(0,'scripts/rl')
from edge_reflex import evaluate
cfgs={'oracle overhang, no snap':dict(),
 'oracle overhang, snap':dict(snap=True),
 'oracle overhang, snap+halt 1s':dict(snap=True,halt_s=1.0),
 'oracle dist<0.20':dict(oracle_dist=0.20,snap=True),
 'oracle dist<0.30 snap':dict(oracle_dist=0.30,snap=True),
 'oracle dist<0.30 snap+halt':dict(oracle_dist=0.30,snap=True,halt_s=1.0)}
for n,kw in cfgs.items():
    print(n,end=' -> '); evaluate('oracle',36,kw,None)
