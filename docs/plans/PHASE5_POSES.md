# Phase 5: Rubik's-snake pose tools (n = 11)

## Source and convention check (2026-11-26)
Li, Hou, Bishop, "Computational Design and Analysis of a Magic Snake", J. Mech. Robot. 12(5), 2020 (PMC8208313). Encodes a snake as one number per pin, R in {0,1,2,3} = multiples of 90 deg. Names only one shape (the 24-wedge "ball", sequence 1,3,3,1,3,1,1,3,1,3,3,1,3,1,1,3,1,3,3,1,3,1,1,(3)); says cat / dog etc. are not unique. No collision method (listed as future work): MuJoCo contacts do that here.

Mapping check: the sim was built with 24 modules IN MEMORY ONLY (MAX_MODULES and urdf_gen.MAX_N patched, no file changed) and the ball set as joint targets q_i = R_i x 90 deg.
- Plain mapping (q = R x 90 deg, same sign every joint): head-tail gap 5.3 cm (module spacing 5.9 cm: the ring CLOSES), 6 self-contacts, max penetration 6.7 mm (touching, not intersecting).
- Alternating-sign mapping: also closes, but 71 self-contacts, 31 mm penetration = interpenetrating, physically impossible, rejected.
- Sign and a pi offset cannot be told apart by this (symmetric) sequence. They do not matter for a library: R -> R+2 relabels the same reachable set. Only naming of poses would depend on it.
=> Discrete pose search uses q_i = R_i x 90 deg, R in {0..3}^10 at n = 11 (4^10 ~ 1.05 M sequences), self-collision from MuJoCo contacts.
