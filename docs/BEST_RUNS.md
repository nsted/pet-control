# Best run per approach (local maxima)

One row per training approach: the best policy it produced, replayed deterministically on nominal physics. Height = gain over the
flat start pose; "held" = lowest valid height over a trailing 2 s window. Videos are named `<run>_best_ep<N>_<timestamp>.mp4` and
are written automatically at the end of every `petctl rl train` run.

| Approach | Best in training | Replay | Video |
|---|---|---|---|
| reach_still, strict stillness test, 4 h | 9.3 cm (one episode, under noise) | 4.7 cm | `reach_still_4h_best_ep4491_20261105_101656.mp4` |
| reach_still, height-change reward, 15 min | 7.8 cm held | 6.8 cm | `reach_hold_15m_best_ep7892_20261105_064745.mp4` |
| reach_still, + entropy bonus + peak bonus, 15 min | 8.0 cm held (instant peaks 13-15 cm, not held) | 6.9 cm | `reach_hold_15m_explore_best_ep8613_20261105_073149.mp4` |
| static pose search (differential evolution, 4 restarts, no RL) | 17.0 cm held | 17.0 cm | `static_pose_search_best_17.0cm_20261105_101200.mp4` (+ `rank2`..`rank4`) |

All at a 0.5 N.m torque cap, far above the real 0.164 N.m.
