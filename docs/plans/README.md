# petctl plans

Design and implementation plans for major feature areas. These are living documents — update them as decisions are made or scope changes.

When ready to execute a plan, break it into GitHub issues for discrete tasks.

| File | Description |
|------|-------------|
| [BEHAVIOR_SYSTEM.md](BEHAVIOR_SYSTEM.md) | Staged implementation plan for behaviors, perception, valence, and agent |
| [JOINT_FOUNDATIONS.md](JOINT_FOUNDATIONS.md) | Joint-level foundations (limits, recorder, mock dynamics, motor profile). Formerly SHAPE_LAYER. **Re-scoped 2026-11-03:** behavior engine/primitives/migration archived in favour of RL; 1.4a (system ID) kept |
| [PROCESS_SPLIT.md](PROCESS_SPLIT.md) | Split petctl into driver (safety + backend) and brain (Controller) processes over a local bus |

## Convention
- Plans describe *how* and *why*, not just *what*
- Note decisions and deviations inline as they happen
- New plans go here; link them in the table above

## Task states

| Marker | Meaning |
|--------|---------|
| `[ ]` | Implied / upcoming — tracked here, not yet an issue |
| `[ ]` + GitHub issue linked | Actively in flight |
| `[x]` | Done |

Unchecked checkboxes in a plan are the backlog. When work is about to start on an item, create a GitHub issue and optionally link it inline. When done, check it off and close the issue.
