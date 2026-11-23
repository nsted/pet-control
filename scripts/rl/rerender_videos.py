"""Re-render the final video (landscape 1080p + portrait) of the given run dirs: ``python scripts/rl/rerender_videos.py data/rl/chain_c2/barrier_touch/a1 ...`` (seed 0, run's own training distribution)."""
from __future__ import annotations

import sys
import time
from pathlib import Path

from petctl.rl.video import MEDIA_DIR, record_video


def main() -> None:
    """Render each run given on the command line."""
    for arg in sys.argv[1:]:
        run = Path(arg)
        out = MEDIA_DIR / f"final_{run.parts[-3]}_{run.parts[-2]}_{run.name}_hd_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
        m = record_video(run, out)
        print(out, {k: round(v, 3) for k, v in m.items() if k in ("success", "distance", "final_dist", "contact_frac")}, flush=True)


if __name__ == "__main__":
    main()
