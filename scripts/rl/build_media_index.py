"""Build docs/media/index_attempt1.html: every landscape video of the earlier work (attempt 1: RL policies on the old 8-module body, repertoire montages, hug runs, baselines), grouped by run
family, newest first, lazy-loaded. The hand-written review page docs/media/index.html (this session's searches) links to it. docs/media is git-ignored.

    python scripts/rl/build_media_index.py
"""

from __future__ import annotations

import html
import re
from collections import defaultdict
from pathlib import Path

MEDIA = Path("docs/media")
NEW_PREFIXES = ("gaits_n", "gap_bridge", "wrap_n", "statics_n")                 # this session's videos live on the main page
TS = re.compile(r"_(\d{8})_(\d{6})")


def family(name: str) -> str:
    """Group key: the run family (file name without timestamp / attempt suffix)."""
    stem = TS.sub("", name[:-4])
    stem = re.sub(r"_a\d+(_hd)?$", "", stem)
    for key, label in (("final_chain_ab", "Navigation chain A/B (flat, terrain), final policies"), ("final_chain_c2", "Chain C2 (barrier touch), final policies"),
                       ("final_chain_c_barrier", "Chain C (barrier touch), final policies"), ("final_chain_c_dunes", "Dune ladder, final policies"),
                       ("final_chain_c_sparse", "Sparse field, final policies"), ("final_chain_c_terrain_ridges", "Ridges, final policies"),
                       ("final_chain_c_terrain_rings", "Rings, final policies"), ("final_chain_c_terrain_rolling", "Rolling terrain, final policies"),
                       ("final_chain_c_rings", "Rings, final policies"), ("final_hug", "Hug runs, final policies"), ("hoop", "Hoop-roll gait search"),
                       ("gaits_", "Gait repertoire montages"), ("poses_", "Pose repertoire montages"), ("m2_", "Length study M2 (old body)"), ("iter_", "Earlier iteration runs"),
                       ("baseline", "Baselines / previews"), ("preview", "Baselines / previews"), ("gr_smoke", "Baselines / previews")):
        if stem.startswith(key):
            return label
    return "Other"


def main() -> int:
    """Write the page."""
    groups: dict[str, list[Path]] = defaultdict(list)
    for p in sorted(MEDIA.glob("*.mp4")):
        if "portrait" in p.name or p.name.startswith(("progress_",)) or p.name.startswith(NEW_PREFIXES):
            continue
        groups[family(p.name)].append(p)
    out = ['<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>PET attempt 1 videos</title>',
           "<style>:root{--bg:#fafaf8;--fg:#1d1d1b;--muted:#6b6b66;--card:#fff;--line:#e2e2dc}@media(prefers-color-scheme:dark){:root{--bg:#161615;--fg:#ececE8;--muted:#a2a29b;--card:#20201e;--line:#34342f}}",
           "body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.5 -apple-system,'Segoe UI',sans-serif}main{max-width:1100px;margin:0 auto;padding:24px 16px 64px}",
           "details{border:1px solid var(--line);border-radius:8px;margin:10px 0;background:var(--card)}summary{cursor:pointer;padding:10px 12px;font-weight:600}",
           ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(300px,1fr));gap:12px;padding:12px}video{width:100%;border-radius:4px;background:#000}",
           ".c p{margin:2px 0;color:var(--muted);font-size:12.5px;word-break:break-all}</style></head><body><main>",
           "<h1>Attempt 1: earlier training (old 8-module body)</h1>",
           "<p style='color:var(--muted)'>RL policies and repertoire searches from before the autonomous-trainer restart (reference and seed material only, not a baseline to beat). "
           "Landscape videos only, newest first. <a href='index.html'>Back to the main review page</a>. Regenerate: <code>python scripts/rl/build_media_index.py</code>.</p>"]
    for label in sorted(groups, key=lambda k: (k == "Other", k)):
        files = sorted(groups[label], key=lambda p: TS.search(p.name).group(0) if TS.search(p.name) else p.name, reverse=True)
        out.append(f"<details><summary>{html.escape(label)} ({len(files)})</summary><div class='grid'>")
        for p in files:
            m = TS.search(p.name)
            when = f"{m.group(1)[:4]}-{m.group(1)[4:6]}-{m.group(1)[6:]} {m.group(2)[:2]}:{m.group(2)[2:4]}" if m else ""
            out.append(f"<div class='c'><video controls preload='none' src='{html.escape(p.name)}'></video><p>{html.escape(p.name)}</p><p>{when}</p></div>")
        out.append("</div></details>")
    out.append("</main></body></html>")
    (MEDIA / "index_attempt1.html").write_text("\n".join(out))
    print(f"wrote index_attempt1.html: {sum(len(v) for v in groups.values())} videos in {len(groups)} groups")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
