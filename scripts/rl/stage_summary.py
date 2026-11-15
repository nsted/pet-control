"""Summary table of every PASSED chain stage (and the attempts that led to it): success + Wilson CI, current mean / peak, time to goal, duration, steps, ablations, skill and video paths.

    python scripts/rl/stage_summary.py            # prints the markdown table and rewrites docs/plans/STAGE_SUMMARIES.md
"""
from __future__ import annotations

import glob
import json
from pathlib import Path

ROOT = Path("data/rl/chain_ab")
OUT = Path("docs/plans/STAGE_SUMMARIES.md")
ORDER = ["flat_nav", "flat_omni", "terrain_easy", "terrain_full"]


def _lines() -> list[dict]:
    f = ROOT / "summary.jsonl"
    return [json.loads(x) for x in f.read_text().splitlines() if x.strip()] if f.exists() else []


def main() -> None:
    """Build the markdown summary of passed stages."""
    L = _lines()
    rows, notes = [], []
    for st in ORDER:
        gates = [d for d in L if d.get("phase") == "gate" and d.get("stage") == st]
        passed = [d for d in gates if d.get("passed")]
        if not passed:
            continue
        g = passed[-1]
        att = g["attempt"]
        trains = [d for d in L if d.get("phase") == "train" and d.get("stage") == st]
        wall = sum(float(t.get("wall_s", 0.0)) for t in trains) / 60.0
        steps = sum(int(t.get("steps", 0)) for t in trains)
        sps = [float(t.get("steps_per_s", 0.0)) for t in trains if t.get("steps_per_s")]
        lat = g.get("lateral") or {}
        abl = g.get("ablations") or {}
        skill = next((d.get("skill") for d in L if d.get("phase") == "skill" and d.get("stage") == st and d.get("attempt") == att), "")
        import re
        import time as _t
        gt = _t.mktime(_t.strptime(g["time"], "%Y-%m-%d %H:%M:%S"))
        cand = sorted(glob.glob(f"docs/media/final_chain_ab_*_a{att}_*.mp4"))
        def _ts(f):
            m = re.search(r"_(\d{8}_\d{6})\.mp4$", f)
            return _t.mktime(_t.strptime(m.group(1), "%Y%m%d_%H%M%S")) if m else 0.0
        near = [f for f in cand if abs(_ts(f) - gt) <= 1500]            # the final video rendered at the end of the passing attempt (old names do not carry the stage)
        vids = sorted(near, key=lambda f: abs(_ts(f) - gt))
        rows.append({
            "Stage": st, "Attempts": f"{len(gates)} ({len([d for d in gates if not d.get('passed')])} failed first)" if len(gates) > 1 else "1",
            "Success": f"{g['success']:.2f}", "Wilson 95% lo-hi": f"{g['wilson_lo']:.3f}-{g['wilson_hi']:.3f}", "Threshold": f"{g['threshold']:.2f}",
            "t_goal median (s)": f"{g.get('t_goal_median_s', float('nan')):.1f}", "Bus mean (A, median)": f"{g['bus_mean_a_median']:.2f}", "Bus peak (A, median)": f"{g['bus_peak_a_median']:.1f}",
            "Episodes >10 A": f"{g['frac_episodes_over_10a']:.2f}", "Glitch": f"{g['glitch_rate']:.2f}",
            "Train time (min)": f"{wall:.0f}", "Steps (M)": f"{steps / 1e6:.1f}", "Steps/s": f"{(sum(sps) / len(sps)) if sps else 0:.0f}",
            "Lateral (m, median / share>=0.4)": f"{lat['median_m']:.2f} / {lat['frac_ge_min']:.2f}" if lat else "-",
            "Ablations (drop)": ", ".join(f"{k}: {v}" for k, v in abl.items()) if abl else "-"})
        notes.append(f"- **{st}**: skill `{skill}`; gate JSON `{g.get('gate2_json')}`; final video `{vids[0] if vids else '-'}`; fail reasons before passing: " +
                     ("; ".join(r for d in gates if not d.get("passed") for r in d.get("fail_reasons", [])) or "none"))
    if not rows:
        print("no passed stages yet")
        return
    cols = list(rows[0])
    md = "# Passed stages (L=8, held-out n=100, randomisation 0.6)\n\n| " + " | ".join(cols) + " |\n|" + "---|" * len(cols) + "\n"
    md += "\n".join("| " + " | ".join(r[c] for c in cols) + " |" for r in rows) + "\n\n" + "\n".join(notes) + "\n"
    OUT.write_text(md)
    print(md)


if __name__ == "__main__":
    main()
