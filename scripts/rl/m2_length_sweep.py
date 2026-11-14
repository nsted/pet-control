"""M2 body-length sweep driver (Regimen v2 plan sec 3): for each length L, task in (m2_flat, m2_terrain, m2_hug) and seed, train from scratch, then gate2 (n=100).

One training job at a time (8 envs); gate evaluation runs on a small process pool (training is idle then). Resumable: a (L, task, seed) whose summary line exists is skipped; a
finished training (``meta.json``) is not retrained. Output: ``<root>/<L>/<task>/seed<k>/`` (model, policy.npz, episodes.jsonl, gate2_<task>.json) and one line per run in
``<root>/summary.jsonl``. No videos; ``--sheets`` writes one contact-sheet PNG per (L, task) from the best seed.

    python scripts/rl/m2_length_sweep.py --lengths 8 10 12            # the coarse sweep
    python scripts/rl/m2_length_sweep.py --lengths 8 --steps 20000 --gate-n 6 --root data/rl/m2_smoke   # smoke test
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

from petctl.rl.config import EnvConfig
from petctl.rl.curriculum.early_stop import HealthStopCallback
from petctl.rl.curriculum.gate2 import Gate2Spec, PolicyEvaluator, Row, contact_sheet, run_gate2
from petctl.rl.curriculum.runner import build_config
from petctl.rl.curriculum.stages import M2_TASKS, Stage, get_stage

RECIPE_ENV = {"PETCTL_GAMMA": "0.997", "PETCTL_NSTEPS": "1024", "PETCTL_GAE": "0.97"}   # plan sec 6 (lr 3e-4 linear, 8 epochs, batch 2048 are the train() defaults)


def _eval_chunk(args: tuple[str, str, dict[str, float], float, float | None, dict[str, float], list[int], bool]) -> list[Row]:
    run, policy_file, params, dr, ep_s, overrides, seeds, leak = args
    return PolicyEvaluator(Path(run), policy_file, params, dr, ep_s)(overrides, seeds, leak=leak)


class PoolEvaluator:
    """Gate evaluator that splits the seeds over a process pool (each worker replays the policy on its chunk); results are returned in seed order."""

    def __init__(self, run: Path, policy_file: str, params: dict[str, float], dr: float, ep_s: float | None, workers: int) -> None:
        """``workers``: pool size (1 = in-process)."""
        self.run, self.policy_file, self.params, self.dr, self.ep_s, self.workers = run, policy_file, params, dr, ep_s, workers

    def __call__(self, overrides: dict[str, float], seeds: list[int], leak: bool = False, dr_scale: float | None = None) -> list[Row]:
        """Evaluate ``seeds`` under ``overrides``."""
        dr = self.dr if dr_scale is None else dr_scale
        chunks = [seeds[i::self.workers] for i in range(self.workers) if seeds[i::self.workers]]
        jobs = [(str(self.run), self.policy_file, self.params, dr, self.ep_s, overrides, c, leak) for c in chunks]
        if len(jobs) == 1:
            return _eval_chunk(jobs[0])
        with ProcessPoolExecutor(len(jobs)) as ex:
            rows = [r for part in ex.map(_eval_chunk, jobs) for r in part]
        return sorted(rows, key=lambda r: r["seed"])


def run_dir(root: Path, length: int, task: str, seed: int) -> Path:
    """``<root>/<L>/<task>/seed<k>``."""
    return root / str(length) / task / f"seed{seed}"


def done_keys(root: Path) -> set[tuple[int, str, int]]:
    """(L, task, seed) of every line already in the summary."""
    f = root / "summary.jsonl"
    keys = set()
    if f.exists():
        for line in f.read_text().splitlines():
            if line.strip():
                d = json.loads(line)
                keys.add((int(d["L"]), d["task"], int(d["seed"])))
    return keys


def _nz(v: Any) -> float | None:
    return None if v is None or (isinstance(v, float) and not math.isfinite(v)) else float(v)


def summarise(length: int, task: str, seed: int, res: dict[str, Any] | None, meta: dict[str, Any], status: str, note: str = "") -> dict[str, Any]:
    """One summary line."""
    out: dict[str, Any] = {"L": length, "task": task, "seed": seed, "status": status, "note": note, "steps": meta.get("steps"), "wall_s": meta.get("wall_s"),
                           "steps_per_s": meta.get("steps_per_s"), "time": time.strftime("%Y-%m-%d %H:%M:%S")}
    if res is not None:
        rb, mm = res["robust"], res["robust"]["metric_median"]
        cur = res["current"]
        out.update({"n": rb["n"], "success": rb["rate"], "wilson_lo": rb["wilson_lo"], "wilson_hi": rb["wilson_hi"], "t_goal_median_s": _nz(mm.get("time_to_goal_s")),
                    "t_hug_median_s": _nz(mm.get("t_hug_s")), "hug_modules_median": _nz(mm.get("hug_modules")), "energy_per_m_median": _nz(mm.get("energy_per_m")),
                    "bus_mean_a": cur.get("bus_mean_a_median"), "bus_peak_a_median": cur.get("bus_peak_a_median"), "frac_over_10a": cur.get("frac_episodes_over_10a"),
                    "glitch_rate": res["glitch_rate"], "gate2_json": f"gate2_{res['stage']}.json"})
    return out


def train_one(stage: Stage, length: int, seed: int, out: Path, steps: int, n_envs: int) -> dict[str, Any]:
    """Train from scratch (asymmetric critic, plan recipe); returns ``{steps, wall_s, steps_per_s, early_stop}``. Skips training if ``out/meta.json`` exists."""
    from petctl.rl.train import train

    meta_f = out / "meta.json"
    if meta_f.exists() and (out / "policy.npz").exists():
        m = json.loads(meta_f.read_text())
        return {"steps": m["steps"], "wall_s": m["wall_s"], "steps_per_s": m["steps"] / max(m["wall_s"], 1e-9), "early_stop": (out / "early_stop.txt").read_text() if (out / "early_stop.txt").exists() else None}
    os.environ.update(RECIPE_ENV)
    cfg: EnvConfig = build_config(stage, {"n_modules": float(length)})
    health = HealthStopCallback()
    t0 = time.time()
    train(cfg, steps, n_envs, out, seed, "ppo", None, stage.ent_coef, None, video=False, asymmetric=True, extra_callbacks=[health])
    wall = time.time() - t0
    stopped = health.reason
    if stopped:
        (out / "early_stop.txt").write_text(stopped)
    m = json.loads(meta_f.read_text())
    steps_done = int(m.get("steps", steps))
    return {"steps": steps_done, "wall_s": wall, "steps_per_s": steps_done / max(wall, 1e-9), "early_stop": stopped}


def gate_one(stage: Stage, out: Path, n: int, workers: int) -> dict[str, Any]:
    """gate2 with n episodes at the stage's gate distribution (robust dr 0.6, no nominal pass, no ablations)."""
    spec = Gate2Spec(n=n, dr_scale=0.6, nominal=False, threshold=0.0)
    params = dict(stage.gate.task_params)
    params.setdefault("bus_model", 1.0)
    ev = PoolEvaluator(out, "policy.npz", params, spec.dr_scale, stage.gate.episode_s, workers)
    return run_gate2(out, stage, spec, evaluator=ev)


def sheets(root: Path, lengths: list[int], seeds: list[int]) -> None:
    """One contact-sheet PNG per (L, task): the best-success seed's best gate episode (no video)."""
    rows = [json.loads(line) for line in (root / "summary.jsonl").read_text().splitlines() if line.strip()]
    for L in lengths:
        for task in M2_TASKS:
            cand = [r for r in rows if r["L"] == L and r["task"] == task and r.get("success") is not None]
            if not cand:
                continue
            best = max(cand, key=lambda r: r["success"])
            run = run_dir(root, L, task, best["seed"])
            try:
                png = contact_sheet(run, task, out=root / f"sheet_L{L}_{task}.png")
                print(f"[sheet] {png}", flush=True)
            except Exception as e:      # no successful episode, or no offscreen GL
                print(f"[sheet] L={L} {task}: skipped ({type(e).__name__}: {e})", flush=True)


def main() -> int:
    """CLI entry."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lengths", type=int, nargs="+", default=[8, 10, 12])
    ap.add_argument("--tasks", nargs="+", default=list(M2_TASKS))
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1])
    ap.add_argument("--steps", type=int, default=2_000_000)
    ap.add_argument("--gate-n", type=int, default=100)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--eval-workers", type=int, default=4)
    ap.add_argument("--root", type=Path, default=Path("data/rl/m2_sweep"))
    ap.add_argument("--sheets", action="store_true", help="only write the contact sheets from existing results")
    a = ap.parse_args()
    root: Path = a.root
    root.mkdir(parents=True, exist_ok=True)
    if a.sheets:
        sheets(root, a.lengths, a.seeds)
        return 0
    for L in a.lengths:
        for task in a.tasks:
            for seed in a.seeds:
                if (L, task, seed) in done_keys(root):
                    print(f"[skip] L={L} {task} seed{seed}: already in summary", flush=True)
                    continue
                stage, out = get_stage(task), run_dir(root, L, task, seed)
                print(f"[run] L={L} {task} seed{seed} -> {out}  ({time.strftime('%H:%M:%S')})", flush=True)
                out.mkdir(parents=True, exist_ok=True)
                meta: dict[str, Any] = {}
                try:
                    meta = train_one(stage, L, seed, out, a.steps, a.n_envs)
                    status = "early_stopped" if meta.get("early_stop") else "ok"
                    res = gate_one(stage, out, a.gate_n, a.eval_workers)
                    line = summarise(L, task, seed, res, meta, status, meta.get("early_stop") or "")
                except Exception as e:
                    traceback.print_exc()
                    line = summarise(L, task, seed, None, meta, "failed", f"{type(e).__name__}: {e}")
                with open(root / "summary.jsonl", "a") as f:
                    f.write(json.dumps(line) + "\n")
                print(f"[done] {json.dumps(line)}", flush=True)
    sheets(root, a.lengths, a.seeds)
    print("[sweep complete]", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
