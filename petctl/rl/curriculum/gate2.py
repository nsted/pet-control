"""Regimen v2 gate tooling (plan sec 4.1): n=100 held-out seeds, Wilson CI pass rule, paired ablations, blind baseline, current budget, second ground-truth metrics,
post-leak test, and the best-episode seed for the single allowed video. Does not touch the legacy ``gate.py`` (existing stages keep their behaviour).

Statistics. Success rate: Wilson 95% score interval; the gate passes iff the CI LOWER bound >= the threshold. Ablation drop = success(base) - success(ablated) on
identical seeds (same layouts); its CI is a PAIRED PERCENTILE BOOTSTRAP over seeds (``N_BOOT`` resamples, fixed rng), and a sensing claim passes iff the LOWER bound
of the drop >= the stated minimum. The exact McNemar test (discordant counts ``base_only`` / ``abl_only``) is reported alongside. The blind baseline is judged on its
point estimate against the ceiling (Wilson interval reported); exceeding the ceiling makes the gate VOID, not failed.

Evaluation goes through an *evaluator*: ``evaluator(overrides, seeds, leak=False, dr_scale=None) -> list[row]`` with ``row = {seed, ret, steps, metrics, gt}``. ``PolicyEvaluator``
replays an exported ``policy.npz``; tests may inject any callable.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np

from petctl.config import REGIMEN_BUS_MEAN_A
from petctl.rl.curriculum.stages import Stage
from petctl.rl.env import PetEnv
from petctl.rl.policy import GaitPolicy

N_BOOT = 2000
Z95 = 1.959963984540054
BLIND_OVERRIDES = {"touch_off": 1.0, "capacitive_off": 1.0, "imu_off": 1.0}
LEAK_MOVE_M = 1.0

Row = dict[str, Any]
Evaluator = Callable[..., list[Row]]


# ---------------------------------------------------------------- statistics
def wilson_ci(k: int, n: int, z: float = Z95) -> tuple[float, float]:
    """Wilson score interval for ``k`` successes in ``n`` trials (``(0, 1)`` for n = 0)."""
    if n <= 0:
        return 0.0, 1.0
    p = k / n
    den = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0.0, centre - half), min(1.0, centre + half)


def mcnemar_exact_p(b: int, c: int) -> float:
    """Two-sided exact McNemar p-value for discordant counts ``b`` (base only) and ``c`` (ablated only)."""
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    tail = sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n
    return float(min(1.0, 2 * tail))


def paired_drop(base: np.ndarray, abl: np.ndarray, n_boot: int = N_BOOT, seed: int = 0, alpha: float = 0.05) -> dict[str, float]:
    """Paired success drop ``mean(base) - mean(abl)`` over the same seeds: percentile-bootstrap CI (resampling seed pairs) plus the exact McNemar test."""
    base, abl = np.asarray(base, float), np.asarray(abl, float)
    if base.shape != abl.shape or base.size == 0:
        raise ValueError("paired_drop needs equal-length, non-empty success vectors")
    d = base - abl
    rng = np.random.default_rng(seed)
    boots = d[rng.integers(0, d.size, (n_boot, d.size))].mean(axis=1)
    lo, hi = np.quantile(boots, [alpha / 2, 1 - alpha / 2])
    b, c = int(np.sum((base > 0.5) & (abl <= 0.5))), int(np.sum((base <= 0.5) & (abl > 0.5)))
    return {"drop": float(d.mean()), "ci_lo": float(lo), "ci_hi": float(hi), "method": "paired percentile bootstrap", "n_boot": n_boot,
            "base_only": b, "abl_only": c, "mcnemar_p": mcnemar_exact_p(b, c)}


def success_summary(succ: np.ndarray) -> dict[str, float]:
    """``k``, ``n``, rate and the Wilson 95% interval of a 0/1 vector."""
    k, n = int(np.sum(np.asarray(succ) > 0.5)), int(len(succ))
    lo, hi = wilson_ci(k, n)
    return {"k": k, "n": n, "rate": k / n if n else float("nan"), "wilson_lo": lo, "wilson_hi": hi}


# ---------------------------------------------------------------- spec
@dataclass(frozen=True)
class Ablation:
    """A paired condition: task-param overrides on the same seeds. ``min_drop``: the lower CI bound of the success drop must reach it; ``min_success``: the Wilson lower
    bound of the ablated success must reach it (e.g. a delta-bias robustness requirement). Both None: reported only."""

    name: str
    overrides: dict[str, float]
    min_drop: float | None = None
    min_success: float | None = None


@dataclass
class Gate2Spec:
    """Everything a v2 gate needs beyond the stage's frozen threshold. ``threshold``/``task_params``/``episode_s`` default to the stage's ``Gate``."""

    n: int = 100
    seed0: int = 10_000
    dr_scale: float = 0.6
    nominal: bool = True                                  # also report a dr 0 condition (not part of the pass rule)
    threshold: float | None = None
    task_params: dict[str, float] | None = None
    episode_s: float | None = None
    success_key: str = "success"                          # row metrics / gt key that defines success (the gate metric may differ from the reward)
    second_metrics: dict[str, Callable[[Any, dict[str, Any]], float]] = field(default_factory=dict)   # name -> fn(env, final info); stored in row["gt"]
    ablations: tuple[Ablation, ...] = ()
    blind_overrides: dict[str, float] | None = None       # e.g. BLIND_OVERRIDES; None: no blind baseline
    blind_ceiling: float | None = None
    blind_run: Path | None = None                         # evaluate another policy (e.g. the stage-A straight walker) instead; see ``blind_env``
    blind_env: dict[str, Any] = field(default_factory=dict)   # config patch for ``blind_run``: {"task": ..., "task_params": {...}}
    leak_test: bool = False                               # post moved ``leak_move_m`` after first contact (hug gates)
    leak_move_m: float = LEAK_MOVE_M
    leak_max_success: float | None = None                 # if set: success under the leak must be <= this
    bus_model: bool = True                                # evaluate with the per-robot current model (adds bus metrics)
    max_bus_mean_a: float = REGIMEN_BUS_MEAN_A
    max_frac_ticks_over: float = 0.05
    max_glitch: float = 0.0
    min_median: dict[str, float] = field(default_factory=dict)
    group_of: Callable[[int], str] | None = None          # seed -> group name (e.g. held-out shape); per-group success is reported
    min_group_success: float | None = None                # every group's success rate (point estimate) must reach this

    def jsonable(self) -> dict[str, Any]:
        """Serialisable summary of the spec."""
        return {"n": self.n, "seed0": self.seed0, "dr_scale": self.dr_scale, "threshold": self.threshold, "success_key": self.success_key,
                "ablations": [{"name": a.name, "overrides": a.overrides, "min_drop": a.min_drop, "min_success": a.min_success} for a in self.ablations],
                "blind_overrides": self.blind_overrides, "blind_ceiling": self.blind_ceiling, "blind_run": None if self.blind_run is None else str(self.blind_run),
                "leak_test": self.leak_test, "leak_move_m": self.leak_move_m, "leak_max_success": self.leak_max_success, "bus_model": self.bus_model,
                "max_bus_mean_a": self.max_bus_mean_a, "max_frac_ticks_over": self.max_frac_ticks_over, "max_glitch": self.max_glitch, "second_metrics": sorted(self.second_metrics),
                "min_median": self.min_median, "min_group_success": self.min_group_success, "grouped": self.group_of is not None}


# ---------------------------------------------------------------- evaluator
def move_post(env: PetEnv, distance: float) -> None:
    """Shift the (static) post ``distance`` metres away from the body's centre of mass in the floor plane (the leak test)."""
    import mujoco

    from petctl.sim.model_builder import set_post_xy

    sim = env.sim
    assert sim is not None and sim.p.post is not None
    post = np.array(sim.p.post, float)
    away = post - sim.data.subtree_com[1][:2]
    away = away / max(float(np.linalg.norm(away)), 1e-9)
    new = post + distance * away
    set_post_xy(sim.model, new)
    sim.p.post = (float(new[0]), float(new[1]))
    mujoco.mj_forward(sim.model, sim.data)


class PolicyEvaluator:
    """Replays ``run/policy_file`` deterministically on given seeds with the gate's task params plus per-call overrides."""

    def __init__(self, run: Path, policy_file: str, task_params: dict[str, float], dr_scale: float, episode_s: float | None = None,
                 second_metrics: dict[str, Callable[[Any, dict[str, Any]], float]] | None = None, cfg_patch: dict[str, Any] | None = None, leak_move_m: float = LEAK_MOVE_M) -> None:
        """``cfg_patch``: optional ``task`` / ``task_params`` replacement of the run's env config (for a different policy on the gate layout)."""
        self.run, self.policy_file, self.task_params, self.dr_scale, self.episode_s = run, policy_file, dict(task_params), dr_scale, episode_s
        self.second_metrics, self.cfg_patch, self.leak_move_m = second_metrics or {}, cfg_patch or {}, leak_move_m

    def __call__(self, overrides: dict[str, float], seeds: list[int], leak: bool = False, dr_scale: float | None = None) -> list[Row]:
        """One episode per seed; ``leak`` moves the post after first contact."""
        pol = GaitPolicy(self.run, self.policy_file)
        cfg = pol.cfg
        if "task" in self.cfg_patch:
            cfg.task = self.cfg_patch["task"]
        cfg.task_params = {**self.task_params, **self.cfg_patch.get("task_params", {}), **overrides}
        cfg.dr.scale = self.dr_scale if dr_scale is None else dr_scale
        if self.episode_s is not None:
            cfg.episode_s = self.episode_s
        env = PetEnv(cfg)
        if env.observation_space.shape[0] != pol._mean.shape[0]:
            raise ValueError(f"policy expects {pol._mean.shape[0]} observation entries but task '{cfg.task}' provides {env.observation_space.shape[0]}")
        rows: list[Row] = []
        for s in seeds:
            obs, _ = env.reset(seed=s)
            done, info, ret, steps, moved_at = False, {}, 0.0, 0, -1
            while not done:
                obs, r, te, tr, info = env.step(pol.action(obs))
                ret, steps, done = ret + float(r), steps + 1, te or tr
                if leak and moved_at < 0 and not math.isnan(getattr(env.task, "_hug").t_contact):
                    move_post(env, self.leak_move_m)
                    moved_at = steps
            rows.append({"seed": int(s), "ret": ret, "steps": steps, "metrics": {k: float(v) for k, v in info.get("metrics", {}).items()},
                         "gt": {k: float(fn(env, info)) for k, fn in self.second_metrics.items()}, "leak_moved_at_step": moved_at})
        return rows


# ---------------------------------------------------------------- aggregation
def _val(row: Row, key: str) -> float:
    if key in row.get("gt", {}):
        return float(row["gt"][key])
    return float(row.get("metrics", {}).get(key, 0.0))


def success_vector(rows: list[Row], key: str) -> np.ndarray:
    """0/1 success per row, from ``gt[key]`` or ``metrics[key]``."""
    return np.array([1.0 if _val(r, key) > 0.5 else 0.0 for r in rows])


def _medians(rows: list[Row]) -> dict[str, float]:
    keys = sorted({k for r in rows for k in r["metrics"]} | {f"gt_{k}" for r in rows for k in r["gt"]})
    out = {}
    for k in keys:
        vals = [r["gt"][k[3:]] if k.startswith("gt_") and k[3:] in r["gt"] else r["metrics"].get(k, float("nan")) for r in rows]
        vals = [v for v in vals if math.isfinite(v)]
        out[k] = float(np.median(vals)) if vals else float("nan")
    return out


def _condition(rows: list[Row], key: str) -> dict[str, Any]:
    s = success_vector(rows, key)
    ret = [r["ret"] for r in rows]
    gts = {k: float(np.mean([r["gt"][k] for r in rows])) for k in (rows[0]["gt"] if rows else {})}
    return {**success_summary(s), "success_rate": float(s.mean()) if len(s) else float("nan"), "return_median": float(np.median(ret)) if ret else float("nan"),
            "glitch_rate": float(np.mean([_val(r, "glitch") for r in rows])) if rows else float("nan"), "metric_median": _medians(rows), "second_metric_mean": gts}


def current_budget(rows: list[Row], spec: Gate2Spec) -> dict[str, Any]:
    """Hard current-budget checks: median episode mean bus current, and the fraction of episodes with any tick over the peak limit."""
    if not rows or any("bus_mean_a" not in r["metrics"] for r in rows):
        return {"measured": False, "ok": False}
    mean_a = np.array([r["metrics"]["bus_mean_a"] for r in rows])
    over = np.array([r["metrics"].get("frac_ticks_over_10a", 0.0) for r in rows])
    frac_eps = float(np.mean(over > 0.0))
    return {"measured": True, "bus_mean_a_median": float(np.median(mean_a)), "bus_mean_a_max": float(mean_a.max()),
            "bus_peak_a_median": float(np.median([r["metrics"].get("bus_peak_a", 0.0) for r in rows])),
            "frac_episodes_over_10a": frac_eps, "frac_ticks_over_10a_mean": float(over.mean()),
            "ok": bool(np.median(mean_a) <= spec.max_bus_mean_a and frac_eps <= spec.max_frac_ticks_over)}


# ---------------------------------------------------------------- gate
def run_gate2(run: Path, stage: Stage, spec: Gate2Spec | None = None, policy_file: str = "policy.npz", evaluator: Evaluator | None = None,
              write: bool = True) -> dict[str, Any]:
    """Evaluate ``run/policy_file`` against ``stage`` with the v2 rules; writes ``run/gate2_<stage>.json`` (unless ``write`` is False) and returns the result.

    ``status`` is PASS, FAIL or VOID (blind baseline above its ceiling: the gate layout does not need the sensing under test).
    """
    spec = spec or Gate2Spec()
    threshold = stage.gate.min_success if spec.threshold is None else spec.threshold
    params = dict(stage.gate.task_params if spec.task_params is None else spec.task_params)
    if spec.bus_model:
        params.setdefault("bus_model", 1.0)
    ep_s = stage.gate.episode_s if spec.episode_s is None else spec.episode_s
    seeds = [spec.seed0 + i for i in range(spec.n)]
    ev: Evaluator = evaluator or PolicyEvaluator(run, policy_file, params, spec.dr_scale, ep_s, spec.second_metrics, leak_move_m=spec.leak_move_m)
    key = spec.success_key
    reasons: list[str] = []

    base_rows = ev({}, seeds)
    robust = _condition(base_rows, key)
    base_s = success_vector(base_rows, key)
    if robust["wilson_lo"] < threshold:
        reasons.append(f"success {robust['rate']:.3f} (Wilson lo {robust['wilson_lo']:.3f}) < threshold {threshold:.2f}")
    mm = robust["metric_median"]
    for k, v in {**stage.gate.min_median, **spec.min_median}.items():
        if not mm.get(k, float("nan")) >= v:
            reasons.append(f"median {k} {mm.get(k, float('nan')):.3f} < {v:.3f}")
    res: dict[str, Any] = {"stage": stage.name, "run": str(run), "policy_file": policy_file, "spec": spec.jsonable(), "ci_method": "Wilson 95% (success), paired bootstrap (drops)",
                           "n": spec.n, "seeds": [seeds[0], seeds[-1]], "threshold": threshold, "dr_scale": spec.dr_scale, "eval_task_params": params, "episode_s": ep_s}
    if spec.nominal:
        res["nominal"] = _condition(ev({}, seeds, dr_scale=0.0), key)
    res["robust"] = robust

    if spec.group_of is not None:
        groups: dict[str, list[float]] = {}
        for r, sv in zip(base_rows, base_s):
            groups.setdefault(spec.group_of(int(r["seed"])), []).append(float(sv))
        res["groups"] = {g: success_summary(np.array(v)) for g, v in groups.items()}
        if spec.min_group_success is not None:
            for g, sm in res["groups"].items():
                if sm["rate"] < spec.min_group_success:
                    reasons.append(f"group {g}: success {sm['rate']:.2f} < required {spec.min_group_success:.2f}")
    cur = current_budget(base_rows, spec)
    res["current"] = cur
    if not cur["ok"]:
        reasons.append("current budget not measured" if not cur["measured"] else
                       f"current budget: median bus_mean_a {cur['bus_mean_a_median']:.2f} (max {spec.max_bus_mean_a}), episodes over peak {cur['frac_episodes_over_10a']:.2f} (max {spec.max_frac_ticks_over})")
    if robust["glitch_rate"] > spec.max_glitch:
        reasons.append(f"glitch rate {robust['glitch_rate']:.3f} > {spec.max_glitch}")

    abl: dict[str, Any] = {}
    for a in spec.ablations:
        rows = ev(dict(a.overrides), seeds)
        s = success_vector(rows, key)
        d = paired_drop(base_s, s)
        sm = success_summary(s)
        ok = True
        if a.min_drop is not None and d["ci_lo"] < a.min_drop:
            ok = False
            reasons.append(f"ablation {a.name}: drop {d['drop']:.3f} (CI lo {d['ci_lo']:.3f}) < required {a.min_drop:.2f}")
        if a.min_success is not None and sm["wilson_lo"] < a.min_success:
            ok = False
            reasons.append(f"ablation {a.name}: success Wilson lo {sm['wilson_lo']:.3f} < required {a.min_success:.2f}")
        abl[a.name] = {"overrides": a.overrides, "success": sm, "drop": d, "min_drop": a.min_drop, "min_success": a.min_success, "passed": ok,
                       "bus_mean_a_median": current_budget(rows, spec).get("bus_mean_a_median")}
    res["ablations"] = abl

    void = False
    if spec.blind_overrides is not None or spec.blind_run is not None:
        if spec.blind_run is not None:
            bev: Evaluator = PolicyEvaluator(spec.blind_run, "policy.npz", params, spec.dr_scale, ep_s, spec.second_metrics, cfg_patch=spec.blind_env)
            brows = bev({}, seeds)
        else:
            brows = ev(dict(spec.blind_overrides or {}), seeds)
        bs = success_vector(brows, key)
        blind = {**success_summary(bs), "overrides": spec.blind_overrides, "run": None if spec.blind_run is None else str(spec.blind_run), "ceiling": spec.blind_ceiling,
                 "drop": paired_drop(base_s, bs)}
        if spec.blind_ceiling is not None:
            blind["within_ceiling"] = bool(blind["rate"] <= spec.blind_ceiling)
            if not blind["within_ceiling"]:
                void = True
                reasons.append(f"VOID: blind baseline success {blind['rate']:.3f} > ceiling {spec.blind_ceiling:.2f}")
        res["blind"] = blind

    if spec.leak_test:
        lrows = ev({}, seeds, leak=True)
        ls = success_vector(lrows, key)
        moved = [r["leak_moved_at_step"] >= 0 for r in lrows]
        leak = {**success_summary(ls), "moved_fraction": float(np.mean(moved)), "post_moved_m": spec.leak_move_m, "drop": paired_drop(base_s, ls), "max_success": spec.leak_max_success,
                "metric_median": _medians(lrows)}
        if spec.leak_max_success is not None and leak["rate"] > spec.leak_max_success:
            reasons.append(f"leak test: success {leak['rate']:.3f} > {spec.leak_max_success:.2f} (policy kept hugging a post that moved away)")
        res["leak"] = leak

    wins = [r for r, s in zip(base_rows, base_s) if s > 0.5]
    best = max(wins, key=lambda r: r["ret"]) if wins else None
    res["best_episode"] = None if best is None else {"seed": best["seed"], "return": best["ret"], "dr_scale": spec.dr_scale}
    res["per_seed"] = [{"seed": r["seed"], "success": float(s), "return": r["ret"], "steps": r["steps"], "t_s": r["metrics"].get("time_to_goal_s")} for r, s in zip(base_rows, base_s)]
    res["glitch_rate"] = robust["glitch_rate"]
    res["fail_reasons"] = reasons
    res["void"] = void
    res["passed"] = not reasons
    res["status"] = "VOID" if void else ("PASS" if not reasons else "FAIL")
    if write:
        (run / f"gate2_{stage.name}.json").write_text(json.dumps(res, indent=1, default=lambda o: None if o is None else str(o)))
    return res


# ---------------------------------------------------------------- video / contact sheet helpers
def _gate_episode(run: Path, stage_name: str, seed: int | None) -> tuple[dict[str, Any], int]:
    res = json.loads((run / f"gate2_{stage_name}.json").read_text())
    if seed is None:
        if not res.get("best_episode"):
            raise ValueError(f"gate2_{stage_name}.json has no successful episode to show")
        seed = int(res["best_episode"]["seed"])
    return res, int(seed)


def render_best(run: Path, stage: str, out: Path | None = None, policy_file: str = "policy.npz") -> Path:
    """Render ONLY the best successful gate episode of ``run`` (highest return among successes, from ``gate2_<stage>.json``) with ``record_video`` (captioned
    with the stage/task). Returns the mp4 path."""
    import time

    from petctl.rl import video
    from petctl.rl.curriculum.stages import get_stage

    res, seed = _gate_episode(run, stage, None)
    st = get_stage(stage)
    out = out or video.MEDIA_DIR / f"{stage}_{run.name}_gate2_best_seed{seed}_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
    label = f"{st.name} ({run.name}) gate best episode seed {seed}, dr {res['dr_scale']}: {st.description}"
    video.record_video(run, out, policy_file, seconds=res.get("episode_s"), seed=seed, dr_scale=float(res["dr_scale"]), task_params=res["eval_task_params"], label=label,
                       replace_task_params=True)
    return out


def contact_sheet_episode(run: Path, out: Path, seed: int, task_params: dict[str, float], dr_scale: float, episode_s: float | None = None, policy_file: str = "policy.npz",
                          n_frames: int = 6, cols: int = 3, size: tuple[int, int] = (480, 270), label: str | None = None, distance: float = 0.9, azimuth: float = 130.0) -> Path:
    """Write one PNG with ``n_frames`` evenly spaced frames of one deterministic episode (no video, no render budget use)."""
    import mujoco
    from PIL import Image

    from petctl.rl.markers import draw_markers
    from petctl.rl.video import burn_caption, training_label

    pol = GaitPolicy(run, policy_file)
    pol.cfg.task_params, pol.cfg.dr.scale = dict(task_params), dr_scale
    if episode_s is not None:
        pol.cfg.episode_s = episode_s
    env = PetEnv(pol.cfg)
    obs, _ = env.reset(seed=seed)
    w, h = size
    env.sim.model.vis.global_.offwidth, env.sim.model.vis.global_.offheight = max(w, 640), max(h, 480)
    rend = mujoco.Renderer(env.sim.model, height=h, width=w)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = distance, -35, azimuth
    caption = label or training_label(run, pol.cfg.task)

    def grab() -> np.ndarray:
        cam.lookat[:] = env.sim.data.subtree_com[1]
        rend.update_scene(env.sim.data, camera=cam)
        draw_markers(rend.scene, env.task.markers(env.sim))
        return rend.render().copy()

    stride = 15
    frames: list[tuple[float, np.ndarray]] = [(0.0, grab())]
    done, step = False, 0
    while not done:
        obs, _, te, tr, info = env.step(pol.action(obs))
        done, step = te or tr, step + 1
        if step % stride == 0 and not done:
            frames.append((env._t * env.dt, grab()))
    frames.append((env._t * env.dt, grab()))
    idx = np.unique(np.linspace(0, len(frames) - 1, n_frames).round().astype(int))
    tiles = [burn_caption(frames[i][1], f"{caption[:44]} seed {seed} t={frames[i][0]:4.1f}s") for i in idx]
    while len(tiles) % cols:
        tiles.append(np.zeros_like(tiles[0]))
    grid = np.vstack([np.hstack(tiles[r:r + cols]) for r in range(0, len(tiles), cols)])
    out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(grid).save(out)
    return out


def contact_sheet(run: Path, stage: str, seed: int | None = None, out: Path | None = None, policy_file: str = "policy.npz", n_frames: int = 6) -> Path:
    """Contact-sheet PNG of a gate episode of ``run``: the best successful seed by default, or any ``seed`` (e.g. a failure from ``per_seed``)."""
    res, seed = _gate_episode(run, stage, seed)
    out = out or run / f"contact_{stage}_seed{seed}.png"
    return contact_sheet_episode(run, out, seed, res["eval_task_params"], float(res["dr_scale"]), res.get("episode_s"), policy_file, n_frames,
                                 label=f"{stage} {run.name}")
