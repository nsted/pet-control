"""``petctl rl ...`` subcommands (train / eval). Imported lazily so the base install does not need gymnasium/stable-baselines3."""

from __future__ import annotations

from pathlib import Path

import typer

from petctl.rl.config import DEFAULT_URDF_DIR


rl_app = typer.Typer(help="Reinforcement-learning experiments (needs: pip install -e '.[rl]')", no_args_is_help=True)


@rl_app.command()
def train(steps: int = typer.Option(200_000, help="Total environment steps"),
          n_envs: int = typer.Option(8, help="Parallel worker processes"),
          seed: int = typer.Option(0), task: str = typer.Option("walk_straight"),
          out: Path = typer.Option(None, help="Output directory (default data/rl/<task>_<seed>)"),
          dr_scale: float = typer.Option(1.0, help="Domain-randomisation scale in [0,1]"),
          algo: str = typer.Option("ppo"),
          action_scale: float = typer.Option(None, help="Joint-target range in rad for action=+-1 (default 0.5; reach_up needs ~2)"),
          episode_s: float = typer.Option(None, help="Episode length in seconds"),
          cap_nm: float = typer.Option(None, help="Fix the per-joint torque cap (N.m) instead of randomising it"),
          action_ema: float = typer.Option(None, help="Action smoothing: target = ema*prev + (1-ema)*new (default 0.6; lower = snappier)"),
          max_step_rad: float = typer.Option(None, help="Per-step target slew limit in rad (default 0.35; raise for explosive motion)"),
          ent_coef: float = typer.Option(0.0, help="PPO entropy bonus (keeps exploration alive)"),
          hours: float = typer.Option(None, help="Stop after this many wall-clock hours (steps is then an upper bound)"),
          task_param: list[str] = typer.Option(None, "--task-param", help="Task knob KEY=VALUE, repeatable (e.g. goal_dist_max=0.4, bearing_deg=90)"),
          init_from: Path = typer.Option(None, help="Warm-start from this PPO run directory (model.zip + vecnormalize.pkl); a larger observation is padded with zeros"),
          video: bool = typer.Option(True, "--video/--no-video", help="Render the end-of-run video into docs/media (off for sweeps)"),
          urdf: str = typer.Option(DEFAULT_URDF_DIR, help="PET_description folder (mesh collision model); '' for the simple box model")) -> None:
    """Train a policy with SB3 and save model, normalisation stats, config and metadata."""
    from petctl.rl.config import EnvConfig
    from petctl.rl.train import train as _train

    cfg = EnvConfig(task=task, seed=seed, urdf_dir=urdf or None)
    cfg.dr.scale = dr_scale
    if action_scale is not None:
        cfg.action_scale_rad = action_scale
    if episode_s is not None:
        cfg.episode_s = episode_s
    if action_ema is not None:
        cfg.action_ema = action_ema
    if max_step_rad is not None:
        cfg.max_target_step_rad = max_step_rad
    if cap_nm is not None:
        cfg.dr.cap_nm = (cap_nm, cap_nm)
    for kv in task_param or []:
        k, _, v = kv.partition("=")
        cfg.task_params[k.strip()] = float(v)
    path = _train(cfg, steps, n_envs, out or Path(f"data/rl/{task}_{seed}"), seed, algo, hours, ent_coef, init_from, video)
    typer.echo(f"Saved to {path}")


@rl_app.command("eval")
def eval_(run: Path = typer.Argument(..., help="Run directory"), episodes: int = typer.Option(20),
          dr_scale: float = typer.Option(None, help="Override DR scale (default: as trained)")) -> None:
    """Deterministic evaluation over fixed held-out seeds."""
    from petctl.rl.train import evaluate

    for k, v in evaluate(run, episodes, dr_scale).items():
        typer.echo(f"{k}: {v:.4f}")


@rl_app.command("export")
def export(run: Path = typer.Argument(..., help="Run directory")) -> None:
    """Write policy.npz (inference-only weights) for an already-trained PPO run."""
    from petctl.rl.policy import export_policy

    typer.echo(f"Wrote {export_policy(run)}")


@rl_app.command("run")
def run_(run: str = typer.Argument(..., help="Skill id/name (see `petctl rl skills`) or a run directory containing policy.npz"), seconds: float = typer.Option(15.0),
         seed: int = typer.Option(0), dr_scale: float = typer.Option(0.0, help="0 = nominal plant"),
         view: bool = typer.Option(False, help="Open the MuJoCo viewer (launch via mjpython)"),
         policy_file: str = typer.Option("policy.npz", help="e.g. best_policy.npz for the policy saved at the best record")) -> None:
    """Enact a trained gait in the sim with no training: loads policy.npz only."""
    from petctl.rl.play import play
    from petctl.rl.skills import resolve

    run = resolve(run)
    for k, v in play(run, seconds, seed, dr_scale, view, policy_file).items():
        typer.echo(f"{k}: {v:.4f}")


rl_app.command("play")(run_)


@rl_app.command("skills")
def skills(task: str = typer.Option(None, help="Only this task")) -> None:
    """List the skill library: verified score per skill, best per task marked."""
    from petctl.rl.skills import list_skills

    rows = list_skills(task)
    for r in rows:
        sc, c = r["score"], r["conditions"]
        typer.echo(f"{'*' if r['best'] else ' '} {r['id']:<34} {100 * sc['nominal_median']:7.1f} cm {r['score_label'].removeprefix('m ')} (best {100 * sc['nominal_best']:.1f}, "
                   f"with randomisation {100 * sc['robust_median']:.1f})  cap {max(c['torque_cap_nm']):g} N.m  {r['created'][:10]}")
        for cav in r["caveats"]:
            typer.echo(f"      ! {cav}")
    if not rows:
        typer.echo("no skills yet (add one with: petctl rl skill-add RUN --name NAME)")


@rl_app.command("skill-add")
def skill_add(run: Path = typer.Argument(..., help="Run directory"), name: str = typer.Option(..., help="Skill name, e.g. jump_20m_v1"),
              policy_file: str = typer.Option("policy.npz", help="e.g. best_policy.npz"), note: str = typer.Option("", help="Free-text description"),
              no_video: bool = typer.Option(False, help="Skip the demo video")) -> None:
    """Verify a trained policy by replay and add it to the skill library (models/skills/<task>/<name>)."""
    from petctl.rl.skills import add_skill

    dest = add_skill(run, name, policy_file, note, render=not no_video)
    typer.echo(f"Added {dest}")


@rl_app.command("skill-video")
def skill_video(name: str = typer.Argument(..., help="Skill id or name"), seed: int = typer.Option(0)) -> None:
    """Re-record a skill's demo.mp4."""
    from petctl.rl.skills import render_demo, resolve

    typer.echo(f"Wrote {render_demo(resolve(name), seed)}")


@rl_app.command("finalize")
def finalize(session: Path = typer.Argument(..., help="Session folder under docs/media/sessions (after an aborted run)")) -> None:
    """Mark the best video and write index.md/json for a session whose training was stopped early."""
    from petctl.rl.video import finalize_session

    best_video = finalize_session(session)
    typer.echo(f"best: {best_video}" if best_video else "no videos in this session")


@rl_app.command("best")
def best(runs: list[Path] = typer.Argument(None, help="Run directories (default: every data/rl/*/best.json)")) -> None:
    """Print the best height reached so far in each run, highest first."""
    import json

    files = [r / "best.json" for r in runs] if runs else sorted(Path("data/rl").glob("*/best.json"))
    rows = []
    for f in files:
        if f.exists():
            b = json.loads(f.read_text())
            rows.append((b["height_gain_cm"], b["highest_point_above_floor_cm"], f.parent.name, b["timesteps"], b["wall_s"] / 60, b.get("saved", ""), b.get("kind", "height")))
    for gain, top, name, steps, mins, saved, kind in sorted(rows, reverse=True):
        what = f"clearance {gain:.1f} cm (lowest point off the floor)" if kind == "clearance" else f"+{gain:.1f} cm (highest point {top:.1f} cm above floor)"
        typer.echo(f"{name}: {what} at {steps} steps, {mins:.0f} min, {saved}")
    if not rows:
        typer.echo("no best.json found")


@rl_app.command("stages")
def stages() -> None:
    """List the curriculum stages, their gates and tunable knobs."""
    from petctl.rl.curriculum.stages import STAGES

    for st in STAGES.values():
        g = st.gate
        typer.echo(f"{st.name}  [{st.task}]  parent: {st.parent or '-'}\n  {st.description}\n  gate: success >= {g.min_success} at DR {g.dr_scale}, {g.episodes} eps, goal {g.task_params}\n"
                   f"  knobs: {', '.join(f'{k} {lo:g}..{hi:g}' for k, (lo, hi) in st.knobs.items())}")


@rl_app.command("stage-train")
def stage_train(stage: str = typer.Argument(..., help="Stage name (see `petctl rl stages`)"),
                set_: list[str] = typer.Option(None, "--set", help="Knob override KEY=VALUE (validated against the stage's allowed ranges), repeatable"),
                out: Path = typer.Option(None, help="Run directory (default data/rl/curriculum/<stage>/iter_NN)"),
                init_from: Path = typer.Option(None, help="Warm-start run (default: latest passing run of the parent stage)"),
                seed: int = typer.Option(0), note: str = typer.Option("", help="Why this iteration (recorded in iteration.json)")) -> None:
    """Train one iteration of a curriculum stage with guard-railed overrides."""
    from petctl.rl.curriculum.runner import launch
    from petctl.rl.curriculum.stages import get_stage

    overrides = {k.strip(): float(v) for k, _, v in (kv.partition("=") for kv in set_ or [])}
    typer.echo(f"Saved to {launch(get_stage(stage), overrides, out, init_from, seed, note)}")


@rl_app.command("watch")
def watch_(run: Path = typer.Argument(..., help="Run directory"), stage: str = typer.Option(None, help="Stage whose gate level defines 'passing'"),
           window: int = typer.Option(100, help="Episodes per rolling window"),
           until_actionable: bool = typer.Option(False, help="Poll until the verdict needs action (passing/plateau/collapse/glitch/stalled/finished)"),
           poll_s: float = typer.Option(60.0), max_wait_h: float = typer.Option(6.0)) -> None:
    """Classify a training run from its episode log; prints the verdict JSON and writes <run>/verdict.json (no LLM involved)."""
    import json
    from dataclasses import asdict

    from petctl.rl.curriculum.stages import get_stage
    from petctl.rl.curriculum.watch import watch

    pass_score = get_stage(stage).gate.min_success if stage else None
    typer.echo(json.dumps(asdict(watch(run, pass_score, window, until_actionable, poll_s, max_wait_h * 3600)), indent=1))


@rl_app.command("gate")
def gate_(run: Path = typer.Argument(..., help="Run directory"), stage: str = typer.Option(..., help="Stage whose frozen gate to apply"),
          policy_file: str = typer.Option("policy.npz", help="e.g. best_policy.npz")) -> None:
    """Run the stage's frozen gate on a policy; prints JSON, exit code 0 = pass, 1 = fail."""
    import json

    from petctl.rl.curriculum.gate import run_gate
    from petctl.rl.curriculum.stages import get_stage

    res = run_gate(run, get_stage(stage), policy_file)
    typer.echo(json.dumps(res, indent=1))
    raise typer.Exit(0 if res["passed"] else 1)
