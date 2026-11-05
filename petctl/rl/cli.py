"""``petctl rl ...`` subcommands (train / eval). Imported lazily so the base install does not need gymnasium/stable-baselines3."""

from __future__ import annotations

from pathlib import Path

import typer

DEFAULT_URDF = "/Users/nsted/Documents/PET/body/URDF/PET_description"

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
          urdf: str = typer.Option(DEFAULT_URDF, help="PET_description folder (mesh collision model); '' for the simple box model")) -> None:
    """Train a policy with SB3 and save model, normalisation stats, config and metadata."""
    from petctl.rl.config import EnvConfig
    from petctl.rl.train import train as _train

    cfg = EnvConfig(task=task, seed=seed, urdf_dir=urdf or None)
    cfg.dr.scale = dr_scale
    if action_scale is not None:
        cfg.action_scale_rad = action_scale
    if episode_s is not None:
        cfg.episode_s = episode_s
    if cap_nm is not None:
        cfg.dr.cap_nm = (cap_nm, cap_nm)
    path = _train(cfg, steps, n_envs, out or Path(f"data/rl/{task}_{seed}"), seed, algo)
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
def run_(run: Path = typer.Argument(..., help="Run directory containing policy.npz"), seconds: float = typer.Option(15.0),
         seed: int = typer.Option(0), dr_scale: float = typer.Option(0.0, help="0 = nominal plant"),
         view: bool = typer.Option(False, help="Open the MuJoCo viewer (launch via mjpython)")) -> None:
    """Enact a trained gait in the sim with no training: loads policy.npz only."""
    from petctl.rl.play import play

    for k, v in play(run, seconds, seed, dr_scale, view).items():
        typer.echo(f"{k}: {v:.4f}")
