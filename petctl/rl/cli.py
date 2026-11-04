"""``petctl rl ...`` subcommands (train / eval). Imported lazily so the base install does not need gymnasium/stable-baselines3."""

from __future__ import annotations

from pathlib import Path

import typer

rl_app = typer.Typer(help="Reinforcement-learning experiments (needs: pip install -e '.[rl]')", no_args_is_help=True)


@rl_app.command()
def train(steps: int = typer.Option(200_000, help="Total environment steps"),
          n_envs: int = typer.Option(8, help="Parallel worker processes"),
          seed: int = typer.Option(0), task: str = typer.Option("locomote"),
          out: Path = typer.Option(None, help="Output directory (default data/rl/<task>_<seed>)"),
          dr_scale: float = typer.Option(1.0, help="Domain-randomisation scale in [0,1]"),
          algo: str = typer.Option("ppo")) -> None:
    """Train a policy with SB3 and save model, normalisation stats, config and metadata."""
    from petctl.rl.config import EnvConfig
    from petctl.rl.train import train as _train

    cfg = EnvConfig(task=task, seed=seed)
    cfg.dr.scale = dr_scale
    path = _train(cfg, steps, n_envs, out or Path(f"data/rl/{task}_{seed}"), seed, algo)
    typer.echo(f"Saved to {path}")


@rl_app.command("eval")
def eval_(run: Path = typer.Argument(..., help="Run directory"), episodes: int = typer.Option(20),
          dr_scale: float = typer.Option(None, help="Override DR scale (default: as trained)")) -> None:
    """Deterministic evaluation over fixed held-out seeds."""
    from petctl.rl.train import evaluate

    for k, v in evaluate(run, episodes, dr_scale).items():
        typer.echo(f"{k}: {v:.4f}")
