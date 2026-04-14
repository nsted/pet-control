"""
petctl command-line interface.

Usage:
    petctl run                                  # keyboard + Rerun, mock backend
    petctl run --backend robot                  # use real robot
    petctl run --backend mock --mode sine       # animated sensor data
    petctl run --control sine                   # sine wave on real or mock robot
    petctl run --backend mock --state s.json    # load sensor values from file
    petctl run --no-viz                         # headless (sensors only, no Rerun)
    petctl run --dry-run                        # never send servo commands
    petctl run --host 192.168.1.42              # connect to robot by IP
    petctl info                                 # connect and print robot status
"""

import asyncio
import sys
from typing import Optional

import typer

from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT

app = typer.Typer(
    name="petctl",
    help="PET robot control framework",
    add_completion=False,
)


def _parse_motor_ids(s: Optional[str]) -> Optional[tuple[int, ...]]:
    """Parse --motors \"1\" or \"1,3\" into a tuple of ints; None if unset."""
    if not s or not str(s).strip():
        return None
    parts = [int(p.strip()) for p in str(s).split(",") if p.strip()]
    return tuple(parts) if parts else None


@app.command()
def run(
    backend: str = typer.Option(
        "mock",
        help="Backend: 'mock' (no robot) or 'robot' (real robot)",
    ),
    control: str = typer.Option(
        "keyboard",
        help="Control scheme: 'keyboard', 'passthrough', or 'sine'",
    ),
    no_viz: bool = typer.Option(
        False,
        "--no-viz",
        help="Disable Rerun visualizer (headless mode)",
    ),
    dry_run: bool = typer.Option(
        False,
        help="Read sensors and run scheme but do NOT send servo commands",
    ),
    limp: bool = typer.Option(
        False,
        "--limp",
        help="Disable motor torque after connect so joints move freely; visualizer still updates from read positions",
    ),
    # MockBackend options
    mode: str = typer.Option(
        "interactive",
        help="Mock mode: 'interactive', 'file', 'sine', 'noise'",
    ),
    state: Optional[str] = typer.Option(
        None,
        "--state",
        help="Path to mock_state.json for file/interactive modes",
    ),
    num_modules: int = typer.Option(
        8,
        help="Number of simulated modules (mock backend only)",
    ),
    # RobotBackend options
    host: str = typer.Option(
        ROBOT_DEFAULT_HOST,
        help="Robot hostname or IP address (robot backend)",
    ),
    port: int = typer.Option(
        ROBOT_DEFAULT_PORT,
        help="Robot WebSocket port (robot backend)",
    ),
    no_calibrate: bool = typer.Option(
        False,
        "--no-calibrate",
        help="Skip sensor calibration on connect (robot backend)",
    ),
    motors: Optional[str] = typer.Option(
        None,
        "--motors",
        help="Comma-separated MIT motor IDs to use (e.g. 1). Skips CAN feedback discovery; use for partial hardware.",
    ),
    # SineControlScheme options
    servo_id: Optional[int] = typer.Option(
        None,
        "--servo-id",
        help="Servo ID to target with sine control (default: all active servos)",
    ),
    # KeyboardControlScheme options
    step: float = typer.Option(
        0.5,
        help="Degrees per keypress for keyboard control",
    ),
) -> None:
    """Run the petctl controller."""

    # --- Build backend ---
    if backend == "mock":
        from petctl.backends.mock import MockBackend
        _backend = MockBackend(
            mode=mode,
            state_file=state,
            num_modules=num_modules,
        )
    elif backend == "robot":
        from petctl.backends.robot import RobotBackend
        _backend = RobotBackend(
            host=host,
            port=port,
            calibrate_on_connect=not no_calibrate,
            motor_ids=_parse_motor_ids(motors),
        )
    else:
        typer.echo(f"Unknown backend '{backend}'. Choose: mock, robot", err=True)
        raise typer.Exit(1)

    # --- Build control scheme ---
    if control == "keyboard":
        from petctl.schemes.keyboard import KeyboardControlScheme
        _scheme = KeyboardControlScheme(step_deg=step)
    elif control == "passthrough":
        from petctl.schemes.passthrough import PassthroughControlScheme
        _scheme = PassthroughControlScheme()
    elif control == "sine":
        from petctl.schemes.sine import SineControlScheme
        _scheme = SineControlScheme(servo_id=servo_id)
    else:
        typer.echo(f"Unknown control scheme '{control}'. Choose: keyboard, passthrough, sine", err=True)
        raise typer.Exit(1)

    # --- Build visualizers ---
    _visualizers = []
    if not no_viz:
        from petctl.visualizers.rerun_viz import RerunVisualizer
        _visualizers.append(RerunVisualizer())

    # --- Run ---
    from petctl.controller import Controller

    async def _run() -> None:
        ctrl = Controller(
            backend=_backend,
            scheme=_scheme,
            visualizers=_visualizers,
            dry_run=dry_run,
            limp=limp,
        )
        await ctrl.run()

    try:
        asyncio.run(_run())
    except KeyboardInterrupt:
        pass


@app.command()
def info(
    host: str = typer.Option(ROBOT_DEFAULT_HOST, help="Robot hostname or IP"),
    port: int = typer.Option(ROBOT_DEFAULT_PORT, help="Robot WebSocket port"),
    motors: Optional[str] = typer.Option(
        None,
        "--motors",
        help="Comma-separated motor IDs (same as petctl run --backend robot --motors)",
    ),
) -> None:
    """Connect to the real robot and print discovered modules and servos."""

    async def _info() -> None:
        from petctl.backends.robot import RobotBackend

        backend = RobotBackend(
            host=host,
            port=port,
            calibrate_on_connect=False,
            motor_ids=_parse_motor_ids(motors),
        )
        typer.echo(f"Connecting to {host}:{port}...")
        ok = await backend.connect()
        if not ok:
            typer.echo(f"Could not connect to {host}:{port}", err=True)
            raise typer.Exit(1)

        try:
            typer.echo(f"\n  Connected:        {host}:{port}")
            typer.echo(f"  Active modules:   {sorted(backend.discovered_modules)}")
            typer.echo(f"  Discovered servos:{sorted(backend.discovered_servos)}")
        finally:
            await backend.disconnect()

    asyncio.run(_info())


def main() -> None:
    app()


if __name__ == "__main__":
    main()
