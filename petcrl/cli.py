"""
petcrl command-line interface.

Usage:
    petcrl run                                  # keyboard + Rerun, mock backend
    petcrl run --backend grapple                # use real robot
    petcrl run --backend mock --mode sine       # animated sensor data
    petcrl run --backend mock --state s.json    # load sensor values from file
    petcrl run --no-viz                         # headless (sensors only, no Rerun)
    petcrl run --dry-run                        # never send servo commands
    petcrl run --hz 10                          # 10 Hz control loop
    petcrl run --host 192.168.1.42              # connect to robot by IP
    petcrl info                                 # connect and print robot status
"""

import asyncio
import sys
from typing import Optional

import typer

app = typer.Typer(
    name="petcrl",
    help="PET robot control framework",
    add_completion=False,
)


@app.command()
def run(
    backend: str = typer.Option(
        "mock",
        help="Backend: 'mock' (no robot) or 'grapple' (real robot)",
    ),
    control: str = typer.Option(
        "keyboard",
        help="Control scheme: 'keyboard' or 'passthrough'",
    ),
    no_viz: bool = typer.Option(
        False,
        "--no-viz",
        help="Disable Rerun visualizer (headless mode)",
    ),
    hz: float = typer.Option(
        20.0,
        help="Control loop frequency in Hz",
    ),
    dry_run: bool = typer.Option(
        False,
        help="Read sensors and run scheme but do NOT send servo commands",
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
        4,
        help="Number of simulated modules (mock backend only)",
    ),
    # GrappleBackend options
    host: str = typer.Option(
        "pet-robot.local",
        help="Robot hostname or IP address (grapple backend)",
    ),
    port: int = typer.Option(
        8080,
        help="Robot WebSocket port (grapple backend)",
    ),
    no_calibrate: bool = typer.Option(
        False,
        "--no-calibrate",
        help="Skip sensor calibration on connect (grapple backend)",
    ),
    # KeyboardControlScheme options
    step: float = typer.Option(
        5.0,
        help="Degrees per keypress for keyboard control",
    ),
) -> None:
    """Run the petcrl controller."""

    # --- Build backend ---
    if backend == "mock":
        from petcrl.backends.mock import MockBackend
        _backend = MockBackend(
            mode=mode,
            state_file=state,
            num_modules=num_modules,
        )
    elif backend == "grapple":
        from petcrl.backends.grapple import GrappleBackend
        _backend = GrappleBackend(
            host=host,
            port=port,
            calibrate_on_connect=not no_calibrate,
        )
    else:
        typer.echo(f"Unknown backend '{backend}'. Choose: mock, grapple", err=True)
        raise typer.Exit(1)

    # --- Build control scheme ---
    if control == "keyboard":
        from petcrl.schemes.keyboard import KeyboardControlScheme
        _scheme = KeyboardControlScheme(step_deg=step)
    elif control == "passthrough":
        from petcrl.schemes.passthrough import PassthroughControlScheme
        _scheme = PassthroughControlScheme()
    else:
        typer.echo(f"Unknown control scheme '{control}'. Choose: keyboard, passthrough", err=True)
        raise typer.Exit(1)

    # --- Build visualizers ---
    _visualizers = []
    if not no_viz:
        from petcrl.visualizers.rerun_viz import RerunVisualizer
        _visualizers.append(RerunVisualizer())

    # --- Run ---
    from petcrl.controller import Controller

    async def _run() -> None:
        ctrl = Controller(
            backend=_backend,
            scheme=_scheme,
            visualizers=_visualizers,
            poll_hz=hz,
            dry_run=dry_run,
        )
        await ctrl.run()

    try:
        asyncio.run(_run())
    except KeyboardInterrupt:
        pass


@app.command()
def info(
    host: str = typer.Option("pet-robot.local", help="Robot hostname or IP"),
    port: int = typer.Option(8080, help="Robot WebSocket port"),
) -> None:
    """Connect to the real robot and print discovered modules and servos."""

    async def _info() -> None:
        try:
            from grapple import GrappleRobot
        except ImportError:
            typer.echo(
                "grapple-python not installed.\n"
                "Run: pip install -e /path/to/grapple-python",
                err=True,
            )
            raise typer.Exit(1)

        robot = GrappleRobot(host=host, port=port)
        typer.echo(f"Connecting to {host}:{port}...")
        ok = await robot.connect()
        if not ok:
            typer.echo(f"Could not connect to {host}:{port}", err=True)
            raise typer.Exit(1)

        typer.echo(f"\n  Connected:        {host}:{port}")
        typer.echo(f"  Active modules:   {sorted(robot.discovered_modules)}")
        typer.echo(f"  Discovered servos:{sorted(robot.discovered_servos)}")
        typer.echo(f"  Calibrated:       {robot.is_calibrated()}")

        await robot.disconnect()

    asyncio.run(_info())


def main() -> None:
    app()


if __name__ == "__main__":
    main()
