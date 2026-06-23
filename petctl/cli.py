"""
petctl command-line interface.

Usage:
    petctl run                                  # keyboard + Rerun, mock backend
    petctl run --backend robot                  # use real robot
    petctl run --backend mock --control sine    # animated sensor data
    petctl run --control sine                   # sine wave on real or mock robot
    petctl run --backend mock --state s.json    # load sensor values from file
    petctl run --no-viz                         # headless (sensors only, no Rerun)
    petctl run --dry-run                        # never send servo commands
    petctl run --host 192.168.1.42              # connect to robot by IP
    petctl info                                 # connect and print robot status
"""

import asyncio
import logging
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
        "curl-towards",
        help=(
            "Control scheme: keyboard, passthrough, sine, command, ollama, "
            "wiggle, nuzzle, purr, contort, writhe, engage, withdraw, seek-touch, avoid-touch, "
            "yield, curl, snuggle, explore, twitch, struggle, idle, "
            "cascade, coil, curl-towards, freeze, pose, pulse, stroke, stroke-curl, balanced-torque"
        ),
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
        help="Mock mode: 'interactive', 'file', 'mock-sensor-sine', 'noise'",
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
    calibrate: bool = typer.Option(
        False,
        "--calibrate",
        help="Re-zero software offsets to current pose on connect (robot backend)",
    ),
    motors: Optional[str] = typer.Option(
        None,
        "--motors",
        help="Comma-separated MIT motor IDs to use (e.g. 1). Skips CAN feedback discovery; use for partial hardware.",
    ),
    # SineMotion options
    servo_id: Optional[int] = typer.Option(
        None,
        "--servo-id",
        help="Servo ID to target with sine control (default: all active servos)",
    ),
    # KeyboardMotion options
    step: float = typer.Option(
        4.0,
        help="Degrees per keypress for keyboard control",
    ),
    log_mit: bool = typer.Option(
        False,
        "--log-mit",
        help="Print MIT motor feedback table (pos/vel/torque) every 2s",
    ),
    log_touch: bool = typer.Option(
        False,
        "--log-touch",
        help="Print touch/contact type logs to console",
    ),
    log_ollama_input: bool = typer.Option(
        False,
        "--log-ollama-input",
        help="Print the full JSON payload sent to Ollama on each LLM call",
    ),
    log_loop: bool = typer.Option(
        False,
        "--log-loop",
        help="Print control loop timing stats (Hz, min/mean/max ms) every 5s",
    ),
    log_robot: bool = typer.Option(
        False,
        "--log-robot",
        help="Enable RobotBackend connection and discovery logs",
    ),
    log_imu: bool = typer.Option(
        False,
        "--log-imu",
        help="Print IMU quaternion diagnostics every ~5s",
    ),
    log_power: bool = typer.Option(
        False,
        "--log-power",
        help="Enable PowerManager debug logs (bin seeding, promotion, thermal, voltage events)",
    ),
    dev_ui: bool = typer.Option(
        False,
        "--dev-ui",
        help="Serve pattern dev UI in browser (localhost:8765 by default)",
    ),
    ui_port: int = typer.Option(
        8765,
        "--ui-port",
        help="Port for the dev UI HTTP server",
    ),
) -> None:
    """Run the petctl controller."""

    if log_robot:
        logging.getLogger("petctl.backends.robot").setLevel(logging.DEBUG)
    if log_imu:
        logging.getLogger("petctl.visualizers.rerun_viz").setLevel(logging.DEBUG)
    if log_power:
        logging.getLogger("petctl.power_manager").setLevel(logging.DEBUG)

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
            calibrate_on_connect=calibrate,
            motor_ids=_parse_motor_ids(motors),
        )
    else:
        typer.echo(f"Unknown backend '{backend}'. Choose: mock, robot", err=True)
        raise typer.Exit(1)

    # --- Build motion source ---
    if control == "keyboard":
        from petctl.schemes.keyboard import KeyboardMotion
        _scheme = KeyboardMotion(step_deg=step)
    elif control == "passthrough":
        from petctl.schemes.passthrough import PassthroughMotion
        _scheme = PassthroughMotion()
    elif control == "sine":
        from petctl.schemes.sine import SineMotion
        _scheme = SineMotion(servo_id=servo_id)
    elif control == "command":
        from petctl.schemes.command import CommandMotion
        _scheme = CommandMotion()
    elif control == "ollama":
        from petctl.schemes.ollama_scheme import OllamaMotion
        _scheme = OllamaMotion(log_input=log_ollama_input, llm_enabled=not dev_ui)
    else:
        from petctl.schemes.patterns import ALL_PATTERNS, PATTERN_NAMES
        try:
            _scheme = next(cls() for cls in ALL_PATTERNS if cls.name == control)
        except StopIteration:
            typer.echo(
                f"Unknown control scheme '{control}'. "
                f"Choose: keyboard, passthrough, sine, command, ollama, {', '.join(sorted(PATTERN_NAMES))}",
                err=True,
            )
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
            motion=_scheme,
            visualizers=_visualizers,
            dry_run=dry_run,
            limp=limp,
            log_mit=log_mit,
            log_touch=log_touch,
            log_loop=log_loop,
        )
        _ui = None
        if dev_ui:
            from petctl.dev_ui import DevUI
            _ui = DevUI(ctrl, port=ui_port)
            _ui.start()
            print(f"[DevUI] http://localhost:{ui_port}")
        try:
            await ctrl.run()
        finally:
            if _ui is not None:
                _ui.stop()

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


@app.command()
def calibrate_touch(
    host: str = typer.Option(ROBOT_DEFAULT_HOST, help="Robot hostname or IP"),
    port: int = typer.Option(ROBOT_DEFAULT_PORT, help="Robot WebSocket port"),
) -> None:
    """Re-calibrate MPR121 baseline on all modules.

    Forces all MPR121 capacitive-touch ICs (head + body modules) to reinitialise
    and recapture a fresh baseline.  Use when pads are stuck 'on' after a
    prolonged touch — the MPR121 adaptive baseline drifts during extended contact
    and this resets it.
    """

    async def _run() -> None:
        from petctl.backends.robot import RobotBackend

        backend = RobotBackend(host=host, port=port, calibrate_on_connect=False)
        typer.echo(f"Connecting to {host}:{port}...")
        ok = await backend.connect()
        if not ok:
            typer.echo(f"Could not connect to {host}:{port}", err=True)
            raise typer.Exit(1)
        try:
            typer.echo("Sending calibratetouch...")
            ack = await backend.calibrate_touch()
            if ack:
                typer.echo("Done — MPR121 baselines reset on all modules.")
            else:
                typer.echo("No acknowledgement from robot.", err=True)
                raise typer.Exit(1)
        finally:
            await backend.disconnect()

    asyncio.run(_run())


@app.command()
def touch_threshold(
    touch12: int = typer.Argument(help="Touch threshold for side-face electrodes (mpr1/mpr2)"),
    release12: int = typer.Argument(help="Release threshold for side-face electrodes (must be < touch12)"),
    touch3: Optional[int] = typer.Option(
        None,
        "--touch3",
        help="Touch threshold for top/middle electrodes (mpr3). Defaults to touch12.",
    ),
    release3: Optional[int] = typer.Option(
        None,
        "--release3",
        help="Release threshold for top/middle electrodes (mpr3). Defaults to release12.",
    ),
    host: str = typer.Option(ROBOT_DEFAULT_HOST, help="Robot hostname or IP"),
    port: int = typer.Option(ROBOT_DEFAULT_PORT, help="Robot WebSocket port"),
) -> None:
    """Set MPR121 touch/release thresholds on all modules.

    Thresholds are set on the head directly and broadcast to all body modules
    via CAN.  Changes are volatile — they reset on next reboot.

    Firmware defaults: touch12=5, release12=2, touch3=6/3, release3=3/1 (head/body).

    Raise thresholds to reduce spurious triggers; lower them to increase sensitivity.
    Constraint: touch > release > 0.
    """
    if release12 >= touch12:
        typer.echo(f"Invalid: release12={release12} must be < touch12={touch12}", err=True)
        raise typer.Exit(1)
    if touch3 is not None and release3 is not None and release3 >= touch3:
        typer.echo(f"Invalid: release3={release3} must be < touch3={touch3}", err=True)
        raise typer.Exit(1)

    async def _run() -> None:
        from petctl.backends.robot import RobotBackend

        backend = RobotBackend(host=host, port=port, calibrate_on_connect=False)
        typer.echo(f"Connecting to {host}:{port}...")
        ok = await backend.connect()
        if not ok:
            typer.echo(f"Could not connect to {host}:{port}", err=True)
            raise typer.Exit(1)
        try:
            t3_str = str(touch3) if touch3 is not None else f"{touch12} (=touch12)"
            r3_str = str(release3) if release3 is not None else f"{release12} (=release12)"
            typer.echo(
                f"Setting thresholds — mpr12: touch={touch12} release={release12}"
                f"  mpr3: touch={t3_str} release={r3_str}"
            )
            ack = await backend.set_touch_threshold(touch12, release12, touch3, release3)
            if ack:
                typer.echo("Done.")
            else:
                typer.echo("Robot rejected the command (check: touch > release > 0).", err=True)
                raise typer.Exit(1)
        finally:
            await backend.disconnect()

    asyncio.run(_run())


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s.%(msecs)03d  %(message)s",
        datefmt="%H:%M:%S",
    )
    app()


if __name__ == "__main__":
    main()
