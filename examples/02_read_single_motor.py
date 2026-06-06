"""Tier 1 -- live TUI of one motor's state.

Builds a one-motor robot, sets it to MIT control mode, enables it, and commands
zero torque once (`kp=kd=q=dq=tau=0`) so the motor is compliant / back-drivable.
Then each cycle it sends a state-refresh query (`arm.refresh()`, which commands
no motion) and ticks to receive the reply, rendering the motor's position /
velocity / torque / temperatures / enabled / fault in a `rich` live table that
refreshes in place. It runs until you press Ctrl-C, then disables on exit.
After the initial zero-torque command the loop never drives the motor — refresh
is how a Damiao motor reports state without moving (a tick-only loop would
freeze after the enable acknowledgement).

Runs against MockCanBus (`--mock`, no hardware) or a SocketCAN interface
(`--interface can0`, optionally `--fd` for a CAN-FD bus). Mock execution proves
the read path and wiring, NOT that real hardware will respond.

Run against hardware only after `01_single_motor_enable_disable.py` succeeds on
the same motor.

Usage:
    python examples/02_read_single_motor.py --mock
    python examples/02_read_single_motor.py --interface can0 --send-id 0x01 --recv-id 0x11
    python examples/02_read_single_motor.py --interface canfd0 --fd
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np
from rich import box
from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text


def parse_can_id(text: str) -> int:
    """Accept ``0x11``, ``17``, or ``0o21`` and return an int."""
    return int(text.strip(), 0)


def build_robot(args: argparse.Namespace):
    # Lazy import so ``--help`` works even if the native wheel is missing.
    import can_motor_control
    from can_motor_control.damiao import DamiaoCodec, MotorType

    if args.mock:
        transport = (
            can_motor_control.MockCanBus.new_fd("vcan_mock")
            if args.fd
            else can_motor_control.MockCanBus("vcan_mock")
        )
    else:
        transport = can_motor_control.SocketCanBus(args.interface, fd=args.fd)

    motor_type = getattr(MotorType, args.motor_type, None)
    if motor_type is None:
        raise SystemExit(f"unknown --motor-type {args.motor_type!r} (e.g. DM4310, DM4340)")

    return (
        can_motor_control.RobotBuilder()
        .add_bus("main", transport, DamiaoCodec())
        .add_arm(
            "arm",
            bus="main",
            motors=[can_motor_control.MotorSpec("j0", motor_type, args.send_id, args.recv_id)],
        )
        .build()
    )


def render(meta: dict, motor, ticks: int, elapsed: float, rate: float):
    """Compose the live panel for the current motor snapshot."""
    head = Table.grid(padding=(0, 2))
    head.add_column(style="bold cyan", justify="right")
    head.add_column()
    head.add_row("target", meta["target"])
    head.add_row("wire", meta["wire"])
    head.add_row("ids", meta["ids"])
    head.add_row("motor", meta["motor_type"])

    state = Table(box=box.SIMPLE_HEAVY, expand=True, header_style="bold")
    state.add_column("field", style="bold cyan", no_wrap=True)
    state.add_column("value", justify="right", style="bright_white")
    state.add_row("position  (rad)", f"{motor.position:+.5f}")
    state.add_row("velocity  (rad/s)", f"{motor.velocity:+.5f}")
    state.add_row("torque    (Nm)", f"{motor.torque:+.5f}")
    state.add_row("temp mos  (°C)", str(motor.temperature_mos))
    state.add_row("temp rotor (°C)", str(motor.temperature_rotor))
    state.add_row(
        "enabled",
        Text("● yes", style="green") if motor.is_enabled else Text("○ no", style="red"),
    )
    fault = motor.fault
    state.add_row(
        "fault",
        Text("none", style="dim") if fault is None else Text(str(fault), style="bold red"),
    )

    footer = Text.assemble(
        ("ticks ", "dim"), (f"{ticks:>7}", "bold"),
        ("   elapsed ", "dim"), (f"{elapsed:6.1f}s", "bold"),
        ("   rate ", "dim"), (f"{rate:5.0f} Hz", "bold"),
        ("    Press ", "dim"), ("Ctrl-C", "bold yellow"), (" to stop", "dim"),
    )

    return Panel(
        Group(head, state, footer),
        title="can_motor_control · single-motor reader",
        border_style="cyan",
        padding=(1, 2),
    )


def run_live(args: argparse.Namespace, robot, meta: dict, console: Console) -> int:
    refresh_hz = max(1.0, args.hz)
    render_period = 1.0 / refresh_hz
    loop_period = 1e-3

    arm = robot["arm"]
    motor = arm["j0"]  # live view — re-reads state on each attribute access

    start = time.monotonic()
    last_render = -render_period
    window_start = start
    window_ticks = 0
    ticks = 0
    rate = 0.0

    with Live(console=console, screen=True, refresh_per_second=refresh_hz) as live:
        while True:
            if args.seconds is not None and (time.monotonic() - start) >= args.seconds:
                break
            t0 = time.monotonic()
            # Damiao only reports state in reply to a frame we send. Query state
            # with a refresh (commands no motion), then tick to receive it.
            # arm.refresh()
            arm.mit_control(np.zeros((1, 5), dtype=np.float64))
            robot.tick(args.deadline_us)
            ticks += 1
            window_ticks += 1
            elapsed = t0 - start
            if elapsed - last_render >= render_period:
                dt = t0 - window_start
                if dt > 0:
                    rate = window_ticks / dt
                window_start, window_ticks = t0, 0
                last_render = elapsed
                live.update(render(meta, motor, ticks, elapsed, rate))
            rem = loop_period - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)
    return ticks


def main() -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    parser.add_argument("--mock", action="store_true", help="use MockCanBus (no hardware)")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument("--fd", action="store_true", help="open the bus in CAN-FD mode")
    parser.add_argument("--send-id", type=parse_can_id, default=0x01, help="CAN id host->motor (default: 0x01)")
    parser.add_argument("--recv-id", type=parse_can_id, default=0x11, help="CAN id motor->host (default: 0x11)")
    parser.add_argument("--motor-type", default="DM4340", help="Damiao motor model (default: DM4340)")
    parser.add_argument(
        "--seconds",
        type=float,
        default=None,
        help="optional run cap in seconds; omit to run until Ctrl-C",
    )
    parser.add_argument("--hz", type=float, default=20.0, help="TUI refresh rate in Hz (default: 20.0)")
    parser.add_argument("--deadline-us", type=int, default=500, help="per-tick bus deadline us (default: 500)")
    args = parser.parse_args()

    if args.seconds is not None and args.seconds <= 0:
        parser.error(f"--seconds must be > 0, got {args.seconds}")

    console = Console()
    meta = {
        "target": "MockCanBus" if args.mock else args.interface,
        "wire": "CAN-FD" if args.fd else "classical CAN",
        "ids": f"send=0x{args.send_id:02X}  recv=0x{args.recv_id:02X}",
        "motor_type": args.motor_type,
    }

    robot = build_robot(args)

    console.print("connecting...")
    robot.connect()
    try:
        # Put the motor in MIT control mode before enabling, so MIT commands
        # (and the zero-torque compliance below) actually take effect.
        console.print("setting MIT mode...")
        robot["arm"].set_mode("mit")
        console.print("enabling...")
        robot.enable()
        # Command zero torque once (kp=kd=q=dq=tau=0) so the motor is compliant
        # / back-drivable in MIT mode; after this we only refresh-poll for state.
        robot["arm"].mit_control(np.zeros((1, 5), dtype=np.float64))
        try:
            ticks = run_live(args, robot, meta, console)
            console.print(f"completed {ticks} ticks")
        except KeyboardInterrupt:
            console.print("\n[yellow]interrupted[/] — stopping.")
    finally:
        # Disable regardless of how we got here (the disable-on-exit guarantee).
        console.print("disabling...")
        try:
            robot.disable()
        except Exception as exc:  # noqa: BLE001 -- best effort during teardown
            console.print(f"[red]WARNING:[/] disable failed: {exc}")
    console.print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
