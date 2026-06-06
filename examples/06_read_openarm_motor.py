"""Tier 3 -- live TUI of the whole OpenArm single-arm setup.

Builds the OpenArm single arm modeled faithfully as two groups on one bus: a
7-joint **arm** (DM8009, DM8009, DM4340, DM4340, DM4310, DM4310, DM4310) and a
1-motor **gripper** (DM4310). It sets every motor to MIT control mode, enables
them, and commands zero torque once (`kp=kd=q=dq=tau=0`) so the arm and gripper
are compliant / back-drivable. Then each cycle it sends a zero-torque MIT command
to the arm (a `(7,5)` array) and the gripper (scalar args) — which holds zero
torque *and* makes each motor report state — and ticks to receive the replies,
rendering per-motor group / type / position / velocity / torque / temperatures /
enabled / fault in a `rich` live table that refreshes in place. It runs until you
press Ctrl-C, then disables on exit. The zero-torque command nets no motion — it
keeps the arm back-drivable while a Damiao motor will only report state in reply
to a frame we send (a tick-only loop would freeze after the enable
acknowledgement).

The per-motor model matters: the codec decodes position/velocity/torque against
each type's limits, so the 8009 and 4310 readings would be mis-scaled if every
motor were declared the same type. Edit the `ARM_MOTORS` table to match your arm.

Runs against MockCanBus (`--mock`, no hardware) or a SocketCAN interface
(`--interface can0`, optionally `--fd` for a CAN-FD bus). Mock execution proves
the read path and wiring, NOT that real hardware will respond.

Run on hardware only after each individual motor has passed `01` and `02`, and
after gripper-only control has been checked with `05_gripper_control.py`.

Usage:
    python examples/06_read_openarm_motor.py --mock
    python examples/06_read_openarm_motor.py --interface can0
    python examples/06_read_openarm_motor.py --interface canfd0 --fd
"""

from __future__ import annotations

import argparse
import time

import numpy as np
from rich import box
from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

# OpenArm single arm, modeled faithfully as a 7-joint arm group + a 1-motor
# gripper group on one bus. Per-motor (name, Damiao model, send_id, recv_id) —
# the model matters because the codec decodes position/velocity/torque against
# each type's limits. Edit to match your arm; CAN ids use send=0x0N / recv=0x1N.
ARM_MOTORS = [
    ("j1", "DM8009", 0x01, 0x11),
    ("j2", "DM8009", 0x02, 0x12),
    ("j3", "DM4340", 0x03, 0x13),
    ("j4", "DM4340", 0x04, 0x14),
    ("j5", "DM4310", 0x05, 0x15),
    ("j6", "DM4310", 0x06, 0x16),
    ("j7", "DM4310", 0x07, 0x17),
]
GRIPPER = ("grip", "DM4310", 0x08, 0x18)


def _resolve(MotorType, name: str, type_str: str):
    mt = getattr(MotorType, type_str, None)
    if mt is None:
        raise SystemExit(f"unknown motor type {type_str!r} for {name} (e.g. DM4310, DM4340, DM8009)")
    return mt


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

    arm_specs = [
        can_motor_control.MotorSpec(name, _resolve(MotorType, name, type_str), send_id, recv_id)
        for (name, type_str, send_id, recv_id) in ARM_MOTORS
    ]
    g_name, g_type, g_send, g_recv = GRIPPER
    grip_spec = can_motor_control.MotorSpec(g_name, _resolve(MotorType, g_name, g_type), g_send, g_recv)

    robot = (
        can_motor_control.RobotBuilder()
        .add_bus("main", transport, DamiaoCodec())
        .add_arm("arm", bus="main", motors=arm_specs)
        .add_gripper("grip", bus="main", motor=grip_spec)
        .build()
    )
    return robot


def _add_motor_row(table, group: str, name: str, type_str: str, m, pos, vel, tau):
    """Append one motor's state as a table row (pos/vel/tau passed in so the arm
    can use its batch arrays and the gripper its single Motor)."""
    enabled = Text("●", style="green") if m.is_enabled else Text("○", style="red")
    fault = m.fault
    fault_cell = Text("none", style="dim") if fault is None else Text(str(fault), style="bold red")
    table.add_row(
        group,
        name,
        type_str,
        f"{pos:+.5f}",
        f"{vel:+.5f}",
        f"{tau:+.4f}",
        str(m.temperature_mos),
        str(m.temperature_rotor),
        enabled,
        fault_cell,
    )


def render(meta, arm, arm_motors, gripper, gripper_type, ticks, elapsed, rate):
    """Compose the live panel for the current arm + gripper snapshot.

    ``arm_motors`` is the ordered list of arm ``(name, type)`` pairs, matching
    the insertion order of the arm's batch state arrays.
    """
    head = Table.grid(padding=(0, 2))
    head.add_column(style="bold cyan", justify="right")
    head.add_column()
    head.add_row("target", meta["target"])
    head.add_row("wire", meta["wire"])
    head.add_row("groups", f"arm ({len(arm_motors)}) + gripper (1)")

    state = Table(box=box.SIMPLE_HEAVY, expand=True, header_style="bold")
    state.add_column("group", style="dim", no_wrap=True)
    state.add_column("motor", style="bold cyan", no_wrap=True)
    state.add_column("type", style="cyan", no_wrap=True)
    state.add_column("pos (rad)", justify="right", style="bright_white")
    state.add_column("vel (rad/s)", justify="right", style="bright_white")
    state.add_column("τ (Nm)", justify="right", style="bright_white")
    state.add_column("mos", justify="right")
    state.add_column("rotor", justify="right")
    state.add_column("en", justify="center")
    state.add_column("fault", justify="left")

    # Arm joints from the batch arrays (one read each, in insertion order).
    pos = arm.positions()
    vel = arm.velocities()
    tau = arm.torques()
    for i, (name, type_str) in enumerate(arm_motors):
        _add_motor_row(state, "arm", name, type_str, arm[name], pos[i], vel[i], tau[i])

    # Gripper from its single motor.
    gm = gripper.motor
    _add_motor_row(state, "gripper", gm.name, gripper_type, gm, gm.position, gm.velocity, gm.torque)

    footer = Text.assemble(
        ("ticks ", "dim"), (f"{ticks:>7}", "bold"),
        ("   elapsed ", "dim"), (f"{elapsed:6.1f}s", "bold"),
        ("   rate ", "dim"), (f"{rate:5.0f} Hz", "bold"),
        ("    Press ", "dim"), ("Ctrl-C", "bold yellow"), (" to stop", "dim"),
    )

    return Panel(
        Group(head, state, footer),
        title="can_motor_control · OpenArm reader",
        border_style="cyan",
        padding=(1, 2),
    )


def run_live(args: argparse.Namespace, robot, meta: dict, console: Console) -> int:
    refresh_hz = max(1.0, args.hz)
    render_period = 1.0 / refresh_hz
    loop_period = 1e-3

    arm = robot["arm"]
    gripper = robot["grip"]
    arm_motors = [(name, type_str) for (name, type_str, _s, _r) in ARM_MOTORS]
    gripper_type = GRIPPER[1]
    hold = np.zeros((len(arm_motors), 5), dtype=np.float64)  # zero-torque MIT for the arm

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
            # A Damiao motor only reports state in reply to a frame we send. The
            # zero-torque MIT command nets no motion yet elicits a state reply
            # from each motor; tick receives them. (Alternative: arm.refresh() /
            # gripper.refresh().) The gripper's MIT command takes scalar args.
            arm.mit_control(hold)
            gripper.mit_control(0.0, 0.0, 0.0, 0.0, 0.0)
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
                live.update(
                    render(meta, arm, arm_motors, gripper, gripper_type, ticks, elapsed, rate)
                )
            rem = loop_period - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)
    return ticks


def main() -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    parser.add_argument("--mock", action="store_true", help="use MockCanBus (no hardware)")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument("--fd", action="store_true", help="open the bus in CAN-FD mode")
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
    }

    robot = build_robot(args)

    console.print("connecting...")
    robot.connect()
    try:
        arm = robot["arm"]
        gripper = robot["grip"]
        # Put every motor (arm + gripper) in MIT control mode before enabling, so
        # MIT commands (and the zero-torque compliance below) take effect.
        console.print("setting MIT mode...")
        robot.set_mode("mit")
        console.print("enabling...")
        robot.enable()
        # Command zero torque once so the arm + gripper are compliant /
        # back-drivable; the loop then keeps them that way while polling state.
        arm.mit_control(np.zeros((len(ARM_MOTORS), 5), dtype=np.float64))
        gripper.mit_control(0.0, 0.0, 0.0, 0.0, 0.0)
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
