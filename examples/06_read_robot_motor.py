"""Tier 3 -- live TUI of an OpenArm or OpenYam single-arm setup.

Builds the selected robot as two groups on one bus: OpenArm has seven joints
and OpenYam has six, while both have a 1-motor gripper (motor 8). It sets every
motor to MIT control mode, enables them, and commands zero torque once
(`kp=kd=q=dq=tau=0`) so the arm and gripper are compliant / back-drivable.
Then each cycle it sends a zero-torque MIT command to the selected arm and the
gripper, which holds zero torque and makes each motor report state, and ticks
to receive the replies. It runs until Ctrl-C, then disables on exit.

``--robot`` defaults to ``openarm`` for compatibility with the original
example; use ``--robot openyam`` for the six-joint layout. Runs against
MockCanBus (``--mock``, no hardware) or SocketCAN.

Usage:
    python examples/06_read_robot_motor.py --mock
    python examples/06_read_robot_motor.py --mock --robot openyam
    python examples/06_read_robot_motor.py --interface can0 --robot openarm
    python examples/06_read_robot_motor.py --interface canfd0 --fd --robot openyam
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


ARM_MOTORS = [
    ("j1", "DM8009", 0x01, 0x11),
    ("j2", "DM8009", 0x02, 0x12),
    ("j3", "DM4340", 0x03, 0x13),
    ("j4", "DM4340", 0x04, 0x14),
    ("j5", "DM4310", 0x05, 0x15),
    ("j6", "DM4310", 0x06, 0x16),
    ("j7", "DM4310", 0x07, 0x17),
]
OPENYAM_ARM_MOTORS = [
    ("j1", "DM4340", 0x01, 0x11),
    ("j2", "DM4340", 0x02, 0x12),
    ("j3", "DM4340", 0x03, 0x13),
    ("j4", "DM4310", 0x04, 0x14),
    ("j5", "DM4310", 0x05, 0x15),
    ("j6", "DM4310", 0x06, 0x16),
]
GRIPPER = ("grip", "DM4310", 0x08, 0x18)
ROBOT_LAYOUTS = {
    "openarm": ARM_MOTORS,
    "openyam": OPENYAM_ARM_MOTORS,
}


def _resolve(MotorType, name: str, type_str: str):
    mt = getattr(MotorType, type_str, None)
    if mt is None:
        raise SystemExit(f"unknown motor type {type_str!r} for {name} (e.g. DM4310, DM4340, DM8009)")
    return mt


def build_robot(args: argparse.Namespace):
    import can_motor_control
    from _native_transport import open_native_transport
    from can_motor_control.damiao import DamiaoCodec, MotorType

    arm_layout = ROBOT_LAYOUTS[args.robot]
    transport = (
        can_motor_control.MockCanBus.new_fd("vcan_mock") if args.mock and args.fd
        else can_motor_control.MockCanBus("vcan_mock") if args.mock
        else open_native_transport(args.interface, fd=args.fd)
    )
    arm_specs = [
        can_motor_control.MotorSpec(name, _resolve(MotorType, name, type_str), send_id, recv_id)
        for name, type_str, send_id, recv_id in arm_layout
    ]
    g_name, g_type, g_send, g_recv = GRIPPER
    grip_spec = can_motor_control.MotorSpec(g_name, _resolve(MotorType, g_name, g_type), g_send, g_recv)
    return (
        can_motor_control.RobotBuilder()
        .add_bus("main", transport, DamiaoCodec())
        .add_arm("arm", bus="main", motors=arm_specs)
        .add_gripper("grip", bus="main", motor=grip_spec)
        .build()
    )


def _add_motor_row(table, group, name, type_str, m, pos, vel, tau):
    enabled = Text("●", style="green") if m.is_enabled else Text("○", style="red")
    fault_cell = Text("none", style="dim") if m.fault is None else Text(str(m.fault), style="bold red")
    table.add_row(group, name, type_str, f"{pos:+.5f}", f"{vel:+.5f}", f"{tau:+.4f}",
                  str(m.temperature_mos), str(m.temperature_rotor), enabled, fault_cell)


def render(meta, arm, arm_motors, gripper, gripper_type, ticks, elapsed, rate):
    head = Table.grid(padding=(0, 2))
    head.add_column(style="bold cyan", justify="right")
    head.add_column()
    head.add_row("target", meta["target"])
    head.add_row("wire", meta["wire"])
    head.add_row("groups", f"arm ({len(arm_motors)}) + gripper (1)")
    state = Table(box=box.SIMPLE_HEAVY, expand=True, header_style="bold")
    for column, style, justify in [("group", "dim", None), ("motor", "bold cyan", None),
                                    ("type", "cyan", None), ("pos (rad)", "bright_white", "right"),
                                    ("vel (rad/s)", "bright_white", "right"), ("τ (Nm)", "bright_white", "right"),
                                    ("mos", None, "right"), ("rotor", None, "right"), ("en", None, "center"), ("fault", None, "left")]:
        state.add_column(column, style=style, justify=justify, no_wrap=True)
    pos, vel, tau = arm.positions(), arm.velocities(), arm.torques()
    for i, (name, type_str) in enumerate(arm_motors):
        _add_motor_row(state, "arm", name, type_str, arm[name], pos[i], vel[i], tau[i])
    gm = gripper.motor
    _add_motor_row(state, "gripper", gm.name, gripper_type, gm, gm.position, gm.velocity, gm.torque)
    footer = Text.assemble(("ticks ", "dim"), (f"{ticks:>7}", "bold"), ("   elapsed ", "dim"),
                           (f"{elapsed:6.1f}s", "bold"), ("   rate ", "dim"), (f"{rate:5.0f} Hz", "bold"),
                           ("    Press ", "dim"), ("Ctrl-C", "bold yellow"), (" to stop", "dim"))
    return Panel(Group(head, state, footer), title=f"can_motor_control · {meta['robot']} reader",
                 border_style="cyan", padding=(1, 2))


def run_live(args, robot, meta, console):
    refresh_hz = max(1.0, args.hz)
    render_period, loop_period = 1.0 / refresh_hz, 1e-3
    arm, gripper = robot["arm"], robot["grip"]
    arm_motors = [(name, type_str) for name, type_str, _, _ in ROBOT_LAYOUTS[args.robot]]
    hold = np.zeros((len(arm_motors), 5), dtype=np.float64)
    start = time.monotonic(); last_render = -render_period; window_start = start
    window_ticks = ticks = 0; rate = 0.0
    with Live(console=console, screen=True, refresh_per_second=refresh_hz) as live:
        while args.seconds is None or time.monotonic() - start < args.seconds:
            t0 = time.monotonic()
            arm.mit_control(hold); gripper.mit_control(0.0, 0.0, 0.0, 0.0, 0.0); robot.tick(args.deadline_us)
            ticks += 1; window_ticks += 1; elapsed = t0 - start
            if elapsed - last_render >= render_period:
                dt = t0 - window_start
                if dt > 0: rate = window_ticks / dt
                window_start, window_ticks, last_render = t0, 0, elapsed
                live.update(render(meta, arm, arm_motors, gripper, GRIPPER[1], ticks, elapsed, rate))
            rem = loop_period - (time.monotonic() - t0)
            if rem > 0: time.sleep(rem)
    return ticks


def main() -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    parser.add_argument("--robot", choices=tuple(ROBOT_LAYOUTS), default="openarm",
                        help="robot layout (default: openarm; openyam omits J7)")
    parser.add_argument("--mock", action="store_true", help="use MockCanBus (no hardware)")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument("--fd", action="store_true", help="open the bus in CAN-FD mode")
    parser.add_argument("--seconds", type=float, default=None, help="optional run cap in seconds; omit to run until Ctrl-C")
    parser.add_argument("--hz", type=float, default=20.0, help="TUI refresh rate in Hz (default: 20.0)")
    parser.add_argument("--deadline-us", type=int, default=500, help="per-tick bus deadline us (default: 500)")
    args = parser.parse_args()
    if args.seconds is not None and args.seconds <= 0: parser.error(f"--seconds must be > 0, got {args.seconds}")
    console = Console()
    meta = {"robot": "OpenArm" if args.robot == "openarm" else "OpenYam",
            "target": "MockCanBus" if args.mock else args.interface, "wire": "CAN-FD" if args.fd else "classical CAN"}
    robot = build_robot(args)
    console.print("connecting..."); robot.connect()
    try:
        arm, gripper = robot["arm"], robot["grip"]
        console.print("setting MIT mode..."); robot.set_mode("mit")
        console.print("enabling..."); robot.enable()
        arm.mit_control(np.zeros((len(ROBOT_LAYOUTS[args.robot]), 5), dtype=np.float64))
        gripper.mit_control(0.0, 0.0, 0.0, 0.0, 0.0)
        try: console.print(f"completed {run_live(args, robot, meta, console)} ticks")
        except KeyboardInterrupt: console.print("\n[yellow]interrupted[/] — stopping.")
    finally:
        console.print("disabling...")
        try: robot.disable()
        except Exception as exc: console.print(f"[red]WARNING:[/] disable failed: {exc}")
    console.print("done."); return 0


if __name__ == "__main__":
    raise SystemExit(main())
