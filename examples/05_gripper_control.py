"""Tier 3 -- gripper PosForce sequence, with MIT and PosVel alternatives.

Default behavior is a conservative PosForce open/close demo: set the gripper to
PosForce mode, then play `-π/4 -> 0 -> -π/4 -> 0`. PosForce and PosVel use
`dq=25.0 rad/s`; PosForce also uses `i_pu=0.15`. The same sequence is used for
PosForce, MIT, and PosVel; pass `--positions` or `--q` to play a custom target
in any mode.

Run only after the gripper joint individually passed `01` and `02`.

Usage:
    python examples/05_gripper_control.py --mock --seconds 0.1

    python examples/05_gripper_control.py --interface can0 \
        --send-id 0x08 --recv-id 0x18 --motor-type DM4310

    python examples/05_gripper_control.py --mode mit --q 0.0 \
        --interface can0 --send-id 0x08 --recv-id 0x18 --motor-type DM4310
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Protocol, cast

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from examples._bringup_common import (
    add_bounded_run_args,
    add_fd_arg,
    add_interface_arg,
    add_single_motor_args,
    print_assumptions,
    resolve_motor_type,
    validate_seconds,
)


class MotorState(Protocol):
    position: float
    velocity: float
    torque: float
    temperature_mos: int
    temperature_rotor: int


class GripperHandle(Protocol):
    @property
    def motor(self) -> MotorState: ...
    def set_mode(self, mode: str) -> None: ...
    def enable(self) -> None: ...
    def disable(self) -> None: ...
    def mit_control(self, kp: float, kd: float, q: float, dq: float, tau: float) -> None: ...
    def pos_vel_control(self, q: float, dq: float) -> None: ...
    def pos_force_control(self, q: float, dq: float, i_pu: float) -> None: ...


class RobotHandle(Protocol):
    def tick(self, per_bus_deadline_us: int) -> None: ...
    def disable(self) -> None: ...


MODES = ("pos-force", "mit", "pos-vel")
MODE_FOR_API = {"pos-force": "pos_force", "pos-vel": "pos_vel", "mit": "mit"}
OPENARM_POSITIONS = (-math.pi / 4.0, 0.0, -math.pi / 4.0, 0.0)
MODE_LATCH_SECONDS = 0.1


def parse_position_token(token: str) -> float:
    normalized = token.strip().lower().replace("π", "pi")
    if normalized == "pi":
        return math.pi
    if normalized == "-pi":
        return -math.pi
    if normalized == "pi/2":
        return math.pi / 2.0
    if normalized == "-pi/2":
        return -math.pi / 2.0
    return float(normalized)


def parse_positions(text: str) -> tuple[float, ...]:
    positions = tuple(
        parse_position_token(part)
        for part in text.split(",")
        if part.strip()
    )
    if not positions:
        raise argparse.ArgumentTypeError("--positions must include at least one value")
    return positions


def command_description(mode: str, positions: Sequence[float], args: argparse.Namespace) -> str:
    q_values = ", ".join(f"{q:.5g}" for q in positions)
    if mode == "mit":
        return f"mit q=[{q_values}] kp={args.kp} kd={args.kd} dq={args.dq} tau={args.tau}"
    if mode == "pos-vel":
        return f"pos-vel q=[{q_values}] dq={args.dq}"
    return f"pos-force q=[{q_values}] dq={args.dq} i_pu={args.i_pu}"


def send_command(gripper: GripperHandle, mode: str, q: float, args: argparse.Namespace) -> None:
    if mode == "mit":
        gripper.mit_control(args.kp, args.kd, q, args.dq, args.tau)
    elif mode == "pos-vel":
        gripper.pos_vel_control(q, args.dq)
    else:
        gripper.pos_force_control(q, args.dq, args.i_pu)


def run_position(robot: RobotHandle, gripper: GripperHandle, mode: str, q: float, args: argparse.Namespace) -> int:
    deadline = time.monotonic() + cast(float, args.seconds)
    period = 1e-3
    ticks = 0
    while time.monotonic() < deadline:
        tick_started = time.monotonic()
        robot.tick(args.deadline_us)
        send_command(gripper, mode, q, args)
        ticks += 1
        remaining = period - (time.monotonic() - tick_started)
        if remaining > 0:
            time.sleep(remaining)
    return ticks


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(__doc__ or "").splitlines()[0]
    )
    parser.add_argument(
        "--mode",
        choices=MODES,
        default="pos-force",
        help="gripper control mode (default: pos-force)",
    )
    parser.add_argument("--mock", action="store_true", help="use MockCanBus (no hardware)")
    add_interface_arg(parser)
    add_fd_arg(parser)
    add_single_motor_args(
        parser,
        default_send_id=0x08,
        default_recv_id=0x18,
        default_motor_type="DM4310",
    )
    add_bounded_run_args(parser, default_seconds=2.0)
    parser.add_argument("--kp", type=float, default=2.0, help="(mit) position gain (default: 2.0)")
    parser.add_argument("--kd", type=float, default=0.5, help="(mit) velocity gain (default: 0.5)")
    parser.add_argument("--q", type=float, default=None, help="single target position rad, used instead of the default sequence")
    parser.add_argument("--dq", type=float, default=None, help="target velocity rad/s (default: 25.0 for pos-force/pos-vel, 0.0 for mit)")
    parser.add_argument("--tau", type=float, default=0.0, help="(mit) feed-forward torque (default: 0.0)")
    parser.add_argument("--i-pu", type=float, default=0.15, help="(pos-force) current per-unit (default: 0.15)")
    parser.add_argument(
        "--positions",
        type=parse_positions,
        default=None,
        help="comma-separated q sequence, accepts pi, pi/2, and pi/4 (default: -pi/4,0,-pi/4,0)",
    )
    args = parser.parse_args()
    validate_seconds(args, max_seconds=10.0)

    if args.kp < 0.0 or args.kd < 0.0:
        raise SystemExit("--kp and --kd must be non-negative")
    if args.dq is None:
        args.dq = 25.0 if args.mode in {"pos-force", "pos-vel"} else 0.0

    positions = cast(tuple[float, ...] | None, args.positions)
    if positions is None:
        positions = OPENARM_POSITIONS if args.q is None else (cast(float, args.q),)

    motor_type = resolve_motor_type(args.motor_type)
    command_desc = command_description(args.mode, positions, args)

    print_assumptions(
        f"05 gripper control ({args.mode})",
        [
            f"interface         : {args.interface}",
            f"target            : {'MockCanBus' if args.mock else args.interface}",
            f"send_id / recv_id : 0x{args.send_id:02X} / 0x{args.recv_id:02X}",
            f"motor_type        : {args.motor_type}",
            f"duration          : {args.seconds:.2f} s per position (hard cap 10 s)",
            f"mode setup        : set_mode({MODE_FOR_API[args.mode]!r}) before enable",
            f"command           : {command_desc}",
            "safety            : clear the gripper jaws before sending commands",
        ],
        fd=args.fd,
    )

    import can_motor_control
    from _native_transport import open_native_transport
    from can_motor_control.damiao import DamiaoCodec

    transport = (
        can_motor_control.MockCanBus.new_fd("vcan_mock")
        if args.mock and args.fd
        else can_motor_control.MockCanBus("vcan_mock")
        if args.mock
        else open_native_transport(args.interface, fd=args.fd)
    )
    robot = (
        can_motor_control.RobotBuilder()
        .add_bus("main", transport, DamiaoCodec())
        .add_gripper(
            "grip",
            bus="main",
            motor=can_motor_control.MotorSpec("g", motor_type, args.send_id, args.recv_id),
        )
        .build()
    )

    print("connecting...")
    robot.connect()
    gripper = None
    try:
        gripper = cast(GripperHandle, robot["grip"])
        print(f"setting gripper mode to {MODE_FOR_API[args.mode]}...")
        gripper.set_mode(MODE_FOR_API[args.mode])
        time.sleep(MODE_LATCH_SECONDS)
        print("enabling gripper...")
        gripper.enable()
        ticks = 0
        for index, q in enumerate(positions):
            print(f"command[{index}] q={q:.5g}...")
            ticks += run_position(robot, gripper, args.mode, q, args)
        m = gripper.motor
        print(
            f"completed {ticks} ticks; gripper "
            f"pos={m.position:+.5f} vel={m.velocity:+.5f} tau={m.torque:+.5f} "
            f"t_mos={m.temperature_mos} t_rotor={m.temperature_rotor}"
        )
    finally:
        print("disabling gripper...")
        if gripper is not None:
            try:
                gripper.disable()
            except Exception:  # noqa: BLE001
                pass
        try:
            robot.disable()
        except Exception as exc:  # noqa: BLE001
            print(f"WARNING: disable failed: {exc}", file=sys.stderr)
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
