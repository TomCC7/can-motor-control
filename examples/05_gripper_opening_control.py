"""Tier 3 -- normalized gripper opening control.

This example demonstrates the formal gripper API: opening `0.0` is fully closed
and opening `1.0` is fully open. Entering the robot context connects, enables,
and calibrates gripper opening before user code commands `open`, `set_opening`,
and `close`.

Usage:
    python examples/05_gripper_opening_control.py --mock \
        --i-understand-this-moves-the-gripper --seconds 0.1

    python examples/05_gripper_opening_control.py --interface can0 \
        --send-id 0x08 --recv-id 0x18 --motor-type DM4310 \
        --opening-direction increasing_position \
        --i-understand-this-moves-the-gripper
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

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


def tick_for(robot, seconds: float, deadline_us: int) -> int:
    deadline = time.monotonic() + seconds
    ticks = 0
    while time.monotonic() < deadline:
        robot.tick(deadline_us)
        ticks += 1
        time.sleep(0.001)
    return ticks


def main() -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    parser.add_argument("--mock", action="store_true", help="use MockCanBus (no hardware)")
    parser.add_argument(
        "--opening-direction",
        choices=("increasing_position", "decreasing_position"),
        default="increasing_position",
        help="whether increasing raw motor position opens the gripper",
    )
    parser.add_argument(
        "--default-current",
        type=float,
        default=0.15,
        help="default opening current in per-unit motor current (default: 0.15)",
    )
    parser.add_argument(
        "--current",
        type=float,
        default=None,
        help="optional per-command current override for the midpoint command",
    )
    parser.add_argument(
        "--i-understand-this-moves-the-gripper",
        action="store_true",
        help="required safety acknowledgement",
    )
    add_interface_arg(parser)
    add_fd_arg(parser)
    add_single_motor_args(
        parser,
        default_send_id=0x08,
        default_recv_id=0x18,
        default_motor_type="DM4310",
    )
    add_bounded_run_args(parser, default_seconds=2.0)
    args = parser.parse_args()
    validate_seconds(args, max_seconds=10.0)

    if not args.i_understand_this_moves_the_gripper:
        raise SystemExit("Pass --i-understand-this-moves-the-gripper to run this example.")
    if args.default_current <= 0.0 or args.default_current > 1.0:
        raise SystemExit("--default-current must be > 0.0 and <= 1.0")
    if args.current is not None and (args.current <= 0.0 or args.current > 1.0):
        raise SystemExit("--current must be > 0.0 and <= 1.0")

    motor_type = resolve_motor_type(args.motor_type)
    print_assumptions(
        "05 gripper normalized opening control",
        [
            f"interface          : {args.interface}",
            f"target             : {'MockCanBus' if args.mock else args.interface}",
            f"send_id / recv_id  : 0x{args.send_id:02X} / 0x{args.recv_id:02X}",
            f"motor_type         : {args.motor_type}",
            f"opening_direction  : {args.opening_direction}",
            f"default_current    : {args.default_current}",
            f"duration           : {args.seconds:.2f} s per opening (hard cap 10 s)",
            "lifecycle          : connect -> enable -> automatic opening calibration -> commands",
            "commands           : open() -> set_opening(0.5) -> close()",
            "safety             : clear the gripper jaws before sending commands",
        ],
        fd=args.fd,
    )

    import can_motor_control
    from can_motor_control.damiao import DamiaoCodec

    transport = (
        can_motor_control.MockCanBus.new_fd("vcan_mock")
        if args.mock and args.fd
        else can_motor_control.MockCanBus("vcan_mock")
        if args.mock
        else can_motor_control.SocketCanBus(args.interface, fd=args.fd)
    )
    codec = can_motor_control.MockFeedbackCodec(args.recv_id) if args.mock else DamiaoCodec()
    robot = (
        can_motor_control.RobotBuilder()
        .add_bus("main", transport, codec)
        .add_gripper(
            "grip",
            bus="main",
            motor=can_motor_control.MotorSpec("g", motor_type, args.send_id, args.recv_id),
            opening_direction=args.opening_direction,
            default_current=args.default_current,
        )
        .build()
    )

    with robot:
        gripper = robot["grip"]
        print("commanding open()...")
        gripper.open()
        ticks = tick_for(robot, args.seconds, args.deadline_us)

        print("commanding set_opening(0.5)...")
        gripper.set_opening(0.5, current=args.current)
        ticks += tick_for(robot, args.seconds, args.deadline_us)

        print("commanding close()...")
        gripper.close()
        ticks += tick_for(robot, args.seconds, args.deadline_us)

        motor = gripper.motor
        print(
            f"completed {ticks} ticks; gripper "
            f"pos={motor.position:+.5f} vel={motor.velocity:+.5f} tau={motor.torque:+.5f} "
            f"t_mos={motor.temperature_mos} t_rotor={motor.temperature_rotor}"
        )
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
