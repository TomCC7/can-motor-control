"""Tier 2 -- first motion: conservative MIT hold at zero.

Run a bounded MIT control loop on a single motor with low default gains
and zero setpoint. The motor should hold its current position softly. The
duration is hard-capped so a forgotten loop cannot run indefinitely.

Run only after `02_read_single_motor.py` shows sane position /
velocity / temperature for this motor.

Defaults are intentionally conservative:
  * kp =  5.0   -- soft position stiffness
  * kd =  1.0   -- modest damping
  * q  =  0.0   -- target position (zero)
  * dq =  0.0   -- target velocity (zero)
  * tau=  0.0   -- feed-forward torque (none)
  * --seconds default 2 s, hard cap 10 s

Usage:
    python examples/03_single_motor_mit_hold.py \\
        --interface can0 --send-id 0x01 --recv-id 0x11 --motor-type DM4340
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _bringup_common import (  # noqa: E402
    add_bounded_run_args,
    add_fd_arg,
    add_interface_arg,
    add_single_motor_args,
    print_assumptions,
    resolve_motor_type,
    validate_seconds,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(__doc__ or "").splitlines()[0]
    )
    add_interface_arg(parser)
    add_fd_arg(parser)
    add_single_motor_args(parser)
    add_bounded_run_args(parser, default_seconds=2.0)
    # MIT gains/setpoint -- user must opt in to anything non-trivial.
    parser.add_argument("--kp", type=float, default=5.0, help="position gain (default: 5.0)")
    parser.add_argument("--kd", type=float, default=1.0, help="velocity gain (default: 1.0)")
    parser.add_argument("--q", type=float, default=0.0, help="target position rad (default: 0.0)")
    parser.add_argument("--dq", type=float, default=0.0, help="target velocity rad/s (default: 0.0)")
    parser.add_argument("--tau", type=float, default=0.0, help="feed-forward torque Nm (default: 0.0)")
    args = parser.parse_args()
    validate_seconds(args, max_seconds=10.0)

    motor_type = resolve_motor_type(args.motor_type)

    print_assumptions(
        "03 single motor MIT hold",
        [
            f"interface         : {args.interface}",
            f"send_id / recv_id : 0x{args.send_id:02X} / 0x{args.recv_id:02X}",
            f"motor_type        : {args.motor_type}",
            f"duration          : {args.seconds:.2f} s (hard cap 10 s)",
            f"mit gains         : kp={args.kp} kd={args.kd}",
            f"mit setpoint      : q={args.q} dq={args.dq} tau={args.tau}",
            "safety            : motor will hold softly; clear workspace around it",
        ],
        fd=args.fd,
    )

    import can_motor_control
    from can_motor_control.damiao import DamiaoCodec

    transport = can_motor_control.SocketCanBus(args.interface, fd=args.fd)
    robot = (
        can_motor_control.RobotBuilder()
        .add_bus("main", transport, DamiaoCodec())
        .add_arm(
            "arm",
            bus="main",
            motors=[
                can_motor_control.MotorSpec("j0", motor_type, args.send_id, args.recv_id),
            ],
        )
        .build()
    )

    cmds = np.array(
        [[args.kp, args.kd, args.q, args.dq, args.tau]], dtype=np.float64
    )  # shape (1, 5) -- one motor

    print("connecting...")
    robot.connect()
    try:
        print("enabling...")
        robot.enable()
        arm = robot["arm"]
        deadline = time.monotonic() + args.seconds
        ticks = 0
        period = 1e-3
        while time.monotonic() < deadline:
            t0 = time.monotonic()
            robot.tick(args.deadline_us)
            arm.mit_control(cmds)
            ticks += 1
            rem = period - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)
        m = arm["j0"]
        print(
            f"completed {ticks} ticks; final pos={m.position:+.5f} "
            f"vel={m.velocity:+.5f} tau={m.torque:+.5f}"
        )
    finally:
        print("disabling...")
        try:
            robot.disable()
        except Exception as exc:  # noqa: BLE001
            print(f"WARNING: disable failed: {exc}", file=sys.stderr)
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
