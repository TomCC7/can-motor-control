"""Tier 3 -- gripper scene (MIT or PosVel only).

The current public Python `Gripper` exposes:
    enable()                   -- power on
    disable()                  -- power off
    mit_control(kp,kd,q,dq,tau)
    pos_vel_control(q,dq)

This example uses ONLY those. It does not call `open`, `close`, or
`pos_force_control` -- those are not part of the current public API.
If you want them, propose them in a separate change.

Run only after the gripper joint individually passed `01` and `02`.

Usage:
    # MIT hold at zero (default, low gains):
    python examples/07_gripper_control.py --interface can0 \\
        --send-id 0x08 --recv-id 0x18 --motor-type DM4310

    # PosVel: move toward q (radians), capped velocity:
    python examples/07_gripper_control.py --mode pos-vel --q 0.0 --dq 0.0 \\
        --interface can0 --send-id 0x08 --recv-id 0x18 --motor-type DM4310
"""

from __future__ import annotations

import argparse
import os
import sys
import time

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


MODES = ("mit", "pos-vel")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(__doc__ or "").splitlines()[0]
    )
    parser.add_argument(
        "--mode",
        choices=MODES,
        default="mit",
        help="gripper control mode (default: mit; PosForce is intentionally NOT exposed)",
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
    parser.add_argument("--kp", type=float, default=2.0, help="(mit) position gain (default: 2.0)")
    parser.add_argument("--kd", type=float, default=0.5, help="(mit) velocity gain (default: 0.5)")
    parser.add_argument("--q", type=float, default=0.0, help="target position rad (default: 0.0)")
    parser.add_argument("--dq", type=float, default=0.0, help="target velocity rad/s (default: 0.0)")
    parser.add_argument("--tau", type=float, default=0.0, help="(mit) feed-forward torque (default: 0.0)")
    args = parser.parse_args()
    validate_seconds(args, max_seconds=10.0)

    motor_type = resolve_motor_type(args.motor_type)

    if args.mode == "mit":
        cmd_desc = f"mit kp={args.kp} kd={args.kd} q={args.q} dq={args.dq} tau={args.tau}"
    else:
        cmd_desc = f"pos-vel q={args.q} dq={args.dq}"

    print_assumptions(
        f"07 gripper control ({args.mode})",
        [
            f"interface         : {args.interface}",
            f"send_id / recv_id : 0x{args.send_id:02X} / 0x{args.recv_id:02X}",
            f"motor_type        : {args.motor_type}",
            f"duration          : {args.seconds:.2f} s (hard cap 10 s)",
            f"command           : {cmd_desc}",
            "safety            : verify nothing fragile is between the gripper jaws",
        ],
        fd=args.fd,
    )

    import can_motor_control
    from can_motor_control.damiao import DamiaoCodec

    transport = can_motor_control.SocketCanBus(args.interface, fd=args.fd)
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
        gripper = robot["grip"]
        print("enabling gripper...")
        gripper.enable()
        deadline = time.monotonic() + args.seconds
        period = 1e-3
        ticks = 0
        while time.monotonic() < deadline:
            t0 = time.monotonic()
            robot.tick(args.deadline_us)
            if args.mode == "mit":
                gripper.mit_control(args.kp, args.kd, args.q, args.dq, args.tau)
            else:
                gripper.pos_vel_control(args.q, args.dq)
            ticks += 1
            rem = period - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)
        m = gripper.motor
        print(
            f"completed {ticks} ticks; gripper "
            f"pos={m.position:+.5f} vel={m.velocity:+.5f} tau={m.torque:+.5f}"
        )
    finally:
        print("disabling gripper...")
        # Disable via the gripper handle while we still hold it (idempotent
        # with robot.disable() below).
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
