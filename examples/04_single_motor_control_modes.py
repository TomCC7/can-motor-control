"""Tier 2 -- one control mode per run on a single motor.

Selects exactly one of the four control modes exposed by `Arm`:

    --mode mit         shape (n, 5)  kp, kd, q, dq, tau
    --mode pos-vel     shape (n, 2)  q, dq
    --mode vel         shape (n,)    dq
    --mode pos-force   shape (n, 3)  q, dq, i_pu

Defaults are zero or near-zero so the bus carries valid frames in the
chosen mode without actually commanding motion. Any non-zero motion value
must be supplied through a flag; this is intentional friction.

Run only after `03_single_motor_mit_hold.py` worked safely.

Usage:
    python examples/04_single_motor_control_modes.py --mode mit
    python examples/04_single_motor_control_modes.py --mode vel --dq 0.0
    python examples/04_single_motor_control_modes.py --mode pos-force --i-pu 0.0
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


MODES = ("mit", "pos-vel", "vel", "pos-force")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(__doc__ or "").splitlines()[0]
    )
    parser.add_argument(
        "--mode",
        choices=MODES,
        required=True,
        help="exactly one control mode per run",
    )
    add_interface_arg(parser)
    add_fd_arg(parser)
    add_single_motor_args(parser)
    add_bounded_run_args(parser, default_seconds=2.0)
    # Per-mode zero-default knobs; user must opt in to any non-zero magnitude.
    parser.add_argument("--kp", type=float, default=5.0, help="(mit) position gain")
    parser.add_argument("--kd", type=float, default=1.0, help="(mit) velocity gain")
    parser.add_argument("--q", type=float, default=0.0, help="(mit/pos-vel/pos-force) target position")
    parser.add_argument("--dq", type=float, default=0.0, help="(any) target velocity")
    parser.add_argument("--tau", type=float, default=0.0, help="(mit) feed-forward torque")
    parser.add_argument("--i-pu", type=float, default=0.0, help="(pos-force) current per-unit")
    args = parser.parse_args()
    validate_seconds(args, max_seconds=10.0)

    motor_type = resolve_motor_type(args.motor_type)

    if args.mode == "mit":
        cmd_desc = f"mit kp={args.kp} kd={args.kd} q={args.q} dq={args.dq} tau={args.tau}"
    elif args.mode == "pos-vel":
        cmd_desc = f"pos-vel q={args.q} dq={args.dq}"
    elif args.mode == "vel":
        cmd_desc = f"vel dq={args.dq}"
    else:  # pos-force
        cmd_desc = f"pos-force q={args.q} dq={args.dq} i_pu={args.i_pu}"

    print_assumptions(
        f"04 single motor control mode = {args.mode}",
        [
            f"interface         : {args.interface}",
            f"send_id / recv_id : 0x{args.send_id:02X} / 0x{args.recv_id:02X}",
            f"motor_type        : {args.motor_type}",
            f"duration          : {args.seconds:.2f} s (hard cap 10 s)",
            f"command           : {cmd_desc}",
            "safety            : non-zero motion values are opt-in via flags",
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

    # Build the right ndarray shape for the chosen mode. The Python `Arm`
    # API requires:
    #   mit       (n, 5)   pos-vel  (n, 2)   vel  (n,)   pos-force (n, 3)
    if args.mode == "mit":
        cmds = np.array([[args.kp, args.kd, args.q, args.dq, args.tau]], dtype=np.float64)
        send = lambda arm: arm.mit_control(cmds)
    elif args.mode == "pos-vel":
        cmds = np.array([[args.q, args.dq]], dtype=np.float64)
        send = lambda arm: arm.pos_vel_control(cmds)
    elif args.mode == "vel":
        cmds = np.array([args.dq], dtype=np.float64)
        send = lambda arm: arm.vel_control(cmds)
    else:  # pos-force
        cmds = np.array([[args.q, args.dq, args.i_pu]], dtype=np.float64)
        send = lambda arm: arm.pos_force_control(cmds)

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
            send(arm)
            ticks += 1
            rem = period - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)
        m = arm["j0"]
        print(
            f"completed {ticks} ticks in mode {args.mode}; final "
            f"pos={m.position:+.5f} vel={m.velocity:+.5f} tau={m.torque:+.5f}"
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
