"""Tier 1 -- read state, no motion commands.

Enable one motor, then each cycle send a state-refresh query
(`arm.refresh()` -- the Damiao `refresh_motor_status` poll, which commands no
motion) and run `tick()` to receive the reply, print every state field, and
disable. Refresh is required because a Damiao motor only reports state in reply
to a frame we send; a tick-only loop would freeze after the enable ack. This
example contains NO calls to `mit_control`, `pos_vel_control`, `vel_control`,
or `pos_force_control`; grep for those names and you should find no hits.

Run only after `01_single_motor_enable_disable.py` succeeds on the same
motor.

Usage:
    python examples/02_single_motor_read_state.py \\
        --interface can0 --send-id 0x01 --recv-id 0x11 --motor-type DM4340
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


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(__doc__ or "").splitlines()[0]
    )
    add_interface_arg(parser)
    add_fd_arg(parser)
    add_single_motor_args(parser)
    add_bounded_run_args(parser, default_seconds=2.0)
    parser.add_argument(
        "--print-every",
        type=int,
        default=100,
        help="print state every N ticks (default: 100)",
    )
    args = parser.parse_args()
    validate_seconds(args, max_seconds=30.0)

    motor_type = resolve_motor_type(args.motor_type)

    print_assumptions(
        "02 single motor read state",
        [
            f"interface         : {args.interface}",
            f"send_id / recv_id : 0x{args.send_id:02X} / 0x{args.recv_id:02X}",
            f"motor_type        : {args.motor_type}",
            f"duration          : {args.seconds:.2f} s",
            f"print cadence     : every {args.print_every} ticks",
            "sends             : enable + read-only ticks + disable",
            "motion commands   : NONE",
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
            # Query state (no motion), then tick to receive the reply. A Damiao
            # motor only reports state in response to a frame we send.
            arm.refresh()
            robot.tick(args.deadline_us)
            ticks += 1
            if ticks % args.print_every == 0:
                m = arm["j0"]
                print(
                    f"tick={ticks:>6} pos={m.position:+.5f} vel={m.velocity:+.5f} "
                    f"tau={m.torque:+.5f} t_mos={m.temperature_mos} "
                    f"t_rotor={m.temperature_rotor} fault={m.fault}"
                )
            rem = period - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)
        print(f"completed {ticks} ticks")
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
