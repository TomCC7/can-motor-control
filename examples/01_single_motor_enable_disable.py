"""Tier 1 -- enable/disable only, no motion commands.

Build a one-motor robot from explicit CAN IDs, connect, enable, tick a few
times so the host sees a state reply, then disable. This is the first
example that talks to the bus, and it deliberately sends nothing other
than the enable and disable frames the lifecycle emits.

Run only after `00_can_interface_check.py` confirms the interface and
`candump <iface>` shows a quiet bus.

Usage:
    python examples/01_single_motor_enable_disable.py \\
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
    add_bounded_run_args(parser, default_seconds=1.0)
    args = parser.parse_args()
    validate_seconds(args, max_seconds=10.0)

    motor_type = resolve_motor_type(args.motor_type)

    print_assumptions(
        "01 single motor enable / disable",
        [
            f"interface         : {args.interface}",
            f"send_id / recv_id : 0x{args.send_id:02X} / 0x{args.recv_id:02X}",
            f"motor_type        : {args.motor_type}",
            f"duration          : {args.seconds:.2f} s of ticks after enable",
            "sends             : enable + a few state-read ticks + disable",
            "motion commands   : NONE (no mit/pos/vel/force frames)",
            "safety            : keep clear of the actuator; powered hardware",
        ],
        fd=args.fd,
    )

    # Lazy imports so `--help` works even if the native wheel is missing.
    import can_motor_control
    from _native_transport import open_native_transport
    from can_motor_control.damiao import DamiaoCodec

    transport = open_native_transport(args.interface, fd=args.fd)
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
        deadline = time.monotonic() + args.seconds
        ticks = 0
        while time.monotonic() < deadline:
            robot.tick(args.deadline_us)
            ticks += 1
            time.sleep(1e-3)
        motor = robot["arm"]["j0"]
        print(f"ticked {ticks} times")
        print(
            f"motor j0: enabled={motor.is_enabled} pos={motor.position:.4f} "
            f"vel={motor.velocity:.4f} tau={motor.torque:.4f}"
        )
    finally:
        # Disable regardless of how we got here: enable succeeded, tick blew up,
        # KeyboardInterrupt, anything. This is the disable-on-exit guarantee.
        print("disabling...")
        try:
            robot.disable()
        except Exception as exc:  # noqa: BLE001 -- best effort during teardown
            print(f"WARNING: disable failed: {exc}", file=sys.stderr)
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
