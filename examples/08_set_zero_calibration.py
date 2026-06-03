"""Tier 4 -- set-zero calibration (DANGEROUS, persistent).

Sets the current physical position of every motor in a group as the
electrical zero. This change is persistent in the motor; if the arm is
at the wrong pose when you run this, the rest of your stack will see a
wrong absolute zero forever (until you set zero again at a known pose).

Sequence: disable -> set_zero_all -> disable. Disabling before and after
is intentional; the operator should not be running motion control during
calibration.

CURRENT API LIMITATIONS:

  * Python `Arm` exposes only `set_zero_all()` -- a group-wide operation.
    You cannot target a single joint inside an arm group without
    constructing a one-motor group; if you need that, build an Arm
    containing only the joint you want to zero (see `01_*` for shape).
  * `Gripper` does not currently expose set_zero. To zero a gripper
    joint, instantiate it as a one-motor Arm group via RobotBuilder.

CONFIRMATION:

  This example refuses to do anything without `--i-understand-this-writes-zero`.
  Running without that flag exits BEFORE constructing or enabling the robot,
  so you cannot trip it accidentally on the wrong arm.

Usage:
    # Print what would happen (no robot is built):
    python examples/08_set_zero_calibration.py --config configs/openarm_single.toml

    # Actually write zero on every motor in the configured arm group:
    python examples/08_set_zero_calibration.py \\
        --config configs/openarm_single.toml \\
        --i-understand-this-writes-zero
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _bringup_common import (  # noqa: E402
    add_fd_arg,
    add_interface_arg,
    add_single_motor_args,
    print_assumptions,
    resolve_motor_type,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(__doc__ or "").splitlines()[0]
    )
    parser.add_argument(
        "--config",
        default=None,
        help="TOML robot config; mutually exclusive with --single-motor",
    )
    parser.add_argument(
        "--group-name",
        default="arm",
        help="arm group name in the config (default: arm)",
    )
    parser.add_argument(
        "--single-motor",
        action="store_true",
        help="zero a single motor instead of a configured arm (uses --interface, --send-id, --recv-id, --motor-type)",
    )
    add_interface_arg(parser)
    add_fd_arg(parser)
    add_single_motor_args(parser)
    parser.add_argument(
        "--i-understand-this-writes-zero",
        action="store_true",
        dest="confirmed",
        help="REQUIRED to actually send the set-zero command. Without this, the example exits before building hardware.",
    )
    args = parser.parse_args()

    if args.config is None and not args.single_motor:
        parser.error("must pass either --config or --single-motor")
    if args.config is not None and args.single_motor:
        parser.error("--config and --single-motor are mutually exclusive")

    # Describe the target BEFORE doing anything.
    if args.single_motor:
        target_lines = [
            "target            : single motor",
            f"interface         : {args.interface}",
            f"send_id / recv_id : 0x{args.send_id:02X} / 0x{args.recv_id:02X}",
            f"motor_type        : {args.motor_type}",
        ]
    else:
        target_lines = [
            "target            : every motor in the configured arm group",
            f"config            : {args.config}",
            f"group name        : {args.group_name}",
        ]

    print_assumptions(
        "08 set-zero calibration",
        [
            *target_lines,
            "operation         : disable -> set_zero_all -> disable",
            "persistence       : PERSISTENT in motor flash; affects every future run",
            f"confirmed         : {args.confirmed}",
        ],
        # Direct single-motor path honors --fd; the config path is config-governed.
        fd=args.fd if args.single_motor else None,
    )

    if not args.confirmed:
        print(
            "\nDRY RUN: --i-understand-this-writes-zero was NOT passed.",
            "Exiting BEFORE constructing or enabling the robot.",
            "No CAN frames will be sent.",
            sep="\n",
        )
        return 0

    # Lazy imports kept inside the confirmation branch so dry-run does not
    # require the native wheel.
    import dm_control
    from dm_control.damiao import DamiaoCodec

    if args.single_motor:
        motor_type = resolve_motor_type(args.motor_type)
        transport = dm_control.SocketCanBus(args.interface, fd=args.fd)
        robot = (
            dm_control.RobotBuilder()
            .add_bus("main", transport, DamiaoCodec())
            .add_arm(
                "arm",
                bus="main",
                motors=[
                    dm_control.MotorSpec("j0", motor_type, args.send_id, args.recv_id),
                ],
            )
            .build()
        )
        group_name = "arm"
    else:
        robot = dm_control.Robot.from_config(args.config)
        group_name = args.group_name

    print("connecting...")
    robot.connect()
    try:
        arm = robot[group_name]
        print(f"target group has {len(arm)} motor(s)")
        # The sequence is: disable first so the motor is not actively
        # controlled while it accepts the zero write; set_zero; disable
        # again as a paranoid post-state.
        print("disabling before set_zero...")
        arm.disable_all()
        print("calling set_zero_all() -- PERSISTENT write to motor flash...")
        arm.set_zero_all()
        print("disabling after set_zero...")
        arm.disable_all()
    finally:
        try:
            robot.disable()
        except Exception as exc:  # noqa: BLE001
            print(f"WARNING: robot.disable() failed: {exc}", file=sys.stderr)
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
