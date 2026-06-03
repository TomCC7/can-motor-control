"""Tier 3 -- arm-wide state monitor, no motion commands.

Load a multi-motor arm from either a TOML config (e.g.
`configs/openarm_single.toml`) or from explicit `--motors` IDs. Enable
the arm, then each cycle refresh-poll state (`arm.refresh()`, no motion) and
tick for a bounded time, print per-joint state, and disable.

This is the multi-motor analog of `02_single_motor_read_state.py`. It
contains NO motion-command calls (refresh is a state query, not motion).

Run only after every joint in the arm has individually passed `01` and
`02`. Bringing up a full arm before single-motor bring-up is the most
common cause of confusing "which joint is broken?" debugging.

Usage:
    # From the TOML config:
    python examples/05_single_arm_state_monitor.py --config configs/openarm_single.toml

    # Or from explicit IDs (one --motor flag per joint):
    python examples/05_single_arm_state_monitor.py --interface can0 \\
        --motor 0x01,0x11,DM4340 --motor 0x02,0x12,DM4340 \\
        --motor 0x03,0x13,DM4340 --motor 0x04,0x14,DM4340
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
    parse_can_id,
    print_assumptions,
    resolve_motor_type,
    validate_seconds,
)


def parse_motor_triple(text: str) -> tuple[int, int, str]:
    """Parse `send_id,recv_id,motor_type` (comma-separated) from --motor."""
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"--motor expects send_id,recv_id,motor_type, got {text!r}"
        )
    return parse_can_id(parts[0]), parse_can_id(parts[1]), parts[2]


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(__doc__ or "").splitlines()[0]
    )
    add_interface_arg(parser)
    add_fd_arg(parser)
    add_bounded_run_args(parser, default_seconds=2.0)
    parser.add_argument(
        "--config",
        default=None,
        help="path to TOML robot config; mutually exclusive with --motor",
    )
    parser.add_argument(
        "--motor",
        action="append",
        type=parse_motor_triple,
        default=None,
        help="send_id,recv_id,motor_type (repeatable). Ignored if --config is set.",
    )
    parser.add_argument(
        "--group-name",
        default="arm",
        help="group name to read when --config is used (default: arm)",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=100,
        help="print arm state every N ticks (default: 100)",
    )
    args = parser.parse_args()
    validate_seconds(args, max_seconds=30.0)

    if args.config is None and not args.motor:
        parser.error("must pass either --config or one or more --motor flags")
    if args.config is not None and args.motor:
        parser.error("--config and --motor are mutually exclusive")

    import dm_control
    from dm_control.damiao import DamiaoCodec

    if args.config is not None:
        print_assumptions(
            "05 single arm state monitor (from config)",
            [
                f"config            : {args.config}",
                f"group name        : {args.group_name}",
                f"duration          : {args.seconds:.2f} s",
                "sends             : enable + read-only ticks + disable",
                "motion commands   : NONE",
            ],
            fd=None,  # wire format governed by the config file
        )
        robot = dm_control.Robot.from_config(args.config)
        group_name = args.group_name
    else:
        motor_descs = [
            f"  send=0x{s:02X} recv=0x{r:02X} type={t}" for (s, r, t) in args.motor
        ]
        print_assumptions(
            "05 single arm state monitor (explicit IDs)",
            [
                f"interface         : {args.interface}",
                f"duration          : {args.seconds:.2f} s",
                f"motors            : {len(args.motor)}",
                *motor_descs,
                "sends             : enable + read-only ticks + disable",
                "motion commands   : NONE",
            ],
            fd=args.fd,
        )
        transport = dm_control.SocketCanBus(args.interface, fd=args.fd)
        specs = [
            dm_control.MotorSpec(f"j{i}", resolve_motor_type(t), s, r)
            for i, (s, r, t) in enumerate(args.motor)
        ]
        robot = (
            dm_control.RobotBuilder()
            .add_bus("main", transport, DamiaoCodec())
            .add_arm("arm", bus="main", motors=specs)
            .build()
        )
        group_name = "arm"

    print("connecting...")
    robot.connect()
    try:
        print("enabling...")
        robot.enable()
        arm = robot[group_name]
        n = len(arm)
        deadline = time.monotonic() + args.seconds
        ticks = 0
        period = 1e-3
        while time.monotonic() < deadline:
            t0 = time.monotonic()
            # Query state (no motion), then tick to receive the replies.
            arm.refresh()
            robot.tick(args.deadline_us)
            ticks += 1
            if ticks % args.print_every == 0:
                pos = arm.positions()
                vel = arm.velocities()
                tau = arm.torques()
                print(f"tick={ticks:>6}")
                for i in range(n):
                    print(
                        f"  j{i}: pos={pos[i]:+.5f} vel={vel[i]:+.5f} tau={tau[i]:+.5f}"
                    )
            rem = period - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)
        print(f"completed {ticks} ticks over {n} motors")
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
