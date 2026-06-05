"""Tier 3 -- bounded low-gain MIT hold across a configured arm.

Multi-motor analog of `03_single_motor_mit_hold.py`. Loads the arm from
a TOML config and runs an MIT hold loop with conservative default gains
and a bounded duration. Prints final tick count and effective frequency.

Run only after `05_single_arm_state_monitor.py` looks correct.

Defaults are intentionally conservative:
  * kp =  5.0   -- soft position stiffness on every joint
  * kd =  1.0   -- modest damping on every joint
  * q  =  0.0   -- target position (zero) on every joint
  * --seconds default 2 s, hard cap 10 s

The hold is at q=0 by default. Depending on the physical pose of the
arm at enable time, q=0 may not correspond to its current position; keep
gains low and a hand near the e-stop the first time.

Usage:
    python examples/06_single_arm_mit_hold.py --config configs/openarm_single.toml
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
    print_assumptions,
    validate_seconds,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(__doc__ or "").splitlines()[0]
    )
    parser.add_argument(
        "--config",
        required=True,
        help=(
            "path to TOML robot config (e.g. configs/openarm_single.toml; "
            "use configs/openarm_canfd.toml for a CAN-FD bus)"
        ),
    )
    parser.add_argument(
        "--group-name",
        default="arm",
        help="arm group name in the config (default: arm)",
    )
    add_bounded_run_args(parser, default_seconds=2.0)
    parser.add_argument("--kp", type=float, default=5.0, help="MIT kp on every joint (default: 5.0)")
    parser.add_argument("--kd", type=float, default=1.0, help="MIT kd on every joint (default: 1.0)")
    parser.add_argument("--q", type=float, default=0.0, help="MIT q on every joint (default: 0.0)")
    args = parser.parse_args()
    validate_seconds(args, max_seconds=10.0)

    print_assumptions(
        "06 single arm MIT hold",
        [
            f"config            : {args.config}",
            f"group name        : {args.group_name}",
            f"duration          : {args.seconds:.2f} s (hard cap 10 s)",
            f"mit gains         : kp={args.kp} kd={args.kd} on every joint",
            f"mit setpoint      : q={args.q} dq=0 tau=0 on every joint",
            "safety            : low gains -- soft hold; clear workspace and watch first",
        ],
        fd=None,  # wire format governed by the config file
    )

    import can_motor_control

    robot = can_motor_control.Robot.from_config(args.config)
    print("connecting...")
    robot.connect()
    try:
        print("enabling...")
        robot.enable()
        arm = robot[args.group_name]
        n = len(arm)
        cmds = np.zeros((n, 5), dtype=np.float64)
        cmds[:, 0] = args.kp
        cmds[:, 1] = args.kd
        cmds[:, 2] = args.q
        deadline = time.monotonic() + args.seconds
        period = 1e-3
        ticks = 0
        loop_start = time.monotonic()
        while time.monotonic() < deadline:
            t0 = time.monotonic()
            robot.tick(args.deadline_us)
            arm.mit_control(cmds)
            ticks += 1
            rem = period - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)
        elapsed = time.monotonic() - loop_start
        hz = ticks / elapsed if elapsed > 0 else float("nan")
        print(
            f"completed {ticks} ticks over {n} motors in {elapsed:.3f} s "
            f"(~{hz:.1f} Hz)"
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
