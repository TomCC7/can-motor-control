"""Tier 0 -- no frames sent.

Verify that a SocketCAN interface is present and print the assumptions a
real bring-up run depends on. This example never constructs a Robot, never
calls `SocketCanBus`, and never sends a CAN frame; running it can not move
a motor. It is safe to run on any host, including one with no CAN hardware.

What it does:
  * checks `/sys/class/net/<interface>` to see if the kernel knows the
    interface name,
  * prints v1 classical-CAN assumptions,
  * suggests `ip link show <interface>` and `candump <interface>` as the
    operator's next two manual checks,
  * exits non-zero with a clear diagnostic if the interface is missing.

Usage:
    python examples/00_can_interface_check.py
    python examples/00_can_interface_check.py --interface vcan0
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _bringup_common import (  # noqa: E402
    add_interface_arg,
    interface_exists,
    print_assumptions,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(__doc__ or "").splitlines()[0]
    )
    add_interface_arg(parser)
    args = parser.parse_args()

    print_assumptions(
        "00 CAN interface check",
        [
            f"interface         : {args.interface}",
            "sends CAN frames? : no -- this example never opens the bus",
            "hardware required : none",
        ],
    )

    if not interface_exists(args.interface):
        print(
            f"\nERROR: /sys/class/net/{args.interface} does not exist.",
            file=sys.stderr,
        )
        print(
            "  * For a real interface, plug in the adapter and check `dmesg`.",
            file=sys.stderr,
        )
        print(
            "  * For a virtual interface, set it up with:",
            file=sys.stderr,
        )
        print(
            f"      sudo ip link add dev {args.interface} type vcan",
            file=sys.stderr,
        )
        print(
            f"      sudo ip link set {args.interface} up",
            file=sys.stderr,
        )
        print(
            "  * Verify with: ip link show",
            file=sys.stderr,
        )
        return 2

    print(f"\nOK: /sys/class/net/{args.interface} is present.")
    print("Next manual checks (run in another terminal):")
    print(f"  ip link show {args.interface}")
    print(f"  candump {args.interface}")
    print("\nWhen `candump` is quiet and the interface is `UP`, proceed to:")
    print("  examples/01_single_motor_enable_disable.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
