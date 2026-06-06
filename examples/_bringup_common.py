"""Shared conventions for hardware bring-up examples (`00_`-`06_`).

These helpers exist so every bring-up example uses the same flag names,
the same defaults, and prints the same assumption block before sending
anything to a motor. They are intentionally tiny; if you find yourself
reaching for them from non-bring-up code, copy the snippet instead.

Conventions enforced here:

* `--interface`        SocketCAN interface (default `can0`).
* `--send-id` / `--recv-id`  single-motor CAN IDs as hex or decimal.
* `--motor-type`       Damiao motor model (e.g. `DM4340`).
* `--seconds`          bounded run duration; 0 is rejected to prevent
                       unbounded loops on accident.
* `--deadline-us`      per-bus tick deadline (default 500 us).
* `--mock`             developer-only: use MockCanBus. Bring-up examples
                       that exercise hardware behavior should NOT accept
                       `--mock`; only examples that purely test argument
                       handling should opt in.

Assumption-printing is centralized in `print_assumptions()` so a reader
can grep `examples/` and confirm every hardware-affecting example tells
the operator what it is about to do before doing it.
"""

from __future__ import annotations

import argparse
import os
import sys
from collections.abc import Iterable
from typing import cast


# Examples print the wire format before the link comes up so the operator knows
# whether classical CAN or CAN-FD is in play. `fd` is a tri-state:
#   False -> classical CAN (the default, --fd not passed)
#   True  -> CAN-FD (--fd passed; interface must be FD-capable)
#   None  -> bus is built from a --config file; fd is governed by the config.
def wire_format_lines(fd: "bool | None") -> "list[str]":
    if fd is None:
        return [
            "Wire format is set by the bus `fd =` field in --config.",
            "Use configs/openarm_canfd.toml for a CAN-FD bus.",
            "Confirm the bus is up and `candump <iface>` shows traffic before powering motors.",
        ]
    if fd:
        return [
            "This example opens the bus in CAN-FD mode (fd=True).",
            "The interface must be CAN-FD-capable (e.g. `ip link set <iface> mtu 72`).",
            "Confirm the bus is up and `candump <iface>` shows traffic before powering motors.",
        ]
    return [
        "This example uses classical CAN (pass --fd for a CAN-FD bus).",
        "Confirm the bus is up at the expected bitrate (commonly 1 Mbit/s)",
        "and that `candump <iface>` shows traffic before powering motors.",
    ]


def parse_can_id(text: str) -> int:
    """Accept `0x11`, `17`, or `0o21` and return an int. Used for --send/recv-id."""
    text = text.strip()
    return int(text, 0)


def add_interface_arg(parser: argparse.ArgumentParser, *, default: str = "can0") -> None:
    _ = parser.add_argument(
        "--interface",
        default=default,
        help=f"SocketCAN interface name (default: {default})",
    )


def add_fd_arg(parser: argparse.ArgumentParser) -> None:
    """Add `--fd` to open the bus in CAN-FD mode (interface must be FD-capable)."""
    _ = parser.add_argument(
        "--fd",
        action="store_true",
        help="open the bus in CAN-FD mode; the interface must be FD-capable",
    )


def add_single_motor_args(
    parser: argparse.ArgumentParser,
    *,
    default_send_id: int = 0x01,
    default_recv_id: int = 0x11,
    default_motor_type: str = "DM4340",
) -> None:
    """Add --send-id, --recv-id, --motor-type. Defaults must be printed before use."""
    _ = parser.add_argument(
        "--send-id",
        type=parse_can_id,
        default=default_send_id,
        help=f"CAN id host->motor (default: 0x{default_send_id:02X})",
    )
    _ = parser.add_argument(
        "--recv-id",
        type=parse_can_id,
        default=default_recv_id,
        help=f"CAN id motor->host (default: 0x{default_recv_id:02X})",
    )
    _ = parser.add_argument(
        "--motor-type",
        default=default_motor_type,
        help=f"Damiao motor model, e.g. DM4310, DM4340 (default: {default_motor_type})",
    )


def add_bounded_run_args(
    parser: argparse.ArgumentParser,
    *,
    default_seconds: float = 2.0,
    default_deadline_us: int = 500,
) -> None:
    """Add --seconds (must be > 0) and --deadline-us."""
    _ = parser.add_argument(
        "--seconds",
        type=float,
        default=default_seconds,
        help=f"bounded duration in seconds, must be > 0 (default: {default_seconds})",
    )
    _ = parser.add_argument(
        "--deadline-us",
        type=int,
        default=default_deadline_us,
        help=f"per-bus tick deadline in microseconds (default: {default_deadline_us})",
    )


def validate_seconds(args: argparse.Namespace, *, max_seconds: float = 30.0) -> None:
    """Reject 0/negative durations and cap unreasonably long ones for bring-up."""
    seconds = cast(float, args.seconds)
    if seconds <= 0:
        raise SystemExit(f"--seconds must be > 0, got {seconds}")
    if seconds > max_seconds:
        raise SystemExit(
            f"--seconds={seconds} exceeds bring-up cap of {max_seconds}s. "
            + "Bring-up examples intentionally refuse unbounded/long runs. "
            + "Edit the example and raise the cap explicitly if you understand the risk."
        )


def resolve_motor_type(name: str) -> int:
    """Return a `can_motor_control.damiao.MotorType` member from a string.

    Imported lazily so `--help` works even if the native wheel is missing.
    """
    from can_motor_control.damiao import MotorType  # noqa: WPS433 (lazy by design)

    motor_type = getattr(MotorType, name, None)
    if motor_type is not None:
        return cast(int, motor_type)

    valid = ", ".join(n for n in dir(MotorType) if n.startswith("DM"))
    raise SystemExit(f"unknown --motor-type {name!r}; valid: {valid}")


def interface_exists(interface: str) -> bool:
    """True iff `/sys/class/net/<interface>` is present on this host."""
    return os.path.exists(f"/sys/class/net/{interface}")


def print_assumptions(
    title: str,
    items: Iterable[str],
    *,
    fd: "bool | None" = False,
    include_wire_banner: bool = True,
) -> None:
    """Print a uniform pre-flight block. Always call BEFORE sending CAN frames.

    `fd` selects the wire-format banner: False = classical, True = CAN-FD,
    None = governed by a --config file. See `wire_format_lines`.
    """
    print(f"=== {title} ===")
    for item in items:
        print(f"  - {item}")
    if include_wire_banner:
        print()
        for line in wire_format_lines(fd):
            print(f"  {line}")
    print("=" * (len(title) + 8))
    _ = sys.stdout.flush()
