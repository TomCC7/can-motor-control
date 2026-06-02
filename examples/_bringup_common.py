"""Shared conventions for hardware bring-up examples (`00_`-`08_`).

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
from typing import Iterable


# v1 supports classical CAN only; the bus rejects fd=True. Examples must
# print this so the operator knows the wire format before the link comes up.
V1_CAN_BANNER = (
    "dm_control v1 assumes classical CAN (no CAN-FD).\n"
    "Confirm the bus is up at the expected bitrate (commonly 1 Mbit/s)\n"
    "and that `candump <iface>` shows traffic before powering motors."
)


def parse_can_id(text: str) -> int:
    """Accept `0x11`, `17`, or `0o21` and return an int. Used for --send/recv-id."""
    text = text.strip()
    return int(text, 0)


def add_interface_arg(parser: argparse.ArgumentParser, *, default: str = "can0") -> None:
    parser.add_argument(
        "--interface",
        default=default,
        help=f"SocketCAN interface name (default: {default})",
    )


def add_single_motor_args(
    parser: argparse.ArgumentParser,
    *,
    default_send_id: int = 0x01,
    default_recv_id: int = 0x11,
    default_motor_type: str = "DM4340",
) -> None:
    """Add --send-id, --recv-id, --motor-type. Defaults must be printed before use."""
    parser.add_argument(
        "--send-id",
        type=parse_can_id,
        default=default_send_id,
        help=f"CAN id host->motor (default: 0x{default_send_id:02X})",
    )
    parser.add_argument(
        "--recv-id",
        type=parse_can_id,
        default=default_recv_id,
        help=f"CAN id motor->host (default: 0x{default_recv_id:02X})",
    )
    parser.add_argument(
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
    parser.add_argument(
        "--seconds",
        type=float,
        default=default_seconds,
        help=f"bounded duration in seconds, must be > 0 (default: {default_seconds})",
    )
    parser.add_argument(
        "--deadline-us",
        type=int,
        default=default_deadline_us,
        help=f"per-bus tick deadline in microseconds (default: {default_deadline_us})",
    )


def validate_seconds(args: argparse.Namespace, *, max_seconds: float = 30.0) -> None:
    """Reject 0/negative durations and cap unreasonably long ones for bring-up."""
    if args.seconds <= 0:
        raise SystemExit(f"--seconds must be > 0, got {args.seconds}")
    if args.seconds > max_seconds:
        raise SystemExit(
            f"--seconds={args.seconds} exceeds bring-up cap of {max_seconds}s. "
            "Bring-up examples intentionally refuse unbounded/long runs. "
            "Edit the example and raise the cap explicitly if you understand the risk."
        )


def resolve_motor_type(name: str):
    """Return a `dm_control.damiao.MotorType` member from a string.

    Imported lazily so `--help` works even if the native wheel is missing.
    """
    from dm_control.damiao import MotorType  # noqa: WPS433 (lazy by design)

    try:
        return MotorType[name]
    except KeyError as exc:  # pragma: no cover - argparse usually catches first
        valid = ", ".join(m.name for m in MotorType)
        raise SystemExit(f"unknown --motor-type {name!r}; valid: {valid}") from exc


def interface_exists(interface: str) -> bool:
    """True iff `/sys/class/net/<interface>` is present on this host."""
    return os.path.exists(f"/sys/class/net/{interface}")


def print_assumptions(
    title: str,
    items: Iterable[str],
    *,
    include_v1_banner: bool = True,
) -> None:
    """Print a uniform pre-flight block. Always call BEFORE sending CAN frames."""
    print(f"=== {title} ===")
    for item in items:
        print(f"  - {item}")
    if include_v1_banner:
        print()
        for line in V1_CAN_BANNER.splitlines():
            print(f"  {line}")
    print("=" * (len(title) + 8))
    sys.stdout.flush()
