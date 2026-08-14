"""Target-native hardware transport selection shared by bring-up examples."""

from __future__ import annotations

import sys
from typing import Any

import can_motor_control


def open_native_transport(selector: str, *, fd: bool = False) -> Any:
    if sys.platform == "linux":
        return can_motor_control.SocketCanBus(selector, fd=fd)
    if sys.platform == "darwin":
        if fd:
            raise ValueError("the macOS gs_usb transport supports classical CAN only")
        try:
            vendor, product = selector.split(":", maxsplit=1)
            vendor_id = int(vendor, 16)
            product_id = int(product, 16)
        except ValueError as exc:
            raise ValueError(
                "on macOS, --interface must be a hexadecimal USB VID:PID pair, such as 1d50:606f"
            ) from exc
        return can_motor_control.GsUsbBus(vendor_id=vendor_id, product_id=product_id)
    raise RuntimeError(f"no native hardware CAN transport on {sys.platform}")
