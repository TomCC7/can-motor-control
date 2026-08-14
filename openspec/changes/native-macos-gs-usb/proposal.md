## Why

`can-motor-control` currently exposes only Linux SocketCAN for real hardware, so DimOS cannot use the merged OpenYAM motor stack natively on macOS. A hardware-tested Python workaround establishes the gs_usb behaviors that matter, but moving the transport into Rust is necessary to preserve the existing `CanBus` ownership and polling model without placing Python, the GIL, or a system `libusb` dependency in the control path.

## What Changes

- Add a native macOS `GsUsbBus` that implements the existing synchronous `CanBus` contract over `nusb` for one-channel classical CAN adapters.
- Continuously drain queued USB input in a single background worker that owns discovery, initialization, TX, RX, echo handling, failure, and shutdown.
- Select adapters explicitly by VID/PID and either serial number or enumeration index, calculate nominal bit timing from reported device capabilities, and establish a clean session boundary before construction succeeds.
- Expose `GsUsbBus` and read-only transport statistics through the macOS Python extension and allow `RobotBuilder.add_bus()` to consume it once.
- **BREAKING**: Make hardware transport exports target-native: Linux builds expose `SocketCanBus`, macOS builds expose `GsUsbBus`, and unsupported transport placeholders are absent.
- Make `nusb` a macOS-target dependency; do not introduce `rusb`, system `libusb`, PyUSB fallback, Tokio, SLCAN, or CAN-FD support.
- Extend CI, release-candidate, and release workflows to build, clean-install, import, and test native macOS arm64 wheels and to compile all macOS Rust targets.
- Update platform documentation and descriptions that currently promise Linux-only operation.

## Capabilities

### New Capabilities

- `macos-gs-usb-transport`: Native macOS gs_usb discovery, configuration, classical-CAN transport behavior, lifecycle, statistics, target-native Rust/Python APIs, and hardware acceptance.

### Modified Capabilities

- `release-automation`: Add macOS arm64 Rust and Python wheel validation and publishable macOS wheel artifacts to the existing CI and release contract.

## Impact

- Rust core transport, error, configuration, polling documentation, examples, crate exports, and target-specific dependencies.
- PyO3 transport wrappers, builder type acceptance, native module exports, pure-Python exports, type stubs, and Python tests.
- GitHub Actions CI, release-candidate, and tag-driven release workflows.
- Public Rust and Python platform APIs: `SocketCanBus` becomes Linux-only and `GsUsbBus` macOS-only.
- Downstream DimOS can replace the OpenYAM Python USB workaround with `can_motor_control.GsUsbBus` after an upstream release; the separate Galaxea integration remains out of scope.
