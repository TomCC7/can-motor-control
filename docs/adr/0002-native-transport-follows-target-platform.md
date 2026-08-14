# Build the native CAN transport for each target platform

Every Linux build exposes `SocketCanBus`, and every macOS build exposes `GsUsbBus`; unsupported transports are absent rather than represented by failing placeholders. `nusb` is therefore a macOS-target-specific dependency, not a user-facing opt-in feature. This keeps the Rust and Python APIs aligned and prevents a valid macOS installation from silently omitting its only native hardware transport.
