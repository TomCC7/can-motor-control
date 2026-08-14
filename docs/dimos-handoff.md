# DimOS handoff

Apply the downstream integration only after a release contains both Linux and
macOS wheels.

1. Remove DimOS's Linux-only dependency marker for `can-motor-control`.
2. Select `SocketCanBus` on Linux and `GsUsbBus` on macOS. Pass the OpenYAM
   adapter's VID, PID, and stable serial number explicitly.
3. Delete only OpenYAM's Python libusb/PyUSB routing. The Rust transport now
   owns its USB session and worker.
4. Leave Galaxea's separate Python gs_usb integration unchanged.

Do not add an import fallback between the two hardware transports. Each wheel
exports only the transport supported by its target. Roll back by pinning DimOS
to the prior `can-motor-control` release.
