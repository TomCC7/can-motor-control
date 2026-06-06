# Changelog

## v0.1.0 — unreleased

Initial walking-skeleton release.

### Added

- `can-motor-codec` package (import crate `motor_codec`, no_std): vendor-agnostic `MotorCodec` trait, unified
  `CanFrame` (classical + CAN-FD), `FrameFlags`, `BusCapabilities`,
  `MotorTypeId` (non-exhaustive enum), `Command` (MIT / PosVel / Vel /
  PosForce), `Event` (State / ParamReply / Fault), `Limits`, `CodecError`.
- `can-motor-damiao-codec` package (import crate `damiao_codec`, no_std): Damiao implementation of `MotorCodec` covering
  13 motor SKUs (DM3507, DM4310, DM4310_48V, DM4340, DM4340_48V, DM6006,
  DM8006, DM8009, DM10010L, DM10010, DMH3510, DMH6215, DMG6220). MIT
  bit-packing, CAN ID offsets (0x100/0x200/0x300), enable/disable/setzero
  (0xFC/0xFD/0xFE). `DamiaoCodecExt` for the 0x7FF parameter sub-protocol.
- `can-motor-control` crate: `CanBus` trait, `SocketCanBus` (Linux SocketCAN over
  raw libc), `MockCanBus` (in-memory loopback for tests), `BusPoller`
  (mio-based multi-bus multiplexing), `Motor`, `MotorGroup`, `Arm`,
  `Gripper` (v1-minimal one-motor), `Generic`, `GroupKind`, `Bus`, `Robot`,
  `RobotBuilder`, `CodecRegistry`, TOML config loader.
- `can-motor-control-py` crate: PyO3 bindings, `can_motor_control` Python package,
  `can_motor_control.damiao` submodule, numpy interop, context-manager protocol on
  `Robot`, GIL release on blocking calls, `DmError` hierarchy
  (`TransportError`, `CodecError`, `ConfigError`, `LifecycleError`),
  `.pyi` type stubs.
- End-to-end examples: `crates/can-motor-control/examples/single_arm.rs`,
  `examples/single_arm.py`.
- Integration tests for full lifecycle (build → connect → enable → 100-tick
  MIT loop → disable → drop) in both Rust and Python.
- CI workflow with fmt / clippy / test / no_std cross-build / vendor
  isolation grep / maturin wheel / pytest / docs jobs, plus release-candidate
  and protected registry publishing workflows.

### Publish order

When publishing the workspace to crates.io, the dependency order matters:

```
cargo publish -p can-motor-codec
cargo publish -p can-motor-damiao-codec
cargo publish -p can-motor-control
# (can-motor-control-py is published as a wheel via maturin, not crates.io)
```

See `docs/release.md` for the full PyPI and crates.io release runbook,
including first-release crates.io token/manual fallback and Trusted Publishing
setup for later releases.

### Limitations

- Linux SocketCAN only (the transport is Linux-specific; the codec is
  platform-agnostic).
- Classical CAN only. CAN-FD types ship; FD operation is reserved for v2.
  See `docs/can-fd.md`.
- Damiao codec only. The trait surface accommodates additional vendors; see
  `docs/multi-vendor.md`.
- Gripper exposes only `mit_control` / `pos_vel_control` / enable / disable
  in v1. `open` / `close` / `set_force` / `calibrate` reserved for a
  follow-up change.
- Sync API only; no async runtime.
