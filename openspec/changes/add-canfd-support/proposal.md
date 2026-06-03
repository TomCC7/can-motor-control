## Why

The library was deliberately built "FD-ready": the unified `CanFrame` already
carries a 64-byte payload and `FD_FORMAT`/`BRS`/`ESI` flags, `BusCapabilities`
already models `supports_fd`, and the `MotorCodec` trait already takes a
`bind_to_bus(caps)` hint — but every path that would actually move an FD frame
is stubbed out with an explicit rejection (`FdNotImplementedInV1`). Damiao's
newer actuators and several adjacent vendors run CAN-FD for higher data-phase
bitrates and larger parameter frames; v1's classical-only ceiling blocks those
buses entirely. This change lands the FD data path additively, turning the
reserved seams into working behavior without breaking classical CAN users.

## What Changes

- **Transport (`SocketCanBus`)**: when opened with `fd=true`, set
  `CAN_RAW_FD_FRAMES` on the raw socket, advertise `BusCapabilities::fd()`, and
  send via `canfd_frame` for FD frames (classical frames still send via
  `can_frame`). The inbound path already size-discriminates classical vs FD; it
  becomes reachable instead of dead code.
- **Remove the v1 rejection points**, replacing each with the intended behavior:
  - `SocketCanBus::open(_, fd=true)` no longer returns `FdNotImplementedInV1`.
  - The send-path "defense in depth" FD reject is removed; `validate_send`
    keeps the legitimate `FdFrameOnNonFdBus` guard (FD frame on a classical bus).
  - Config parsing no longer rejects `fd = true`; it opens an FD bus.
- **Codec (`DamiaoCodec`)**: gate FD emission on the bound
  `caps.supports_fd`. Classical buses keep emitting 8-byte classical frames
  (unchanged); the decode path accepts FD state frames when bound to an FD bus.
- **Config / examples**: `fd = true` becomes valid; document the FD knob and
  ship an FD example config. `MockCanBus` gains an FD-capable constructor so the
  FD path is testable without hardware.
- **Python surface**: `SocketCanBus(..., fd=True)` now opens an FD bus instead
  of raising; docstrings and the `ConfigError` "unsupported CAN-FD" wording are
  updated. No signature changes — the `fd` parameter already exists.
- **Error surface (BREAKING, internal)**: the `FdNotImplementedInV1` variant is
  removed from `TransportError`/`Error`. This is a Rust-internal enum variant;
  the Python `TransportError`/`ConfigError` classes are unchanged.

## Capabilities

### New Capabilities
- `can-fd`: Opening, sending, and receiving CAN-FD frames over SocketCAN;
  capability negotiation between bus and codec; config/Python knobs to enable
  FD; and the invariant that classical-CAN behavior is unchanged when FD is off.

### Modified Capabilities
<!-- No existing capability spec covers transport/codec/config behavior, so
     there are no requirement deltas to an existing spec. python-docs-site is
     unrelated. -->

## Impact

- **Code**:
  - `crates/dm-control/src/transport/socketcan.rs` (open, send, caps; remove
    FD reject)
  - `crates/dm-control/src/transport/mod.rs` and `crates/dm-control/src/error.rs`
    (remove `FdNotImplementedInV1`)
  - `crates/dm-control/src/transport/mock.rs` (FD-capable mock)
  - `crates/dm-control/src/config.rs` (stop rejecting `fd = true`)
  - `crates/dm-codec/src/codec.rs` (gate FD emit/decode on bound caps)
  - `crates/dm-control-py/src/transport.rs` + `src/lib.rs` (docstrings, error
    wording)
- **Configs/examples**: `configs/` gains an FD example; existing
  `fd = false` configs are unaffected.
- **Tests**: `crates/dm-control/tests/config_loader.rs` flips the
  "fd_true_rejected" expectation; new FD round-trip tests over `MockCanBus`.
- **Docs**: `docs/can-fd.md` moves from "not implemented / reserved" to a usage
  guide; README platform/status notes updated.
- **Dependencies**: none added (transport is a direct libc impl; no `socketcan`
  crate).
- **Platforms**: FD requires an FD-capable interface + Linux kernel CAN_RAW_FD;
  classical-only and `MockCanBus` paths remain available everywhere.
