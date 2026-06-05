## Context

v1 shipped a single-vendor (Damiao), classical-CAN, single-arm walking skeleton
but was architected so CAN-FD could land additively. The seams already exist:

- `motor_codec::CanFrame` is a single unified type with a `[u8; 64]` inline
  payload and `FrameFlags` (`FD_FORMAT`, `BIT_RATE_SWITCH`, `ERROR_STATE`). There
  is no separate `CanFdFrame`. (`crates/motor-codec/src/frame.rs`)
- `BusCapabilities { supports_fd, max_payload_len }` already has `classical()`
  and `fd()` constructors. (`crates/motor-codec/src/caps.rs`)
- `MotorCodec::bind_to_bus(caps)` already exists so a codec can decide format
  from capabilities. `DamiaoCodec` already stores the caps but ignores them.
- `SocketCanBus` is a direct libc implementation (no `socketcan` crate). It
  already defines `KernelCanFrame` (16 bytes) and `KernelCanFdFrame` (72 bytes),
  already allocates an FD-sized receive buffer, and already has a
  size-discriminating receive `match` with a `decode_kernel_fd` arm. The send
  path and `open` are the only classical-only parts.

The work is therefore mostly *removing rejections and wiring the existing FD
arms*, plus tests and docs — not new architecture.

Current explicit rejection points (all return `FdNotImplementedInV1`):
1. `SocketCanBus::open(_, fd=true)` — `transport/socketcan.rs`
2. send-path "defense in depth" `if frame.is_fd()` — `transport/socketcan.rs`
3. config parse loop `if bus.fd` — `config.rs`
4. (Python) `transport.rs` surfaces these as `TransportError`/`ConfigError`.

A *legitimate* guard, `FdFrameOnNonFdBus` in `validate_send`, must stay.

## Goals / Non-Goals

**Goals:**
- Open an FD-capable SocketCAN bus (`CAN_RAW_FD_FRAMES`) and send/receive
  `canfd_frame`s.
- Make the bus↔codec capability handshake actually drive format selection.
- Keep classical-CAN byte-for-byte identical when FD is off.
- Make the FD path testable without hardware via an FD-capable `MockCanBus`.
- Update Python docstrings/error wording and ship an FD example config + guide.

**Non-Goals:**
- Changing Damiao's *application* protocol to use >8-byte payloads. v1 Damiao
  command/state frames are 8 bytes; FD support here is about the transport and
  the format flag, not redesigning the codec's packing. FD emission for Damiao
  stays conservative (classical 8-byte frames are still valid on an FD bus); the
  decode path simply stops discarding FD frames.
- Multi-vendor FD codecs, BRS tuning / data-bitrate configuration beyond the
  kernel default, and any change to `Robot`/`Arm`/`Gripper`/`MotorGroup`.
- macOS/Windows support — FD still requires Linux SocketCAN.

## Decisions

### 1. Capability is the single source of truth for format
`bind_to_bus(caps)` is what the codec keys off; the bus advertises `fd()` vs
`classical()` from how it was opened. No separate "fd mode" flag is threaded
through call sites. *Why:* the seam already exists and keeps upper layers
format-agnostic (preserving the `group.rs` grep invariant). *Alternative
considered:* a per-call `send_fd` API — rejected as it leaks format into every
caller and breaks the agnostic upper layers.

### 2. Remove `FdNotImplementedInV1`, keep `FdFrameOnNonFdBus`
The "not implemented" variant is deleted from `TransportError` and `Error`; the
mismatch guard (FD frame on a classical bus) is retained and is the real
invariant. *Why:* once FD is implemented the "not implemented" error is a lie;
leaving it as dead-but-reachable invites confusion. This is a Rust-internal
enum change (BREAKING for any Rust code matching that variant), but the Python
exception *classes* are unchanged — only the message wording updates.

### 3. Send path branches on the frame, not the bus
`send` chooses `canfd_frame` iff `frame.is_fd()`. A classical frame on an FD bus
still goes out as a `can_frame`. *Why:* matches kernel semantics (an FD socket
can still send classical frames) and keeps Damiao's existing 8-byte output valid
on an FD bus with zero codec changes. The `validate_send` `FdFrameOnNonFdBus`
check runs first so FD-on-classical is rejected before serialization.

### 4. FD-capable MockCanBus for hardware-free tests
Add an FD constructor to `MockCanBus` advertising `BusCapabilities::fd()` so
round-trip FD encode→send→recv→decode is testable in CI. *Why:* the SocketCAN FD
path needs a real FD interface (`vcanfd`); the mock proves the format-selection
and capability logic without one. *Alternative:* rely on `vcan` in CI — rejected
as not all CI kernels expose FD vcan, and it wouldn't exercise the codec
capability gating in isolation.

### 5. Receive path is already correct — just reachable
`recv` already allocates `KERNEL_FD_LEN` and matches read length to
`decode_kernel_classical` / `decode_kernel_fd`. Enabling `CAN_RAW_FD_FRAMES` at
`open` is what makes the FD arm reachable; no receive rewrite needed.

## Risks / Trade-offs

- **Kernel struct layout assumptions** (`#[repr(C)]` sizes 16/72 are x86_64
  Linux) → keep the existing size-based discrimination; add a test asserting
  `size_of::<KernelCanFdFrame>()` so a layout drift fails loudly rather than
  mis-parsing frames.
- **Removing an enum variant is BREAKING for Rust matchers** → it's pre-1.0,
  internal, and the variant only ever signalled "not yet"; call it out in the
  changelog. Python is unaffected.
- **FD requires real FD hardware/interface** → CI covers logic via `MockCanBus`;
  the SocketCAN FD path is validated on an FD-capable interface during hardware
  bring-up, documented as such (mock proves API, not hardware).
- **Conservative Damiao FD emission** (still 8-byte frames) means "FD support"
  doesn't immediately yield larger Damiao frames → intended; this change unblocks
  FD buses and FD-frame transport. Vendor protocols that actually need >8-byte
  payloads are a follow-up that now has a working transport to build on.
- **`docs/can-fd.md` currently documents rejection as a feature** → rewrite it
  to a usage guide and update README status/platform notes so docs don't claim
  FD is reserved after it ships.

## Migration Plan

1. Land transport + codec + config changes behind the existing `fd` flag
   (default `false`), so every existing config and the entire classical path is
   untouched.
2. Flip `crates/can-motor-control/tests/config_loader.rs::fd_true_rejected_at_parse_time`
   to assert success; add FD round-trip tests over `MockCanBus`.
3. Update Python docstrings, `ConfigError` wording, `docs/can-fd.md`, README.
4. Rollback is reverting the change set; no persisted state or data migration is
   involved (`fd = false` users never exercised any new path).

## Open Questions

- Should the FD example config target a conventional `vcanfd0`/`can1` name, or
  reuse `can0`? (Leaning: a distinct `canfd0` to avoid implying every `can0` is
  FD-capable.) Resolved during tasks unless hardware naming dictates otherwise.
- Do we expose data-phase bitrate (BRS) configuration now or defer? (Leaning:
  defer — rely on the interface's configured dbitrate; BRS flag is preserved but
  not tuned here.)
