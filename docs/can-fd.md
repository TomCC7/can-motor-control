# CAN-FD in v1

## Status

CAN-FD is **not implemented** in v1. The types and trait surface are
FD-aware (`FrameFlags::FD_FORMAT`, `BusCapabilities::fd()`, unified
`CanFrame` with a 64-byte inline payload), so v2 can land FD support
**additively** — no breaking changes to upper-layer code.

## What "FD-ready" means

Every layer except the transport and codec implementations is fully
classical/FD agnostic:

| Layer | FD handling in v1 |
|---|---|
| `dm_control::Robot` / `Arm` / `Gripper` | None — never branches on `is_fd()`. |
| `dm_control::Bus` | Forwards frames; one decode per frame regardless of format. |
| `motor_codec::MotorCodec` trait | Signature unchanged — codecs can choose to emit FD. |
| `dm_codec::DamiaoCodec` | Always emits classical (`FD_FORMAT` unset) regardless of bus capability. |
| `dm_control::SocketCanBus` | Opens classical sockets only (no `CAN_RAW_FD_FRAMES`). Rejects `fd=true` at `open()`. |

## Explicit rejection points

In v1, attempting to enable CAN-FD anywhere returns an error rather than
silently falling back to classical:

- `SocketCanBus::open(_, fd=true)` → `Err(TransportError::FdNotImplementedInV1)` *before* any syscall.
- `bus.send(fd_frame)` on a `supports_fd=false` bus → `Err(TransportError::FdFrameOnNonFdBus)`.
- TOML `fd = true` → `Err(Error::FdNotImplementedInV1 { bus_name })` at parse time, before any socket open.
- Python `SocketCanBus("vcan0", fd=True)` → `dm_control.TransportError` with the substring `"CAN-FD is reserved for a future change; set fd=false"`.

## What unblocks v2

To land CAN-FD support, a future change needs to:

1. **`SocketCanBus`**: enable `CAN_RAW_FD_FRAMES` on the socket when constructed with `fd=true`; emit `canfd_frame` writes when `frame.is_fd()`. The inbound parser already detects FD frames by struct size.
2. **`DamiaoCodec`**: optionally emit FD frames when the bound `BusCapabilities::supports_fd` is true. This is a behavior change inside the codec; the trait signature is unchanged.
3. **Remove the rejection points** listed above; replace them with the intended behavior.

No changes are required to `Robot`, `Arm`, `Gripper`, `MotorGroup`, the
`MotorCodec` trait, `CanBus` trait, or any of the public Python surface.
