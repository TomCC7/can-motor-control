# CAN-FD

CAN-FD is **supported**. A bus opened in FD mode uses the kernel's CAN-FD
frame format (`CAN_RAW_FD_FRAMES`), can carry payloads up to 64 bytes, and
preserves the bit-rate-switch (BRS) and error-state (ESI) flags. Classical CAN
remains the default and is unchanged when FD is off.

## Enabling FD

The interface itself must be CAN-FD-capable. For a virtual interface:

```bash
sudo ip link add dev vcanfd0 type vcan
sudo ip link set vcanfd0 mtu 72        # 72 = sizeof(struct canfd_frame)
sudo ip link set up vcanfd0
```

**TOML config** — set `fd = true` on the bus (see
[`configs/openarm_canfd.toml`](https://github.com/cc/dm_control_rs/blob/main/configs/openarm_canfd.toml)):

```toml
[bus.main]
kind      = "socketcan"
interface = "canfd0"
fd        = true
vendor    = "damiao"
```

**Rust:**

```rust
let bus = dm_control::SocketCanBus::open("canfd0", /* fd_enabled = */ true)?;
assert!(bus.capabilities().supports_fd);
```

**Python:**

```python
import dm_control
bus = dm_control.SocketCanBus("canfd0", fd=True)
```

## How format is chosen

A bus advertises [`BusCapabilities`]: `classical()` (8-byte cap) or `fd()`
(64-byte cap). This is the single source of truth — upper layers (`Robot`,
`Arm`, `Gripper`, `MotorGroup`) never branch on frame format.

| Layer | Behavior |
|---|---|
| `SocketCanBus` | Opened with `fd=true` → sets `CAN_RAW_FD_FRAMES`, advertises `fd()`. Sends a frame as `canfd_frame` iff its `FD_FORMAT` flag is set, otherwise as a classical `can_frame` — an FD socket carries both. |
| `validate_send` | Rejects an FD frame on a classical bus (`FdFrameOnNonFdBus`). |
| `MotorCodec::bind_to_bus(caps)` | The codec keys decode/emit decisions off the bound capabilities. |
| `DamiaoCodec` | **Conservative**: emits classical 8-byte frames on any bus (those are valid CAN-FD frames). When bound to an FD bus it additionally *accepts* FD-format state frames on decode; bound classical, it reproduces the v1 byte layout exactly. |

## Scope

Enabling FD unlocks the FD transport path and FD-frame decode. It does **not**,
by itself, widen Damiao command payloads beyond 8 bytes — Damiao's v1
command/state protocol is 8-byte. A vendor protocol that genuinely needs
>8-byte payloads can now build on a working FD transport.

BRS data-phase bitrate is taken from the interface's configured `dbitrate`; the
BRS flag is preserved on frames but not tuned by this library.

## Testing without FD hardware

`MockCanBus::new_fd(name)` (Rust) advertises FD capabilities and loops FD frames
back, so the FD send/receive and codec-gating paths are exercisable in CI
without an FD-capable interface. The SocketCAN FD path itself is validated
against a real or virtual FD interface during hardware bring-up.

[`BusCapabilities`]: https://docs.rs/dm-control
