## ADDED Requirements

### Requirement: CanBus trait defines the transport contract

The `motor-codec` and `dm-control` crates SHALL depend on a single `CanBus` trait that defines every operation a CAN transport must support. The trait MUST be object-safe so that `Box<dyn CanBus>` is usable as a field type. The trait MUST require `Send` so that buses can be owned by groups that may be moved between threads (background IO thread, future async adapter). The trait MUST NOT require `Sync` — buses are owned mutably by their `Robot`. A single CanBus trait MUST handle both classical CAN and CAN-FD; there is no separate `CanFdBus` trait.

#### Scenario: Trait is object-safe
- **WHEN** a developer writes `let bus: Box<dyn CanBus> = Box::new(SocketCanBus::open("vcan0", false)?);`
- **THEN** the code compiles without "the trait `CanBus` is not dyn compatible" errors

#### Scenario: Trait surface stays minimal
- **WHEN** the trait definition is inspected
- **THEN** it exposes exactly these methods: `name(&self) -> &str`, `capabilities(&self) -> BusCapabilities`, `send(&mut self, &CanFrame) -> Result<(), TransportError>`, `drain_inbound_nonblocking(&mut self) -> Result<Vec<CanFrame>, TransportError>`, `raw_fd(&self) -> Option<RawFd>`

### Requirement: CanFrame is a single unified frame representation

The `CanFrame` type SHALL represent both classical CAN and CAN-FD frames as a single struct with fields for `id: u32`, `flags: FrameFlags`, `len: u8`, and a fixed inline `data: [u8; 64]` payload buffer. The struct MUST be `Copy`. Whether the frame is classical or CAN-FD MUST be determined by the presence of the `FD_FORMAT` flag, not by storage shape. The type MUST live in the `motor-codec` crate so the codec can construct frames without depending on any transport.

The 64-byte inline payload is sized for CAN-FD's maximum DLC; classical frames use only the first 8 bytes. The ~56 bytes of trailing zeros on classical frames are accepted as the cost of a single unified type that gives all upper-layer code full agnosticism between classical and FD.

#### Scenario: Classical 8-byte frame constructed by codec
- **WHEN** a codec calls `CanFrame::classical(0x101, &[0xFF; 8])`
- **THEN** the returned frame has `id == 0x101`, `len == 8`, `flags.contains(FD_FORMAT) == false`, `flags.contains(EXTENDED_ID) == false`, and `frame.payload() == &[0xFF; 8]`

#### Scenario: Classical frame payload longer than 8 fails
- **WHEN** a codec calls `CanFrame::classical(0x101, &[0; 9])`
- **THEN** construction returns `Err(FrameError::PayloadTooLong)`

#### Scenario: FD frame constructed by codec
- **WHEN** a codec calls `CanFrame::fd(0x101, &[0xAA; 16])`
- **THEN** the returned frame has `id == 0x101`, `len == 16`, `flags.contains(FD_FORMAT) == true`, `flags.contains(BIT_RATE_SWITCH) == true`, and `frame.payload() == &[0xAA; 16]`

#### Scenario: FD frame with invalid DLC fails
- **WHEN** a codec calls `CanFrame::fd(0x101, &[0; 9])` (9 bytes is not a valid FD DLC)
- **THEN** construction returns `Err(FrameError::InvalidFdLength { got: 9 })`

#### Scenario: is_fd reflects the flag
- **WHEN** `frame.is_fd()` is called
- **THEN** it returns `frame.flags.contains(FrameFlags::FD_FORMAT)`

### Requirement: FrameFlags enumerates CAN frame metadata bits

A `FrameFlags` bitflags type SHALL exist in `motor-codec` with at minimum these named bits: `EXTENDED_ID` (29-bit ID vs 11-bit), `FD_FORMAT` (FDF — CAN-FD vs classical), `BIT_RATE_SWITCH` (BRS — FD data phase uses higher rate), `ERROR_STATE` (ESI — FD only), `REMOTE_REQUEST` (RTR — classical only, mutually exclusive with FD_FORMAT). The type MUST derive `Copy`, `Clone`, `Debug`, `Default`, `PartialEq`, `Eq`.

#### Scenario: All flag bits accessible
- **WHEN** test code constructs `FrameFlags::FD_FORMAT | FrameFlags::BIT_RATE_SWITCH`
- **THEN** the resulting value's `.contains(FD_FORMAT)` and `.contains(BIT_RATE_SWITCH)` both return `true`

#### Scenario: FD_FORMAT and REMOTE_REQUEST cannot coexist on a constructed frame
- **WHEN** a developer attempts to construct a `CanFrame` whose flags include both `FD_FORMAT` and `REMOTE_REQUEST`
- **THEN** `CanFrame::classical` and `CanFrame::fd` constructors reject the input with `Err(FrameError::IncompatibleFlags { .. })`

### Requirement: BusCapabilities describes runtime transport features

A `BusCapabilities` struct SHALL exist in `motor-codec` exposing `supports_fd: bool` and `max_payload_len: u8`. `max_payload_len` MUST equal 8 when `supports_fd` is false, and 64 when `supports_fd` is true. The struct MUST be `Copy`, `Clone`, `Debug`, `PartialEq`, `Eq`.

#### Scenario: Classical capability values
- **WHEN** `BusCapabilities { supports_fd: false, .. }` is constructed
- **THEN** the constructor (or invariant check) enforces `max_payload_len == 8`

#### Scenario: FD capability values
- **WHEN** `BusCapabilities { supports_fd: true, .. }` is constructed
- **THEN** the constructor enforces `max_payload_len == 64`

### Requirement: SocketCanBus binds to a Linux SocketCAN interface

`SocketCanBus::open(interface_name, fd_enabled)` SHALL open a `PF_CAN` socket bound to the named interface (`can0`, `vcan0`, etc.) and configure it as non-blocking. The constructor MUST return an error if the interface does not exist, permissions are insufficient, or the kernel rejects CAN-FD enablement on a non-FD-capable interface. The constructor MUST NOT panic on any platform-level failure. In v1, `fd_enabled = true` MUST return `Err(TransportError::FdNotImplementedInV1)` without opening the socket; this guarantees users get an explicit error rather than silently misbehaving classical-only operation on a misconfigured FD bus.

#### Scenario: Bind to nonexistent interface
- **WHEN** a caller invokes `SocketCanBus::open("ghost0", false)` with no `ghost0` interface present
- **THEN** the call returns `Err(TransportError::InterfaceNotFound("ghost0"))`

#### Scenario: Bind to virtual CAN interface
- **WHEN** a `vcan0` interface has been created via `ip link add dev vcan0 type vcan` and brought up
- **AND** a caller invokes `SocketCanBus::open("vcan0", false)`
- **THEN** the call returns `Ok(bus)` and the resulting `bus.name()` is `"vcan0"`
- **AND** `bus.capabilities() == BusCapabilities { supports_fd: false, max_payload_len: 8 }`

#### Scenario: fd_enabled=true rejected in v1
- **WHEN** a caller invokes `SocketCanBus::open("vcan0", true)`
- **THEN** the call returns `Err(TransportError::FdNotImplementedInV1)`
- **AND** no socket has been opened (verifiable via fd count before/after)

### Requirement: capabilities reports the bus's runtime feature set

`CanBus::capabilities` SHALL return a `BusCapabilities` value reflecting what the underlying transport actually supports at runtime, not what the type could in principle support. `MockCanBus` MUST expose its capabilities as constructor-time configurable so tests can simulate both classical and FD buses. `SocketCanBus` in v1 MUST always return `BusCapabilities { supports_fd: false, max_payload_len: 8 }` because v1 does not open FD-enabled sockets.

#### Scenario: SocketCanBus reports classical capabilities in v1
- **WHEN** `SocketCanBus::open("vcan0", false)?` is followed by `bus.capabilities()`
- **THEN** the returned value is `BusCapabilities { supports_fd: false, max_payload_len: 8 }`

#### Scenario: MockCanBus reports configured capabilities
- **WHEN** `MockCanBus::with_capabilities(BusCapabilities { supports_fd: true, max_payload_len: 64 })` is constructed
- **THEN** `mock.capabilities()` returns that exact value

### Requirement: send validates the frame against bus capabilities

`CanBus::send` SHALL inspect the frame's flags and length before issuing any IO. The following validations are required in order:
1. If `frame.flags.contains(FD_FORMAT)` and `!self.capabilities().supports_fd` → return `Err(TransportError::FdFrameOnNonFdBus)`.
2. If `frame.len > self.capabilities().max_payload_len` → return `Err(TransportError::PayloadExceedsBusCapacity { len: frame.len, max: caps.max_payload_len })`.
3. If `frame.flags.contains(EXTENDED_ID)` and the transport cannot send 29-bit IDs → return `Err(TransportError::ExtendedIdNotSupported)`.

Only after all validations pass does the transport issue a kernel write.

#### Scenario: FD frame rejected on classical bus
- **WHEN** `bus.send(&CanFrame::fd(0x101, &[0; 16])?)` is called on a bus whose `capabilities().supports_fd == false`
- **THEN** the call returns `Err(TransportError::FdFrameOnNonFdBus)`
- **AND** no kernel write is issued

### Requirement: send writes a single classical frame without blocking on inbound

`SocketCanBus::send`, when given a frame that passes capability validation and has `is_fd() == false`, SHALL serialize the `CanFrame` into the SocketCAN `can_frame` layout and call `write(2)` exactly once. It MUST NOT call `read` or `poll` during a send. On `EAGAIN` (kernel tx buffer full) the implementation MUST retry up to a configurable bounded retry count, then return `TransportError::SendBufferFull`. Other write failures MUST return `TransportError::Io(io::Error)` without retry. In v1, SocketCanBus MUST reject `is_fd() == true` frames with `TransportError::FdNotImplementedInV1` (this is reachable only if a future codec emits FD frames and the bus was somehow constructed with FD capability — defense in depth).

#### Scenario: Successful classical send
- **WHEN** a classical frame is sent on a quiet `vcan0`
- **THEN** `send` returns `Ok(())` and a peer reader on the same interface observes the same frame bytes

#### Scenario: Send on torn-down interface
- **WHEN** the interface is brought down via `ip link set vcan0 down` after `open`
- **AND** `send` is called
- **THEN** `send` returns `Err(TransportError::Io(_))`

#### Scenario: FD send rejected in v1 SocketCanBus
- **WHEN** a hypothetical caller bypasses capability validation and forces an FD frame to `SocketCanBus::send`
- **THEN** the call returns `Err(TransportError::FdNotImplementedInV1)` and no kernel write is issued

### Requirement: drain_inbound_nonblocking returns immediately and detects FD format

`drain_inbound_nonblocking` SHALL read all frames currently available in the socket's receive queue and return them as a `Vec<CanFrame>` in arrival order. It MUST NOT block waiting for new frames; an empty receive queue MUST return `Ok(Vec::new())`. It MUST handle `EAGAIN` as "no more frames available" and stop reading rather than treating it as an error.

For each kernel-returned frame, the implementation MUST inspect the kernel struct size or flag to determine whether it is a `can_frame` (16 bytes total) or `canfd_frame` (72 bytes total) and populate `CanFrame.flags` accordingly: classical frames have `FD_FORMAT` unset; FD frames have `FD_FORMAT` set, plus `BIT_RATE_SWITCH` and/or `ERROR_STATE` mirroring the kernel struct's `flags` byte. Since v1 only opens classical sockets, FD frames are never delivered by the kernel — but the decode path MUST still be implemented correctly so v2 unlocks without re-touching this code.

#### Scenario: Empty receive queue
- **WHEN** `drain_inbound_nonblocking` is called on a quiet bus
- **THEN** it returns `Ok(vec)` with `vec.is_empty()` true within 1ms

#### Scenario: Multiple pending classical frames returned in arrival order
- **WHEN** three classical frames with IDs `0x10`, `0x11`, `0x12` are sent by a peer in that order
- **AND** `drain_inbound_nonblocking` is called once on the receiving side
- **THEN** it returns `Ok(vec)` with `vec.iter().map(|f| f.id).collect::<Vec<_>>() == [0x10, 0x11, 0x12]`
- **AND** each frame has `flags.contains(FD_FORMAT) == false`

#### Scenario: FD frame decoded with FD_FORMAT flag set (forward-compat)
- **WHEN** a unit test injects a synthetic `canfd_frame` byte sequence into the socket read path
- **AND** `drain_inbound_nonblocking` parses it
- **THEN** the resulting `CanFrame` has `flags.contains(FD_FORMAT) == true` and `len` matches the FD payload length

### Requirement: raw_fd exposes a pollable file descriptor for SocketCanBus

`SocketCanBus::raw_fd` SHALL return `Some(fd)` where `fd` is the underlying `PF_CAN` socket file descriptor. Other transport implementations MAY return `None` if no single pollable fd backs the transport. Callers using `mio::Poll` MUST treat `None` as "this transport must be polled out-of-band" (e.g. on its own thread).

#### Scenario: SocketCanBus exposes its fd
- **WHEN** `bus.raw_fd()` is called on an open `SocketCanBus`
- **THEN** it returns `Some(fd)` with `fd >= 0`

### Requirement: MockCanBus provides an in-memory loopback for tests

A `MockCanBus` impl SHALL be provided behind the `mock` feature (enabled by default in `cfg(test)`). It MUST behave as a single-process loopback: frames sent by one `MockCanBus` are visible to a paired `MockCanBus` via `drain_inbound_nonblocking`. It MUST allow injecting frames programmatically and inspecting sent frames for assertions.

#### Scenario: Loopback delivers frame
- **WHEN** a frame is sent on one end of a `MockCanBus` pair
- **AND** the other end's `drain_inbound_nonblocking` is called
- **THEN** the returned vec contains exactly the sent frame

#### Scenario: Inject and inspect
- **WHEN** test code calls `bus.inject_frame(frame)` and then `drain_inbound_nonblocking`
- **THEN** the injected frame is returned
- **AND** `bus.sent_frames()` returns the frames the system-under-test transmitted, in send order

### Requirement: Multiple buses can be multiplexed via poll(2)

The `dm-control` crate SHALL provide a `BusPoller` helper that registers `raw_fd`s from multiple `Box<dyn CanBus>` instances with `mio::Poll` and wakes when any of them have data available. The helper MUST honor an overall deadline so a single tick has a bounded duration even if no bus becomes readable. Buses that return `None` from `raw_fd` MUST be polled out-of-band (the helper documents this constraint; out-of-band polling itself is out of v1 scope).

#### Scenario: Wake on first readable bus
- **WHEN** three buses are registered and a frame arrives on the second bus
- **AND** `BusPoller::wait(Duration::from_millis(10))` is called
- **THEN** the returned event set contains the second bus's token and excludes the others

#### Scenario: Deadline expires with no readable buses
- **WHEN** three quiet buses are registered
- **AND** `BusPoller::wait(Duration::from_millis(5))` is called
- **THEN** it returns an empty event set within 6ms

### Requirement: Transport errors are typed via thiserror

The `TransportError` enum SHALL use `thiserror::Error` and cover at minimum: `InterfaceNotFound(String)`, `PermissionDenied`, `Io(std::io::Error)`, `SendBufferFull`, `FrameError(FrameError)`, `FdNotImplementedInV1`, `FdFrameOnNonFdBus`, `PayloadExceedsBusCapacity { len: u8, max: u8 }`, `ExtendedIdNotSupported`. The enum MUST be `#[non_exhaustive]` so new variants do not break downstream `match` exhaustiveness.

#### Scenario: Error covers IO failures without losing detail
- **WHEN** a `send` fails because the interface was torn down
- **THEN** the returned `TransportError::Io(e)` retains `e.kind()` and `e.raw_os_error()` for diagnostics

#### Scenario: FD-not-implemented error message is actionable
- **WHEN** `TransportError::FdNotImplementedInV1` is rendered via `Display`
- **THEN** the message includes the phrase "CAN-FD is reserved for a future change; set fd=false"
