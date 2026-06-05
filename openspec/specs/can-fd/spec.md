# can-fd Specification

## Purpose

Specify CAN-FD transport, codec, configuration, documentation, and compatibility behavior while preserving classical CAN behavior by default.

## Requirements

### Requirement: SocketCAN transport opens an FD-capable bus on request

The `SocketCanBus` transport SHALL, when opened with FD enabled, configure the
underlying raw CAN socket for CAN-FD (`CAN_RAW_FD_FRAMES`) and advertise
`BusCapabilities` whose `supports_fd` is true and whose `max_payload_len` is 64.
When opened with FD disabled, it SHALL behave exactly as the classical v1 bus:
no FD socket option, `supports_fd` false, `max_payload_len` 8. Opening with FD
enabled SHALL NOT return any "not implemented" error.

#### Scenario: Open an FD bus
- **WHEN** `SocketCanBus::open(interface, fd=true)` is called on an FD-capable interface
- **THEN** the socket has `CAN_RAW_FD_FRAMES` set
- **AND** the bus reports `caps.supports_fd == true` and `caps.max_payload_len == 64`

#### Scenario: Open a classical bus is unchanged
- **WHEN** `SocketCanBus::open(interface, fd=false)` is called
- **THEN** the socket does not enable `CAN_RAW_FD_FRAMES`
- **AND** the bus reports `caps.supports_fd == false` and `caps.max_payload_len == 8`

#### Scenario: FD open no longer rejected
- **WHEN** any caller opens a bus with FD enabled
- **THEN** the call MUST NOT return a "CAN-FD not implemented / reserved" error

### Requirement: FD frames are transmitted as canfd frames

When the bus is FD-capable, sending a frame whose `FD_FORMAT` flag is set SHALL
serialize it as a kernel `canfd_frame` (preserving the `BRS` bit-rate-switch and
`ESI` flags and payload up to 64 bytes at a valid FD DLC). Sending a frame
without `FD_FORMAT` set SHALL still serialize as a classical `can_frame`,
regardless of the bus's FD capability.

#### Scenario: FD frame sent on an FD bus
- **WHEN** an FD frame (≤64-byte payload, valid FD DLC) is sent on an FD-capable bus
- **THEN** it is written as a `canfd_frame`
- **AND** its payload and `BRS`/`ESI` flags are preserved on the wire

#### Scenario: Classical frame on an FD bus stays classical
- **WHEN** a frame without `FD_FORMAT` is sent on an FD-capable bus
- **THEN** it is written as a classical `can_frame`

### Requirement: FD frames are rejected on a classical bus

Sending a frame with `FD_FORMAT` set on a bus whose `supports_fd` is false SHALL
return a transport error identifying the FD-on-classical-bus mismatch. This
guard is independent of the removed "not implemented" rejection and SHALL remain
in place.

#### Scenario: FD frame on a classical bus
- **WHEN** an FD frame is sent on a bus with `supports_fd == false`
- **THEN** the send returns the FD-frame-on-non-FD-bus transport error
- **AND** no frame is written to the socket

### Requirement: Inbound FD frames are received and decoded

An FD-capable bus SHALL receive both classical and FD frames, discriminating by
the kernel read length, and surface each as a `CanFrame` with `FD_FORMAT` set
iff the frame arrived as a `canfd_frame`. A classical-only bus continues to
receive classical frames only.

#### Scenario: Receive an FD frame
- **WHEN** a `canfd_frame` arrives on an FD-capable bus
- **THEN** `recv` yields a `CanFrame` with `FD_FORMAT` set and the full FD payload

#### Scenario: Receive a classical frame on an FD bus
- **WHEN** a classical `can_frame` arrives on an FD-capable bus
- **THEN** `recv` yields a `CanFrame` without `FD_FORMAT` set

### Requirement: Codec emits and decodes FD frames only when bound to an FD bus

The Damiao codec SHALL decide frame format from the `BusCapabilities` passed to
`bind_to_bus`. When bound to a classical bus (`supports_fd == false`) its
encoded output SHALL be byte-for-byte identical to the v1 classical encoding.
When bound to an FD bus it MAY emit FD frames, and its decode path SHALL accept
FD-format state frames in addition to classical ones.

#### Scenario: Classical binding preserves v1 output
- **WHEN** the codec is bound to a classical bus and encodes any command
- **THEN** the produced frames are classical and identical to the v1 byte layout

#### Scenario: FD binding accepts FD state frames
- **WHEN** the codec is bound to an FD bus and an FD-format state frame arrives
- **THEN** the codec decodes it instead of discarding it as "not ours"

### Requirement: Configuration enables FD without rejection

Robot configuration SHALL treat `fd = true` on a bus as a valid request to open
that bus in FD mode, not as a parse-time error. `fd = false` (and an omitted
`fd`) SHALL continue to open a classical bus. The Python `SocketCanBus(...,
fd=True)` constructor SHALL open an FD bus rather than raising.

#### Scenario: TOML fd=true opens an FD bus
- **WHEN** a robot is built from a config with `fd = true` on a bus
- **THEN** the build succeeds and that bus is FD-capable
- **AND** no "CAN-FD reserved / not implemented" error is returned

#### Scenario: Python fd=True opens an FD bus
- **WHEN** `SocketCanBus("vcanfd0", fd=True)` is constructed
- **THEN** it returns an FD-capable bus object instead of raising `TransportError`

### Requirement: Classical-CAN behavior is unchanged when FD is off

Enabling CAN-FD support SHALL NOT alter any observable classical-CAN behavior.
Buses opened without FD, the Damiao classical byte layouts, and existing
`fd = false` configs SHALL produce identical results to v1. Upper layers
(`Robot`, `Arm`, `Gripper`, `MotorGroup`, the `MotorCodec`/`CanBus` traits) SHALL
NOT branch on frame format.

#### Scenario: Existing classical config is unaffected
- **WHEN** an existing `fd = false` config is loaded after this change
- **THEN** the robot behaves identically to v1

#### Scenario: Upper layers never inspect frame format
- **WHEN** the `group.rs` source-invariant grep test runs
- **THEN** `Arm`/`Gripper`/`Generic` impls still contain no `is_fd(`/`FD_FORMAT` references
