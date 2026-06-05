## ADDED Requirements

### Requirement: MotorCodec trait is the vendor-agnostic codec contract

The `motor-codec` crate SHALL define an object-safe `MotorCodec: Send + Sync` trait that every vendor codec implementation must satisfy. The trait MUST NOT use associated types so that `Box<dyn MotorCodec>` is usable in upper layers. It MUST be the single point through which `can-motor-control` interacts with vendor-specific protocol details — `can-motor-control` MUST NOT depend on any vendor codec crate. A single trait MUST handle both classical CAN and CAN-FD; there is no separate `MotorCodecFd` sibling trait. Codecs MAY choose to emit classical or CAN-FD frames based on the bound bus capabilities; both code paths produce the same unified `CanFrame` type with the `FD_FORMAT` flag distinguishing them.

#### Scenario: Trait is object-safe
- **WHEN** a developer writes `let codec: Box<dyn MotorCodec> = Box::new(DamiaoCodec::new());`
- **THEN** the code compiles without "the trait `MotorCodec` is not dyn compatible" errors

#### Scenario: can-motor-control has no vendor dependency
- **WHEN** `cargo tree -p can-motor-control` is run
- **THEN** the output MUST NOT contain `damiao-codec` or any other vendor codec crate

### Requirement: motor-codec crate is no_std

The `motor-codec` crate SHALL declare `#![no_std]` at its crate root and MUST NOT depend on `std`. It MAY depend on `alloc` for `Vec`/`String` usage. This constraint enables the trait and shared types to be reused in MCU firmware that drives motors directly.

#### Scenario: Crate compiles without std
- **WHEN** `cargo build -p motor-codec --no-default-features --target thumbv7em-none-eabihf` is run
- **THEN** the build succeeds

### Requirement: MotorTypeId discriminates motors by vendor

A `MotorTypeId` enum SHALL exist in `motor-codec` with one variant per supported vendor. Each variant MUST carry an opaque `u16` discriminant that the owning vendor codec interprets internally. The enum MUST be `#[non_exhaustive]` so adding a future vendor variant does not break downstream `match` statements.

#### Scenario: Damiao variant carries opaque discriminant
- **WHEN** a `MotorTypeId::Damiao(7)` value is matched against the enum
- **THEN** only `DamiaoCodec::supports(MotorTypeId::Damiao(7))` may return true; codecs for other vendors MUST return false

#### Scenario: Adding a new variant does not break consumers
- **WHEN** a `MotorTypeId::Robostride(0)` variant is added in a future change
- **AND** existing downstream code uses `match motor_type_id { MotorTypeId::Damiao(d) => ..., _ => fallback() }`
- **THEN** the existing code continues to compile and the new variant falls through to `fallback()`

### Requirement: Command enum captures the shared control-mode surface

A `Command` enum SHALL exist in `motor-codec` covering at minimum `Mit { kp, kd, q, dq, tau }`, `PosVel { q, dq }`, `Vel { dq }`, `PosForce { q, dq, i_pu }`. All fields MUST be `f64`. The enum represents the control-mode commands that any motor in the Damiao/Robostride/MyActuator/CubeMars family is expected to support; vendor-specific commands DO NOT appear on this enum.

#### Scenario: All four common control modes are constructible
- **WHEN** a test constructs `Command::Mit { kp: 50.0, kd: 1.0, q: 0.0, dq: 0.0, tau: 0.0 }`, `Command::PosVel { q: 1.0, dq: 2.0 }`, `Command::Vel { dq: 1.0 }`, and `Command::PosForce { q: 1.0, dq: 2.0, i_pu: 0.5 }`
- **THEN** all four variants compile and the values round-trip through `Debug`

### Requirement: Event enum captures decoded inbound frames

An `Event` enum SHALL exist in `motor-codec` covering at minimum `State { motor_id, q, dq, tau, t_mos, t_rotor }`, `ParamReply { motor_id, rid, value }`, and `Fault { motor_id, code }`. `motor_id` MUST be the recv CAN ID of the responding motor. `ParamValue` MUST cover the union of float and integer parameter encodings used by Damiao and the protocol family.

#### Scenario: State event carries all five mechanical fields
- **WHEN** an `Event::State` is constructed
- **THEN** the variant exposes `motor_id: u32`, `q: f64`, `dq: f64`, `tau: f64`, `t_mos: i16`, `t_rotor: i16`

### Requirement: encode_command produces a CanFrame or vendor-not-supported error

`MotorCodec::encode_command(&self, motor: MotorRef<'_>, cmd: &Command) -> Result<CanFrame, CodecError>` SHALL encode the command into a unified `CanFrame` using the vendor's protocol. The returned frame's `FD_FORMAT` flag MUST be set if and only if the codec is emitting a CAN-FD-formatted frame; classical frames MUST have `FD_FORMAT` unset. If the vendor does not support the requested mode for the given motor type, the call MUST return `Err(CodecError::CommandNotSupported { vendor, mode })` instead of silently substituting another mode.

#### Scenario: Damiao codec encodes MIT successfully as classical
- **WHEN** `DamiaoCodec::encode_command(motor_ref_dm4340, &Command::Mit { kp: 50.0, kd: 1.0, q: 0.0, dq: 0.0, tau: 0.0 })` is called on a Damiao codec bound to a classical bus
- **THEN** it returns `Ok(frame)` with `frame.id == motor.send_id`, `frame.len == 8`, and `frame.flags.contains(FD_FORMAT) == false`

#### Scenario: Unsupported mode returns explicit error
- **WHEN** a hypothetical vendor codec is called with a control mode it does not implement
- **THEN** it returns `Err(CodecError::CommandNotSupported { vendor: "<name>", mode: <kind> })`

### Requirement: decode parses a frame or returns Ok(None)

`MotorCodec::decode(&self, frame: &CanFrame) -> Result<Option<Event>, CodecError>` SHALL inspect the frame's CAN ID and payload and return `Ok(Some(event))` if the frame is a recognized inbound message from this vendor's motors. Frames the codec does not recognize MUST return `Ok(None)`, not an error. Frames the codec recognizes but cannot decode (malformed payload, unsupported sub-protocol revision) MUST return `Err(CodecError::DecodeFailed { reason })`.

#### Scenario: Foreign frame returns None
- **WHEN** a non-Damiao frame is passed to `DamiaoCodec::decode`
- **THEN** it returns `Ok(None)`

#### Scenario: Malformed Damiao state frame returns DecodeFailed
- **WHEN** a frame with the correct ID but truncated payload is passed to `DamiaoCodec::decode`
- **THEN** it returns `Err(CodecError::DecodeFailed { .. })`

### Requirement: limits exposes per-motor-type ranges

`MotorCodec::limits(&self, motor_type: MotorTypeId) -> Result<Limits, CodecError>` SHALL return the position, velocity, and torque maxima used to scale MIT commands and decode state frames. The `Limits` struct MUST expose `p_max: f64`, `v_max: f64`, `t_max: f64`. Unknown motor types MUST return `Err(CodecError::UnknownMotorType { vendor, type_id })`.

#### Scenario: DM4340 limits resolved
- **WHEN** `DamiaoCodec::limits(MotorTypeId::Damiao(<DM4340 discriminant>))` is called
- **THEN** it returns `Ok(Limits { p_max: 12.5, v_max: 10.0, t_max: 28.0 })`

#### Scenario: Unknown motor type
- **WHEN** `DamiaoCodec::limits(MotorTypeId::Damiao(0xFFFF))` is called
- **THEN** it returns `Err(CodecError::UnknownMotorType { vendor: "damiao", type_id: 0xFFFF })`

### Requirement: encode_enable / encode_disable / encode_set_zero are required

`MotorCodec` SHALL require `encode_enable(motor: MotorRef<'_>) -> Result<CanFrame, CodecError>`, `encode_disable(motor: MotorRef<'_>) -> Result<CanFrame, CodecError>`, and `encode_set_zero(motor: MotorRef<'_>) -> Result<CanFrame, CodecError>`. These three operations are the minimum lifecycle every vendor in the target protocol family supports; a codec that cannot implement them is out of scope for this trait.

#### Scenario: All three lifecycle commands encode for Damiao
- **WHEN** `encode_enable`, `encode_disable`, `encode_set_zero` are called for a DM4340 motor
- **THEN** each returns `Ok(frame)` with `frame.id == motor.send_id` and `frame.len == 8`

### Requirement: supports declares per-motor-type compatibility

`MotorCodec::supports(&self, motor_type: MotorTypeId) -> bool` SHALL return true if and only if the codec can encode commands for and decode events from that motor type. The Robot builder MUST consult this method when motors are added to a group and reject any motor whose type the group's codec does not support.

#### Scenario: Damiao codec rejects non-Damiao motor types
- **WHEN** `DamiaoCodec::supports(MotorTypeId::Robostride(0))` is called
- **THEN** it returns `false`

### Requirement: vendor-specific operations require downcast

The `MotorCodec` trait SHALL NOT expose any vendor-exclusive command (e.g. Damiao's parameter sub-protocol on CAN ID `0x7FF`). Vendor-specific operations MUST be defined in per-vendor extension traits in the vendor's own crate (e.g. `DamiaoCodecExt` in `damiao-codec`). Callers reach them via `&dyn Any` downcast from the boxed codec.

#### Scenario: Damiao param write reachable via downcast
- **WHEN** user code calls `group.codec_ext::<DamiaoCodec>()` on a Damiao-backed group
- **THEN** it returns `Some(&DamiaoCodec)` and the user can invoke `DamiaoCodecExt` methods on the reference

#### Scenario: Downcast to wrong vendor returns None
- **WHEN** user code calls `group.codec_ext::<DamiaoCodec>()` on a hypothetical Robostride-backed group
- **THEN** it returns `None`

### Requirement: bind_to_bus communicates bus capabilities to the codec

The `MotorCodec` trait SHALL include `fn bind_to_bus(&mut self, caps: BusCapabilities)`. The group builder MUST call this exactly once per codec instance, immediately after the codec is moved into the group and before any encode call. The codec MAY store the capabilities for later use (e.g. to decide whether to emit FD frames). Calling encode methods before `bind_to_bus` has been called MUST be a programming error; codecs MAY assume `bind_to_bus` has been called by the time any encode is invoked.

Since each codec instance is owned by exactly one `Bus` (the builder takes `Box<dyn MotorCodec>` and moves it into the `Bus` struct), there is no sharing concern across buses — a codec is bound to at most one bus's capabilities for its lifetime. The codec IS shared across multiple groups attached to the same bus; that sharing is fine because all groups on a bus see the same `BusCapabilities` and the same vendor protocol.

#### Scenario: Builder invokes bind_to_bus during add_bus
- **WHEN** `RobotBuilder::add_bus("main", Box::new(SocketCanBus::open("vcan0", false)?), Box::new(DamiaoCodec::new()))` is called
- **THEN** the codec's `bind_to_bus` method has been invoked exactly once with `transport.capabilities()` before the builder stores the `Bus`

#### Scenario: bind_to_bus is NOT re-invoked when groups are added
- **WHEN** `add_bus("main", t, c)` is followed by `add_arm("arm", "main", motors)` and `add_gripper("g", "main", motor)`
- **THEN** `bind_to_bus` has been called exactly once across the whole sequence — at `add_bus` time only

#### Scenario: Damiao codec on FD-capable bus still emits classical in v1
- **WHEN** a future test constructs a bus with `BusCapabilities { supports_fd: true, max_payload_len: 64 }` (via a hypothetical FD-capable `MockCanBus`)
- **AND** a `DamiaoCodec` is the bus's codec (its `bind_to_bus` was called with FD-capable caps)
- **AND** `bus.codec.encode_command(motor, &Command::Mit { .. })` is called
- **THEN** the returned frame has `flags.contains(FD_FORMAT) == false` (v1 Damiao codec ignores FD capability)

### Requirement: CodecError is typed via thiserror

The `CodecError` enum SHALL use `thiserror::Error` and cover at minimum: `UnknownMotorType { vendor: &'static str, type_id: u16 }`, `CommandNotSupported { vendor: &'static str, mode: CommandKind }`, `DecodeFailed { reason: &'static str }`, `OutOfRange { field: &'static str }`. The enum MUST be `#[non_exhaustive]`.

#### Scenario: OutOfRange on MIT torque beyond t_max
- **WHEN** `DamiaoCodec::encode_command` is called with `Command::Mit { tau: 1000.0, .. }` for a motor whose `t_max = 28.0`
- **THEN** it returns `Err(CodecError::OutOfRange { field: "tau" })`
