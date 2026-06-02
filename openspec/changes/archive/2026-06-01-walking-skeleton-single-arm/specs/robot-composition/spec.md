## ADDED Requirements

### Requirement: Bus struct bundles a transport and a vendor codec

A `Bus` struct SHALL bundle a transport (`Box<dyn CanBus>`) and a vendor codec (`Box<dyn MotorCodec>`). One `Bus` instance per CAN interface; the codec is shared across every group attached to that bus. A bus's vendor is determined by its codec at construction time and is immutable for the bus's lifetime. The bus additionally owns a recv-id routing table (populated at `connect()`; see "Connect builds per-bus recv_id routing table").

```rust
pub struct Bus {
    transport: Box<dyn CanBus>,
    codec:     Box<dyn MotorCodec>,
    routes:    HashMap<u32, RouteKey>,   // recv_id → (group_name, motor_index); empty until connect()
}
pub struct RouteKey { pub group_name: String, pub motor_index: usize }
```

The bus exposes accessors for its codec's vendor name and capabilities so groups can perform pre-flight validation:

```rust
impl Bus {
    pub fn vendor(&self) -> &str { self.codec.vendor_name() }
    pub fn capabilities(&self) -> BusCapabilities { self.transport.capabilities() }
    pub fn codec_supports(&self, mt: MotorTypeId) -> bool { self.codec.supports(mt) }
}
```

#### Scenario: Bus reports its vendor and capabilities
- **WHEN** a `Bus` is constructed with `SocketCanBus("vcan0", false)` and `DamiaoCodec::new()`
- **THEN** `bus.vendor() == "damiao"` and `bus.capabilities() == BusCapabilities { supports_fd: false, max_payload_len: 8 }`

#### Scenario: One codec instance per bus regardless of group count
- **WHEN** a robot has one bus with three groups attached (arm, gripper, generic)
- **THEN** there is exactly one `Box<dyn MotorCodec>` instance in the entire robot; the three groups all use that same codec via the bus

### Requirement: Robot owns named buses and named groups

`Robot` SHALL hold buses as `HashMap<String, Bus>` (where `Bus` is the struct defined above) and groups as `HashMap<String, GroupKind>`. Bus and group names MUST be non-empty strings; names with leading/trailing whitespace MUST be rejected at builder time. Bus names MUST be unique within the robot; group names MUST be unique within the robot.

#### Scenario: Duplicate bus name rejected

- **WHEN** `RobotBuilder::new().add_bus("main", t1, c1).add_bus("main", t2, c2).build()` is called
- **THEN** `build()` returns `Err(Error::DuplicateBusName("main"))`

#### Scenario: Duplicate group name rejected

- **WHEN** two `add_arm` calls with the same name are chained
- **THEN** `build()` returns `Err(Error::DuplicateGroupName("..."))`

### Requirement: RobotBuilder is the canonical construction path

`RobotBuilder` SHALL be the single mechanism that constructs a `Robot`. `Robot::from_config` MUST be implemented as a thin wrapper that parses TOML into a `RobotConfig` then calls `RobotBuilder::from_config(cfg).build()`. The builder MUST expose:

- `new() -> Self`
- `from_config(path) -> Result<Self, Error>`
- `add_bus(name: impl Into<String>, transport: Box<dyn CanBus>, codec: Box<dyn MotorCodec>) -> Self` — registers both transport and codec together. The builder MUST call `codec.bind_to_bus(transport.capabilities())` exactly once during this call.
- `add_arm(name: impl Into<String>, bus_name: impl Into<String>, motors: Vec<MotorSpec>) -> Self` — no codec parameter; vendor is determined by the bus
- `add_gripper(name: impl Into<String>, bus_name: impl Into<String>, motor: MotorSpec) -> Self`
- `add_generic(name: impl Into<String>, bus_name: impl Into<String>, motors: Vec<MotorSpec>) -> Self`
- `build() -> Result<Robot, Error>`

#### Scenario: add_bus binds codec to capabilities
- **WHEN** `RobotBuilder::new().add_bus("main", Box::new(SocketCanBus::open("vcan0", false)?), Box::new(DamiaoCodec::new()))` is called
- **THEN** the codec's `bind_to_bus` method has been invoked exactly once with `transport.capabilities()` before the builder stores the bus

#### Scenario: add_arm rejects unknown bus
- **WHEN** `RobotBuilder::new().add_arm("arm", "ghost", motors).build()` is called and no bus "ghost" exists
- **THEN** `build()` returns `Err(Error::UnknownBusName("ghost"))`

#### Scenario: from_config produces a Robot equivalent to builder

- **WHEN** the same robot is constructed via `RobotBuilder` directly and via `from_config` reading a TOML file describing the same topology
- **THEN** both Robots have identical `bus_names()` and `group_names()`, and each bus has the same vendor

### Requirement: TOML config schema is well-defined

The config schema SHALL accept:

- `[bus.<name>]` tables with required `kind` ("socketcan" in v1), required `vendor` (the codec to load — "damiao" in v1), kind-specific fields (`interface`, `fd` for socketcan; `fd` defaults to false and in v1 MUST be false)
- `[[group]]` array-of-tables with required `name`, `kind` ("arm" | "gripper" | "generic"), `bus` fields; an optional `default_control_mode`; motors as either `motors = [ ... ]` (for arm/generic) or `motor = { ... }` (for gripper). The `vendor` field MUST NOT appear on a group (vendor is bus-level).
- per-motor `{ name, type, send_id, recv_id }` records with `type` resolved by the referenced bus's codec

Unknown top-level keys MUST be rejected with a clear error. Unknown per-table keys MUST be rejected. A `vendor` key on a group MUST be rejected with a hint to move it to the bus. The loader MUST report errors with file path and line number when possible.

#### Scenario: Minimal single-arm config parses

- **WHEN** a config with one bus (vendor = "damiao") and one arm (four motors) is loaded
- **THEN** `from_config` returns `Ok(robot)` and `robot.bus_names()` yields one name, `robot.group_names()` yields one name

#### Scenario: Unknown key rejected

- **WHEN** a config contains `[bus.main] vendor = "damiao" enable_speed = 5` (no such bus field)
- **THEN** `from_config` returns `Err(Error::ConfigSchema { .. })` with the offending key in the message

#### Scenario: Vendor on group rejected with helpful message

- **WHEN** a config contains `[[group]] name = "arm" bus = "main" vendor = "damiao" motors = [...]`
- **THEN** `from_config` returns `Err(Error::ConfigSchema { .. })` whose message contains "vendor belongs on [bus.<name>], not on [[group]]"

#### Scenario: Bus name reference validated

- **WHEN** a config contains `[[group]] bus = "ghost"` with no `[bus.ghost]` defined
- **THEN** `from_config` returns `Err(Error::UnknownBusName("ghost"))`

### Requirement: TOML fd = true is rejected in v1

The TOML loader SHALL reject any `[bus.<name>]` table with `fd = true` in v1, returning `Err(Error::FdNotImplementedInV1 { bus_name })` at parse time before any socket is opened. `fd = false` and the absence of the `fd` key MUST both be accepted (absence is treated as false). The error message MUST name the offending bus and direct the user to set `fd = false`.

#### Scenario: fd = false accepted

- **WHEN** a config contains `[bus.main] kind = "socketcan" interface = "can0" fd = false`
- **THEN** `from_config` returns `Ok(robot)` and the loaded bus has `bus.capabilities().supports_fd == false`

#### Scenario: fd key absent accepted as false

- **WHEN** a config contains `[bus.main] kind = "socketcan" interface = "can0"` with no `fd` key
- **THEN** `from_config` returns `Ok(robot)` and the loaded bus has `bus.capabilities().supports_fd == false`

#### Scenario: fd = true rejected at parse time

- **WHEN** a config contains `[bus.main] kind = "socketcan" interface = "can0" fd = true`
- **THEN** `from_config` returns `Err(Error::FdNotImplementedInV1 { bus_name: "main" })`
- **AND** no socket has been opened (verifiable via fd count before/after)

### Requirement: vendor field on bus selects the MotorCodec

The TOML `vendor` field on each `[bus.<name>]` table SHALL select the codec implementation constructed for that bus. The loader MUST maintain a registry of vendor name → codec factory (`fn() -> Box<dyn MotorCodec>`); in v1 the registry MUST contain "damiao" → `|| Box::new(DamiaoCodec::new())`. Vendors not in the registry MUST produce `Err(Error::UnknownVendor("..."))` at load time, not at run time.

#### Scenario: damiao vendor resolves

- **WHEN** a config specifies `[bus.main] vendor = "damiao"`
- **THEN** the loaded bus has `bus.vendor() == "damiao"` and its codec is an instance of `DamiaoCodec`

#### Scenario: Unregistered vendor fails

- **WHEN** a config specifies `[bus.main] vendor = "ghost"` and no codec is registered under that name
- **THEN** `from_config` returns `Err(Error::UnknownVendor("ghost"))`

### Requirement: Validation runs before any IO

All builder validation (name uniqueness, bus references, motor-codec support via the bus's codec, gripper one-motor rule) SHALL execute inside `RobotBuilder::build()` before any socket is opened. `Robot::from_config` MUST complete without opening sockets; opening sockets is the responsibility of `connect()`.

#### Scenario: from_config without hardware

- **WHEN** `Robot::from_config(path)` is called on a host with no CAN interfaces
- **THEN** the call returns `Ok(robot)` (or fails only due to config-content errors, not IO)

#### Scenario: Codec mismatch caught at build time

- **WHEN** a builder declares a Damiao-vendor bus and an arm on it whose motors include a non-Damiao `MotorTypeId`
- **THEN** `build()` returns `Err(Error::MotorNotSupportedByCodec { .. })` and no socket has been opened

### Requirement: connect opens sockets, builds bus routes, and locks topology

`Robot::connect(&mut self)` SHALL, for each bus:

1. Initialize the underlying transport (call `transport.connect()` or analogous setup), exposing the socket fd.
2. Build the bus's recv-id routing table: iterate every group whose `bus_name` matches this bus, and for each motor in the group insert `bus.routes[motor.recv_id] = RouteKey { group_name, motor_index }`. If a motor's `recv_id` is already present in the routing table for this bus, return `Err(Error::CanIdCollision { bus_name, recv_id, existing: existing_route, attempted: new_route })` and abort connect.
3. Register the bus's fd with the `BusPoller` if non-`None`.

After all buses are processed, set an internal `connected` flag. Once `connected` is true, any subsequent call to a builder-style mutation (e.g. `Robot::add_group`) MUST return `Err(Error::TopologyLocked)`. The routing table population MUST be deterministic: groups are iterated in insertion order, motors within a group in insertion order.

#### Scenario: Successful connect populates per-bus routes

- **WHEN** `connect()` is called on a robot with one `vcan0` bus and one arm of three motors (recv_ids `0x11`, `0x12`, `0x13`)
- **THEN** the call returns `Ok(())`, `robot.is_connected()` returns `true`, and `robot.bus("main")?.routes` contains exactly three entries keyed by `0x11`, `0x12`, `0x13`

#### Scenario: CAN ID collision across groups on same bus fails connect

- **WHEN** `connect()` is called on a robot where arm has a motor with `recv_id = 0x18` and gripper on the same bus also has a motor with `recv_id = 0x18`
- **THEN** the call returns `Err(Error::CanIdCollision { bus_name: "main", recv_id: 0x18, existing: RouteKey { group_name: "arm", motor_index: <n> }, attempted: RouteKey { group_name: "gripper", motor_index: 0 } })`

#### Scenario: CAN ID collision across buses is not an error

- **WHEN** `connect()` is called on a robot where bus "left" has motor recv_id `0x11` and bus "right" also has motor recv_id `0x11`
- **THEN** the call returns `Ok(())` — collisions are per-bus, not global

#### Scenario: Add after connect rejected

- **WHEN** `robot.add_group(...)` is called after `connect()`
- **THEN** the call returns `Err(Error::TopologyLocked)`

### Requirement: enable / disable iterate over groups

`Robot::enable(&mut self)` SHALL call `group.enable_all()` on every group, in insertion order. `Robot::disable(&mut self)` MUST do the inverse (in reverse insertion order to support graceful shutdown semantics). Both MUST short-circuit on the first error and return it.

#### Scenario: Enable propagates to every group

- **WHEN** `robot.enable()` is called for a robot with two groups
- **THEN** the underlying mock bus has received one enable frame per motor across both groups
- **AND** the frames are ordered: group A motors in declaration order, then group B motors in declaration order

### Requirement: tick multiplexes across all buses with a deadline and dispatches via per-bus routes

`Robot::tick(&mut self, per_tick_deadline: Duration)` SHALL register each bus's `raw_fd` (when `Some`) with a `BusPoller`, call `wait(per_tick_deadline)`, then for every readable bus:

1. Call `bus.transport.drain_inbound_nonblocking()` to retrieve frames.
2. For each frame, call `bus.codec.decode(&frame)` **exactly once** to produce `Option<Event>`.
3. If `Some(event)`, look up `bus.routes[event.motor_id]`. If a `RouteKey { group_name, motor_index }` is found, call `robot.groups[&group_name].apply_event(motor_index, event)`. If no route, silently drop the event (the bus saw a frame for some motor the robot doesn't track).
4. If `Ok(None)` from decode, silently drop the frame (codec didn't recognize it).

The call MUST return `Ok(())` within at most `per_tick_deadline + epsilon` regardless of how many frames arrived. Buses returning `None` from `raw_fd` MUST be skipped with a log warning (out-of-band polling is out of v1 scope).

#### Scenario: Tick on quiet bus respects deadline

- **WHEN** `robot.tick(Duration::from_millis(2))` is called on a robot with quiet buses
- **THEN** the call returns `Ok(())` within 3ms

#### Scenario: Tick dispatches frame to matching motor in matching group

- **WHEN** a frame is injected on bus "main" whose `motor_id` matches motor index 2 of group "arm"
- **AND** `robot.tick(Duration::from_millis(2))` is called
- **THEN** `arm.motor_at(2)?.position()` reflects the injected frame's contents
- **AND** `bus.codec.decode` was called exactly once for that frame (verifiable via `MockMotorCodec::decode_call_count`)

#### Scenario: Tick with no matching route drops frame silently

- **WHEN** a frame is injected on bus "main" whose `motor_id` matches no motor in any group on "main"
- **AND** `robot.tick(Duration::from_millis(2))` is called
- **THEN** the call returns `Ok(())` and no group state is modified

#### Scenario: One decode per frame even with multiple groups on the bus

- **WHEN** bus "main" hosts both `arm` (4 motors) and `gripper` (1 motor)
- **AND** a single frame arrives whose `motor_id` matches one of arm's motors
- **THEN** `bus.codec.decode` is called exactly once (not once per group)

### Requirement: Sends are never coupled to inbound reads

No `Robot` or `MotorGroup` method other than `tick` SHALL call `drain_inbound_nonblocking` on any bus. Command-sending methods MUST only call `bus.transport.send`. This MUST be enforced by code review and by the absence of any other call site in the implementation.

#### Scenario: mit_control issues only send calls

- **WHEN** `arm.mit_control(&cmds)` is called for a three-motor arm on a bus whose transport is `MockCanBus`
- **THEN** `bus.transport.recorded_calls()` shows exactly three `send` calls and zero `drain_inbound_nonblocking` calls

### Requirement: Lifecycle methods are explicit and ordered

The user-visible lifecycle SHALL be exactly: `from_config` (or `builder().build()`) → `connect` → `enable` → (any number of `tick` + send/read) → `disable` → `drop`. Calling `enable` before `connect` MUST return `Err(Error::NotConnected)`. Calling `tick` before `connect` MUST return `Err(Error::NotConnected)`. Calling `disable` before `enable` MUST be a no-op (return `Ok(())`).

#### Scenario: tick before connect

- **WHEN** `robot.tick(Duration::from_millis(1))` is called on a freshly-built unconnected robot
- **THEN** the call returns `Err(Error::NotConnected)`

#### Scenario: Disable without enable is a no-op

- **WHEN** `robot.disable()` is called on a connected but never-enabled robot
- **THEN** the call returns `Ok(())` and no frames are sent

### Requirement: Drop closes all sockets

`Robot` SHALL implement `Drop` such that every bus's underlying socket is closed. The `Drop` impl MUST NOT panic on already-disabled motors and MUST NOT attempt to send disable frames (the user is responsible for explicit shutdown sequencing).

#### Scenario: Drop releases fds

- **WHEN** a connected robot is dropped
- **THEN** the file descriptors previously returned by `bus.raw_fd()` are closed (verifiable via `/proc/self/fd` inspection in tests)

### Requirement: Error enum is layered via thiserror

The `dm_control::Error` enum SHALL use `thiserror::Error` and cover at minimum: `Transport(TransportError)`, `Codec(CodecError)`, `ConfigSchema { .. }`, `ConfigIo(io::Error)`, `UnknownBusName(String)`, `UnknownVendor(String)`, `DuplicateBusName(String)`, `DuplicateGroupName(String)`, `MotorNotSupportedByCodec { vendor: String, motor_type: MotorTypeId, bus_name: String }`, `GripperRequiresOneMotor { got: usize }`, `CommandLengthMismatch { expected: usize, got: usize }`, `NotConnected`, `TopologyLocked`, `FdNotImplementedInV1 { bus_name: String }`, `CanIdCollision { bus_name: String, recv_id: u32, existing: RouteKey, attempted: RouteKey }`. The enum MUST be `#[non_exhaustive]`.

#### Scenario: TransportError wraps cleanly

- **WHEN** `bus.send(&frame)` returns `Err(TransportError::SendBufferFull)` during a `tick`
- **THEN** the call returns `Err(Error::Transport(TransportError::SendBufferFull))` and the original error variant is preserved

