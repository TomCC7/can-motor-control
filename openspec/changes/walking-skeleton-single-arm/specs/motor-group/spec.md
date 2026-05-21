## ADDED Requirements

### Requirement: Motor holds identity and a state cache

`Motor` SHALL expose `name() -> &str`, `motor_type() -> MotorTypeId`, `send_id() -> u32`, `recv_id() -> u32`, `position() -> f64`, `velocity() -> f64`, `torque() -> f64`, `temperature_mos() -> i16`, `temperature_rotor() -> i16`, `is_enabled() -> bool`, `fault() -> Option<FaultCode>`. State accessors MUST return the most recently dispatched value (zero on construction). Identity accessors MUST return the values supplied at construction.

#### Scenario: Newly constructed motor has zero state and known identity

- **WHEN** `Motor::new("j0", MotorTypeId::Damiao(d), 0x01, 0x11)` is constructed
- **THEN** `motor.name() == "j0"`, `motor.send_id() == 0x01`, `motor.recv_id() == 0x11`, `motor.position() == 0.0`, `motor.is_enabled() == false`

### Requirement: Motor state is updated only by group dispatch

`Motor` SHALL NOT expose any public `set_`* method for state fields. State MUST be updated exclusively by `MotorGroup` via crate-private interfaces called from the group's frame-dispatch path. The `is_enabled` flag MUST be set true on a successful enable acknowledgement and false on disable.

#### Scenario: User cannot mutate state directly

- **WHEN** a developer attempts `motor.position = 1.0` or `motor.set_position(1.0)`
- **THEN** the code fails to compile

#### Scenario: Dispatch updates state

- **WHEN** a `MotorGroup` dispatches an `Event::State { motor_id == motor.recv_id, q: 0.5, dq: 0.1, tau: 0.0, t_mos: 30, t_rotor: 35 }`
- **THEN** subsequent `motor.position() == 0.5`, `motor.velocity() == 0.1`, `motor.torque() == 0.0`, `motor.temperature_mos() == 30`, `motor.temperature_rotor() == 35`

### Requirement: MotorGroup references its bus by name; the codec lives on the bus

`MotorGroup` SHALL NOT own a codec. It MUST reference its bus by string key (`bus_name: String`); the codec is owned by the bus and is shared across every group on that bus. This means one codec instance per bus, regardless of how many groups (arm, gripper, etc.) attach to it. Two consequences follow: every group on a bus MUST use the bus's vendor, and inbound frames on a bus are decoded exactly once per frame (by the bus's codec), then routed to the owning group via the bus's recv-id routing table.

Send operations from group methods MUST encode via `bus.codec.encode_*` and transmit via `bus.transport.send`. The group code MUST NOT branch on classical vs CAN-FD frame type; the unified `CanFrame` type handles both transparently.

#### Scenario: Group records its bus name

- **WHEN** a group is constructed with `bus_name = "left"`
- **THEN** `group.bus_name() == "left"`

#### Scenario: Group does not hold a codec field

- **WHEN** the `MotorGroup`, `Arm`, `Gripper`, `Generic` struct definitions are inspected
- **THEN** none of them contain a `codec` field of type `Box<dyn MotorCodec>` or any equivalent

#### Scenario: Sending requires Robot context

- **WHEN** an isolated `MotorGroup` (not yet added to a `Robot`) attempts to send
- **THEN** the API requires the caller go through `Robot`'s borrow-splitting access (e.g. an `ArmHandle` that holds simultaneous mutable references to the arm and to its bus), since the group cannot encode/send without the bus's codec

#### Scenario: Group code contains no CAN/CAN-FD discrimination

- **WHEN** the implementation of `Arm`, `Gripper`, `Generic`, and `MotorGroup` is inspected
- **THEN** no method body references `flags.contains(FD_FORMAT)`, `is_fd()`, or matches on classical vs FD frame variants; frame-type handling is confined to the codec and bus impls

### Requirement: Motor membership is validated against the bus's codec at add time

When a motor is added to a group (via `RobotBuilder::add_arm`, `add_gripper`, `add_generic`, or via the TOML loader), the builder MUST look up the group's bus and consult `bus.codec.supports(motor.motor_type)`. If `supports` returns `false`, the add MUST fail with `Error::MotorNotSupportedByCodec { vendor, motor_type, bus_name }`. This check MUST happen at builder time, before `connect()`. If the referenced bus does not exist (or has not been added yet), the call MUST fail with `Error::UnknownBusName(name)`.

#### Scenario: Adding a Damiao motor to a Damiao-vendor bus succeeds

- **WHEN** `add_bus("main", transport, Box::new(DamiaoCodec::new()))` is followed by `add_arm("arm", "main", vec![MotorSpec::new("j0", DM4340, ...)])`
- **THEN** the call succeeds and the motor count increases by one

#### Scenario: Adding a Robostride motor to a Damiao-vendor bus fails

- **WHEN** `add_arm("arm", "main", vec![MotorSpec::new("j0", MotorTypeId::Robostride(0), ...)])` is called on a bus whose codec is `DamiaoCodec`
- **THEN** the call returns `Err(Error::MotorNotSupportedByCodec { vendor: "damiao", motor_type: MotorTypeId::Robostride(0), bus_name: "main" })`

#### Scenario: Adding a group to a nonexistent bus fails

- **WHEN** `add_arm("arm", "ghost", motors)` is called and no bus named "ghost" has been added
- **THEN** the call returns `Err(Error::UnknownBusName("ghost"))`

### Requirement: MotorGroup provides name-based and index-based motor access

`MotorGroup` SHALL expose `len() -> usize`, `motor(name: &str) -> Option<&Motor>`, `motor_mut(name: &str) -> Option<&mut Motor>`, `motor_at(idx: usize) -> Option<&Motor>`. Motors MUST be stored in a `Vec<Motor>` in insertion order, with a parallel `HashMap<String, usize>` for name lookup. Iteration MUST yield motors in insertion order.

#### Scenario: Name and index access agree

- **WHEN** motors `"j0"`, `"j1"`, `"j2"` are added in that order
- **THEN** `group.motor("j1").map(Motor::name) == Some("j1")` and `group.motor_at(1).map(Motor::name) == Some("j1")`

#### Scenario: Missing name returns None

- **WHEN** `group.motor("nonexistent")` is called
- **THEN** the call returns `None`

### Requirement: GroupKind enum dispatches to Arm / Gripper / Generic

A `GroupKind` enum SHALL exist with at least three variants: `Arm(Arm)`, `Gripper(Gripper)`, `Generic(MotorGroup)`. The enum MUST be `#[non_exhaustive]`. Each variant MUST own its specialized newtype wrapping a `MotorGroup`. `GroupKind` MUST expose `as_arm()`, `as_arm_mut()`, `as_gripper()`, `as_gripper_mut()`, `as_generic()` returning `Option<&...>` / `Option<&mut ...>`.

#### Scenario: Downcast to correct variant

- **WHEN** a `GroupKind::Arm(arm)` is matched via `kind.as_arm()`
- **THEN** the call returns `Some(&arm)`
- **AND** `kind.as_gripper()` returns `None`

### Requirement: Arm exposes batch command and batch state-view APIs

`Arm` SHALL expose `positions() -> &[f64]`, `velocities() -> &[f64]`, `torques() -> &[f64]`, each returning a slice in insertion order of length `arm.len()`. It MUST expose `mit_control(&mut self, cmds: &[MitCmd])`, `pos_vel_control(&mut self, cmds: &[PosVelCmd])`, `vel_control(&mut self, cmds: &[VelCmd])`, `pos_force_control(&mut self, cmds: &[PosForceCmd])`. The length of `cmds` MUST equal `arm.len()`; a mismatch MUST return `Err(Error::CommandLengthMismatch { expected, got })`.

#### Scenario: Batch positions length matches motor count

- **WHEN** an arm with three motors is queried via `arm.positions()`
- **THEN** the returned slice has length 3 in insertion order

#### Scenario: MIT batch length mismatch

- **WHEN** `arm.mit_control(&cmds)` is called with `cmds.len() != arm.len()`
- **THEN** the call returns `Err(Error::CommandLengthMismatch { expected: arm.len(), got: cmds.len() })`
- **AND** no frames are sent on the bus

### Requirement: Arm batch send encodes per-motor via the bus's codec

For each motor in insertion order, `Arm` SHALL invoke `bus.codec.encode_command` with the motor's reference and the corresponding command, then call `bus.transport.send(&frame)` once per motor. The send sequence MUST stop on the first send error and return it; motors not yet sent in that call MUST NOT have their state interpreted as commanded. `bus` here refers to the `Bus` struct that bundles transport and codec, accessed by the group's `bus_name`.

#### Scenario: Send error halts batch

- **WHEN** the bus errors on the second motor of a three-motor MIT batch
- **THEN** `mit_control` returns `Err(Error::Transport(_))`
- **AND** exactly two `send` calls have been made on the bus (one successful, one failed)

### Requirement: Gripper is a v1-minimal one-motor newtype

`Gripper` SHALL wrap a `MotorGroup` whose motor count is exactly one. Construction (via `RobotBuilder::add_gripper`) MUST fail with `Error::GripperRequiresOneMotor { got }` if a non-one motor count is supplied (the API takes a single `MotorSpec`, so this enforcement is type-level for the public surface; for the internal constructor that takes `Vec<MotorSpec>` for symmetry with arm/generic, the check is runtime). In v1, `Gripper` MUST expose `motor() -> &Motor`, `motor_mut() -> &mut Motor`, `enable() -> Result<(), Error>`, `disable() -> Result<(), Error>`, `mit_control(MitCmd) -> Result<(), Error>`, `pos_vel_control(PosVelCmd) -> Result<(), Error>`. It MUST NOT expose `open`, `close`, `set_force`, or `calibrate` methods in v1 (reserved for a follow-up change). Like `Arm`, `Gripper` does not own a codec; it encodes via its bus's codec.

#### Scenario: Construction with one motor succeeds

- **WHEN** `RobotBuilder::add_gripper("g", "main", MotorSpec::new("g", DM4310, 0x08, 0x18))` is called on a builder whose "main" bus uses `DamiaoCodec`
- **THEN** `build()` succeeds and the resulting `Gripper`'s `motor().name() == "g"`

#### Scenario: Internal constructor rejects two motors

- **WHEN** the internal `Gripper::from_motors(motors_vec)` is called with two MotorSpecs
- **THEN** the call returns `Err(Error::GripperRequiresOneMotor { got: 2 })`

#### Scenario: v1-reserved methods absent

- **WHEN** a developer attempts `gripper.open(0.5)` in v1
- **THEN** the code fails to compile

### Requirement: Group lifecycle commands batch over all motors

`enable_all()`, `disable_all()`, `set_zero_all()` SHALL iterate motors in insertion order, encoding the appropriate command via `bus.codec.encode_enable` / `encode_disable` / `encode_set_zero` and sending each frame via `bus.transport.send`. `enable_all` MUST set `motor.is_enabled = true` upon receiving the corresponding state acknowledgement from each motor (the dispatch path observes this). `disable_all` MUST clear the flag analogously.

#### Scenario: enable_all emits one frame per motor

- **WHEN** `arm.enable_all()` is called for a three-motor arm whose bus uses `MockCanBus` with `DamiaoCodec`
- **THEN** `bus.transport.sent_frames()` contains exactly three frames in motor insertion order
- **AND** each frame matches `bus.codec.encode_enable` output for the corresponding motor

### Requirement: Apply-event-to-motor is the dispatch entry point on groups

The `MotorGroup` SHALL provide an `apply_event(&mut self, motor_index: usize, event: Event)` method (crate-private to `dm-control`). It MUST update the motor at the supplied index according to the event variant. Decoding the inbound frame and resolving the correct group + motor_index is the bus router's responsibility (see `robot-composition` spec, "Connect builds per-bus recv_id routing table"); the group MUST NOT itself call `decode`.

#### Scenario: apply_event updates the targeted motor

- **WHEN** `group.apply_event(2, Event::State { motor_id: <recv_id of motor 2>, q: 0.5, dq: 0.1, tau: 0.0, t_mos: 30, t_rotor: 35 })` is called
- **THEN** `group.motor_at(2)?.position() == 0.5`, `.velocity() == 0.1`, etc.

#### Scenario: apply_event with mismatched index panics in debug, no-ops in release

- **WHEN** `group.apply_event(motor_index_out_of_range, event)` is called
- **THEN** in debug builds the call panics with an index assertion; in release builds the call is a no-op (the router should never produce out-of-range indices because it is built from the group's own motor list at connect)

