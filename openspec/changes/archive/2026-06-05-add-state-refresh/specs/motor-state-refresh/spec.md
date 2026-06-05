## ADDED Requirements

### Requirement: Codec exposes an optional state-refresh encoder

The `MotorCodec` trait SHALL provide `encode_refresh(&self, motor: MotorRef) ->
Result<Option<CanFrame>, CodecError>` with a default implementation returning
`Ok(None)` (refresh unsupported). A codec MAY override it to return `Ok(Some(frame))`
where `frame` requests current motor state **without commanding motion**. Callers
SHALL treat `Ok(None)` as "this motor's codec has no refresh" and skip it. The
method MUST NOT require downcasting to a vendor-specific type.

#### Scenario: Default codec reports no refresh
- **WHEN** `encode_refresh` is called on a codec that does not override it
- **THEN** it returns `Ok(None)`

#### Scenario: Vendor codec returns a refresh frame
- **WHEN** `encode_refresh` is called on a codec that supports it
- **THEN** it returns `Ok(Some(frame))` and the frame commands no motion

### Requirement: Damiao implements refresh as the 0xCC state query

`DamiaoCodec::encode_refresh` SHALL return a classical frame on CAN ID `0x7FF`
with payload `[send_id & 0xFF, (send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0]` —
the Damiao `refresh_motor_status` query, byte-for-byte matching the openarm
reference `create_refresh_command`. This frame requests a feedback frame and
applies no torque.

#### Scenario: Refresh frame layout
- **WHEN** `DamiaoCodec::encode_refresh` is called for a motor with send id `0x01`
- **THEN** the frame has id `0x7FF`, length 8, and payload `[0x01, 0x00, 0xCC, 0, 0, 0, 0, 0]`

### Requirement: Group and Robot send one refresh per motor

`MotorGroup::refresh_all()` (and `Arm`/`Gripper`/`Robot::refresh()`) SHALL, for
each motor in declaration order, encode a refresh via the bus's codec and, when
`Some`, send exactly one frame via the bus transport. Motors whose codec returns
`None` SHALL be skipped. Refresh is **send-only**: it MUST NOT drain or read
inbound frames (consistent with the "sends are never coupled to reads" rule —
`tick` remains the sole inbound reader).

#### Scenario: Refresh sends one frame per supported motor
- **WHEN** `refresh()` is called on an arm of three Damiao motors backed by a mock bus
- **THEN** the bus records exactly three `send` calls and zero `drain_inbound_nonblocking` calls

#### Scenario: Unsupported motor is skipped
- **WHEN** `refresh()` is called and a motor's codec returns `Ok(None)`
- **THEN** no frame is sent for that motor and no error is raised

#### Scenario: Refresh before connect is rejected
- **WHEN** `refresh()` is called before `Robot::connect()`
- **THEN** it returns `Err(Error::NotConnected)`

### Requirement: Refresh replies update motor state via the normal path

A motor's reply to a refresh query SHALL be decoded and dispatched to the motor's
state cache exactly like a reply to a control command — no new decode path is
required. A loop of `refresh()` then `tick(deadline)` SHALL therefore yield
continuously-updating state without commanding motion.

#### Scenario: refresh then tick updates state
- **WHEN** `arm.refresh()` is called, the motor replies with a feedback frame, and `robot.tick(deadline)` runs
- **THEN** the motor's `position`/`velocity`/`torque` reflect the feedback values

### Requirement: Python exposes refresh

The Python bindings SHALL expose `Arm.refresh()`, `Gripper.refresh()`, and
`Robot.refresh()` that invoke the corresponding Rust methods and release the GIL
for the underlying sends (as `tick` does). The `.pyi` stubs SHALL declare them.

#### Scenario: Python read loop stays fresh
- **WHEN** a Python loop repeatedly calls `arm.refresh()` then `robot.tick(deadline)`
- **THEN** `arm.positions()` returns updated values each cycle and no motion command is sent
