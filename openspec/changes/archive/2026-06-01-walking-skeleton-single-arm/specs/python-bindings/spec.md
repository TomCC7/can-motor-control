## ADDED Requirements

### Requirement: can_motor_control package is importable after wheel install

A wheel produced by `maturin build --release` SHALL install a Python package named `can_motor_control` such that `import can_motor_control` succeeds on a fresh Python 3.10+ environment with only `numpy` already installed. The wheel MUST be self-contained (no runtime native dependencies beyond the system libc).

#### Scenario: Fresh-env import
- **WHEN** `pip install <wheel>` is run on a fresh venv that has only `numpy`
- **AND** `python -c "import can_motor_control; print(can_motor_control.__version__)"` is executed
- **THEN** the command prints the version and exits with status 0

### Requirement: Top-level types are exposed at can_motor_control

The `can_motor_control` namespace SHALL expose `Robot`, `RobotBuilder`, `SocketCanBus`, `MotorSpec`, `DmError` at the top level. Group wrapper classes (`Arm`, `Gripper`, `MotorGroup`) and `Motor` SHALL be exposed for type-checking purposes but typically obtained via accessor methods rather than constructed directly.

#### Scenario: Direct imports succeed
- **WHEN** `from can_motor_control import Robot, RobotBuilder, SocketCanBus, MotorSpec, DmError` is executed
- **THEN** the import succeeds without error

### Requirement: Vendor codecs are namespaced under can_motor_control.<vendor>

Vendor codec classes SHALL be exposed under a vendor-specific submodule, e.g. `can_motor_control.damiao.DamiaoCodec`, `can_motor_control.damiao.MotorType` (the Damiao SKU enum). Adding a future vendor MUST NOT collide with existing imports. Vendor codecs are passed to `RobotBuilder.add_bus` (alongside the transport); they are NOT passed to `add_arm` / `add_gripper` / `add_generic`, because codec ownership in v1 is per-bus, not per-group.

#### Scenario: Damiao codec accessible
- **WHEN** `from can_motor_control.damiao import DamiaoCodec, MotorType` is executed
- **THEN** the import succeeds and `MotorType.DM4340` is accessible

#### Scenario: RobotBuilder.add_bus accepts (name, transport, codec)
- **WHEN** Python code calls `can_motor_control.RobotBuilder().add_bus("main", bus, can_motor_control.damiao.DamiaoCodec())`
- **THEN** the call returns the builder for chaining and the codec's `bind_to_bus` has been invoked exactly once with the transport's capabilities

#### Scenario: RobotBuilder.add_arm does NOT accept a codec parameter
- **WHEN** Python code calls `builder.add_arm("arm", bus="main", codec=some_codec, motors=[...])` with a `codec` keyword
- **THEN** the call raises `TypeError` ("unexpected keyword argument 'codec'") — the signature is `add_arm(name, *, bus, motors)`

### Requirement: Robot supports the context manager protocol

`Robot.__enter__` SHALL call `connect()` then `enable()` and return `self`. `Robot.__exit__` MUST call `disable()`, swallowing exceptions raised during disable to avoid masking the original exception, but logging them via Python's `logging` module. Sockets close on garbage collection via the Rust `Drop` impl.

#### Scenario: with-block lifecycle
- **WHEN** Python code runs `with robot: arm = robot["arm"]; ...`
- **THEN** entering the block leaves `robot` in the enabled+connected state and exiting the block calls `disable()` exactly once

#### Scenario: Exception in body does not mask original
- **WHEN** an exception is raised inside a `with robot:` block and `disable()` also fails
- **THEN** the original body exception propagates and the disable failure is logged but not raised

### Requirement: Group access via __getitem__ returns the kind-dispatched wrapper

`Robot.__getitem__(name)` SHALL return an `Arm`, `Gripper`, or `MotorGroup` Python wrapper depending on the underlying `GroupKind` variant. A missing name MUST raise `KeyError(name)`. Attribute access (`robot.arm`) MUST NOT be implemented; users use only the `__getitem__` path to ensure typos fail loudly.

#### Scenario: Arm group returns Arm wrapper
- **WHEN** `arm = robot["arm"]` is called for a group constructed as `add_arm("arm", ...)`
- **THEN** `isinstance(arm, can_motor_control.Arm)` is true

#### Scenario: Missing group raises KeyError
- **WHEN** `robot["ghost"]` is called for a group not in the robot
- **THEN** the call raises `KeyError("ghost")`

#### Scenario: Attribute access not supported
- **WHEN** `robot.arm` is accessed (no underscore prefix, no attribute defined)
- **THEN** the access raises `AttributeError`

### Requirement: Batch state accessors return numpy arrays

`Arm.positions()`, `Arm.velocities()`, `Arm.torques()` SHALL each return a `numpy.ndarray` of dtype `float64` and shape `(N,)` where `N == len(arm)`. The arrays MUST be in motor-insertion order and MAY be read-only views into the underlying Rust per-motor state cache when memory layout permits, otherwise a contiguous copy.

#### Scenario: Shape and dtype
- **WHEN** `q = arm.positions()` is called for a four-motor arm
- **THEN** `q.shape == (4,)` and `q.dtype == np.float64`

#### Scenario: Order matches motor insertion
- **WHEN** an arm has motors `"j0", "j1", "j2"` (in that insertion order) with cached positions 0.1, 0.2, 0.3
- **THEN** `arm.positions()` equals `np.array([0.1, 0.2, 0.3])`

### Requirement: Batch send accepts shape-checked numpy arrays

`Arm.mit_control(cmds)` SHALL accept a `numpy.ndarray` of dtype `float64` and shape `(N, 5)` with columns `(kp, kd, q, dq, tau)`. `Arm.pos_vel_control` accepts shape `(N, 2)`. `Arm.vel_control` accepts shape `(N,)`. `Arm.pos_force_control` accepts shape `(N, 3)`. Wrong dtype MUST raise `TypeError`. Wrong shape MUST raise `ValueError` with a message naming the expected and actual shapes.

#### Scenario: Correct shape MIT batch
- **WHEN** `arm.mit_control(np.zeros((4, 5)))` is called for a four-motor arm
- **THEN** the call returns `None` and four frames are sent on the bus

#### Scenario: Wrong dtype rejected
- **WHEN** `arm.mit_control(np.zeros((4, 5), dtype=np.float32))` is called
- **THEN** the call raises `TypeError` with a message naming `float64`

#### Scenario: Wrong shape rejected
- **WHEN** `arm.mit_control(np.zeros((3, 5)))` is called for a four-motor arm
- **THEN** the call raises `ValueError` with a message naming `(4, 5)` expected vs `(3, 5)` got

### Requirement: Blocking methods release the GIL

`Robot.tick`, `Robot.enable`, `Robot.disable`, `Arm.mit_control`, `Arm.pos_vel_control`, `Arm.vel_control`, `Arm.pos_force_control`, and `Arm.enable_all` / `Arm.disable_all` SHALL release the GIL for the duration of their underlying syscalls via `Python::allow_threads`. Pure cache-read methods (`positions`, `velocities`, `torques`, `motor.position`, etc.) MAY hold the GIL since they do no IO.

#### Scenario: Concurrent Python thread can run during tick
- **WHEN** one Python thread runs a 100ms `robot.tick(100_000)` and another Python thread runs a tight Python loop incrementing a counter
- **THEN** the counter advances by a meaningful amount (>1) during the tick window, demonstrating GIL release

### Requirement: Errors raise a DmError hierarchy

All Rust `Error` variants SHALL be mapped to Python exceptions that subclass `can_motor_control.DmError`. The hierarchy MUST include at minimum: `DmError` (root), `TransportError`, `CodecError`, `ConfigError`, `LifecycleError`. Each subclass's `args` MUST include the underlying Rust error's stringified detail and any structured fields (e.g. `motor_type`).

#### Scenario: Transport failure raises TransportError
- **WHEN** a bus operation fails with `Err(TransportError::Io(_))` in Rust
- **THEN** the Python call raises `can_motor_control.TransportError` and the exception is also catchable as `can_motor_control.DmError`

#### Scenario: NotConnected raises LifecycleError
- **WHEN** `robot.tick(1000)` is called before `robot.connect()`
- **THEN** the call raises `can_motor_control.LifecycleError` with a message naming `NotConnected`

### Requirement: SocketCanBus(fd=True) raises an actionable error in v1

The `SocketCanBus.__init__` binding SHALL accept an `fd: bool = False` keyword argument. Calling `SocketCanBus(interface, fd=True)` in v1 MUST raise `can_motor_control.TransportError` whose message includes the phrase "CAN-FD is reserved for a future change; set fd=false". The error MUST be raised before any socket-opening syscall is issued.

#### Scenario: fd=False succeeds
- **WHEN** `SocketCanBus("vcan0", fd=False)` is constructed against a present `vcan0`
- **THEN** the call returns a bus instance and no exception is raised

#### Scenario: fd=True rejected
- **WHEN** `SocketCanBus("vcan0", fd=True)` is constructed
- **THEN** the call raises `can_motor_control.TransportError` with the substring "CAN-FD is reserved" in the message

### Requirement: TOML fd = true raises ConfigError at load time

`Robot.from_config(path)` SHALL raise `can_motor_control.ConfigError` if the TOML file contains any `[bus.<name>]` table with `fd = true`. The error message MUST name the offending bus and direct the user to set `fd = false`. The error MUST be raised before any socket is opened.

#### Scenario: TOML fd=true rejected
- **WHEN** `Robot.from_config(path)` is called with a TOML file containing `[bus.main] fd = true`
- **THEN** the call raises `can_motor_control.ConfigError` whose message names "main" and "fd = false"

### Requirement: Type stubs accompany the wheel

A `can_motor_control-stubs` package or in-wheel `.pyi` files SHALL provide static type signatures for the public API matching the surface specified above. Stubs MUST cover at minimum `Robot`, `RobotBuilder`, `Arm`, `Gripper`, `Motor`, `MotorSpec`, `SocketCanBus`, `DmError`, and the `can_motor_control.damiao` submodule.

#### Scenario: mypy resolves the public API
- **WHEN** `mypy` is run against a small Python script importing the documented public surface
- **THEN** mypy reports zero "Cannot find module" or "has no attribute" errors against `can_motor_control`

### Requirement: maturin build succeeds on Linux x86_64 stable

The crate `can-motor-control-py` SHALL build via `maturin build --release` on Linux x86_64 with stable Rust 1.75 or newer, producing a wheel tagged for the appropriate `manylinux` standard. The build MUST NOT require nightly Rust features.

#### Scenario: CI maturin build green
- **WHEN** CI runs `maturin build --release` on `ubuntu-latest`
- **THEN** the job completes successfully and uploads exactly one wheel artifact

### Requirement: Python smoke test runs against the MockCanBus

A Python smoke test SHALL exist that constructs a `Robot` with a `MockCanBus`, runs `enable`, executes a 10-tick control loop with MIT commands, asserts the mock bus received the expected frame count, and runs `disable` — all without requiring real CAN hardware. The test MUST run in CI.

#### Scenario: Smoke test green in CI
- **WHEN** CI runs `pytest tests/python/test_smoke.py`
- **THEN** the test passes and reports nonzero frames sent and nonzero frames received via the mock loopback
