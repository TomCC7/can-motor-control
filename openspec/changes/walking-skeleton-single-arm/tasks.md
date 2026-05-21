> **Test gating rule.** Every implementation task (`Impl:`) is paired with a test task (`Test:`) immediately following it. The implementation task is NOT considered done until its paired test task is green. Test tasks reference the spec scenarios they cover. Whole task groups (`## N.`) are gated on every task in that group being green before downstream groups start, except where explicit dependencies allow parallelism (called out in the task text).

## 1. Workspace and CI scaffolding

- 1.1 Impl: create Cargo workspace at repo root with members `crates/motor-codec`, `crates/dm-codec`, `crates/dm-control`, `crates/dm-control-py`; pin `rust-toolchain.toml` to stable 1.75+; populate workspace `[dependencies]` with shared crates (`thiserror`, `serde`, `toml`, `bitflags`, `mio`, `socketcan`, `pyo3`, `numpy`, `nix` or `libc`)
- 1.2 Test: `cargo metadata --format-version 1` parses successfully; `cargo build --workspace` completes against the empty crates; `cargo tree -p dm-control | grep -v dm-codec` confirms no vendor dependency leak
- 1.3 Impl: add `.github/workflows/ci.yml` with jobs for `cargo fmt --check`, `cargo clippy --workspace -- -D warnings`, `cargo test --workspace`, and `cargo build -p motor-codec --no-default-features --target thumbv7em-none-eabihf` (no_std verification)
- 1.4 Test: push a trivial commit; verify CI runs all jobs green on `ubuntu-latest`
- 1.5 Impl: add `maturin` build job in CI building `dm-control-py` wheel; add `pytest` job running Python tests against built wheel in a venv
- 1.6 Test: CI maturin job produces exactly one wheel artifact tagged for `manylinux_2_28_x86_64` or compatible; pytest job runs with at least one passing placeholder test
- 1.7 Impl: add top-level `README.md` with project description, installation instructions placeholder, link to `openspec/` for the source of truth
- 1.8 Test: README links resolve; no broken cross-references in the openspec dir

## 2. motor-codec crate (foundation, no_std)

- 2.1 Impl: `#![no_std]` declaration; add `alloc` extern crate where needed; configure `Cargo.toml` with `default-features = []` and explicit feature flags for `std` (default off)
- 2.2 Test: `cargo build -p motor-codec --no-default-features` succeeds; `cargo test -p motor-codec` runs (allow `std` feature for tests)
- 2.3 Impl: `FrameFlags` bitflags type with `EXTENDED_ID`, `FD_FORMAT`, `BIT_RATE_SWITCH`, `ERROR_STATE`, `REMOTE_REQUEST`; derive `Copy`, `Clone`, `Debug`, `Default`, `PartialEq`, `Eq`
- 2.4 Test: covers can-transport spec "FrameFlags enumerates CAN frame metadata bits" scenarios — all flag bits accessible; FD_FORMAT + REMOTE_REQUEST combination rejected
- 2.5 Impl: `BusCapabilities` struct with `supports_fd: bool`, `max_payload_len: u8`; `BusCapabilities::classical()` and `::fd()` constructors enforcing the `max_payload_len` invariant
- 2.6 Test: covers can-transport spec "BusCapabilities describes runtime transport features" scenarios — classical caps yield max_payload_len 8, fd caps yield 64
- 2.7 Impl: `CanFrame` struct (id, flags, len, `[u8; 64]` data); constructors `classical(id, &[u8])`, `classical_extended(id, &[u8])`, `fd(id, &[u8])`; helpers `is_fd()`, `payload()`, `is_extended()`; `is_valid_fd_dlc()` helper for FD length validation
- 2.8 Test: covers can-transport spec "CanFrame is a single unified frame representation" scenarios — classical 8-byte, classical >8 fails, FD 16-byte, FD invalid DLC fails, `is_fd()` reflects flag
- 2.9 Impl: `MotorTypeId` enum (`#[non_exhaustive]`) with `Damiao(u16)` variant; `MotorRef<'a>` struct with motor_type, send_id, recv_id, name
- 2.10 Test: covers motor-codec spec "MotorTypeId discriminates motors by vendor" scenarios — Damiao discriminant carrying, non-exhaustive matching, fallback through `_`
- 2.11 Impl: `Command` enum (`Mit { kp, kd, q, dq, tau }`, `PosVel { q, dq }`, `Vel { dq }`, `PosForce { q, dq, i_pu }`); `CommandKind` enum for error reporting
- 2.12 Test: covers motor-codec spec "Command enum captures the shared control-mode surface" — all four variants constructible; Debug round-trip
- 2.13 Impl: `Event` enum (`State { motor_id, q, dq, tau, t_mos, t_rotor }`, `ParamReply { motor_id, rid, value }`, `Fault { motor_id, code }`); `ParamValue` union (`Float(f64)` | `UInt(u32)`)
- 2.14 Test: covers motor-codec spec "Event enum captures decoded inbound frames" — State variant carries all five fields with correct types
- 2.15 Impl: `Limits` struct (p_max, v_max, t_max as f64); `FrameError` enum (PayloadTooLong, InvalidFdLength, IncompatibleFlags) via `thiserror`
- 2.16 Test: each error variant renders an informative Display string
- 2.17 Impl: `MotorCodec` trait with `vendor_name`, `supports`, `limits`, `bind_to_bus`, `encode_enable`, `encode_disable`, `encode_set_zero`, `encode_command`, `decode`; require `Send + Sync`; verify object-safety via `let _: Box<dyn MotorCodec>` line in doc test
- 2.18 Test: covers motor-codec spec "MotorCodec trait is the vendor-agnostic codec contract" — trait object-safe, dm-control has no vendor dep (asserted by cargo tree filter in CI)
- 2.19 Impl: `CodecError` enum via `thiserror` with `UnknownMotorType`, `CommandNotSupported`, `DecodeFailed`, `OutOfRange`; `#[non_exhaustive]`
- 2.20 Test: covers motor-codec spec "CodecError is typed via thiserror" — OutOfRange returned for over-range tau (will be exercised in §3 when DamiaoCodec is implemented; placeholder unit test now confirms variant constructible)

## 3. dm-codec crate (Damiao implementation, no_std)

- 3.1 Impl: `#![no_std]` crate skeleton depending on `motor-codec`; `DamiaoCodec` struct with internal `bound_caps: Option<BusCapabilities>`
- 3.2 Test: covers dm-codec spec "DamiaoCodec implements MotorCodec" and "dm-codec crate is no_std" — constructible, trait-object-compatible, builds on `thumbv7em-none-eabihf`
- 3.3 Impl: `DamiaoMotorType` enum with all 13 SKU variants; `From<DamiaoMotorType> for MotorTypeId`; `parse_motor_type(&str)` returning `Option<MotorTypeId>`
- 3.4 Test: covers dm-codec spec "Damiao motor type strings parse to MotorTypeId" — all 13 known SKU strings round-trip; "DM_DOES_NOT_EXIST" returns None
- 3.5 Impl: limit-param table for all 13 SKUs with (p_max, v_max, t_max) values from Damiao's protocol manual; `supports()`, `limits()` implementations consulting the table
- 3.6 Test: covers dm-codec spec "Supports the Damiao motor SKUs targeted by v1" — every listed SKU has limits with positive values; DM4340 returns (12.5, 10.0, 28.0); unknown discriminant rejected; `supports(Robostride(0))` returns false
- 3.7 Impl: bit-pack utilities `float_to_uint(x, min, max, bits) -> u16`, `uint_to_float(u, min, max, bits) -> f64`, `pack_mit_payload`, `unpack_mit_payload`
- 3.8 Test: utilities round-trip f64 → bits → f64 within ±1 LSB for each of (16, 12, 12, 12, 12) bit widths; out-of-range inputs clamp to the boundary value
- 3.9 Impl: `encode_command(Command::Mit)` per dm-codec spec bit layout; `bind_to_bus` stored but unused
- 3.10 Test: covers dm-codec spec "MIT command encoding matches Damiao bit layout" — zero MIT on DM4340 produces the expected payload bytes (compare against reference Python implementation cycled through the same scaling); out-of-range torque returns `Err(CodecError::OutOfRange { field: "tau" })`
- 3.11 Impl: `encode_command(Command::PosVel)` with `id = 0x100 + send_id`, `id = 0x200 + send_id` for Vel, `id = 0x300 + send_id` for PosForce; integer-scale dq*100 and i_pu*10000 for PosForce
- 3.12 Test: covers dm-codec specs "PosVel uses CAN ID offset 0x100", "Vel uses CAN ID offset 0x200", "PosForce uses CAN ID offset 0x300" — frame IDs and payload byte layouts match the spec verbatim
- 3.13 Impl: `encode_enable`, `encode_disable`, `encode_set_zero` producing the `[0xFF; 7] ++ [0xFC|0xFD|0xFE]` patterns
- 3.14 Test: covers dm-codec spec "encode_enable / encode_disable / encode_set_zero use the special command pattern" — each frame has correct ID, len 8, payload pattern
- 3.15 Impl: `decode` parsing state-response frames with command byte `0x11`; unpack q, dq, tau via limit-scaled `uint_to_float`; unpack t_mos, t_rotor from dedicated bytes; return `Ok(None)` for foreign frames; `Err(CodecError::DecodeFailed)` for malformed Damiao-shaped frames
- 3.16 Test: covers dm-codec spec "decode parses Damiao state response frames" — round-trip MIT state frame; non-Damiao frame returns Ok(None); truncated payload returns DecodeFailed
- 3.17 Impl: `DamiaoRid` enum (`#[non_exhaustive]`) with all documented register IDs; `From<DamiaoRid> for u8`
- 3.18 Test: covers dm-codec spec "DamiaoRid enumerates the parameter register IDs" — CTRL_MODE → expected byte value
- 3.19 Impl: `DamiaoCodecExt` trait with `encode_read_param`, `encode_write_param`, `encode_save_to_flash`, `encode_refresh`; impl for `DamiaoCodec`
- 3.20 Test: covers dm-codec spec "DamiaoCodecExt exposes the 0x7FF parameter sub-protocol" — write_param frame layout: id 0x7FF, payload bytes match spec; read_param uses 0x33; save uses 0xAA; refresh uses 0xCC
- 3.21 Impl: assert via test that `DamiaoCodec` emits classical frames regardless of bound bus FD capability (use a `MockBusCaps` value to bind FD-capable caps; assert emitted frames have `FD_FORMAT` unset)
- 3.22 Test: covers dm-codec spec "DamiaoCodec emits classical CAN frames in v1 regardless of bus capability" — both classical-bound and (mock) FD-bound paths produce non-FD frames

## 4. dm-control: transport layer

- 4.1 Impl: `CanBus` trait (object-safe, `Send`) with `name`, `capabilities`, `send`, `drain_inbound_nonblocking`, `raw_fd`; `TransportError` enum (`#[non_exhaustive]`) via `thiserror` covering all variants listed in can-transport spec
- 4.2 Test: covers can-transport specs "CanBus trait defines the transport contract" and "Transport errors are typed via thiserror" — `Box<dyn CanBus>` compiles; trait surface contains exactly the documented methods; `FdNotImplementedInV1` Display includes "CAN-FD is reserved for a future change; set fd=false"
- 4.3 Impl: `MockCanBus` with configurable `BusCapabilities` (`with_capabilities`), in-memory loopback paired by channel, `inject_frame`, `sent_frames`, `recorded_calls` for test inspection
- 4.4 Test: covers can-transport spec "MockCanBus provides an in-memory loopback for tests" — loopback delivers; inject + drain round-trip; sent_frames recording; FD frame on classical mock returns `FdFrameOnNonFdBus`
- 4.5 Impl: `SocketCanBus::open(interface, fd_enabled)`; `fd_enabled = true` returns `Err(FdNotImplementedInV1)` before opening anything; otherwise `socket(PF_CAN, SOCK_RAW, CAN_RAW)`, `ioctl(SIOCGIFINDEX)`, `bind`, set non-blocking; `capabilities()` returns classical-only
- 4.6 Test: covers can-transport spec "SocketCanBus binds to a Linux SocketCAN interface" and "fd_enabled=true rejected in v1" — vcan0 open succeeds; nonexistent interface fails with `InterfaceNotFound`; fd=true fails before socket open (verify via fd-count delta in /proc/self/fd); capabilities reports classical
- 4.7 Impl: `SocketCanBus::send` with capability validation (FD frame on non-FD bus, payload too long, extended ID unsupported); classical kernel write via `can_frame` layout; retry on `EAGAIN` up to bounded count
- 4.8 Test: covers can-transport spec "send validates the frame against bus capabilities" and "send writes a single classical frame without blocking on inbound" — quiet vcan0 round-trip; torn-down interface returns Io error; FD frame on classical bus returns `FdFrameOnNonFdBus`; oversized payload returns `PayloadExceedsBusCapacity`
- 4.9 Impl: `SocketCanBus::drain_inbound_nonblocking` reading from kernel until EAGAIN; detect FD vs classical by struct length; populate `FrameFlags` accordingly
- 4.10 Test: covers can-transport spec "drain_inbound_nonblocking returns immediately and detects FD format" — quiet bus returns empty vec under 1ms; three frames returned in arrival order; synthetic injected canfd_frame bytes decode with `FD_FORMAT` set (unit test on the parsing logic in isolation, since vcan0 sockets in v1 are classical-only)
- 4.11 Impl: `BusPoller` wrapping `mio::Poll`; methods `register(token, raw_fd)`, `wait(deadline) -> Vec<Token>`; handle `EINTR` correctly
- 4.12 Test: covers can-transport spec "Multiple buses can be multiplexed via poll(2)" — wake on first readable bus; deadline expires on quiet buses within deadline+1ms

## 5. dm-control: Motor, MotorGroup, Arm, Gripper, Generic

- 5.1 Impl: `Motor` struct (identity fields private; state fields with crate-private setters via `pub(crate)`); public getters as specified; `FaultCode` placeholder enum
- 5.2 Test: covers motor-group spec "Motor holds identity and a state cache" and "Motor state is updated only by group dispatch" — newly constructed Motor has zero state; `pub(crate)` setters update; user code cannot mutate state (verify via `trybuild` compile-fail test)
- 5.3 Impl: `MotorGroup` base with `motors: Vec<Motor>`, `by_name: HashMap<String, usize>`, `bus_name: String`; `len`, `motor`, `motor_mut`, `motor_at`, `bus_name` methods; NO codec field
- 5.4 Test: covers motor-group specs "MotorGroup references its bus by name; the codec lives on the bus" and "MotorGroup provides name-based and index-based motor access" — group does not hold codec field (inspected via reflection-style test using `std::mem::size_of` or field grep in test code); name/index access agree
- 5.5 Impl: `apply_event(motor_index, Event)` crate-private; updates motor state on `Event::State`, fault on `Event::Fault`, parameter dict on `Event::ParamReply`
- 5.6 Test: covers motor-group spec "Apply-event-to-motor is the dispatch entry point on groups" — apply_event updates targeted motor; out-of-range index panics in debug
- 5.7 Impl: `GroupKind` enum (`#[non_exhaustive]`) with `Arm(Arm)`, `Gripper(Gripper)`, `Generic(MotorGroup)`; `as_arm`/`as_gripper`/`as_generic` plus `_mut` variants
- 5.8 Test: covers motor-group spec "GroupKind enum dispatches to Arm / Gripper / Generic" — downcast to correct variant; wrong variant returns None
- 5.9 Impl: `Arm` newtype wrapping `MotorGroup`; `len`, `motor` accessors; `positions/velocities/torques` slice views; `mit_control`, `pos_vel_control`, `vel_control`, `pos_force_control` with length validation; helpers that internally invoke `bus.codec.encode_`* + `bus.transport.send` via a bus resolver
- 5.10 Test: covers motor-group specs "Arm exposes batch command and batch state-view APIs", "Arm batch send encodes per-motor via the bus's codec", and "Group code contains no CAN/CAN-FD discrimination" — batch state length matches motor count; MIT length mismatch returns `CommandLengthMismatch` with no frames sent; bus errors halt batch after first failure; grep test confirms no `is_fd()` / `FD_FORMAT` references in Arm method bodies
- 5.11 Impl: `Gripper` newtype enforcing one-motor invariant at internal constructor; `motor`, `motor_mut`, `enable`, `disable`, `mit_control(MitCmd)`, `pos_vel_control(PosVelCmd)`; intentionally no `open`/`close`/`set_force`/`calibrate`
- 5.12 Test: covers motor-group spec "Gripper is a v1-minimal one-motor newtype" — one-motor construction succeeds; two-motor construction returns `GripperRequiresOneMotor { got: 2 }`; v1-reserved methods don't exist (compile-fail test via `trybuild` confirming `gripper.open(0.5)` fails)
- 5.13 Impl: `enable_all`, `disable_all`, `set_zero_all` on `MotorGroup` (delegated to via Arm/Gripper/Generic); encode each via `bus.codec`, send via `bus.transport`
- 5.14 Test: covers motor-group spec "Group lifecycle commands batch over all motors" — three-motor arm enable produces three frames in motor insertion order via MockCanBus's `sent_frames()` snapshot; each frame matches `bus.codec.encode_enable` output

## 6. dm-control: Bus, Robot, RobotBuilder, lifecycle, tick

- 6.1 Impl: `Bus` struct with `transport: Box<dyn CanBus>`, `codec: Box<dyn MotorCodec>`, `routes: HashMap<u32, RouteKey>`; `vendor`, `capabilities`, `codec_supports` accessors; `RouteKey { group_name, motor_index }`
- 6.2 Test: covers robot-composition spec "Bus struct bundles a transport and a vendor codec" — `bus.vendor()` and `bus.capabilities()` round-trip; one codec instance per bus regardless of group count
- 6.3 Impl: `Robot` struct with `buses: HashMap<String, Bus>`, `groups: HashMap<String, GroupKind>`, `connected: bool`, plus `group_names`/`bus_names`/`is_connected` accessors; `__getitem__`-equivalent `group`/`group_mut` returning `Option<&GroupKind>`
- 6.4 Test: name listing returns insertion order; missing group returns None
- 6.5 Impl: `RobotBuilder` with `new`, `add_bus(name, transport, codec)`, `add_arm(name, bus, motors)`, `add_gripper(name, bus, motor)`, `add_generic(name, bus, motors)`, `build`; `add_bus` invokes `codec.bind_to_bus(transport.capabilities())` exactly once
- 6.6 Test: covers robot-composition specs "Robot owns named buses and named groups", "RobotBuilder is the canonical construction path", and motor-codec spec "bind_to_bus" scenarios — duplicate bus/group names rejected; add_arm with unknown bus name returns `UnknownBusName`; `bind_to_bus` invoked exactly once at add_bus time and NOT re-invoked on add_arm/add_gripper (verify via `MockMotorCodec::bind_count`)
- 6.7 Impl: builder-time validation — name non-emptiness, motor membership via `bus.codec.supports`, gripper one-motor rule; all errors returned from `build()` before any IO
- 6.8 Test: covers motor-group spec "Motor membership is validated against the bus's codec at add time" and robot-composition spec "Validation runs before any IO" — Damiao bus + Damiao motor succeeds; Damiao bus + Robostride motor returns `MotorNotSupportedByCodec { vendor: "damiao", motor_type: Robostride(0), bus_name }`; from_config on host without CAN interfaces returns Ok
- 6.9 Impl: `Robot::connect` — initialize each transport (if not already), walk groups, populate per-bus recv_id → RouteKey table; detect collisions; set `connected` flag
- 6.10 Test: covers robot-composition spec "connect opens sockets, builds bus routes, and locks topology" — successful connect builds routes for three-motor arm; CAN ID collision across groups on same bus returns `CanIdCollision { bus_name, recv_id, existing, attempted }`; same recv_id on different buses succeeds; add after connect returns `TopologyLocked`
- 6.11 Impl: `Robot::enable` iterates groups in insertion order calling `group.enable_all()`; `Robot::disable` iterates in reverse insertion order; both short-circuit on first error
- 6.12 Test: covers robot-composition spec "enable / disable iterate over groups" — enable propagates one frame per motor across all groups in insertion order (assert via MockCanBus `sent_frames()`); disable runs reverse order
- 6.13 Impl: `Robot::tick(deadline)` using `BusPoller`; for each readable bus call drain, then for each frame call `bus.codec.decode` once, look up `bus.routes[event.motor_id]`, dispatch via `groups[group_name].apply_event(motor_index, event)`
- 6.14 Test: covers robot-composition spec "tick multiplexes across all buses with a deadline and dispatches via per-bus routes" — quiet bus returns within deadline+1ms; injected frame updates target motor; one decode per frame even with arm+gripper sharing a bus (assert via `MockMotorCodec::decode_call_count`); foreign motor_id dropped silently
- 6.15 Impl: lifecycle gates — `enable`/`tick` before `connect` returns `NotConnected`; `disable` before `enable` is no-op `Ok(())`; mutation methods (`add_group` style) after connect return `TopologyLocked`
- 6.16 Test: covers robot-composition spec "Lifecycle methods are explicit and ordered" — tick before connect returns `NotConnected`; disable without enable is no-op sending zero frames
- 6.17 Impl: `Drop for Robot` closes every bus's underlying socket; non-panicking; no implicit disable
- 6.18 Test: covers robot-composition spec "Drop closes all sockets" — fds previously returned by `bus.raw_fd()` are closed after drop (verify via /proc/self/fd snapshot before/after)
- 6.19 Impl: `dm_control::Error` enum via `thiserror` with all variants listed in robot-composition spec including `FdNotImplementedInV1`, `CanIdCollision`
- 6.20 Test: covers robot-composition spec "Error enum is layered via thiserror" — TransportError wraps cleanly into Error::Transport without losing inner variant; each variant's Display string contains the identifying information for diagnostics
- 6.21 Impl: assert that no `MotorGroup`/`Arm`/`Gripper`/`Generic` method body calls `drain_inbound_nonblocking` — enforce via a `grep` test in CI that runs against the source files
- 6.22 Test: covers robot-composition spec "Sends are never coupled to inbound reads" — `arm.mit_control` produces exactly N send calls and zero drain calls on `MockCanBus::recorded_calls()`; grep test passes

## 7. dm-control: TOML config loader

- 7.1 Impl: serde-derived `RobotConfig`, `BusConfig`, `GroupConfig`, `MotorConfig` types; deny unknown fields via `#[serde(deny_unknown_fields)]`; reject `vendor` key on group with a custom error path
- 7.2 Test: covers robot-composition spec "TOML config schema is well-defined" — minimal config parses; unknown bus key rejected with offending key in message; bus name reference validated; vendor-on-group rejected with the documented "vendor belongs on [bus.]" hint
- 7.3 Impl: TOML loader's `fd` handling — reject `fd = true` at parse time before opening any socket, return `FdNotImplementedInV1 { bus_name }`; accept `fd = false` and absent fd key
- 7.4 Test: covers robot-composition spec "TOML fd = true is rejected in v1" — all three scenarios (true rejected, false accepted, absent accepted), fd count unchanged
- 7.5 Impl: vendor registry `HashMap<&'static str, fn() -> Box<dyn MotorCodec>>` populated with `"damiao" → || Box::new(DamiaoCodec::new())`; `Robot::from_config(path)` reads TOML, walks config, instantiates Bus structs and groups via the builder, returns Robot
- 7.6 Test: covers robot-composition spec "vendor field on bus selects the MotorCodec" — `bus.main vendor = "damiao"` resolves to DamiaoCodec; `vendor = "ghost"` returns `UnknownVendor("ghost")`
- 7.7 Impl: write `configs/openarm_single.toml` matching the example in design.md API surface section
- 7.8 Test: `Robot::from_config("configs/openarm_single.toml")` returns Ok and the loaded robot has expected bus_names and group_names

## 8. dm-control-py: PyO3 bindings + Python smoke tests

- 8.1 Impl: `dm-control-py/Cargo.toml` with `cdylib`, `pyo3`, `numpy`; `maturin.pyproject.toml`; module entry point exporting `dm_control` Python module
- 8.2 Test: covers python-bindings spec "maturin build succeeds on Linux x86_64 stable" — `maturin build --release` produces a manylinux wheel; `pip install` into venv with only numpy preinstalled succeeds
- 8.3 Impl: `PyCanFrame` wrapper exposing `id`, `flags`, `len`, `payload()` (returns bytes), `is_fd()`, `is_extended()`; classmethods `classical(id, payload)` and `fd(id, payload)`
- 8.4 Test: `dm_control.CanFrame.classical(0x101, b"\xff" * 8).payload() == b"\xff" * 8`; `CanFrame.fd(0x101, b"\x00" * 9)` raises ValueError; `is_fd()` reflects flag
- 8.5 Impl: `PyMotorSpec` wrapper; `PySocketCanBus` with `__init__(interface, fd=False)` that raises `TransportError` when `fd=True`
- 8.6 Test: covers python-bindings spec "SocketCanBus(fd=True) raises an actionable error in v1" — `SocketCanBus("vcan0", fd=False)` works on a present vcan0 (gate on env), `SocketCanBus("vcan0", fd=True)` raises `dm_control.TransportError` with substring "CAN-FD is reserved"
- 8.7 Impl: `dm_control.damiao` submodule with `PyDamiaoCodec` and `MotorType` enum (Python IntEnum exposing all 13 SKUs)
- 8.8 Test: covers python-bindings spec "Vendor codecs are namespaced under dm_control." — `from dm_control.damiao import DamiaoCodec, MotorType` succeeds; `MotorType.DM4340` accessible; `DamiaoCodec()` instantiable
- 8.9 Impl: `PyRobotBuilder` with `add_bus(name, transport, codec)`, `add_arm(name, *, bus, motors)`, `add_gripper(name, *, bus, motor)`, `add_generic(name, *, bus, motors)`, `build`
- 8.10 Test: covers python-bindings spec "RobotBuilder.add_bus accepts (name, transport, codec)" and "RobotBuilder.add_arm does NOT accept a codec parameter" — chained call returns builder; codec.bind_to_bus invoked once (assert via mock codec exposing a Python-visible call counter); `builder.add_arm(codec=...)` raises TypeError
- 8.11 Impl: `PyRobot` with `from_config` classmethod, `connect`, `enable`, `disable`, `tick`, `__enter_`_, `__exit__`, `__getitem__`, `__contains__`, `group_names`, `bus_names`
- 8.12 Test: covers python-bindings specs "Robot supports the context manager protocol" and "Group access via **getitem** returns the kind-dispatched wrapper" — with-block enters connected+enabled and exits with disable called once; KeyError on missing group; `__getattr__` not implemented (AttributeError on `robot.arm`); exception in body propagates with disable failure logged
- 8.13 Impl: `PyArm` with numpy-backed `positions()`, `velocities()`, `torques()` returning read-only `np.ndarray` views (use `numpy::PyArray1::borrow_from_slice` where memory layout permits, else contiguous copy); shape-checked `mit_control`, `pos_vel_control`, `vel_control`, `pos_force_control`
- 8.14 Test: covers python-bindings specs "Batch state accessors return numpy arrays" and "Batch send accepts shape-checked numpy arrays" — shape and dtype correct; insertion order matches; wrong dtype raises TypeError naming float64; wrong shape raises ValueError naming expected vs got shapes
- 8.15 Impl: `PyGripper`, `PyMotor` wrappers with documented surface
- 8.16 Test: PyGripper exposes only the v1 methods; `gripper.open(0.5)` raises AttributeError; PyMotor properties return cached state
- 8.17 Impl: wrap all blocking syscalls (tick, enable, disable, send methods, enable_all, disable_all) in `Python::allow_threads`
- 8.18 Test: covers python-bindings spec "Blocking methods release the GIL" — spawn two Python threads, one running 100ms tick, the other incrementing a counter; assert counter advanced by >1 during the tick window
- 8.19 Impl: PyO3 `create_exception!` for `DmError` (root) and subclasses `TransportError`, `CodecError`, `ConfigError`, `LifecycleError`; map every Rust `Error` variant in a `From<Error> for PyErr` impl
- 8.20 Test: covers python-bindings spec "Errors raise a DmError hierarchy" — `Err(TransportError::Io(_))` raises `dm_control.TransportError` catchable as `dm_control.DmError`; `tick` before `connect` raises `LifecycleError` naming NotConnected
- 8.21 Impl: TOML loader exposed via `Robot.from_config(path)`; `fd = true` raises `ConfigError`
- 8.22 Test: covers python-bindings spec "TOML fd = true raises ConfigError at load time" — loading TOML with `fd = true` raises `dm_control.ConfigError` whose message names the offending bus and "fd = false"
- 8.23 Impl: ship `.pyi` type stubs (either embedded in the wheel or as a sibling `dm_control-stubs` package) covering all public symbols
- 8.24 Test: covers python-bindings spec "Type stubs accompany the wheel" — `mypy --strict tests/python/test_types.py` reports zero "Cannot find module" or "has no attribute" errors
- 8.25 Impl: write `tests/python/test_smoke.py` constructing a Robot with `MockCanBus`, running enable, 10-tick control loop with MIT commands, asserting expected frame count, running disable
- 8.26 Test: covers python-bindings spec "Python smoke test runs against the MockCanBus" — CI pytest job runs smoke test green; assertion includes nonzero sent and received frame counts via the mock loopback

## 9. End-to-end examples and integration tests

- 9.1 Impl: `examples/single_arm.rs` matching the design.md API surface example — constructs Robot via builder, runs 10-second 1kHz control loop holding zero position via MIT
- 9.2 Test: `cargo build --example single_arm` succeeds; CI runs the example against `vcan0` for 1 second (gated on vcan0 availability) with simulated motor responses injected via a paired writer
- 9.3 Impl: `examples/single_arm.py` matching the design.md Python API surface example
- 9.4 Test: CI pytest runs `examples/single_arm.py --mock` mode (env-controlled) using `MockCanBus`; asserts loop completes without exception
- 9.5 Impl: integration test in `tests/integration_single_arm.rs` — full end-to-end: build, connect, enable, 100-tick loop, disable, drop; uses `MockCanBus` with a paired writer that produces synthetic Damiao state response frames
- 9.6 Test: integration test runs in `cargo test --test integration_single_arm` and asserts (a) every commanded motor receives a state update within the tick deadline, (b) no decode errors, (c) drop closes fds
- 9.7 Impl: integration test in `tests/python/test_integration_single_arm.py` mirroring the Rust integration test
- 9.8 Test: CI pytest runs the Python integration test green

## 10. Documentation and release prep

- 10.1 Impl: top-level `README.md` with quickstart (build wheel, install, run smoke test); link to `openspec/` for the source of truth; explicit v1 scope note (Linux SocketCAN only, classical CAN, Damiao only)
- 10.2 Test: README's quickstart commands run end-to-end on a fresh container (manual one-time verification noted in PR description)
- 10.3 Impl: `docs/can-fd.md` explaining the v1 FD limitations, the type-readiness for v2, and the explicit error paths
- 10.4 Impl: `docs/multi-vendor.md` explaining the codec abstraction, what it would take to add Robostride, and the bus-vendor binding rule
- 10.5 Impl: `cargo doc --workspace --no-deps` produces clean output (no broken intra-doc links); every public type has at least one line of doc
- 10.6 Test: `cargo doc --workspace --no-deps -- -D rustdoc::broken_intra_doc_links` runs green in CI
- 10.7 Impl: `LICENSE` file (Apache-2.0 + MIT dual license is the Rust ecosystem default; confirm with user before committing)
- 10.8 Impl: `CHANGELOG.md` with `## v0.1.0 (unreleased)` entry summarizing the walking skeleton
- 10.9 Test: `cargo publish --dry-run -p motor-codec` and analogous for dm-codec, dm-control succeed (no metadata or license issues)

