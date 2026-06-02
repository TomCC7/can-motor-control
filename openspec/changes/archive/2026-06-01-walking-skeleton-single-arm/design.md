## Context

The two reference implementations — `cmjang/DM_Control_Python` (single-file pyserial-based script) and `enactic/openarm_can` (C++ library with nanobind Python wrapper) — solve overlapping problems with opposite shapes. cmjang is flat, fuses sends with receives, and hard-codes a USB-CAN dongle's serial framing. openarm_can is properly layered (`CANSocket` → `CANDevice` → `DMCANDevice` → `DMDeviceCollection` → `ArmComponent`/`GripperComponent` → `OpenArm`) but inheritance-heavy, Linux-only by construction, and rebuilds the Python binding on every release because it lives in the C++ tree.

The wire protocol is identical across both: Damiao motors expose four control modes (MIT, PosVel, Vel, PosForce) on offset CAN IDs (`slave_id`, `0x100 + slave_id`, `0x200 + slave_id`, `0x300 + slave_id`), reply on a configurable `master_id`, and accept a parameter sub-protocol over CAN ID `0x7FF` with command byte 0x33 (read) / 0x55 (write) / 0xAA (save) / 0xCC (refresh) / 0xFC,0xFD,0xFE (enable, disable, set zero). Frame payload is always 8 bytes; MIT mode bit-packs (q, dq, kp, kd, tau) into uints whose range depends on motor type via a fixed limit-param table.

This change picks the openarm layering as the skeleton, rebuilds it in Rust with composition instead of inheritance, swaps the C++ binding tree for a separate PyO3/maturin crate, and tightens the read-path semantics. It targets one Damiao arm over Linux SocketCAN — the smallest end-to-end slice that exercises every layer.

## Goals / Non-Goals

**Goals**

- Validate the five-layer abstraction (transport / codec / motor / group / robot) on real hardware before committing to multi-bus, multi-arm, or humanoid scope.
- Establish a Rust public-API shape that PyO3 can wrap without generics gymnastics, so the binding stays a thin wrapper for the life of the library.
- Make the `CanBus` trait the single seam where future transports (serial USB-CAN, SLCAN, CAN-FD, mocked, simulated) plug in without changes elsewhere.
- Decouple inbound frame handling (one owner: `tick()`) from outbound commands (cheap, non-blocking sends), so jitter and stale state are bounded by tick period and never by the most recently issued command.
- Ship one Rust example and one Python example that compile, link, and run a 500–1000 Hz control loop on hardware. Demo-readiness is the bar.
- Set up CI that exercises every crate without requiring a CAN interface, using the mock transport.

**Non-Goals**

- Serial / USB-CAN-dongle transport (cmjang-style). The `CanBus` trait will accommodate it; no impl in v1.
- CAN-FD code paths. The trait will leave room; no impl in v1.
- Kinematics, IK, trajectory generation, URDF loading. Out of library scope permanently.
- Multi-arm robots, humanoid configs, Hand / Leg group kinds. The architecture is designed to accommodate them — none ship in v1.
- Gripper specialization beyond a thin newtype. The variant is reserved; the API surface stays empty pending a dedicated design pass.
- Async runtime, managed background IO thread, `async`-flavored API. Sync-only in v1.
- Windows / macOS hardware support. Codec crate compiles cross-platform; transport is Linux-only.

## Decisions

### Decision 1 — Six-layer abstraction with two trait-object boundaries

```
   L5  Robot              ── owns buses + named groups, drives tick() across all buses
   L4  MotorGroup         ── Arm / Gripper / (reserved) — batch ops, name access
   L3  Motor              ── identity + state cache (q, dq, tau, t_mos, t_rotor, ...)
   L2b vendor codec impls ── DamiaoCodec (this change), RobostrideCodec (future, additive)
   L2a MotorCodec trait   ── encode_{mit, pos_vel, ...} / decode / limits / vendor extensions
   L1  CanBus trait       ── send / drain_inbound_nonblocking / raw_fd
```

Two trait-object seams: **L1** for transport (multiple transports planned, must mock for tests), **L2a** for codec (multiple vendors planned, must keep upper layers vendor-agnostic). L2a is a tiny trait-only crate (`motor-codec`) with `#![no_std]` and no dependencies beyond `CanFrame`. L2b vendor codecs live in separate crates (`dm-codec` ships in this change; `robostride-codec` etc. are purely additive future crates). L3–L5 in `dm-control` use trait objects (`Box<dyn CanBus>`, `Box<dyn MotorCodec>`) and have zero vendor knowledge.

*Alternative considered:* fewer layers (collapse L1+L2 into one transport module, collapse L4+L5 into one robot module). Rejected because L2 is the only thing reusable in a firmware target, and L4 ≠ L5 — L4 is a single homogeneous control unit, L5 is composition of N L4s across M L1s. Merging them blocks future humanoid composition.

*Alternative considered:* inheritance-based group kinds (openarm's `ArmComponent : DMDeviceCollection`). Rejected — Rust prefers composition; `Arm { inner: MotorGroup }` newtype is cleaner, easier for PyO3 to dispatch on, and avoids virtual-call overhead in the hot path.

*Alternative considered:* generic `MotorGroup<C: MotorCodec>` instead of `Box<dyn MotorCodec>`. Rejected for the same reason as `CanBus`: generics multiply PyO3 wrapper classes per concrete codec, and runtime codec selection from TOML config requires erasure anyway.

### Decision 2 — Explicit `tick()` is the sole inbound reader; sends never receive

```
  user loop:                     bus internal:
    robot.tick(deadline) ──────▶ for each bus:
                                   poll(fd, remaining)
                                   read all available frames
                                   dispatch each via codec → group → motor cache
    s = group.states()  ──────▶ read from cache (no IO)
    group.mit_control() ──────▶ codec → bus.send() (no IO read)
```

cmjang fuses send+recv inside every command function. That's the most obvious anti-pattern in the reference implementations: it races on response arrival, drops frames when the buffer fills between calls, and miss async fault messages. The cure is a single inbound owner.

*Alternative considered:* implicit recv on every send (cmjang). Rejected for the reasons above.

*Alternative considered:* per-motor message queues and explicit `motor.read()`. Rejected because per-motor state is the natural abstraction users want (`motor.position()` cheap, always returns last known value); the cache is a small per-motor struct guarded by group ownership.

*Alternative considered:* core spawns a background thread that ticks at fixed rate, user code reads cached state without explicit ticking. Rejected as the default because it imposes a threading dependency on users who don't need it; offered later as an opt-in `spawn_io_thread()` helper.

### Decision 3 — Sync API with `mio`/`poll(2)` for multi-bus multiplexing

The earlier conversation settled sync over async. Multi-bus could naively iterate buses with per-bus timeouts, but that introduces head-of-line blocking: a quiet bus eats the whole tick deadline. The Linux `poll(2)` syscall waits on multiple file descriptors with a single timeout and wakes on any of them. SocketCAN sockets are pollable; `mio::Poll` is the idiomatic Rust wrapper.

```
   tick(deadline):
     mio::poll(events, deadline - now)   ── blocks until any fd readable
     for event in events:
       buses[event.token].drain_nonblocking()
```

*Alternative considered:* raw `nix::poll`/`libc::poll`. Rejected — `mio` is the standard, handles edge cases (interruption by signal, `EINTR` retry), and integrates with future async via `tokio` if needed. The dependency cost is small.

*Alternative considered:* one thread per bus with blocking reads. Rejected — adds Send/Sync constraints on Motor/Group state, requires channels for outbound commands, complicates user code.

*Alternative considered:* `async` API (tokio-socketcan). Rejected for v1 — the user picked sync explicitly; defer to an additive `dm-control-async` adapter crate if a real consumer appears.

### Decision 4 — Generic `Robot` with named groups; group kinds as a discriminated `GroupKind`

```
pub struct Robot {
    buses:  HashMap<String, Box<dyn CanBus>>,
    groups: HashMap<String, GroupKind>,
}
pub enum GroupKind {
    Arm(Arm),
    Gripper(Gripper),
    Generic(MotorGroup),   // escape hatch for unforeseen layouts
}
```

Hard-coding `SingleArm`, `DualArm`, `Humanoid` as distinct Rust types looks tidy until someone wires up a three-arm rig or a quadruped — then every new topology costs a new struct + PyO3 wrapper. The named-group bag scales arbitrarily; specialization lives in the group kinds (where `Arm` and `Gripper` carry their own methods), not in the Robot type.

*Alternative considered:* typed robot flavors (Shape A from exploration). Rejected for rigidity.

*Alternative considered:* fully untyped groups, `MotorGroup` only (Shape B without `GroupKind`). Rejected because gripper-specific helpers (in v2: open/close, force control) need a home, and they don't belong on a generic group.

*Alternative considered:* typed views over the generic robot (`OpenArmView(robot).left_arm.mit_control(...)`). Deferred — additive over the generic robot, can ship in a later change once usage patterns emerge.

### Decision 5 — `Box<dyn CanBus>` and string keys everywhere in public API; no generics

PyO3 cannot easily expose Rust generics to Python — each instantiation would need its own wrapper class. The library targets PyO3 from day 1, so the Rust public API must be PyO3-friendly: transports are `Box<dyn CanBus>` (and codecs are `Box<dyn MotorCodec>`), buses and groups address each other by `String` name, motors address each other by name within a group. The `Bus` struct bundles transport and codec into a single Robot-owned value (see Decision 15).

```
   pub trait CanBus: Send {
       fn send(&mut self, frame: &CanFrame) -> Result<()>;
       fn drain_inbound_nonblocking(&mut self) -> Result<Vec<CanFrame>>;
       fn raw_fd(&self) -> Option<RawFd>;
   }
```

`raw_fd` returns `Option` because non-socket transports (future serial / SLCAN) may not have a pollable fd; those transports require a fallback path (likely a per-bus thread) when added.

*Trade-off:* one vtable indirection per `send()` (~1ns) and one `HashMap<String, _>` lookup per group access (~50ns). Both vanish next to the SocketCAN syscall (~5–10µs). Negligible.

*Alternative considered:* generic over `B: CanBus` throughout. Rejected — would force one PyO3 wrapper per concrete bus type, multiplied by every group kind.

### Decision 6 — Builder is canonical; TOML loader is sugar

```
   Robot::from_config(path)
       ├── parse TOML into a RobotConfig struct (serde)
       └── apply to RobotBuilder::from_config(cfg).build()

   Robot::builder()
       .add_bus("left", Box::new(SocketCanBus::open("can0")?))
       .add_arm("left_arm", "left", [MotorSpec::new("j0", DM4340, 0x01, 0x11), ...])
       .build()
```

This guarantees the validation paths are identical: missing bus names, duplicate motor IDs, gripper with >1 motor, unknown motor types — all caught by one validator regardless of construction path. The TOML loader cannot drift behavior away from the programmatic API because it's a one-line conversion.

*Alternative considered:* TOML-only construction. Rejected — testing benefits from in-process builder use, and Python users sometimes want programmatic construction (dynamic robot topologies).

*Alternative considered:* builder-only, no TOML loader in v1. Rejected because the Python ergonomics win from declarative config is too big to skip; humanoid configs will be unmanageable without it.

### Decision 7 — Three-step lifecycle: `from_config` / `connect` / `enable`

```
   Robot::from_config(path)   ── parse + validate, no IO, no socket
        │
        ▼
   robot.connect()            ── open sockets, register dispatch, lock topology
        │
        ▼
   robot.enable()             ── send enable to all motors, await first state
        │
        ▼  control loop
        │
   robot.disable()            ── send disable, sockets remain open
        │
        ▼
   drop(robot)                ── Drop closes sockets
```

Three steps because three distinct failure modes deserve distinct error sites: config errors (no hardware needed), connection errors (interface down, permission denied), enable errors (motor unresponsive, in fault state). Folding into one constructor obscures which step failed.

*Alternative considered:* RAII (construct = connect = enable). Rejected — blocks CI from validating configs without hardware, blocks diagnostic tools that want to read state without commanding.

### Decision 8 — Topology locked after `connect()`

Adding a group after `connect()` would require mid-flight CAN ID re-dispatch registration. Doable, but a footgun: existing groups may have outstanding state, the user may not realize the new group's IDs collide with existing ones, and the failure mode (silently miss frames) is hard to debug. Builder operations after `connect()` return `Error::TopologyLocked`. Hot-reload of robot topology is a future change if ever needed.

### Decision 9 — Names primary, indices secondary

Group access by name (`robot["left_arm"]`), motor access by name within group (`arm["shoulder_pitch"]`), parallel `Vec<Motor>` for declaration-order index access (`arm.positions() -> [f64]`). Hot loops use the array path; configuration and debugging use names.

```
   pub struct MotorGroup {
       motors: Vec<Motor>,                   // declaration order
       by_name: HashMap<String, usize>,      // name → index lookup
   }
```

*Alternative considered:* numeric indices only (openarm). Rejected — humanoid debugging is intractable without names.

*Alternative considered:* names only. Rejected — string keying every per-motor field access in a 1kHz numpy loop is wasted Python cycles.

### Decision 10 — Four-crate workspace, codec trait separate from impl

```
   crates/
     motor-codec/      no_std. trait + common Command/Event/MotorTypeId/Limits.
                       No vendor knowledge. Depends on a CanFrame type only.
     dm-codec/         no_std. impl MotorCodec for DamiaoCodec.
                       Damiao motor-type table, bit-packing, CAN ID scheme,
                       DamiaoCodecExt for the 0x7FF param sub-protocol.
     dm-control/       std. depends on motor-codec ONLY (not dm-codec).
                       Holds Box<dyn MotorCodec>. Features: socketcan (default), mock.
     dm-control-py/    std, cdylib. PyO3 bindings.
                       depends on dm-control + dm-codec (binds Damiao support today).
```

The codec is small (a few hundred lines of bit-packing, enums, tables) and pure. Splitting trait from impl means:
- `dm-control` has zero vendor knowledge — adding Robostride later doesn't recompile `dm-control`
- Each vendor crate can be `no_std`, reusable in embedded MCU firmware that drives that vendor's motors directly
- The `motor-codec` crate is tiny and stable; vendors implement against it without coordinating with each other

The cost is two extra `Cargo.toml`s; the discipline forces L2a (the contract) to stay genuinely vendor-agnostic and forces L2b (the impl) to stay self-contained.

*Alternative considered:* single crate with feature flags per vendor. Rejected — `no_std` discipline tends to erode under feature flags; feature unification across the workspace means enabling `damiao` in one consumer forces it on every consumer; separation makes the contract explicit.

*Alternative considered:* three crates (fold the codec trait into `dm-control`). Rejected — `dm-control` is `std` (transport + IO + serde + socketcan); pulling the trait in there blocks embedded reuse of the trait and forces every vendor impl crate to depend on `std`.

*Alternative considered:* split transport into its own crate. Deferred — `SocketCanBus` is the only impl in v1; splitting transport into its own crate is YAGNI until a second transport ships.

### Decision 11 — Vendor-agnostic `MotorCodec` trait with vendor extension via downcast

```rust
// motor-codec crate — common surface every vendor supports
pub enum MotorTypeId {
    Damiao(u16),       // sub-enum's discriminant lives in the vendor crate
    Robostride(u16),   // future; the trait is unchanged when added
    // additional variants added by future PRs to the motor-codec crate
}

pub struct MotorRef<'a> {
    pub motor_type: MotorTypeId,
    pub send_id: u32,
    pub recv_id: u32,
    // ...
}

pub enum Command {
    Mit     { kp: f64, kd: f64, q: f64, dq: f64, tau: f64 },
    PosVel  { q: f64,  dq: f64 },
    Vel     { dq: f64 },
    PosForce{ q: f64,  dq: f64, i_pu: f64 },
}

pub enum Event {
    State { motor_id: u32, q: f64, dq: f64, tau: f64, t_mos: i16, t_rotor: i16 },
    ParamReply { motor_id: u32, rid: u16, value: ParamValue },
    Fault { motor_id: u32, code: u16 },
}

pub trait MotorCodec: Send + Sync {
    fn vendor_name(&self) -> &'static str;
    fn supports(&self, motor_type: MotorTypeId) -> bool;
    fn limits(&self, motor_type: MotorTypeId) -> Result<Limits, CodecError>;

    /// Called by the group builder once per codec instance, before any encode.
    /// Codec MAY store caps for later use (e.g. v2: decide whether to emit FD frames).
    fn bind_to_bus(&mut self, caps: BusCapabilities);

    fn encode_enable(&self, motor: MotorRef<'_>) -> Result<CanFrame, CodecError>;
    fn encode_disable(&self, motor: MotorRef<'_>) -> Result<CanFrame, CodecError>;
    fn encode_set_zero(&self, motor: MotorRef<'_>) -> Result<CanFrame, CodecError>;
    fn encode_command(&self, motor: MotorRef<'_>, cmd: &Command) -> Result<CanFrame, CodecError>;

    fn decode(&self, frame: &CanFrame) -> Result<Option<Event>, CodecError>;
}
```

Vendor-exclusive commands (Damiao's `0x7FF` parameter sub-protocol; Robostride's mode-config registers) don't belong on the common trait. They live behind a per-vendor extension trait:

```rust
// dm-codec crate
pub trait DamiaoCodecExt {
    fn encode_read_param(&self, motor: MotorRef<'_>, rid: DamiaoRid) -> CanFrame;
    fn encode_write_param<T: Into<ParamValue>>(&self, motor: MotorRef<'_>, rid: DamiaoRid, val: T) -> CanFrame;
    fn encode_save_to_flash(&self, motor: MotorRef<'_>) -> CanFrame;
    fn encode_refresh(&self, motor: MotorRef<'_>) -> CanFrame;
}
impl DamiaoCodecExt for DamiaoCodec { ... }
```

User code that needs Damiao-specific operations downcasts the trait object:

```rust
let group = robot.group_mut("left_arm");
let dm_ext = group.codec_ext::<DamiaoCodec>()
    .ok_or(Error::WrongVendor)?;
let frame = dm_ext.encode_write_param(motor_ref, DamiaoRid::MstId, 0x11);
group.bus_mut().send(&frame)?;
```

This pattern (common trait + vendor extension downcast) keeps the 90% of cross-vendor code clean and gives the 10% of vendor-specific code an explicit, type-checked path that fails loudly when the vendor doesn't match.

*Alternative considered:* put every vendor's full surface on the common trait with `Option`/`NotSupported` returns. Rejected — bloats the common surface, makes the trait churn every time a vendor adds a feature, and gives users a false impression that the operations are portable.

*Alternative considered:* associated types on `MotorCodec` (`type MotorType; type Limits;`). Rejected — breaks object safety, blocks `Box<dyn MotorCodec>`, blocks runtime codec selection from TOML.

### Decision 12 — Per-frame allocation discipline

Hot path is `send_mit_command_all`, called at 1kHz with N frames per call. Allocating a `Vec<u8>` per frame would create GC pressure (in Python) and jitter (in Rust). The codec encodes into a fixed `[u8; 8]` or `[u8; 64]` (CAN-FD); the transport `send()` accepts `&CanFrame` by reference; inbound `drain_inbound_nonblocking` returns a borrowed slice or fills a user-provided buffer. `Vec` allocations are allowed at setup time, never inside `tick()` or a control call.

### Decision 13 — Error model: per-crate `Error` enums via `thiserror`, single `DmError` Python facade

```
   motor_codec::CodecError → contract-level encoding/decoding errors (unknown motor type,
                             vendor mismatch, command not supported by this codec)
   dm_codec::Error         → Damiao-specific decoding errors (bad frame format, etc.)
   dm_control::Error       → wraps CodecError + transport IO + topology + lifecycle
   PyO3 layer              → flattens to a single dm_control.DmError hierarchy in Python
```

Rust gets layered, exhaustive errors; Python gets a flat exception hierarchy that's easy to catch. PyO3 conversions live in the binding crate so the Rust types stay clean.

### Decision 14 — Unified `CanFrame` covers classical and CAN-FD; FD impl deferred to v2

CAN-FD differs from classical CAN in two ways the library has to handle: larger payloads (up to 64 bytes vs 8) and a separate kernel struct (`canfd_frame` vs `can_frame`). The choice was between a single unified frame type (72 B inline regardless of format) and split classical/FD types (16 B / 72 B).

Choice: **single unified `CanFrame` type** with a `FrameFlags` field carrying `FD_FORMAT`, `BIT_RATE_SWITCH`, `ERROR_STATE`, `EXTENDED_ID`, `REMOTE_REQUEST`. All upper-layer code (Robot, MotorGroup, Arm, Gripper, user code) handles frames without ever distinguishing classical from FD. Bus impls and codec impls inspect/set `FD_FORMAT` internally.

```rust
pub struct CanFrame {
    pub id:    u32,
    pub flags: FrameFlags,
    pub len:   u8,
    data:      [u8; 64],     // inline; classical uses first 8 bytes
}
// 72 bytes total. ~5× larger than a classical-only 16 B representation;
// negligible vs syscall cost (~5µs each); buys full agnosticism.

pub struct BusCapabilities {
    pub supports_fd:      bool,
    pub max_payload_len:  u8,    // 8 if !supports_fd, 64 otherwise
}
```

Rationale for unified:
- Damiao payloads are always 8 bytes regardless of bus format — the type's 64-byte capacity is rarely fully used, so the memory cost is mostly latent.
- One frame type means one `CanBus::send`, one `MotorCodec::encode_command`, one `Arm::mit_control` per control mode. No `CanFdBus`, no `MotorCodecFd`, no `GroupTransport` enum to disambiguate.
- The classical/FD dispatch is hidden in two places: codec impl (chooses whether to set `FD_FORMAT` when emitting) and bus impl (chooses kernel write path based on `FD_FORMAT`). Both impls already know their target; both already branch on protocol/transport specifics.
- Adding actual FD operation in v2 requires zero changes to upper layers, zero changes to public types or traits. Only the codec impl gains "emit FD when bound to FD-capable bus" and the transport impl gains the FD write/read paths.

v1 scope: `CanFrame` type, `FrameFlags`, `BusCapabilities`, and the unified trait surface all ship. SocketCanBus rejects `fd=true` at construction. DamiaoCodec always emits classical (`FD_FORMAT` unset) regardless of bound bus capability. TOML `fd = true` errors at parse time.

```
   v1 rejection points (explicit errors, never silent fallback):
     SocketCanBus::open(_, fd=true)              → TransportError::FdNotImplementedInV1
     SocketCanBus::send(frame.is_fd())           → TransportError::FdNotImplementedInV1
     bus.send(fd_frame) on supports_fd=false bus → TransportError::FdFrameOnNonFdBus
     TOML fd = true                               → Error::FdNotImplementedInV1 { bus_name }
```

*Alternative considered:* split `CanFrame` (16 B) + `CanFdFrame` (72 B) types with sibling `CanBus` + `CanFdBus` traits. Rejected — would require either group-level branching (`GroupTransport` enum) or doubling the codec/bus surface, both of which leak the classical/FD distinction into upper layers. The ~56-byte-per-frame waste was rejected as the cost concern; the team's preference was full upper-layer agnosticism.

*Alternative considered:* borrowed `CanFrameView<'a>` + write-into-buffer pattern (~24 B per frame view, zero allocation). Rejected — unusual API ergonomics, lifetime propagation, awkward PyO3 binding. Worth revisiting only if memory cost becomes a real constraint.

*Alternative considered:* defer all FD-related types from v1, add as additive sibling types in v2. Rejected — would force a breaking change to `FrameFlags` and `BusCapabilities` (or require declaring them stable later). The cost of declaring the FD-ready types in v1 is small.

### Decision 15 — Codec lives with the bus, not with the group

Two reasonable places to own a `Box<dyn MotorCodec>`: per group (each `Arm` / `Gripper` / `Generic` carries its own codec instance) or per bus (one codec per CAN interface, shared across every group attached to it). Choice: **codec lives with the bus**.

```rust
pub struct Bus {
    transport: Box<dyn CanBus>,
    codec:     Box<dyn MotorCodec>,
    routes:    HashMap<u32, RouteKey>,   // recv_id → (group_name, motor_index); built at connect()
}
pub struct RouteKey { pub group_name: String, pub motor_index: usize }

pub struct Robot {
    buses:  HashMap<String, Bus>,
    groups: HashMap<String, GroupKind>,
}
```

`MotorGroup` carries `bus_name: String` and no codec field. All encode operations route through `bus.codec.encode_*`; all sends through `bus.transport.send`; all decodes through `bus.codec.decode` (exactly once per inbound frame regardless of how many groups attach to the bus).

Inbound dispatch:
```
   Robot::tick():
     for each readable bus:
       frames = bus.transport.drain_inbound_nonblocking()
       for frame in frames:
         match bus.codec.decode(&frame)?:                    ← one decode per frame
           Some(event) =>
             if let Some(route) = bus.routes.get(&event.motor_id):
                 robot.groups[route.group_name].apply_event(route.motor_index, event)
             else: silently drop (foreign motor_id)
           None: silently drop (codec didn't recognize)
```

Rationale:
- **Decode happens once per frame.** With codec-per-group, every group on a shared bus would decode every frame; only one would route it to a motor, the rest waste work. arm + gripper on one bus (the openarm pattern) doubles decode cost under codec-per-group.
- **CAN ID collisions detected at `connect()` time.** When the bus's routing table is populated by walking all groups on the bus, two motors with the same recv_id surface as `Error::CanIdCollision` instead of being a silent runtime hazard.
- **One codec instance per bus regardless of group count.** Memory footprint is smaller and more predictable; `bind_to_bus` is called exactly once per codec.
- **Vendor is a bus-level concept.** All motors on one CAN wire speak the same protocol — the codec encodes that fact at the type level. Mixed-vendor on a single bus is unusual; the design trades that flexibility for the routing efficiency and collision detection.

Builder API consequence:

```rust
// add_bus takes both transport AND codec; bind_to_bus is called here
RobotBuilder::new()
    .add_bus("main",
        Box::new(SocketCanBus::open("can0", false)?),
        Box::new(DamiaoCodec::new()))
    .add_arm("arm", "main", motors)                  // no codec param
    .add_gripper("g", "main", MotorSpec::new(...))   // no codec param
    .build()?
```

TOML schema consequence: `vendor` field moves from `[[group]]` to `[bus.<name>]`:

```toml
[bus.main]
kind = "socketcan"
interface = "can0"
fd = false
vendor = "damiao"        # ← was on group; now on bus

[[group]]
name = "arm"
kind = "arm"
bus = "main"
# vendor field MUST NOT appear here (loader rejects with helpful message)
motors = [...]
```

`connect()` consequence: builds per-bus recv_id routing table from all groups on the bus. CAN ID collision across groups on the same bus aborts connect with `Error::CanIdCollision`. Collisions across different buses are not errors (CAN IDs are bus-scoped).

*Alternative considered:* codec per group. Rejected for the decode-duplication cost on shared buses; rejected as not actually buying anything (mixed-vendor on one bus is rare and can be added later with a bus-level codec registry if ever needed).

*Alternative considered:* hybrid — codec stays with group, bus owns a recv_id → (group, motor) routing table for efficient dispatch (Option 3 in the discussion that led here). Rejected by explicit team preference for the simpler ownership model where one CAN interface ↔ one vendor codec.

*Alternative considered:* codec per bus, but bus.routes lives at the Robot level (not in the Bus struct). Rejected because the routing table is logically per-bus (collisions are per-bus); putting it elsewhere creates a coordination bug surface.

*Trade-off:* a future hardware setup that mixes Damiao and Robostride motors on the same CAN wire would need to be either reconfigured (split into two buses) or supported via a future "multi-vendor bus" change. The walking-skeleton scope explicitly accepts this constraint.

## API Surface

This section pins the user-visible Rust and Python APIs at the level of method signatures and example usage. Lower-level details (private fields, internal helpers) belong in the specs. Examples here are normative — they should compile (Rust) or run (Python) once implementation lands.

### Rust API — single-arm end-to-end

```rust
use dm_control::{Robot, RobotBuilder, MotorSpec, SocketCanBus, MitCmd, Command};
use dm_codec::{DamiaoCodec, DamiaoMotorType::DM4340};
use std::time::{Duration, Instant};

// === construction via builder ===
// codec is paired with the transport at add_bus time
let mut robot = RobotBuilder::new()
    .add_bus("main",
        Box::new(SocketCanBus::open("can0", /* fd */ false)?),
        Box::new(DamiaoCodec::new()))
    .add_arm("arm", "main", vec![
        MotorSpec::new("j0", DM4340, /* send */ 0x01, /* recv */ 0x11),
        MotorSpec::new("j1", DM4340, 0x02, 0x12),
        MotorSpec::new("j2", DM4340, 0x03, 0x13),
        MotorSpec::new("j3", DM4340, 0x04, 0x14),
    ])
    .build()?;

// === or: construction via TOML config ===
// let mut robot = Robot::from_config("configs/openarm_single.toml")?;

// === lifecycle ===
robot.connect()?;     // opens sockets, registers dispatch, locks topology
robot.enable()?;      // sends enable to all motors, awaits first state

// === control loop ===
let deadline_per_tick = Duration::from_micros(500);
let loop_period      = Duration::from_millis(1);

let start = Instant::now();
while start.elapsed() < Duration::from_secs(10) {
    let tick_start = Instant::now();

    robot.tick(deadline_per_tick)?;             // drain inbound, update caches

    let arm = robot.group_mut("arm").as_arm_mut().unwrap();
    let q   = arm.positions();                  // &[f64] in declaration order
    let dq  = arm.velocities();
    let tau = arm.torques();

    // user controller produces N MIT commands
    let cmds: Vec<MitCmd> = (0..arm.len()).map(|i| MitCmd {
        kp: 50.0, kd: 1.0,
        q:  0.0, dq: 0.0, tau: 0.0,
    }).collect();

    arm.mit_control(&cmds)?;                    // batch send, no recv

    // single-motor access by name when you need it:
    let j0_pos = arm.motor("j0").unwrap().position();
    let _ = j0_pos;

    // pace
    if let Some(rem) = loop_period.checked_sub(tick_start.elapsed()) {
        std::thread::sleep(rem);
    }
}

robot.disable()?;
// drop(robot) closes sockets via Drop impl
```

Key Rust types and their public surface:

```rust
// dm-control crate

pub struct Robot { /* ... */ }
impl Robot {
    pub fn from_config<P: AsRef<Path>>(path: P) -> Result<Self, Error>;
    pub fn builder() -> RobotBuilder;
    pub fn connect(&mut self) -> Result<(), Error>;
    pub fn enable(&mut self)  -> Result<(), Error>;
    pub fn disable(&mut self) -> Result<(), Error>;
    pub fn tick(&mut self, per_bus_deadline: Duration) -> Result<(), Error>;

    pub fn group(&self, name: &str)         -> Option<&GroupKind>;
    pub fn group_mut(&mut self, name: &str) -> Option<&mut GroupKind>;
    pub fn group_names(&self) -> impl Iterator<Item = &str>;
    pub fn bus_names(&self)   -> impl Iterator<Item = &str>;
}

pub enum GroupKind { Arm(Arm), Gripper(Gripper), Generic(MotorGroup) }
impl GroupKind {
    pub fn as_arm(&self)         -> Option<&Arm>;
    pub fn as_arm_mut(&mut self) -> Option<&mut Arm>;
    pub fn as_gripper(&self)     -> Option<&Gripper>;
    pub fn as_gripper_mut(&mut self) -> Option<&mut Gripper>;
    pub fn as_generic(&self)     -> Option<&MotorGroup>;
}

pub struct Arm { /* inner: MotorGroup */ }
impl Arm {
    pub fn len(&self) -> usize;
    pub fn motor(&self, name: &str)         -> Option<&Motor>;
    pub fn motor_mut(&mut self, name: &str) -> Option<&mut Motor>;
    pub fn motor_at(&self, idx: usize)      -> Option<&Motor>;

    // batch views (live borrow into the per-motor cache)
    pub fn positions(&self)  -> &[f64];
    pub fn velocities(&self) -> &[f64];
    pub fn torques(&self)    -> &[f64];

    // batch sends — slice length must equal self.len()
    pub fn mit_control(&mut self,      cmds: &[MitCmd])      -> Result<(), Error>;
    pub fn pos_vel_control(&mut self,  cmds: &[PosVelCmd])   -> Result<(), Error>;
    pub fn vel_control(&mut self,      cmds: &[VelCmd])      -> Result<(), Error>;
    pub fn pos_force_control(&mut self,cmds: &[PosForceCmd]) -> Result<(), Error>;

    // group lifecycle (also reachable via Robot::enable/disable)
    pub fn enable_all(&mut self)  -> Result<(), Error>;
    pub fn disable_all(&mut self) -> Result<(), Error>;
    pub fn set_zero_all(&mut self) -> Result<(), Error>;

    // codec extension downcast for vendor-specific operations
    pub fn codec_ext<C: 'static>(&self) -> Option<&C>;
}

pub struct Gripper { /* inner: MotorGroup, v1 thin newtype */ }
impl Gripper {
    pub fn motor(&self)         -> &Motor;
    pub fn motor_mut(&mut self) -> &mut Motor;
    pub fn enable(&mut self)  -> Result<(), Error>;
    pub fn disable(&mut self) -> Result<(), Error>;
    pub fn mit_control(&mut self,     cmd: MitCmd)    -> Result<(), Error>;
    pub fn pos_vel_control(&mut self, cmd: PosVelCmd) -> Result<(), Error>;
    // Reserved: open(), close(), set_force(), calibrate() — designed in a follow-up change.
}

pub struct Motor { /* state cache + identity */ }
impl Motor {
    pub fn name(&self)       -> &str;
    pub fn motor_type(&self) -> MotorTypeId;
    pub fn send_id(&self)    -> u32;
    pub fn recv_id(&self)    -> u32;
    pub fn position(&self)   -> f64;
    pub fn velocity(&self)   -> f64;
    pub fn torque(&self)     -> f64;
    pub fn temperature_mos(&self)   -> i16;
    pub fn temperature_rotor(&self) -> i16;
    pub fn is_enabled(&self) -> bool;
    pub fn fault(&self)      -> Option<FaultCode>;
}

pub struct RobotBuilder { /* ... */ }
impl RobotBuilder {
    pub fn new() -> Self;
    pub fn from_config<P: AsRef<Path>>(path: P) -> Result<Self, Error>;
    // add_bus takes BOTH transport AND codec; bind_to_bus is called here
    pub fn add_bus(self,
        name: impl Into<String>,
        transport: Box<dyn CanBus>,
        codec: Box<dyn MotorCodec>,
    ) -> Self;
    pub fn add_arm(self,
        name: impl Into<String>,
        bus_name: impl Into<String>,
        motors: Vec<MotorSpec>,
    ) -> Self;
    pub fn add_gripper(self,
        name: impl Into<String>,
        bus_name: impl Into<String>,
        motor: MotorSpec,
    ) -> Self;
    pub fn add_generic(self,
        name: impl Into<String>,
        bus_name: impl Into<String>,
        motors: Vec<MotorSpec>,
    ) -> Self;
    pub fn build(self) -> Result<Robot, Error>;
}

// Command shapes (re-exported from motor-codec)
pub struct MitCmd      { pub kp: f64, pub kd: f64, pub q: f64, pub dq: f64, pub tau: f64 }
pub struct PosVelCmd   { pub q: f64,  pub dq: f64 }
pub struct VelCmd      { pub dq: f64 }
pub struct PosForceCmd { pub q: f64,  pub dq: f64, pub i_pu: f64 }
```

### Python API — same flow, Pythonic surface

```python
import numpy as np
import time
import dm_control
from dm_control.damiao import DamiaoCodec, MotorType

# === construction via builder ===
# codec is paired with the transport at add_bus
robot = (dm_control.RobotBuilder()
    .add_bus("main",
        dm_control.SocketCanBus("can0", fd=False),
        DamiaoCodec())
    .add_arm("arm", bus="main", motors=[
        dm_control.MotorSpec("j0", MotorType.DM4340, send_id=0x01, recv_id=0x11),
        dm_control.MotorSpec("j1", MotorType.DM4340, send_id=0x02, recv_id=0x12),
        dm_control.MotorSpec("j2", MotorType.DM4340, send_id=0x03, recv_id=0x13),
        dm_control.MotorSpec("j3", MotorType.DM4340, send_id=0x04, recv_id=0x14),
    ])
    .build())

# === or: construction via TOML config ===
# robot = dm_control.Robot.from_config("configs/openarm_single.toml")

# === lifecycle as a context manager: connect+enable on __enter__, disable on __exit__ ===
with robot:
    arm = robot["arm"]                  # → PyArm; raises KeyError if missing
    N = len(arm)
    cmds = np.zeros((N, 5), dtype=np.float64)  # columns: kp, kd, q, dq, tau
    cmds[:, 0] = 50.0                          # kp
    cmds[:, 1] = 1.0                           # kd

    deadline_us = 500
    period      = 1e-3

    t_end = time.monotonic() + 10.0
    while time.monotonic() < t_end:
        t0 = time.monotonic()

        robot.tick(deadline_us)                # drain inbound, update caches

        # batch read — zero-copy numpy view into the per-motor cache (read-only)
        q   = arm.positions()                  # np.ndarray (N,) float64
        dq  = arm.velocities()                 # np.ndarray (N,) float64
        tau = arm.torques()                    # np.ndarray (N,) float64

        # user controller writes into cmds[:, 2:5] = desired q, dq, tau
        # ...

        arm.mit_control(cmds)                  # batch send, no recv

        # single-motor access by name when you need it
        j0_pos = arm["j0"].position

        rem = period - (time.monotonic() - t0)
        if rem > 0:
            time.sleep(rem)
# __exit__ sends disable, sockets close on garbage collection
```

Key Python types and surface:

```python
# dm_control package

class SocketCanBus:
    def __init__(self, interface: str, fd: bool = False) -> None: ...

class MotorSpec:
    def __init__(self, name: str, type: MotorType, send_id: int, recv_id: int) -> None: ...

class RobotBuilder:
    def add_bus(self, name: str, transport: CanBus, codec: MotorCodec) -> "RobotBuilder": ...
    def add_arm(self, name: str, *, bus: str, motors: list[MotorSpec]) -> "RobotBuilder": ...
    def add_gripper(self, name: str, *, bus: str, motor: MotorSpec) -> "RobotBuilder": ...
    def add_generic(self, name: str, *, bus: str, motors: list[MotorSpec]) -> "RobotBuilder": ...
    def build(self) -> "Robot": ...

class Robot:
    @classmethod
    def from_config(cls, path: str | Path) -> "Robot": ...
    @classmethod
    def builder(cls) -> RobotBuilder: ...

    def connect(self) -> None: ...
    def enable(self)  -> None: ...
    def disable(self) -> None: ...
    def tick(self, per_bus_deadline_us: int) -> None: ...

    # __enter__ → connect() then enable()
    # __exit__  → disable() (sockets close on GC)
    def __enter__(self) -> "Robot": ...
    def __exit__(self, *exc) -> None: ...

    # dict-like access; returns PyArm | PyGripper | PyMotorGroup
    def __getitem__(self, name: str) -> "Group": ...
    def __contains__(self, name: str) -> bool: ...
    def group_names(self) -> list[str]: ...
    def bus_names(self)   -> list[str]: ...

class PyArm:           # what robot["some_arm"] returns
    def __len__(self) -> int: ...
    def __getitem__(self, name: str) -> "PyMotor": ...    # by name
    def __iter__(self): ...                                # iterates motors

    # numpy views into the per-motor cache (zero-copy where possible, read-only)
    def positions(self)  -> np.ndarray: ...     # shape (N,), float64
    def velocities(self) -> np.ndarray: ...
    def torques(self)    -> np.ndarray: ...

    # batch send — accepts np.ndarray with required shape/dtype
    def mit_control(self,       cmds: np.ndarray) -> None: ...   # (N, 5) float64
    def pos_vel_control(self,   cmds: np.ndarray) -> None: ...   # (N, 2) float64
    def vel_control(self,       cmds: np.ndarray) -> None: ...   # (N,)   float64
    def pos_force_control(self, cmds: np.ndarray) -> None: ...   # (N, 3) float64

    def enable_all(self)  -> None: ...
    def disable_all(self) -> None: ...
    def set_zero_all(self) -> None: ...

class PyGripper:        # what robot["some_gripper"] returns (v1: thin)
    @property
    def motor(self) -> "PyMotor": ...
    def enable(self)  -> None: ...
    def disable(self) -> None: ...
    def mit_control(self,     kp: float, kd: float, q: float, dq: float, tau: float) -> None: ...
    def pos_vel_control(self, q: float, dq: float) -> None: ...
    # open / close / set_force / calibrate are NOT in v1.

class PyMotor:          # what arm["j0"] returns
    @property
    def name(self) -> str: ...
    @property
    def position(self) -> float: ...
    @property
    def velocity(self) -> float: ...
    @property
    def torque(self) -> float: ...
    @property
    def temperature_mos(self)   -> int: ...
    @property
    def temperature_rotor(self) -> int: ...
    @property
    def is_enabled(self) -> bool: ...
    @property
    def fault(self) -> int | None: ...
```

Notes on the Python surface:
- `__getitem__` on Robot and Arm is the primary access pattern; attribute access (`robot.arm`) is intentionally not provided so typos fail loudly with `KeyError` instead of silently creating attributes.
- All blocking methods (`tick`, `enable`, `disable`, `mit_control`, etc.) release the GIL during the underlying syscall via `Python::allow_threads`.
- Numpy reads return read-only views where possible; if the underlying memory layout doesn't permit (e.g. struct-of-arrays storage), they return a contiguous copy — documented per method.
- The context manager protocol gives the cmjang one-liner experience back: `with robot: ...` collapses connect+enable+disable into a block.
- Vendor codecs live under namespaced submodules (`dm_control.damiao`, future: `dm_control.robostride`) so a user installing the wheel sees only the vendors that shipped, and adding a vendor doesn't risk colliding with existing imports.

### TOML config schema

```toml
# configs/openarm_single.toml

[bus.main]
kind = "socketcan"        # discriminator; future: "serial", "slcan"
interface = "can0"
fd = false
vendor = "damiao"         # selects codec for this bus; required

[[group]]
name = "arm"
kind = "arm"              # discriminator: arm | gripper | generic
bus = "main"
default_control_mode = "mit"
# vendor field MUST NOT appear here (loader rejects with a helpful message)
motors = [
  { name = "j0", type = "DM4340", send_id = 0x01, recv_id = 0x11 },
  { name = "j1", type = "DM4340", send_id = 0x02, recv_id = 0x12 },
  { name = "j2", type = "DM4340", send_id = 0x03, recv_id = 0x13 },
  { name = "j3", type = "DM4340", send_id = 0x04, recv_id = 0x14 },
]

# Example gripper on the same bus (illustrates codec sharing)
# [[group]]
# name = "grip"
# kind = "gripper"
# bus = "main"             # shares "main" with arm; same codec serves both
# motor = { name = "g", type = "DM4310", send_id = 0x08, recv_id = 0x18 }
# # Future: [group.grip.gripper] block with calibration / limits / force config.
```

The `vendor` field on each bus selects which `MotorCodec` impl to load; that codec serves every group attached to the bus. `type` strings are resolved by the bus's codec (Damiao's codec knows `DM4340`, future Robostride codec knows e.g. `RS01`). Validation fires at `from_config` time, before any IO. CAN ID collisions between groups on the same bus are caught at `connect()` time via the per-bus routing table.

## Risks / Trade-offs

- **[Risk] `mio` adds a dependency for what's effectively three syscalls.** → Mitigation: keep `mio` use confined to a single `BusPoller` struct so it can be swapped for direct `nix::poll` if the dep becomes burdensome.

- **[Risk] Trait-object `CanBus` prevents a future user from passing in their own transport without it implementing the trait.** → Mitigation: the trait is intentionally minimal (3 methods); document the contract carefully in the spec; provide a `MockCanBus` that's also the reference implementation for the contract tests.

- **[Risk] Reserving `Gripper` as a thin wrapper now and adding rich semantics later may force breaking changes in the Gripper API.** → Mitigation: in v1 expose Gripper only as `inner: MotorGroup` access plus the most generic `mit_control` / `pos_vel_control` — no opinionated semantics that could be wrong. Users who need today's hand-coded gripper logic can drop down to `MotorGroup` ops.

- **[Risk] PyO3 binding shipped day 1 means Rust API refactors hurt twice (Rust + bindings) for the life of the library.** → Mitigation: keep the binding crate intentionally thin (no logic, only wrapping); design the Rust API around PyO3 constraints from the start (trait objects, string keys) so refactors are unlikely to be PyO3-driven.

- **[Risk] Mock-only CI cannot catch frame-timing bugs.** → Mitigation: include a `vcan0`-based integration test that runs in CI if a virtual CAN interface is available, exercising the real codec ↔ `SocketCanBus` ↔ another `SocketCanBus` roundtrip; require the hardware loop test in a HW-CI runner (deferred to a later change).

- **[Risk] Three-step lifecycle is more ceremony than the cmjang one-liner experience.** → Mitigation: the Python context manager pattern (`with Robot.from_config(...) as r:`) collapses `connect()` and `enable()` into the `__enter__`, restoring the simple feel for the common case.

- **[Risk] `Box<dyn CanBus>` and `HashMap<String, _>` lookups in the dispatch path add overhead that may matter at very high frame rates.** → Mitigation: benchmark in v1 with a 4kHz loop on `MockCanBus`; if dispatch is meaningful, intern bus names into `BusId(u8)` after `connect()` and switch the dispatch path to indexed lookup.

- **[Trade-off] Linux-only in v1 limits the addressable user base.** → Accepted; the codec crate is portable and other transports unlock other platforms (SLCAN over serial works on macOS/Windows).

- **[Trade-off] Single-crate codec without a transport crate may need to be split later.** → Accepted; the migration is mechanical (file move + `Cargo.toml` edit) and only happens if a second transport demands it.

- **[Risk] The `MotorCodec` trait shape is set in v1 based on Damiao alone; the first non-Damiao vendor may need methods we didn't anticipate.** → Mitigation: the trait is deliberately small (encode_enable / encode_disable / encode_set_zero / encode_command / decode / limits / supports). Vendor-specific operations go behind per-vendor extension traits, not on the common trait. If a second vendor needs a genuinely common method that wasn't anticipated, adding it to `motor-codec` is a minor version bump and all vendor crates implement it.

- **[Risk] `MotorTypeId` enum lives in the `motor-codec` crate and gains a variant per vendor, creating coordination friction.** → Mitigation: each variant carries an opaque `u16` discriminant that the vendor codec interprets internally, so adding `Robostride(u16)` doesn't constrain Damiao's discriminant encoding. The enum stays narrow (one variant per vendor); per-motor-type strings (`"DM4340"`, `"RS01"`) are resolved through the codec's `supports()` and parsing methods, not through the enum.

- **[Trade-off] Topology locking after `connect()` precludes hot-add of motors during runtime.** → Accepted; a future change can add a `reconfigure()` step that drains the bus, re-registers dispatch, and resumes.

## Open Questions

- **Damiao firmware variant.** cmjang has both `enable()` (new firmware) and `enable_old(ControlMode)` (legacy firmware with control-mode-dependent enable IDs). Do we target new firmware only in v1, or detect-and-support both? Recommendation: new firmware only, document the firmware version requirement.
- **Master-ID-zero handling.** cmjang has a special branch where motors with `master_id=0` get their state routed via the slave ID. Is this a configuration users actually pick, or a bug-tolerance hack? Investigate before deciding whether to support it.
- **Parameter-write acknowledgement semantics.** cmjang loops with 20× retries waiting for an echo to confirm the write took. openarm provides a callback-mode switch. Pick one approach for `WriteParam` — synchronous wait-with-timeout, or fire-and-forget with a separate read for verification?
- **Whether the `Codec` exposes a raw-frame escape hatch.** Power users may want to send custom frames (calibration sequences, factory-test commands) without going through `Command`. Recommendation: yes, but only on the bus object (`bus.send(&CanFrame)`), not on the codec.
- **Numpy array layout for batch state reads.** Two options: structured array `(N,)` with fields `('q','dq','tau','tmos','trotor')`, or plain `(N, 5)` float64. Structured is self-documenting; plain is what most controllers actually want. Recommendation: provide both via `arm.states()` (structured) and `arm.q()`, `arm.dq()`, `arm.tau()` (plain views).
- **Error granularity for hot-path failures.** A motor going into fault mid-loop — should `tick()` return an error, or accumulate into a per-motor fault flag the user polls? Recommendation: latter; `tick()` returns `Ok(())` unless the *bus itself* is broken, and per-motor fault state lives on `Motor`.
- **Vendor crate naming convention.** `dm-codec`, `robostride-codec`, `myactuator-codec`, etc. — flat names at the workspace level, or grouped under a `vendors/` directory (`vendors/damiao/`, `vendors/robostride/`)? Recommendation: flat at workspace level, grouped under `dm_control.vendors.*` in the Python namespace. Crates can still publish independently to crates.io.
- **Trait extension downcast ergonomics.** Calling `group.codec_ext::<DamiaoCodec>()?.encode_write_param(...)` is verbose. Consider whether to expose a `damiao_ext()` convenience on `Arm`/`Gripper` when the codec is known at config-load time, or keep the downcast pattern as the only path. Recommendation: downcast only in v1 — convenience layer is additive once usage shows it's worth the API surface cost.
