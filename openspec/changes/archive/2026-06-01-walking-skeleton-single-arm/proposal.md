## Why

Damiao motor based robots are a fast-growing category of low-cost, high-torque-density actuators used in research arms, dual-arm rigs, and humanoids, but the available driver libraries are two unmaintained single-file Python scripts (`cmjang/DM_Control_Python`, `enactic/openarm_can`) that either bake in one transport, ship no abstractions above the motor, or both. A Rust core with a Python binding ships as a single wheel, installs in seconds on a fresh machine, runs in hard-realtime control loops without GIL contention, and gives both Rust services and Python ML/teleop code the same robot model. This first change proves the full architecture stack end-to-end on a single arm so the layering decisions are validated before broader multi-bus and humanoid work begins.

## What Changes

- Establish a Cargo workspace with four crates: `motor-codec` (pure, no IO — the vendor-agnostic codec trait + common types), `damiao-codec` (Damiao implementation of the trait), `can-motor-control` (transport + motor + group + robot, generic over codec via trait objects), `can-motor-control-py` (PyO3 binding built by maturin).
- Introduce a layered abstraction: `CanBus` trait → `MotorCodec` trait → `Motor` → `MotorGroup` → `Robot` — with `Arm` and `Gripper` as named group kinds (gripper kept intentionally thin in v1).
- Make the motor codec a swappable trait so Damiao stays one implementation among future vendors (Robostride, MyActuator, CubeMars all use the same protocol family). Groups own a `Box<dyn MotorCodec>`; vendor-specific extensions (e.g. Damiao's `0x7FF` param sub-protocol) are reachable via a downcast extension trait.
- Implement `SocketCanBus` as the v1 transport, multiplexed across multiple buses via `mio` / `poll(2)`. No async runtime, no threads spawned by the core API.
- Define an explicit `tick()` read path — sends never implicitly receive; one owner of inbound frames; per-motor state cache updated only by `tick`.
- Provide both `Robot::builder()` (programmatic) and `Robot::from_config(path)` (TOML) construction paths, sharing the same validation. The TOML loader is sugar over the builder.
- Reserve `GroupKind::Gripper(Gripper)` as a variant and a config block so a richer gripper API (force control, open/close, self-calibration) can be added later without touching `Robot` or the bus dispatcher.
- Ship PyO3 bindings on day 1 so Rust public types stay binding-friendly from the start: trait-object transports (no generics in public API), `Box<dyn CanBus>`, string-keyed buses and groups, numpy interop for batch reads/writes, GIL released during blocking calls.
- Deliver one working example each in Rust (`examples/single_arm.rs`) and Python (`examples/single_arm.py`) that enables, runs a control loop at ~500–1000 Hz against a single Damiao arm on SocketCAN, and disables cleanly.
- Set up CI for: cargo fmt/clippy/test on `dm-can-codec` and `can-motor-control` (mock transport, no real hardware), maturin wheel build for `can-motor-control-py`, Python smoke test against the mock bus.

Out of scope for this change (deliberate non-goals): serial / USB-CAN-dongle transport, SLCAN, CAN-FD code paths (the trait will accommodate but no impl yet), kinematics / IK, URDF loading, async runtime, multi-arm robots, Hand / Leg group kinds, gripper specialization beyond a thin newtype, managed background IO thread.

## Capabilities

### New Capabilities

- `can-transport`: The `CanBus` trait and `SocketCanBus` impl. Frame send, non-blocking inbound drain, raw-fd exposure for `poll(2)` multiplexing, `MockCanBus` for tests. Defines what every present and future transport must support. Vendor-agnostic.
- `motor-codec`: The `MotorCodec` trait — the seam that lets the rest of the stack stay vendor-agnostic. Defines the shared `Command` and `Event` enums (MIT, PosVel, Vel, PosForce, Enable, Disable, SetZero — the protocol family's common surface), the vendor-tagged `MotorTypeId`, the `Limits` value type, and the contract every vendor implementation must satisfy. Pure, no IO, `no_std`-friendly. Future vendor codecs (Robostride, MyActuator, CubeMars) plug in here without changes elsewhere.
- `damiao-codec`: Damiao's implementation of the `motor-codec` trait. Damiao motor-type table (DM4310, DM4340, DM6006, ...), MIT bit-packing per Damiao's limit-param ranges, CAN ID assignment scheme (`slave_id`, `0x100 + slave_id`, `0x200 + slave_id`, `0x300 + slave_id`), enable/disable framing (`0xFC`/`0xFD`/`0xFE`), and the Damiao-exclusive `0x7FF` parameter sub-protocol exposed via a `DamiaoCodecExt` downcast trait.
- `motor-group`: `Motor` state cache + `MotorGroup` collection + the named group kinds (`Arm`, `Gripper`). Each group owns a `Box<dyn MotorCodec>` so different groups in one robot can run different vendors. Batch send, name- and index-based motor access, the per-tick state update path, group-level enable/disable/zero. Gripper is the reserved thin newtype.
- `robot-composition`: `Robot`, `RobotBuilder`, the TOML config schema and loader, the multi-bus `tick()` loop using `mio::poll`, and the three-step lifecycle (`from_config` → `connect` → `enable`). TOML schema includes a per-group `vendor` field selecting the codec. Names buses and groups; locks topology after `connect()`.
- `python-bindings`: PyO3 wrapper crate `can-motor-control-py`, maturin build, the Python-facing class hierarchy (`Robot`, `RobotBuilder`, `SocketCanBus`, `MotorSpec`, kind-dispatched `PyArm` / `PyGripper`), numpy roundtrip for batch state and commands, GIL released during blocking calls, context-manager protocol on `Robot`. Bindings ship Damiao support in this change; future vendor codecs surface as additional submodules (`can_motor_control.vendors.robostride`, etc.).

### Modified Capabilities

None — this is the project's first change.

## Impact

- **Code**: Net-new repository. New `Cargo.toml` workspace at root; new crates `crates/motor-codec`, `crates/damiao-codec`, `crates/can-motor-control`, `crates/can-motor-control-py`; new `python/can_motor_control/` package skeleton; new `configs/` directory with one example TOML; new `examples/` for both Rust and Python.
- **Public APIs**: All net-new. Two parallel surfaces: Rust API rooted at `can_motor_control::{Robot, RobotBuilder, Arm, Gripper, CanBus, SocketCanBus, ...}` and Python API rooted at `can_motor_control.{Robot, RobotBuilder, SocketCanBus, MotorSpec, ...}` mirroring the Rust shape with PEP 8 names.
- **Dependencies (Rust)**: `socketcan`, `mio`, `thiserror`, `serde`, `toml`, `pyo3`, `numpy` (pyo3 ecosystem), plus `nix` or `libc` for low-level fd handling if `mio` doesn't cover everything. `maturin` as the build backend for the Python wheel.
- **Dependencies (Python)**: `numpy` at runtime; the wheel is self-contained otherwise.
- **Platforms**: Linux only in v1 (SocketCAN is a Linux kernel feature). The codec crate is platform-agnostic. macOS / Windows users can build and run unit tests against the mock bus only.
- **CI**: New GitHub Actions workflow — fmt/clippy/test on Linux, maturin-build smoke for the Python wheel, Python `pytest` against the mock bus. No real-hardware testing in CI.
- **Hardware required for end-to-end verification**: One Damiao motor (any supported type), one Linux host with a SocketCAN-capable interface (Peak, Innomaker, or kernel virtual `vcan0` for plumbing-only validation).
- **Future changes unblocked**: Serial / SLCAN transports plug into the same `CanBus` trait; gripper specialization extends the reserved variant; multi-arm and humanoid configs are additional TOML files, no Rust changes; async adapter crate is purely additive.

