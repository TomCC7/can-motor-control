> **Test gating rule.** Every implementation task that creates or changes executable examples must have a paired verification task. Mock-based verification may prove syntax/API shape, but it does not count as hardware validation.

## 1. Example structure and shared conventions

- [x] 1.1 Impl: define the hardware-example naming/order under `examples/` using numeric prefixes from `00_` through `08_`, and remove or de-emphasize any mock-first example naming that suggests mock execution validates hardware.
- [x] 1.2 Test: verify the top-level example list is ordered from no-frame checks to calibration and that no mock example appears before hardware bring-up examples.
- [x] 1.3 Impl: add small shared Python conventions inline or in a local helper only if needed: argument parsing for `--interface`, `--send-id`, `--recv-id`, `--motor-type`, `--seconds`, `--deadline-us`, and clear printed assumptions before commands are sent.
- [x] 1.4 Test: syntax-check all Python examples with `python -m py_compile` or equivalent without requiring `can_motor_control` to open hardware.

## 2. Tier 0: interface / environment check

- [x] 2.1 Impl: add `examples/00_can_interface_check.py` that checks the selected SocketCAN interface path, prints v1 classical-CAN assumptions, and sends no CAN frames.
- [x] 2.2 Test: run the interface check against a missing interface and verify it exits with a clear diagnostic rather than trying to construct a robot or send frames.

## 3. Tier 1: one motor, no motion

- [x] 3.1 Impl: add `examples/01_single_motor_enable_disable.py` that builds a one-motor robot from explicit IDs, connects, enables, waits/ticks briefly, disables, and prints every ID and timing assumption before acting.
- [x] 3.2 Test: syntax-check the example and manually inspect that all hardware-affecting commands are inside a disable-on-exit path.
- [x] 3.3 Impl: add `examples/02_single_motor_read_state.py` that enables one motor, performs bounded `tick()` calls, prints position/velocity/torque/temperature/fault fields, and disables.
- [x] 3.4 Test: syntax-check and confirm the example contains no motion command calls (`mit_control`, `pos_vel_control`, `vel_control`, `pos_force_control`).

## 4. Tier 2: one motor, bounded control

- [x] 4.1 Impl: add `examples/03_single_motor_mit_hold.py` with conservative default MIT gains, zero setpoint, bounded `--seconds`, and printed safety assumptions.
- [x] 4.2 Test: verify defaults are bounded and conservative, and that duration cannot be unbounded.
- [x] 4.3 Impl: add `examples/04_single_motor_control_modes.py` with an explicit `--mode` selector for `mit`, `pos-vel`, `vel`, and `pos-force`; default command values must be zero or near-zero and non-zero values must come from user flags.
- [x] 4.4 Test: syntax-check and verify each mode maps to the currently exposed Python `Arm` API shape: MIT `(n,5)`, PosVel `(n,2)`, Vel `(n,)`, PosForce `(n,3)`.

## 5. Tier 3: arm and gripper scenes

- [x] 5.1 Impl: add `examples/05_single_arm_state_monitor.py` that loads either explicit arm IDs or `configs/openarm_single.toml`, enables, monitors state for a bounded time, and disables without sending motion commands.
- [x] 5.2 Test: confirm the monitor example has no control command calls and documents that it should be run after one-motor bring-up.
- [x] 5.3 Impl: add `examples/06_single_arm_mit_hold.py` that runs a bounded low-gain hold loop using the configured arm and prints tick count/frequency summary.
- [x] 5.4 Test: syntax-check and verify default gains/duration are conservative.
- [x] 5.5 Impl: add `examples/07_gripper_control.py` using only currently exposed gripper operations (`enable`, `disable`, `mit_control`, `pos_vel_control`) unless implementation intentionally expands the gripper API in a separate scoped task.
- [x] 5.6 Test: verify the gripper example does not call unimplemented `open`, `close`, or PosForce gripper methods.

## 6. Tier 4: calibration / set-zero

- [x] 6.1 Impl: add `examples/08_set_zero_calibration.py` as an isolated hardware-only flow requiring an explicit confirmation flag before any set-zero command is sent.
- [x] 6.2 Test: run the example without the confirmation flag and verify it exits before constructing/enabling hardware.
- [x] 6.3 Impl: document the disable → set-zero → disable sequence and limitations of any group-level set-zero operation exposed by the current API.
- [x] 6.4 Test: inspect that the example prints target IDs/groups before sending set-zero and cannot run silently.

## 7. README and docs

- [x] 7.1 Impl: update `README.md` with a hardware bring-up section listing the examples in order and marking risk level: no frames, enable/disable only, bounded control, calibration.
- [x] 7.2 Test: verify README instructions do not claim mock examples validate hardware.
- [x] 7.3 Impl: add or update docs with SocketCAN setup reminders, `candump` recommendation, v1 classical-CAN-only status, and guidance for skipping real-hardware examples in CI.
- [x] 7.4 Test: verify docs distinguish mock tests, `vcan0`, and real `can0` hardware.

## 8. Verification and manual QA

- [x] 8.1 Test: run `cargo test --workspace` and `cargo test --workspace --doc`.
- [x] 8.2 Test: run Python syntax checks for all examples.
- [x] 8.3 Test: run the no-frame `00_can_interface_check.py` in the current environment.
- [x] 8.4 Test: if `vcan0` is available, run the examples that can safely exercise SocketCAN without real motors; otherwise document that host CAN is unavailable.
- [ ] 8.5 Manual QA: on real hardware, execute the ladder in order through at least `03_single_motor_mit_hold.py` before declaring the hardware example suite validated.

## 9. Rust scoped bring-up examples

- [x] 9.1 Impl: add `crates/can-motor-control/examples/can_interface_check.rs` mirroring Python tier 0, checking `/sys/class/net/<iface>`, printing classical-CAN assumptions, and sending no CAN frames.
- [x] 9.2 Test: run the Rust interface check help/missing-interface path and verify it exits with a clear diagnostic before opening a socket.
- [x] 9.3 Impl: add `crates/can-motor-control/examples/single_motor_enable_disable.rs` mirroring Python tier 1, building one motor from explicit IDs, connecting, enabling, bounded ticking, disabling, and printing assumptions before acting.
- [x] 9.4 Test: build/check the Rust examples and run the single-motor example help path without touching hardware.
- [x] 9.5 Impl: add `crates/can-motor-control/examples/single_motor_mit_hold.rs` mirroring Python tier 2, with conservative MIT defaults, bounded runtime, and disable-on-exit behavior.
- [x] 9.6 Test: build/check the Rust MIT hold example and run help/bad-input paths without touching hardware.
- [x] 9.7 Impl: add `crates/can-motor-control/examples/gripper_control.rs` mirroring Python tier 3, using only current Rust gripper `enable`, `disable`, `mit_control`, and `pos_vel_control` operations.
- [x] 9.8 Test: build/check the Rust gripper example and run help/bad-input paths without touching hardware.
- [x] 9.9 Impl: add Rust gripper PosForce support and `crates/can-motor-control/examples/gripper_posforce.rs`, following only openarm_can's fixed `π/2 → 0 → π/2 → 0` sequence with speed/current defaults.
- [x] 9.10 Test: build/check the Rust gripper PosForce example and run help/bad-input paths without touching hardware.
