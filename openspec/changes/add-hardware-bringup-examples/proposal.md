## Why

The current project is implemented, but its runnable examples still behave like a walking-skeleton validation: one Rust full-loop example and one Python full-loop example jump directly from construction to a multi-motor MIT loop. That is useful for proving the stack exists, but it is not how a user safely brings up real Damiao/OpenArm hardware.

Mock examples do not validate the intended user path. They prove API shape and CI wiring, but they cannot catch wrong interface setup, wrong CAN IDs, missing motor replies, unsafe zeroing, stale state reads, or confusing control-mode usage. Those belong in tests. The separated examples should instead be controlled hardware scenes where each step adds exactly one kind of risk.

`enactic/openarm_can` points at the missing shape: users first configure SocketCAN, discover or verify one motor, enable/disable, read state, try one control mode with conservative values, then scale to arm/gripper flows and zero-position calibration. `socketcan-rs` also separates pure tests from `vcan0`/host-dependent tests. This change makes `dm_control_rs` examples follow that hardware-first bring-up ladder.

## What Changes

- Replace the example strategy from “mock/full-loop first” to “real hardware bring-up first.”
- Keep mock usage in tests and, at most, a small developer smoke example that is clearly not hardware validation.
- Add a staged Python-first hardware example suite under `examples/`, with matching Rust examples only where Rust users need the same low-level surface.
- Cover progressively riskier scenes:
  - CAN interface availability and setup notes.
  - Single-motor enable/disable with explicit send/recv IDs.
  - Single-motor state monitoring without motion.
  - Single-motor conservative MIT hold.
  - Single-motor control-mode samples for MIT, PosVel, Vel, and PosForce.
  - Single-arm read-only monitoring.
  - Single-arm conservative MIT hold.
  - Gripper MIT/PosVel scene, with PosForce deferred unless implementation explicitly expands the gripper API in scope.
  - Set-zero calibration as an explicitly dangerous/manual flow.
- Update README/docs so users know which examples are safe, which require `vcan0`, and which require real powered hardware.
- Preserve current tests and mock coverage for CI, but stop presenting mock execution as meaningful hardware validation.

## Non-Goals

- No new control algorithms, trajectories, IK, teleoperation, ROS integration, or kinematics.
- No CAN-FD enablement; v1 remains classical CAN only and continues rejecting `fd=true`.
- No new transport implementation.
- No fake simulation layer or expanded mock tutorial suite.
- No automatic motor discovery unless the current public API can support it safely without adding parameter-query features.

## Success Criteria

- A new user can follow the examples in order and test one real motor before any multi-motor or motion example.
- Hardware examples default to conservative durations, gains, and command magnitudes.
- Every hardware-affecting example prints its assumptions and exits through a disable path.
- Mock examples are not emphasized as hardware validation.
- Existing Rust and Python tests still pass, and example syntax is checked without requiring real hardware where possible.
