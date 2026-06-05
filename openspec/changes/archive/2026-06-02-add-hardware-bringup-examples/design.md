## Context

The implemented stack has the pieces needed for hardware bring-up: `SocketCanBus`, `RobotBuilder`, `Robot::from_config`, `Robot::connect`, `Robot::enable`, `Robot::tick`, `Robot::disable`, `Arm` batch state/control methods, and a minimal `Gripper`. The current examples do not expose those pieces as a safe sequence. They combine construction, enable, ticking, and MIT commands in one full-loop flow.

The reference projects show two useful patterns:

- `openarm_can` examples and CLI split operations into hardware scenes: configure CAN, enable/disable, query/read state, run one control mode, monitor frequency, set zero.
- `socketcan-rs` keeps host-dependent `vcan0` tests gated separately from pure tests, because interface availability and root privileges are environmental facts, not unit-test facts.

This change adopts those patterns without adding new protocol features.

## Goals / Non-Goals

**Goals**

- Make examples useful for controlled real hardware testing, not just API demonstration.
- Add risk in small increments: interface check → one motor no-motion → one motor controlled motion → multi-motor monitoring → multi-motor hold → gripper/calibration.
- Prefer Python examples because the package is explicitly first-class for ML/teleop users and can be run one file at a time.
- Provide Rust equivalents only for core Rust-user surfaces: interface check, single-motor enable/read, and single-arm hold.
- Keep examples bounded by explicit `--seconds`, `--deadline-us`, IDs, interface, and conservative command defaults.
- Print assumptions before sending frames: powered hardware, correct CAN bitrate, classical CAN v1, motor send/recv IDs, clear workspace around the actuator.
- Ensure every example that enables hardware attempts to disable on normal exit and interrupt.

**Non-Goals**

- No new public APIs unless an example cannot be made safe with the current surface.
- No mock-driven tutorial suite. Mock usage remains in tests.
- No CAN-FD flows; examples must pass `fd=false` and describe v1 classical CAN only.
- No automatic discovery or parameter query examples until the public API exposes Damiao parameter queries through `can-motor-control`/Python.
- No set-zero automation without explicit confirmation flags.

## Example Tiers

### Tier 0: Environment and interface checks

Examples in this tier must not send motor-control frames. They should verify and print:

- The selected interface name, usually `can0`.
- Whether `/sys/class/net/<interface>` exists.
- Whether users should run `ip link show <interface>` and `candump <interface>`.
- That v1 uses classical CAN and rejects CAN-FD.

Candidates: `examples/00_can_interface_check.py` and
`crates/can-motor-control/examples/can_interface_check.rs`.

### Tier 1: One motor, no motion

These examples test communication while minimizing physical risk.

Candidate examples:

- `examples/01_single_motor_enable_disable.py`
- `examples/02_single_motor_read_state.py`
- `crates/can-motor-control/examples/single_motor_enable_disable.rs`

The user supplies `--interface`, `--send-id`, `--recv-id`, and `--motor-type`. Defaults may mirror common Damiao/OpenArm values, but examples must print them before acting. `read_state` may call `enable`, `tick`, print `position`, `velocity`, `torque`, temperatures, then disable. It must not send a motion command.

### Tier 2: One motor, bounded control

These examples introduce commands one mode at a time and avoid infinite loops.

Candidate examples:

- `examples/03_single_motor_mit_hold.py`
- `examples/04_single_motor_control_modes.py`

`mit_hold` should default to low gains and zero setpoint for a short duration. `control_modes` should support selecting exactly one mode per run (`mit`, `pos-vel`, `vel`, `pos-force`) and default all command magnitudes to zero or near-zero. The user should have to opt into non-zero motion values through flags.

### Tier 3: Arm and gripper scenes

Multi-motor examples are only safe after one-motor bring-up.

Candidate examples:

- `examples/05_single_arm_state_monitor.py`
- `examples/06_single_arm_mit_hold.py`
- `examples/07_gripper_control.py`

Arm examples should accept either explicit IDs or a TOML path such as `configs/openarm_single.toml`. The monitor example should be read-oriented. The hold example should use conservative MIT values and bounded runtime. Gripper examples must use only the current public gripper methods (`enable`, `disable`, `mit_control`, `pos_vel_control`) unless implementation chooses to add safe PosForce gripper support separately.

### Tier 4: Calibration / set-zero

Set-zero is mechanically meaningful and potentially destructive when used at the wrong pose. It should be isolated.

Candidate: `examples/08_set_zero_calibration.py`.

This example must require an explicit confirmation flag such as `--i-understand-this-writes-zero`, print the target IDs, and use a disable → set-zero → disable sequence. If current Python gripper/arm APIs expose only group-level `set_zero_all`, the example must avoid pretending it can safely target arbitrary individual motors.

## Mock Policy

Mocks remain valuable for CI, shape tests, and Python wheel smoke tests. They should not be presented as the main example path.

- Keep `MockCanBus` tests in `tests/python/` and `crates/can-motor-control/tests/`.
- Do not add multiple separated mock examples.
- If a mock example is retained, name it as developer-only, e.g. `examples/dev_api_smoke.py`, and state that it does not validate hardware bring-up.

## Documentation

Update the top-level README with a short “Hardware bring-up examples” section listing the examples in order and marking each as one of:

- No frames sent.
- Sends enable/disable only.
- Sends bounded control commands.
- Calibration / persistent motor-state change.

Also document a minimal SocketCAN setup reminder and recommend running `candump` in a separate terminal before hardware examples.

## Verification Strategy

- Syntax-check Python examples without hardware.
- Run Rust tests and doctests normally.
- Run mock tests normally, but treat them as API/CI validation only.
- Gate any `vcan0` or real `can0` execution behind manual instructions or ignored tests; do not make CI depend on host CAN interfaces.
- For manual QA, run the no-frame interface check in this environment and record when real CAN execution is skipped due missing interface/hardware.
