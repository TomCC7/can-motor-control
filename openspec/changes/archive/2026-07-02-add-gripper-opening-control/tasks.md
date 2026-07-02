## 1. Domain and configuration model

- [x] 1.1 Add Rust types for gripper opening direction and per-session opening calibration state.
- [x] 1.2 Extend gripper group configuration parsing to accept `opening_direction = "increasing_position" | "decreasing_position"`.
- [x] 1.3 Extend gripper group configuration parsing to accept a default opening current.
- [x] 1.4 Add configuration error tests for invalid opening direction values, invalid default opening current values, and missing direction where normalized opening control is required.
- [x] 1.5 Update example TOML gripper comments/config to show `opening_direction` and default opening current.

## 2. Rust gripper opening API

- [x] 2.1 Add Rust `Gripper` methods for `set_opening`, `open`, and `close` using normalized opening semantics.
- [x] 2.2 Reject openings outside `[0.0, 1.0]` with an explicit error.
- [x] 2.3 Map calibrated opening endpoints to raw motor position, supporting both increasing-position and decreasing-position mechanisms.
- [x] 2.4 Support per-call and configured default per-unit current for opening commands while retaining existing raw gripper methods.
- [x] 2.5 Add Rust tests for valid opening mapping, out-of-range rejection, convenience methods, and raw method preservation.

## 3. Opening calibration lifecycle

- [x] 3.1 Add a robot enable lifecycle stage that runs opening calibration for grippers configured for normalized opening control.
- [x] 3.2 Implement conservative calibration motion with bounded current, velocity, timeout, minimum movement, and minimum endpoint-span checks.
- [x] 3.3 Ensure calibration failure fails `Robot.enable()` and leaves normalized opening commands unavailable.
- [x] 3.4 Add lifecycle tests for successful calibration, timeout/failure, insufficient span, and calling `set_opening` before calibration.

## 4. Python API and typing

- [x] 4.1 Expose `Gripper.set_opening`, `Gripper.open`, and `Gripper.close` through PyO3.
- [x] 4.2 Expose optional keyword-only `current` for `set_opening`, `open`, and `close` in Python, falling back to configured default opening current.
- [x] 4.3 Update Python `.pyi` stubs with precise normalized gripper signatures.
- [x] 4.4 Add Python tests for `RobotBuilder.add_gripper`, `robot["grip"]` returning `Gripper`, normalized opening commands, range errors, lifecycle errors, and stubs/runtime alignment.

## 5. Examples and documentation

- [x] 5.1 Add or update a Python gripper example that demonstrates automatic calibration on enable, then `open`, intermediate `set_opening`, and `close`.
- [x] 5.2 Add or update a Rust gripper example mirroring the Python normalized opening flow.
- [x] 5.3 Require explicit safety confirmation flags in both normalized gripper examples.
- [x] 5.4 Update README/example inventory and changelog limitations to describe normalized opening support.
- [x] 5.5 Update PyO3 docstrings/docs site inputs so the Python API reference documents opening, current, calibration lifecycle, and raw controls.

## 6. Verification

- [x] 6.1 Run Rust unit/integration tests for the workspace.
- [x] 6.2 Build/install the Python package and run Python pytest coverage.
- [x] 6.3 Build or check Rust and Python examples without touching hardware where possible.
- [x] 6.4 Run docs/reference validation sufficient to confirm new gripper API prose/signatures render or are importable.
