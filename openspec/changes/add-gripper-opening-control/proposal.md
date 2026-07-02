## Why

The current gripper API exposes a one-motor raw control surface, so end users must reason about motor position, control mode, and current when they really want to command how open the gripper should be. Formal gripper support should make the common operation normalized and safe: `0.0` means closed, `1.0` means open, and raw motor controls remain available for bring-up and advanced use.

## What Changes

- Add a normalized gripper opening API where users command an `opening` in `[0.0, 1.0]`.
- Add convenience `open()` and `close()` methods built on the same opening semantics.
- Add optional per-unit current regulation for opening commands, backed by the existing PosForce command path.
- Add a configured default opening current used by `set_opening`, `open`, and `close` when no per-call current is provided.
- Add per-session opening calibration as part of the robot enable lifecycle, before normal gripper opening control becomes available.
- Add gripper configuration for `opening_direction`, using explicit values for whether increasing raw motor position opens or closes the gripper.
- Make opening commands fail if opening calibration has not completed or fails during enable.
- Add Rust and Python examples showing the normalized gripper workflow as the primary user-facing gripper control path.
- Keep existing raw gripper methods (`set_mode`, `mit_control`, `pos_vel_control`, `pos_force_control`, `refresh`, `enable`, `disable`) for advanced use.

## Capabilities

### New Capabilities
- `gripper-opening-control`: Normalized gripper opening commands, automatic per-session opening calibration, opening-direction configuration, and Rust/Python example coverage.

### Modified Capabilities
- `python-docs-site`: Document the normalized gripper API and lifecycle behavior in the generated Python API reference and user-facing examples.

## Impact

- Rust control crate: `Gripper`, robot lifecycle, config parsing, errors, and tests.
- Python bindings: `Gripper` methods, stubs, lifecycle errors, docs, and tests.
- Config files: gripper groups gain `opening_direction` and default opening current for normalized opening support.
- Examples: add or replace gripper examples to demonstrate `set_opening`, `open`, and `close` in both Rust and Python.
- Documentation: update gripper limitations and API reference so they match the formal gripper opening surface.
