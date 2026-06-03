## Why

A Damiao motor only emits a state-feedback frame **in reply to a frame we send**;
`Robot.tick()` is receive-only, so a read loop that just enables and ticks gets
the enable-ack state once and then **freezes** (the bug seen in
`examples/read_single_motor.py` on hardware). The reference implementation
(`enactic/openarm_can`) reads state with a dedicated **refresh query** — CAN
`0x7FF` payload `[id_lo, id_hi, 0xCC, 0,0,0,0,0]` (`refresh_motor_status`) — which
requests feedback **without commanding motion**, then receives.

This repo already has that exact encoder (`DamiaoCodecExt::encode_refresh`), but
it is mislabeled "refresh from flash", is not reachable from a connected
`Robot`/`Arm`, and is not exposed to Python. This change wires it through as a
first-class `refresh` operation and switches the reader example to the
openarm-aligned `refresh → tick` loop.

This is the standalone refresh primitive only — the higher-level per-cycle
`step()` loop (one frame/motor, auto-refresh-when-idle) remains a separate future
change (`add-control-loop-step`) that will build on this.

## What Changes

- **`MotorCodec` trait**: add `encode_refresh(&self, motor) -> Result<Option<CanFrame>, CodecError>`
  with a default `Ok(None)` (unsupported). Vendors opt in; callers stay
  vendor-agnostic (no downcast).
- **`DamiaoCodec`**: implement `encode_refresh` returning the `0x7FF`/`0xCC`
  `refresh_motor_status` frame (reusing the existing `DamiaoCodecExt` bytes); fix
  the "refresh from flash" doc to "request a state-feedback frame — commands no
  motion."
- **`dm-control`**: add `MotorGroup::refresh_all()` and `Arm`/`Gripper`/`Robot::refresh()`
  that send one refresh frame per motor via the bus transport (send-only, like
  `enable_all`), skipping motors whose codec returns `None`. The decode/route path
  is unchanged — replies dispatch to motor state exactly like command replies.
- **Python bindings**: expose `arm.refresh()` / `robot.refresh()` (and gripper),
  GIL-released like `tick`; add `.pyi` stubs.
- **`examples/read_single_motor.py`**: loop becomes `arm.refresh(); robot.tick();
  render()` (one frame/motor/cycle, no motion); remove the stray debug print;
  update the docstring.

## Capabilities

### New Capabilities
- `motor-state-refresh`: a vendor-optional "request current motor state" operation
  that elicits a feedback frame without commanding motion, exposed from the codec
  through `Group`/`Robot` and the Python API; plus the read-loop pattern
  (`refresh → tick`) it enables.

### Modified Capabilities
<!-- The motor-codec / motor-group / robot-composition behavioral specs are not
     synced into openspec/specs/ (they live only in the archived walking-skeleton
     change), so there are no live requirement deltas to record here. The new
     trait method and Group/Robot/Python methods are additive. -->

## Impact

- **Code**:
  - `crates/motor-codec/src/codec.rs` (trait method + default)
  - `crates/dm-codec/src/codec.rs` (impl), `crates/dm-codec/src/ext.rs` (doc fix)
  - `crates/dm-control/src/group.rs`, `src/robot.rs` (`refresh_all`/`refresh`)
  - `crates/dm-control-py/src/robot.rs` + `python/dm_control/__init__.pyi` (bindings)
- **Examples**: `examples/read_single_motor.py` switches to `refresh → tick`.
  `02_single_motor_read_state.py` / `05_single_arm_state_monitor.py` have the same
  freeze and can adopt the same pattern (kept motion-free) — included as tasks.
- **Tests**: `encode_refresh` layout unit test (matches openarm's
  `create_refresh_command`); a `MockCanBus` test that `refresh()` issues one
  `send` per motor and no `drain`.
- **Docs**: note the refresh-poll read pattern in `docs/` / README bring-up table
  wording ("reads" → "refresh-polls state").
- **Not in scope**: the `step()`/staging control-loop (separate change); changing
  a motor's control mode; motor stiffness on enable (firmware/mode behavior —
  refresh commands no torque and does not alter it).
- **Dependencies / breaking**: none. The trait method has a default; all new
  surface is additive.
