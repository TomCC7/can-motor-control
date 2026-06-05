## 1. Codec trait + Damiao implementation

- [x] 1.1 Add `encode_refresh(&self, motor: MotorRef) -> Result<Option<CanFrame>, CodecError>` to the `MotorCodec` trait (`crates/motor-codec/src/codec.rs`) with a default body returning `Ok(None)`.
- [x] 1.2 Implement `encode_refresh` on `DamiaoCodec` (`crates/damiao-codec/src/codec.rs`) returning `Ok(Some(..))` with the `0x7FF`/`0xCC` frame; reuse/delegate to the existing `DamiaoCodecExt::encode_refresh` bytes.
- [x] 1.3 Fix the `ext.rs` doc: "refresh from flash" → "request a state-feedback frame (`refresh_motor_status`, 0xCC on 0x7FF) — commands no motion."
- [x] 1.4 Unit test: `DamiaoCodec::encode_refresh` for send id `0x01` yields id `0x7FF`, len 8, payload `[0x01,0x00,0xCC,0,0,0,0,0]` (matches openarm `create_refresh_command`).

## 2. can-motor-control: batch refresh over motors

- [x] 2.1 Add `MotorGroup::refresh_all()` that, per motor, calls `bus.codec.encode_refresh` and sends via `bus.transport.send` when `Some` (skip `None`); send-only, never drains. Mirror `enable_all`/`disable_all` (`group.rs:206/223`).
- [x] 2.2 Expose `Arm::refresh()` / `Gripper::refresh()` delegating to `refresh_all`.
- [x] 2.3 Add `Robot::refresh()` that calls each group's refresh; return `Err(Error::NotConnected)` if not connected.
- [x] 2.4 Test (MockCanBus): `refresh()` on a 3-motor arm records exactly three `send` and zero `drain_inbound_nonblocking` (via `recorded_calls()`).

## 3. Python bindings

- [x] 3.1 Expose `Arm.refresh()`, `Gripper.refresh()`, `Robot.refresh()` in `crates/can-motor-control-py/src/robot.rs`, GIL-released (`Python::allow_threads`) like `tick`.
- [x] 3.2 Add the three `refresh()` signatures to `crates/can-motor-control-py/python/can_motor_control/__init__.pyi`.

## 4. Examples + docs

- [x] 4.1 `examples/read_single_motor.py`: change the loop to `arm.refresh(); robot.tick(args.deadline_us); render(...)`; remove the stray `print(motor)` debug line; update the docstring to say it refresh-polls (no motion commanded).
- [x] 4.2 Apply the same `refresh → tick` pattern to `examples/02_single_motor_read_state.py` and `examples/05_single_arm_state_monitor.py` (kept motion-free); update their docstrings.
- [x] 4.3 Update the README bring-up table wording for 02/05 from "reads" to "refresh-polls state".

## 5. Verify

- [x] 5.1 `cargo build --workspace`, `cargo test --workspace`, `cargo clippy --workspace` clean.
- [x] 5.2 Rebuild the extension (`uv sync --reinstall-package can_motor_control`); `uv run --with pytest python -m pytest tests/python` green.
- [x] 5.3 No-hardware: `uv run python examples/read_single_motor.py --mock` runs, renders, disables, exits 0.
- [ ] 5.4 Hardware (manual, user): `uv run python examples/read_single_motor.py --interface can0 --fd` shows live-updating state with no motion commanded.
