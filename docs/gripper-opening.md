# Gripper opening control

Normalized gripper opening control lets user code command the gripper by aperture
instead of raw motor position:

- `0.0` means fully closed.
- `1.0` means fully open.
- `current` is motor current in per-unit, not physical jaw force.

## Configuration

Normalized opening control requires an opening direction so startup calibration
knows which raw motor direction opens the mechanism:

```toml
[[group]]
name              = "grip"
kind              = "gripper"
bus               = "main"
opening_direction = "increasing_position" # or "decreasing_position"
default_current   = 0.2                    # per-unit motor current
motor = { name = "g", type = "DM4310", send_id = 0x08, recv_id = 0x18 }
```

## Lifecycle

Opening calibration runs during `Robot.enable()` and Python context-manager
entry, before normal opening commands are accepted.

```text
connect -> enable -> opening calibration -> ready for set_opening/open/close
```

If calibration fails, enabling fails and normalized opening commands remain
unavailable.

## Python

```python
with robot:
    gripper = robot["grip"]
    gripper.open()
    gripper.set_opening(0.5, current=0.2)
    gripper.close()
```

## Rust

```rust
robot.connect()?;
robot.enable()?;

let gripper = robot
    .group_mut("grip")
    .and_then(|group| group.as_gripper_mut())
    .ok_or("missing gripper")?;

gripper.open(None)?;
gripper.set_opening(0.5, Some(0.2))?;
gripper.close(None)?;
```

For hardware bring-up, see `examples/05_gripper_opening_control.py` and the
Rust `05_gripper_opening_control` example.
