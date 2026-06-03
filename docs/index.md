# dm_control

Rust control library for Damiao-family CAN motors, with first-class Python
bindings.

The Python package `dm_control` is a thin [PyO3](https://pyo3.rs) layer over a
Rust core, so it installs as a single wheel and runs in realtime control loops
without GIL contention.

## Where to go

- **[SocketCAN setup](socketcan-setup.md)** — bring up a Linux CAN interface
  for the first time.
- **[CAN-FD](can-fd.md)** — notes on classical CAN vs CAN-FD.
- **[Multi-vendor](multi-vendor.md)** — how the vendor-agnostic codec seam works.
- **[Python API](reference.md)** — the full `dm_control` reference.
- **[Rust API](rust.md)** — pointer to the rustdoc for the Rust crates.

## Quick taste

```python
import numpy as np
from dm_control import Robot

# Connect + enable on entry, disable on exit.
with Robot.from_config("configs/single_arm.toml") as robot:
    arm = robot["arm"]
    for _ in range(1000):
        # One row [kp, kd, q, dq, tau] per motor.
        cmds = np.zeros((len(arm), 5))
        arm.mit_control(cmds)
        robot.tick(per_bus_deadline_us=2000)
        print(arm.positions())
```

## Status

v0.1.0 (in development) — walking-skeleton scope: a single arm end-to-end over
Linux SocketCAN. The architecture accommodates future vendors (Robostride,
MyActuator, CubeMars) and CAN-FD; only Damiao on classical CAN ships in v1.

Linux only in v1 (SocketCAN is a Linux kernel feature). The codec crates
compile cross-platform; other platforms can run against `MockCanBus`.
