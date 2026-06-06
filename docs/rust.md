# Rust API

The Rust crates are documented with **rustdoc**. There is no hosted copy yet
(the repository is private), so generate and open the docs locally:

```bash
# Build and open the rustdoc for the whole workspace.
cargo doc --no-deps --workspace --open
```

This opens the reference for each crate:

| crates.io package | Rust import crate | Role |
| ----- | ----- | ---- |
| `can-motor-codec` | `motor_codec` | `no_std`, vendor-agnostic `MotorCodec` trait + shared types |
| `can-motor-damiao-codec` | `damiao_codec` | `no_std`, Damiao implementation of `MotorCodec` |
| `can-motor-control` | `can_motor_control` | `std`, SocketCAN transport + `Robot` / group / motor + builder |

The Python package documented under [Python API](reference.md) is a thin PyO3
binding over `can-motor-control`; the Rust docs are the place to understand the
underlying types and the vendor-agnostic codec seam.
