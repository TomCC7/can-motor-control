# Rust API

The Rust crates are documented with **rustdoc** and deployed alongside this
MkDocs site:

<ul>
  <li><a href="../rustdoc/index.html">Rust workspace rustdoc</a></li>
  <li><a href="../rustdoc/can_motor_control/index.html"><code>can_motor_control</code></a></li>
  <li><a href="../rustdoc/damiao_codec/index.html"><code>damiao_codec</code></a></li>
  <li><a href="../rustdoc/motor_codec/index.html"><code>motor_codec</code></a></li>
</ul>

Generate the same docs locally with:

```bash
make docs-build
```

The hosted rustdoc contains the reference for each crate:

| crates.io package | Rust import crate | Role |
| ----- | ----- | ---- |
| `can-motor-codec` | `motor_codec` | `no_std`, vendor-agnostic `MotorCodec` trait + shared types |
| `can-motor-damiao-codec` | `damiao_codec` | `no_std`, Damiao implementation of `MotorCodec` |
| `can-motor-control` | `can_motor_control` | `std`, SocketCAN transport + `Robot` / group / motor + builder |

The Python package documented under [Python API](reference.md) is a thin PyO3
binding over `can-motor-control`; the Rust docs are the place to understand the
underlying types and the vendor-agnostic codec seam.
