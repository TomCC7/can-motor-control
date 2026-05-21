# dm_control_rs

Rust control library for Damiao-family CAN motors, with first-class Python bindings.

## Status

v0.1.0 (in development) — walking-skeleton scope: single arm end-to-end over Linux SocketCAN. The architecture accommodates future vendors (Robostride, MyActuator, CubeMars) and CAN-FD; only Damiao on classical CAN ships in v1.

## Why

The two reference implementations — `cmjang/DM_Control_Python` and `enactic/openarm_can` — are either a single-file script that fuses sends with receives, or a C++ tree with a binding crate that rebuilds on every release. This library is built as a Rust core with a thin PyO3 binding so it installs as a single wheel and runs in realtime control loops without GIL contention.

## Installation

```bash
# Python (once 0.1.0 is published):
pip install dm_control

# Rust:
[dependencies]
dm-control = "0.1"
dm-codec   = "0.1"   # Damiao codec; add other vendor codec crates as needed
```

Build from source:

```bash
# Rust workspace
cargo build --workspace --release

# Python wheel
cd crates/dm-control-py
maturin build --release
pip install target/wheels/dm_control-*.whl
```

## Layout

```
crates/
  motor-codec/      # no_std, vendor-agnostic MotorCodec trait + shared types
  dm-codec/         # no_std, Damiao implementation of MotorCodec
  dm-control/       # std, transport (SocketCAN) + Robot/Group/Motor + builder
  dm-control-py/    # PyO3 bindings, built by maturin
configs/            # example TOML robot configs
examples/           # one Rust + one Python example
tests/python/       # Python smoke + integration tests
docs/               # architecture + multi-vendor + CAN-FD notes
openspec/           # source of truth: proposals, designs, specs, tasks
```

## Source of truth

The architecture, every requirement, and the implementation plan live under [`openspec/`](./openspec). New features should propose changes there first.

## Platforms

Linux only in v1 (SocketCAN is a Linux kernel feature). The codec crates compile cross-platform; macOS / Windows users can run unit tests against `MockCanBus`.

## License

Dual-licensed under Apache-2.0 OR MIT.
