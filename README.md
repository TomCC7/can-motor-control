# can-motor-control

Rust control library for Damiao-family CAN motors, with first-class Python bindings.

## Status

v0.1.0 (in development) — walking-skeleton scope: single arm end-to-end over Linux SocketCAN. CAN-FD is supported (classical CAN remains the default); see [`docs/can-fd.md`](./docs/can-fd.md). The architecture accommodates future vendors (Robostride, MyActuator, CubeMars). Only the Damiao codec ships in v1, and it emits classical 8-byte frames on either bus type.

## Why

The two reference implementations — `cmjang/DM_Control_Python` and `enactic/openarm_can` — are either a single-file script that fuses sends with receives, or a C++ tree with a binding crate that rebuilds on every release. This library is built as a Rust core with a thin PyO3 binding so it installs as a single wheel and runs in realtime control loops without GIL contention.

## Installation

```bash
# Python (once 0.1.0 is published):
pip install can-motor-control

# Rust:
[dependencies]
can-motor-control = "0.1"
damiao-codec   = "0.1"   # Damiao codec; add other vendor codec crates as needed
```

Build from source:

```bash
# Rust workspace
cargo build --workspace --release

# Python wheel
cd crates/can-motor-control-py
maturin build --release
pip install target/wheels/can_motor_control-*.whl
```

## Layout

```
crates/
  motor-codec/      # no_std, vendor-agnostic MotorCodec trait + shared types
  damiao-codec/         # no_std, Damiao implementation of MotorCodec
  can-motor-control/       # std, transport (SocketCAN) + Robot/Group/Motor + builder
  can-motor-control-py/    # PyO3 bindings, built by maturin
configs/            # example TOML robot configs
examples/           # Python hardware bring-up ladder (00_-06_) + Rust example
tests/python/       # Python smoke + integration tests
docs/               # architecture + multi-vendor + CAN-FD notes
openspec/           # source of truth: proposals, designs, specs, tasks
```

## Hardware bring-up examples

Real hardware bring-up should be done by walking the `examples/00_*` ->
`examples/06_*` ladder in order. Each step adds exactly one kind of risk;
do not skip steps when bringing up an arm for the first time. Mock and
`MockCanBus`-driven tests prove the API surface and CI wiring, **not**
that real hardware will respond correctly.

| Python example                         | Rust example           | Sends                  | Hardware required             |
| -------------------------------------- | ---------------------- | ---------------------- | ----------------------------- |
| `00_can_interface_check.py`            | `00_can_interface_check` | nothing              | none (checks `/sys/class/net`) |
| `01_single_motor_enable_disable.py`    | `01_single_motor_enable_disable` | enable + disable | one powered motor             |
| `02_read_single_motor.py`              | --                     | enable + live refresh-poll + disable | one powered motor |
| `03_single_motor_mit_hold.py`          | `03_single_motor_mit_hold` | bounded MIT hold   | one powered motor             |
| `04_single_motor_control_modes.py`     | --                     | one bounded mode/run   | one powered motor             |
| `05_gripper_control.py`                | `05_gripper_control`   | gripper PosForce sequence, MIT, or PosVel | one gripper motor |
| `06_read_openarm_motor.py`             | --                     | live OpenArm zero-torque reader | configured OpenArm single arm |

Each example prints its assumptions before sending anything and disables
on exit. Durations are hard-capped; live readers accept `--mock` for API-shape
checks, but mock execution does not validate hardware.

For first-time setup of a Linux SocketCAN interface, see
[`docs/socketcan-setup.md`](./docs/socketcan-setup.md).
Run Rust examples with `cargo run -p can-motor-control --example <Rust example> -- ...`.

## Documentation

A local docs site (Python API reference + the guides under `docs/`) is built
with MkDocs. The Python API prose is generated from the package's docstrings,
so the native extension is built first. The environment is managed by
[uv](https://docs.astral.sh/uv/) and pinned by `uv.lock` for reproducibility
(the root `pyproject.toml` is a dev-only environment, not a published package).

```bash
# Serve with live reload at http://127.0.0.1:8000
make docs

# Or render a static site into ./site
make docs-build
```

Both targets run `uv sync --reinstall-package can-motor-control` first, which builds
the extension (editable, via maturin) and installs the locked docs toolchain,
then invoke MkDocs. No separate virtualenv or `pip install` step is needed —
uv provisions everything from `uv.lock`. The same two-step flow ports directly
to CI later (`uv sync --frozen` + `uv run mkdocs build`).

The Rust crates are documented separately with rustdoc:
`cargo doc --no-deps --workspace --open`.

## Releasing

Maintainers should follow the release runbook in
[`docs/release.md`](./docs/release.md) before publishing to PyPI or crates.io.
It covers registry setup, release-candidate dry-runs, crates.io publish order,
PyPI Trusted Publishing, and post-publish install checks.

## Source of truth

The architecture, every requirement, and the implementation plan live under [`openspec/`](./openspec). New features should propose changes there first.

## Platforms

Linux only in v1 (SocketCAN is a Linux kernel feature). The codec crates compile cross-platform; macOS / Windows users can run unit tests against `MockCanBus`.

## License

Dual-licensed under Apache-2.0 OR MIT.
