## Why

The project is preparing for a first public release, but its current public
identity (`can-motor-control`, `can-motor-control`, `can_motor_control`) incorrectly suggests a
Damiao-only library. The implementation has already grown into a vendor-agnostic
CAN motor-control stack with a Damiao codec as one vendor backend and explicit
room for Robostride, MyActuator, CubeMars, robot arms, grippers, and generic
motor groups.

Renaming before publication avoids locking users into misleading package names,
wheel names, docs URLs, import paths, and examples. The target public name is
`can-motor-control`, with Rust/Python import spelling `can_motor_control`.

## What Changes

- **BREAKING: Public package identity changes** from the Damiao-oriented
  `can-motor-control` / `can_motor_control` family to `can-motor-control` / `can_motor_control`.
- **Rust workspace naming** is updated so the primary runtime crate publishes as
  `can-motor-control` and imports as `can_motor_control`.
- **Python distribution and import path** are updated so users install
  `can-motor-control` and import `can_motor_control`; the PyO3 module name,
  `#[pyclass(module = ...)]` metadata, exceptions, pure-Python package directory,
  stubs, examples, tests, docs, and CI wheel smoke checks move together.
- **Vendor crate naming is made explicit**: `damiao-codec` becomes `damiao-codec`
  unless implementation discovers a strong reason to keep the shorter published
  crate name. The vendor module remains `damiao` under the Python package.
- **Trait crate naming is intentionally conservative**: keep `motor-codec` unless
  the implementation pass finds it unpublished and worth renaming to
  `can-motor-codec`; it is already vendor-neutral and accurately describes the
  shared contract.
- **Docs, examples, configs, CI, changelog, and OpenSpec references** are updated
  so release-facing text consistently describes the project as CAN motor control,
  not Damiao-only control.
- **Registry/publishing checks** are part of the implementation: crates.io, PyPI,
  docs.rs, and the GitHub repo namespace must be verified for the normalized name
  variants before publish/dry-run.

## Capabilities

### New Capabilities
- `project-identity`: the canonical project, Rust crate, Python distribution,
  Python import, documentation, repository, and publishing identity for the
  `can-motor-control` release.

### Modified Capabilities
- `python-docs-site`: the docs site currently builds/imports `can_motor_control`; it
  must build/import `can_motor_control`, preload the renamed native module, and
  render the renamed public package without stale aliases.

## Impact

- **Rust code and metadata**: root `Cargo.toml`, workspace dependency aliases,
  `crates/can-motor-control`, `crates/can-motor-control-py`, `crates/damiao-codec`, package
  descriptions, `use can_motor_control::...` / `use damiao_codec::...` imports, examples,
  integration tests, and publish dry-run commands.
- **Python packaging**: `crates/can-motor-control-py/pyproject.toml`, `module-name`,
  package directory, `.pyi` stubs, `__all__`, exceptions' `__module__`, logging
  namespace, tests, examples, generated wheel names, uv root dependency, and docs
  extra rebuild commands.
- **Documentation and release text**: README, `docs/`, `mkdocs.yml`, rustdoc
  references, CHANGELOG publish order, OpenSpec specs/designs/tasks where they
  describe current public behavior, and any user-facing install/import snippets.
- **CI and verification**: GitHub Actions artifact names, wheel install/import
  smoke, vendor-isolation grep, docs build, cargo fmt/clippy/test, Python pytest,
  maturin build/develop, and crates.io dry-runs in dependency order.
- **Dependencies**: no new runtime dependencies expected.
