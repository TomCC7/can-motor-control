# project-identity Specification

## Purpose

Define the repository's canonical public identity across Rust crates, Python distribution/import names, documentation, and release-facing text.

## Requirements

### Requirement: Canonical public identity uses can-motor-control
The project SHALL use `can-motor-control` as the canonical public project,
repository, Rust package, Python distribution, documentation, and release-facing
name. Runtime import/module identifiers SHALL use the valid identifier spelling
`can_motor_control`.

#### Scenario: Rust package name maps to Rust import name
- **WHEN** a Rust user depends on the published package `can-motor-control`
- **THEN** the crate is importable as `can_motor_control`

#### Scenario: Python distribution maps to Python import name
- **WHEN** a Python user installs the published distribution `can-motor-control`
- **THEN** `import can_motor_control` succeeds

#### Scenario: Release-facing text uses the canonical name
- **WHEN** a user reads README, docs, examples, CI artifact names, changelog, or install snippets
- **THEN** the public project identity is presented as `can-motor-control` / `can_motor_control`, not `can-motor-control`, `can-motor-control`, or `can_motor_control`

### Requirement: Damiao remains a vendor namespace, not the project identity
Damiao-specific protocol support SHALL remain isolated to the Damiao vendor
codec and the `damiao` vendor namespace. The top-level project name and primary
runtime package MUST NOT imply Damiao-only scope.

#### Scenario: Damiao codec is imported below the renamed package
- **WHEN** a Python user imports the bundled Damiao codec
- **THEN** the codec is available from `can_motor_control.damiao`

#### Scenario: Rust vendor codec remains separate from runtime crate
- **WHEN** `cargo tree -p can-motor-control --edges normal` is inspected
- **THEN** the primary runtime crate does not depend on Damiao or any other vendor codec as a normal runtime dependency

### Requirement: Vendor and trait crate names are explicit and publishable
The implementation SHALL choose publishable crate names for the shared trait
crate and Damiao codec before release. The shared trait crate SHALL publish as
`can-motor-codec`; the Damiao codec SHALL publish as
`can-motor-damiao-codec`. Their Rust import crate names SHALL remain
`motor_codec` and `damiao_codec`.

#### Scenario: Publish names are checked before dry-run
- **WHEN** the rename implementation reaches publish verification
- **THEN** crates.io availability has been checked for `can-motor-control`, the selected Damiao codec crate name, and the selected shared trait crate name

#### Scenario: Publish order reflects selected crate names
- **WHEN** the changelog and release instructions describe publishing
- **THEN** they list the selected crate names in dependency order before the Python wheel

### Requirement: Normalized name variants are verified before publication
The implementation SHALL verify normalized name variants before publishing so
the selected names do not collide across crates.io, PyPI, docs.rs, or the GitHub
repository namespace. For Python, `can-motor-control`, `can_motor_control`, and
`can.motor.control` MUST be treated as equivalent lookup names. For crates.io,
hyphen and underscore variants MUST be treated as collision-equivalent.

#### Scenario: Registry checks are documented
- **WHEN** the rename implementation is complete
- **THEN** the final verification notes include the registry/name checks that were run and their results

#### Scenario: Name collision blocks publish steps
- **WHEN** any required registry check finds an occupied or conflicting name
- **THEN** publishing steps are not marked complete until the name decision is updated or the conflict is explicitly accepted

### Requirement: Legacy names do not remain in release-facing surfaces
The implementation MUST prevent stale legacy public identity references in
release-facing surfaces.

After the rename, release-facing code, docs, examples, tests, generated docs
configuration, and CI MUST NOT require users to type the old top-level names
`can-motor-control`, `can-motor-control`, or `can_motor_control`.

#### Scenario: Repository search finds no stale public identity
- **WHEN** the implementation runs a repository search for old top-level public names outside archived historical OpenSpec context
- **THEN** no active release-facing source, docs, CI, examples, or tests still instruct users to install, import, or depend on the old top-level identity

#### Scenario: Historical OpenSpec context is allowed when clearly archived
- **WHEN** old names appear in archived OpenSpec artifacts that describe past decisions
- **THEN** those references are either updated where they describe current release behavior or intentionally left only as historical context
