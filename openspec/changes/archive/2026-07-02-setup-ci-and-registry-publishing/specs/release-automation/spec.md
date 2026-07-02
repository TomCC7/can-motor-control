## ADDED Requirements

### Requirement: Pull-request CI validates Rust, Python, docs, and packaging surfaces
The repository SHALL run pull-request and main-branch CI that validates the Rust workspace, Python binding package, documentation build, and release-relevant packaging checks before changes are merged.

#### Scenario: Pull request exercises all quality gates
- **WHEN** a pull request targets `main`
- **THEN** CI runs Rust formatting, Clippy with warnings denied, workspace tests, no_std codec builds, vendor dependency isolation, Python wheel build/import checks, Python smoke tests, and docs build checks

#### Scenario: CI artifacts prove wheel installability
- **WHEN** CI builds the `can-motor-control` Python wheel from `crates/can-motor-control-py`
- **THEN** the workflow installs the built wheel into a clean environment and imports `can_motor_control` successfully before reporting success

### Requirement: Release candidate validation is dry-run only
The repository SHALL provide a release-candidate validation path that proves all publishable artifacts can be built and packaged without uploading anything to PyPI or crates.io.

#### Scenario: Rust crates pass registry dry-runs in dependency order after upstream crates exist
- **WHEN** a maintainer runs release-candidate validation
- **THEN** the workflow runs crates.io dry-runs for `can-motor-codec`, `can-motor-damiao-codec`, and `can-motor-control` in dependency order when each crate's upstream workspace dependencies already exist on crates.io

#### Scenario: Initial crates.io release uses staged downstream checks
- **WHEN** a maintainer validates a first crates.io release before upstream workspace crates exist on crates.io
- **THEN** the workflow dry-runs the first publishable crate, skips downstream registry checks that cannot resolve yet, and the release guide tells the maintainer to rerun dry-runs after each upstream crate is published

#### Scenario: Python distributions pass PyPI compatibility checks
- **WHEN** a maintainer runs release-candidate validation
- **THEN** the workflow builds the Python wheel and sdist with PyPI-compatible tags and uploads them only as workflow artifacts

### Requirement: Tag-driven release publishes to registries through protected jobs
The repository SHALL publish registry artifacts only from an explicit release trigger guarded by a GitHub environment and registry-specific authentication.

#### Scenario: PyPI publish uses trusted publishing
- **WHEN** a release workflow publishes the Python package to PyPI
- **THEN** the publish job uses PyPI Trusted Publishing/OIDC with `id-token: write`, retrieves prebuilt distributions, and uploads them without storing a long-lived PyPI token

#### Scenario: crates.io publish preserves dependency order
- **WHEN** a release workflow publishes Rust crates to crates.io
- **THEN** it publishes `can-motor-codec` before `can-motor-damiao-codec`, publishes `can-motor-damiao-codec` before `can-motor-control`, and never publishes `can-motor-control-py` as a Rust crate

#### Scenario: first crates.io release is handled explicitly
- **WHEN** a crate has not yet been published to crates.io
- **THEN** the release guide explains that the initial publication requires a crates.io API token or manual maintainer publish before crates.io Trusted Publishing can be configured for subsequent releases

### Requirement: Release guide documents maintainer preconditions and commands
The repository SHALL include maintainer-facing release documentation that describes registry setup, version checks, dry-run commands, publish order, and post-publish verification.

#### Scenario: Maintainer follows documented preflight checks
- **WHEN** a maintainer prepares a release
- **THEN** the guide tells them to verify package names on PyPI and crates.io, confirm workspace/package versions, run dry-run packaging checks, and ensure protected GitHub environments are configured

#### Scenario: Maintainer verifies published artifacts
- **WHEN** registry publication completes
- **THEN** the guide tells the maintainer how to verify `pip install can-motor-control` and Cargo dependency installation against the newly published versions
