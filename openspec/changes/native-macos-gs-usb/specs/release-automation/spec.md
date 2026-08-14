## ADDED Requirements

### Requirement: Automation builds and validates native macOS arm64 distributions

CI, release-candidate validation, and tag-driven release automation SHALL build the Rust workspace and Python distribution on a pinned GitHub-hosted macOS arm64 runner in addition to existing Linux jobs. The macOS jobs SHALL verify the runner architecture, target-native API, wheel platform tag, clean installation, import, and Python smoke tests. Release automation SHALL include the validated macOS wheel in the distributions supplied to the protected PyPI publish job.

#### Scenario: Pull request validates macOS Rust surfaces
- **WHEN** a pull request targets `main`
- **THEN** a pinned macOS arm64 job asserts `uname -m` is `arm64` and runs formatting-independent Rust build, Clippy with warnings denied, and workspace tests for all macOS-compatible targets

#### Scenario: macOS wheel installs cleanly
- **WHEN** CI builds the Python distribution on macOS arm64
- **THEN** the wheel has a macOS arm64-compatible platform tag, installs into a clean environment with declared runtime dependencies, imports `can_motor_control`, exposes `GsUsbBus`, and does not expose `SocketCanBus`

#### Scenario: Linux wheel retains SocketCAN API
- **WHEN** existing Linux wheel validation imports `can_motor_control`
- **THEN** it exposes `SocketCanBus` and does not expose `GsUsbBus`

#### Scenario: Release candidate contains both platform wheels
- **WHEN** a maintainer runs release-candidate validation
- **THEN** workflow artifacts contain independently clean-installed Linux and macOS arm64 wheels without uploading either to PyPI

#### Scenario: Protected release publishes the macOS wheel
- **WHEN** the tag-driven PyPI publish job retrieves validated Python distributions
- **THEN** its distribution set includes the macOS arm64 wheel together with the supported Linux wheel and source distribution

