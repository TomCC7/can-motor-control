## ADDED Requirements

### Requirement: Single local command builds and serves the docs site

The project SHALL provide a single command that builds the `can_motor_control` native extension and then builds/serves the documentation site locally, with no CI or publishing step required. Running the command on a clean checkout (after dev dependencies are installed) SHALL produce a browsable site.

#### Scenario: Developer serves the site locally

- **WHEN** a developer runs the documented local docs command (e.g. `make docs` or the equivalent serve target)
- **THEN** the native extension is built via `maturin develop` and a local web server serves the rendered site reachable in a browser

#### Scenario: Developer produces a static build

- **WHEN** a developer runs the documented static-build docs command
- **THEN** a static HTML site is generated into a local output directory without starting a server

#### Scenario: Command fails clearly when dev dependencies are missing

- **WHEN** the docs command runs without the documentation toolchain installed
- **THEN** it exits non-zero with a message identifying the missing tool and how to install the docs dependency group

### Requirement: Python API reference is generated from the package

The site SHALL include a Python API reference auto-generated from the `can_motor_control` package via mkdocstrings/griffe, covering the publicly exported surface (`Robot`, `RobotBuilder`, `Arm`, `Gripper`, `Motor`, `MotorGroup`, `MotorSpec`, `CanFrame`, the bus transports, the `damiao` codec module, and the error types). Member signatures SHALL be rendered, using the `.pyi` stubs where the compiled extension does not expose them.

#### Scenario: Public class appears in the reference

- **WHEN** the site is built and a user opens the Python API reference page
- **THEN** each publicly exported class from `can_motor_control.__all__` is listed with its methods and their signatures

#### Scenario: Reference stays in sync with the package

- **WHEN** a new public method is added to the package and the site is rebuilt
- **THEN** the method appears in the rendered reference without manual page edits

### Requirement: API prose comes from PyO3 docstrings and reaches the REPL

User-facing documentation prose for the Python API SHALL be authored as Rust `///` doc comments on the PyO3 `#[pyclass]` / `#[pymethods]` items so it compiles into the module's `__doc__`. The same text SHALL be the source rendered on the API reference page, so the rendered site and `help()` cannot diverge.

#### Scenario: help() shows the documented prose

- **WHEN** a user calls `help()` (or Jupyter `?`) on a documented public class or method after building the extension
- **THEN** the docstring authored in the Rust source is displayed

#### Scenario: Rendered reference shows the same prose

- **WHEN** the same item is viewed on the rendered API reference page
- **THEN** the displayed description matches the docstring authored in the Rust source

### Requirement: Existing prose guides are part of the site

The existing guides under `docs/` (SocketCAN setup, CAN-FD, multi-vendor) SHALL be reachable from the site navigation, alongside an overview/home page and a bridge page linking to the Rust rustdoc.

#### Scenario: Guides appear in navigation

- **WHEN** a user opens the built site
- **THEN** the SocketCAN setup, CAN-FD, and multi-vendor guides are reachable from the site navigation

#### Scenario: Rust docs are reachable

- **WHEN** a user looks for the Rust API
- **THEN** the site provides a page or nav entry linking to the rustdoc output

### Requirement: Documentation toolchain is dev-only

The documentation dependencies (mkdocs-material, mkdocstrings/griffe) SHALL be declared in a development/optional dependency group and SHALL NOT be added to the published wheel's runtime dependencies.

#### Scenario: Runtime install excludes docs tooling

- **WHEN** the `can_motor_control` wheel is installed without the docs extra
- **THEN** none of the documentation toolchain packages are pulled in as runtime dependencies
