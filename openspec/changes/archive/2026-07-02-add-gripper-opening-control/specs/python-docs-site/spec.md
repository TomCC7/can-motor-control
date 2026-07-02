## MODIFIED Requirements

### Requirement: Python API reference is generated from the package

The site SHALL include a Python API reference auto-generated from the `can_motor_control` package via mkdocstrings/griffe, covering the publicly exported surface (`Robot`, `RobotBuilder`, `Arm`, `Gripper`, `Motor`, `MotorGroup`, `MotorSpec`, `CanFrame`, the bus transports, the `damiao` codec module, and the error types). Member signatures SHALL be rendered, using the `.pyi` stubs where the compiled extension does not expose them. The rendered `Gripper` reference SHALL include the normalized opening API, including `set_opening`, `open`, `close`, optional per-unit current arguments, default opening current behavior, opening calibration lifecycle behavior, and the fact that raw gripper controls remain available.

#### Scenario: Public class appears in the reference

- **WHEN** the site is built and a user opens the Python API reference page
- **THEN** each publicly exported class from `can_motor_control.__all__` is listed with its methods and their signatures

#### Scenario: Reference stays in sync with the package

- **WHEN** a new public method is added to the package and the site is rebuilt
- **THEN** the method appears in the rendered reference without manual page edits

#### Scenario: Gripper opening API appears in the reference

- **WHEN** the site is built and a user opens the `Gripper` reference
- **THEN** the normalized opening methods, per-call/default current behavior, and opening semantics are visible alongside the raw gripper control methods

### Requirement: Existing prose guides are part of the site

The existing guides under `docs/` (SocketCAN setup, CAN-FD, multi-vendor) SHALL be reachable from the site navigation, alongside an overview/home page, a bridge page linking to the Rust rustdoc, and the normalized gripper opening example or guide.

#### Scenario: Guides appear in navigation

- **WHEN** a user opens the built site
- **THEN** the SocketCAN setup, CAN-FD, multi-vendor, and normalized gripper opening materials are reachable from the site navigation

#### Scenario: Rust docs are reachable

- **WHEN** a user looks for the Rust API
- **THEN** the site provides a page or nav entry linking to the rustdoc output
