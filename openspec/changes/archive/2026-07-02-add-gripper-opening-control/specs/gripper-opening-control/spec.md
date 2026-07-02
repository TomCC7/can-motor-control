## ADDED Requirements

### Requirement: Gripper opening is commanded as a normalized value
The system SHALL expose a gripper opening command where `0.0` means fully closed and `1.0` means fully open. The command SHALL reject openings outside the inclusive `[0.0, 1.0]` range and SHALL NOT silently clamp them.

#### Scenario: Command a valid opening
- **WHEN** a caller commands `set_opening(0.5)` on a calibrated gripper
- **THEN** the system commands the gripper toward the midpoint between the calibrated closed and open endpoints

#### Scenario: Reject opening below range
- **WHEN** a caller commands `set_opening(-0.1)`
- **THEN** the system returns an error indicating the opening must be between `0.0` and `1.0`

#### Scenario: Reject opening above range
- **WHEN** a caller commands `set_opening(1.1)`
- **THEN** the system returns an error indicating the opening must be between `0.0` and `1.0`

### Requirement: Open and close are opening-command conveniences
The system SHALL expose gripper `open(current)` and `close(current)` convenience commands equivalent to commanding opening `1.0` and `0.0` respectively.

#### Scenario: Open convenience command
- **WHEN** a caller invokes `open()` on a calibrated gripper
- **THEN** the system commands the gripper toward opening `1.0`

#### Scenario: Close convenience command
- **WHEN** a caller invokes `close()` on a calibrated gripper
- **THEN** the system commands the gripper toward opening `0.0`

### Requirement: Opening commands optionally regulate per-unit current
The system SHALL allow normalized opening commands to provide an optional per-unit motor current value for the underlying position-force command path. The API SHALL name this value as current, not physical force, unless a physical force calibration is introduced by a future change. Current resolution SHALL use this precedence: per-call current, configured default opening current, then a conservative library default.

#### Scenario: Command opening with explicit current
- **WHEN** a caller commands `set_opening(0.5, current = 0.2)` on a calibrated gripper
- **THEN** the system uses the calibrated midpoint target and the explicit per-unit current value in the underlying command

#### Scenario: Command opening with default current
- **WHEN** a caller commands `set_opening(0.5)` on a calibrated gripper
- **THEN** the system uses the calibrated midpoint target and the configured or library default per-unit current for opening control

#### Scenario: Open with explicit current
- **WHEN** a caller invokes `open(current = 0.2)` on a calibrated gripper
- **THEN** the system commands opening `1.0` using the explicit per-unit current value

#### Scenario: Close with default current
- **WHEN** a caller invokes `close()` on a calibrated gripper
- **THEN** the system commands opening `0.0` using the configured or library default per-unit current

### Requirement: Default opening current is configurable
The system SHALL allow gripper configuration to specify a default opening current used when normalized opening commands omit a per-call current. The configured value SHALL be interpreted as per-unit motor current, not physical jaw force.

#### Scenario: Configured default opening current is used
- **WHEN** a gripper is configured with a default opening current and the caller invokes `set_opening(0.5)` without a current argument
- **THEN** the system uses the configured default opening current in the underlying command

#### Scenario: Per-call current overrides configured default
- **WHEN** a gripper is configured with a default opening current and the caller invokes `set_opening(0.5, current = 0.3)`
- **THEN** the system uses `0.3` as the per-unit current for that command

### Requirement: Opening direction is required for opening calibration
The system SHALL support gripper opening direction configuration with explicit values `increasing_position` and `decreasing_position`. Opening direction SHALL define whether increasing raw motor position increases or decreases normalized opening.

#### Scenario: Increasing position opens the gripper
- **WHEN** a gripper is configured with `opening_direction = "increasing_position"`
- **THEN** opening calibration treats increasing raw motor position as motion toward opening `1.0`

#### Scenario: Decreasing position opens the gripper
- **WHEN** a gripper is configured with `opening_direction = "decreasing_position"`
- **THEN** opening calibration treats decreasing raw motor position as motion toward opening `1.0`

#### Scenario: Invalid opening direction is rejected
- **WHEN** a gripper configuration uses an unknown `opening_direction` value
- **THEN** configuration loading fails with a configuration error

### Requirement: Opening calibration runs during enable before ready
The system SHALL perform per-session opening calibration for normalized gripper control during the robot enable lifecycle, after connection and before the robot is ready for normal opening commands. Opening calibration SHALL discover the raw motor positions that correspond to fully open and fully closed openings.

#### Scenario: Enable calibrates gripper opening
- **WHEN** a robot with a gripper configured for opening control is connected and enabled
- **THEN** the enable lifecycle calibrates that gripper's open and closed endpoints before returning success

#### Scenario: Context manager calibrates before user code
- **WHEN** Python code enters a `with Robot.from_config(...) as robot:` block for a robot with a gripper configured for opening control
- **THEN** the context-manager entry completes opening calibration before user code inside the block runs

#### Scenario: Opening command before calibration is rejected
- **WHEN** a caller invokes `set_opening` before opening calibration has completed
- **THEN** the system returns a lifecycle error and does not command normalized gripper motion

### Requirement: Opening calibration failure fails the lifecycle
The system SHALL fail the robot enable lifecycle if opening calibration fails. Calibration failure SHALL prevent normal opening commands from being accepted.

#### Scenario: Calibration timeout fails enable
- **WHEN** opening calibration times out before discovering both endpoints
- **THEN** `Robot.enable()` fails and the gripper is not considered ready for normalized opening commands

#### Scenario: Calibration detects insufficient span
- **WHEN** opening calibration discovers open and closed positions whose span is too small to be valid
- **THEN** `Robot.enable()` fails with a lifecycle or calibration error

### Requirement: Raw gripper controls remain available
The system SHALL preserve existing raw gripper controls for hardware bring-up and advanced users. Adding normalized opening control SHALL NOT remove `set_mode`, `mit_control`, `pos_vel_control`, `pos_force_control`, `refresh`, `enable`, or `disable`.

#### Scenario: Raw PosForce remains callable
- **WHEN** a caller invokes the existing raw `pos_force_control` gripper method
- **THEN** the system accepts the command using the existing raw motor-oriented semantics

### Requirement: Normalized gripper examples are provided in Rust and Python
The project SHALL provide Rust and Python examples that demonstrate normalized gripper opening control as the primary user-facing gripper workflow.

#### Scenario: Python normalized opening example exists
- **WHEN** a user runs the Python normalized gripper example with its required safety confirmation
- **THEN** the example connects, enables and calibrates the gripper, commands open, commands an intermediate opening, demonstrates configured or per-call current, commands close, and disables safely

#### Scenario: Rust normalized opening example exists
- **WHEN** a user runs the Rust normalized gripper example with its required safety confirmation
- **THEN** the example connects, enables and calibrates the gripper, commands open, commands an intermediate opening, demonstrates configured or per-call current, commands close, and disables safely
