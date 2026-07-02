# can-motor-control

Domain language for the motor-control and robot-composition concepts in this repository.

## Language

**Opening**:
A normalized commanded gripper aperture where `0.0` means fully closed and `1.0` means fully open. It is not raw motor position and not necessarily measured jaw distance.
_Avoid_: Position, normalized position, aperture fraction

**Opening calibration**:
A per-session discovery of the motor positions that correspond to fully closed and fully open gripper openings.
_Avoid_: Static calibration, startup homing

**Opening direction**:
Whether increasing raw motor position increases or decreases gripper opening. It is configuration for opening calibration, not a runtime control command.
_Avoid_: Direction sign, motor polarity

**Default opening current**:
The per-gripper configured per-unit motor current used by opening commands when a call does not provide an explicit current.
_Avoid_: Default force, grip force
