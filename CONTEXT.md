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

**Command-tick pair**:
A control-loop exchange where one outbound command or query is followed by one `Robot.tick()` to receive that command's response. Do not expect `tick()` to produce fresh motor feedback unless a command or query was sent first.
_Avoid_: Passive polling, free-running feedback

**Send acceptance**:
Confirmation that a CAN transport has accepted responsibility for attempting an outbound frame. It does not mean the frame reached the wire, was transmitted successfully, or was acknowledged by another CAN node.
_Avoid_: Delivery confirmation, transmission acknowledgment

**Nominal bit timing**:
The CAN controller parameters selected from an adapter's reported clock and timing constraints to achieve the requested classical-CAN bitrate.
_Avoid_: Bitrate table, clock preset

**gs_usb device selector**:
The exclusive choice of either a USB serial number or an enumeration index used with a vendor/product ID pair to identify one adapter. A serial number is stable deployment identity; an index is a bring-up convenience tied to current enumeration order.
_Avoid_: Device precedence, serial/index fallback

**Transport parity**:
Equivalent observable `CanBus` behavior across platform-native transports despite different operating-system or device machinery. Construction, send acceptance, inbound draining, error reporting, and ownership-driven cleanup should differ only where the hardware requires it.
_Avoid_: Identical implementation, platform fallback
