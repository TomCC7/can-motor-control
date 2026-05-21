//! Builder input types.

use motor_codec::MotorTypeId;

/// User-supplied motor specification for the [`crate::RobotBuilder`].
#[derive(Clone, Debug, PartialEq)]
pub struct MotorSpec {
    /// Human-readable motor name unique within the group.
    pub name: String,
    /// Vendor type identifier (e.g. `MotorTypeId::Damiao(<DM4340 disc>)`).
    pub motor_type: MotorTypeId,
    /// CAN ID this motor accepts commands on.
    pub send_id: u32,
    /// CAN ID this motor emits state and replies from.
    pub recv_id: u32,
}

impl MotorSpec {
    /// Convenience constructor.
    pub fn new(
        name: impl Into<String>,
        motor_type: impl Into<MotorTypeId>,
        send_id: u32,
        recv_id: u32,
    ) -> Self {
        Self {
            name: name.into(),
            motor_type: motor_type.into(),
            send_id,
            recv_id,
        }
    }
}

/// Group kind discriminator the builder uses.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum GroupSpecKind {
    /// Articulated arm.
    Arm,
    /// One-motor gripper.
    Gripper,
    /// Generic catch-all.
    Generic,
}
