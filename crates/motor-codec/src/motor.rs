//! Vendor-tagged motor identifiers and value types.

/// Vendor-tagged motor type identifier.
///
/// Each variant carries an opaque `u16` discriminant that the owning vendor's
/// codec interprets internally — no two vendors need to coordinate their
/// discriminant spaces.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum MotorTypeId {
    /// Damiao motor SKU (DM4310, DM4340, ...). The `u16` is the SKU discriminant
    /// defined in `can-motor-damiao-codec`.
    Damiao(u16),
    /// Robostride motor SKU. Reserved for a future vendor codec crate.
    Robostride(u16),
}

/// Borrowed view of a motor's identity, suitable for passing into codec encode
/// methods without taking ownership of the motor's mutable state.
#[derive(Copy, Clone, Debug)]
pub struct MotorRef<'a> {
    /// The motor's vendor type identifier.
    pub motor_type: MotorTypeId,
    /// CAN ID this motor accepts commands on.
    pub send_id: u32,
    /// CAN ID this motor emits state and replies from.
    pub recv_id: u32,
    /// Human-readable motor name from the robot config.
    pub name: &'a str,
}

/// Per-motor-type physical limits used to scale MIT commands and unscale
/// state replies.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Limits {
    /// Position magnitude limit (radians).
    pub p_max: f64,
    /// Velocity magnitude limit (rad/s).
    pub v_max: f64,
    /// Torque magnitude limit (Nm).
    pub t_max: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn damiao_variant_round_trips() {
        let id = MotorTypeId::Damiao(7);
        match id {
            MotorTypeId::Damiao(d) => assert_eq!(d, 7),
            _ => panic!("wrong variant"),
        }
    }

    #[test]
    fn non_exhaustive_match_falls_through() {
        // Simulate the consumer pattern in design.md Decision 11.
        fn classify(t: MotorTypeId) -> &'static str {
            match t {
                MotorTypeId::Damiao(_) => "damiao",
                _ => "other",
            }
        }
        assert_eq!(classify(MotorTypeId::Damiao(0)), "damiao");
        assert_eq!(classify(MotorTypeId::Robostride(0)), "other");
    }

    #[test]
    fn motor_ref_construction() {
        let r = MotorRef {
            motor_type: MotorTypeId::Damiao(0),
            send_id: 0x01,
            recv_id: 0x11,
            name: "j0",
        };
        assert_eq!(r.name, "j0");
        assert_eq!(r.send_id, 0x01);
    }
}
