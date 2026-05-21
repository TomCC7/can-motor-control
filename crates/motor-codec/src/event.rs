//! Decoded inbound-frame events.

/// A decoded inbound message from a motor.
#[derive(Copy, Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum Event {
    /// Motor state response (the per-tick mechanical state).
    State {
        /// recv CAN ID of the responding motor.
        motor_id: u32,
        /// Position (radians).
        q: f64,
        /// Velocity (rad/s).
        dq: f64,
        /// Estimated torque (Nm).
        tau: f64,
        /// MOSFET temperature (degrees C).
        t_mos: i16,
        /// Rotor temperature (degrees C).
        t_rotor: i16,
    },
    /// Reply to a parameter read or write (vendor-specific sub-protocol).
    ParamReply {
        /// recv CAN ID of the responding motor.
        motor_id: u32,
        /// Register identifier the reply refers to.
        rid: u16,
        /// Decoded value.
        value: ParamValue,
    },
    /// A motor fault notification.
    Fault {
        /// recv CAN ID of the responding motor.
        motor_id: u32,
        /// Vendor-defined fault code.
        code: u16,
    },
}

/// Union of value encodings used by vendor parameter sub-protocols.
#[derive(Copy, Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum ParamValue {
    /// IEEE-754 float (typical for gains, limits).
    Float(f64),
    /// Unsigned integer (typical for IDs, mode selectors).
    UInt(u32),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state_event_carries_all_five_fields() {
        let e = Event::State {
            motor_id: 0x11,
            q: 0.5,
            dq: 0.1,
            tau: 0.2,
            t_mos: 30,
            t_rotor: 35,
        };
        match e {
            Event::State {
                motor_id,
                q,
                dq,
                tau,
                t_mos,
                t_rotor,
            } => {
                assert_eq!(motor_id, 0x11);
                assert_eq!(q, 0.5);
                assert_eq!(dq, 0.1);
                assert_eq!(tau, 0.2);
                assert_eq!(t_mos, 30);
                assert_eq!(t_rotor, 35);
            }
            _ => panic!("wrong variant"),
        }
    }

    #[test]
    fn param_value_union() {
        let f = ParamValue::Float(1.5);
        let u = ParamValue::UInt(42);
        assert!(matches!(f, ParamValue::Float(_)));
        assert!(matches!(u, ParamValue::UInt(_)));
    }
}
