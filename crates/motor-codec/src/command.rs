//! Vendor-agnostic motor command surface.

/// A control command directed at a single motor.
///
/// Vendor-specific commands (e.g. Damiao's `0x7FF` parameter sub-protocol) do
/// not appear here; reach them via vendor-extension traits exposed by the
/// vendor's codec crate.
#[derive(Copy, Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum Command {
    /// MIT impedance control: kp, kd gains plus q (position), dq (velocity),
    /// tau (feed-forward torque) setpoints.
    Mit {
        /// Position gain.
        kp: f64,
        /// Velocity gain.
        kd: f64,
        /// Position setpoint (radians).
        q: f64,
        /// Velocity setpoint (rad/s).
        dq: f64,
        /// Feed-forward torque (Nm).
        tau: f64,
    },
    /// Position + velocity setpoint control.
    PosVel {
        /// Position setpoint (radians).
        q: f64,
        /// Velocity setpoint (rad/s).
        dq: f64,
    },
    /// Pure velocity control.
    Vel {
        /// Velocity setpoint (rad/s).
        dq: f64,
    },
    /// Position + velocity + current-percent-units control.
    PosForce {
        /// Position setpoint (radians).
        q: f64,
        /// Velocity setpoint (rad/s).
        dq: f64,
        /// Current setpoint in per-unit (0.0 to 1.0).
        i_pu: f64,
    },
}

/// Tagged discriminant of [`Command`], suitable for error-reporting
/// "this codec does not support mode X".
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum CommandKind {
    /// Corresponds to [`Command::Mit`].
    Mit,
    /// Corresponds to [`Command::PosVel`].
    PosVel,
    /// Corresponds to [`Command::Vel`].
    Vel,
    /// Corresponds to [`Command::PosForce`].
    PosForce,
}

impl Command {
    /// Tagged discriminant.
    pub fn kind(&self) -> CommandKind {
        match self {
            Command::Mit { .. } => CommandKind::Mit,
            Command::PosVel { .. } => CommandKind::PosVel,
            Command::Vel { .. } => CommandKind::Vel,
            Command::PosForce { .. } => CommandKind::PosForce,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_four_modes_construct() {
        let mit = Command::Mit {
            kp: 50.0,
            kd: 1.0,
            q: 0.0,
            dq: 0.0,
            tau: 0.0,
        };
        let pv = Command::PosVel { q: 1.0, dq: 2.0 };
        let v = Command::Vel { dq: 1.0 };
        let pf = Command::PosForce {
            q: 1.0,
            dq: 2.0,
            i_pu: 0.5,
        };
        assert_eq!(mit.kind(), CommandKind::Mit);
        assert_eq!(pv.kind(), CommandKind::PosVel);
        assert_eq!(v.kind(), CommandKind::Vel);
        assert_eq!(pf.kind(), CommandKind::PosForce);
    }

    #[test]
    fn debug_round_trip() {
        let cmd = Command::Mit {
            kp: 50.0,
            kd: 1.0,
            q: 0.0,
            dq: 0.0,
            tau: 0.0,
        };
        let s = alloc::format!("{:?}", cmd);
        assert!(s.contains("Mit"));
        assert!(s.contains("50"));
    }
}
