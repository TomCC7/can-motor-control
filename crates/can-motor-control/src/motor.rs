//! Per-motor identity and state cache.

use motor_codec::{Event, MotorTypeId};

/// Vendor fault code as reported in [`motor_codec::Event::Fault`].
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct FaultCode(pub u16);

/// Identity + state-cache for one motor.
///
/// State mutators are `pub(crate)` so user code cannot directly mutate the
/// cache; updates happen exclusively from the bus-router dispatch path
/// inside [`crate::Robot::tick`].
#[derive(Debug, Clone)]
pub struct Motor {
    name: String,
    motor_type: MotorTypeId,
    send_id: u32,
    recv_id: u32,
    position: f64,
    velocity: f64,
    torque: f64,
    t_mos: i16,
    t_rotor: i16,
    is_enabled: bool,
    fault: Option<FaultCode>,
    state_sequence: u64,
}

impl Motor {
    pub(crate) fn new(name: String, motor_type: MotorTypeId, send_id: u32, recv_id: u32) -> Self {
        Self {
            name,
            motor_type,
            send_id,
            recv_id,
            position: 0.0,
            velocity: 0.0,
            torque: 0.0,
            t_mos: 0,
            t_rotor: 0,
            is_enabled: false,
            fault: None,
            state_sequence: 0,
        }
    }

    /// Human-readable motor name from the robot config.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Vendor type identifier (e.g. `MotorTypeId::Damiao(<DM4340>)`).
    pub fn motor_type(&self) -> MotorTypeId {
        self.motor_type
    }

    /// CAN ID this motor accepts commands on.
    pub fn send_id(&self) -> u32 {
        self.send_id
    }

    /// CAN ID this motor emits state and replies from.
    pub fn recv_id(&self) -> u32 {
        self.recv_id
    }

    /// Most recently received position (radians).
    pub fn position(&self) -> f64 {
        self.position
    }

    /// Most recently received velocity (rad/s).
    pub fn velocity(&self) -> f64 {
        self.velocity
    }

    /// Most recently received torque estimate (Nm).
    pub fn torque(&self) -> f64 {
        self.torque
    }

    /// MOSFET temperature (degrees C).
    pub fn temperature_mos(&self) -> i16 {
        self.t_mos
    }

    /// Rotor temperature (degrees C).
    pub fn temperature_rotor(&self) -> i16 {
        self.t_rotor
    }

    /// True after a successful enable ACK; false after disable.
    pub fn is_enabled(&self) -> bool {
        self.is_enabled
    }

    /// Latched fault code, if any.
    pub fn fault(&self) -> Option<FaultCode> {
        self.fault
    }

    pub(crate) fn state_sequence(&self) -> u64 {
        self.state_sequence
    }

    pub(crate) fn apply_event(&mut self, ev: &Event) {
        match *ev {
            Event::State {
                q,
                dq,
                tau,
                t_mos,
                t_rotor,
                ..
            } => {
                self.position = q;
                self.velocity = dq;
                self.torque = tau;
                self.t_mos = t_mos;
                self.t_rotor = t_rotor;
                self.fault = None;
                self.state_sequence = self.state_sequence.wrapping_add(1);
                // is_enabled stays whatever the lifecycle setter last set.
            }
            Event::Fault { code, .. } => {
                self.fault = Some(FaultCode(code));
            }
            Event::ParamReply { .. } => {
                // v1 doesn't cache param replies on Motor; user reads them via
                // a future param-poll API.
            }
            _ => {
                // Non-exhaustive: ignore unknown event variants.
            }
        }
    }

    pub(crate) fn set_enabled(&mut self, on: bool) {
        self.is_enabled = on;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn newly_constructed_has_zero_state() {
        let m = Motor::new("j0".into(), MotorTypeId::Damiao(3), 0x01, 0x11);
        assert_eq!(m.name(), "j0");
        assert_eq!(m.send_id(), 0x01);
        assert_eq!(m.recv_id(), 0x11);
        assert_eq!(m.position(), 0.0);
        assert!(!m.is_enabled());
        assert!(m.fault().is_none());
    }

    #[test]
    fn apply_state_updates_cache() {
        let mut m = Motor::new("j0".into(), MotorTypeId::Damiao(3), 0x01, 0x11);
        m.apply_event(&Event::State {
            motor_id: 0x11,
            q: 0.5,
            dq: 0.1,
            tau: 0.2,
            t_mos: 30,
            t_rotor: 35,
        });
        assert_eq!(m.position(), 0.5);
        assert_eq!(m.velocity(), 0.1);
        assert_eq!(m.torque(), 0.2);
        assert_eq!(m.temperature_mos(), 30);
        assert_eq!(m.temperature_rotor(), 35);
    }

    #[test]
    fn apply_fault_latches_code() {
        let mut m = Motor::new("j0".into(), MotorTypeId::Damiao(3), 0x01, 0x11);
        m.apply_event(&Event::Fault {
            motor_id: 0x11,
            code: 7,
        });
        assert_eq!(m.fault(), Some(FaultCode(7)));
    }

    #[test]
    fn set_enabled_flips_flag() {
        let mut m = Motor::new("j0".into(), MotorTypeId::Damiao(3), 0x01, 0x11);
        assert!(!m.is_enabled());
        m.set_enabled(true);
        assert!(m.is_enabled());
        m.set_enabled(false);
        assert!(!m.is_enabled());
    }
}
