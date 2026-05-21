//! [`DamiaoCodec`] — implements [`motor_codec::MotorCodec`] for Damiao motors.

use motor_codec::{
    BusCapabilities, CanFrame, CodecError, Command, Event, FrameFlags, Limits, MotorCodec,
    MotorRef, MotorTypeId,
};

use crate::bitpack::{pack_mit_payload, unpack_state_payload};
use crate::limits::limits_for;
use crate::types::DamiaoMotorType;

const VENDOR: &str = "damiao";

/// Damiao motor codec.
///
/// Construct with [`DamiaoCodec::new`]. Implements
/// [`MotorCodec`] for the vendor-agnostic surface and [`crate::DamiaoCodecExt`]
/// for the Damiao-specific `0x7FF` parameter sub-protocol.
#[derive(Default, Debug, Clone)]
pub struct DamiaoCodec {
    bound_caps: Option<BusCapabilities>,
}

impl DamiaoCodec {
    /// Construct a new codec. `bind_to_bus` must be called before any encode.
    pub fn new() -> Self {
        Self { bound_caps: None }
    }

    fn motor_type(&self, id: MotorTypeId) -> Result<DamiaoMotorType, CodecError> {
        match id {
            MotorTypeId::Damiao(d) => {
                DamiaoMotorType::from_discriminant(d).ok_or(CodecError::UnknownMotorType {
                    vendor: VENDOR,
                    type_id: d,
                })
            }
            _ => Err(CodecError::UnknownMotorType {
                vendor: VENDOR,
                type_id: 0,
            }),
        }
    }

    fn encode_special(
        &self,
        motor: MotorRef<'_>,
        trailing_byte: u8,
    ) -> Result<CanFrame, CodecError> {
        let payload = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, trailing_byte];
        CanFrame::classical(motor.send_id, &payload).map_err(|_| CodecError::DecodeFailed {
            reason: "classical frame construction failed",
        })
    }
}

impl MotorCodec for DamiaoCodec {
    fn vendor_name(&self) -> &'static str {
        VENDOR
    }

    fn supports(&self, motor_type: MotorTypeId) -> bool {
        matches!(motor_type, MotorTypeId::Damiao(d) if DamiaoMotorType::from_discriminant(d).is_some())
    }

    fn limits(&self, motor_type: MotorTypeId) -> Result<Limits, CodecError> {
        let t = self.motor_type(motor_type)?;
        Ok(limits_for(t))
    }

    fn bind_to_bus(&mut self, caps: BusCapabilities) {
        // v1 stores the caps but always emits classical regardless. v2 may
        // gate FD emission on caps.supports_fd.
        self.bound_caps = Some(caps);
    }

    fn encode_enable(&self, motor: MotorRef<'_>) -> Result<CanFrame, CodecError> {
        self.encode_special(motor, 0xFC)
    }

    fn encode_disable(&self, motor: MotorRef<'_>) -> Result<CanFrame, CodecError> {
        self.encode_special(motor, 0xFD)
    }

    fn encode_set_zero(&self, motor: MotorRef<'_>) -> Result<CanFrame, CodecError> {
        self.encode_special(motor, 0xFE)
    }

    fn encode_command(&self, motor: MotorRef<'_>, cmd: &Command) -> Result<CanFrame, CodecError> {
        let t = self.motor_type(motor.motor_type)?;
        let lim = limits_for(t);
        match cmd {
            Command::Mit { kp, kd, q, dq, tau } => {
                check_range("q", *q, lim.p_max)?;
                check_range("dq", *dq, lim.v_max)?;
                check_range("tau", *tau, lim.t_max)?;
                check_unsigned("kp", *kp, 500.0)?;
                check_unsigned("kd", *kd, 5.0)?;
                let payload =
                    pack_mit_payload(*q, *dq, *kp, *kd, *tau, lim.p_max, lim.v_max, lim.t_max);
                CanFrame::classical(motor.send_id, &payload).map_err(|_| CodecError::DecodeFailed {
                    reason: "MIT frame construction failed",
                })
            }
            Command::PosVel { q, dq } => {
                let mut payload = [0u8; 8];
                payload[0..4].copy_from_slice(&(*q as f32).to_le_bytes());
                payload[4..8].copy_from_slice(&(*dq as f32).to_le_bytes());
                CanFrame::classical(0x100 + motor.send_id, &payload).map_err(|_| {
                    CodecError::DecodeFailed {
                        reason: "PosVel frame construction failed",
                    }
                })
            }
            Command::Vel { dq } => {
                let mut payload = [0u8; 8];
                payload[0..4].copy_from_slice(&(*dq as f32).to_le_bytes());
                CanFrame::classical(0x200 + motor.send_id, &payload).map_err(|_| {
                    CodecError::DecodeFailed {
                        reason: "Vel frame construction failed",
                    }
                })
            }
            Command::PosForce { q, dq, i_pu } => {
                let mut payload = [0u8; 8];
                payload[0..4].copy_from_slice(&(*q as f32).to_le_bytes());
                let dq_u = (dq * 100.0) as u16;
                let i_u = (i_pu * 10000.0) as u16;
                payload[4..6].copy_from_slice(&dq_u.to_le_bytes());
                payload[6..8].copy_from_slice(&i_u.to_le_bytes());
                CanFrame::classical(0x300 + motor.send_id, &payload).map_err(|_| {
                    CodecError::DecodeFailed {
                        reason: "PosForce frame construction failed",
                    }
                })
            }
            _ => Err(CodecError::CommandNotSupported {
                vendor: VENDOR,
                mode: cmd.kind(),
            }),
        }
    }

    fn decode(&self, frame: &CanFrame) -> Result<Option<Event>, CodecError> {
        // Damiao state responses come on the recv_id assigned to the motor, with
        // the response command in the top nibble of byte 0.
        if frame.is_fd() {
            // v1 never emits FD frames from Damiao; ignore as not-ours.
            return Ok(None);
        }
        if frame.flags.contains(FrameFlags::REMOTE_REQUEST) {
            return Ok(None);
        }
        if frame.len != 8 {
            // Damiao always uses 8 bytes; a different length is not a Damiao state frame.
            return Ok(None);
        }
        // Damiao state response byte 0 layout (per cmjang `__process_packet`):
        //   high nibble = err code (0 = no fault)
        //   low nibble  = cmd_id triggering this response (1=MIT, 2=PosVel,
        //                 3=Vel, 4=PosForce). Other low-nibble values are not
        //                 control-mode state responses (e.g. parameter sub-
        //                 protocol replies on 0x7FF).
        let byte0 = frame.payload()[0];
        let cmd_id = byte0 & 0x0f;
        let err = (byte0 >> 4) & 0x0f;
        if !matches!(cmd_id, 1..=6) {
            return Ok(None);
        }
        // Limits: the codec doesn't have a per-recv_id motor-type registry in
        // v1, so it uses DM4340 limits as the OpenArm walking-skeleton default.
        // A future change will register per-motor limits at bind time so mixed-
        // SKU buses decode each motor with its own limits.
        let payload: [u8; 8] =
            frame
                .payload()
                .try_into()
                .map_err(|_| CodecError::DecodeFailed {
                    reason: "expected 8-byte payload",
                })?;
        let dm = limits_for(DamiaoMotorType::DM4340);
        let (_, _, q, dq, tau, t_mos, t_rotor) =
            unpack_state_payload(&payload, dm.p_max, dm.v_max, dm.t_max);
        if err != 0 {
            return Ok(Some(Event::Fault {
                motor_id: frame.id,
                code: err as u16,
            }));
        }
        Ok(Some(Event::State {
            motor_id: frame.id,
            q,
            dq,
            tau,
            t_mos,
            t_rotor,
        }))
    }
}

fn check_range(field: &'static str, x: f64, magnitude: f64) -> Result<(), CodecError> {
    if x.abs() > magnitude {
        Err(CodecError::OutOfRange { field })
    } else {
        Ok(())
    }
}

fn check_unsigned(field: &'static str, x: f64, max: f64) -> Result<(), CodecError> {
    if !(0.0..=max).contains(&x) {
        Err(CodecError::OutOfRange { field })
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::boxed::Box;

    fn ref_motor(name: &str, mt: DamiaoMotorType, send: u32, recv: u32) -> MotorRef<'_> {
        MotorRef {
            motor_type: mt.into(),
            send_id: send,
            recv_id: recv,
            name,
        }
    }

    #[test]
    fn constructible_as_trait_object() {
        let c: Box<dyn MotorCodec> = Box::new(DamiaoCodec::new());
        assert_eq!(c.vendor_name(), "damiao");
    }

    #[test]
    fn supports_all_known_skus() {
        let c = DamiaoCodec::new();
        for d in 0..=12u16 {
            assert!(c.supports(MotorTypeId::Damiao(d)), "discriminant {d}");
            assert!(c.limits(MotorTypeId::Damiao(d)).is_ok());
        }
    }

    #[test]
    fn rejects_unknown_and_other_vendors() {
        let c = DamiaoCodec::new();
        assert!(!c.supports(MotorTypeId::Damiao(0xFFFF)));
        assert!(!c.supports(MotorTypeId::Robostride(0)));
        assert!(matches!(
            c.limits(MotorTypeId::Damiao(0xFFFF)),
            Err(CodecError::UnknownMotorType {
                vendor: "damiao",
                type_id: 0xFFFF,
            })
        ));
    }

    #[test]
    fn enable_disable_setzero_byte_patterns() {
        let c = DamiaoCodec::new();
        let m = ref_motor("g", DamiaoMotorType::DM4310, 0x05, 0x18);
        let e = c.encode_enable(m).unwrap();
        assert_eq!(e.id, 0x05);
        assert_eq!(e.len, 8);
        assert_eq!(
            e.payload(),
            &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
        );

        let d = c.encode_disable(m).unwrap();
        assert_eq!(d.payload()[7], 0xFD);
        let z = c.encode_set_zero(m).unwrap();
        assert_eq!(z.payload()[7], 0xFE);
    }

    #[test]
    fn mit_frame_id_equals_send_id_and_not_fd() {
        let c = DamiaoCodec::new();
        let m = ref_motor("j0", DamiaoMotorType::DM4340, 0x01, 0x11);
        let f = c
            .encode_command(
                m,
                &Command::Mit {
                    kp: 50.0,
                    kd: 1.0,
                    q: 0.0,
                    dq: 0.0,
                    tau: 0.0,
                },
            )
            .unwrap();
        assert_eq!(f.id, 0x01);
        assert_eq!(f.len, 8);
        assert!(!f.is_fd());
    }

    #[test]
    fn mit_out_of_range_tau() {
        let c = DamiaoCodec::new();
        let m = ref_motor("j0", DamiaoMotorType::DM4340, 0x01, 0x11);
        let r = c.encode_command(
            m,
            &Command::Mit {
                kp: 0.0,
                kd: 0.0,
                q: 0.0,
                dq: 0.0,
                tau: 1000.0,
            },
        );
        assert!(matches!(r, Err(CodecError::OutOfRange { field: "tau" })));
    }

    #[test]
    fn posvel_uses_0x100_offset() {
        let c = DamiaoCodec::new();
        let m = ref_motor("j0", DamiaoMotorType::DM4340, 0x01, 0x11);
        let f = c
            .encode_command(m, &Command::PosVel { q: 1.0, dq: 2.0 })
            .unwrap();
        assert_eq!(f.id, 0x101);
        assert_eq!(&f.payload()[0..4], &1.0f32.to_le_bytes());
        assert_eq!(&f.payload()[4..8], &2.0f32.to_le_bytes());
    }

    #[test]
    fn vel_uses_0x200_offset() {
        let c = DamiaoCodec::new();
        let m = ref_motor("j0", DamiaoMotorType::DM4340, 0x01, 0x11);
        let f = c.encode_command(m, &Command::Vel { dq: 1.5 }).unwrap();
        assert_eq!(f.id, 0x201);
        assert_eq!(&f.payload()[0..4], &1.5f32.to_le_bytes());
    }

    #[test]
    fn posforce_uses_0x300_offset_with_integer_scaling() {
        let c = DamiaoCodec::new();
        let m = ref_motor("j0", DamiaoMotorType::DM4340, 0x01, 0x11);
        let f = c
            .encode_command(
                m,
                &Command::PosForce {
                    q: 1.0,
                    dq: 2.0,
                    i_pu: 0.5,
                },
            )
            .unwrap();
        assert_eq!(f.id, 0x301);
        assert_eq!(&f.payload()[0..4], &1.0f32.to_le_bytes());
        assert_eq!(&f.payload()[4..6], &200u16.to_le_bytes());
        assert_eq!(&f.payload()[6..8], &5000u16.to_le_bytes());
    }

    #[test]
    fn decode_foreign_frame_returns_none() {
        let c = DamiaoCodec::new();
        // byte 0 low nibble = 0 means cmd_id 0, outside the 1..=6 control-mode
        // response range, so this frame is not a Damiao state response.
        let f = CanFrame::classical(0x11, &[0x00; 8]).unwrap();
        assert!(matches!(c.decode(&f), Ok(None)));
        // 0x99 in byte 0 → cmd_id = 9, also outside the response range.
        let f = CanFrame::classical(0x11, &[0x99, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        assert!(matches!(c.decode(&f), Ok(None)));
    }

    #[test]
    fn decode_truncated_returns_none() {
        let c = DamiaoCodec::new();
        let f = CanFrame::classical(0x11, &[0x11, 0x00, 0x00]).unwrap();
        assert!(matches!(c.decode(&f), Ok(None)));
    }

    #[test]
    fn decode_state_frame_round_trips_within_lsb() {
        let c = DamiaoCodec::new();
        let lim = limits_for(DamiaoMotorType::DM4340);
        let (q, dq, tau) = (1.0, 0.5, 5.0);
        let q_u = crate::bitpack::float_to_uint(q, -lim.p_max, lim.p_max, 16);
        let dq_u = crate::bitpack::float_to_uint(dq, -lim.v_max, lim.v_max, 12);
        let tau_u = crate::bitpack::float_to_uint(tau, -lim.t_max, lim.t_max, 12);
        let payload = [
            // byte 0: high nibble = err (0), low nibble = cmd_id (1 = MIT response)
            0x01,
            ((q_u >> 8) & 0xff) as u8,
            (q_u & 0xff) as u8,
            ((dq_u >> 4) & 0xff) as u8,
            ((((dq_u & 0xf) << 4) | ((tau_u >> 8) & 0xf)) & 0xff) as u8,
            (tau_u & 0xff) as u8,
            30,
            35,
        ];
        let f = CanFrame::classical(0x11, &payload).unwrap();
        match c.decode(&f).unwrap().unwrap() {
            Event::State {
                motor_id,
                q: qo,
                dq: dqo,
                tau: tauo,
                t_mos,
                t_rotor,
            } => {
                assert_eq!(motor_id, 0x11);
                assert!((qo - q).abs() < 0.001);
                assert!((dqo - dq).abs() < 0.01);
                assert!((tauo - tau).abs() < 0.05);
                assert_eq!(t_mos, 30);
                assert_eq!(t_rotor, 35);
            }
            _ => panic!("expected State"),
        }
    }

    #[test]
    fn classical_emission_regardless_of_bound_caps() {
        let m = ref_motor("j0", DamiaoMotorType::DM4340, 0x01, 0x11);
        for caps in [BusCapabilities::classical(), BusCapabilities::fd()] {
            let mut c = DamiaoCodec::new();
            c.bind_to_bus(caps);
            let f = c
                .encode_command(
                    m,
                    &Command::Mit {
                        kp: 0.0,
                        kd: 0.0,
                        q: 0.0,
                        dq: 0.0,
                        tau: 0.0,
                    },
                )
                .unwrap();
            assert!(!f.is_fd(), "caps={caps:?} produced FD frame");
            assert_eq!(f.len, 8);
        }
    }

    extern crate alloc;
}
