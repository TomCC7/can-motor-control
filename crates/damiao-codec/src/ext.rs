//! [`DamiaoCodecExt`] — the Damiao-specific parameter sub-protocol on CAN ID
//! `0x7FF`.
//!
//! Reached from the rest of the stack via downcast on `&dyn MotorCodec`
//! (see `can-motor-control` `Arm::codec_ext::<DamiaoCodec>()`).

use motor_codec::{CanFrame, MotorRef, ParamValue};

use crate::codec::DamiaoCodec;
use crate::types::DamiaoRid;

const PARAM_ID: u32 = 0x7FF;
const CMD_READ: u8 = 0x33;
const CMD_WRITE: u8 = 0x55;
const CMD_SAVE: u8 = 0xAA;
const CMD_REFRESH: u8 = 0xCC;

/// Damiao-only extension: read/write per-motor parameters on CAN ID `0x7FF`.
///
/// `DamiaoCodec` is the only implementor. The trait stands alone so callers
/// can reach it via `&dyn` downcast without bringing in `MotorCodec`.
pub trait DamiaoCodecExt {
    /// Encode a "read parameter" frame (command byte `0x33`).
    fn encode_read_param(&self, motor: MotorRef<'_>, rid: DamiaoRid) -> CanFrame;

    /// Encode a "write parameter" frame (command byte `0x55`).
    fn encode_write_param(&self, motor: MotorRef<'_>, rid: DamiaoRid, val: ParamValue) -> CanFrame;

    /// Encode a "save settings to flash" frame (command byte `0xAA`).
    fn encode_save_to_flash(&self, motor: MotorRef<'_>) -> CanFrame;

    /// Encode a "request state feedback" frame (Damiao `refresh_motor_status`,
    /// command byte `0xCC` on `0x7FF`). The motor replies with a feedback frame;
    /// this commands no motion. (Byte-for-byte the openarm
    /// `create_refresh_command`.)
    fn encode_refresh(&self, motor: MotorRef<'_>) -> CanFrame;
}

fn header(motor: MotorRef<'_>, cmd: u8, rid: DamiaoRid) -> [u8; 4] {
    let id = motor.send_id as u16;
    [
        (id & 0xff) as u8,
        ((id >> 8) & 0xff) as u8,
        cmd,
        u8::from(rid),
    ]
}

impl DamiaoCodecExt for DamiaoCodec {
    fn encode_read_param(&self, motor: MotorRef<'_>, rid: DamiaoRid) -> CanFrame {
        let mut payload = [0u8; 8];
        payload[0..4].copy_from_slice(&header(motor, CMD_READ, rid));
        // bytes 4..8: zero (no value on read).
        CanFrame::classical(PARAM_ID, &payload).expect("8-byte classical frame")
    }

    fn encode_write_param(&self, motor: MotorRef<'_>, rid: DamiaoRid, val: ParamValue) -> CanFrame {
        let mut payload = [0u8; 8];
        payload[0..4].copy_from_slice(&header(motor, CMD_WRITE, rid));
        match val {
            ParamValue::Float(f) => {
                payload[4..8].copy_from_slice(&(f as f32).to_le_bytes());
            }
            ParamValue::UInt(u) => {
                payload[4..8].copy_from_slice(&u.to_le_bytes());
            }
            _ => {
                // ParamValue is #[non_exhaustive]; v2 variants encode to zero
                // until this codec learns about them.
            }
        }
        CanFrame::classical(PARAM_ID, &payload).expect("8-byte classical frame")
    }

    fn encode_save_to_flash(&self, motor: MotorRef<'_>) -> CanFrame {
        // SAVE doesn't carry a register; layout is [id_lo, id_hi, 0xAA, 0, 0, 0, 0, 0]
        let id = motor.send_id as u16;
        let payload = [
            (id & 0xff) as u8,
            ((id >> 8) & 0xff) as u8,
            CMD_SAVE,
            0,
            0,
            0,
            0,
            0,
        ];
        CanFrame::classical(PARAM_ID, &payload).expect("8-byte classical frame")
    }

    fn encode_refresh(&self, motor: MotorRef<'_>) -> CanFrame {
        let id = motor.send_id as u16;
        let payload = [
            (id & 0xff) as u8,
            ((id >> 8) & 0xff) as u8,
            CMD_REFRESH,
            0,
            0,
            0,
            0,
            0,
        ];
        CanFrame::classical(PARAM_ID, &payload).expect("8-byte classical frame")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::DamiaoMotorType;
    use motor_codec::MotorTypeId;

    fn m(send: u32) -> MotorRef<'static> {
        MotorRef {
            motor_type: MotorTypeId::Damiao(DamiaoMotorType::DM4340 as u16),
            send_id: send,
            recv_id: 0x11,
            name: "j",
        }
    }

    #[test]
    fn write_param_layout() {
        let c = DamiaoCodec::new();
        let f = c.encode_write_param(m(0x01), DamiaoRid::MST_ID, ParamValue::UInt(0x11));
        assert_eq!(f.id, PARAM_ID);
        assert_eq!(f.len, 8);
        let p = f.payload();
        assert_eq!(&p[0..2], &0x01u16.to_le_bytes());
        assert_eq!(p[2], CMD_WRITE);
        assert_eq!(p[3], u8::from(DamiaoRid::MST_ID));
        assert_eq!(&p[4..8], &0x11u32.to_le_bytes());
    }

    #[test]
    fn read_param_uses_0x33() {
        let c = DamiaoCodec::new();
        let f = c.encode_read_param(m(0x01), DamiaoRid::CTRL_MODE);
        assert_eq!(f.payload()[2], CMD_READ);
    }

    #[test]
    fn save_uses_0xaa() {
        let c = DamiaoCodec::new();
        let f = c.encode_save_to_flash(m(0x01));
        assert_eq!(f.payload()[2], CMD_SAVE);
    }

    #[test]
    fn refresh_uses_0xcc() {
        let c = DamiaoCodec::new();
        let f = c.encode_refresh(m(0x01));
        assert_eq!(f.payload()[2], CMD_REFRESH);
    }
}
