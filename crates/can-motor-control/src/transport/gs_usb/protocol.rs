//! Narrow classical-CAN subset of the gs_usb wire protocol.

use motor_codec::{CanFrame, FrameFlags};
use thiserror::Error;

pub(crate) const HOST_FRAME_LEN: usize = 20;
pub(crate) const RX_ECHO_ID: u32 = u32::MAX;

const HOST_BYTE_ORDER_LE: u32 = 0x0000_beef;
const CAN_EFF_FLAG: u32 = 0x8000_0000;
const CAN_RTR_FLAG: u32 = 0x4000_0000;
const CAN_ERR_FLAG: u32 = 0x2000_0000;
const CAN_SFF_MASK: u32 = 0x0000_07ff;
const CAN_EFF_MASK: u32 = 0x1fff_ffff;
const GS_CAN_FLAG_FD: u8 = 1 << 1;
const GS_CAN_FLAG_BRS: u8 = 1 << 2;
const GS_CAN_FLAG_ESI: u8 = 1 << 3;

#[derive(Clone, Debug, PartialEq, Eq, Error)]
pub(crate) enum ProtocolError {
    #[error("gs_usb structure is too short: expected {expected} bytes, got {actual}")]
    ShortBuffer { expected: usize, actual: usize },
    #[error("gs_usb host frame DLC {0} exceeds classical CAN capacity")]
    InvalidDlc(u8),
    #[error("gs_usb channel {0} is unsupported; only channel 0 is supported")]
    UnsupportedChannel(u8),
    #[error("gs_usb CAN-FD flags 0x{0:02x} are unsupported")]
    CanFdFlags(u8),
    #[error("gs_usb flags 0x{0:02x} are unsupported")]
    UnsupportedFlags(u8),
    #[error("gs_usb reserved byte must be zero, got 0x{0:02x}")]
    NonzeroReserved(u8),
    #[error("gs_usb CAN error frames are unsupported")]
    ErrorFrame,
    #[error("standard CAN identifier 0x{0:x} exceeds 11 bits")]
    InvalidStandardId(u32),
    #[error("extended CAN identifier 0x{0:x} exceeds 29 bits")]
    InvalidExtendedId(u32),
    #[error("CAN-FD frames are unsupported by the classical gs_usb transport")]
    CanFdFrame,
    #[error("unknown, duplicate, or out-of-range gs_usb echo id {0}")]
    InvalidEchoId(u32),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct DeviceConfig {
    pub(crate) channel_count: u8,
    pub(crate) software_version: u32,
    pub(crate) hardware_version: u32,
}

impl DeviceConfig {
    pub(crate) fn decode(bytes: &[u8]) -> Result<Self, ProtocolError> {
        require_len(bytes, 12)?;
        Ok(Self {
            channel_count: bytes[3].saturating_add(1),
            software_version: le_u32(bytes, 4),
            hardware_version: le_u32(bytes, 8),
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct BitTimingConstraints {
    pub(crate) features: u32,
    pub(crate) clock_hz: u32,
    pub(crate) tseg1_min: u32,
    pub(crate) tseg1_max: u32,
    pub(crate) tseg2_min: u32,
    pub(crate) tseg2_max: u32,
    pub(crate) sjw_max: u32,
    pub(crate) brp_min: u32,
    pub(crate) brp_max: u32,
    pub(crate) brp_increment: u32,
}

impl BitTimingConstraints {
    pub(crate) fn decode(bytes: &[u8]) -> Result<Self, ProtocolError> {
        require_len(bytes, 40)?;
        Ok(Self {
            features: le_u32(bytes, 0),
            clock_hz: le_u32(bytes, 4),
            tseg1_min: le_u32(bytes, 8),
            tseg1_max: le_u32(bytes, 12),
            tseg2_min: le_u32(bytes, 16),
            tseg2_max: le_u32(bytes, 20),
            sjw_max: le_u32(bytes, 24),
            brp_min: le_u32(bytes, 28),
            brp_max: le_u32(bytes, 32),
            brp_increment: le_u32(bytes, 36),
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct NominalBitTiming {
    pub(crate) prop_seg: u32,
    pub(crate) phase_seg1: u32,
    pub(crate) phase_seg2: u32,
    pub(crate) sjw: u32,
    pub(crate) brp: u32,
}

impl NominalBitTiming {
    pub(crate) fn encode(self) -> [u8; 20] {
        let mut bytes = [0_u8; 20];
        put_u32(&mut bytes, 0, self.prop_seg);
        put_u32(&mut bytes, 4, self.phase_seg1);
        put_u32(&mut bytes, 8, self.phase_seg2);
        put_u32(&mut bytes, 12, self.sjw);
        put_u32(&mut bytes, 16, self.brp);
        bytes
    }

    pub(crate) fn tseg1(self) -> u32 {
        self.prop_seg + self.phase_seg1
    }
}

pub(crate) fn encode_host_format() -> [u8; 4] {
    HOST_BYTE_ORDER_LE.to_le_bytes()
}

pub(crate) fn encode_mode(mode: u32, flags: u32) -> [u8; 8] {
    let mut bytes = [0_u8; 8];
    put_u32(&mut bytes, 0, mode);
    put_u32(&mut bytes, 4, flags);
    bytes
}

pub(crate) fn encode_host_frame(
    echo_id: u32,
    frame: &CanFrame,
) -> Result<[u8; HOST_FRAME_LEN], ProtocolError> {
    if frame.is_fd() {
        return Err(ProtocolError::CanFdFrame);
    }
    if frame.len > 8 {
        return Err(ProtocolError::InvalidDlc(frame.len));
    }
    let mut can_id = frame.id;
    if frame.is_extended() {
        if can_id > CAN_EFF_MASK {
            return Err(ProtocolError::InvalidExtendedId(can_id));
        }
        can_id |= CAN_EFF_FLAG;
    } else if can_id > CAN_SFF_MASK {
        return Err(ProtocolError::InvalidStandardId(can_id));
    }
    if frame.flags.contains(FrameFlags::REMOTE_REQUEST) {
        can_id |= CAN_RTR_FLAG;
    }

    let mut bytes = [0_u8; HOST_FRAME_LEN];
    put_u32(&mut bytes, 0, echo_id);
    put_u32(&mut bytes, 4, can_id);
    bytes[8] = frame.len;
    bytes[9] = 0;
    bytes[10] = 0;
    bytes[11] = 0;
    bytes[12..12 + frame.len as usize].copy_from_slice(frame.payload());
    Ok(bytes)
}

pub(crate) fn decode_host_frame(bytes: &[u8]) -> Result<(u32, CanFrame), ProtocolError> {
    require_len(bytes, HOST_FRAME_LEN)?;
    let echo_id = le_u32(bytes, 0);
    let raw_id = le_u32(bytes, 4);
    let dlc = bytes[8];
    let channel = bytes[9];
    let flags = bytes[10];
    let reserved = bytes[11];

    if dlc > 8 {
        return Err(ProtocolError::InvalidDlc(dlc));
    }
    if channel != 0 {
        return Err(ProtocolError::UnsupportedChannel(channel));
    }
    let fd_flags = flags & (GS_CAN_FLAG_FD | GS_CAN_FLAG_BRS | GS_CAN_FLAG_ESI);
    if fd_flags != 0 {
        return Err(ProtocolError::CanFdFlags(fd_flags));
    }
    if flags != 0 {
        return Err(ProtocolError::UnsupportedFlags(flags));
    }
    if reserved != 0 {
        return Err(ProtocolError::NonzeroReserved(reserved));
    }
    if raw_id & CAN_ERR_FLAG != 0 {
        return Err(ProtocolError::ErrorFrame);
    }

    let extended = raw_id & CAN_EFF_FLAG != 0;
    if !extended && raw_id & CAN_EFF_MASK & !CAN_SFF_MASK != 0 {
        return Err(ProtocolError::InvalidStandardId(raw_id & CAN_EFF_MASK));
    }
    let id = raw_id & if extended { CAN_EFF_MASK } else { CAN_SFF_MASK };
    let mut frame_flags = FrameFlags::empty();
    if extended {
        frame_flags |= FrameFlags::EXTENDED_ID;
    }
    if raw_id & CAN_RTR_FLAG != 0 {
        frame_flags |= FrameFlags::REMOTE_REQUEST;
    }
    let frame = CanFrame::from_parts(id, frame_flags, &bytes[12..12 + dlc as usize])
        .expect("validated classical gs_usb frame fields");
    Ok((echo_id, frame))
}

fn require_len(bytes: &[u8], expected: usize) -> Result<(), ProtocolError> {
    if bytes.len() < expected {
        Err(ProtocolError::ShortBuffer {
            expected,
            actual: bytes.len(),
        })
    } else {
        Ok(())
    }
}

fn le_u32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(
        bytes[offset..offset + 4]
            .try_into()
            .expect("checked length"),
    )
}

fn put_u32(bytes: &mut [u8], offset: usize, value: u32) {
    bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_structure_layouts_are_little_endian_and_packed() {
        assert_eq!(encode_host_format(), [0xef, 0xbe, 0, 0]);
        assert_eq!(encode_mode(1, 0), [1, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(
            NominalBitTiming {
                prop_seg: 1,
                phase_seg1: 13,
                phase_seg2: 2,
                sjw: 1,
                brp: 10,
            }
            .encode(),
            [1, 0, 0, 0, 13, 0, 0, 0, 2, 0, 0, 0, 1, 0, 0, 0, 10, 0, 0, 0]
        );
    }

    #[test]
    fn device_and_capability_layouts_decode() {
        let mut device = [0_u8; 12];
        device[3] = 0;
        device[4..8].copy_from_slice(&2_u32.to_le_bytes());
        device[8..12].copy_from_slice(&7_u32.to_le_bytes());
        assert_eq!(
            DeviceConfig::decode(&device).unwrap(),
            DeviceConfig {
                channel_count: 1,
                software_version: 2,
                hardware_version: 7,
            }
        );

        let words = [0x101_u32, 170_000_000, 2, 256, 1, 128, 128, 1, 512, 1];
        let mut capability = [0_u8; 40];
        for (chunk, word) in capability.chunks_exact_mut(4).zip(words) {
            chunk.copy_from_slice(&word.to_le_bytes());
        }
        let decoded = BitTimingConstraints::decode(&capability).unwrap();
        assert_eq!(decoded.clock_hz, 170_000_000);
        assert_eq!(decoded.brp_max, 512);
        assert_eq!(decoded.features, 0x101);
    }

    #[test]
    fn classical_standard_and_extended_frames_round_trip() {
        let standard = CanFrame::classical(0x321, &[1, 2, 3]).unwrap();
        let encoded = encode_host_frame(9, &standard).unwrap();
        assert_eq!(&encoded[0..4], &9_u32.to_le_bytes());
        assert_eq!(encoded[8..12], [3, 0, 0, 0]);
        assert_eq!(&encoded[12..], &[1, 2, 3, 0, 0, 0, 0, 0]);
        assert_eq!(decode_host_frame(&encoded).unwrap(), (9, standard));

        let extended = CanFrame::classical_extended(0x1abc_def0, &[8; 8]).unwrap();
        let encoded = encode_host_frame(RX_ECHO_ID, &extended).unwrap();
        assert_eq!(le_u32(&encoded, 4), CAN_EFF_FLAG | 0x1abc_def0);
        assert_eq!(decode_host_frame(&encoded).unwrap(), (RX_ECHO_ID, extended));
    }

    #[test]
    fn malformed_frames_are_rejected() {
        assert!(matches!(
            decode_host_frame(&[0; 19]),
            Err(ProtocolError::ShortBuffer { .. })
        ));
        for (offset, value, expected) in [
            (8, 9, "DLC"),
            (9, 1, "channel"),
            (10, GS_CAN_FLAG_FD, "CAN-FD"),
            (11, 1, "reserved"),
        ] {
            let mut bytes = [0_u8; HOST_FRAME_LEN];
            bytes[offset] = value;
            assert!(decode_host_frame(&bytes)
                .unwrap_err()
                .to_string()
                .contains(expected));
        }
        let mut error_frame = [0_u8; HOST_FRAME_LEN];
        put_u32(&mut error_frame, 4, CAN_ERR_FLAG);
        assert_eq!(
            decode_host_frame(&error_frame),
            Err(ProtocolError::ErrorFrame)
        );
        let mut oversized_standard_id = [0_u8; HOST_FRAME_LEN];
        put_u32(&mut oversized_standard_id, 4, 0x800);
        assert_eq!(
            decode_host_frame(&oversized_standard_id),
            Err(ProtocolError::InvalidStandardId(0x800))
        );
    }

    #[test]
    fn incompatible_outbound_frames_are_rejected() {
        let bad_standard = CanFrame::classical(0x800, &[]).unwrap();
        assert_eq!(
            encode_host_frame(0, &bad_standard),
            Err(ProtocolError::InvalidStandardId(0x800))
        );
        let fd = CanFrame::fd(1, &[0; 8]).unwrap();
        assert_eq!(encode_host_frame(0, &fd), Err(ProtocolError::CanFdFrame));
    }
}
