//! Unified CAN frame representation covering both classical CAN and CAN-FD.

use bitflags::bitflags;
use thiserror::Error;

bitflags! {
    /// Metadata bits attached to every [`CanFrame`].
    ///
    /// A frame's classical-vs-FD nature is determined entirely by the
    /// [`FrameFlags::FD_FORMAT`] bit — there is no separate `CanFdFrame` type.
    #[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash)]
    pub struct FrameFlags: u8 {
        /// 29-bit extended CAN identifier (vs 11-bit base ID).
        const EXTENDED_ID = 0b0000_0001;
        /// CAN-FD format (FDF).
        const FD_FORMAT = 0b0000_0010;
        /// CAN-FD data-phase uses the higher bit rate (BRS). FD only.
        const BIT_RATE_SWITCH = 0b0000_0100;
        /// CAN-FD error-state indicator (ESI). FD only.
        const ERROR_STATE = 0b0000_1000;
        /// Classical-CAN remote-transmission request (RTR). Mutually exclusive
        /// with [`FrameFlags::FD_FORMAT`].
        const REMOTE_REQUEST = 0b0001_0000;
    }
}

/// Errors that can arise constructing a [`CanFrame`].
#[derive(Debug, Clone, PartialEq, Eq, Error)]
#[non_exhaustive]
pub enum FrameError {
    /// Classical CAN payloads cannot exceed 8 bytes.
    #[error("classical CAN payload too long: got {got} bytes, max is 8")]
    PayloadTooLong {
        /// Length the caller attempted to pack.
        got: usize,
    },

    /// CAN-FD payload length must be one of the valid DLC values
    /// (0..=8, 12, 16, 20, 24, 32, 48, 64).
    #[error("invalid CAN-FD payload length: got {got} bytes")]
    InvalidFdLength {
        /// Length the caller attempted to pack.
        got: usize,
    },

    /// A flag combination forbidden by the CAN specification was supplied.
    #[error("incompatible frame flags: {reason}")]
    IncompatibleFlags {
        /// Human-readable description of the conflict.
        reason: &'static str,
    },
}

/// Unified CAN frame: covers both classical CAN (8-byte payload) and CAN-FD
/// (up to 64-byte payload).
///
/// The inline `[u8; 64]` payload buffer is sized for CAN-FD's maximum DLC;
/// classical frames use only the first 8 bytes and pay ~56 bytes of latent
/// memory in exchange for full upper-layer agnosticism between classical and
/// FD. See `openspec/changes/walking-skeleton-single-arm/design.md` Decision 14.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct CanFrame {
    /// CAN identifier (11-bit base or 29-bit extended; see [`FrameFlags::EXTENDED_ID`]).
    pub id: u32,
    /// Metadata bits (FD/BRS/ESI/EXTENDED/RTR).
    pub flags: FrameFlags,
    /// Payload length in bytes. `len <= 8` for classical, `len <= 64` for FD.
    pub len: u8,
    data: [u8; 64],
}

impl CanFrame {
    /// Construct a classical CAN frame with an 11-bit base identifier.
    pub fn classical(id: u32, payload: &[u8]) -> Result<Self, FrameError> {
        if payload.len() > 8 {
            return Err(FrameError::PayloadTooLong { got: payload.len() });
        }
        let mut data = [0u8; 64];
        data[..payload.len()].copy_from_slice(payload);
        Ok(Self {
            id,
            flags: FrameFlags::empty(),
            len: payload.len() as u8,
            data,
        })
    }

    /// Construct a classical CAN frame with a 29-bit extended identifier.
    pub fn classical_extended(id: u32, payload: &[u8]) -> Result<Self, FrameError> {
        let mut f = Self::classical(id, payload)?;
        f.flags |= FrameFlags::EXTENDED_ID;
        Ok(f)
    }

    /// Construct a CAN-FD frame. Length must be a valid FD DLC; BRS is set by default.
    pub fn fd(id: u32, payload: &[u8]) -> Result<Self, FrameError> {
        if !is_valid_fd_dlc(payload.len()) {
            return Err(FrameError::InvalidFdLength { got: payload.len() });
        }
        let mut data = [0u8; 64];
        data[..payload.len()].copy_from_slice(payload);
        Ok(Self {
            id,
            flags: FrameFlags::FD_FORMAT | FrameFlags::BIT_RATE_SWITCH,
            len: payload.len() as u8,
            data,
        })
    }

    /// Construct a CAN-FD frame with a 29-bit extended identifier.
    pub fn fd_extended(id: u32, payload: &[u8]) -> Result<Self, FrameError> {
        let mut f = Self::fd(id, payload)?;
        f.flags |= FrameFlags::EXTENDED_ID;
        Ok(f)
    }

    /// Construct a frame from raw fields. Performs the flag-compatibility
    /// validation that [`CanFrame::classical`] / [`CanFrame::fd`] enforce
    /// automatically.
    pub fn from_parts(id: u32, flags: FrameFlags, payload: &[u8]) -> Result<Self, FrameError> {
        check_flag_compat(flags)?;
        if flags.contains(FrameFlags::FD_FORMAT) {
            if !is_valid_fd_dlc(payload.len()) {
                return Err(FrameError::InvalidFdLength { got: payload.len() });
            }
        } else if payload.len() > 8 {
            return Err(FrameError::PayloadTooLong { got: payload.len() });
        }
        let mut data = [0u8; 64];
        data[..payload.len()].copy_from_slice(payload);
        Ok(Self {
            id,
            flags,
            len: payload.len() as u8,
            data,
        })
    }

    /// Return the active payload bytes (`&self.data[..self.len]`).
    pub fn payload(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    /// True iff [`FrameFlags::FD_FORMAT`] is set.
    pub fn is_fd(&self) -> bool {
        self.flags.contains(FrameFlags::FD_FORMAT)
    }

    /// True iff [`FrameFlags::EXTENDED_ID`] is set.
    pub fn is_extended(&self) -> bool {
        self.flags.contains(FrameFlags::EXTENDED_ID)
    }
}

/// True iff `len` is a valid CAN-FD payload length (0..=8, 12, 16, 20, 24, 32, 48, 64).
pub fn is_valid_fd_dlc(len: usize) -> bool {
    matches!(len, 0..=8 | 12 | 16 | 20 | 24 | 32 | 48 | 64)
}

fn check_flag_compat(flags: FrameFlags) -> Result<(), FrameError> {
    if flags.contains(FrameFlags::FD_FORMAT) && flags.contains(FrameFlags::REMOTE_REQUEST) {
        return Err(FrameError::IncompatibleFlags {
            reason: "FD_FORMAT and REMOTE_REQUEST are mutually exclusive",
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classical_8byte_ok() {
        let f = CanFrame::classical(0x101, &[0xFF; 8]).unwrap();
        assert_eq!(f.id, 0x101);
        assert_eq!(f.len, 8);
        assert!(!f.is_fd());
        assert!(!f.is_extended());
        assert_eq!(f.payload(), &[0xFF; 8]);
    }

    #[test]
    fn classical_9byte_rejected() {
        let err = CanFrame::classical(0x101, &[0; 9]).unwrap_err();
        assert_eq!(err, FrameError::PayloadTooLong { got: 9 });
    }

    #[test]
    fn fd_16byte_ok() {
        let f = CanFrame::fd(0x101, &[0xAA; 16]).unwrap();
        assert_eq!(f.id, 0x101);
        assert_eq!(f.len, 16);
        assert!(f.is_fd());
        assert!(f.flags.contains(FrameFlags::BIT_RATE_SWITCH));
        assert_eq!(f.payload(), &[0xAA; 16]);
    }

    #[test]
    fn fd_invalid_dlc_rejected() {
        let err = CanFrame::fd(0x101, &[0; 9]).unwrap_err();
        assert_eq!(err, FrameError::InvalidFdLength { got: 9 });
    }

    #[test]
    fn fd_dlc_table() {
        for n in [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64] {
            let payload = alloc::vec![0u8; n];
            assert!(CanFrame::fd(0x100, &payload).is_ok(), "len={n}");
        }
        for n in [9, 10, 11, 13, 17, 33, 65] {
            let payload = alloc::vec![0u8; n];
            assert!(CanFrame::fd(0x100, &payload).is_err(), "len={n}");
        }
    }

    #[test]
    fn is_fd_reflects_flag() {
        let f = CanFrame::classical(0x100, &[]).unwrap();
        assert_eq!(f.is_fd(), f.flags.contains(FrameFlags::FD_FORMAT));
        let f = CanFrame::fd(0x100, &[]).unwrap();
        assert_eq!(f.is_fd(), f.flags.contains(FrameFlags::FD_FORMAT));
    }

    #[test]
    fn fd_and_rtr_incompatible() {
        let bad = FrameFlags::FD_FORMAT | FrameFlags::REMOTE_REQUEST;
        let err = CanFrame::from_parts(0x100, bad, &[]).unwrap_err();
        matches!(err, FrameError::IncompatibleFlags { .. });
    }

    #[test]
    fn flag_bits_compose() {
        let f = FrameFlags::FD_FORMAT | FrameFlags::BIT_RATE_SWITCH;
        assert!(f.contains(FrameFlags::FD_FORMAT));
        assert!(f.contains(FrameFlags::BIT_RATE_SWITCH));
        assert!(!f.contains(FrameFlags::REMOTE_REQUEST));
    }

    #[test]
    fn error_display_is_informative() {
        let s = alloc::format!("{}", FrameError::PayloadTooLong { got: 9 });
        assert!(s.contains("classical"));
        assert!(s.contains("9"));
        let s = alloc::format!("{}", FrameError::InvalidFdLength { got: 13 });
        assert!(s.contains("CAN-FD"));
    }
}
