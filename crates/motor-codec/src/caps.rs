//! Bus capability descriptor.

/// Runtime feature set of a CAN transport.
///
/// Constructed via the [`BusCapabilities::classical`] / [`BusCapabilities::fd`]
/// constructors, which enforce the invariant `supports_fd == (max_payload_len == 64)`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct BusCapabilities {
    /// True if the bus can transmit and receive CAN-FD frames.
    pub supports_fd: bool,
    /// Maximum payload length: 8 for classical, 64 for FD.
    pub max_payload_len: u8,
}

impl BusCapabilities {
    /// Classical CAN: `supports_fd = false`, `max_payload_len = 8`.
    pub const fn classical() -> Self {
        Self {
            supports_fd: false,
            max_payload_len: 8,
        }
    }

    /// CAN-FD: `supports_fd = true`, `max_payload_len = 64`.
    pub const fn fd() -> Self {
        Self {
            supports_fd: true,
            max_payload_len: 64,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classical_constants() {
        let c = BusCapabilities::classical();
        assert!(!c.supports_fd);
        assert_eq!(c.max_payload_len, 8);
    }

    #[test]
    fn fd_constants() {
        let c = BusCapabilities::fd();
        assert!(c.supports_fd);
        assert_eq!(c.max_payload_len, 64);
    }
}
