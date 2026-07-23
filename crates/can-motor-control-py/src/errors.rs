//! Mapping from `can_motor_control::Error` (and friends) to Python exception types.

use can_motor_control::{CodecError, Error, TransportError};
use pyo3::exceptions::PyValueError;
use pyo3::PyErr;

/// Convert a `can_motor_control::Error` into the appropriate Python exception.
pub fn into_pyerr(err: Error) -> PyErr {
    let msg = err.to_string();
    match err {
        Error::Transport(t) => transport_to_pyerr(t),
        Error::Codec(c) => crate::CodecError::new_err(c.to_string()),
        Error::ConfigSchema(_) | Error::ConfigIo(_) => crate::ConfigError::new_err(msg),
        Error::UnknownVendor(_) | Error::UnknownBusName(_) => crate::ConfigError::new_err(msg),
        Error::OpeningOutOfRange { .. } | Error::OpeningCurrentOutOfRange { .. } => {
            PyValueError::new_err(msg)
        }
        Error::NotConnected
        | Error::TopologyLocked
        | Error::OpeningCalibrationRequired
        | Error::OpeningCalibrationFailed { .. }
        | Error::OpeningControlModeVerificationFailed { .. } => crate::LifecycleError::new_err(msg),
        _ => crate::DmError::new_err(msg),
    }
}

/// Convert a `TransportError` directly (used by the SocketCanBus binding).
pub fn transport_to_pyerr(err: TransportError) -> PyErr {
    crate::TransportError::new_err(err.to_string())
}

/// Convert a `CodecError` directly.
#[allow(dead_code)]
pub fn codec_to_pyerr(err: CodecError) -> PyErr {
    crate::CodecError::new_err(err.to_string())
}
