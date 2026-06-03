//! Python wrapper for `dm_control::CanFrame`.

use dm_control::{CanFrame, FrameFlags};
use pyo3::prelude::*;
use pyo3::types::PyBytes;

use crate::errors::transport_to_pyerr;

/// A single CAN frame: an arbitration ``id``, a payload, and flags.
///
/// Construct frames with `classical` (classical CAN, up to 8 payload
/// bytes) or `fd` (CAN-FD, up to 64 bytes). The ``id``, ``flags`` and
/// ``len`` attributes are read-only.
#[pyclass(name = "CanFrame", module = "dm_control")]
#[derive(Clone)]
pub struct PyCanFrame {
    pub(crate) inner: CanFrame,
}

#[pymethods]
impl PyCanFrame {
    /// Build a classical CAN frame.
    ///
    /// ``payload`` must be at most 8 bytes; a longer payload raises
    /// `TransportError`.
    #[staticmethod]
    fn classical(id: u32, payload: &[u8]) -> PyResult<Self> {
        let inner = CanFrame::classical(id, payload)
            .map_err(|e| transport_to_pyerr(dm_control::TransportError::FrameError(e)))?;
        Ok(Self { inner })
    }

    /// Build a CAN-FD frame.
    ///
    /// ``payload`` must be at most 64 bytes; a longer payload raises
    /// `TransportError`.
    #[staticmethod]
    fn fd(id: u32, payload: &[u8]) -> PyResult<Self> {
        let inner = CanFrame::fd(id, payload)
            .map_err(|e| transport_to_pyerr(dm_control::TransportError::FrameError(e)))?;
        Ok(Self { inner })
    }

    /// The CAN arbitration id.
    #[getter]
    fn id(&self) -> u32 {
        self.inner.id
    }

    /// The raw frame flag bits (FD, extended id, ...).
    #[getter]
    fn flags(&self) -> u8 {
        self.inner.flags.bits()
    }

    /// The payload length in bytes.
    #[getter]
    fn len(&self) -> u8 {
        self.inner.len
    }

    /// Return the payload as ``bytes``.
    fn payload<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new_bound(py, self.inner.payload())
    }

    /// ``True`` if this is a CAN-FD frame.
    fn is_fd(&self) -> bool {
        self.inner.is_fd()
    }

    /// ``True`` if this frame uses a 29-bit extended id.
    fn is_extended(&self) -> bool {
        self.inner.is_extended()
    }

    fn __repr__(&self) -> String {
        format!(
            "CanFrame(id={:#x}, len={}, fd={}, ext={})",
            self.inner.id,
            self.inner.len,
            self.inner.is_fd(),
            self.inner.flags.contains(FrameFlags::EXTENDED_ID),
        )
    }
}
