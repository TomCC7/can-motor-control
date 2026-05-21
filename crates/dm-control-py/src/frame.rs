//! Python wrapper for `dm_control::CanFrame`.

use dm_control::{CanFrame, FrameFlags};
use pyo3::prelude::*;
use pyo3::types::PyBytes;

use crate::errors::transport_to_pyerr;

#[pyclass(name = "CanFrame", module = "dm_control")]
#[derive(Clone)]
pub struct PyCanFrame {
    pub(crate) inner: CanFrame,
}

#[pymethods]
impl PyCanFrame {
    #[staticmethod]
    fn classical(id: u32, payload: &[u8]) -> PyResult<Self> {
        let inner = CanFrame::classical(id, payload).map_err(|e| {
            transport_to_pyerr(dm_control::TransportError::FrameError(e))
        })?;
        Ok(Self { inner })
    }

    #[staticmethod]
    fn fd(id: u32, payload: &[u8]) -> PyResult<Self> {
        let inner = CanFrame::fd(id, payload).map_err(|e| {
            transport_to_pyerr(dm_control::TransportError::FrameError(e))
        })?;
        Ok(Self { inner })
    }

    #[getter]
    fn id(&self) -> u32 {
        self.inner.id
    }

    #[getter]
    fn flags(&self) -> u8 {
        self.inner.flags.bits()
    }

    #[getter]
    fn len(&self) -> u8 {
        self.inner.len
    }

    fn payload<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new_bound(py, self.inner.payload())
    }

    fn is_fd(&self) -> bool {
        self.inner.is_fd()
    }

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
