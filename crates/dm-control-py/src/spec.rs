//! Python wrapper for `MotorSpec`.

use dm_control::MotorSpec;
use pyo3::prelude::*;

use crate::codec::PyMotorType;

#[pyclass(name = "MotorSpec", module = "dm_control")]
#[derive(Clone)]
pub struct PyMotorSpec {
    pub(crate) inner: MotorSpec,
}

#[pymethods]
impl PyMotorSpec {
    #[new]
    fn new(name: &str, r#type: PyMotorType, send_id: u32, recv_id: u32) -> Self {
        Self {
            inner: MotorSpec::new(name, r#type, send_id, recv_id),
        }
    }

    #[getter]
    fn name(&self) -> String {
        self.inner.name.clone()
    }
    #[getter]
    fn send_id(&self) -> u32 {
        self.inner.send_id
    }
    #[getter]
    fn recv_id(&self) -> u32 {
        self.inner.recv_id
    }
}
