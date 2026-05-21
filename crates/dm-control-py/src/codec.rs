//! Python wrappers for the Damiao codec (and the vendor's motor-type enum).

use std::sync::{Arc, Mutex};

use dm_codec::{DamiaoCodec, DamiaoMotorType};
use motor_codec::{MotorCodec, MotorTypeId};
use pyo3::prelude::*;

/// Same shape as `TransportHandle`: a Box-trait-object that the builder
/// `take()`s once.
#[derive(Clone)]
pub struct CodecHandle {
    pub(crate) inner: Arc<Mutex<Option<Box<dyn MotorCodec>>>>,
}

impl CodecHandle {
    pub fn new<C: MotorCodec + 'static>(c: C) -> Self {
        Self {
            inner: Arc::new(Mutex::new(Some(Box::new(c)))),
        }
    }
    pub fn take(&self) -> Option<Box<dyn MotorCodec>> {
        self.inner.lock().ok()?.take()
    }
}

#[pyclass(name = "DamiaoCodec", module = "dm_control.damiao")]
pub struct PyDamiaoCodec {
    pub(crate) handle: CodecHandle,
}

#[pymethods]
impl PyDamiaoCodec {
    #[new]
    fn new() -> Self {
        Self {
            handle: CodecHandle::new(DamiaoCodec::new()),
        }
    }
}

/// Damiao motor SKU enum exposed as an IntEnum-compatible Python class.
#[pyclass(name = "MotorType", module = "dm_control.damiao", eq, eq_int, frozen)]
#[derive(Copy, Clone, PartialEq, Eq, Hash)]
#[allow(non_camel_case_types)]
pub enum PyMotorType {
    DM3507 = 0,
    DM4310 = 1,
    DM4310_48V = 2,
    DM4340 = 3,
    DM4340_48V = 4,
    DM6006 = 5,
    DM8006 = 6,
    DM8009 = 7,
    DM10010L = 8,
    DM10010 = 9,
    DMH3510 = 10,
    DMH6215 = 11,
    DMG6220 = 12,
}

impl From<PyMotorType> for MotorTypeId {
    fn from(t: PyMotorType) -> Self {
        let dm = match t {
            PyMotorType::DM3507 => DamiaoMotorType::DM3507,
            PyMotorType::DM4310 => DamiaoMotorType::DM4310,
            PyMotorType::DM4310_48V => DamiaoMotorType::DM4310_48V,
            PyMotorType::DM4340 => DamiaoMotorType::DM4340,
            PyMotorType::DM4340_48V => DamiaoMotorType::DM4340_48V,
            PyMotorType::DM6006 => DamiaoMotorType::DM6006,
            PyMotorType::DM8006 => DamiaoMotorType::DM8006,
            PyMotorType::DM8009 => DamiaoMotorType::DM8009,
            PyMotorType::DM10010L => DamiaoMotorType::DM10010L,
            PyMotorType::DM10010 => DamiaoMotorType::DM10010,
            PyMotorType::DMH3510 => DamiaoMotorType::DMH3510,
            PyMotorType::DMH6215 => DamiaoMotorType::DMH6215,
            PyMotorType::DMG6220 => DamiaoMotorType::DMG6220,
        };
        dm.into()
    }
}
