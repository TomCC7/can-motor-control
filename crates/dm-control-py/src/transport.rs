//! Python wrappers for CAN transports.

use std::sync::{Arc, Mutex};

use dm_control::{CanBus, MockCanBus, SocketCanBus};
use pyo3::prelude::*;

use crate::errors::transport_to_pyerr;

/// Internal trait-object holder shared between Python wrappers and the
/// builder. Once the builder consumes a transport, the Arc has exactly one
/// strong reference inside the bus; the Python wrapper's Arc is dropped.
#[derive(Clone)]
pub struct TransportHandle {
    pub(crate) inner: Arc<Mutex<Option<Box<dyn CanBus>>>>,
}

impl TransportHandle {
    pub fn new<T: CanBus + 'static>(t: T) -> Self {
        Self {
            inner: Arc::new(Mutex::new(Some(Box::new(t)))),
        }
    }

    pub fn take(&self) -> Option<Box<dyn CanBus>> {
        self.inner.lock().ok()?.take()
    }
}

#[pyclass(name = "MockCanBus", module = "dm_control")]
pub struct PyMockCanBus {
    pub(crate) handle: TransportHandle,
}

#[pymethods]
impl PyMockCanBus {
    #[new]
    fn new(name: &str) -> Self {
        Self {
            handle: TransportHandle::new(MockCanBus::new(name)),
        }
    }
}

#[pyclass(name = "SocketCanBus", module = "dm_control")]
pub struct PySocketCanBus {
    pub(crate) handle: TransportHandle,
}

#[pymethods]
impl PySocketCanBus {
    #[new]
    #[pyo3(signature = (interface, *, fd=false))]
    fn new(py: Python<'_>, interface: &str, fd: bool) -> PyResult<Self> {
        // Release the GIL during the syscall.
        let bus = py.allow_threads(|| SocketCanBus::open(interface, fd));
        let bus = bus.map_err(transport_to_pyerr)?;
        Ok(Self {
            handle: TransportHandle::new(bus),
        })
    }
}
