//! Python wrappers for CAN transports.

use std::sync::{Arc, Mutex};

use can_motor_control::{CanBus, MockCanBus, SocketCanBus};
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

/// An in-memory CAN bus for tests and dry runs.
///
/// Drop-in replacement for `SocketCanBus` that talks to no hardware,
/// so robot code can be exercised without a real CAN interface. Pass it to
/// `RobotBuilder.add_bus`. A bus is consumed by the builder and may
/// be added to only one robot.
///
/// The default constructor is classical-CAN; use `MockCanBus.new_fd` for an
/// FD-capable mock that accepts and loops back CAN-FD frames.
#[pyclass(name = "MockCanBus", module = "can_motor_control")]
pub struct PyMockCanBus {
    pub(crate) handle: TransportHandle,
}

#[pymethods]
impl PyMockCanBus {
    /// Create a classical-CAN mock bus identified by ``name`` (used in error messages).
    #[new]
    fn new(name: &str) -> Self {
        Self {
            handle: TransportHandle::new(MockCanBus::new(name)),
        }
    }

    /// Create a CAN-FD-capable mock bus. It advertises FD capabilities and
    /// loops back FD frames, so the FD send/receive path is testable without an
    /// FD-capable interface.
    #[staticmethod]
    fn new_fd(name: &str) -> Self {
        Self {
            handle: TransportHandle::new(MockCanBus::new_fd(name)),
        }
    }
}

/// A CAN bus backed by a Linux SocketCAN interface.
///
/// Opens ``interface`` (e.g. ``"can0"``) on construction; a failure to open
/// raises `TransportError`. Pass it to
/// `RobotBuilder.add_bus`. A bus is consumed by the builder and may
/// be added to only one robot.
#[pyclass(name = "SocketCanBus", module = "can_motor_control")]
pub struct PySocketCanBus {
    pub(crate) handle: TransportHandle,
}

#[pymethods]
impl PySocketCanBus {
    /// Open SocketCAN ``interface``.
    ///
    /// Set ``fd=True`` to open the interface in CAN-FD mode (the interface
    /// itself must be FD-capable). Raises `TransportError` if the
    /// interface cannot be opened.
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
