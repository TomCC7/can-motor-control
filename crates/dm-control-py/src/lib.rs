//! Python bindings for `dm-control`. Built as `dm_control._native` by maturin.

// pyo3 0.22's `create_exception!` macro emits a `cfg(gil-refs)` check that
// cargo flags as unexpected on stable. The feature exists upstream; the
// warning is benign and goes away in pyo3 0.23.
#![allow(unexpected_cfgs)]
// pyo3 macro-expanded code triggers `useless_conversion` on its own
// `.into()` wrappers in some 0.22 versions; that's noise, not user-fixable.
#![allow(clippy::useless_conversion)]

use pyo3::create_exception;
use pyo3::exceptions::PyException;
use pyo3::prelude::*;

mod codec;
mod errors;
mod frame;
mod robot;
mod spec;
mod transport;

use codec::{PyDamiaoCodec, PyMotorType};
use frame::PyCanFrame;
use robot::{PyArm, PyGripper, PyMotor, PyMotorGroup, PyRobot, PyRobotBuilder};
use spec::PyMotorSpec;
use transport::{PyMockCanBus, PySocketCanBus};

create_exception!(_native, DmError, PyException);
create_exception!(_native, TransportError, DmError);
create_exception!(_native, CodecError, DmError);
create_exception!(_native, ConfigError, DmError);
create_exception!(_native, LifecycleError, DmError);

#[pymodule]
fn _native(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Classes.
    m.add_class::<PyCanFrame>()?;
    m.add_class::<PyMockCanBus>()?;
    m.add_class::<PySocketCanBus>()?;
    m.add_class::<PyMotorSpec>()?;
    m.add_class::<PyMotor>()?;
    m.add_class::<PyArm>()?;
    m.add_class::<PyGripper>()?;
    m.add_class::<PyMotorGroup>()?;
    m.add_class::<PyRobot>()?;
    m.add_class::<PyRobotBuilder>()?;
    m.add_class::<PyDamiaoCodec>()?;
    m.add_class::<PyMotorType>()?;

    // Exceptions.
    m.add("DmError", py.get_type_bound::<DmError>())?;
    m.add("TransportError", py.get_type_bound::<TransportError>())?;
    m.add("CodecError", py.get_type_bound::<CodecError>())?;
    m.add("ConfigError", py.get_type_bound::<ConfigError>())?;
    m.add("LifecycleError", py.get_type_bound::<LifecycleError>())?;
    Ok(())
}
