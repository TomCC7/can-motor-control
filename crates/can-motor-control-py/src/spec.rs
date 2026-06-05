//! Python wrapper for `MotorSpec`.

use can_motor_control::MotorSpec;
use pyo3::prelude::*;

use crate::codec::PyMotorType;

/// Declaration of one motor on a bus: its name, SKU, and CAN ids.
///
/// Used when building a robot in code to describe the motors of an arm or
/// gripper (see `RobotBuilder.add_arm`). The ``type`` is a
/// `can_motor_control.damiao.MotorType`; ``send_id`` is the id the host
/// sends commands on and ``recv_id`` is the id the motor replies on.
#[pyclass(name = "MotorSpec", module = "can_motor_control")]
#[derive(Clone)]
pub struct PyMotorSpec {
    pub(crate) inner: MotorSpec,
}

#[pymethods]
impl PyMotorSpec {
    /// Declare a motor named ``name`` of SKU ``type`` at the given CAN ids.
    #[new]
    fn new(name: &str, r#type: PyMotorType, send_id: u32, recv_id: u32) -> Self {
        Self {
            inner: MotorSpec::new(name, r#type, send_id, recv_id),
        }
    }

    /// The motor's name.
    #[getter]
    fn name(&self) -> String {
        self.inner.name.clone()
    }
    /// The CAN id the host sends commands to this motor on.
    #[getter]
    fn send_id(&self) -> u32 {
        self.inner.send_id
    }
    /// The CAN id this motor sends its feedback replies on.
    #[getter]
    fn recv_id(&self) -> u32 {
        self.inner.recv_id
    }
}
