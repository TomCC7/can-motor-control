//! Python wrappers for Robot, RobotBuilder, and the group kinds.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use dm_codec::{parse_motor_type as dm_parse_type, DamiaoCodec, VENDOR_NAME as DAMIAO_VENDOR};
use dm_control::{
    Arm, CanBus, CodecRegistry, Gripper, GroupKind, MitCmd, PosForceCmd, PosVelCmd, Robot,
    RobotBuilder, VelCmd,
};
use motor_codec::MotorCodec;
use numpy::ndarray::Axis;
use numpy::{PyArray1, PyReadonlyArray1, PyReadonlyArray2};
use pyo3::exceptions::{PyKeyError, PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyAnyMethods, PyList};

use crate::codec::PyDamiaoCodec;
use crate::errors::into_pyerr;
use crate::spec::PyMotorSpec;
use crate::transport::{PyMockCanBus, PySocketCanBus};

#[pyclass(name = "Motor", module = "dm_control")]
pub struct PyMotor {
    // Snapshot only — we resync from the robot's group on each property access
    // by going through PyRobot.
    pub(crate) robot: Py<PyRobot>,
    pub(crate) group_name: String,
    pub(crate) motor_index: usize,
}

impl PyMotor {
    fn snapshot(&self, py: Python<'_>) -> PyResult<MotorSnapshot> {
        let robot = self.robot.bind(py).borrow();
        let inner = robot
            .inner
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("robot mutex poisoned"))?;
        let g = inner
            .group(&self.group_name)
            .ok_or_else(|| PyKeyError::new_err(self.group_name.clone()))?;
        let motors = g.inner().motors();
        let m = motors
            .get(self.motor_index)
            .ok_or_else(|| pyo3::exceptions::PyIndexError::new_err(self.motor_index))?;
        Ok(MotorSnapshot {
            name: m.name().to_string(),
            send_id: m.send_id(),
            recv_id: m.recv_id(),
            position: m.position(),
            velocity: m.velocity(),
            torque: m.torque(),
            t_mos: m.temperature_mos(),
            t_rotor: m.temperature_rotor(),
            is_enabled: m.is_enabled(),
            fault: m.fault().map(|f| f.0),
        })
    }
}

#[pymethods]
impl PyMotor {
    #[getter]
    fn name(&self, py: Python<'_>) -> PyResult<String> {
        Ok(self.snapshot(py)?.name)
    }
    #[getter]
    fn send_id(&self, py: Python<'_>) -> PyResult<u32> {
        Ok(self.snapshot(py)?.send_id)
    }
    #[getter]
    fn recv_id(&self, py: Python<'_>) -> PyResult<u32> {
        Ok(self.snapshot(py)?.recv_id)
    }
    #[getter]
    fn position(&self, py: Python<'_>) -> PyResult<f64> {
        Ok(self.snapshot(py)?.position)
    }
    #[getter]
    fn velocity(&self, py: Python<'_>) -> PyResult<f64> {
        Ok(self.snapshot(py)?.velocity)
    }
    #[getter]
    fn torque(&self, py: Python<'_>) -> PyResult<f64> {
        Ok(self.snapshot(py)?.torque)
    }
    #[getter]
    fn temperature_mos(&self, py: Python<'_>) -> PyResult<i16> {
        Ok(self.snapshot(py)?.t_mos)
    }
    #[getter]
    fn temperature_rotor(&self, py: Python<'_>) -> PyResult<i16> {
        Ok(self.snapshot(py)?.t_rotor)
    }
    #[getter]
    fn is_enabled(&self, py: Python<'_>) -> PyResult<bool> {
        Ok(self.snapshot(py)?.is_enabled)
    }
    #[getter]
    fn fault(&self, py: Python<'_>) -> PyResult<Option<u16>> {
        Ok(self.snapshot(py)?.fault)
    }
}

struct MotorSnapshot {
    name: String,
    send_id: u32,
    recv_id: u32,
    position: f64,
    velocity: f64,
    torque: f64,
    t_mos: i16,
    t_rotor: i16,
    is_enabled: bool,
    fault: Option<u16>,
}

/// Apply a closure to a group's `Arm` (or return KeyError / TypeError).
///
/// Note: we do not release the GIL inside this helper because the closure
/// captures `&mut Arm` which is not `Send`. The Robot-level methods
/// (`tick`/`enable`/`disable`) release the GIL at that boundary instead.
fn with_arm<R>(
    robot: &PyRobot,
    group_name: &str,
    f: impl FnOnce(&mut Arm) -> Result<R, dm_control::Error>,
) -> PyResult<R> {
    let mut inner = robot
        .inner
        .lock()
        .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("robot mutex poisoned"))?;
    let g = inner
        .group_mut(group_name)
        .ok_or_else(|| PyKeyError::new_err(group_name.to_string()))?;
    let arm = g
        .as_arm_mut()
        .ok_or_else(|| PyTypeError::new_err(format!("group '{group_name}' is not an arm")))?;
    f(arm).map_err(into_pyerr)
}

fn with_gripper<R>(
    robot: &PyRobot,
    group_name: &str,
    f: impl FnOnce(&mut Gripper) -> Result<R, dm_control::Error>,
) -> PyResult<R> {
    let mut inner = robot
        .inner
        .lock()
        .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("robot mutex poisoned"))?;
    let g = inner
        .group_mut(group_name)
        .ok_or_else(|| PyKeyError::new_err(group_name.to_string()))?;
    let gr = g
        .as_gripper_mut()
        .ok_or_else(|| PyTypeError::new_err(format!("group '{group_name}' is not a gripper")))?;
    f(gr).map_err(into_pyerr)
}

#[pyclass(name = "Arm", module = "dm_control")]
pub struct PyArm {
    pub(crate) robot: Py<PyRobot>,
    pub(crate) name: String,
}

#[pymethods]
impl PyArm {
    fn __len__(&self, py: Python<'_>) -> PyResult<usize> {
        let r = self.robot.bind(py).borrow();
        let inner = r
            .inner
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
        let g = inner
            .group(&self.name)
            .ok_or_else(|| PyKeyError::new_err(self.name.clone()))?;
        Ok(g.inner().len())
    }

    fn __getitem__(&self, py: Python<'_>, motor_name: &str) -> PyResult<PyMotor> {
        let r = self.robot.bind(py).borrow();
        let inner = r
            .inner
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
        let g = inner
            .group(&self.name)
            .ok_or_else(|| PyKeyError::new_err(self.name.clone()))?;
        let idx = g
            .inner()
            .motors()
            .iter()
            .position(|m| m.name() == motor_name)
            .ok_or_else(|| PyKeyError::new_err(motor_name.to_string()))?;
        Ok(PyMotor {
            robot: self.robot.clone_ref(py),
            group_name: self.name.clone(),
            motor_index: idx,
        })
    }

    fn positions<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let r = self.robot.bind(py).borrow();
        let inner = r
            .inner
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
        let g = inner
            .group(&self.name)
            .ok_or_else(|| PyKeyError::new_err(self.name.clone()))?;
        let v: Vec<f64> = g.inner().motors().iter().map(|m| m.position()).collect();
        Ok(PyArray1::from_vec_bound(py, v))
    }
    fn velocities<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let r = self.robot.bind(py).borrow();
        let inner = r
            .inner
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
        let g = inner
            .group(&self.name)
            .ok_or_else(|| PyKeyError::new_err(self.name.clone()))?;
        let v: Vec<f64> = g.inner().motors().iter().map(|m| m.velocity()).collect();
        Ok(PyArray1::from_vec_bound(py, v))
    }
    fn torques<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray1<f64>>> {
        let r = self.robot.bind(py).borrow();
        let inner = r
            .inner
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
        let g = inner
            .group(&self.name)
            .ok_or_else(|| PyKeyError::new_err(self.name.clone()))?;
        let v: Vec<f64> = g.inner().motors().iter().map(|m| m.torque()).collect();
        Ok(PyArray1::from_vec_bound(py, v))
    }

    fn mit_control(&self, py: Python<'_>, cmds: PyReadonlyArray2<'_, f64>) -> PyResult<()> {
        let arr = cmds.as_array();
        let expected_n = {
            let r = self.robot.bind(py).borrow();
            let inner = r
                .inner
                .lock()
                .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
            inner
                .group(&self.name)
                .ok_or_else(|| PyKeyError::new_err(self.name.clone()))?
                .inner()
                .len()
        };
        if arr.shape() != [expected_n, 5] {
            return Err(PyValueError::new_err(format!(
                "expected shape ({expected_n}, 5) float64, got ({}, {})",
                arr.shape()[0],
                arr.shape()[1]
            )));
        }
        let cmds: Vec<MitCmd> = arr
            .axis_iter(Axis(0))
            .map(|row| MitCmd {
                kp: row[0],
                kd: row[1],
                q: row[2],
                dq: row[3],
                tau: row[4],
            })
            .collect();
        let r = self.robot.bind(py).borrow();
        with_arm(&r, &self.name, |arm| arm.mit_control(&cmds))
    }

    fn pos_vel_control(&self, py: Python<'_>, cmds: PyReadonlyArray2<'_, f64>) -> PyResult<()> {
        let arr = cmds.as_array();
        let expected_n = {
            let r = self.robot.bind(py).borrow();
            let inner = r
                .inner
                .lock()
                .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
            inner
                .group(&self.name)
                .ok_or_else(|| PyKeyError::new_err(self.name.clone()))?
                .inner()
                .len()
        };
        if arr.shape() != [expected_n, 2] {
            return Err(PyValueError::new_err(format!(
                "expected shape ({expected_n}, 2) float64, got ({}, {})",
                arr.shape()[0],
                arr.shape()[1]
            )));
        }
        let cmds: Vec<PosVelCmd> = arr
            .axis_iter(Axis(0))
            .map(|r| PosVelCmd { q: r[0], dq: r[1] })
            .collect();
        let r = self.robot.bind(py).borrow();
        with_arm(&r, &self.name, |arm| arm.pos_vel_control(&cmds))
    }

    fn vel_control(&self, py: Python<'_>, cmds: PyReadonlyArray1<'_, f64>) -> PyResult<()> {
        let arr = cmds.as_array();
        let expected_n = {
            let r = self.robot.bind(py).borrow();
            let inner = r
                .inner
                .lock()
                .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
            inner
                .group(&self.name)
                .ok_or_else(|| PyKeyError::new_err(self.name.clone()))?
                .inner()
                .len()
        };
        if arr.shape() != [expected_n] {
            return Err(PyValueError::new_err(format!(
                "expected shape ({expected_n},) float64, got ({},)",
                arr.shape()[0]
            )));
        }
        let cmds: Vec<VelCmd> = arr.iter().map(|&dq| VelCmd { dq }).collect();
        let r = self.robot.bind(py).borrow();
        with_arm(&r, &self.name, |arm| arm.vel_control(&cmds))
    }

    fn pos_force_control(&self, py: Python<'_>, cmds: PyReadonlyArray2<'_, f64>) -> PyResult<()> {
        let arr = cmds.as_array();
        let expected_n = {
            let r = self.robot.bind(py).borrow();
            let inner = r
                .inner
                .lock()
                .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
            inner
                .group(&self.name)
                .ok_or_else(|| PyKeyError::new_err(self.name.clone()))?
                .inner()
                .len()
        };
        if arr.shape() != [expected_n, 3] {
            return Err(PyValueError::new_err(format!(
                "expected shape ({expected_n}, 3) float64, got ({}, {})",
                arr.shape()[0],
                arr.shape()[1]
            )));
        }
        let cmds: Vec<PosForceCmd> = arr
            .axis_iter(Axis(0))
            .map(|r| PosForceCmd {
                q: r[0],
                dq: r[1],
                i_pu: r[2],
            })
            .collect();
        let r = self.robot.bind(py).borrow();
        with_arm(&r, &self.name, |arm| arm.pos_force_control(&cmds))
    }

    fn enable_all(&self, py: Python<'_>) -> PyResult<()> {
        let r = self.robot.bind(py).borrow();
        with_arm(&r, &self.name, |arm| arm.enable_all())
    }
    fn disable_all(&self, py: Python<'_>) -> PyResult<()> {
        let r = self.robot.bind(py).borrow();
        with_arm(&r, &self.name, |arm| arm.disable_all())
    }
    fn set_zero_all(&self, py: Python<'_>) -> PyResult<()> {
        let r = self.robot.bind(py).borrow();
        with_arm(&r, &self.name, |arm| arm.set_zero_all())
    }
}

#[pyclass(name = "Gripper", module = "dm_control")]
pub struct PyGripper {
    pub(crate) robot: Py<PyRobot>,
    pub(crate) name: String,
}

#[pymethods]
impl PyGripper {
    #[getter]
    fn motor(&self, py: Python<'_>) -> PyResult<PyMotor> {
        Ok(PyMotor {
            robot: self.robot.clone_ref(py),
            group_name: self.name.clone(),
            motor_index: 0,
        })
    }

    fn enable(&self, py: Python<'_>) -> PyResult<()> {
        let r = self.robot.bind(py).borrow();
        with_gripper(&r, &self.name, |g| g.enable())
    }
    fn disable(&self, py: Python<'_>) -> PyResult<()> {
        let r = self.robot.bind(py).borrow();
        with_gripper(&r, &self.name, |g| g.disable())
    }
    fn mit_control(
        &self,
        py: Python<'_>,
        kp: f64,
        kd: f64,
        q: f64,
        dq: f64,
        tau: f64,
    ) -> PyResult<()> {
        let cmd = MitCmd { kp, kd, q, dq, tau };
        let r = self.robot.bind(py).borrow();
        with_gripper(&r, &self.name, |g| g.mit_control(cmd))
    }
    fn pos_vel_control(&self, py: Python<'_>, q: f64, dq: f64) -> PyResult<()> {
        let cmd = PosVelCmd { q, dq };
        let r = self.robot.bind(py).borrow();
        with_gripper(&r, &self.name, |g| g.pos_vel_control(cmd))
    }
}

#[pyclass(name = "MotorGroup", module = "dm_control")]
pub struct PyMotorGroup {
    pub(crate) robot: Py<PyRobot>,
    pub(crate) name: String,
}

#[pymethods]
impl PyMotorGroup {
    fn __len__(&self, py: Python<'_>) -> PyResult<usize> {
        let r = self.robot.bind(py).borrow();
        let inner = r
            .inner
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
        let g = inner
            .group(&self.name)
            .ok_or_else(|| PyKeyError::new_err(self.name.clone()))?;
        Ok(g.inner().len())
    }
}

/// Internal RobotBuilder accumulator that holds the Python-friendly handles.
enum PendingGroup {
    Arm(String, String, Vec<PyMotorSpec>),
    Gripper(String, String, PyMotorSpec),
    Generic(String, String, Vec<PyMotorSpec>),
}

type PendingBus = (String, Box<dyn CanBus>, Box<dyn MotorCodec>);

#[pyclass(name = "RobotBuilder", module = "dm_control")]
pub struct PyRobotBuilder {
    buses: Vec<PendingBus>,
    groups: Vec<PendingGroup>,
}

#[pymethods]
impl PyRobotBuilder {
    #[new]
    fn new() -> Self {
        Self {
            buses: Vec::new(),
            groups: Vec::new(),
        }
    }

    fn add_bus(
        mut slf: PyRefMut<'_, Self>,
        _py: Python<'_>,
        name: &str,
        transport: Bound<'_, PyAny>,
        codec: Bound<'_, PyAny>,
    ) -> PyResult<Py<Self>> {
        // Accept MockCanBus or SocketCanBus for transport; PyDamiaoCodec for codec.
        let transport_handle = if let Ok(mock) = transport.extract::<PyRef<'_, PyMockCanBus>>() {
            mock.handle.clone()
        } else if let Ok(sock) = transport.extract::<PyRef<'_, PySocketCanBus>>() {
            sock.handle.clone()
        } else {
            return Err(PyTypeError::new_err(
                "transport must be MockCanBus or SocketCanBus",
            ));
        };
        let codec_handle = if let Ok(dm) = codec.extract::<PyRef<'_, PyDamiaoCodec>>() {
            dm.handle.clone()
        } else {
            return Err(PyTypeError::new_err("codec must be DamiaoCodec"));
        };
        let transport = transport_handle
            .take()
            .ok_or_else(|| PyValueError::new_err("transport already consumed by another bus"))?;
        let codec = codec_handle
            .take()
            .ok_or_else(|| PyValueError::new_err("codec already consumed by another bus"))?;
        slf.buses.push((name.to_string(), transport, codec));
        Ok(slf.into())
    }

    #[pyo3(signature = (name, *, bus, motors))]
    fn add_arm(
        mut slf: PyRefMut<'_, Self>,
        name: &str,
        bus: &str,
        motors: Vec<PyMotorSpec>,
    ) -> Py<Self> {
        slf.groups
            .push(PendingGroup::Arm(name.to_string(), bus.to_string(), motors));
        slf.into()
    }

    #[pyo3(signature = (name, *, bus, motor))]
    fn add_gripper(
        mut slf: PyRefMut<'_, Self>,
        name: &str,
        bus: &str,
        motor: PyMotorSpec,
    ) -> Py<Self> {
        slf.groups.push(PendingGroup::Gripper(
            name.to_string(),
            bus.to_string(),
            motor,
        ));
        slf.into()
    }

    #[pyo3(signature = (name, *, bus, motors))]
    fn add_generic(
        mut slf: PyRefMut<'_, Self>,
        name: &str,
        bus: &str,
        motors: Vec<PyMotorSpec>,
    ) -> Py<Self> {
        slf.groups.push(PendingGroup::Generic(
            name.to_string(),
            bus.to_string(),
            motors,
        ));
        slf.into()
    }

    fn build(&mut self) -> PyResult<PyRobot> {
        let mut builder = RobotBuilder::new();
        for (name, transport, codec) in std::mem::take(&mut self.buses) {
            builder = builder.add_bus(name, transport, codec);
        }
        for g in std::mem::take(&mut self.groups) {
            builder = match g {
                PendingGroup::Arm(name, bus, motors) => {
                    builder.add_arm(name, bus, motors.into_iter().map(|s| s.inner).collect())
                }
                PendingGroup::Gripper(name, bus, motor) => {
                    builder.add_gripper(name, bus, motor.inner)
                }
                PendingGroup::Generic(name, bus, motors) => {
                    builder.add_generic(name, bus, motors.into_iter().map(|s| s.inner).collect())
                }
            };
        }
        let robot = builder.build().map_err(into_pyerr)?;
        Ok(PyRobot {
            inner: Arc::new(Mutex::new(robot)),
        })
    }
}

#[pyclass(name = "Robot", module = "dm_control")]
pub struct PyRobot {
    pub(crate) inner: Arc<Mutex<Robot>>,
}

#[pymethods]
impl PyRobot {
    #[staticmethod]
    fn from_config(py: Python<'_>, path: &str) -> PyResult<Self> {
        let path = path.to_string();
        let robot = py.allow_threads(|| -> Result<Robot, dm_control::Error> {
            let mut reg = CodecRegistry::new();
            reg.register(
                DAMIAO_VENDOR,
                || Box::new(DamiaoCodec::new()),
                dm_parse_type,
            );
            Robot::from_config(&path, &reg)
        });
        let robot = robot.map_err(into_pyerr)?;
        Ok(Self {
            inner: Arc::new(Mutex::new(robot)),
        })
    }

    #[staticmethod]
    fn builder() -> PyRobotBuilder {
        PyRobotBuilder::new()
    }

    fn connect(&self, py: Python<'_>) -> PyResult<()> {
        let inner = self.inner.clone();
        py.allow_threads(|| {
            let mut robot = inner.lock().map_err(|_| dm_control::Error::BusPoisoned)?;
            robot.connect()
        })
        .map_err(into_pyerr)
    }

    fn enable(&self, py: Python<'_>) -> PyResult<()> {
        let inner = self.inner.clone();
        py.allow_threads(|| {
            let mut robot = inner.lock().map_err(|_| dm_control::Error::BusPoisoned)?;
            robot.enable()
        })
        .map_err(into_pyerr)
    }

    fn disable(&self, py: Python<'_>) -> PyResult<()> {
        let inner = self.inner.clone();
        py.allow_threads(|| {
            let mut robot = inner.lock().map_err(|_| dm_control::Error::BusPoisoned)?;
            robot.disable()
        })
        .map_err(into_pyerr)
    }

    fn tick(&self, py: Python<'_>, per_bus_deadline_us: u64) -> PyResult<()> {
        let inner = self.inner.clone();
        py.allow_threads(|| {
            let mut robot = inner.lock().map_err(|_| dm_control::Error::BusPoisoned)?;
            robot.tick(Duration::from_micros(per_bus_deadline_us))
        })
        .map_err(into_pyerr)
    }

    fn __enter__(slf: Py<Self>, py: Python<'_>) -> PyResult<Py<Self>> {
        {
            let r = slf.bind(py).borrow();
            r.connect(py)?;
            r.enable(py)?;
        }
        Ok(slf)
    }

    #[allow(unused_variables)]
    fn __exit__<'py>(
        &self,
        py: Python<'py>,
        exc_type: Bound<'py, PyAny>,
        exc_val: Bound<'py, PyAny>,
        exc_tb: Bound<'py, PyAny>,
    ) -> PyResult<bool> {
        // Swallow disable errors; log via Python's logging. Do not mask the
        // body exception (return False so any in-flight exception propagates).
        if let Err(e) = self.disable(py) {
            let logging = py.import_bound("logging")?;
            let logger = logging.call_method1("getLogger", ("dm_control",))?;
            logger.call_method1("warning", (format!("disable failed: {e}"),))?;
        }
        Ok(false)
    }

    fn __getitem__(slf: Py<Self>, py: Python<'_>, name: &str) -> PyResult<PyObject> {
        let kind = {
            let r = slf.bind(py).borrow();
            let inner = r
                .inner
                .lock()
                .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
            let g = inner
                .group(name)
                .ok_or_else(|| PyKeyError::new_err(name.to_string()))?;
            std::mem::discriminant(g)
        };
        // Quick discriminant check using sample variants. PyO3 doesn't let us
        // pattern-match outside the lock, so re-lock to query.
        let r = slf.bind(py).borrow();
        let inner = r
            .inner
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
        let g = inner
            .group(name)
            .ok_or_else(|| PyKeyError::new_err(name.to_string()))?;
        let _ = kind;
        match g {
            GroupKind::Arm(_) => Ok(Py::new(
                py,
                PyArm {
                    robot: slf.clone_ref(py),
                    name: name.to_string(),
                },
            )?
            .into_any()),
            GroupKind::Gripper(_) => Ok(Py::new(
                py,
                PyGripper {
                    robot: slf.clone_ref(py),
                    name: name.to_string(),
                },
            )?
            .into_any()),
            GroupKind::Generic(_) => Ok(Py::new(
                py,
                PyMotorGroup {
                    robot: slf.clone_ref(py),
                    name: name.to_string(),
                },
            )?
            .into_any()),
            _ => Err(PyTypeError::new_err("unknown group kind variant")),
        }
    }

    fn __contains__(&self, _py: Python<'_>, name: &str) -> PyResult<bool> {
        let inner = self
            .inner
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
        Ok(inner.group(name).is_some())
    }

    fn group_names<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
        let inner = self
            .inner
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
        let names: Vec<String> = inner.group_names().map(str::to_string).collect();
        PyList::new_bound(py, names)
            .into_any()
            .downcast_into::<PyList>()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("list creation"))
    }

    fn bus_names<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
        let inner = self
            .inner
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
        let names: Vec<String> = inner.bus_names().map(str::to_string).collect();
        PyList::new_bound(py, names)
            .into_any()
            .downcast_into::<PyList>()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("list creation"))
    }

    fn is_connected(&self) -> PyResult<bool> {
        let inner = self
            .inner
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
        Ok(inner.is_connected())
    }
}
