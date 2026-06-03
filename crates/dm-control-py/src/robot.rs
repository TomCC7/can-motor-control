//! Python wrappers for Robot, RobotBuilder, and the group kinds.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use dm_codec::{parse_motor_type as dm_parse_type, DamiaoCodec, VENDOR_NAME as DAMIAO_VENDOR};
use dm_control::{
    Arm, CanBus, CodecRegistry, Gripper, GroupKind, MitCmd, PosForceCmd, PosVelCmd, Robot,
    RobotBuilder, VelCmd,
};
use motor_codec::{CommandKind, MotorCodec};
use numpy::ndarray::Axis;
use numpy::{PyArray1, PyReadonlyArray1, PyReadonlyArray2};
use pyo3::exceptions::{PyKeyError, PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyAnyMethods, PyList};

use crate::codec::PyDamiaoCodec;
use crate::errors::into_pyerr;
use crate::spec::PyMotorSpec;
use crate::transport::{PyMockCanBus, PySocketCanBus};

/// A live view of one motor's most recent feedback.
///
/// Obtained by indexing an `Arm` by motor name (``arm["j1"]``) or via
/// `Gripper.motor`. Every attribute access reads the latest values
/// decoded by the most recent `Robot.tick`; the object holds no state
/// of its own.
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
    /// The motor's name, as declared in its `MotorSpec`.
    #[getter]
    fn name(&self, py: Python<'_>) -> PyResult<String> {
        Ok(self.snapshot(py)?.name)
    }
    /// The CAN id the host sends commands to this motor on.
    #[getter]
    fn send_id(&self, py: Python<'_>) -> PyResult<u32> {
        Ok(self.snapshot(py)?.send_id)
    }
    /// The CAN id this motor sends its feedback replies on.
    #[getter]
    fn recv_id(&self, py: Python<'_>) -> PyResult<u32> {
        Ok(self.snapshot(py)?.recv_id)
    }
    /// Latest measured shaft position, in radians.
    #[getter]
    fn position(&self, py: Python<'_>) -> PyResult<f64> {
        Ok(self.snapshot(py)?.position)
    }
    /// Latest measured shaft velocity, in radians per second.
    #[getter]
    fn velocity(&self, py: Python<'_>) -> PyResult<f64> {
        Ok(self.snapshot(py)?.velocity)
    }
    /// Latest estimated output torque, in newton-metres.
    #[getter]
    fn torque(&self, py: Python<'_>) -> PyResult<f64> {
        Ok(self.snapshot(py)?.torque)
    }
    /// Latest MOSFET temperature, in degrees Celsius.
    #[getter]
    fn temperature_mos(&self, py: Python<'_>) -> PyResult<i16> {
        Ok(self.snapshot(py)?.t_mos)
    }
    /// Latest rotor temperature, in degrees Celsius.
    #[getter]
    fn temperature_rotor(&self, py: Python<'_>) -> PyResult<i16> {
        Ok(self.snapshot(py)?.t_rotor)
    }
    /// ``True`` if the motor reported itself enabled at the last tick.
    #[getter]
    fn is_enabled(&self, py: Python<'_>) -> PyResult<bool> {
        Ok(self.snapshot(py)?.is_enabled)
    }
    /// The motor's fault code, or ``None`` if it reports no fault.
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
/// Parse a Python control-mode string (`"mit"`, `"pos_vel"`, `"vel"`,
/// `"pos_force"`; hyphens accepted) into a [`CommandKind`].
fn parse_control_mode(s: &str) -> PyResult<CommandKind> {
    match s.to_ascii_lowercase().replace('-', "_").as_str() {
        "mit" => Ok(CommandKind::Mit),
        "pos_vel" | "posvel" => Ok(CommandKind::PosVel),
        "vel" => Ok(CommandKind::Vel),
        "pos_force" | "posforce" => Ok(CommandKind::PosForce),
        other => Err(PyValueError::new_err(format!(
            "unknown control mode {other:?}; expected one of: mit, pos_vel, vel, pos_force"
        ))),
    }
}

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

/// A named group of motors driven together as an arm.
///
/// Obtained by indexing a `Robot` by group name (``robot["arm"]``).
/// ``len(arm)`` is the motor count and ``arm["j1"]`` returns a
/// `Motor`. The control methods take a NumPy command array with one
/// row per motor, in declaration order; commands are queued and flushed on the
/// next `Robot.tick`.
#[pyclass(name = "Arm", module = "dm_control")]
pub struct PyArm {
    pub(crate) robot: Py<PyRobot>,
    pub(crate) name: String,
}

#[pymethods]
impl PyArm {
    /// The number of motors in the arm.
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

    /// Return the `Motor` named ``motor_name`` (raises ``KeyError``
    /// if no motor in the arm has that name).
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

    /// Latest measured positions (radians) as a ``(n,)`` float64 array, one
    /// entry per motor in declaration order.
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
    /// Latest measured velocities (rad/s) as a ``(n,)`` float64 array, one
    /// entry per motor in declaration order.
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
    /// Latest estimated torques (N·m) as a ``(n,)`` float64 array, one entry
    /// per motor in declaration order.
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

    /// Queue an MIT-mode (impedance) command for every motor.
    ///
    /// ``cmds`` is a ``(n, 5)`` float64 array with one row ``[kp, kd, q, dq,
    /// tau]`` per motor, in declaration order: position gain, velocity gain,
    /// target position (rad), target velocity (rad/s), and feed-forward torque
    /// (N·m). A wrong shape raises ``ValueError``. The command is sent on the
    /// next `Robot.tick`.
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

    /// Queue a position-velocity command for every motor.
    ///
    /// ``cmds`` is a ``(n, 2)`` float64 array with one row ``[q, dq]`` per
    /// motor, in declaration order: target position (rad) and target velocity
    /// (rad/s). A wrong shape raises ``ValueError``. The command is sent on the
    /// next `Robot.tick`.
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

    /// Queue a velocity command for every motor.
    ///
    /// ``cmds`` is a ``(n,)`` float64 array of target velocities (rad/s), one
    /// per motor in declaration order. A wrong shape raises ``ValueError``. The
    /// command is sent on the next `Robot.tick`.
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

    /// Queue a position-force command for every motor.
    ///
    /// ``cmds`` is a ``(n, 3)`` float64 array with one row ``[q, dq, i_pu]``
    /// per motor, in declaration order: target position (rad), target velocity
    /// (rad/s), and current in per-unit. A wrong shape raises ``ValueError``.
    /// The command is sent on the next `Robot.tick`.
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

    /// Enable every motor in the arm (queued for the next tick).
    fn enable_all(&self, py: Python<'_>) -> PyResult<()> {
        let r = self.robot.bind(py).borrow();
        with_arm(&r, &self.name, |arm| arm.enable_all())
    }
    /// Disable every motor in the arm (queued for the next tick).
    fn disable_all(&self, py: Python<'_>) -> PyResult<()> {
        let r = self.robot.bind(py).borrow();
        with_arm(&r, &self.name, |arm| arm.disable_all())
    }
    /// Set the current position of every motor as its new zero reference.
    fn set_zero_all(&self, py: Python<'_>) -> PyResult<()> {
        let r = self.robot.bind(py).borrow();
        with_arm(&r, &self.name, |arm| arm.set_zero_all())
    }
    /// Send a state-refresh query to every motor in the arm (commands no
    /// motion). Pair with `Robot.tick` to receive the replies — this is how a
    /// read loop keeps state fresh without driving the motors.
    fn refresh(&self, py: Python<'_>) -> PyResult<()> {
        let r = self.robot.bind(py).borrow();
        with_arm(&r, &self.name, |arm| arm.refresh())
    }
    /// Set the persistent control mode on every motor in the arm. ``mode`` is
    /// one of ``"mit"``, ``"pos_vel"``, ``"vel"``, ``"pos_force"``. Commands no
    /// motion; call once at startup (after ``connect``, before commanding) so
    /// the matching control commands take effect. Raises `ValueError` for an
    /// unknown mode.
    fn set_mode(&self, py: Python<'_>, mode: &str) -> PyResult<()> {
        let kind = parse_control_mode(mode)?;
        let r = self.robot.bind(py).borrow();
        with_arm(&r, &self.name, |arm| arm.set_mode(kind))
    }
}

/// A single-motor group driven as a gripper.
///
/// Obtained by indexing a `Robot` by group name. Unlike
/// `Arm`, the control methods take scalar arguments for the one
/// motor. Commands are queued and flushed on the next `Robot.tick`.
#[pyclass(name = "Gripper", module = "dm_control")]
pub struct PyGripper {
    pub(crate) robot: Py<PyRobot>,
    pub(crate) name: String,
}

#[pymethods]
impl PyGripper {
    /// The gripper's underlying `Motor`.
    #[getter]
    fn motor(&self, py: Python<'_>) -> PyResult<PyMotor> {
        Ok(PyMotor {
            robot: self.robot.clone_ref(py),
            group_name: self.name.clone(),
            motor_index: 0,
        })
    }

    /// Enable the gripper motor (queued for the next tick).
    fn enable(&self, py: Python<'_>) -> PyResult<()> {
        let r = self.robot.bind(py).borrow();
        with_gripper(&r, &self.name, |g| g.enable())
    }
    /// Disable the gripper motor (queued for the next tick).
    fn disable(&self, py: Python<'_>) -> PyResult<()> {
        let r = self.robot.bind(py).borrow();
        with_gripper(&r, &self.name, |g| g.disable())
    }
    /// Queue an MIT-mode (impedance) command for the gripper motor.
    ///
    /// ``kp``/``kd`` are the position and velocity gains, ``q``/``dq`` the
    /// target position (rad) and velocity (rad/s), and ``tau`` the
    /// feed-forward torque (N·m). Sent on the next `Robot.tick`.
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
    /// Queue a position-velocity command (target position ``q`` rad, target
    /// velocity ``dq`` rad/s) for the gripper motor.
    fn pos_vel_control(&self, py: Python<'_>, q: f64, dq: f64) -> PyResult<()> {
        let cmd = PosVelCmd { q, dq };
        let r = self.robot.bind(py).borrow();
        with_gripper(&r, &self.name, |g| g.pos_vel_control(cmd))
    }
    /// Send a state-refresh query to the gripper motor (commands no motion).
    /// Pair with `Robot.tick` to receive the reply.
    fn refresh(&self, py: Python<'_>) -> PyResult<()> {
        let r = self.robot.bind(py).borrow();
        with_gripper(&r, &self.name, |g| g.refresh())
    }
    /// Set the gripper motor's persistent control mode (``"mit"``, ``"pos_vel"``,
    /// ``"vel"``, ``"pos_force"``). Commands no motion. Raises `ValueError` for
    /// an unknown mode.
    fn set_mode(&self, py: Python<'_>, mode: &str) -> PyResult<()> {
        let kind = parse_control_mode(mode)?;
        let r = self.robot.bind(py).borrow();
        with_gripper(&r, &self.name, |g| g.set_mode(kind))
    }
}

/// A generic, named group of motors with no arm/gripper semantics.
///
/// Created via `RobotBuilder.add_generic` and obtained by indexing a
/// `Robot` by group name. Currently exposes only its motor count.
#[pyclass(name = "MotorGroup", module = "dm_control")]
pub struct PyMotorGroup {
    pub(crate) robot: Py<PyRobot>,
    pub(crate) name: String,
}

#[pymethods]
impl PyMotorGroup {
    /// The number of motors in the group.
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

/// Fluent builder for assembling a `Robot` in code.
///
/// Add buses with `add_bus`, then attach groups with
/// `add_arm`, `add_gripper`, or `add_generic`, and
/// finish with `build`. Each method returns the builder so calls can
/// be chained. Obtain one from `Robot.builder`.
#[pyclass(name = "RobotBuilder", module = "dm_control")]
pub struct PyRobotBuilder {
    buses: Vec<PendingBus>,
    groups: Vec<PendingGroup>,
}

#[pymethods]
impl PyRobotBuilder {
    /// Create an empty builder.
    #[new]
    fn new() -> Self {
        Self {
            buses: Vec::new(),
            groups: Vec::new(),
        }
    }

    /// Register a named bus from a ``transport`` and a ``codec``.
    ///
    /// ``transport`` is a `MockCanBus` or `SocketCanBus`
    /// and ``codec`` is a `dm_control.damiao.DamiaoCodec`. Both are
    /// consumed: passing one already added to another bus raises
    /// ``ValueError``. Returns the builder for chaining.
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

    /// Attach an arm named ``name`` on bus ``bus`` from a list of
    /// `MotorSpec`. Returns the builder for chaining.
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

    /// Attach a single-motor gripper named ``name`` on bus ``bus``. Returns
    /// the builder for chaining.
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

    /// Attach a generic motor group named ``name`` on bus ``bus`` (no
    /// arm/gripper semantics). Returns the builder for chaining.
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

    /// Consume the builder and return the assembled `Robot`.
    ///
    /// Raises `ConfigError` if the topology is invalid (e.g. a group
    /// references an unknown bus).
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

/// The top-level handle to a configured robot.
///
/// Build one from a config file with `from_config` or in code with
/// `builder`. The lifecycle is `connect` →
/// `enable` → repeated `tick` → `disable`; using it
/// as a context manager (``with Robot.from_config(...) as r:``) connects and
/// enables on entry and disables on exit.
///
/// Index by group name to get the corresponding `Arm`,
/// `Gripper`, or `MotorGroup`; use ``in`` to test for a
/// group. Control methods on those groups only queue commands — call
/// `tick` to exchange frames with the hardware.
#[pyclass(name = "Robot", module = "dm_control")]
pub struct PyRobot {
    pub(crate) inner: Arc<Mutex<Robot>>,
}

#[pymethods]
impl PyRobot {
    /// Load a robot from a TOML config file at ``path``.
    ///
    /// Raises `ConfigError` if the file is missing or invalid.
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

    /// Return a fresh `RobotBuilder` for assembling a robot in code.
    #[staticmethod]
    fn builder() -> PyRobotBuilder {
        PyRobotBuilder::new()
    }

    /// Open every bus and prepare the robot for control.
    ///
    /// Call once before `enable`/`tick`. Raises
    /// `TransportError` if a bus cannot be opened.
    fn connect(&self, py: Python<'_>) -> PyResult<()> {
        let inner = self.inner.clone();
        py.allow_threads(|| {
            let mut robot = inner.lock().map_err(|_| dm_control::Error::BusPoisoned)?;
            robot.connect()
        })
        .map_err(into_pyerr)
    }

    /// Enable every motor on the robot.
    ///
    /// Requires the robot to be connected; otherwise raises
    /// `LifecycleError`.
    fn enable(&self, py: Python<'_>) -> PyResult<()> {
        let inner = self.inner.clone();
        py.allow_threads(|| {
            let mut robot = inner.lock().map_err(|_| dm_control::Error::BusPoisoned)?;
            robot.enable()
        })
        .map_err(into_pyerr)
    }

    /// Disable every motor on the robot.
    ///
    /// Safe to call during shutdown; the context-manager exit calls this
    /// automatically.
    fn disable(&self, py: Python<'_>) -> PyResult<()> {
        let inner = self.inner.clone();
        py.allow_threads(|| {
            let mut robot = inner.lock().map_err(|_| dm_control::Error::BusPoisoned)?;
            robot.disable()
        })
        .map_err(into_pyerr)
    }

    /// Exchange one round of frames with every bus.
    ///
    /// Flushes all commands queued on the groups since the last tick and reads
    /// back feedback, blocking up to ``per_bus_deadline_us`` microseconds per
    /// bus. This is the call that drives a realtime control loop; the GIL is
    /// released for its duration. Raises `LifecycleError` if the
    /// robot is not connected, or `TransportError` on a bus failure.
    fn tick(&self, py: Python<'_>, per_bus_deadline_us: u64) -> PyResult<()> {
        let inner = self.inner.clone();
        py.allow_threads(|| {
            let mut robot = inner.lock().map_err(|_| dm_control::Error::BusPoisoned)?;
            robot.tick(Duration::from_micros(per_bus_deadline_us))
        })
        .map_err(into_pyerr)
    }

    /// Send a state-refresh query to every motor in every group (commands no
    /// motion).
    ///
    /// Send-only — pair with `Robot.tick` to receive the replies. The GIL is
    /// released for its duration. Raises `LifecycleError` if the robot is not
    /// connected, or `TransportError` on a bus failure.
    fn refresh(&self, py: Python<'_>) -> PyResult<()> {
        let inner = self.inner.clone();
        py.allow_threads(|| {
            let mut robot = inner.lock().map_err(|_| dm_control::Error::BusPoisoned)?;
            robot.refresh()
        })
        .map_err(into_pyerr)
    }

    /// Set the persistent control mode on every motor in every group. ``mode``
    /// is one of ``"mit"``, ``"pos_vel"``, ``"vel"``, ``"pos_force"``. Commands
    /// no motion; call once after `connect` and before commanding. The GIL is
    /// released for its duration. Raises `ValueError` for an unknown mode,
    /// `LifecycleError` if not connected.
    fn set_mode(&self, py: Python<'_>, mode: &str) -> PyResult<()> {
        let kind = parse_control_mode(mode)?;
        let inner = self.inner.clone();
        py.allow_threads(|| {
            let mut robot = inner.lock().map_err(|_| dm_control::Error::BusPoisoned)?;
            robot.set_mode(kind)
        })
        .map_err(into_pyerr)
    }

    /// Context-manager entry: `connect` then `enable`.
    fn __enter__(slf: Py<Self>, py: Python<'_>) -> PyResult<Py<Self>> {
        {
            let r = slf.bind(py).borrow();
            r.connect(py)?;
            r.enable(py)?;
        }
        Ok(slf)
    }

    /// Context-manager exit: `disable` the robot (errors are logged,
    /// not raised, so an in-flight exception propagates unmasked).
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

    /// Return the group named ``name`` as an `Arm`,
    /// `Gripper`, or `MotorGroup` (raises ``KeyError`` if
    /// there is no such group).
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

    /// ``True`` if a group named ``name`` exists (supports ``name in robot``).
    fn __contains__(&self, _py: Python<'_>, name: &str) -> PyResult<bool> {
        let inner = self
            .inner
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
        Ok(inner.group(name).is_some())
    }

    /// The names of every group on the robot.
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

    /// The names of every bus on the robot.
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

    /// ``True`` once `connect` has succeeded.
    fn is_connected(&self) -> PyResult<bool> {
        let inner = self
            .inner
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("poisoned"))?;
        Ok(inner.is_connected())
    }
}
