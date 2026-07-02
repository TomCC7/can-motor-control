//! Layered error type for the `can-motor-control` crate.

use std::io;

use motor_codec::{CodecError, MotorTypeId};
use thiserror::Error;

use crate::bus::RouteKey;
use crate::transport::TransportError;

/// All errors `can-motor-control` can return.
///
/// Wraps the lower-layer error types ([`TransportError`], [`CodecError`])
/// while adding composition-level variants (config schema, builder validation,
/// lifecycle).
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum Error {
    /// A transport operation failed.
    #[error("transport: {0}")]
    Transport(#[from] TransportError),

    /// A codec operation failed.
    #[error("codec: {0}")]
    Codec(#[from] CodecError),

    /// TOML schema validation failed.
    #[error("config schema: {0}")]
    ConfigSchema(String),

    /// Reading the config file failed.
    #[error("config IO: {0}")]
    ConfigIo(#[from] io::Error),

    /// Reference to a bus name that wasn't registered.
    #[error("unknown bus name: {0}")]
    UnknownBusName(String),

    /// Vendor name not in the codec registry.
    #[error("unknown vendor: {0}")]
    UnknownVendor(String),

    /// Two `add_bus` calls used the same name.
    #[error("duplicate bus name: {0}")]
    DuplicateBusName(String),

    /// Two `add_*` group calls used the same name.
    #[error("duplicate group name: {0}")]
    DuplicateGroupName(String),

    /// The bus's codec does not support a motor type the user tried to add.
    #[error("bus '{bus_name}' codec '{vendor}' does not support motor type {motor_type:?}")]
    MotorNotSupportedByCodec {
        /// Vendor short-name of the bus's codec.
        vendor: String,
        /// Motor type the user attempted to add.
        motor_type: MotorTypeId,
        /// Bus name the motor was being attached to.
        bus_name: String,
    },

    /// Gripper construction tried to attach a number of motors other than 1.
    #[error("gripper requires exactly one motor, got {got}")]
    GripperRequiresOneMotor {
        /// Motor count supplied.
        got: usize,
    },

    /// A batch command's slice length didn't match the group's motor count.
    #[error("command length mismatch: expected {expected}, got {got}")]
    CommandLengthMismatch {
        /// `arm.len()`.
        expected: usize,
        /// Caller-supplied slice length.
        got: usize,
    },

    /// Operation requires `connect()` first.
    #[error("not connected; call Robot::connect() first")]
    NotConnected,

    /// Normalized gripper opening was outside the supported range.
    #[error("gripper opening must be between 0.0 and 1.0, got {got}")]
    OpeningOutOfRange {
        /// Caller-supplied opening.
        got: f64,
    },

    /// Per-unit opening current was outside the supported range.
    #[error("gripper opening current must be > 0.0 and <= 1.0, got {got}")]
    OpeningCurrentOutOfRange {
        /// Caller-supplied per-unit current.
        got: f64,
    },

    /// Normalized opening was requested before session calibration completed.
    #[error("gripper opening calibration has not completed")]
    OpeningCalibrationRequired,

    /// Per-session gripper opening calibration failed.
    #[error("gripper opening calibration failed for '{name}': {reason}")]
    OpeningCalibrationFailed {
        /// Gripper group name.
        name: String,
        /// Human-readable reason.
        reason: &'static str,
    },

    /// Builder-style mutation after `connect()`.
    #[error("topology locked after connect()")]
    TopologyLocked,

    /// Two motors on one bus share the same recv ID.
    #[error("CAN ID collision on bus '{bus_name}': recv_id {recv_id:#x} already routed to {existing:?}, attempted {attempted:?}")]
    CanIdCollision {
        /// Bus where the collision occurred.
        bus_name: String,
        /// The colliding recv_id.
        recv_id: u32,
        /// The existing route.
        existing: RouteKey,
        /// The route the second motor was trying to claim.
        attempted: RouteKey,
    },

    /// The bus mutex was poisoned (a thread panicked while holding it).
    #[error("bus mutex poisoned")]
    BusPoisoned,

    /// Invariant violated.
    #[error("internal error: {0}")]
    Internal(&'static str),
}
