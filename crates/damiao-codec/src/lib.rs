#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]
//! Damiao motor codec — implements [`motor_codec::MotorCodec`] for the Damiao
//! CAN protocol.
//!
//! All Damiao-specific behavior (motor SKU table, MIT bit-packing, CAN-ID
//! offsets, the `0x7FF` parameter sub-protocol exposed via [`DamiaoCodecExt`])
//! is confined to this crate. `can-motor-control` depends on [`motor_codec`] only;
//! adding a future vendor (Robostride, MyActuator) is an additive crate with
//! the same shape.

extern crate alloc;

mod bitpack;
mod codec;
mod ext;
mod limits;
mod types;

pub use codec::DamiaoCodec;
pub use ext::DamiaoCodecExt;
pub use types::{parse_motor_type, DamiaoMotorType, DamiaoRid};

/// Vendor short-name `can-motor-control`'s `CodecRegistry` uses to look up Damiao.
pub const VENDOR_NAME: &str = "damiao";
