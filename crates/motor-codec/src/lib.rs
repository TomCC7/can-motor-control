#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]
//! Vendor-agnostic motor codec contract.
//!
//! This crate defines [`MotorCodec`] — the single trait the rest of the
//! `dm-control` stack uses to talk to a vendor-specific motor protocol —
//! together with the shared types every vendor agrees on ([`CanFrame`],
//! [`Command`], [`Event`], [`MotorTypeId`], [`Limits`], [`BusCapabilities`]).
//!
//! The crate is `no_std`. Vendor codec implementations (e.g. `dm-codec`)
//! live in their own crates; `dm-control` depends on this trait crate only
//! and never on a specific vendor.
//!
//! ```
//! use motor_codec::{CanFrame, MotorCodec};
//! // Trait is object-safe: usable as `Box<dyn MotorCodec>`.
//! fn _accept_dyn(_c: alloc::boxed::Box<dyn MotorCodec>) {}
//! # extern crate alloc;
//! ```

extern crate alloc;

mod caps;
mod codec;
mod command;
mod event;
mod frame;
mod motor;

pub use caps::BusCapabilities;
pub use codec::{CodecError, MotorCodec};
pub use command::{Command, CommandKind};
pub use event::{Event, ParamValue};
pub use frame::{CanFrame, FrameError, FrameFlags};
pub use motor::{Limits, MotorRef, MotorTypeId};
