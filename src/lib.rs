#![no_std]
#![deny(unsafe_code)]
#![warn(missing_docs)]
// The transport is async and single-threaded by design (Embassy and friends);
// the missing `Send` bounds this lint flags are intentional, not an oversight.
#![allow(async_fn_in_trait)]

//! Driver for the Flying Pig Systems / High End "USB DMX Widget Mk3"
//! (Cypress EZ-USB AN2131).
//!
//! The widget holds no firmware of its own, so operating it is a two-stage
//! affair:
//!
//! 1. the device appears as a bootloader ([`identity::PID_UNLOADED`]); upload a
//!    firmware image into its RAM with [`upload`];
//! 2. it drops off the bus and re-enumerates as the loaded device
//!    ([`identity::PID_LOADED`]); stream DMX universes to it with
//!    [`device::Widget`].
//!
//! The firmware image is not distributed with this crate — see the README on the
//! bring-your-own-firmware model. USB host I/O is abstracted behind
//! [`transport::Transport`]; the crate is otherwise `no_std` and `no_alloc`.

pub mod anchor;
pub mod device;
pub mod dmx;
pub mod error;
pub mod firmware;
pub mod identity;
pub mod query;
pub mod receive;
pub mod transport;

pub use anchor::upload;
pub use device::{Mode, Widget};
pub use dmx::Universe;
pub use error::Error;
pub use query::{read_id, read_mode, read_serial};
pub use receive::{Receiver, RxEvent, RxEventKind, RxStats};
pub use transport::Transport;
