//! What the tuner daemon and its clients (the stream command, the BonDriver)
//! share, apart from the device code:
//!
//! * [`proto`] – the request/reply lines
//! * [`ipc`]   – the socket those lines travel on (Unix domain socket, or loopback TCP)
//!
//! Kept apart from `dth-common` so that the chip crates, which depend on
//! that, do not pull in a socket library.

pub mod ipc;
pub mod proto;
