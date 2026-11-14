//! [`Backend`]: one family of devices as the command and the daemon see it
//! (finding units, checking a request before a unit is opened, opening one
//! as a [`Session`], describing one). The family's own options are given to
//! its backend when it is built.

use crate::Result;
use crate::channels::System;
use crate::queue::TsQueue;
use crate::session::Session;

/// A unit a backend found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Unit {
    /// The name requests use: the USB serial number, or a name the backend
    /// makes up for a unit without one.
    pub name: String,
    pub vendor_id: u16,
    pub product_id: u16,
    /// The product name.
    pub label: String,
    /// Where it is (bus and address, or the device node).
    pub location: String,
}

/// What a request asks of a unit before it is opened.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Request {
    pub system: System,
    /// The tuner index (0, or 1 for the satellite / second terrestrial tuner).
    pub tuner: usize,
    /// Raw mode (the TS as received) rather than device mode.
    pub raw: bool,
}

/// One family of devices.
pub trait Backend: Send + Sync {
    /// The family's name, for messages ("DTH10 series").
    fn family(&self) -> &'static str;
    /// The units present now.
    fn units(&self) -> Result<Vec<Unit>>;
    /// Whether unit `name` can serve `req`, before it is opened: what is
    /// missing (a file the user has to supply, a tuner the family does not
    /// have), as a message for the client.
    fn check(&self, name: &str, req: Request) -> std::result::Result<(), String>;
    /// Open unit `name` with tuner 0 streaming into `queue`.
    fn open(&self, name: &str, queue: TsQueue) -> Result<Box<dyn Session>>;
    /// A description of unit `name` (`dth info`), one `key: value` per line.
    fn info(&self, name: &str) -> Result<String>;
}
