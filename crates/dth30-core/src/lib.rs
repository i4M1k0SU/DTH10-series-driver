//! Chip drivers for the BUFFALO DT-H30/U2 family of USB TV tuners
//! (DT-H30/U2, DT-H50/U2, DT-H50/U2W), I/O-agnostic.
//!
//! The hardware is unrelated to the DTH10 series: a ViXS XCode2 ("Camino")
//! PCI chip with a MIPS core sits behind a PLX NET2280 USB controller in PCI
//! host mode, so the host reaches the chip's PCI configuration space and
//! registers through bulk transfers ([`bus::RemotePci`], implemented by the
//! `dth30-usb` crate). The chip runs firmware taken from the user's copy of
//! the vendor driver ([`vixs::firmware`]); the tuner and demodulator are
//! driven by the host over the chip's I2C master ([`i2c::I2c`],
//! [`frontend`]).
//!
//! The analysis this follows is in the private notes (docs/26).
//!
//! Errors reuse [`dth10_core::error`].

pub mod bus;
pub mod frontend;
pub mod i2c;
pub mod model;
pub mod net2280;
pub mod vixs;

pub use dth10_core::error::{Error, Result};
