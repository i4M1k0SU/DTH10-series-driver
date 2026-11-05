//! I/O-agnostic reimplementation of the protocol the vendor's Windows driver
//! speaks to the Cobalt Technologies EVA01 USB tuner bridge found in the
//! BUFFALO DT-H10/H11/H33/H55/H70/F100/F110/F200 series.
//!
//! The crate contains no USB code itself. A transport implements
//! [`protocol::UsbControl`] (vendor control transfers) and the higher layers
//! build on it:
//!
//! * [`protocol`]  – 8-byte request encoding and the `send` primitive
//! * [`tlv`]       – device information blob (wValue 0 / wIndex 0xC0DE) parser
//! * [`device`]    – [`device::Device`]: GPIO, I2C (raw / register / batch), stream control, PID filter
//! * [`kit`]       – what every kit shares: tune request, signal status, lock / field bits
//! * [`tc90517`] + [`mxl13xrf`] – ISDB-T kit `TC90517_MxL13xRF` / `SMT_LJ101` hw 0x102 (tuner/filter #0)
//! * [`terrestrial`] – the kit of tuner #0 by kit name: [`tc90517`], [`pn2031`], [`bq200x`], [`lgdt3305`]
//! * [`lj101`]     – ISDB-S kit `SMT_LJ101` hw 0x402 (tuner/filter #1): TC90517 satellite side + PLL tuner
//! * [`events`]    – interrupt-endpoint event notifications
//! * [`cepm`]      – link encryption of the original stream mode (AES-CTR + ECDH handshake, docs/22)

pub mod bq200x;
pub mod cepm;
pub mod device;
pub mod error;
pub mod events;
pub mod kit;
pub mod lgdt3305;
pub mod lj101;
pub mod mxl13xrf;
pub mod pn2031;
pub mod protocol;
pub mod tc90517;
pub mod terrestrial;
pub mod tlv;
pub mod tuner_bus;

pub use device::{Device, StreamMode};
pub use error::{Error, Result};
pub use tlv::{DeviceInfo, TunerDesc};

/// USB vendor ID of BUFFALO INC.
pub const VID_BUFFALO: u16 = 0x0411;

/// Product IDs handled by the vendor driver's INF.
pub const SUPPORTED_PIDS: &[(u16, &str)] = &[
    (0x015F, "DT-H10/U2"),
    (0x016B, "DT-H10/PCI"),
    (0x017C, "DT-H33/U2"),
    (0x017D, "DT-H33/PCI"),
    (0x0186, "DT-F100/U2"),
    (0x0187, "DT-H55/U2W"),
    (0x0199, "DT-H70/U2"),
    (0x01BE, "DT-H70/PCIE"),
    (0x01D2, "DT-F110/U2"),
    (0x01DA, "DT-H11/U2"),
    (0x01DB, "DT-F200/U2W"),
];

/// Returns the marketing name for a supported product ID.
pub fn product_name(pid: u16) -> Option<&'static str> {
    SUPPORTED_PIDS.iter().find(|(p, _)| *p == pid).map(|(_, n)| *n)
}

/// `Mutex::lock` that carries on when a thread panicked while holding the
/// lock: what these crates keep behind a mutex (queues, handshake sets,
/// connection slots) stays usable, and a poisoned lock would otherwise take
/// the whole tuner down with the one thread.
pub trait LockExt<T> {
    fn locked(&self) -> std::sync::MutexGuard<'_, T>;
}

impl<T> LockExt<T> for std::sync::Mutex<T> {
    fn locked(&self) -> std::sync::MutexGuard<'_, T> {
        self.lock().unwrap_or_else(|e| e.into_inner())
    }
}
