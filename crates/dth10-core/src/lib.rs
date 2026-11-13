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
//! * [`kit`]       – what every kit shares: tune request, signal status, lock / field bits, vendor driver
//! * [`tc90517`] + [`mxl13xrf`] – ISDB-T kit `TC90517_MxL13xRF` / `SMT_LJ101` hw 0x102 (tuner/filter #0)
//! * [`terrestrial`] – the kit of tuner #0 by kit name: [`tc90517`], [`sharp8011`], [`tc9052x`], [`tc90527`], [`pn2031`], [`bq200x`], [`lgdt3305`]
//! * [`lj101`]     – ISDB-S kit `SMT_LJ101` hw 0x402 (tuner/filter #1): TC90517 satellite side + PLL tuner
//! * [`sharp8011`] – ISDB-T / ISDB-S kits `SHARP_8011` hw 0x102 / 0x402: TC90517 + the module's PLL tuners
//! * [`tc9052x`]   – ISDB-T / ISDB-S kits `TC90522_*` / `TC90532_*` hw 0x102 / 0x402: TC90517-class demodulator + tuner
//! * [`tc90527`]   – ISDB-T kits `TC90527_FC5500` / `TC90527_MTV808`: TC90527 demodulator + tuner
//! * [`fc5500`], [`mtv808`] – the terrestrial tuners of the TC9052x and TC90527 kits
//! * [`satellite`] – the kit of the satellite tuner by kit name and hw version: [`lj101`], [`sharp8011`], [`tc9052x`]
//! * [`events`]    – interrupt-endpoint event notifications
//! * [`ts_slot`]   – where the TS packet sits in each 256-byte slot of the bulk pipe
//! * [`cepm`]      – link encryption of the original stream mode (AES-CTR + ECDH handshake, docs/22)

pub mod bq200x;
pub mod cepm;
pub mod device;
pub mod error;
pub mod events;
pub mod fc5500;
pub mod kit;
pub mod lgdt3305;
pub mod lj101;
pub mod mtv808;
pub mod mxl13xrf;
pub mod pn2031;
pub mod protocol;
#[cfg(test)]
mod regmock;
pub mod satellite;
pub mod sharp8011;
pub mod tc90517;
pub mod tc90527;
pub mod tc9052x;
pub mod terrestrial;
pub mod tlv;
pub mod ts_slot;
pub mod tuner_bus;

pub use device::{Device, StreamMode};
pub use error::{Error, Result};
pub use tlv::{DeviceInfo, TunerDesc};

/// USB vendor ID of BUFFALO INC.
pub const VID_BUFFALO: u16 = 0x0411;

/// Product IDs this implementation drives, with their marketing names: the
/// models of the vendor driver's INF, followed by the models of
/// [`UNVERIFIED_PIDS`].
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
    (0x01FF, "DT-H71/U2"),
    (0x0200, "DT-H70B/PCIE"),
    (0x0202, "DT-F120/U2"),
    (0x0218, "DT-H30/PCIEM-TS"),
    (PID_DTH10_U7, "DT-H10/U7"),
];

/// USB product ID of the DT-H10/U7, the Windows 7 Media Center model of the
/// DT-H10 (unverified, see [`UNVERIFIED_PIDS`]). Its own vendor driver speaks
/// the same control protocol, but its firmware puts the TS packets into the
/// bulk slots differently ([`ts_slot::SlotLayout::Split`]), and it reports the
/// `PN2031_MxL135RF` kit, which that driver sets up with its own register
/// sequences ([`pn2031::Pn2031Variant::Dth10U7`]). Its firmware has no card
/// status query (`0xCBDA`/`0xBCAC`), no GPIO status read (`0x1010`) and no
/// antenna (LNB) supply; nothing on the open / tune / stream path sends them.
pub const PID_DTH10_U7: u16 = 0x017B;

/// Supported product IDs that are not in the vendor driver's INF this
/// implementation follows and that have not been checked on a unit: the
/// successor models, listed in the INFs of newer vendor drivers (0x01FF,
/// 0x0200 and 0x0202 in the 5.1.9.33 one of the series, 0x0218 in the
/// separate DT-H30/PCIEM-TS driver 5.1.9.46). Their boards may report a
/// tuner kit that is not implemented here. And the DT-H10/U7
/// ([`PID_DTH10_U7`]), which has a vendor driver of its own and a different
/// TS slot layout.
pub const UNVERIFIED_PIDS: &[u16] = &[0x01FF, 0x0200, 0x0202, 0x0218, PID_DTH10_U7];

/// Returns the marketing name for a supported product ID.
pub fn product_name(pid: u16) -> Option<&'static str> {
    SUPPORTED_PIDS.iter().find(|(p, _)| *p == pid).map(|(_, n)| *n)
}

/// Whether `pid` is one of the [`UNVERIFIED_PIDS`].
pub fn is_unverified(pid: u16) -> bool {
    UNVERIFIED_PIDS.contains(&pid)
}

/// The model name to show the user: the marketing name, followed by
/// "(unverified model)" for the [`UNVERIFIED_PIDS`]; "?" for an unknown PID.
pub fn product_label(pid: u16) -> String {
    match product_name(pid) {
        Some(name) if is_unverified(pid) => format!("{name} (unverified model)"),
        Some(name) => name.to_owned(),
        None => "?".to_owned(),
    }
}

/// A kit error of an unverified model ([`UNVERIFIED_PIDS`]) with a request
/// for the device information of the tuner path `tuner` appended; any other
/// error, or the error of another model, is returned unchanged.
pub fn unverified_kit_error(pid: u16, tuner: u8, e: Error) -> Error {
    match e {
        Error::Unsupported(m) if is_unverified(pid) => {
            let cmd = if tuner == 0 {
                "dth10-diag info --raw".to_owned()
            } else {
                format!("dth10-diag --tuner {tuner} info --raw")
            };
            Error::Unsupported(format!(
                "{m}; the {} is an unverified model, its device information dump (`{cmd}`) would help to support it",
                product_name(pid).unwrap_or("?")
            ))
        }
        e => e,
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn successor_models_are_supported_but_unverified() {
        assert_eq!(product_name(0x01FF), Some("DT-H71/U2"));
        assert_eq!(product_name(0x0202), Some("DT-F120/U2"));
        assert_eq!(product_name(0x0200), Some("DT-H70B/PCIE"));
        assert_eq!(product_name(0x0218), Some("DT-H30/PCIEM-TS"));
        assert!(UNVERIFIED_PIDS.iter().all(|&pid| is_unverified(pid)));
        assert_eq!(product_label(0x01FF), "DT-H71/U2 (unverified model)");
        assert_eq!(product_label(0x0202), "DT-F120/U2 (unverified model)");
        assert_eq!(product_label(0x0200), "DT-H70B/PCIE (unverified model)");
        assert_eq!(product_label(0x0218), "DT-H30/PCIEM-TS (unverified model)");
    }

    #[test]
    fn dth10_u7_is_supported_but_unverified() {
        assert_eq!(product_name(0x017B), Some("DT-H10/U7"));
        assert!(is_unverified(0x017B));
        assert_eq!(product_label(0x017B), "DT-H10/U7 (unverified model)");
    }

    #[test]
    fn inf_models_are_not_marked() {
        for &(pid, name) in &SUPPORTED_PIDS[..SUPPORTED_PIDS.len() - UNVERIFIED_PIDS.len()] {
            assert!(!is_unverified(pid), "{name}");
            assert_eq!(product_label(pid), name);
        }
        assert_eq!(product_label(0x0199), "DT-H70/U2");
        assert_eq!(product_label(0x1234), "?");
        assert_eq!(product_name(0x1234), None);
    }

    #[test]
    fn every_unverified_pid_has_a_name() {
        for &pid in UNVERIFIED_PIDS {
            assert!(product_name(pid).is_some(), "0x{pid:04x}");
        }
    }

    #[test]
    fn kit_error_of_an_unverified_model_asks_for_the_dump() {
        let e = unverified_kit_error(0x01FF, 0, Error::Unsupported("terrestrial tuner kit X".into())).to_string();
        assert!(e.contains("terrestrial tuner kit X"), "{e}");
        assert!(e.contains("DT-H71/U2 is an unverified model"), "{e}");
        assert!(e.contains("`dth10-diag info --raw`"), "{e}");
        let e = unverified_kit_error(0x01FF, 1, Error::Unsupported("satellite tuner kit Y".into())).to_string();
        assert!(e.contains("`dth10-diag --tuner 1 info --raw`"), "{e}");
        // Another model's error and other errors stay as they are.
        let e = unverified_kit_error(0x0199, 0, Error::Unsupported("kit".into())).to_string();
        assert_eq!(e, "unsupported: kit");
        std::assert_matches!(unverified_kit_error(0x0202, 0, Error::Timeout), Error::Timeout);
    }
}
