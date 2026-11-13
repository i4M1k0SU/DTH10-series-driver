//! The MTV600 satellite tuner of the TC90522 / TC90532 satellite kits, as the
//! two vendor drivers program it: 2-byte register writes `{reg, value}`
//! behind the demodulator's I2C repeater (or directly on the bridge bus when
//! the descriptor's flags bit3 is set), and register reads (write `{reg}`,
//! read one byte). The reference is 32 MHz; the PLL word is
//! `V = (f / 32000) << 20 + ((f % 32000) << 16) / 2000` (f in kHz).
//!
//! The two drivers differ:
//!
//! * [`VendorDriver::Series`]: its own init table ([`INIT_SERIES`]); a tune
//!   writes `0x24 = V >> 7` and `0x25 = V << 1 | V[29]`, the divider (0x23
//!   bits 7:6) is 2 / 1 / 0 for (900, 1100) / [1100, 1650) / [1650, 2200) MHz.
//!   No sleep or wake.
//! * [`VendorDriver::Dth30`]: its own init table ([`INIT_DTH30`]) and 0x26
//!   bit0 from the descriptor's tuner option bit1; the first tune after the
//!   init rewrites 0x26 / 0x23 / 0x22; above 1280 MHz the word is halved
//!   (0x22 bit0 = 1, 0x26 bits 3:1 = 0, else 0 / 1); 0x24 is not written and
//!   `0x25 = V[29]`; the divider changes at 1175 / 1750 MHz. Sleep / wake set /
//!   clear 0x23 bit0.

use core::time::Duration;
use std::thread::sleep;

use crate::device::Device;
use crate::error::{Error, Result};
use crate::kit::VendorDriver;
use crate::protocol::UsbControl;
use crate::tlv::TunerDesc;
use crate::tuner_bus::TunerBus;

/// The init table of the series driver.
pub const INIT_SERIES: [(u8, u8); 36] = [
    (0x20, 0x0B),
    (0x21, 0xB8),
    (0x22, 0xA8),
    (0x23, 0x42),
    (0x24, 0x00),
    (0x25, 0x00),
    (0x26, 0x83),
    (0x27, 0x3B),
    (0x28, 0xE2),
    (0x29, 0xDC),
    (0x2A, 0x02),
    (0x2B, 0x00),
    (0x2C, 0x00),
    (0x2D, 0x2F),
    (0x2E, 0x21),
    (0x2F, 0x80),
    (0x30, 0x10),
    (0x31, 0x8C),
    (0x32, 0x16),
    (0x33, 0x34),
    (0x34, 0x30),
    (0x35, 0x4A),
    (0x36, 0x81),
    (0x37, 0x92),
    (0x38, 0x90),
    (0x39, 0x18),
    (0x3A, 0x00),
    (0x3B, 0x44),
    (0x3C, 0x92),
    (0x3D, 0x80),
    (0x3E, 0x00),
    (0x3F, 0x00),
    (0x40, 0xFF),
    (0x41, 0x57),
    (0x42, 0x9A),
    (0x43, 0x04),
];

/// The init table of the DT-H30/PCIEM-TS driver: the series table with
/// 0x22, 0x24, 0x26, 0x27, 0x29, 0x2E and 0x3D changed.
pub const INIT_DTH30: [(u8, u8); 36] = [
    (0x20, 0x0B),
    (0x21, 0xB8),
    (0x22, 0x09),
    (0x23, 0x42),
    (0x24, 0x20),
    (0x25, 0x00),
    (0x26, 0x81),
    (0x27, 0x20),
    (0x28, 0xE2),
    (0x29, 0xD4),
    (0x2A, 0x02),
    (0x2B, 0x00),
    (0x2C, 0x00),
    (0x2D, 0x2F),
    (0x2E, 0x10),
    (0x2F, 0x80),
    (0x30, 0x10),
    (0x31, 0x8C),
    (0x32, 0x16),
    (0x33, 0x34),
    (0x34, 0x30),
    (0x35, 0x4A),
    (0x36, 0x81),
    (0x37, 0x92),
    (0x38, 0x90),
    (0x39, 0x18),
    (0x3A, 0x00),
    (0x3B, 0x44),
    (0x3C, 0x92),
    (0x3D, 0x85),
    (0x3E, 0x00),
    (0x3F, 0x00),
    (0x40, 0xFF),
    (0x41, 0x57),
    (0x42, 0x9A),
    (0x43, 0x04),
];

/// How far the tuner is set up: the init's settings, or (DT-H30/PCIEM-TS
/// driver) the ones its first tune writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Setup {
    None,
    Init,
    Tuned,
}

/// The PLL word for `khz`, with the halving of the DT-H30/PCIEM-TS driver
/// above 1280 MHz when `halve_high` is set.
pub fn pll_word(khz: u32, halve_high: bool) -> u32 {
    let (q, r) = (khz / 32_000, khz % 32_000);
    let v = (q << 20) + (r << 16) / 2000;
    if halve_high && q >= 40 { v >> 1 } else { v }
}

/// The divider (0x23 bits 7:6) for `khz`; `None` outside the tuner's range.
pub fn divider(driver: VendorDriver, khz: u32) -> Option<u8> {
    let (lo, hi) = match driver {
        VendorDriver::Series => (1_100_000, 1_650_000),
        VendorDriver::Dth30 => (1_175_000, 1_750_000),
    };
    match khz {
        k if k > 900_000 && k < lo => Some(2),
        k if k >= lo && k < hi => Some(1),
        k if k >= hi && k < 2_200_000 => Some(0),
        _ => None,
    }
}

/// The MTV600 tuner.
#[derive(Debug, Clone, Copy)]
pub struct Mtv600 {
    pub bus: TunerBus,
    pub driver: VendorDriver,
    setup: Setup,
}

impl Mtv600 {
    pub fn new(bus: TunerBus, driver: VendorDriver) -> Self {
        Mtv600 { bus, driver, setup: Setup::None }
    }

    fn write<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8, v: u8) -> Result<()> {
        self.bus.write(dev, &[reg, v])
    }

    /// Register read: `{reg}`, then one byte.
    pub fn read<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8) -> Result<u8> {
        self.bus.write(dev, &[reg])?;
        let mut b = [0u8; 1];
        self.bus.read(dev, &mut b)?;
        Ok(b[0])
    }

    /// Read-modify-write: the bits of `mask` from `val`.
    fn update<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8, mask: u8, val: u8) -> Result<()> {
        let v = self.read(dev, reg)?;
        self.write(dev, reg, (mask | v) & (val | !mask))
    }

    /// The settings after the table (also redone by a series tune when the
    /// init did not get that far): `0x26 &= 3F`, 0x23 bits 3:2 = 2, 0x22 bits
    /// 4:3 = 2.
    fn base_setup<T: UsbControl>(&mut self, dev: &mut Device<T>) -> Result<()> {
        let v = self.read(dev, 0x26)?;
        self.write(dev, 0x26, v & 0x3F)?;
        self.update(dev, 0x23, 0x0C, 0x08)?;
        self.update(dev, 0x22, 0x18, 0x10)?;
        self.setup = Setup::Init;
        Ok(())
    }

    /// Init: the table (the first failed write ends it), the base settings,
    /// and with the DT-H30/PCIEM-TS driver 0x26 bit0 = !(tuner option bit1).
    pub fn init<T: UsbControl>(&mut self, dev: &mut Device<T>, desc: &TunerDesc) -> Result<()> {
        let table: &[(u8, u8)] = match self.driver {
            VendorDriver::Series => &INIT_SERIES,
            VendorDriver::Dth30 => &INIT_DTH30,
        };
        for &(r, v) in table {
            self.write(dev, r, v)?;
        }
        self.base_setup(dev)?;
        if self.driver == VendorDriver::Dth30 {
            let v = self.read(dev, 0x26)?;
            let v = if desc.tuner_opts & 0x02 != 0 { v & 0xFE } else { v | 0x01 };
            self.write(dev, 0x26, v)?;
        }
        Ok(())
    }

    /// Sleep (DT-H30/PCIEM-TS driver only): `0x23 |= 1`.
    pub fn sleep<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        let v = self.read(dev, 0x23)?;
        self.write(dev, 0x23, v | 0x01)
    }

    /// Wake (DT-H30/PCIEM-TS driver only): `0x23 &= ~1`.
    pub fn wake<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        let v = self.read(dev, 0x23)?;
        self.write(dev, 0x23, v & !0x01)
    }

    /// Tune to `khz` (the BS/CS IF in kHz), then 1 ms.
    pub fn tune<T: UsbControl>(&mut self, dev: &mut Device<T>, khz: u32) -> Result<()> {
        match self.driver {
            VendorDriver::Series => self.tune_series(dev, khz),
            VendorDriver::Dth30 => self.tune_dth30(dev, khz),
        }
    }

    fn out_of_range(khz: u32) -> Error {
        Error::Unsupported(format!("MTV600 IF {khz} kHz is outside 900-2200 MHz"))
    }

    fn tune_series<T: UsbControl>(&mut self, dev: &mut Device<T>, khz: u32) -> Result<()> {
        if self.setup != Setup::Init {
            self.base_setup(dev)?;
        }
        let d = divider(VendorDriver::Series, khz).ok_or_else(|| Self::out_of_range(khz))?;
        self.update(dev, 0x23, 0xC0, d << 6)?;
        let v = pll_word(khz, false);
        self.write(dev, 0x24, (v >> 7) as u8)?;
        self.write(dev, 0x25, ((v as u8) << 1) | ((v >> 29) & 1) as u8)?;
        self.update(dev, 0x31, 0x01, ((v >> 28) & 1) as u8)?;
        self.update(dev, 0x20, 0x1F, ((v >> 23) & 0x1F) as u8)?;
        self.write(dev, 0x21, (v >> 15) as u8)?;
        sleep(Duration::from_millis(1));
        Ok(())
    }

    fn tune_dth30<T: UsbControl>(&mut self, dev: &mut Device<T>, khz: u32) -> Result<()> {
        if self.setup != Setup::Tuned {
            // A failed read leaves the byte the driver passes in its buffer (0x28).
            let v = self.read(dev, 0x26).unwrap_or(0x28);
            let r = (|| {
                self.write(dev, 0x26, (v & 0x3F) | 0x80)?;
                self.update(dev, 0x23, 0x0C, 0x00)?;
                self.update(dev, 0x22, 0x18, 0x08)
            })();
            self.setup = Setup::Tuned;
            r?;
        }
        let d = divider(VendorDriver::Dth30, khz).ok_or_else(|| Self::out_of_range(khz))?;
        self.update(dev, 0x23, 0xC0, d << 6)?;
        let high = khz / 32_000 >= 40;
        self.update(dev, 0x22, 0x01, high as u8)?;
        self.update(dev, 0x26, 0x0E, if high { 0 } else { 2 })?;
        let v = pll_word(khz, true);
        self.write(dev, 0x25, ((v >> 29) & 1) as u8)?;
        self.update(dev, 0x31, 0x01, ((v >> 28) & 1) as u8)?;
        self.update(dev, 0x20, 0x1F, ((v >> 23) & 0x1F) as u8)?;
        self.write(dev, 0x21, (v >> 15) as u8)?;
        sleep(Duration::from_millis(1));
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::cmd;
    use crate::tuner_bus::TunerPath;
    use core::cell::{Cell, RefCell};
    use std::collections::HashMap;

    /// A tuner register file directly on the bus at 0x60: raw writes
    /// `{reg, value...}` store, `{reg}` sets the read pointer.
    #[derive(Default)]
    struct Regs {
        regs: RefCell<HashMap<u8, u8>>,
        ptr: Cell<u8>,
        writes: RefCell<Vec<(u8, u8)>>,
    }

    impl Regs {
        fn with(init: &[(u8, u8)]) -> Self {
            let r = Regs::default();
            r.regs.borrow_mut().extend(init.iter().copied());
            r
        }
        fn take(&self) -> Vec<(u8, u8)> {
            std::mem::take(&mut *self.writes.borrow_mut())
        }
    }

    impl UsbControl for Regs {
        fn control_out(&self, _: u8, value: u16, index: u16, data: &[u8], _: Duration) -> Result<usize> {
            assert_eq!((value, index), (cmd::I2C_RAW | 0x60, cmd::I2C_RAW_WRITE));
            self.ptr.set(data[0]);
            for (k, &b) in data[1..].iter().enumerate() {
                let r = data[0] + k as u8;
                self.regs.borrow_mut().insert(r, b);
                self.writes.borrow_mut().push((r, b));
            }
            Ok(data.len())
        }
        fn control_in(&self, _: u8, value: u16, index: u16, buf: &mut [u8], _: Duration) -> Result<usize> {
            assert_eq!((value, index), (cmd::I2C_RAW | 0x60, cmd::I2C_RAW_READ));
            buf[0] = *self.regs.borrow().get(&self.ptr.get()).unwrap_or(&0);
            Ok(buf.len())
        }
    }

    const REGS: [(u8, u8); 5] = [(0x20, 0xE0), (0x22, 0xA6), (0x23, 0x3F), (0x26, 0x55), (0x31, 0xF0)];

    fn tuner(driver: VendorDriver, setup: Setup) -> (Mtv600, Device<Regs>) {
        let bus = TunerBus { tuner_addr: 0x60, path: TunerPath::Direct };
        (Mtv600 { bus, driver, setup }, Device::new(Regs::with(&REGS)))
    }

    #[test]
    fn tables_differ_in_seven_registers() {
        let d: Vec<u8> = INIT_SERIES.iter().zip(INIT_DTH30).filter(|(a, b)| **a != *b).map(|(a, _)| a.0).collect();
        assert_eq!(d, [0x22, 0x24, 0x26, 0x27, 0x29, 0x2E, 0x3D]);
        assert!(INIT_SERIES.iter().zip(INIT_DTH30).all(|(a, b)| a.0 == b.0));
    }

    #[test]
    fn init_sequences() {
        for (driver, opts) in [(VendorDriver::Series, 0), (VendorDriver::Dth30, 0), (VendorDriver::Dth30, 2)] {
            let (mut t, mut dev) = tuner(driver, Setup::None);
            let desc = TunerDesc { tuner_opts: opts, ..Default::default() };
            t.init(&mut dev, &desc).unwrap();
            let w = dev.transport().take();
            let table = if driver == VendorDriver::Series { INIT_SERIES } else { INIT_DTH30 };
            assert_eq!(w[..36], table);
            // 0x26 (table value) & 3F, 0x23 = 0x42 with bits 3:2 = 2, 0x22 with bits 4:3 = 2
            let t26 = table[6].1 & 0x3F;
            let mut rest = vec![(0x26, t26), (0x23, 0x4A), (0x22, (table[2].1 & !0x18) | 0x10)];
            if driver == VendorDriver::Dth30 {
                rest.push((0x26, if opts & 2 != 0 { t26 & 0xFE } else { t26 | 1 }));
            }
            assert_eq!(w[36..], rest, "{driver:?} opts {opts}");
            assert_eq!(t.setup, Setup::Init);
        }
    }

    #[test]
    fn divider_band_edges() {
        for (khz, s, h) in [
            (900_000, None, None),
            (900_001, Some(2), Some(2)),
            (1_099_999, Some(2), Some(2)),
            (1_100_000, Some(1), Some(2)),
            (1_174_999, Some(1), Some(2)),
            (1_175_000, Some(1), Some(1)),
            (1_649_999, Some(1), Some(1)),
            (1_650_000, Some(0), Some(1)),
            (1_749_999, Some(0), Some(1)),
            (1_750_000, Some(0), Some(0)),
            (2_199_999, Some(0), Some(0)),
            (2_200_000, None, None),
        ] {
            assert_eq!(divider(VendorDriver::Series, khz), s, "{khz}");
            assert_eq!(divider(VendorDriver::Dth30, khz), h, "{khz}");
        }
    }

    #[test]
    fn pll_words() {
        // (kHz, series word, DT-H30 word)
        for (khz, s, h) in [
            (1_049_480, 0x20C_BD70, 0x20C_BD70),
            (1_318_000, 0x293_0000, 0x149_8000),
            (2_053_000, 0x402_8000, 0x201_4000),
            (1_279_999, 0x27F_FFDF, 0x27F_FFDF),
            (1_280_000, 0x280_0000, 0x140_0000),
            (900_001, 0x1C2_0020, 0x1C2_0020),
            (2_199_999, 0x44B_FFDF, 0x225_FFEF),
        ] {
            assert_eq!(pll_word(khz, false), s, "{khz}");
            assert_eq!(pll_word(khz, true), h, "{khz}");
        }
    }

    #[test]
    fn series_tune_writes() {
        for (khz, w) in [
            (1_049_480, [(0x23, 0xBF), (0x24, 0x7A), (0x25, 0xE0), (0x31, 0xF0), (0x20, 0xE4), (0x21, 0x19)]),
            (1_318_000, [(0x23, 0x7F), (0x24, 0x00), (0x25, 0x00), (0x31, 0xF0), (0x20, 0xE5), (0x21, 0x26)]),
            (2_053_000, [(0x23, 0x3F), (0x24, 0x00), (0x25, 0x00), (0x31, 0xF0), (0x20, 0xE8), (0x21, 0x05)]),
            (1_099_999, [(0x23, 0xBF), (0x24, 0xFF), (0x25, 0xBE), (0x31, 0xF0), (0x20, 0xE4), (0x21, 0x4B)]),
            (1_100_000, [(0x23, 0x7F), (0x24, 0x00), (0x25, 0x00), (0x31, 0xF0), (0x20, 0xE4), (0x21, 0x4C)]),
            (1_649_999, [(0x23, 0x7F), (0x24, 0xFF), (0x25, 0xBE), (0x31, 0xF0), (0x20, 0xE6), (0x21, 0x71)]),
            (1_650_000, [(0x23, 0x3F), (0x24, 0x00), (0x25, 0x00), (0x31, 0xF0), (0x20, 0xE6), (0x21, 0x72)]),
            (900_001, [(0x23, 0xBF), (0x24, 0x00), (0x25, 0x40), (0x31, 0xF0), (0x20, 0xE3), (0x21, 0x84)]),
        ] {
            let (mut t, mut dev) = tuner(VendorDriver::Series, Setup::Init);
            t.tune(&mut dev, khz).unwrap();
            assert_eq!(dev.transport().take(), w, "{khz}");
        }
        // without the init's settings the tune makes them first
        let (mut t, mut dev) = tuner(VendorDriver::Series, Setup::None);
        t.tune(&mut dev, 1_049_480).unwrap();
        assert_eq!(dev.transport().take()[..3], [(0x26, 0x15), (0x23, 0x3B), (0x22, 0xB6)]);
        assert_eq!(t.setup, Setup::Init);
        for khz in [900_000, 2_200_000] {
            let (mut t, mut dev) = tuner(VendorDriver::Series, Setup::Init);
            std::assert_matches!(t.tune(&mut dev, khz), Err(Error::Unsupported(_)));
            assert_eq!(dev.transport().take(), []);
        }
    }

    #[test]
    fn dth30_tune_writes() {
        let first = [(0x26, 0x95), (0x23, 0x33), (0x22, 0xAE)];
        for (khz, w) in [
            (
                1_049_480,
                [(0x23, 0xB3), (0x22, 0xAE), (0x26, 0x93), (0x25, 0x00), (0x31, 0xF0), (0x20, 0xE4), (0x21, 0x19)],
            ),
            (
                1_318_000,
                [(0x23, 0x73), (0x22, 0xAF), (0x26, 0x91), (0x25, 0x00), (0x31, 0xF0), (0x20, 0xE2), (0x21, 0x93)],
            ),
            (
                2_053_000,
                [(0x23, 0x33), (0x22, 0xAF), (0x26, 0x91), (0x25, 0x00), (0x31, 0xF0), (0x20, 0xE4), (0x21, 0x02)],
            ),
            (
                1_174_999,
                [(0x23, 0xB3), (0x22, 0xAE), (0x26, 0x93), (0x25, 0x00), (0x31, 0xF0), (0x20, 0xE4), (0x21, 0x96)],
            ),
            (
                1_175_000,
                [(0x23, 0x73), (0x22, 0xAE), (0x26, 0x93), (0x25, 0x00), (0x31, 0xF0), (0x20, 0xE4), (0x21, 0x97)],
            ),
            (
                1_279_999,
                [(0x23, 0x73), (0x22, 0xAE), (0x26, 0x93), (0x25, 0x00), (0x31, 0xF0), (0x20, 0xE4), (0x21, 0xFF)],
            ),
            (
                1_280_000,
                [(0x23, 0x73), (0x22, 0xAF), (0x26, 0x91), (0x25, 0x00), (0x31, 0xF0), (0x20, 0xE2), (0x21, 0x80)],
            ),
            (
                1_749_999,
                [(0x23, 0x73), (0x22, 0xAF), (0x26, 0x91), (0x25, 0x00), (0x31, 0xF0), (0x20, 0xE3), (0x21, 0x6A)],
            ),
            (
                1_750_000,
                [(0x23, 0x33), (0x22, 0xAF), (0x26, 0x91), (0x25, 0x00), (0x31, 0xF0), (0x20, 0xE3), (0x21, 0x6B)],
            ),
        ] {
            // the first tune after the init rewrites 0x26 / 0x23 / 0x22 first
            let (mut t, mut dev) = tuner(VendorDriver::Dth30, Setup::Init);
            t.tune(&mut dev, khz).unwrap();
            let all = dev.transport().take();
            assert_eq!(all[..3], first, "{khz}");
            assert_eq!(all[3..], w, "{khz}");
            assert_eq!(t.setup, Setup::Tuned);
        }
        // later tunes do not
        let (mut t, mut dev) = tuner(VendorDriver::Dth30, Setup::Tuned);
        t.tune(&mut dev, 1_049_480).unwrap();
        assert_eq!(
            dev.transport().take(),
            [(0x23, 0xBF), (0x22, 0xA6), (0x26, 0x53), (0x25, 0x00), (0x31, 0xF0), (0x20, 0xE4), (0x21, 0x19)]
        );
        // out of range: only the first-tune writes
        let (mut t, mut dev) = tuner(VendorDriver::Dth30, Setup::Init);
        assert!(t.tune(&mut dev, 2_200_000).is_err());
        assert_eq!(dev.transport().take(), first);
    }

    #[test]
    fn sleep_and_wake_toggle_0x23_bit0() {
        let (t, mut dev) = tuner(VendorDriver::Dth30, Setup::Init);
        t.sleep(&mut dev).unwrap();
        t.wake(&mut dev).unwrap();
        assert_eq!(dev.transport().take(), [(0x23, 0x3F), (0x23, 0x3E)]);
    }
}
