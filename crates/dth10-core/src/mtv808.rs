//! The MTV808 terrestrial (ISDB-T) tuner, as the vendor drivers program it in
//! the TC90522 / TC90532 hw 0x102 kits and the `TC90527_MTV808` kit: two-byte
//! register writes and one-byte register reads (a register-pointer write, then
//! a read) through the demodulator's I2C repeater, or directly on the bridge
//! bus (the kit's [`TunerBus`]). Its reference is 32 MHz whatever the
//! descriptor says; frequencies are in kHz (the tune request's Hz / 1000).
//!
//! * Init: the init table, a successive-approximation calibration of register
//!   0x8B (six steps of 10 ms, judged by register 0x0A bit7), the IF mode
//!   registers, then the filter registers 0x3D..0x4A compensated by the chip
//!   temperature (register 0x12).
//! * Tune: one of two filter tables (up to / above 325 MHz), registers 0x61,
//!   0x34 / 0x35 (by a band table), 0x51 / 0x53, the divider word to
//!   0x26..0x2A, 1 ms, then the VCO setting in 0x51 by the lock reading in
//!   register 0x06.
//! * Signal strength from registers 3 / 4 / 5.
//!
//! The two vendor drivers ([`VendorDriver`]) have their own tables, band
//! edges, VCO codes and strength formulas, and differ in which errors they
//! look at; the DT-H30/PCIEM-TS driver also has a tuner wake / standby
//! (register 0x33 bit6), the series driver none.

use core::time::Duration;
use std::thread::sleep;

use crate::device::Device;
use crate::error::{Error, Result};
use crate::kit::{SignalStatus, VendorDriver, field};
use crate::protocol::UsbControl;
use crate::tlv::TunerDesc;
use crate::tuner_bus::TunerBus;

/// Init table of the series driver.
pub const INIT_TABLE_SERIES: [(u8, u8); 43] = [
    (0x23, 0x18),
    (0x2B, 0x08),
    (0x33, 0xA0),
    (0x34, 0x7F),
    (0x35, 0x00),
    (0x36, 0x00),
    (0x3E, 0xFF),
    (0x4D, 0x8C),
    (0x4F, 0x8B),
    (0x50, 0x09),
    (0x51, 0xC9),
    (0x52, 0x92),
    (0x54, 0x41),
    (0x55, 0x08),
    (0x56, 0x00),
    (0x58, 0xDF),
    (0x5A, 0x04),
    (0x5C, 0x02),
    (0x5D, 0x1C),
    (0x5E, 0x2A),
    (0x5F, 0x32),
    (0x60, 0x19),
    (0x61, 0x50),
    (0x62, 0x51),
    (0x63, 0xD4),
    (0x65, 0xFF),
    (0x66, 0xFF),
    (0x68, 0x90),
    (0x69, 0x74),
    (0x6A, 0x64),
    (0x6B, 0x01),
    (0x6C, 0x21),
    (0x75, 0x07),
    (0x78, 0x20),
    (0x79, 0xFF),
    (0x7A, 0xF0),
    (0x7B, 0xB0),
    (0x7C, 0xF0),
    (0x7E, 0x26),
    (0x87, 0xC0),
    (0x8B, 0x13),
    (0x8D, 0xE8),
    (0x91, 0x32),
];

/// Init table of the DT-H30/PCIEM-TS driver (`0x33`, `0x54`, `0x60`, `0x63`,
/// `0x7B` differ from the series driver's; `0x64` and `0x81` are added).
pub const INIT_TABLE_DTH30: [(u8, u8); 45] = [
    (0x23, 0x18),
    (0x2B, 0x08),
    (0x33, 0x00),
    (0x34, 0x7F),
    (0x35, 0x00),
    (0x36, 0x00),
    (0x3E, 0xFF),
    (0x4D, 0x8C),
    (0x4F, 0x8B),
    (0x50, 0x09),
    (0x51, 0xC9),
    (0x52, 0x92),
    (0x54, 0xC1),
    (0x55, 0x08),
    (0x56, 0x00),
    (0x58, 0xDF),
    (0x5A, 0x04),
    (0x5C, 0x02),
    (0x5D, 0x1C),
    (0x5E, 0x2A),
    (0x5F, 0x32),
    (0x60, 0x4C),
    (0x61, 0x50),
    (0x62, 0x51),
    (0x63, 0xDB),
    (0x64, 0xF5),
    (0x65, 0xFF),
    (0x66, 0xFF),
    (0x68, 0x90),
    (0x69, 0x74),
    (0x6A, 0x64),
    (0x6B, 0x01),
    (0x6C, 0x21),
    (0x75, 0x07),
    (0x78, 0x20),
    (0x79, 0xFF),
    (0x7A, 0xF0),
    (0x7B, 0xD0),
    (0x7C, 0xF0),
    (0x7E, 0x26),
    (0x81, 0x08),
    (0x87, 0xC0),
    (0x8B, 0x13),
    (0x8D, 0xE8),
    (0x91, 0x32),
];

/// The filter table for tuning up to 325 MHz, series driver. The DT-H30/PCIEM-TS
/// driver's differs in `0x54` only.
const LOW_TABLE_SERIES: [(u8, u8); 14] = [
    (0x22, 0x00),
    (0x34, 0x1A),
    (0x35, 0xFF),
    (0x36, 0x5F),
    (0x3E, 0xE0),
    (0x4D, 0xCC),
    (0x4E, 0x20),
    (0x50, 0x00),
    (0x51, 0x98),
    (0x54, 0x40),
    (0x6C, 0x20),
    (0x72, 0x80),
    (0x75, 0x00),
    (0x7F, 0xFF),
];
const LOW_TABLE_DTH30: [(u8, u8); 14] = [
    (0x22, 0x00),
    (0x34, 0x1A),
    (0x35, 0xFF),
    (0x36, 0x5F),
    (0x3E, 0xE0),
    (0x4D, 0xCC),
    (0x4E, 0x20),
    (0x50, 0x00),
    (0x51, 0x98),
    (0x54, 0xC1),
    (0x6C, 0x20),
    (0x72, 0x80),
    (0x75, 0x00),
    (0x7F, 0xFF),
];
/// The filter table for tuning above 325 MHz, series driver (the DT-H30/PCIEM-TS
/// driver's differs in `0x54` only).
const HIGH_TABLE_SERIES: [(u8, u8); 14] = [
    (0x22, 0x80),
    (0x34, 0x7F),
    (0x35, 0x00),
    (0x36, 0x00),
    (0x3E, 0xFF),
    (0x4D, 0x8C),
    (0x4E, 0x08),
    (0x50, 0x09),
    (0x51, 0xC9),
    (0x54, 0x41),
    (0x6C, 0x21),
    (0x72, 0xA0),
    (0x75, 0x07),
    (0x7F, 0xF6),
];
const HIGH_TABLE_DTH30: [(u8, u8); 14] = [
    (0x22, 0x80),
    (0x34, 0x7F),
    (0x35, 0x00),
    (0x36, 0x00),
    (0x3E, 0xFF),
    (0x4D, 0x8C),
    (0x4E, 0x08),
    (0x50, 0x09),
    (0x51, 0xC9),
    (0x54, 0xC1),
    (0x6C, 0x21),
    (0x72, 0xA0),
    (0x75, 0x07),
    (0x7F, 0xF6),
];

/// Values for registers 0x34 / 0x35 (bit7 of each kept) by frequency: the
/// first entry whose upper edge (kHz) is at or above the tuned frequency;
/// nothing is written above the last edge.
const BAND_TABLE_DTH30: [(u32, u8, u8); 36] = [
    (57_000, 0x00, 0x50),
    (63_000, 0x00, 0x50),
    (69_000, 0x00, 0x50),
    (75_000, 0x00, 0x50),
    (81_000, 0x00, 0x50),
    (90_000, 0x00, 0x50),
    (100_000, 0x00, 0x50),
    (110_000, 0x00, 0x50),
    (120_000, 0x00, 0x50),
    (130_000, 0x00, 0x50),
    (143_000, 0x00, 0x50),
    (150_000, 0x00, 0x40),
    (160_000, 0x00, 0x40),
    (170_000, 0x00, 0x40),
    (180_000, 0x00, 0x40),
    (190_000, 0x00, 0x40),
    (200_000, 0x00, 0x40),
    (214_000, 0x00, 0x40),
    (230_000, 0x10, 0x40),
    (235_000, 0x10, 0x40),
    (245_000, 0x10, 0x40),
    (250_000, 0x10, 0x40),
    (256_000, 0x10, 0x40),
    (265_000, 0x10, 0x40),
    (275_000, 0x10, 0x40),
    (295_000, 0x10, 0x40),
    (305_000, 0x10, 0x40),
    (320_000, 0x10, 0x40),
    (325_000, 0x10, 0x40),
    (340_000, 0x4E, 0x15),
    (360_000, 0x7F, 0x15),
    (370_000, 0x7F, 0x05),
    (400_000, 0x7F, 0x17),
    (425_000, 0x20, 0x14),
    (470_000, 0x20, 0x10),
    (850_000, 0x7F, 0x00),
];
/// The series driver's band table has a single entry with edge 0: it writes
/// registers 0x34 / 0x35 only for a frequency of 0 kHz, that is never for a
/// frequency it can tune.
const BAND_TABLE_SERIES: [(u32, u8, u8); 1] = [(0, 0x00, 0x00)];

/// The temperature-compensated filter values (both drivers): base, slope per
/// temperature step (×1/32) and upper limit of each of the 14 values, of which
/// values 0 and 1 are not compensated.
const COMP_BASE: [u32; 14] = [2, 2, 65, 65, 65, 120, 62, 120, 60, 117, 109, 26, 39, 9];
const COMP_SLOPE: [u32; 14] = [0, 0, 90, 77, 77, 78, 40, 118, 48, 75, 70, 53, 25, 6];
const COMP_MAX: [i32; 14] = [0, 0, 127, 127, 127, 255, 127, 255, 127, 255, 511, 127, 127, 31];

/// What the two drivers do differently.
struct Profile {
    init: &'static [(u8, u8)],
    low: &'static [(u8, u8); 14],
    high: &'static [(u8, u8); 14],
    bands: &'static [(u32, u8, u8)],
    /// Lower band edges (kHz) of the divider ×1 / ×4 / ×8 bands, and the upper
    /// limit of the tuning range.
    edges: [u32; 4],
    /// Register 0x61 is 0x50 above this frequency (kHz), else 0x63.
    r61_edge: u32,
    /// VCO code in register 0x51 bits 5:3 before the lock reading, and after
    /// it for register 0x06 up to 0x1F / up to 0x7F / above.
    vco: [u8; 4],
}

static SERIES: Profile = Profile {
    init: &INIT_TABLE_SERIES,
    low: &LOW_TABLE_SERIES,
    high: &HIGH_TABLE_SERIES,
    bands: &BAND_TABLE_SERIES,
    edges: [928_000, 464_000, 232_000, 116_000],
    r61_edge: 460_000,
    vco: [5, 5, 4, 2],
};

static DTH30: Profile = Profile {
    init: &INIT_TABLE_DTH30,
    low: &LOW_TABLE_DTH30,
    high: &HIGH_TABLE_DTH30,
    bands: &BAND_TABLE_DTH30,
    edges: [912_000, 456_000, 228_000, 114_000],
    r61_edge: 456_000,
    vco: [7, 7, 6, 4],
};

fn profile(driver: VendorDriver) -> &'static Profile {
    match driver {
        VendorDriver::Series => &SERIES,
        VendorDriver::Dth30 => &DTH30,
    }
}

/// The divider band (4..=7) and the 32-bit divider word for `khz`:
/// `fM = f · M` (M = 1 / 4 / 8 / 16 by band), `V = ((fM / 32000) << 20) +
/// ((fM % 32000) << 16) / 2000` (a 32 MHz reference). `None` outside the
/// tuning range (from 50 MHz up to 928 MHz, 912 MHz for the DT-H30/PCIEM-TS
/// driver).
pub fn divider(driver: VendorDriver, khz: u32) -> Option<(u8, u32)> {
    let e = profile(driver).edges;
    let (band, mult) = if khz >= e[0] || khz < 50_000 {
        return None;
    } else if khz >= e[1] {
        (7, 1)
    } else if khz >= e[2] {
        (6, 4)
    } else if khz >= e[3] {
        (5, 8)
    } else {
        (4, 16)
    };
    let fm = khz * mult;
    Some((band, ((fm / 32_000) << 20) + ((fm % 32_000) << 16) / 2_000))
}

/// Registers 0x26..0x2A for `khz`: band, filter selection (3 up to 325 MHz,
/// else 1) and the top bits of the divider word, then the rest of the word
/// (the last register with bit0 set). `None` outside the tuning range.
pub fn divider_registers(driver: VendorDriver, khz: u32) -> Option<[(u8, u8); 5]> {
    let (band, v) = divider(driver, khz)?;
    let sel: u8 = if khz <= 325_000 { 3 } else { 1 };
    Some([
        (0x26, ((sel * 8 + band) * 4).wrapping_add((v >> 28) as u8)),
        (0x27, (v >> 20) as u8),
        (0x28, (v >> 12) as u8),
        (0x29, (v >> 4) as u8),
        (0x2A, ((v << 4) | 1) as u8),
    ])
}

/// Register 0x53 for `khz`: 0x7F in 760-816, 380-408, 190-204 and 95-102 MHz,
/// 0xFF elsewhere (both drivers).
fn reg53(khz: u32) -> u8 {
    match khz {
        760_000..816_000 | 380_000..408_000 | 190_000..204_000 | 95_000..102_000 => 0x7F,
        _ => 0xFF,
    }
}

/// The signal strength value (the kit reports 100 times it) from registers 3
/// and 4 and the 2-bit gain range `lna` (register 5 bits 1:0), with the
/// formulas for a tuner whose filter registers are compensated
/// (`compensated`, the state after a successful Init) or not.
fn strength_value(driver: VendorDriver, compensated: bool, r3: u8, r4: u8, lna: u8) -> i32 {
    let (a, b, c): (f64, f64, f64) = match (driver, lna, compensated) {
        (_, 0, false) => (-3.0, -2.4, 430.0),
        (_, 1, false) => (-5.0, -2.0, 230.0),
        (_, 2, false) => (-6.0, -2.0, 0.0),
        (_, _, false) => (-4.5, -4.0, 90.0),
        (VendorDriver::Series, 0, true) => (-4.2, -2.2, 350.0),
        (VendorDriver::Series, 1, true) => (-5.699999999999999, -2.2, 210.0),
        (VendorDriver::Series, 2, true) => (-7.0, -2.3000000000000003, 0.0),
        (VendorDriver::Series, _, true) => (-4.5, -4.0, 0.0),
        (VendorDriver::Dth30, 0, true) => (-3.5, -2.5, 430.0),
        (VendorDriver::Dth30, 1, true) => (-8.0, -4.0, 530.0),
        (VendorDriver::Dth30, 2, true) => (-8.0, -4.0, 280.0),
        (VendorDriver::Dth30, _, true) => (-4.6000000000000005, -3.9000000000000004, 55.0),
    };
    let x = r3 as f64 * a + r4 as f64 * b;
    // The formulas without a constant add none (adding 0.0 would not change the value).
    (x + c) as i32
}

/// The MTV808 tuner of one kit.
#[derive(Debug, Clone, Copy)]
pub struct Mtv808 {
    pub bus: TunerBus,
    pub driver: VendorDriver,
    /// The filter table in the chip: 0 = up to 325 MHz, 1 = above (Init sets 1
    /// without writing it: the init table is the "above" setting).
    filter: u8,
    /// Whether the compensated filter registers are written.
    compensated: bool,
    /// The IF mode registers in the chip: 0, or 5 for a 4 MHz IF (DT-H30/PCIEM-TS driver).
    mode: u8,
    /// The temperature-compensated filter values of the last Init.
    comp: [u32; 14],
}

impl Mtv808 {
    pub fn new(bus: TunerBus, driver: VendorDriver) -> Self {
        Mtv808 { bus, driver, filter: 0, compensated: false, mode: 0, comp: [0; 14] }
    }

    fn write<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8, val: u8) -> Result<()> {
        self.bus.write(dev, &[reg, val])
    }

    fn write_all<T: UsbControl>(&self, dev: &mut Device<T>, regs: &[(u8, u8)]) -> Result<()> {
        for &(r, v) in regs {
            self.write(dev, r, v)?;
        }
        Ok(())
    }

    /// A register read: the register number, then one byte.
    fn read<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8) -> Result<u8> {
        self.bus.write(dev, &[reg])?;
        let mut b = [0u8; 1];
        self.bus.read(dev, &mut b)?;
        Ok(b[0])
    }

    /// Init, as the kits' Init and the Open of a kit with descriptor flags bit5
    /// do it after the demodulator configuration. The series driver goes on
    /// after a failed init table write and always uses IF mode 0; the
    /// DT-H30/PCIEM-TS driver stops after the calibration when the table
    /// failed and uses IF mode 5 for a 4 MHz IF (`TunerDesc::if_freq_hz`).
    /// Neither looks at the calibration's result.
    pub fn init<T: UsbControl>(&mut self, dev: &mut Device<T>, desc: &TunerDesc) -> Result<()> {
        let p = profile(self.driver);
        self.filter = 1;
        let table = self.write_all(dev, p.init);
        if let Err(e) = self.calibrate(dev) {
            log::debug!("MTV808 calibration: {e}");
        }
        let mode = match self.driver {
            VendorDriver::Series => {
                if let Err(e) = table {
                    log::debug!("MTV808 init table: {e}");
                }
                0
            }
            VendorDriver::Dth30 => {
                table?;
                if desc.if_freq_hz == 4_000_000 { 5 } else { 0 }
            }
        };
        self.set_mode(dev, mode)?;
        self.compensate(dev)
    }

    /// The successive-approximation calibration of register 0x8B: from 0x1F,
    /// for bits 5..0 in turn the bit is set when register 0x0A reads below
    /// 0x80 after 10 ms, and each next trial clears the next lower bit.
    fn calibrate<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.write_all(dev, &[(0x7F, 0x36), (0x80, 0xA0), (0x2F, 0x00), (0x3D, 0xBA)])?;
        let mut v: u8 = 0x1F;
        self.write(dev, 0x8B, v)?;
        for bit in (0..6u8).rev() {
            sleep(Duration::from_millis(10));
            if self.read(dev, 0x0A)? < 0x80 {
                v |= 1 << bit;
            }
            self.write(dev, 0x8B, v)?;
            if bit == 0 {
                break;
            }
            v &= 0x3F - (1 << (bit - 1));
            self.write(dev, 0x8B, v)?;
        }
        self.write_all(dev, &[(0x2F, 0x01), (0x3D, 0x3A), (0x7F, 0xF6), (0x80, 0x20), (0x89, 0x27)])
    }

    /// The IF mode registers: mode 0 = `24=00 30=2E 39=00`, mode 5 =
    /// `25=A2 88=00 24=09 30=0E 39=80 5E=5D 60=4C`.
    fn set_mode<T: UsbControl>(&mut self, dev: &mut Device<T>, mode: u8) -> Result<()> {
        if mode == 0 {
            self.write_all(dev, &[(0x24, 0x00), (0x30, 0x2E), (0x39, 0x00)])?;
        } else {
            self.write_all(
                dev,
                &[(0x25, 0xA2), (0x88, 0x00), (0x24, 0x09), (0x30, 0x0E), (0x39, 0x80), (0x5E, 0x5D), (0x60, 0x4C)],
            )?;
        }
        self.mode = mode;
        Ok(())
    }

    /// The temperature compensation: register 0x12 bits 4:0 give the step `t`
    /// (bit4 set: the byte `-(t & 0xF)`, which the drivers then use as an
    /// unsigned factor, so the values go to their limits), each value is
    /// `base + (t · slope) / 32` up to its limit, and the filter registers are
    /// written from them.
    fn compensate<T: UsbControl>(&mut self, dev: &mut Device<T>) -> Result<()> {
        let mut t = self.read(dev, 0x12)? & 0x1F;
        if t & 0x10 != 0 {
            t = (!(t & 0x0F)).wrapping_add(1);
        }
        let t = t as u32;
        for j in 0..14 {
            self.comp[j] = if j < 2 {
                COMP_BASE[j]
            } else {
                let v = COMP_BASE[j].wrapping_add(COMP_SLOPE[j].wrapping_mul(t) >> 5);
                if v as i32 > COMP_MAX[j] { COMP_MAX[j] as u32 } else { v }
            };
        }
        self.write_comp(dev)?;
        self.compensated = true;
        Ok(())
    }

    /// The filter registers 0x3D, 0x3F..0x4A from the compensated values.
    fn write_comp<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        let c = self.comp;
        self.write(dev, 0x3D, ((c[0] as u8) << 2).wrapping_add(c[1] as u8) | 0x30)?;
        for (reg, j) in [(0x3F, 2), (0x40, 3), (0x41, 4), (0x42, 5), (0x43, 6), (0x44, 7), (0x45, 8), (0x46, 9)] {
            self.write(dev, reg, c[j] as u8)?;
        }
        self.write(dev, 0x47, (((c[10] >> 1) & 0x80) as u8).wrapping_add(c[11] as u8))?;
        self.write(dev, 0x48, c[10] as u8)?;
        self.write(dev, 0x49, c[13] as u8 & 0x1F)?;
        self.write(dev, 0x4A, c[12] as u8)
    }

    /// Select the filter table (0 = up to 325 MHz, 1 = above) unless it is
    /// already set. The table counts as set even when a write failed.
    fn select_filter<T: UsbControl>(&mut self, dev: &mut Device<T>, filter: u8) -> Result<()> {
        if self.filter == filter {
            return Ok(());
        }
        let p = profile(self.driver);
        let r = self.write_all(dev, if filter == 1 { p.high } else { p.low });
        self.filter = filter;
        r
    }

    /// Tune to the request frequency `freq_hz` (its kHz, truncated). Outside
    /// the tuning range ([`divider`]) the steps before the divider are done
    /// and an error is returned, as in the vendor drivers.
    pub fn tune<T: UsbControl>(&mut self, dev: &mut Device<T>, freq_hz: u32) -> Result<()> {
        let khz = freq_hz / 1000;
        let p = profile(self.driver);
        let dth30 = self.driver == VendorDriver::Dth30;
        // Only the DT-H30/PCIEM-TS driver looks at the result of some writes.
        let checked = |r: Result<()>| if dth30 { r } else { Ok(()) };
        if let Err(e) = self.select_filter(dev, if khz <= 325_000 { 0 } else { 1 }) {
            log::debug!("MTV808 filter table: {e}");
        }
        checked(self.write(dev, 0x61, if khz > p.r61_edge { 0x50 } else { 0x63 }))?;
        // The series driver puts the IF mode back to 0; the other driver keeps
        // the one of the Init.
        if !dth30 && self.mode != 0 {
            self.set_mode(dev, 0)?;
        }
        if !self.compensated {
            let r = self.write_comp(dev);
            if r.is_ok() || dth30 {
                self.compensated = true;
            }
            r?;
        }
        if let Some(&(_, b34, b35)) = p.bands.iter().find(|e| e.0 >= khz) {
            let (r34, r35) = if dth30 {
                (self.read(dev, 0x34).unwrap_or(0), self.read(dev, 0x35).unwrap_or(0))
            } else {
                (self.read(dev, 0x34)?, self.read(dev, 0x35)?)
            };
            self.write(dev, 0x34, (r34 & 0x80) | b34)?;
            self.write(dev, 0x35, (r35 & 0x80) | b35)?;
        }
        let regs = divider_registers(self.driver, khz)
            .ok_or_else(|| Error::Unsupported(format!("MTV808 tuner frequency {khz} kHz")))?;
        let keep = self.read(dev, 0x51)? & 0x87;
        checked(self.write(dev, 0x51, keep | (p.vco[0] << 3)))?;
        checked(self.write(dev, 0x53, reg53(khz)))?;
        for (r, v) in regs {
            checked(self.write(dev, r, v))?;
        }
        sleep(Duration::from_millis(1));
        let vco = match self.read(dev, 0x06)? {
            0..=0x1F => p.vco[1],
            0x20..=0x7F => p.vco[2],
            _ => p.vco[3],
        };
        self.write(dev, 0x51, keep | (vco << 3))
    }

    /// Wake (Open without descriptor flags bit5): register 0x33 bit6 cleared
    /// (DT-H30/PCIEM-TS driver); the series driver has no tuner wake.
    pub fn wake<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        if self.driver == VendorDriver::Series {
            return Ok(());
        }
        let v = self.read(dev, 0x33)?;
        self.write(dev, 0x33, v & !0x40)
    }

    /// Standby (end of Init, Close): register 0x33 bit6 set (DT-H30/PCIEM-TS
    /// driver); the series driver has no tuner standby.
    pub fn standby<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        if self.driver == VendorDriver::Series {
            return Ok(());
        }
        let v = self.read(dev, 0x33)?;
        self.write(dev, 0x33, v | 0x40)
    }

    /// The tuner's part of the signal status, after the demodulator's: the
    /// strength is 100 times the value of registers 3 / 4 / 5 (0 when a read
    /// fails, which is not an error) and its field is marked valid (the series
    /// driver also marks the C/N field).
    pub fn status<T: UsbControl>(&mut self, dev: &mut Device<T>, st: &mut SignalStatus) -> Result<()> {
        let regs = (|| Ok::<_, Error>((self.read(dev, 3)?, self.read(dev, 4)?, self.read(dev, 5)?)))();
        let s = match regs {
            Ok((r3, r4, r5)) => strength_value(self.driver, self.compensated, r3, r4, r5 & 3),
            Err(e) => {
                log::debug!("MTV808 strength: {e}");
                0
            }
        };
        st.mask |= match self.driver {
            VendorDriver::Series => field::STRENGTH | field::CNR,
            VendorDriver::Dth30 => field::STRENGTH,
        };
        st.strength = s.wrapping_mul(100);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::cmd;
    use crate::tuner_bus::TunerPath;
    use core::cell::{Cell, RefCell};
    use std::collections::{HashMap, VecDeque};

    const DEMOD: u8 = 0x18;
    const TUNER: u8 = 0x60;

    /// A tuner access as the tuner sees it: a register write, or a register read.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Op {
        W(u8, u8),
        R(u8),
    }
    use Op::{R, W};

    /// A tuner behind the demodulator's repeater: written registers read back,
    /// unless a queued value for the register is pending; one register's
    /// writes or reads can be made to fail.
    #[derive(Default)]
    struct Sim {
        log: RefCell<Vec<Op>>,
        regs: RefCell<HashMap<u8, u8>>,
        queue: RefCell<HashMap<u8, VecDeque<u8>>>,
        ptr: Cell<u8>,
        fail_write: Cell<Option<u8>>,
        fail_read: Cell<Option<u8>>,
    }

    impl Sim {
        fn queue(&self, reg: u8, vals: &[u8]) {
            self.queue.borrow_mut().entry(reg).or_default().extend(vals);
        }
        fn take(&self) -> Vec<Op> {
            std::mem::take(&mut *self.log.borrow_mut())
        }
    }

    fn nak() -> Error {
        Error::I2c { addr: DEMOD, reg: None, msg: "nak" }
    }

    impl UsbControl for Sim {
        fn control_out(&self, _: u8, value: u16, index: u16, data: &[u8], _: Duration) -> Result<usize> {
            if value == cmd::I2C_RAW | DEMOD as u16 && index == cmd::I2C_RAW_WRITE {
                assert_eq!(data[0], 0xFE, "{data:02x?}");
                if data[1] == (TUNER << 1) | 1 {
                    return Ok(data.len());
                }
                assert_eq!(data[1], TUNER << 1, "{data:02x?}");
                match data[2..] {
                    [reg] => self.ptr.set(reg),
                    [reg, val] => {
                        self.log.borrow_mut().push(W(reg, val));
                        if self.fail_write.get() == Some(reg) {
                            return Err(nak());
                        }
                        self.regs.borrow_mut().insert(reg, val);
                    }
                    _ => panic!("tuner write {data:02x?}"),
                }
            }
            Ok(data.len())
        }

        fn control_in(&self, _: u8, value: u16, index: u16, buf: &mut [u8], _: Duration) -> Result<usize> {
            assert_eq!((value, index), (cmd::I2C_RAW | DEMOD as u16, cmd::I2C_RAW_READ));
            assert_eq!(buf.len(), 1);
            let reg = self.ptr.get();
            self.log.borrow_mut().push(R(reg));
            if self.fail_read.get() == Some(reg) {
                return Err(nak());
            }
            let queued = self.queue.borrow_mut().get_mut(&reg).and_then(|q| q.pop_front());
            buf[0] = queued.unwrap_or_else(|| *self.regs.borrow().get(&reg).unwrap_or(&0));
            Ok(1)
        }
    }

    fn tuner(driver: VendorDriver) -> (Mtv808, Device<Sim>) {
        let bus = TunerBus { tuner_addr: TUNER, path: TunerPath::ViaDemod { demod_addr: DEMOD } };
        (Mtv808::new(bus, driver), Device::new(Sim::default()))
    }

    fn desc(if_hz: i32) -> TunerDesc {
        TunerDesc { if_freq_hz: if_hz, ..Default::default() }
    }

    // The expected sequences and values below come from a separate model of
    // the vendor drivers' code.
    const INIT_DTH30_IF4: &[Op] = &[
        W(0x23, 0x18),
        W(0x2B, 0x08),
        W(0x33, 0x00),
        W(0x34, 0x7F),
        W(0x35, 0x00),
        W(0x36, 0x00),
        W(0x3E, 0xFF),
        W(0x4D, 0x8C),
        W(0x4F, 0x8B),
        W(0x50, 0x09),
        W(0x51, 0xC9),
        W(0x52, 0x92),
        W(0x54, 0xC1),
        W(0x55, 0x08),
        W(0x56, 0x00),
        W(0x58, 0xDF),
        W(0x5A, 0x04),
        W(0x5C, 0x02),
        W(0x5D, 0x1C),
        W(0x5E, 0x2A),
        W(0x5F, 0x32),
        W(0x60, 0x4C),
        W(0x61, 0x50),
        W(0x62, 0x51),
        W(0x63, 0xDB),
        W(0x64, 0xF5),
        W(0x65, 0xFF),
        W(0x66, 0xFF),
        W(0x68, 0x90),
        W(0x69, 0x74),
        W(0x6A, 0x64),
        W(0x6B, 0x01),
        W(0x6C, 0x21),
        W(0x75, 0x07),
        W(0x78, 0x20),
        W(0x79, 0xFF),
        W(0x7A, 0xF0),
        W(0x7B, 0xD0),
        W(0x7C, 0xF0),
        W(0x7E, 0x26),
        W(0x81, 0x08),
        W(0x87, 0xC0),
        W(0x8B, 0x13),
        W(0x8D, 0xE8),
        W(0x91, 0x32),
        W(0x7F, 0x36),
        W(0x80, 0xA0),
        W(0x2F, 0x00),
        W(0x3D, 0xBA),
        W(0x8B, 0x1F),
        R(0x0A),
        W(0x8B, 0x3F),
        W(0x8B, 0x2F),
        R(0x0A),
        W(0x8B, 0x2F),
        W(0x8B, 0x27),
        R(0x0A),
        W(0x8B, 0x27),
        W(0x8B, 0x23),
        R(0x0A),
        W(0x8B, 0x27),
        W(0x8B, 0x25),
        R(0x0A),
        W(0x8B, 0x27),
        W(0x8B, 0x26),
        R(0x0A),
        W(0x8B, 0x26),
        W(0x2F, 0x01),
        W(0x3D, 0x3A),
        W(0x7F, 0xF6),
        W(0x80, 0x20),
        W(0x89, 0x27),
        W(0x25, 0xA2),
        W(0x88, 0x00),
        W(0x24, 0x09),
        W(0x30, 0x0E),
        W(0x39, 0x80),
        W(0x5E, 0x5D),
        W(0x60, 0x4C),
        R(0x12),
        W(0x3D, 0x3A),
        W(0x3F, 0x4F),
        W(0x40, 0x4D),
        W(0x41, 0x4D),
        W(0x42, 0x84),
        W(0x43, 0x44),
        W(0x44, 0x8A),
        W(0x45, 0x43),
        W(0x46, 0x80),
        W(0x47, 0x22),
        W(0x48, 0x77),
        W(0x49, 0x09),
        W(0x4A, 0x2A),
    ];
    const INIT_SERIES: &[Op] = &[
        W(0x23, 0x18),
        W(0x2B, 0x08),
        W(0x33, 0xA0),
        W(0x34, 0x7F),
        W(0x35, 0x00),
        W(0x36, 0x00),
        W(0x3E, 0xFF),
        W(0x4D, 0x8C),
        W(0x4F, 0x8B),
        W(0x50, 0x09),
        W(0x51, 0xC9),
        W(0x52, 0x92),
        W(0x54, 0x41),
        W(0x55, 0x08),
        W(0x56, 0x00),
        W(0x58, 0xDF),
        W(0x5A, 0x04),
        W(0x5C, 0x02),
        W(0x5D, 0x1C),
        W(0x5E, 0x2A),
        W(0x5F, 0x32),
        W(0x60, 0x19),
        W(0x61, 0x50),
        W(0x62, 0x51),
        W(0x63, 0xD4),
        W(0x65, 0xFF),
        W(0x66, 0xFF),
        W(0x68, 0x90),
        W(0x69, 0x74),
        W(0x6A, 0x64),
        W(0x6B, 0x01),
        W(0x6C, 0x21),
        W(0x75, 0x07),
        W(0x78, 0x20),
        W(0x79, 0xFF),
        W(0x7A, 0xF0),
        W(0x7B, 0xB0),
        W(0x7C, 0xF0),
        W(0x7E, 0x26),
        W(0x87, 0xC0),
        W(0x8B, 0x13),
        W(0x8D, 0xE8),
        W(0x91, 0x32),
        W(0x7F, 0x36),
        W(0x80, 0xA0),
        W(0x2F, 0x00),
        W(0x3D, 0xBA),
        W(0x8B, 0x1F),
        R(0x0A),
        W(0x8B, 0x1F),
        W(0x8B, 0x0F),
        R(0x0A),
        W(0x8B, 0x1F),
        W(0x8B, 0x17),
        R(0x0A),
        W(0x8B, 0x1F),
        W(0x8B, 0x1B),
        R(0x0A),
        W(0x8B, 0x1B),
        W(0x8B, 0x19),
        R(0x0A),
        W(0x8B, 0x1B),
        W(0x8B, 0x1A),
        R(0x0A),
        W(0x8B, 0x1A),
        W(0x2F, 0x01),
        W(0x3D, 0x3A),
        W(0x7F, 0xF6),
        W(0x80, 0x20),
        W(0x89, 0x27),
        W(0x24, 0x00),
        W(0x30, 0x2E),
        W(0x39, 0x00),
        R(0x12),
        W(0x3D, 0x3A),
        W(0x3F, 0x7F),
        W(0x40, 0x7F),
        W(0x41, 0x7F),
        W(0x42, 0xFF),
        W(0x43, 0x7F),
        W(0x44, 0xFF),
        W(0x45, 0x7F),
        W(0x46, 0xFF),
        W(0x47, 0xFF),
        W(0x48, 0xFF),
        W(0x49, 0x1F),
        W(0x4A, 0x7F),
    ];
    const INIT_DTH30_IF0: &[Op] = &[
        W(0x23, 0x18),
        W(0x2B, 0x08),
        W(0x33, 0x00),
        W(0x34, 0x7F),
        W(0x35, 0x00),
        W(0x36, 0x00),
        W(0x3E, 0xFF),
        W(0x4D, 0x8C),
        W(0x4F, 0x8B),
        W(0x50, 0x09),
        W(0x51, 0xC9),
        W(0x52, 0x92),
        W(0x54, 0xC1),
        W(0x55, 0x08),
        W(0x56, 0x00),
        W(0x58, 0xDF),
        W(0x5A, 0x04),
        W(0x5C, 0x02),
        W(0x5D, 0x1C),
        W(0x5E, 0x2A),
        W(0x5F, 0x32),
        W(0x60, 0x4C),
        W(0x61, 0x50),
        W(0x62, 0x51),
        W(0x63, 0xDB),
        W(0x64, 0xF5),
        W(0x65, 0xFF),
        W(0x66, 0xFF),
        W(0x68, 0x90),
        W(0x69, 0x74),
        W(0x6A, 0x64),
        W(0x6B, 0x01),
        W(0x6C, 0x21),
        W(0x75, 0x07),
        W(0x78, 0x20),
        W(0x79, 0xFF),
        W(0x7A, 0xF0),
        W(0x7B, 0xD0),
        W(0x7C, 0xF0),
        W(0x7E, 0x26),
        W(0x81, 0x08),
        W(0x87, 0xC0),
        W(0x8B, 0x13),
        W(0x8D, 0xE8),
        W(0x91, 0x32),
        W(0x7F, 0x36),
        W(0x80, 0xA0),
        W(0x2F, 0x00),
        W(0x3D, 0xBA),
        W(0x8B, 0x1F),
        R(0x0A),
        W(0x8B, 0x1F),
        W(0x8B, 0x0F),
        R(0x0A),
        W(0x8B, 0x0F),
        W(0x8B, 0x07),
        R(0x0A),
        W(0x8B, 0x07),
        W(0x8B, 0x03),
        R(0x0A),
        W(0x8B, 0x03),
        W(0x8B, 0x01),
        R(0x0A),
        W(0x8B, 0x01),
        W(0x8B, 0x00),
        R(0x0A),
        W(0x8B, 0x00),
        W(0x2F, 0x01),
        W(0x3D, 0x3A),
        W(0x7F, 0xF6),
        W(0x80, 0x20),
        W(0x89, 0x27),
        W(0x24, 0x00),
        W(0x30, 0x2E),
        W(0x39, 0x00),
        R(0x12),
        W(0x3D, 0x3A),
        W(0x3F, 0x6B),
        W(0x40, 0x65),
        W(0x41, 0x65),
        W(0x42, 0x9C),
        W(0x43, 0x50),
        W(0x44, 0xAF),
        W(0x45, 0x52),
        W(0x46, 0x98),
        W(0x47, 0x32),
        W(0x48, 0x8D),
        W(0x49, 0x0B),
        W(0x4A, 0x32),
    ];
    const TUNE_DTH30_473: &[Op] = &[
        W(0x61, 0x50),
        R(0x34),
        R(0x35),
        W(0x34, 0x7F),
        W(0x35, 0x00),
        R(0x51),
        W(0x51, 0xB9),
        W(0x53, 0xFF),
        W(0x26, 0x3C),
        W(0x27, 0x0E),
        W(0x28, 0xC9),
        W(0x29, 0x22),
        W(0x2A, 0xD1),
        R(0x06),
        W(0x51, 0xB9),
    ];
    const TUNE_DTH30_219: &[Op] = &[
        W(0x22, 0x00),
        W(0x34, 0x1A),
        W(0x35, 0xFF),
        W(0x36, 0x5F),
        W(0x3E, 0xE0),
        W(0x4D, 0xCC),
        W(0x4E, 0x20),
        W(0x50, 0x00),
        W(0x51, 0x98),
        W(0x54, 0xC1),
        W(0x6C, 0x20),
        W(0x72, 0x80),
        W(0x75, 0x00),
        W(0x7F, 0xFF),
        W(0x61, 0x63),
        R(0x34),
        R(0x35),
        W(0x34, 0x10),
        W(0x35, 0xC0),
        R(0x51),
        W(0x51, 0xB8),
        W(0x53, 0xFF),
        W(0x26, 0x74),
        W(0x27, 0x36),
        W(0x28, 0xC9),
        W(0x29, 0x16),
        W(0x2A, 0x81),
        R(0x06),
        W(0x51, 0xB0),
    ];
    const TUNE_DTH30_767: &[Op] = &[
        W(0x22, 0x80),
        W(0x34, 0x7F),
        W(0x35, 0x00),
        W(0x36, 0x00),
        W(0x3E, 0xFF),
        W(0x4D, 0x8C),
        W(0x4E, 0x08),
        W(0x50, 0x09),
        W(0x51, 0xC9),
        W(0x54, 0xC1),
        W(0x6C, 0x21),
        W(0x72, 0xA0),
        W(0x75, 0x07),
        W(0x7F, 0xF6),
        W(0x61, 0x50),
        R(0x34),
        R(0x35),
        W(0x34, 0x7F),
        W(0x35, 0x00),
        R(0x51),
        W(0x51, 0xB9),
        W(0x53, 0x7F),
        W(0x26, 0x3C),
        W(0x27, 0x17),
        W(0x28, 0xF9),
        W(0x29, 0x22),
        W(0x2A, 0xD1),
        R(0x06),
        W(0x51, 0xA1),
    ];
    const TUNE_DTH30_557: &[Op] = &[
        W(0x61, 0x50),
        R(0x34),
        R(0x35),
        W(0x34, 0x7F),
        W(0x35, 0x00),
        R(0x51),
        W(0x51, 0xB9),
        W(0x53, 0xFF),
        W(0x26, 0x3C),
        W(0x27, 0x11),
        W(0x28, 0x69),
        W(0x29, 0x22),
        W(0x2A, 0xD1),
        R(0x06),
        W(0x51, 0xB1),
    ];
    const TUNE_SERIES_473: &[Op] = &[
        W(0x61, 0x50),
        R(0x51),
        W(0x51, 0xA9),
        W(0x53, 0xFF),
        W(0x26, 0x3C),
        W(0x27, 0x0E),
        W(0x28, 0xC9),
        W(0x29, 0x22),
        W(0x2A, 0xD1),
        R(0x06),
        W(0x51, 0xA9),
    ];
    const TUNE_SERIES_219: &[Op] = &[
        W(0x22, 0x00),
        W(0x34, 0x1A),
        W(0x35, 0xFF),
        W(0x36, 0x5F),
        W(0x3E, 0xE0),
        W(0x4D, 0xCC),
        W(0x4E, 0x20),
        W(0x50, 0x00),
        W(0x51, 0x98),
        W(0x54, 0x40),
        W(0x6C, 0x20),
        W(0x72, 0x80),
        W(0x75, 0x00),
        W(0x7F, 0xFF),
        W(0x61, 0x63),
        R(0x51),
        W(0x51, 0xA8),
        W(0x53, 0xFF),
        W(0x26, 0x74),
        W(0x27, 0x36),
        W(0x28, 0xC9),
        W(0x29, 0x16),
        W(0x2A, 0x81),
        R(0x06),
        W(0x51, 0xA0),
    ];
    const TUNE_SERIES_767: &[Op] = &[
        W(0x22, 0x80),
        W(0x34, 0x7F),
        W(0x35, 0x00),
        W(0x36, 0x00),
        W(0x3E, 0xFF),
        W(0x4D, 0x8C),
        W(0x4E, 0x08),
        W(0x50, 0x09),
        W(0x51, 0xC9),
        W(0x54, 0x41),
        W(0x6C, 0x21),
        W(0x72, 0xA0),
        W(0x75, 0x07),
        W(0x7F, 0xF6),
        W(0x61, 0x50),
        R(0x51),
        W(0x51, 0xA9),
        W(0x53, 0x7F),
        W(0x26, 0x3C),
        W(0x27, 0x17),
        W(0x28, 0xF9),
        W(0x29, 0x22),
        W(0x2A, 0xD1),
        R(0x06),
        W(0x51, 0x91),
    ];
    const TUNE_SERIES_557: &[Op] = &[
        W(0x61, 0x50),
        R(0x51),
        W(0x51, 0xA9),
        W(0x53, 0xFF),
        W(0x26, 0x3C),
        W(0x27, 0x11),
        W(0x28, 0x69),
        W(0x29, 0x22),
        W(0x2A, 0xD1),
        R(0x06),
        W(0x51, 0xA1),
    ];
    type Divider = Option<(u8, u32)>;
    const DIVIDER: &[(u32, Divider, Divider)] = &[
        (0, None, None),
        (49999, None, None),
        (50000, Some((4, 26214400)), Some((4, 26214400))),
        (57000, Some((4, 29884416)), Some((4, 29884416))),
        (93142, Some((4, 48833232)), Some((4, 48833232))),
        (113999, Some((4, 59768307)), Some((4, 59768307))),
        (114000, Some((4, 59768832)), Some((5, 29884416))),
        (115999, Some((4, 60816883)), Some((5, 30408441))),
        (116000, Some((5, 30408704)), Some((5, 30408704))),
        (219142, Some((5, 57446760)), Some((5, 57446760))),
        (227999, Some((5, 59768569)), Some((5, 59768569))),
        (228000, Some((5, 59768832)), Some((6, 29884416))),
        (231999, Some((5, 60817145)), Some((6, 30408572))),
        (232000, Some((6, 30408704)), Some((6, 30408704))),
        (325000, Some((6, 42598400)), Some((6, 42598400))),
        (325001, Some((6, 42598531)), Some((6, 42598531))),
        (455999, Some((6, 59768700)), Some((6, 59768700))),
        (456000, Some((6, 59768832)), Some((7, 14942208))),
        (460000, Some((6, 60293120)), Some((7, 15073280))),
        (463999, Some((6, 60817276)), Some((7, 15204319))),
        (464000, Some((7, 15204352)), Some((7, 15204352))),
        (473142, Some((7, 15503917)), Some((7, 15503917))),
        (557142, Some((7, 18256429)), Some((7, 18256429))),
        (767142, Some((7, 25137709)), Some((7, 25137709))),
        (911999, Some((7, 29884383)), Some((7, 29884383))),
        (912000, Some((7, 29884416)), None),
        (927999, Some((7, 30408671)), None),
        (928000, None, None),
        (1000000, None, None),
    ];
    const REG53_61: &[(u32, u8, u8, u8)] = &[
        (50000, 0xFF, 0x63, 0x63),
        (94999, 0xFF, 0x63, 0x63),
        (95000, 0x7F, 0x63, 0x63),
        (101999, 0x7F, 0x63, 0x63),
        (102000, 0xFF, 0x63, 0x63),
        (189999, 0xFF, 0x63, 0x63),
        (190000, 0x7F, 0x63, 0x63),
        (203999, 0x7F, 0x63, 0x63),
        (204000, 0xFF, 0x63, 0x63),
        (379999, 0xFF, 0x63, 0x63),
        (380000, 0x7F, 0x63, 0x63),
        (407999, 0x7F, 0x63, 0x63),
        (408000, 0xFF, 0x63, 0x63),
        (456000, 0xFF, 0x63, 0x63),
        (456001, 0xFF, 0x63, 0x50),
        (460000, 0xFF, 0x63, 0x50),
        (460001, 0xFF, 0x50, 0x50),
        (759999, 0xFF, 0x50, 0x50),
        (760000, 0x7F, 0x50, 0x50),
        (815999, 0x7F, 0x50, 0x50),
        (816000, 0xFF, 0x50, 0x50),
        (900000, 0xFF, 0x50, 0x50),
    ];
    const STRENGTH: &[(u8, u8, u8, bool, i32, i32)] = &[
        (0, 0, 0, false, 43000, 43000),
        (0, 0, 0, true, 35000, 43000),
        (0, 0, 1, false, 23000, 23000),
        (0, 0, 1, true, 21000, 53000),
        (0, 0, 2, false, 0, 0),
        (0, 0, 2, true, 0, 28000),
        (0, 0, 3, false, 9000, 9000),
        (0, 0, 3, true, 0, 5500),
        (10, 20, 0, false, 35200, 35200),
        (10, 20, 0, true, 26400, 34500),
        (10, 20, 1, false, 14000, 14000),
        (10, 20, 1, true, 10900, 37000),
        (10, 20, 2, false, -10000, -10000),
        (10, 20, 2, true, -11600, 12000),
        (10, 20, 3, false, -3500, -3500),
        (10, 20, 3, true, -12500, -6900),
        (37, 91, 0, false, 10000, 10000),
        (37, 91, 0, true, -500, 7300),
        (37, 91, 1, false, -13700, -13700),
        (37, 91, 1, true, -20100, -13000),
        (37, 91, 2, false, -40400, -40400),
        (37, 91, 2, true, -46800, -38000),
        (37, 91, 3, false, -44000, -44000),
        (37, 91, 3, true, -53000, -47000),
        (100, 3, 0, false, 12200, 12200),
        (100, 3, 0, true, -7600, 7200),
        (100, 3, 1, false, -27600, -27600),
        (100, 3, 1, true, -36600, -28200),
        (100, 3, 2, false, -60600, -60600),
        (100, 3, 2, true, -70600, -53200),
        (100, 3, 3, false, -37200, -37200),
        (100, 3, 3, true, -46200, -41600),
        (255, 255, 0, false, -94700, -94700),
        (255, 255, 0, true, -128200, -110000),
        (255, 255, 1, false, -155500, -155500),
        (255, 255, 1, true, -180400, -253000),
        (255, 255, 2, false, -204000, -204000),
        (255, 255, 2, true, -237100, -278000),
        (255, 255, 3, false, -207700, -207700),
        (255, 255, 3, true, -216700, -211200),
    ];
    const FAIL61_SERIES: &[Op] = &[
        W(0x61, 0x50),
        R(0x51),
        W(0x51, 0xA9),
        W(0x53, 0xFF),
        W(0x26, 0x3C),
        W(0x27, 0x0E),
        W(0x28, 0xC9),
        W(0x29, 0x22),
        W(0x2A, 0xD1),
        R(0x06),
        W(0x51, 0xA9),
    ];
    const FAIL61_DTH30: &[Op] = &[W(0x61, 0x50)];
    const FAIL34_DTH30: &[Op] = &[
        W(0x61, 0x50),
        R(0x34),
        R(0x35),
        W(0x34, 0x7F),
        W(0x35, 0x80),
        R(0x51),
        W(0x51, 0xB9),
        W(0x53, 0xFF),
        W(0x26, 0x3C),
        W(0x27, 0x0E),
        W(0x28, 0xC9),
        W(0x29, 0x22),
        W(0x2A, 0xD1),
        R(0x06),
        W(0x51, 0xB9),
    ];
    const RANGE_SERIES: &[Op] = &[W(0x61, 0x50)];
    const RANGE_DTH30: &[Op] = &[W(0x61, 0x50)];

    #[test]
    fn init_dth30_with_a_4_mhz_if() {
        let (mut t, mut dev) = tuner(VendorDriver::Dth30);
        dev.transport().queue(0x0A, &[0x10, 0x90, 0x90, 0x10, 0x00, 0xFF]);
        dev.transport().queue(0x12, &[0x05]);
        t.init(&mut dev, &desc(4_000_000)).unwrap();
        assert_eq!(dev.transport().take(), INIT_DTH30_IF4);
        assert_eq!((t.mode, t.compensated, t.filter), (5, true, 1));
    }

    #[test]
    fn init_dth30_with_another_if() {
        let (mut t, mut dev) = tuner(VendorDriver::Dth30);
        dev.transport().queue(0x0A, &[0x80; 6]);
        dev.transport().queue(0x12, &[0x0F]);
        t.init(&mut dev, &desc(0)).unwrap();
        assert_eq!(dev.transport().take(), INIT_DTH30_IF0);
        assert_eq!(t.mode, 0);
    }

    #[test]
    fn init_series_ignores_the_if_and_limits_a_negative_temperature() {
        let (mut t, mut dev) = tuner(VendorDriver::Series);
        dev.transport().queue(0x0A, &[0x90, 0x10, 0x10, 0x90, 0x10, 0x90]);
        dev.transport().queue(0x12, &[0x13]);
        t.init(&mut dev, &desc(4_000_000)).unwrap();
        assert_eq!(dev.transport().take(), INIT_SERIES);
        assert_eq!(t.comp, [2, 2, 127, 127, 127, 255, 127, 255, 127, 255, 511, 127, 127, 31]);
        assert_eq!(t.mode, 0);
    }

    #[test]
    fn init_table_failure() {
        // The DT-H30/PCIEM-TS driver calibrates, then stops; the series driver goes on.
        for driver in [VendorDriver::Dth30, VendorDriver::Series] {
            let (mut t, mut dev) = tuner(driver);
            dev.transport().fail_write.set(Some(0x2B));
            let r = t.init(&mut dev, &desc(0));
            let ops = dev.transport().take();
            assert_eq!(&ops[..3], &[W(0x23, 0x18), W(0x2B, 0x08), W(0x7F, 0x36)], "{driver:?}");
            assert_eq!(r.is_err(), driver == VendorDriver::Dth30, "{driver:?}");
            assert_eq!(ops.contains(&W(0x24, 0x00)), driver == VendorDriver::Series, "{driver:?}");
        }
    }

    fn tune_after_init(driver: VendorDriver, ops: &[(u32, u8, &[Op])], init: impl Fn(&Sim)) {
        let (mut t, mut dev) = tuner(driver);
        init(dev.transport());
        t.init(&mut dev, &desc(4_000_000)).unwrap();
        dev.transport().take();
        for &(hz, r06, expect) in ops {
            dev.transport().queue(0x06, &[r06]);
            t.tune(&mut dev, hz).unwrap();
            assert_eq!(dev.transport().take(), expect, "{driver:?} {hz} Hz");
        }
    }

    #[test]
    fn tune_sequences_dth30() {
        tune_after_init(
            VendorDriver::Dth30,
            &[
                (473_142_857, 0x10, TUNE_DTH30_473),
                (219_142_857, 0x40, TUNE_DTH30_219),
                (767_142_857, 0x90, TUNE_DTH30_767),
                (557_142_857, 0x20, TUNE_DTH30_557),
            ],
            |s| {
                s.queue(0x0A, &[0x10, 0x90, 0x90, 0x10, 0x00, 0xFF]);
                s.queue(0x12, &[0x05]);
            },
        );
    }

    #[test]
    fn tune_sequences_series() {
        tune_after_init(
            VendorDriver::Series,
            &[
                (473_142_857, 0x10, TUNE_SERIES_473),
                (219_142_857, 0x40, TUNE_SERIES_219),
                (767_142_857, 0x90, TUNE_SERIES_767),
                (557_142_857, 0x20, TUNE_SERIES_557),
            ],
            |s| {
                s.queue(0x0A, &[0x90, 0x10, 0x10, 0x90, 0x10, 0x90]);
                s.queue(0x12, &[0x13]);
            },
        );
    }

    #[test]
    fn divider_words_and_band_edges() {
        for &(khz, series, dth30) in DIVIDER {
            assert_eq!(divider(VendorDriver::Series, khz), series, "series {khz} kHz");
            assert_eq!(divider(VendorDriver::Dth30, khz), dth30, "DT-H30 {khz} kHz");
        }
        // UHF 27 centre: 557142 kHz / 32 MHz = 17.41 in 12.20 fixed point, to 0x26..0x2A
        assert_eq!(
            divider_registers(VendorDriver::Dth30, 557_142),
            Some([(0x26, 0x3C), (0x27, 0x11), (0x28, 0x69), (0x29, 0x22), (0x2A, 0xD1)])
        );
    }

    #[test]
    fn registers_0x53_and_0x61() {
        for &(khz, r53, r61_series, r61_dth30) in REG53_61 {
            assert_eq!(reg53(khz), r53, "{khz} kHz");
            assert_eq!(if khz > SERIES.r61_edge { 0x50 } else { 0x63 }, r61_series, "series {khz} kHz");
            assert_eq!(if khz > DTH30.r61_edge { 0x50 } else { 0x63 }, r61_dth30, "DT-H30 {khz} kHz");
        }
    }

    #[test]
    fn strength_formulas() {
        for &(r3, r4, lna, comp, series, dth30) in STRENGTH {
            for (driver, expect) in [(VendorDriver::Series, series), (VendorDriver::Dth30, dth30)] {
                let (mut t, mut dev) = tuner(driver);
                t.compensated = comp;
                for (reg, v) in [(3, r3), (4, r4), (5, 0xFC | lna)] {
                    dev.transport().regs.borrow_mut().insert(reg, v);
                }
                let mut st = SignalStatus::default();
                t.status(&mut dev, &mut st).unwrap();
                assert_eq!(st.strength, expect, "{driver:?} {r3} {r4} {lna} {comp}");
                let mask = if driver == VendorDriver::Series { 0x0A } else { 0x02 };
                assert_eq!(st.mask, mask);
                assert_eq!(dev.transport().take(), [R(3), R(4), R(5)]);
            }
        }
    }

    #[test]
    fn strength_is_0_when_a_read_fails() {
        let (mut t, mut dev) = tuner(VendorDriver::Dth30);
        dev.transport().regs.borrow_mut().insert(3, 0x10);
        dev.transport().fail_read.set(Some(4));
        let mut st = SignalStatus { strength: 1234, ..Default::default() };
        t.status(&mut dev, &mut st).unwrap();
        assert_eq!((st.strength, st.mask), (0, field::STRENGTH));
        assert_eq!(dev.transport().take(), [R(3), R(4)]);
    }

    #[test]
    fn wake_and_standby() {
        let (t, mut dev) = tuner(VendorDriver::Dth30);
        dev.transport().regs.borrow_mut().insert(0x33, 0x80);
        t.standby(&mut dev).unwrap();
        t.wake(&mut dev).unwrap();
        assert_eq!(dev.transport().take(), [R(0x33), W(0x33, 0xC0), R(0x33), W(0x33, 0x80)]);
        let (t, mut dev) = tuner(VendorDriver::Series);
        t.standby(&mut dev).unwrap();
        t.wake(&mut dev).unwrap();
        assert_eq!(dev.transport().take(), []);
    }

    fn after_init(driver: VendorDriver) -> (Mtv808, Device<Sim>) {
        let (mut t, mut dev) = tuner(driver);
        dev.transport().queue(0x12, &[0x05]);
        t.init(&mut dev, &desc(0)).unwrap();
        dev.transport().take();
        dev.transport().queue(0x06, &[0]);
        (t, dev)
    }

    #[test]
    fn a_failed_0x61_write_stops_the_dth30_driver_only() {
        let (mut t, mut dev) = after_init(VendorDriver::Series);
        dev.transport().fail_write.set(Some(0x61));
        t.tune(&mut dev, 473_142_857).unwrap();
        assert_eq!(dev.transport().take(), FAIL61_SERIES);
        let (mut t, mut dev) = after_init(VendorDriver::Dth30);
        dev.transport().fail_write.set(Some(0x61));
        assert!(t.tune(&mut dev, 473_142_857).is_err());
        assert_eq!(dev.transport().take(), FAIL61_DTH30);
    }

    #[test]
    fn dth30_band_table_reads_may_fail() {
        let (mut t, mut dev) = after_init(VendorDriver::Dth30);
        dev.transport().fail_read.set(Some(0x34));
        dev.transport().regs.borrow_mut().insert(0x35, 0xFF);
        t.tune(&mut dev, 473_142_857).unwrap();
        assert_eq!(dev.transport().take(), FAIL34_DTH30);
    }

    #[test]
    fn out_of_range() {
        for (driver, expect) in [(VendorDriver::Series, RANGE_SERIES), (VendorDriver::Dth30, RANGE_DTH30)] {
            let (mut t, mut dev) = after_init(driver);
            let e = t.tune(&mut dev, 930_000_000).unwrap_err();
            std::assert_matches!(e, Error::Unsupported(_), "{e}");
            assert_eq!(dev.transport().take(), expect, "{driver:?}");
        }
    }

    #[test]
    fn compensation_is_written_by_the_first_tune_without_an_init() {
        let (mut t, mut dev) = tuner(VendorDriver::Series);
        dev.transport().queue(0x06, &[0]);
        t.tune(&mut dev, 557_142_857).unwrap();
        let ops = dev.transport().take();
        assert_eq!(&ops[..14], &HIGH_TABLE_SERIES.map(|(r, v)| W(r, v)));
        assert_eq!(ops[14], W(0x61, 0x50));
        assert_eq!(ops[15], W(0x3D, 0x30));
        assert!(t.compensated);
    }
}
