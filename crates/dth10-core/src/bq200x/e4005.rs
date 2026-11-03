//! Elonics E4005 tuner, as the original driver's `BQ200X_E4005` kits drive it.
//!
//! Registers are written and read one byte at a time with register transfers
//! (`0x2D00|addr`, wIndex = register) on the tuner address, through the
//! BQ200X's I2C gate (or directly, descriptor flags bit3). The crystal is fixed
//! at 28.8 MHz in the driver (the descriptor's tuner crystal is not used); the
//! IF (kHz, from the descriptor) selects the IF filter set and, when it is not
//! positive, a second-harmonic step.
//!
//! Tune: DC calibration (0x1F/0x1B), PLL (band 0x07, divider 0x0D, N 0x09,
//! fraction 0x0A/0x0B, RF tracking filter 0x10), a 90-register gain table,
//! IF filter 0x11..0x13/0x43.

use super::demod::TunerGate;
use crate::device::Device;
use crate::error::{Error, Result};
use crate::protocol::UsbControl;
use crate::tlv::TunerDesc;

/// Crystal assumed by the frequency calculation, in kHz.
pub const XTAL_KHZ: i32 = 28_800;

/// First initialisation table.
pub const INIT_TABLE_A: [(u8, u8); 42] = [
    (0x50, 0x21),
    (0x51, 0x2F),
    (0x52, 0x1F),
    (0x53, 0x0B),
    (0x54, 0x1A),
    (0x55, 0x2B),
    (0x56, 0x13),
    (0x57, 0x3A),
    (0x58, 0x20),
    (0x59, 0x23),
    (0x5A, 0x1D),
    (0x5B, 0x24),
    (0x5C, 0x16),
    (0x5D, 0x11),
    (0x60, 0x2A),
    (0x61, 0x2B),
    (0x62, 0x34),
    (0x63, 0x39),
    (0x64, 0x16),
    (0x65, 0x1F),
    (0x66, 0x33),
    (0x67, 0x28),
    (0x68, 0x2D),
    (0x69, 0x31),
    (0x6A, 0x3B),
    (0x6B, 0x0F),
    (0x6C, 0x28),
    (0x6D, 0x2D),
    (0x70, 0x88),
    (0x71, 0x87),
    (0x72, 0x88),
    (0x73, 0x87),
    (0x74, 0x98),
    (0x75, 0x95),
    (0x76, 0x99),
    (0x77, 0x87),
    (0x78, 0x88),
    (0x79, 0x87),
    (0x7A, 0x88),
    (0x7B, 0x96),
    (0x7C, 0x98),
    (0x7D, 0x94),
];

/// Second initialisation table.
pub const INIT_TABLE_B: [(u8, u8); 21] = [
    (0x00, 0x07),
    (0x8E, 0x01),
    (0x8F, 0xFE),
    (0x06, 0x20),
    (0x1F, 0x0F),
    (0x1A, 0x00),
    (0x16, 0x00),
    (0x17, 0x00),
    (0x18, 0x00),
    (0xF7, 0x00),
    (0x24, 0x07),
    (0x80, 0x19),
    (0x81, 0x19),
    (0xFA, 0x08),
    (0x1D, 0x18),
    (0x1E, 0x10),
    (0x2D, 0x1C),
    (0xFE, 0x40),
    (0xF4, 0x38),
    (0xF5, 0x2C),
    (0x14, 0x00),
];

/// First register of the gain table written at every tune.
pub const GAIN_TABLE_BASE: u8 = 0x9A;

/// Gain table: registers 0x9A..=0xF3. The driver holds two copies (chosen by
/// band) with identical contents.
pub const GAIN_TABLE: [u8; 90] = [
    0x01, 0x11, 0x01, 0x11, 0x01, 0x11, 0x41, 0x11, 0x21, 0x11, 0x48, 0x13, 0x61, 0x11, 0x21, 0x13, 0x41, 0x14, 0x49,
    0x11, 0x29, 0x11, 0x69, 0x11, 0x41, 0x15, 0x21, 0x14, 0x41, 0x16, 0x31, 0x13, 0x41, 0x17, 0x21, 0x15, 0x49, 0x16,
    0x69, 0x14, 0x2A, 0x13, 0x22, 0x14, 0x2B, 0x13, 0x19, 0x14, 0x1B, 0x13, 0x2B, 0x14, 0x12, 0x14, 0x52, 0x14, 0x53,
    0x14, 0x5B, 0x14, 0x3B, 0x14, 0x7B, 0x14, 0x53, 0x15, 0x5B, 0x15, 0x64, 0x17, 0x65, 0x17, 0x6D, 0x17, 0x66, 0x17,
    0x6E, 0x17, 0x1D, 0x17, 0x16, 0x17, 0x1F, 0x17, 0x5F, 0x17, 0x3F, 0x17, 0x7F, 0x1F,
];

/// RF tracking-filter centre table in MHz: 16 rows (filter index 0..15) × 4
/// columns (selected by `band - 1`), followed by the 4 IF classes in kHz. The
/// driver indexes it flat as `[4 * index + band - 1]`, so band 5 reads the next
/// row's first column (and index 15 the first IF class); that is kept.
pub const TRACK_TABLE: [i32; 68] = [
    268, 509, 360, 1300, //
    268, 509, 380, 1320, //
    268, 509, 405, 1360, //
    268, 509, 425, 1410, //
    268, 509, 450, 1445, //
    268, 509, 475, 1460, //
    268, 509, 505, 1490, //
    268, 509, 540, 1530, //
    299, 656, 575, 1560, //
    299, 656, 615, 1590, //
    299, 656, 670, 1640, //
    299, 656, 720, 1660, //
    299, 656, 760, 1680, //
    299, 656, 840, 1700, //
    299, 656, 890, 1720, //
    299, 656, 970, 1750, //
    0, 2000, 4000, 4570,
];

/// IF classes in kHz (the nearest one to the IF selects the IF filter set).
pub const IF_CLASSES: [i32; 4] = [0, 2000, 4000, 4570];

/// One IF filter setting: IF class, bandwidth in kHz, registers 0x11 / 0x12 / 0x13 and 0x43 bits 4:2.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IfFilter {
    pub if_class: i32,
    pub bw_khz: i32,
    pub regs: [u8; 4],
}

const fn flt(if_class: i32, bw_khz: i32, regs: [u8; 4]) -> IfFilter {
    IfFilter { if_class, bw_khz, regs }
}

/// IF filter settings for IF class 0 (zero / negative IF).
pub const IF_FILTERS_ZERO: [IfFilter; 6] = [
    flt(0, 8000, [9, 17, 0, 28]),
    flt(0, 7000, [11, 20, 0, 28]),
    flt(0, 6000, [13, 5, 0, 28]),
    flt(0, 5000, [15, 21, 0, 28]),
    flt(0, 1500, [15, 62, 0, 28]),
    flt(0, 0, [0, 0, 0, 0]),
];

/// IF filter settings for the other IF classes.
pub const IF_FILTERS: [IfFilter; 15] = [
    flt(4570, 8000, [9, 17, 0, 10]),
    flt(4570, 7000, [11, 20, 0, 18]),
    flt(4570, 6000, [13, 5, 0, 22]),
    flt(4570, 5000, [15, 21, 0, 30]),
    flt(4570, 1500, [15, 62, 0, 30]),
    flt(4000, 8000, [13, 5, 0, 30]),
    flt(4000, 7000, [13, 5, 0, 30]),
    flt(4000, 6000, [13, 5, 0, 30]),
    flt(4000, 5000, [13, 5, 0, 30]),
    flt(4000, 1500, [13, 5, 0, 30]),
    flt(2000, 8000, [15, 62, 0, 30]),
    flt(2000, 7000, [15, 62, 0, 30]),
    flt(2000, 6000, [15, 62, 0, 30]),
    flt(2000, 5000, [15, 62, 0, 30]),
    flt(2000, 1500, [15, 62, 0, 30]),
];

fn within(f: i32, lo: i32, span: u32) -> bool {
    (f.wrapping_sub(lo) as u32) <= span
}

/// Band of a frequency in kHz: 1 (44–130 MHz), 3 (135–388 MHz and 470–900 MHz),
/// 5 (393–466 MHz), 4 (1452–1680 MHz), -1 otherwise; with the UHF sub-band
/// (1 ≤ 600 MHz, 2 ≤ 700, 3 ≤ 800, 4 ≤ 900; 0 elsewhere).
pub fn band(f_khz: i32) -> (i32, i32) {
    if within(f_khz, 44_000, 86_000) {
        (1, 0)
    } else if within(f_khz, 135_000, 253_000) {
        (3, 0)
    } else if within(f_khz, 393_000, 73_000) {
        (5, 0)
    } else if within(f_khz, 470_000, 430_000) {
        let mut sub = 0;
        if f_khz <= 600_000 {
            sub = 1;
        }
        if within(f_khz, 600_001, 99_999) {
            sub = 2;
        }
        if within(f_khz, 700_001, 99_999) {
            sub = 3;
        }
        if within(f_khz, 800_001, 99_999) {
            sub = 4;
        }
        (3, sub)
    } else if within(f_khz, 1_452_000, 228_000) {
        (4, 0)
    } else {
        (-1, 0)
    }
}

/// The divider selection: `sel` is 1 for band 1, 0 for the other bands, -1 out of range.
fn divider_sel(f_khz: i32) -> i32 {
    match band(f_khz).0 {
        b if b < 0 => -1,
        b => (b <= 2) as i32,
    }
}

/// VCO divider for a frequency: `(code for register 0x0D bits 2:0, multiplier)`.
pub fn divider(f_khz: i32, sel: i32) -> Option<(u8, i32)> {
    let table: &[(i32, u32, u8, i32)] = if sel == 1 {
        &[
            (470_000, 505_511, 0, 4),
            (360_000, 110_000, 1, 8),
            (251_000, 109_000, 2, 12),
            (169_000, 82_000, 3, 16),
            (115_000, 54_000, 4, 24),
            (97_000, 18_000, 5, 32),
            (92_000, 5_000, 6, 40),
            (55_000, 37_000, 7, 48),
        ]
    } else {
        &[
            (975_000, 974_999, 0, 2),
            (668_000, 307_000, 1, 4),
            (482_000, 186_000, 2, 6),
            (342_000, 140_000, 3, 8),
            (217_000, 125_000, 4, 12),
            (163_000, 54_000, 5, 16),
            (130_000, 33_000, 6, 20),
            (109_000, 21_000, 7, 24),
        ]
    };
    table.iter().find(|e| within(f_khz, e.0, e.1)).map(|e| (e.2, e.3))
}

/// Multiplier back from a divider code (-1 for an unknown code).
pub fn multiplier_from_code(code: u8, sel: i32) -> i32 {
    let t: [i32; 8] = if sel == 1 { [4, 8, 12, 16, 24, 32, 40, 48] } else { [2, 4, 6, 8, 12, 16, 20, 24] };
    t.get(code as usize).copied().unwrap_or(-1)
}

/// PLL words: integer N (register 0x09) and 16-bit fraction (0x0A low, 0x0B high),
/// `f · mult / 28.8 MHz`; the fraction is held within 0x0800..=0xF800.
pub fn pll_words(f_khz: i32, mult: i32) -> (u8, u16) {
    let x = f_khz as f64 / 1000.0 * mult as f64 / 28.8;
    let mut n = x as i32;
    let frac = ((x - n as f64) * 65536.0) as i32;
    let mut v = 0x800;
    if frac >= 0x800 {
        v = frac.min(0xF800);
        if v == 0xFFFF {
            n += 1;
            v = 0;
        }
    }
    (n as u8, v as u16)
}

/// RF tracking-filter index (register 0x10): the entry of the band's column
/// nearest to the frequency.
pub fn track_index(f_khz: i32, band: i32) -> u8 {
    let at = |i: i32| 1000 * TRACK_TABLE[(4 * i + band - 1) as usize];
    for i in 1..16 {
        if at(i) >= f_khz {
            let d1 = (at(i) - f_khz) as f64;
            let d0 = (f_khz - at(i - 1)) as f64;
            return if d1.abs() > d0.abs() { (i - 1) as u8 } else { i as u8 };
        }
        if i == 15 {
            return 15;
        }
    }
    15
}

/// IF filter setting for a bandwidth and IF (both kHz): nearest IF class, then the
/// nearest bandwidth among that class's entries (ties go to the later entry; the
/// search starts from the table's first entry whatever its class).
pub fn if_filter(bw_khz: i32, if_khz: i32) -> IfFilter {
    let dist = |a: i32, b: i32| (a as i64 - b as i64).abs();
    let mut best = 0;
    for (i, &c) in IF_CLASSES.iter().enumerate() {
        if dist(IF_CLASSES[best], if_khz) > dist(c, if_khz) {
            best = i;
        }
    }
    let class = IF_CLASSES[best];
    let table: &[IfFilter] = if class == 0 { &IF_FILTERS_ZERO } else { &IF_FILTERS };
    let mut sel = 0;
    for (j, e) in table.iter().enumerate() {
        if e.if_class == class && dist(table[sel].bw_khz, bw_khz) >= dist(e.bw_khz, bw_khz) {
            sel = j;
        }
    }
    table[sel]
}

/// Register 0x44 value of the IF step (positive IF): `2 · round(VCO / (IF · 16 · k))`
/// with VCO = `mult · f`, k = 3 for divider codes 0, 2, 4, 7 and 2 otherwise.
pub fn if_step(f_khz: i32, code: u8, if_khz: i32) -> u8 {
    let mult = multiplier_from_code(code, divider_sel(f_khz));
    let k = if matches!(code, 0 | 2 | 4 | 7) { 3 } else { 2 };
    let vco_mhz = (mult.wrapping_mul(f_khz) / 1000) as f64;
    let den = (if_khz as f64 / 1000.0) * (k as f64 * 16.0);
    ((vco_mhz / den).round() as i32 as u8).wrapping_mul(2)
}

/// Register 0x07 value of the second-harmonic step (IF not positive), or 0 when
/// the channel is not within one bandwidth below a crystal-harmonic edge.
pub fn harmonic_band(f_khz: i32, bw_khz: i32) -> u8 {
    let f = f_khz as f64 / 1000.0;
    let x = XTAL_KHZ as f64 / 1000.0;
    let bw = (bw_khz / 1000) as f64;
    let t = (f / x).round() as i32;
    let edge = ((((t as f64 * x + bw * 0.5) * 10000.0).round() as i32) / 10000) as f64;
    let fi = (((f * 10000.0).round() as i32) / 10000) as f64;
    let d = edge - fi;
    if bw < d || d < 0.0 {
        return 0;
    }
    let mult = match fi {
        v if v < 82.0 => 40,
        v if v < 108.0 => 32,
        v if v < 162.0 => 24,
        v if v < 226.0 => 16,
        v if v < 325.0 => 12,
        v if v < 487.0 => 8,
        v if v < 668.0 => 6,
        v if v < 975.0 => 4,
        _ => 2,
    };
    let div = match fi {
        v if v < 110.0 => 2,
        v if v < 226.0 => 4,
        v if v < 274.0 => 8,
        _ => 16,
    };
    let x2 = mult as f64 * edge / (div * mult) as f64;
    let hi = match x2 {
        v if v < 34.0 => 0,
        v if v < 47.0 => 8,
        v if v < 60.0 => 16,
        _ => 24,
    } & 0x18;
    hi | match fi {
        v if v < 140.0 => 1,
        v if v < 460.0 => 3,
        v if v < 868.0 => 5,
        _ => 7,
    }
}

/// The E4005 tuner object.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct E4005 {
    pub addr: u8,
    /// IF in kHz, taken from the descriptor at initialisation.
    pub if_khz: i32,
}

impl E4005 {
    pub fn new(addr: u8) -> Self {
        E4005 { addr, if_khz: 0 }
    }

    fn wr<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8, v: u8) -> Result<()> {
        dev.i2c_reg_write(self.addr, reg, &[v])
    }

    fn rd<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8) -> Result<u8> {
        let mut b = [0u8; 1];
        dev.i2c_reg_read(self.addr, reg, &mut b)?;
        Ok(b[0])
    }

    /// Initialisation: 0x09/0x05 cleared then 0x09 = 0x80, the two tables,
    /// 0x06 = 0x20, 0x8A = 0x96. The gate is closed again only on success (as in
    /// the original).
    pub fn init<T: UsbControl>(&mut self, dev: &mut Device<T>, desc: &TunerDesc, gate: &TunerGate) -> Result<()> {
        self.if_khz = desc.if_freq_hz / 1000;
        gate.open(dev)?;
        self.wr(dev, 0x09, 0x00)?;
        self.wr(dev, 0x05, 0x00)?;
        self.wr(dev, 0x09, 0x80)?;
        for &(r, v) in INIT_TABLE_A.iter().chain(INIT_TABLE_B.iter()) {
            self.wr(dev, r, v)?;
        }
        self.wr(dev, 0x06, 0x20)?;
        self.wr(dev, 0x8A, 0x96)?;
        let _ = gate.close(dev);
        Ok(())
    }

    /// Standby: register 0x00 = 0x00.
    pub fn standby<T: UsbControl>(&self, dev: &mut Device<T>, gate: &TunerGate) -> Result<()> {
        gate.with(dev, |d| self.wr(d, 0x00, 0x00))
    }

    /// Wake: register 0x00 = 0x02.
    pub fn wake<T: UsbControl>(&self, dev: &mut Device<T>, gate: &TunerGate) -> Result<()> {
        gate.with(dev, |d| self.wr(d, 0x00, 0x02))
    }

    /// DC calibration: 0x1F = 0x1F fifteen times, then the average of 100 reads of
    /// 0x1B (each after 0x1F = 0x1F) goes back to 0x1B with 0x1F = 0x0F, 0xFA = 0x08.
    fn calibrate<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        for _ in 0..14 {
            self.wr(dev, 0x1F, 0x1F)?;
        }
        self.wr(dev, 0x1F, 0x1F)?;
        self.rd(dev, 0x1B)?;
        let mut sum = 0i32;
        for _ in 0..100 {
            self.wr(dev, 0x1F, 0x1F)?;
            sum += self.rd(dev, 0x1B)? as i32;
        }
        self.wr(dev, 0x1F, 0x0F)?;
        self.wr(dev, 0xFA, 0x08)?;
        self.wr(dev, 0x1B, (sum / 100) as u8)
    }

    /// Start of a tune: calibration, 0x00 = 2, 0x1A = 0x1C, 0x20 bits 1:0 = 0b00 with bit2 set.
    fn prepare<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.rd(dev, 0x1A)?;
        let r20 = self.rd(dev, 0x20)?;
        let r8d = self.rd(dev, 0x8D)?;
        // The original ignores the calibration result.
        let _ = self.calibrate(dev);
        self.wr(dev, 0x00, 0x02)?;
        self.wr(dev, 0x1A, 0x1C)?;
        self.wr(dev, 0x20, (r20 & 0xFC) | 0x04)?;
        self.wr(dev, 0x8D, r8d)
    }

    /// IF step for a positive IF (registers 0x43 / 0x44 / 0x96).
    fn if_step_regs<T: UsbControl>(&self, dev: &mut Device<T>, f_khz: i32) -> Result<()> {
        let code = self.rd(dev, 0x0D)? & 7;
        if self.if_khz == 0 {
            self.wr(dev, 0x43, 0x00)?;
            self.wr(dev, 0x44, 0x24)?;
            return self.wr(dev, 0x96, 0x20);
        }
        self.wr(dev, 0x43, 0x0B)?;
        self.wr(dev, 0x44, if_step(f_khz, code, self.if_khz))?;
        self.wr(dev, 0x96, 0xFF)
    }

    /// PLL and RF section of a tune.
    fn set_pll<T: UsbControl>(&self, dev: &mut Device<T>, f_khz: i32) -> Result<()> {
        let r07 = self.rd(dev, 0x07)?;
        let r0d = self.rd(dev, 0x0D)?;
        let r8d = self.rd(dev, 0x8D)?;
        let r13 = self.rd(dev, 0x13)?;
        let r20 = self.rd(dev, 0x20)?;
        let (b, _) = band(f_khz);
        if b < 0 {
            return Err(Error::Unsupported(format!("E4005: frequency {f_khz} kHz out of range")));
        }
        let r07 = (r07 & 0xF9) | ((2 * b - 2) as u8);
        let r88 = if b > 3 { 0 } else { 3 };
        let (_, sub) = band(f_khz);
        let sel = divider_sel(f_khz);
        // Without a divider the multiplier slot keeps the sub-band value (as in the original).
        let (code, mult) = divider(f_khz, sel).unwrap_or((0xFF, sub));
        let r0d = code | ((((sel as u8) << 3) | (r0d & 0xF7)) & 0xF8);
        self.wr(dev, 0x0D, r0d)?;
        if self.if_khz > 0 {
            let _ = self.if_step_regs(dev, f_khz);
        } else {
            let _ = self.rd(dev, 0x0D).and_then(|_| {
                self.wr(dev, 0x43, 0x00)?;
                self.wr(dev, 0x44, 0x24)?;
                self.wr(dev, 0x96, 0x20)
            });
        }
        let (n, frac) = pll_words(f_khz, mult);
        let idx = track_index(f_khz, b);
        self.wr(dev, 0x07, r07)?;
        self.wr(dev, 0x88, r88)?;
        self.wr(dev, 0x8D, r8d)?;
        self.wr(dev, 0x13, r13)?;
        self.wr(dev, 0x09, n)?;
        self.wr(dev, 0x0A, frac as u8)?;
        self.wr(dev, 0x0B, (frac >> 8) as u8)?;
        self.wr(dev, 0x10, idx)?;
        for (i, &v) in GAIN_TABLE.iter().enumerate() {
            if self.wr(dev, GAIN_TABLE_BASE + i as u8, v).is_err() {
                break;
            }
        }
        if self.rd(dev, 0x8D).is_ok() && self.wr(dev, 0x29, 0x01).is_ok() {
            let _ = self.wr(dev, 0x2D, 0x1C);
        }
        self.wr(dev, 0x20, (r20 & 0x07) | 0x60)
    }

    /// IF filter: 0x11 / 0x12 / 0x13 and 0x43 bits 4:2.
    fn set_if_filter<T: UsbControl>(&self, dev: &mut Device<T>, bw_khz: i32) -> Result<()> {
        let r43 = self.rd(dev, 0x43)?;
        let f = if_filter(bw_khz, self.if_khz);
        self.wr(dev, 0x11, f.regs[0])?;
        self.wr(dev, 0x12, f.regs[1])?;
        self.wr(dev, 0x13, f.regs[2])?;
        self.wr(dev, 0x43, (r43 & 0xE3) | f.regs[3])
    }

    /// Tune to `freq_hz` with bandwidth `bw_hz` (both used in kHz). The gate is
    /// closed again only on success (as in the original).
    pub fn set_frequency<T: UsbControl>(
        &self,
        dev: &mut Device<T>,
        gate: &TunerGate,
        freq_hz: u32,
        bw_hz: u32,
    ) -> Result<()> {
        let f_khz = (freq_hz / 1000) as i32;
        let bw_khz = (bw_hz / 1000) as i32;
        gate.open(dev)?;
        self.prepare(dev)?;
        let r8d = self.rd(dev, 0x8D)?;
        self.wr(dev, 0x8D, (r8d & 0xF9) | 0x04)?;
        self.set_pll(dev, f_khz)?;
        self.set_if_filter(dev, bw_khz)?;
        if self.if_khz <= 0 {
            let v = harmonic_band(f_khz, bw_khz);
            if v != 0 {
                self.wr(dev, 0x07, v)?;
                self.wr(dev, 0x05, 0x03)?;
            }
        }
        let _ = gate.close(dev);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bands() {
        assert_eq!(band(557_142), (3, 1));
        assert_eq!(band(650_000), (3, 2));
        assert_eq!(band(700_001), (3, 3));
        assert_eq!(band(899_999), (3, 4));
        assert_eq!(band(100_000), (1, 0));
        assert_eq!(band(200_000), (3, 0));
        assert_eq!(band(400_000), (5, 0));
        assert_eq!(band(1_500_000), (4, 0));
        assert_eq!(band(468_000), (-1, 0));
    }

    #[test]
    fn dividers() {
        assert_eq!(divider(557_142, 0), Some((2, 6)));
        assert_eq!(divider(767_142, 0), Some((1, 4)));
        assert_eq!(divider(100_000, 1), Some((5, 32)));
        assert_eq!(divider(1_000_000, 0), Some((0, 2)));
        assert_eq!(divider(50_000, 0), None);
        for code in 0..8u8 {
            assert_eq!(multiplier_from_code(code, 0), [2, 4, 6, 8, 12, 16, 20, 24][code as usize]);
        }
        assert_eq!(multiplier_from_code(8, 1), -1);
    }

    #[test]
    fn pll_557_142857() {
        // 557.142 MHz × 6 / 28.8 MHz = 116.07125 → N 0x74, fraction 0x123D.
        assert_eq!(pll_words(557_142, 6), (0x74, 0x123D));
        // Small fractions are raised to 0x0800.
        assert_eq!(pll_words(28_800, 1), (1, 0x0800));
        // Large ones are held at 0xF800.
        assert_eq!(pll_words(28_799, 1), (0, 0xF800));
    }

    #[test]
    fn tracking_filter() {
        // Column of band 3: ... 540, 575 MHz: 557.142 is nearer to 540.
        assert_eq!(track_index(557_142, 3), 7);
        assert_eq!(track_index(560_000, 3), 8);
        assert_eq!(track_index(470_000, 3), 5);
        assert_eq!(track_index(990_000, 3), 15);
    }

    #[test]
    fn if_filters() {
        assert_eq!(if_filter(6000, 4000).regs, [13, 5, 0, 30]);
        assert_eq!(if_filter(6000, 4570).regs, [13, 5, 0, 22]);
        assert_eq!(if_filter(8000, 2000).regs, [15, 62, 0, 30]);
        assert_eq!(if_filter(6000, -4000).regs, [13, 5, 0, 28]);
        assert_eq!(if_filter(6000, 0).regs, [13, 5, 0, 28]);
    }

    #[test]
    fn if_and_harmonic_steps() {
        // VCO 3342 MHz / (4 MHz × 48) = 17.4 → 17 → 34.
        assert_eq!(if_step(557_142, 2, 4000), 34);
        // 547 MHz is 3 MHz below the 19th crystal harmonic edge (550 MHz).
        assert_eq!(harmonic_band(547_000, 6000), 0x0D);
        // 557 MHz is above it.
        assert_eq!(harmonic_band(557_142, 6000), 0);
    }
}
