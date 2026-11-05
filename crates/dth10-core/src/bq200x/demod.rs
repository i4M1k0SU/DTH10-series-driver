//! BQ200X ISDB-T demodulator, as the original driver's `BQ200X_*` kits drive it.
//!
//! The chip is reached with register transfers (`0x2D00|addr`, wIndex = register),
//! one byte at a time, all immediate (no batch). It works either alone or as a
//! master/slave pair (diversity: two chips, each with its own tuner on the same
//! channel); the pair is addressed with the descriptor's demodulator address
//! (master) and the byte of TLV tag 'F' (slave).
//!
//! What the driver does with it:
//!
//! * configuration (Init, or Open when the descriptor says so): clock / PLL words
//!   from the crystal, IF NCO from the IF, a fixed register set, the TS output
//!   bits from the stream parameter ('X'), a reset pulse on 0x1D;
//! * tune: the request is not looked at (the tuner gets the frequency); the
//!   chip's acquisition is restarted (0x01 bit4 pulse, then up to 30 × 10 ms
//!   waiting for 0x3A bit6) and the time is remembered;
//! * status: lock flags, a slow carrier-offset tracking on 0x69/0x6A, C/N from a
//!   lookup table, BER per layer and an error watchdog that restarts the
//!   acquisition when layer A keeps failing.
//!
//! The tuner I2C gate is register 0x02 bit1 of the master (cleared = open).

use core::time::Duration;
use std::thread::sleep;
use std::time::Instant;

use crate::device::Device;
use crate::error::{Error, Result};
use crate::kit::{SignalStatus, field};
use crate::protocol::UsbControl;
use crate::tlv::TunerDesc;

/// Lock flag bits of [`SignalStatus::lock`] as the BQ200X kits fill them
/// (same bit positions as the TC90517 kits, which the filter layer reads).
pub mod lock {
    /// Register 0x3A bit7 clear.
    pub const REG3A_BIT7_CLEAR: u32 = 0x001;
    /// Register 0x70 bit7 set (signal present; quality 20).
    pub const SIGNAL: u32 = 0x002;
    /// Register 0x92 bit0 and the TMCC / frame lock bit (single chip: 0xDB bit6,
    /// diversity master: 0xAB bit1): demodulator locked (quality 60).
    pub const LOCKED: u32 = 0x010;
    /// Register 0xEC bit0 (meaning unknown).
    pub const REGEC_BIT0: u32 = 0x100;
}

/// Crystal value that selects the second set of clock constants; every other
/// crystal value gets the first set.
pub const XTAL_19M2: u32 = 19_200_000;

/// Base clock constant of the clock / PLL words: 2^46 / 1000 for the default
/// crystal, 2^46 / 853.33 for 19.2 MHz.
pub fn clock_constant(xtal_hz: u32) -> i64 {
    if xtal_hz == XTAL_19M2 { 0x13_3333_3333 } else { 0x10_624D_D2F2 }
}

/// Registers 0x79..0x7C (big-endian, bit 31 cleared).
pub fn clock_word(xtal_hz: u32) -> u32 {
    let v6 = 63 * clock_constant(xtal_hz);
    ((v6 / 2048) as u32) & 0x7FFF_FFFF
}

/// Value of registers 0x64 (low nibble) / 0x65 / 0x66: `2^39 / round(63·2048·k / 2^31)`.
pub fn sample_word(xtal_hz: u32) -> u32 {
    let v6 = 63 * clock_constant(xtal_hz);
    let d = (((v6 + 0x80000) << 11) >> 31) as u32;
    ((1u64 << 39) / d as u64) as u32
}

/// Value of registers 0x6C (bits 1:0) / 0x6D.
pub fn timing_word(xtal_hz: u32) -> u16 {
    let v6 = 63 * clock_constant(xtal_hz);
    let d3 = ((v6 + (1 << 30)) >> 31) as u32;
    let t = 0x80000u32 / d3;
    ((((t as i64) << 32) / 411_763 + 0x8000) / 65536) as u16
}

/// IF NCO value (registers 0x68 bits 5:0 = bits 29:24, 0x69 = bits 23:16) for
/// an IF of `if_khz` kHz.
pub fn nco_word(xtal_hz: u32, if_khz: u32) -> u64 {
    let k: i64 = if xtal_hz == XTAL_19M2 { 39_322 } else { 33_554 };
    let d = ((1000 * k + 512) >> 10) as u32 as u64;
    ((if_khz as i64 as u64) << 30) / d
}

/// TS output configuration (register 0xF3) from the stream parameter ('X').
pub fn ts_output_bits(stream_param: u32) -> u8 {
    let x = stream_param;
    let mut v = !((x as u8) << 7) & 0x80;
    if x & 0x10 != 0 {
        v |= 0x10;
    }
    if x & 0x20 != 0 {
        v |= 0x08;
    }
    if x & 0x100 != 0 {
        v |= 0x04;
    }
    if x & 0x1000 != 0 {
        v |= 0x02;
    }
    v
}

/// C/N lookup table for layers with modulation code 0..2: `(largest raw value, C/N in 0.001 dB)`.
pub const CNR_TABLE_LOW: [(u16, i32); 71] = [
    (0x0045, 40000),
    (0x004D, 39500),
    (0x0056, 39000),
    (0x0061, 38500),
    (0x006D, 38000),
    (0x007A, 37500),
    (0x0089, 37000),
    (0x009A, 36500),
    (0x00AD, 36000),
    (0x00C2, 35500),
    (0x00DA, 35000),
    (0x00F4, 34500),
    (0x0112, 34000),
    (0x0134, 33500),
    (0x0159, 33000),
    (0x0183, 32500),
    (0x01B2, 32000),
    (0x0573, 31500),
    (0x059A, 31000),
    (0x05DC, 30500),
    (0x0622, 30000),
    (0x0682, 29500),
    (0x06E7, 29000),
    (0x071A, 28500),
    (0x0794, 28000),
    (0x081E, 27500),
    (0x0891, 27000),
    (0x08FC, 26500),
    (0x0960, 26000),
    (0x0A50, 25500),
    (0x0B59, 25000),
    (0x0C1C, 24500),
    (0x0CE4, 24000),
    (0x0E10, 23500),
    (0x0FA0, 23000),
    (0x1162, 22500),
    (0x1324, 22000),
    (0x1518, 21500),
    (0x173E, 21000),
    (0x1932, 20500),
    (0x1B58, 20000),
    (0x1E00, 19500),
    (0x2200, 19000),
    (0x2600, 18500),
    (0x2880, 18000),
    (0x2C80, 17500),
    (0x3080, 17000),
    (0x3500, 16500),
    (0x3980, 16000),
    (0x3E00, 15500),
    (0x4280, 15000),
    (0x4780, 14500),
    (0x4B80, 14000),
    (0x4F80, 13500),
    (0x5380, 13000),
    (0x5700, 12500),
    (0x5A00, 12000),
    (0x5D00, 11500),
    (0x6000, 11000),
    (0x6280, 10500),
    (0x6480, 10000),
    (0x6680, 9500),
    (0x6880, 9000),
    (0x6A00, 8500),
    (0x6B80, 8000),
    (0x6D00, 7500),
    (0x6E00, 7000),
    (0x6F00, 6500),
    (0x7000, 6000),
    (0x7100, 5500),
    (0xFFFF, 5000),
];

/// C/N lookup table for layers with modulation code 3..5.
pub const CNR_TABLE_HIGH: [(u16, i32); 53] = [
    (0x0048, 40000),
    (0x0051, 39500),
    (0x005B, 39000),
    (0x0066, 38500),
    (0x0073, 38000),
    (0x0081, 37500),
    (0x0090, 37000),
    (0x00A2, 36500),
    (0x00B6, 36000),
    (0x00CC, 35500),
    (0x00E5, 35000),
    (0x0101, 34500),
    (0x0120, 34000),
    (0x0143, 33500),
    (0x016B, 33000),
    (0x0197, 32500),
    (0x01C9, 32000),
    (0x0201, 31500),
    (0x023F, 31000),
    (0x0285, 30500),
    (0x02D4, 30000),
    (0x032C, 29500),
    (0x0390, 29000),
    (0x03FF, 28500),
    (0x047B, 28000),
    (0x0507, 27500),
    (0x05A4, 27000),
    (0x0654, 26500),
    (0x071A, 26000),
    (0x07F8, 25500),
    (0x08F0, 25000),
    (0x0A06, 24500),
    (0x0B3C, 24000),
    (0x0C96, 23500),
    (0x0E15, 23000),
    (0x0FBA, 22500),
    (0x1185, 22000),
    (0x1374, 21500),
    (0x1582, 21000),
    (0x17A7, 20500),
    (0x19DF, 20000),
    (0x1C20, 19500),
    (0x1E63, 19000),
    (0x20A6, 18500),
    (0x22D7, 18000),
    (0x24E9, 17500),
    (0x26E8, 17000),
    (0x28C4, 16500),
    (0x2A7E, 16000),
    (0x2C14, 15500),
    (0x2E08, 15000),
    (0x3060, 14500),
    (0xFFFF, 14000),
];

/// C/N in 0.001 dB from the 16-bit raw value and the layer's modulation code (0..5).
pub fn cnr_mdb_from_raw(raw: u16, modulation: u32) -> i32 {
    let table: &[(u16, i32)] = if modulation <= 2 { &CNR_TABLE_LOW } else { &CNR_TABLE_HIGH };
    table.iter().find(|&&(max, _)| raw <= max).map(|&(_, v)| v).unwrap_or(5000)
}

/// BER ×1e-9 from an error count and a period in 204-byte packets (0 when the period is 0).
pub fn ber_e9(err: u32, period: u32) -> u32 {
    if period == 0 {
        return 0;
    }
    ((1_000_000_000u64 * err as u64) / (1632 * period) as u64) as u32
}

/// Modulation code of a layer from its 3-bit TMCC field: 0..3 as is, 4..6 → 4, 7 → 5.
pub fn modulation_code(field3: u8) -> u32 {
    match field3 & 7 {
        v @ 0..=3 => v as u32,
        4..=6 => 4,
        _ => 5,
    }
}

/// Code-rate code of a layer from its 3-bit TMCC field: 0..4 as is, 5..6 → 5, 7 → 6.
pub fn code_rate_code(field3: u8) -> u32 {
    match field3 & 7 {
        v @ 0..=4 => v as u32,
        5 | 6 => 5,
        _ => 6,
    }
}

/// Signed carrier-offset value from registers 0x70 (bits 3:0) and 0x71.
pub fn carrier_offset(r70: u8, r71: u8) -> i32 {
    let v = (((r70 & 7) as i32) << 16) + ((r71 as i32) << 8);
    if r70 & 0x08 != 0 { -(!v & 0x7FF00) } else { v }
}

/// The tuner I2C path: through the master's gate (register 0x02 bit1, cleared =
/// open) or, with descriptor flags bit3, directly on the bridge bus (no gate).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TunerGate {
    /// Demodulator whose gate is switched; `None` = tuner directly on the bus.
    pub demod: Option<u8>,
}

impl TunerGate {
    pub fn open<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        match self.demod {
            Some(a) => Bq200x::rmw(dev, a, 0x02, |v| v & 0xFD),
            None => Ok(()),
        }
    }

    pub fn close<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        match self.demod {
            Some(a) => Bq200x::rmw(dev, a, 0x02, |v| v | 0x02),
            None => Ok(()),
        }
    }

    /// Open, run `f`, close (whatever `f` returned; the close result is ignored).
    pub fn with<T: UsbControl, R>(
        &self,
        dev: &mut Device<T>,
        f: impl FnOnce(&mut Device<T>) -> Result<R>,
    ) -> Result<R> {
        self.open(dev)?;
        let r = f(dev);
        let _ = self.close(dev);
        r
    }
}

/// The BQ200X object (one chip, or a master/slave pair).
#[derive(Debug, Clone)]
pub struct Bq200x {
    /// Master (or only) chip: `TunerDesc::demod_i2c_addr`.
    pub addr: u8,
    /// Slave chip of a diversity pair: `TunerDesc::byte_46`.
    pub slave: Option<u8>,
    /// Capture the carrier-offset reference on the next status read.
    afc_capture: bool,
    afc_ref: i32,
    /// Carrier offset currently outside ±256 of the reference (kept, never reported).
    afc_drifting: bool,
    /// Code-rate dependent register 0xBE still to be written (master / slave).
    code_rate_pending: bool,
    slave_code_rate_pending: bool,
    /// Status reads while unlocked more than 3 s after the last tune.
    unlocked_polls: u8,
    /// Watchdog counters: layer A error flag (0xD9 bit2) and layer A error count >= 50.
    wd_flag: u8,
    wd_errors: u8,
    /// Diversity: polls with 0xAB bit3 clear (master / slave).
    master_ab: u8,
    slave_ab: u8,
    tuned_at: Option<Instant>,
}

impl Bq200x {
    pub fn new(addr: u8, slave: Option<u8>) -> Self {
        Bq200x {
            addr,
            slave,
            afc_capture: false,
            afc_ref: 0,
            afc_drifting: false,
            code_rate_pending: false,
            slave_code_rate_pending: false,
            unlocked_polls: 0,
            wd_flag: 0,
            wd_errors: 0,
            master_ab: 0,
            slave_ab: 0,
            tuned_at: None,
        }
    }

    pub fn is_diversity(&self) -> bool {
        self.slave.is_some()
    }

    fn rd<T: UsbControl>(dev: &mut Device<T>, addr: u8, reg: u8) -> Result<u8> {
        let mut b = [0u8; 1];
        dev.i2c_reg_read(addr, reg, &mut b)?;
        Ok(b[0])
    }

    fn wr<T: UsbControl>(dev: &mut Device<T>, addr: u8, reg: u8, v: u8) -> Result<()> {
        dev.i2c_reg_write(addr, reg, &[v])
    }

    fn rmw<T: UsbControl>(dev: &mut Device<T>, addr: u8, reg: u8, f: impl FnOnce(u8) -> u8) -> Result<()> {
        let v = Self::rd(dev, addr, reg)?;
        Self::wr(dev, addr, reg, f(v))
    }

    /// Configuration (Init / Open). Requires the IF in the descriptor; tag 'b'
    /// (`val_62`), when present, replaces it as the IF the chip is set to.
    pub fn configure<T: UsbControl>(&mut self, dev: &mut Device<T>, desc: &TunerDesc) -> Result<()> {
        if desc.present & 0x4000 == 0 {
            return Err(Error::Unsupported("BQ200X needs the IF frequency in the descriptor".into()));
        }
        let mut if_hz = desc.if_freq_hz.wrapping_abs();
        if desc.present & 0x20000 != 0 {
            if_hz = desc.val_62 as i32;
        }
        let if_khz = if_hz / 1000;
        let xtal = desc.demod_xtal_hz;
        let ts = ts_output_bits(desc.stream_param);
        match self.slave {
            None => {
                self.chip_config(dev, self.addr, xtal, 0, 0, if_khz)?;
                Self::wr(dev, self.addr, 0xF3, ts)?;
                Self::reset_pulse(dev, self.addr)
            }
            Some(slave) => {
                self.chip_config(dev, slave, xtal, 1, 1, if_khz)?;
                self.chip_config(dev, self.addr, xtal, 1, 2, if_khz)?;
                Self::wr(dev, self.addr, 0xC2, 0x2C)?;
                Self::wr(dev, self.addr, 0xC3, 0x0F)?;
                Self::wr(dev, self.addr, 0xF3, ts)?;
                Self::reset_pulse(dev, slave)?;
                Self::reset_pulse(dev, self.addr)
            }
        }
    }

    /// 0x1D bit5 low for 20 ms.
    fn reset_pulse<T: UsbControl>(dev: &mut Device<T>, addr: u8) -> Result<()> {
        let v = Self::rd(dev, addr, 0x1D)?;
        Self::wr(dev, addr, 0x1D, v & 0xDF)?;
        sleep(Duration::from_millis(20));
        Self::wr(dev, addr, 0x1D, v | 0x20)
    }

    /// Configuration of one chip. `role_a` / `role_b` go to register 0x00 bits 1:0 / 3:2
    /// (0/0 alone, 1/1 slave, 1/2 master). Ends with an acquisition restart.
    fn chip_config<T: UsbControl>(
        &mut self,
        dev: &mut Device<T>,
        a: u8,
        xtal: u32,
        role_a: u8,
        role_b: u8,
        if_khz: i32,
    ) -> Result<()> {
        self.afc_capture = true;
        self.afc_drifting = false;
        // The original ignores the result of the clock setup.
        let _ = Self::clock_setup(dev, a, xtal, if_khz < 0, if_khz != 0);
        Self::wr(dev, a, 0x1E, if if_khz != 0 { 0xCF } else { 0xEF })?;
        Self::wr(dev, a, 0x1F, 0x20)?;
        Self::wr(dev, a, 0x5A, 0x08)?;
        Self::wr(dev, a, 0xA2, 0x40)?;
        Self::wr(dev, a, 0xA3, 0x32)?;
        Self::wr(dev, a, 0x97, 0x00)?;
        Self::rmw(dev, a, 0xA5, |v| v & 0x7F)?;
        Self::wr(dev, a, 0x9E, 0x00)?;
        Self::wr(dev, a, 0x22, 0x0F)?;
        Self::rmw(dev, a, 0x00, |v| v & 0xBF)?;
        Self::rmw(dev, a, 0x00, |v| (v & 0xFC) | (role_a & 3))?;
        Self::rmw(dev, a, 0x00, |v| (v & 0xF3) | ((role_b & 3) << 2))?;
        // The original ignores the results of the next two steps.
        let _ = Self::nco_setup(dev, a, xtal, if_khz.unsigned_abs());
        let _ = Self::sample_setup(dev, a, xtal);
        Self::wr(dev, a, 0x95, 0x03)?;
        Self::rmw(dev, a, 0x56, |v| v & 0xF7)?;
        Self::wr(dev, a, 0x85, 0x02)?;
        Self::wr(dev, a, 0xBA, 0x12)?;
        Self::wr(dev, a, 0xBB, 0x11)?;
        Self::rmw(dev, a, 0x01, |v| v | 0x1F)?;
        Self::rmw(dev, a, 0x6C, |v| v | 0x0C)?;
        for (reg, v) in [(0x88, 0x08), (0x89, 0x00), (0x8A, 0x00), (0x8B, 0x08), (0x8C, 0x00), (0x8D, 0x00)] {
            Self::wr(dev, a, reg, v)?;
        }
        Self::rmw(dev, a, 0x1F, |v| v & 0xF7)?;
        Self::rmw(dev, a, 0x30, |v| v & 0x7F)?;
        for (reg, v) in [
            (0x29, 0x68),
            (0x2A, 0xAB),
            (0x2D, 0x20),
            (0x40, 0x2C),
            (0x75, 0xD4),
            (0xA7, 0x01),
            (0xB4, 0x50),
            (0xB5, 0x08),
            (0xF3, 0x80),
        ] {
            Self::wr(dev, a, reg, v)?;
        }
        Self::rmw(dev, a, 0x45, |v| (v & 0xDF) | 0x80)?;
        self.sync_start(dev, a)
    }

    /// Clock section: inversion / IF-present bits, fixed values, PLL words.
    fn clock_setup<T: UsbControl>(dev: &mut Device<T>, a: u8, xtal: u32, inverted: bool, if_set: bool) -> Result<()> {
        Self::rmw(dev, a, 0x03, |v| (v & 0xEF) | 0x20)?;
        Self::rmw(dev, a, 0x02, |v| if inverted { v & 0xFB } else { v | 0x04 })?;
        Self::rmw(dev, a, 0x56, |v| if inverted { v | 0x60 } else { v & 0x9F })?;
        Self::rmw(dev, a, 0x02, |v| if if_set { v | 0x08 } else { v & 0xF7 })?;
        Self::wr(dev, a, 0xB6, 0xC0)?;
        Self::wr(dev, a, 0x2F, 0x04)?;
        Self::wr(dev, a, 0x90, 0xE0)?;
        Self::wr(dev, a, 0x61, 0xA9)?;
        let r21 = Self::rd(dev, a, 0x21)?;
        let (r62, r63) = if r21 & 1 != 0 { (0x33, 0x33) } else { (0x35, 0x55) };
        Self::wr(dev, a, 0x62, r62)?;
        Self::wr(dev, a, 0x63, r63)?;
        Self::pll_setup(dev, a, xtal)?;
        Self::wr(dev, a, 0xF3, 0x80)?;
        Self::rmw(dev, a, 0x04, |v| v & 0xFE)
    }

    fn pll_setup<T: UsbControl>(dev: &mut Device<T>, a: u8, xtal: u32) -> Result<()> {
        let cw = clock_word(xtal);
        Self::wr(dev, a, 0x79, (cw >> 24) as u8)?;
        Self::wr(dev, a, 0x7A, (cw >> 16) as u8)?;
        Self::wr(dev, a, 0x7B, (cw >> 8) as u8)?;
        Self::wr(dev, a, 0x7C, cw as u8)?;
        Self::write_sample_word(dev, a, xtal)?;
        Self::wr(dev, a, 0x6E, 0x01)?;
        Self::wr(dev, a, 0x6F, 0xFF)?;
        let tw = timing_word(xtal);
        Self::rmw(dev, a, 0x6C, |v| (v & !0x03) | ((tw >> 8) as u8 & 0x03))?;
        Self::wr(dev, a, 0x6D, tw as u8)?;
        Self::rmw(dev, a, 0x56, |v| v | 0x80)?;
        Self::rmw(dev, a, 0x75, |v| v | 0x10)
    }

    fn write_sample_word<T: UsbControl>(dev: &mut Device<T>, a: u8, xtal: u32) -> Result<()> {
        let q = sample_word(xtal);
        Self::rmw(dev, a, 0x64, |v| (v & 0xE0) | ((q >> 16) as u8 & 0x0F))?;
        Self::wr(dev, a, 0x65, (q >> 8) as u8)?;
        Self::wr(dev, a, 0x66, q as u8)
    }

    fn nco_setup<T: UsbControl>(dev: &mut Device<T>, a: u8, xtal: u32, if_khz: u32) -> Result<()> {
        let q = nco_word(xtal, if_khz);
        Self::rmw(dev, a, 0x68, |v| (v & 0xC0) | ((q >> 24) as u8 & 0x3F))?;
        Self::wr(dev, a, 0x69, (q >> 16) as u8)?;
        Self::wr(dev, a, 0x6A, 0x80)?;
        Self::wr(dev, a, 0x6B, 0x00)
    }

    fn sample_setup<T: UsbControl>(dev: &mut Device<T>, a: u8, xtal: u32) -> Result<()> {
        Self::rmw(dev, a, 0x00, |v| v & 0xCF)?;
        Self::write_sample_word(dev, a, xtal)?;
        Self::rmw(dev, a, 0x5E, |v| v | 0x80)?;
        Self::rmw(dev, a, 0x71, |v| v | 0x10)
    }

    /// Acquisition restart: 0x29/0x2A = 68/AB, 0x01 bit4 low for 20 ms, then up to
    /// 30 × 10 ms for 0x3A bit6, which sets 0x45 bit5 and 0x29/0x2A = 66/66.
    /// Giving up after 30 polls is not an error.
    fn sync_start<T: UsbControl>(&mut self, dev: &mut Device<T>, a: u8) -> Result<()> {
        let r01 = Self::rd(dev, a, 0x01)?;
        Self::wr(dev, a, 0x29, 0x68)?;
        Self::wr(dev, a, 0x2A, 0xAB)?;
        Self::wr(dev, a, 0x01, r01 & 0xEF)?;
        sleep(Duration::from_millis(20));
        Self::wr(dev, a, 0x01, r01 | 0x10)?;
        self.wd_flag = 0;
        self.wd_errors = 0;
        for _ in 0..30 {
            sleep(Duration::from_millis(10));
            if Self::rd(dev, a, 0x3A)? & 0x40 != 0 {
                Self::rmw(dev, a, 0x45, |v| v | 0x20)?;
                Self::wr(dev, a, 0x29, 0x66)?;
                return Self::wr(dev, a, 0x2A, 0x66);
            }
        }
        Ok(())
    }

    /// Tune: restart the acquisition (slave first) and remember the time.
    pub fn restart<T: UsbControl>(&mut self, dev: &mut Device<T>) -> Result<()> {
        let r = match self.slave {
            None => self.sync_start(dev, self.addr),
            Some(slave) => self.sync_start(dev, slave).and_then(|_| self.sync_start(dev, self.addr)),
        };
        self.tuned_at = Some(Instant::now());
        r
    }

    /// Watchdog recovery: capture a new carrier-offset reference, restart the acquisition,
    /// write 0xBE again on the next status.
    fn resync<T: UsbControl>(&mut self, dev: &mut Device<T>) -> Result<()> {
        self.afc_capture = true;
        self.afc_drifting = false;
        match self.slave {
            None => self.sync_start(dev, self.addr)?,
            Some(slave) => {
                self.sync_start(dev, slave)?;
                self.sync_start(dev, self.addr)?;
                self.slave_code_rate_pending = true;
            }
        }
        self.code_rate_pending = true;
        self.unlocked_polls = 0;
        Ok(())
    }

    fn tmcc_locked<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<bool> {
        Ok(match self.slave {
            None => Self::rd(dev, self.addr, 0xDB)? & 0x40 != 0,
            Some(_) => Self::rd(dev, self.addr, 0xAB)? & 0x02 != 0,
        })
    }

    /// Modulation code of layer `sel` (0 = A, 1 = B, 2 = C) from 0xDA / 0xDB.
    fn layer_modulation<T: UsbControl>(dev: &mut Device<T>, a: u8, sel: u8) -> Result<u32> {
        let f = match sel {
            0 => Self::rd(dev, a, 0xDA)? >> 3,
            1 => Self::rd(dev, a, 0xDA)?,
            _ => Self::rd(dev, a, 0xDB)? >> 3,
        };
        Ok(modulation_code(f))
    }

    /// Code-rate code of layer `sel` from 0xDB / 0xDC.
    fn layer_code_rate<T: UsbControl>(dev: &mut Device<T>, a: u8, sel: u8) -> Result<u32> {
        let f = match sel {
            0 => Self::rd(dev, a, 0xDB)?,
            1 => Self::rd(dev, a, 0xDC)? >> 3,
            _ => Self::rd(dev, a, 0xDC)?,
        };
        Ok(code_rate_code(f))
    }

    /// C/N of layer `sel`: raw value from 0xAC/0xAD, 0xAE/0xAF or 0xB0/0xB1.
    fn cnr<T: UsbControl>(dev: &mut Device<T>, a: u8, sel: u8) -> Result<i32> {
        let m = Self::layer_modulation(dev, a, sel)?;
        let hi = 0xAC + 2 * sel;
        let raw = ((Self::rd(dev, a, hi)? as u16) << 8) | Self::rd(dev, a, hi + 1)? as u16;
        Ok(cnr_mdb_from_raw(raw, m))
    }

    /// BER of layer `sel`: 21-bit error count (0xCA.., 0xCD.., 0xD0..) over a
    /// 14-bit period (0xC4.., 0xC6.., 0xC8..).
    fn ber<T: UsbControl>(dev: &mut Device<T>, a: u8, sel: u8) -> Result<u32> {
        let e = 0xCA + 3 * sel;
        let p = 0xC4 + 2 * sel;
        let err = ((Self::rd(dev, a, e)? as u32 & 0x1F) << 16)
            | ((Self::rd(dev, a, e + 1)? as u32) << 8)
            | Self::rd(dev, a, e + 2)? as u32;
        let period = ((Self::rd(dev, a, p)? as u32 & 0x3F) << 8) | Self::rd(dev, a, p + 1)? as u32;
        Ok(ber_e9(err, period))
    }

    /// 14-bit error counter of a layer (0xD3/0xD4, 0xD5/0xD6, 0xD7/0xD8).
    fn error_count<T: UsbControl>(dev: &mut Device<T>, a: u8, sel: u8) -> Result<u32> {
        let r = 0xD3 + 2 * sel;
        Ok(((Self::rd(dev, a, r)? as u32 & 0x3F) << 8) | Self::rd(dev, a, r + 1)? as u32)
    }

    /// Writes 0xBE from the code rate of layer A (single-layer broadcast) or B once
    /// 0xDB bit6 is set. Returns `true` while it still has to be done.
    fn fix_code_rate<T: UsbControl>(dev: &mut Device<T>, a: u8) -> Result<bool> {
        if Self::rd(dev, a, 0xDB)? & 0x40 == 0 {
            return Ok(true);
        }
        let segments = Self::rd(dev, a, 0xDE)? & 0x0F;
        let rate = Self::layer_code_rate(dev, a, if segments == 13 { 0 } else { 1 })?;
        let v = match rate {
            0..=2 => 0x38,
            3 => 0x32,
            4 => 0x36,
            _ => return Ok(false),
        };
        Self::wr(dev, a, 0xBE, v)?;
        Ok(false)
    }

    /// 0x01 bit3 then bit2 pulsed low (diversity recovery of one chip).
    fn pulse_01<T: UsbControl>(dev: &mut Device<T>, a: u8) -> Result<()> {
        let v = Self::rd(dev, a, 0x01)?;
        Self::wr(dev, a, 0x01, v & 0xF7)?;
        Self::wr(dev, a, 0x01, v | 0x08)?;
        let v = Self::rd(dev, a, 0x01)?;
        Self::wr(dev, a, 0x01, v & 0xFB)?;
        Self::wr(dev, a, 0x01, v | 0x04)
    }

    /// Error watchdog. Only layer A is watched (the caller passes no other layer):
    /// more than 15 consecutive polls with 0xD9 bit2 set or with >= 50 errors
    /// restart the acquisition.
    fn watchdog<T: UsbControl>(&mut self, dev: &mut Device<T>, layer_a: bool, err_a: u32) -> Result<()> {
        let d9 = Self::rd(dev, self.addr, 0xD9)?;
        if layer_a {
            self.wd_flag = if d9 & 0x04 != 0 { self.wd_flag.wrapping_add(1) } else { 0 };
            self.wd_errors = if err_a >= 50 { self.wd_errors.wrapping_add(1) } else { 0 };
            if self.wd_flag > 15 || self.wd_errors > 15 {
                // The result of the recovery is not checked by the original.
                let _ = self.resync(dev);
                self.master_ab = 0;
                self.slave_ab = 0;
                self.wd_flag = 0;
                self.wd_errors = 0;
            }
        } else {
            self.wd_flag = 0;
            self.wd_errors = 0;
        }
        match self.slave {
            None => {
                if self.code_rate_pending {
                    self.code_rate_pending = Self::fix_code_rate(dev, self.addr)?;
                }
                Ok(())
            }
            Some(slave) => {
                if self.slave_code_rate_pending {
                    self.slave_code_rate_pending = Self::fix_code_rate(dev, slave)?;
                }
                if self.code_rate_pending {
                    self.code_rate_pending = Self::fix_code_rate(dev, self.addr)?;
                }
                let m = Self::rd(dev, self.addr, 0xAB)? & 0x08 != 0;
                self.master_ab = if m { 0 } else { self.master_ab.wrapping_add(1) };
                let s = Self::rd(dev, slave, 0xAB)? & 0x08 != 0;
                self.slave_ab = if s { 0 } else { self.slave_ab.wrapping_add(1) };
                if self.slave_ab > 4 {
                    self.slave_ab = 0;
                    Self::pulse_01(dev, slave)?;
                }
                if self.master_ab > 4 {
                    self.master_ab = 0;
                    Self::pulse_01(dev, self.addr)?;
                }
                Ok(())
            }
        }
    }

    /// Carrier-offset tracking: the first status after a configuration or a recovery
    /// captures the offset; later, a drift of more than 256 moves the 16-bit value
    /// of 0x69:0x6A one step against it.
    fn track_offset<T: UsbControl>(&mut self, dev: &mut Device<T>) -> Result<()> {
        let a = self.addr;
        let r70 = Self::rd(dev, a, 0x70)?;
        let r71 = Self::rd(dev, a, 0x71)?;
        let off = carrier_offset(r70, r71);
        if self.afc_capture {
            self.afc_ref = off;
            self.afc_capture = false;
            return Ok(());
        }
        let d = self.afc_ref.wrapping_sub(off);
        self.afc_drifting = true;
        if d > 256 {
            let lo = Self::rd(dev, a, 0x6A)?;
            if lo != 0 {
                Self::wr(dev, a, 0x6A, lo - 1)
            } else {
                let hi = Self::rd(dev, a, 0x69)?;
                Self::wr(dev, a, 0x69, hi.wrapping_sub(1))?;
                Self::wr(dev, a, 0x6A, 0xFF)
            }
        } else if d >= -256 {
            self.afc_drifting = false;
            Ok(())
        } else {
            let lo = Self::rd(dev, a, 0x6A)?;
            if lo != 0xFF {
                Self::wr(dev, a, 0x6A, lo + 1)
            } else {
                let hi = Self::rd(dev, a, 0x69)?;
                Self::wr(dev, a, 0x69, hi.wrapping_add(1))?;
                Self::wr(dev, a, 0x6A, 0x00)
            }
        }
    }

    /// Signal status. `st.mask` selects the BER layers; lock and quality are
    /// always produced, C/N / strength unless the chip has been unlocked for
    /// more than 3 s since the last tune (then only the lock flags are reported
    /// and every 7th such poll restarts the acquisition).
    pub fn status<T: UsbControl>(&mut self, dev: &mut Device<T>, st: &mut SignalStatus) -> Result<()> {
        let a = self.addr;
        let requested = st.mask;
        st.mask = 0;
        st.lock = 0;
        st.quality = 0;
        if Self::rd(dev, a, 0x3A)? & 0x80 == 0 {
            st.lock |= lock::REG3A_BIT7_CLEAR;
        }
        if Self::rd(dev, a, 0x70)? & 0x80 != 0 {
            st.lock |= lock::SIGNAL;
        }
        st.mask |= field::LOCK;
        if st.lock & lock::SIGNAL != 0 {
            st.quality = 20;
            st.mask |= field::QUALITY;
        }
        if Self::rd(dev, a, 0x92)? & 0x01 != 0 && self.tmcc_locked(dev)? {
            st.lock |= lock::LOCKED;
        }
        if st.lock & lock::LOCKED != 0 {
            st.mask |= field::QUALITY;
            st.quality = 60;
        } else {
            let elapsed = self.tuned_at.map_or(Duration::MAX, |t| t.elapsed());
            if elapsed > Duration::from_secs(3) {
                self.unlocked_polls = self.unlocked_polls.wrapping_add(1);
                if self.unlocked_polls > 6 {
                    let _ = self.resync(dev);
                }
                return Ok(());
            }
        }
        if Self::rd(dev, a, 0xEC)? & 0x01 != 0 {
            st.lock |= lock::REGEC_BIT0;
        }
        self.track_offset(dev)?;

        // Layer A holding all 13 segments: single-layer broadcast, C/N of layer A;
        // otherwise layer B's C/N unless it is 15 dB or less.
        let segments = Self::rd(dev, a, 0xDE)? & 0x0F;
        st.sync_state = segments;
        let (cn, layers) = if segments == 13 {
            (Self::cnr(dev, a, 0)?, 4u8)
        } else {
            let b = Self::cnr(dev, a, 1)?;
            (if b <= 15000 { Self::cnr(dev, a, 0)? } else { b }, 6u8)
        };
        st.mask |= field::STRENGTH | field::CNR;
        st.cnr_mdb = cn;
        st.strength = 18000 - cn;
        if requested & field::BER_A != 0 {
            st.ber[0] = Self::ber(dev, a, 0)?;
            st.mask |= field::BER_A;
        }
        if requested & field::BER_B != 0 {
            st.ber[1] = if layers & 2 != 0 { Self::ber(dev, a, 1)? } else { crate::kit::BER_UNUSED };
            st.mask |= field::BER_B;
        }
        if requested & field::BER_C != 0 {
            st.ber[2] = crate::kit::BER_UNUSED;
            st.mask |= field::BER_C;
        }
        let err_a = Self::error_count(dev, a, 0)?;
        if layers & 2 != 0 {
            // Read like the original (layer B's counter), not evaluated by the watchdog.
            let _ = Self::error_count(dev, a, 1)?;
        }
        self.watchdog(dev, true, err_a)?;
        if cn > 22000 {
            st.quality = 80;
        }
        if cn > 23000 {
            st.quality = 90;
        }
        if cn > 24000 {
            st.quality = 100;
        }
        st.mask |= field::QUALITY;
        Ok(())
    }

    /// Whether the carrier offset was found outside ±256 of the reference at the last status.
    pub fn offset_drifting(&self) -> bool {
        self.afc_drifting
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_words() {
        assert_eq!(clock_word(16_000_000), 0x0106_24DD);
        assert_eq!(clock_word(XTAL_19M2), 0x1733_3333);
        assert_eq!(sample_word(16_000_000), 0x1_FBEF);
        assert_eq!(sample_word(XTAL_19M2), 0x1_B170);
        assert_eq!(timing_word(16_000_000), 0x28);
        assert_eq!(timing_word(XTAL_19M2), 0x22);
    }

    #[test]
    fn nco_words() {
        assert_eq!(nco_word(16_000_000, 4000), 0x07D0_0000);
        assert_eq!(nco_word(XTAL_19M2, 4000), 0x06AA_AAAA);
        assert_eq!(nco_word(16_000_000, 57_000), 0x6F54_0000);
    }

    #[test]
    fn ts_bits() {
        assert_eq!(ts_output_bits(0), 0x80);
        assert_eq!(ts_output_bits(1), 0x00);
        assert_eq!(ts_output_bits(0x10), 0x90);
        assert_eq!(ts_output_bits(0x1131), 0x1E);
    }

    #[test]
    fn cnr_tables() {
        assert_eq!(cnr_mdb_from_raw(0, 0), 40000);
        assert_eq!(cnr_mdb_from_raw(0x0046, 1), 39500);
        assert_eq!(cnr_mdb_from_raw(0x0300, 2), 31500);
        assert_eq!(cnr_mdb_from_raw(0x0CE4, 0), 24000);
        assert_eq!(cnr_mdb_from_raw(0x0CE5, 0), 23500);
        assert_eq!(cnr_mdb_from_raw(0x7101, 0), 5000);
        assert_eq!(cnr_mdb_from_raw(0x0048, 3), 40000);
        assert_eq!(cnr_mdb_from_raw(0x0049, 5), 39500);
        assert_eq!(cnr_mdb_from_raw(0x0B3C, 4), 24000);
        assert_eq!(cnr_mdb_from_raw(0x3061, 3), 14000);
        assert_eq!(cnr_mdb_from_raw(0xFFFF, 3), 14000);
    }

    #[test]
    fn tmcc_codes() {
        assert_eq!((0..8).map(modulation_code).collect::<Vec<_>>(), vec![0, 1, 2, 3, 4, 4, 4, 5]);
        assert_eq!((0..8).map(code_rate_code).collect::<Vec<_>>(), vec![0, 1, 2, 3, 4, 5, 5, 6]);
    }

    #[test]
    fn offset_and_ber() {
        assert_eq!(carrier_offset(0x02, 0x10), 0x2_1000);
        assert_eq!(carrier_offset(0x0F, 0xFF), 0);
        assert_eq!(carrier_offset(0x0F, 0x00), -0xFF00);
        assert_eq!(carrier_offset(0x08, 0x00), -0x7FF00);
        assert_eq!(ber_e9(1632, 1), 1_000_000_000);
        assert_eq!(ber_e9(5, 0), 0);
    }
}
