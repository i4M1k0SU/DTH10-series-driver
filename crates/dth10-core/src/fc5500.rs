//! The FC5500 terrestrial tuner of the TC90522 / TC90532 / TC90527 kits, as
//! the vendor drivers program it: every access is a raw I2C transfer to the
//! tuner (two-byte `{reg, value}` writes, a one-byte register select before
//! each one-byte read), through the demodulator's I2C repeater or directly on
//! the bridge bus ([`TunerBus`]).
//!
//! * Init: an 84-entry register table (bit6 of register 0x02 and bit1 of 0x44 from
//!   the descriptor's tuner options), after a `{00, 00}` write; the
//!   DT-H30/PCIEM-TS driver then runs a filter calibration.
//! * Tune: a fractional-N PLL, `N = f·M / ref`, `F = ((f·M % ref) << 16 + ref
//!   / 32) / (ref / 16) | 1` (f in kHz, M = 4 / 8 / 12 / 16 / 24 / 32 / 48 by
//!   band, ref = the tuner crystal in kHz, halved from 25 MHz on), with
//!   band-dependent RF registers and a VCO calibration (register 0x0D toggled
//!   five times). The two drivers write the same values in a different order
//!   and with different band registers.
//! * Signal strength from registers 0x71..0x74, in dBm (×1000).
//!
//! The series driver (5.1.9.33) has no tuner wake, standby or parameters; the
//! DT-H30/PCIEM-TS driver (5.1.9.46) has them, but its wake and standby only
//! act once parameter 4 has been set to a positive value.

use core::time::Duration;
use std::thread::sleep;

use crate::device::Device;
use crate::error::{Error, Result};
use crate::kit::{SignalStatus, VendorDriver, field};
use crate::protocol::UsbControl;
use crate::tlv::TunerDesc;
use crate::tuner_bus::TunerBus;

/// The init table, `(register, value)` in write order (before the option bits).
const INIT_TABLE: [(u8, u8); 84] = [
    (0x4C, 0x02),
    (0x09, 0x78),
    (0x0A, 0x40),
    (0x0D, 0x7F),
    (0x0F, 0x30),
    (0x12, 0x14),
    (0x15, 0x0C),
    (0x19, 0x47),
    (0x1A, 0x77),
    (0x13, 0xCE),
    (0x1C, 0x23),
    (0x25, 0x00),
    (0x2F, 0x04),
    (0x3E, 0x00),
    (0x32, 0x34),
    (0x2E, 0x08),
    (0x2E, 0x00),
    (0x38, 0xA2),
    (0x39, 0xA2),
    (0x3A, 0xAA),
    (0x3B, 0xAA),
    (0x3C, 0x04),
    (0x3D, 0x08),
    (0x27, 0x43),
    (0x03, 0x05),
    (0x08, 0x10),
    (0x44, 0xC0),
    (0x45, 0xD8),
    (0x46, 0xE0),
    (0x4B, 0xFF),
    (0x4F, 0x32),
    (0xC1, 0xC8),
    (0xC8, 0xC8),
    (0xCE, 0xC8),
    (0xA0, 0x08),
    (0xA3, 0x08),
    (0xA6, 0x08),
    (0x6D, 0x46),
    (0x6E, 0x1E),
    (0x5C, 0x58),
    (0xC8, 0xC8),
    (0xCC, 0x00),
    (0xC9, 0xEE),
    (0xC7, 0xA4),
    (0xCE, 0xC8),
    (0xD2, 0x00),
    (0xCF, 0xEE),
    (0xCD, 0xA4),
    (0xC1, 0xCE),
    (0xC6, 0x0E),
    (0xC2, 0x40),
    (0xC0, 0xA0),
    (0xD4, 0x80),
    (0xD5, 0x80),
    (0xD3, 0xC0),
    (0x5D, 0x0A),
    (0x5E, 0x01),
    (0x5F, 0x0A),
    (0x60, 0x01),
    (0x61, 0x0A),
    (0x62, 0x01),
    (0x63, 0x0A),
    (0x64, 0x01),
    (0x65, 0x0A),
    (0x66, 0x01),
    (0x67, 0x0A),
    (0x68, 0x01),
    (0x69, 0x1F),
    (0x6A, 0x01),
    (0x02, 0x9E),
    (0x28, 0x06),
    (0x50, 0x3C),
    (0x53, 0x3C),
    (0xA1, 0x2A),
    (0xA2, 0x20),
    (0xA4, 0x18),
    (0xA5, 0x12),
    (0xA7, 0x19),
    (0xA8, 0x11),
    (0x45, 0xE8),
    (0xC5, 0x28),
    (0xC0, 0x8E),
    (0x52, 0xFF),
    (0x45, 0xE8),
];

/// The init table with the descriptor's tuner options `opts` applied:
/// register 0x02 bit6 = `opts` bit1 (the series driver always clears it and
/// sets bit5 instead), register 0x44 bit1 = not `opts` bit2.
pub fn init_registers(driver: VendorDriver, opts: u32) -> [(u8, u8); 84] {
    let mut t = INIT_TABLE;
    for (reg, val) in t.iter_mut() {
        match *reg {
            0x08 => *val = 0x10,
            0x02 => {
                *val &= !0x40;
                *val |= match driver {
                    VendorDriver::Series => 0x20,
                    VendorDriver::Dth30 => ((opts & 2) << 5) as u8,
                };
            }
            0x44 if opts & 4 != 0 => *val &= !0x02,
            0x44 => *val |= 0x02,
            _ => {}
        }
    }
    t
}

/// The PLL's band of a frequency in kHz: the multiplier M and the band code
/// (written to register 0x0C below 450 MHz as `0x30 + code`).
pub fn band(khz: u32) -> (u32, u8) {
    match khz {
        450_000.. => (4, 0),
        237_000.. => (8, 0),
        158_000.. => (12, 1),
        118_500.. => (16, 2),
        79_000.. => (24, 3),
        59_300.. => (32, 6),
        _ => (48, 7),
    }
}

/// The PLL words for `khz` with a tuner crystal of `xtal_hz`: the integer
/// part N (register 0x1E) and the 20-bit fraction F with bit0 set (registers
/// 0x1F / 0x20 / 0x21 = `F >> 12`, `F >> 4`, `F << 4`). `None` when the
/// crystal is too low to divide by.
pub fn pll_words(khz: u32, xtal_hz: u32) -> Option<(u32, u32)> {
    let (m, _) = band(khz);
    let r = if xtal_hz < 25_000_000 { 1 } else { 2 };
    let rf = xtal_hz / 1000 / r;
    if rf >> 4 == 0 {
        return None;
    }
    let v = khz.wrapping_mul(m);
    let n = v / rf;
    let frac = ((((v % rf) << 16).wrapping_add(rf >> 5)) / (rf >> 4)) | 1;
    Some((n, frac))
}

/// The series driver's tune writes for `khz` and the bandwidth `bw_hz`, in
/// order: one fixed list of registers whose values depend on the band, the
/// bandwidth (register 0x32: 0x54 for 6 MHz, 0x4E for 8 MHz, else 0x34) and
/// the PLL words; 447143 kHz gets its own 0x15 / 0x27 values.
pub fn tune_writes_series(khz: u32, bw_hz: u32, n: u32, frac: u32) -> [(u8, u8); 13] {
    let (_, code) = band(khz);
    let uhf = khz >= 450_000;
    let r32 = match bw_hz {
        6_000_000 => 0x54,
        8_000_000 => 0x4E,
        _ => 0x34,
    };
    let r0a = if khz > 450_000 && khz <= 470_000 { 0x80 } else { 0x40 };
    let (r15, r27) = if khz == 447_143 { (0x06, 0x47) } else { (0x0C, 0x43) };
    [
        (0x02, if uhf { 0x9E } else { 0x1E }),
        (0x0C, if uhf { 0x20 } else { 0x30 + code }),
        (0x0A, r0a),
        (0x15, r15),
        (0x1E, n as u8),
        (0x1F, (frac >> 12) as u8),
        (0x20, (frac >> 4) as u8),
        (0x21, (frac << 4) as u8),
        (0x26, 0xE1),
        (0x27, r27),
        (0x32, r32),
        (0x5D, if uhf && khz >= 786_000 { 0x0F } else { 0x0A }),
        (0x32, r32),
    ]
}

/// The DT-H30/PCIEM-TS driver's band registers after the PLL lock, in order,
/// for `khz` with tuner options `opts` (bit1 = the LNA switch of register
/// 0x02 bit6).
pub fn band_writes_dth30(khz: u32, opts: u32) -> Vec<(u8, u8)> {
    let lna = opts & 2 != 0;
    if khz < 450_000 {
        let (_, code) = band(khz);
        return vec![(0x02, if lna { 0x5E } else { 0x1E }), (0x0C, 0x30 + code)];
    }
    let (r26, r28, r5d) = match khz {
        786_000.. => (0x59, 0x0A, 0x0F),
        674_000.. => (0x59, 0x0A, 0x0A),
        620_000.. => (0x59, 0x06, 0x0A),
        542_000.. => (0x51, 0x06, 0x0A),
        _ => (0x49, 0x06, 0x0A),
    };
    vec![(0x02, if lna { 0xDE } else { 0x9E }), (0x26, r26), (0x28, r28), (0x5D, r5d), (0x0A, 0x40), (0x0C, 0x20)]
}

/// The attenuation step of register 0x71 (`0` when the register is 0).
fn gain_step(r71: u8) -> u8 {
    match r71 {
        0 => 0,
        1 => 6,
        2 => 12,
        3 => 18,
        4 => 24,
        5 => 30,
        _ => 37,
    }
}

/// The series driver's signal strength in dBm (×1000) from registers 0x71..0x74,
/// computed in 8 bits as the driver does:
/// `step(0x71) − 0x74 / 4 + 6·(0x73 & 7) + 0x72 − 75`.
pub fn strength_series(r71: u8, r72: u8, r73: u8, r74: u8) -> i32 {
    let v = gain_step(r71)
        .wrapping_sub(r74 / 4)
        .wrapping_add(6 * (r73 & 7))
        .wrapping_add(r72)
        .wrapping_sub(85)
        .wrapping_add(10);
    v as i8 as i32 * 1000
}

/// The DT-H30/PCIEM-TS driver's signal level (dBm) from registers 0x71..0x74:
/// `step(0x71) − 0x74 / 4 + 6·(0x73 & 7) + 0x72 − 85`.
pub fn level_dth30(r71: u8, r72: u8, r73: u8, r74: u8) -> i32 {
    gain_step(r71) as i32 - (r74 / 4) as i32 + (6 * (r73 & 7)) as i32 + r72 as i32 - 85
}

/// The DT-H30/PCIEM-TS driver's LNA follow-up for a signal level `level`
/// (dBm) when the descriptor's tuner options have bit1 set: by the LNA state in
/// register 0x02 bits 7:6, the reported level (dBm) and the `{02, 5F}` values
/// to switch it to, if the level calls for a switch.
pub fn lna_dth30(level: i32, r02: u8) -> (i32, Option<[u8; 2]>) {
    match (r02 >> 7, (r02 >> 6) & 1) {
        (1, 1) => (level, (level > -20).then_some([0x9E, 0x10])),
        (1, _) => (level + 35, (level < -60).then_some([0xDE, 0x0A])),
        (_, 1) => (level - 10, (level > -20).then_some([0x1E, 0x0A])),
        _ => (level + 35, (level < -70).then_some([0x5E, 0x0A])),
    }
}

/// The FC5500 tuner.
#[derive(Debug, Clone, Copy)]
pub struct Fc5500 {
    pub bus: TunerBus,
    /// Which vendor driver's code to follow.
    pub driver: VendorDriver,
    /// The descriptor's tuner crystal (Hz), taken at [`init`](Self::init).
    xtal_hz: u32,
    /// The descriptor's tuner option bits, taken at [`init`](Self::init).
    opts: u32,
    /// Parameter 4: wake and standby act only when it is positive.
    power_control: i8,
    /// Parameter 5: report the level of parameter 8 instead of the measured one.
    level_fixed: bool,
    /// Parameter 8: the level (dBm) reported while parameter 5 is set.
    level_value: i32,
}

impl Fc5500 {
    pub fn new(bus: TunerBus, driver: VendorDriver) -> Self {
        Fc5500 { bus, driver, xtal_hz: 0, opts: 0, power_control: 0, level_fixed: false, level_value: -100 }
    }

    fn write<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8, val: u8) -> Result<()> {
        self.bus.write(dev, &[reg, val])
    }

    fn read<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8) -> Result<u8> {
        self.bus.write(dev, &[reg])?;
        let mut b = [0u8; 1];
        self.bus.read(dev, &mut b)?;
        Ok(b[0])
    }

    /// Init: takes the descriptor's tuner crystal and options, reads register
    /// 0x01 (the value is not used), writes `{00, 00}` and the init table
    /// ([`init_registers`]); the DT-H30/PCIEM-TS driver then calibrates the
    /// filter (`32 <- 54`, `2E <- 08`, then up to five times 5 ms and register
    /// 0x34 until bits 0 and 4 are set, restarting with `2E <- 00, 08`, and
    /// `2E <- 00` at the end).
    pub fn init<T: UsbControl>(&mut self, dev: &mut Device<T>, desc: &TunerDesc) -> Result<()> {
        self.xtal_hz = desc.tuner_xtal_hz;
        self.opts = desc.tuner_opts;
        let _ = self.read(dev, 0x01);
        self.write(dev, 0x00, 0x00)?;
        for (reg, val) in init_registers(self.driver, self.opts) {
            self.write(dev, reg, val)?;
        }
        if self.driver == VendorDriver::Dth30 {
            self.calibrate(dev)?;
        }
        Ok(())
    }

    fn calibrate<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.write(dev, 0x32, 0x54)?;
        self.write(dev, 0x2E, 0x08)?;
        for _ in 0..5 {
            sleep(Duration::from_millis(5));
            if self.read(dev, 0x34)? & 0x11 == 0x11 {
                break;
            }
            self.write(dev, 0x2E, 0x00)?;
            self.write(dev, 0x2E, 0x08)?;
        }
        self.write(dev, 0x2E, 0x00)
    }

    /// Wake (DT-H30/PCIEM-TS driver, only while parameter 4 is positive):
    /// register 0x02 bit0 cleared and bit6 set when the tuner options have
    /// bit1, then `06 <- F6`. Nothing with the series driver.
    pub fn wake<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        if self.driver != VendorDriver::Dth30 || self.power_control <= 0 {
            return Ok(());
        }
        let r02 = self.read(dev, 0x02)?;
        self.read(dev, 0x06)?;
        let v = (r02 & !0x01) | if self.opts & 2 != 0 { 0x40 } else { 0 };
        self.write(dev, 0x02, v)?;
        self.write(dev, 0x06, 0xF6)
    }

    /// Standby (DT-H30/PCIEM-TS driver, only while parameter 4 is positive):
    /// register 0x02 bit6 cleared, bit0 set. Nothing with the series driver.
    pub fn standby<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        if self.driver != VendorDriver::Dth30 || self.power_control <= 0 {
            return Ok(());
        }
        let r02 = self.read(dev, 0x02)?;
        self.write(dev, 0x02, (r02 & !0x40) | 0x01)
    }

    /// Tune to `freq_hz` (the request's frequency, used in kHz). The series
    /// driver writes the list of [`tune_writes_series`] (with the bandwidth
    /// `bw_hz`); the DT-H30/PCIEM-TS driver ignores the bandwidth, writes the
    /// PLL words (0x1F, 0x20, 0x21, then 0x1E), waits for the lock (up to five
    /// times `0D <- 7C, 7F`, 1 ms, register 0x11 non-zero), then writes the
    /// band registers ([`band_writes_dth30`]). Both then wait 10 ms, read back
    /// 0x1E..0x21 and 0x03 and run the VCO calibration (five times
    /// `0D <- 7C, 7F`, 1 ms, register 0x11); these last steps do not fail the
    /// tune.
    pub fn tune<T: UsbControl>(&self, dev: &mut Device<T>, freq_hz: u32, bw_hz: u32) -> Result<()> {
        let khz = freq_hz / 1000;
        let (n, frac) = pll_words(khz, self.xtal_hz)
            .ok_or_else(|| Error::Unsupported(format!("FC5500 tuner crystal {} Hz", self.xtal_hz)))?;
        match self.driver {
            VendorDriver::Series => {
                for (reg, val) in tune_writes_series(khz, bw_hz, n, frac) {
                    self.write(dev, reg, val)?;
                }
            }
            VendorDriver::Dth30 => {
                self.write(dev, 0x1F, (frac >> 12) as u8)?;
                self.write(dev, 0x20, (frac >> 4) as u8)?;
                self.write(dev, 0x21, (frac << 4) as u8)?;
                self.write(dev, 0x1E, n as u8)?;
                for _ in 0..5 {
                    self.write(dev, 0x0D, 0x7C)?;
                    self.write(dev, 0x0D, 0x7F)?;
                    sleep(Duration::from_millis(1));
                    if self.read(dev, 0x11)? != 0 {
                        break;
                    }
                }
                for (reg, val) in band_writes_dth30(khz, self.opts) {
                    self.write(dev, reg, val)?;
                }
            }
        }
        sleep(Duration::from_millis(10));
        for reg in [0x1E, 0x1F, 0x20, 0x21, 0x03] {
            let _ = self.read(dev, reg);
        }
        for _ in 0..5 {
            let r = self.write(dev, 0x0D, 0x7C).and_then(|()| self.write(dev, 0x0D, 0x7F));
            if r.is_err() {
                break;
            }
            sleep(Duration::from_millis(1));
            if self.read(dev, 0x11).is_err() {
                break;
            }
        }
        Ok(())
    }

    /// The tuner's part of the signal status: the strength (`st.strength`,
    /// dBm ×1000, replacing the demodulator's) from registers 0x71..0x74 and
    /// 0x02. The series driver uses [`strength_series`] and then writes
    /// `03 <- 85, 24 <- C0` when register 0x71 is 6, else `03 <- 05, 24 <- 00`;
    /// the DT-H30/PCIEM-TS driver uses [`level_dth30`] (or parameter 8 while
    /// parameter 5 is set) and, with tuner options bit1, [`lna_dth30`]. Failed
    /// reads and writes are not reported, as in the vendor drivers.
    pub fn status<T: UsbControl>(&self, dev: &mut Device<T>, st: &mut SignalStatus) -> Result<()> {
        let mut rd = |reg| self.read(dev, reg).unwrap_or(0);
        let (r71, r72, r73, r74, r02) = (rd(0x71), rd(0x72), rd(0x73), rd(0x74), rd(0x02));
        let strength = match self.driver {
            VendorDriver::Series => {
                let (r03, r24) = if r71 == 6 { (0x85, 0xC0) } else { (0x05, 0x00) };
                let _ = self.write(dev, 0x03, r03).and_then(|()| self.write(dev, 0x24, r24));
                strength_series(r71, r72, r73, r74)
            }
            VendorDriver::Dth30 => {
                let level = if self.level_fixed { self.level_value } else { level_dth30(r71, r72, r73, r74) };
                let level = if self.opts & 2 != 0 {
                    let (level, switch) = lna_dth30(level, r02);
                    if let Some([v02, v5f]) = switch {
                        let _ = self.write(dev, 0x02, v02).and_then(|()| self.write(dev, 0x5F, v5f));
                    }
                    level
                } else {
                    level
                };
                level.wrapping_mul(1000)
            }
        };
        st.mask |= field::STRENGTH;
        st.strength = strength;
        Ok(())
    }

    /// A kit parameter the demodulator did not take (DT-H30/PCIEM-TS driver):
    /// 1 = register 0x06 `B6` (value non-zero) / `F6` (zero), 4 = wake /
    /// standby on (positive) or off, 5 = report the level of parameter 8,
    /// 8 = that level (dBm). `Ok(false)`: not one of these (or the series
    /// driver, which passes no parameter to the tuner).
    pub fn set_param<T: UsbControl>(&mut self, dev: &mut Device<T>, mode: u32, value: u32) -> Result<bool> {
        if self.driver != VendorDriver::Dth30 {
            return Ok(false);
        }
        match mode {
            1 => self.write(dev, 0x06, if value != 0 { 0xB6 } else { 0xF6 })?,
            4 => self.power_control = value as u8 as i8,
            5 => self.level_fixed = value as u8 != 0,
            8 => self.level_value = value as i32,
            _ => return Ok(false),
        }
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::cmd;
    use crate::regmock::{RegMock, Write};
    use crate::tuner_bus::TunerPath;
    use core::cell::RefCell;
    use std::collections::HashMap;

    /// One transfer to the tuner as seen on a direct bus: a write, or a
    /// one-byte read (of the register selected by the write before it).
    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Op {
        W(Vec<u8>),
        R,
    }

    /// A tuner on the bridge bus: answers one-byte reads from its register
    /// file, by the register the last one-byte write selected.
    #[derive(Default)]
    struct Sim {
        ops: RefCell<Vec<Op>>,
        regs: HashMap<u8, u8>,
        sel: RefCell<u8>,
    }

    impl UsbControl for Sim {
        fn control_out(&self, _: u8, value: u16, index: u16, data: &[u8], _: Duration) -> Result<usize> {
            if value & 0xFF00 == cmd::I2C_RAW && index == cmd::I2C_RAW_WRITE {
                if data.len() == 1 {
                    *self.sel.borrow_mut() = data[0];
                }
                self.ops.borrow_mut().push(Op::W(data.to_vec()));
            }
            Ok(data.len())
        }
        fn control_in(&self, _: u8, value: u16, index: u16, buf: &mut [u8], _: Duration) -> Result<usize> {
            if value & 0xFF00 == cmd::I2C_RAW && index == cmd::I2C_RAW_READ {
                self.ops.borrow_mut().push(Op::R);
                buf.fill(*self.regs.get(&*self.sel.borrow()).unwrap_or(&0));
            } else {
                buf.fill(0);
            }
            Ok(buf.len())
        }
    }

    fn w(d: &[u8]) -> Vec<Op> {
        vec![Op::W(d.to_vec())]
    }

    fn r(reg: u8) -> Vec<Op> {
        vec![Op::W(vec![reg]), Op::R]
    }

    const ADDR: u8 = 0x63;

    fn tuner(driver: VendorDriver, regs: &[(u8, u8)]) -> (Fc5500, Device<Sim>) {
        let sim = Sim { regs: regs.iter().copied().collect(), ..Default::default() };
        (Fc5500::new(TunerBus { tuner_addr: ADDR, path: TunerPath::Direct }, driver), Device::new(sim))
    }

    fn ops(dev: &Device<Sim>) -> Vec<Op> {
        std::mem::take(&mut *dev.transport().ops.borrow_mut())
    }

    fn desc(opts: u32) -> TunerDesc {
        TunerDesc { tuner_xtal_hz: 16_000_000, tuner_opts: opts, ..Default::default() }
    }

    fn table_writes(opts: u32, series: bool) -> Vec<Op> {
        // The table as the model has it, with the option bits applied.
        let mut v = Vec::new();
        for &(reg, val) in INIT_TABLE.iter() {
            let val = match reg {
                0x02 if series => (val & 0xBF) | 0x20,
                0x02 => (val & 0xBF) | ((opts as u8 & 2) << 5),
                0x44 if opts & 4 != 0 => val & 0xFD,
                0x44 => val | 2,
                _ => val,
            };
            v.extend(w(&[reg, val]));
        }
        v
    }

    #[test]
    fn init_table_is_the_drivers() {
        assert_eq!(INIT_TABLE.len(), 84);
        assert_eq!(INIT_TABLE[0], (0x4C, 0x02));
        assert_eq!(INIT_TABLE[69], (0x02, 0x9E));
        assert_eq!(INIT_TABLE[83], (0x45, 0xE8));
        // register 0x02 and 0x44 by driver and options (the model's values)
        for (driver, opts, r02, r44) in [
            (VendorDriver::Series, 0, 0xBE, 0xC2),
            (VendorDriver::Series, 2, 0xBE, 0xC2),
            (VendorDriver::Series, 4, 0xBE, 0xC0),
            (VendorDriver::Dth30, 0, 0x9E, 0xC2),
            (VendorDriver::Dth30, 2, 0xDE, 0xC2),
            (VendorDriver::Dth30, 6, 0xDE, 0xC0),
        ] {
            let t = init_registers(driver, opts);
            assert_eq!(t[69], (0x02, r02), "{driver:?} opts {opts}");
            assert_eq!(t[26], (0x44, r44), "{driver:?} opts {opts}");
            let diff = t.iter().zip(INIT_TABLE.iter()).filter(|(a, b)| a != b).count();
            assert!(diff <= 2, "{driver:?} opts {opts}");
        }
    }

    #[test]
    fn series_init_sequence() {
        let (mut t, mut dev) = tuner(VendorDriver::Series, &[]);
        t.init(&mut dev, &desc(4)).unwrap();
        let mut want = r(0x01);
        want.extend(w(&[0x00, 0x00]));
        want.extend(table_writes(4, true));
        assert_eq!(ops(&dev), want);
    }

    #[test]
    fn dth30_init_sequence_with_calibration() {
        // the calibration flag never comes: five rounds
        let (mut t, mut dev) = tuner(VendorDriver::Dth30, &[]);
        t.init(&mut dev, &desc(2)).unwrap();
        let mut want = r(0x01);
        want.extend(w(&[0x00, 0x00]));
        want.extend(table_writes(2, false));
        want.extend(w(&[0x32, 0x54]));
        want.extend(w(&[0x2E, 0x08]));
        for _ in 0..5 {
            want.extend(r(0x34));
            want.extend(w(&[0x2E, 0x00]));
            want.extend(w(&[0x2E, 0x08]));
        }
        want.extend(w(&[0x2E, 0x00]));
        assert_eq!(ops(&dev), want);
        // the flag (bits 0 and 4) ends it at once
        let (mut t, mut dev) = tuner(VendorDriver::Dth30, &[(0x34, 0x31)]);
        t.init(&mut dev, &desc(0)).unwrap();
        let o = ops(&dev);
        let tail: Vec<Op> = [w(&[0x32, 0x54]), w(&[0x2E, 0x08]), r(0x34), w(&[0x2E, 0x00])].concat();
        assert_eq!(&o[o.len() - tail.len()..], &tail[..]);
        // bit0 alone is not enough
        let (mut t, mut dev) = tuner(VendorDriver::Dth30, &[(0x34, 0x01)]);
        t.init(&mut dev, &desc(0)).unwrap();
        assert_eq!(ops(&dev).iter().filter(|o| **o == Op::W(vec![0x34])).count(), 5);
    }

    #[test]
    fn init_goes_through_the_repeater() {
        let mut dev = Device::new(RegMock::default());
        let mut t = Fc5500::new(
            TunerBus { tuner_addr: 0x63, path: TunerPath::ViaDemod { demod_addr: 0x10 } },
            VendorDriver::Series,
        );
        t.init(&mut dev, &desc(0)).unwrap();
        let wr = dev.transport().take_writes();
        assert_eq!(wr[0], Write::Raw(0x10, vec![0xFE, 0xC6, 0x01]));
        assert_eq!(wr[1], Write::Raw(0x10, vec![0xFE, 0xC7]));
        assert_eq!(wr[2], Write::Raw(0x10, vec![0xFE, 0xC6, 0x00, 0x00]));
        assert_eq!(wr[3], Write::Raw(0x10, vec![0xFE, 0xC6, 0x4C, 0x02]));
        assert_eq!(wr.len(), 3 + 84);
    }

    /// (kHz, crystal Hz, N, F) from the model.
    const PLL: [(u32, u32, u32, u32); 20] = [
        (473_142, 16_000_000, 118, 299_369),
        (557_142, 16_000_000, 139, 299_369),
        (767_142, 16_000_000, 191, 823_657),
        (447_143, 16_000_000, 223, 599_261),
        (449_999, 16_000_000, 224, 1_048_053),
        (450_000, 16_000_000, 112, 524_289),
        (236_999, 16_000_000, 177, 785_647),
        (237_000, 16_000_000, 118, 524_289),
        (157_999, 16_000_000, 157, 1_047_527),
        (158_000, 16_000_000, 118, 524_289),
        (118_499, 16_000_000, 177, 784_859),
        (118_500, 16_000_000, 118, 524_289),
        (78_999, 16_000_000, 157, 1_046_479),
        (79_000, 16_000_000, 118, 524_289),
        (59_299, 16_000_000, 177, 940_573),
        (59_300, 16_000_000, 118, 629_147),
        (90_000, 16_000_000, 135, 1),
        (557_142, 24_000_000, 92, 898_631),
        (557_142, 27_000_000, 165, 83_029),
        (557_142, 36_000_000, 123, 848_649),
    ];

    #[test]
    fn pll_words_at_band_edges() {
        for (khz, xtal, n, f) in PLL {
            assert_eq!(pll_words(khz, xtal), Some((n, f)), "{khz} kHz, crystal {xtal}");
        }
        assert_eq!(pll_words(557_142, 0), None);
        assert_eq!(pll_words(557_142, 15_999), None);
        assert!(pll_words(557_142, 16_000).is_some());
    }

    #[test]
    fn bands() {
        for (khz, m, code) in [
            (770_000, 4, 0),
            (450_000, 4, 0),
            (449_999, 8, 0),
            (237_000, 8, 0),
            (236_999, 12, 1),
            (158_000, 12, 1),
            (157_999, 16, 2),
            (118_500, 16, 2),
            (118_499, 24, 3),
            (79_000, 24, 3),
            (78_999, 32, 6),
            (59_300, 32, 6),
            (59_299, 48, 7),
        ] {
            assert_eq!(band(khz), (m, code), "{khz} kHz");
        }
    }

    fn tail() -> Vec<Op> {
        let mut v = [r(0x1E), r(0x1F), r(0x20), r(0x21), r(0x03)].concat();
        for _ in 0..5 {
            v.extend([w(&[0x0D, 0x7C]), w(&[0x0D, 0x7F]), r(0x11)].concat());
        }
        v
    }

    #[test]
    fn series_tune_sequences() {
        // (Hz, bandwidth, the writes before the common tail) from the model
        let cases: [(u32, u32, [[u8; 2]; 13]); 5] = [
            (
                557_142_857,
                6_000_000,
                [
                    [0x02, 0x9E],
                    [0x0C, 0x20],
                    [0x0A, 0x40],
                    [0x15, 0x0C],
                    [0x1E, 0x8B],
                    [0x1F, 0x49],
                    [0x20, 0x16],
                    [0x21, 0x90],
                    [0x26, 0xE1],
                    [0x27, 0x43],
                    [0x32, 0x54],
                    [0x5D, 0x0A],
                    [0x32, 0x54],
                ],
            ),
            (
                767_142_857,
                8_000_000,
                [
                    [0x02, 0x9E],
                    [0x0C, 0x20],
                    [0x0A, 0x40],
                    [0x15, 0x0C],
                    [0x1E, 0xBF],
                    [0x1F, 0xC9],
                    [0x20, 0x16],
                    [0x21, 0x90],
                    [0x26, 0xE1],
                    [0x27, 0x43],
                    [0x32, 0x4E],
                    [0x5D, 0x0A],
                    [0x32, 0x4E],
                ],
            ),
            (
                447_143_000,
                6_000_000,
                [
                    [0x02, 0x1E],
                    [0x0C, 0x30],
                    [0x0A, 0x40],
                    [0x15, 0x06],
                    [0x1E, 0xDF],
                    [0x1F, 0x92],
                    [0x20, 0x4D],
                    [0x21, 0xD0],
                    [0x26, 0xE1],
                    [0x27, 0x47],
                    [0x32, 0x54],
                    [0x5D, 0x0A],
                    [0x32, 0x54],
                ],
            ),
            (
                461_000_000,
                7_000_000,
                [
                    [0x02, 0x9E],
                    [0x0C, 0x20],
                    [0x0A, 0x80],
                    [0x15, 0x0C],
                    [0x1E, 0x73],
                    [0x1F, 0x40],
                    [0x20, 0x00],
                    [0x21, 0x10],
                    [0x26, 0xE1],
                    [0x27, 0x43],
                    [0x32, 0x34],
                    [0x5D, 0x0A],
                    [0x32, 0x34],
                ],
            ),
            (
                790_000_000,
                6_000_000,
                [
                    [0x02, 0x9E],
                    [0x0C, 0x20],
                    [0x0A, 0x40],
                    [0x15, 0x0C],
                    [0x1E, 0xC5],
                    [0x1F, 0x80],
                    [0x20, 0x00],
                    [0x21, 0x10],
                    [0x26, 0xE1],
                    [0x27, 0x43],
                    [0x32, 0x54],
                    [0x5D, 0x0F],
                    [0x32, 0x54],
                ],
            ),
        ];
        for (hz, bw, writes) in cases {
            let (mut t, mut dev) = tuner(VendorDriver::Series, &[]);
            t.init(&mut dev, &desc(0)).unwrap();
            ops(&dev);
            t.tune(&mut dev, hz, bw).unwrap();
            let mut want: Vec<Op> = writes.iter().flat_map(|p| w(p)).collect();
            want.extend(tail());
            assert_eq!(ops(&dev), want, "{hz} Hz");
        }
    }

    #[test]
    fn series_band_registers() {
        for (khz, r02, r0c, r0a, r5d) in [
            (90_000, 0x1E, 0x33, 0x40, 0x0A),
            (200_000, 0x1E, 0x31, 0x40, 0x0A),
            (449_999, 0x1E, 0x30, 0x40, 0x0A),
            (450_000, 0x9E, 0x20, 0x40, 0x0A),
            (450_001, 0x9E, 0x20, 0x80, 0x0A),
            (470_000, 0x9E, 0x20, 0x80, 0x0A),
            (470_001, 0x9E, 0x20, 0x40, 0x0A),
            (785_999, 0x9E, 0x20, 0x40, 0x0A),
            (786_000, 0x9E, 0x20, 0x40, 0x0F),
        ] {
            let wr = tune_writes_series(khz, 6_000_000, 0, 0);
            assert_eq!((wr[0].1, wr[1].1, wr[2].1, wr[11].1), (r02, r0c, r0a, r5d), "{khz} kHz");
        }
        assert_eq!(tune_writes_series(447_142, 6_000_000, 0, 0)[3], (0x15, 0x0C));
        assert_eq!(tune_writes_series(447_143, 6_000_000, 0, 0)[9], (0x27, 0x47));
    }

    #[test]
    fn dth30_tune_sequences() {
        // (Hz, options, PLL writes, band writes) from the model; register 0x11 = 1 locks at once
        type Case = (u32, u32, [u8; 4], Vec<[u8; 2]>);
        let cases: [Case; 4] = [
            (
                557_142_857,
                0,
                [0x49, 0x16, 0x90, 0x8B],
                vec![[0x02, 0x9E], [0x26, 0x51], [0x28, 0x06], [0x5D, 0x0A], [0x0A, 0x40], [0x0C, 0x20]],
            ),
            (
                767_142_857,
                2,
                [0xC9, 0x16, 0x90, 0xBF],
                vec![[0x02, 0xDE], [0x26, 0x59], [0x28, 0x0A], [0x5D, 0x0A], [0x0A, 0x40], [0x0C, 0x20]],
            ),
            (
                473_142_857,
                0,
                [0x49, 0x16, 0x90, 0x76],
                vec![[0x02, 0x9E], [0x26, 0x49], [0x28, 0x06], [0x5D, 0x0A], [0x0A, 0x40], [0x0C, 0x20]],
            ),
            (200_000_000, 2, [0x00, 0x00, 0x10, 0x96], vec![[0x02, 0x5E], [0x0C, 0x31]]),
        ];
        for (hz, opts, pll, band) in cases {
            let (mut t, mut dev) = tuner(VendorDriver::Dth30, &[(0x11, 1), (0x34, 0x11)]);
            t.init(&mut dev, &desc(opts)).unwrap();
            ops(&dev);
            t.tune(&mut dev, hz, 6_000_000).unwrap();
            let mut want = [w(&[0x1F, pll[0]]), w(&[0x20, pll[1]]), w(&[0x21, pll[2]]), w(&[0x1E, pll[3]])].concat();
            want.extend([w(&[0x0D, 0x7C]), w(&[0x0D, 0x7F]), r(0x11)].concat());
            want.extend(band.iter().flat_map(|p| w(p)));
            want.extend(tail());
            assert_eq!(ops(&dev), want, "{hz} Hz");
        }
        // no lock: five rounds before the band registers
        let (mut t, mut dev) = tuner(VendorDriver::Dth30, &[(0x34, 0x11)]);
        t.init(&mut dev, &desc(0)).unwrap();
        ops(&dev);
        t.tune(&mut dev, 557_142_857, 8_000_000).unwrap();
        let o = ops(&dev);
        assert_eq!(o.iter().filter(|o| **o == Op::W(vec![0x11])).count(), 10);
        assert_eq!(o[4 + 20], Op::W(vec![0x02, 0x9E]));
    }

    #[test]
    fn dth30_band_registers() {
        for (khz, r26, r28, r5d) in [
            (450_000, 0x49, 0x06, 0x0A),
            (541_999, 0x49, 0x06, 0x0A),
            (542_000, 0x51, 0x06, 0x0A),
            (619_999, 0x51, 0x06, 0x0A),
            (620_000, 0x59, 0x06, 0x0A),
            (673_999, 0x59, 0x06, 0x0A),
            (674_000, 0x59, 0x0A, 0x0A),
            (785_999, 0x59, 0x0A, 0x0A),
            (786_000, 0x59, 0x0A, 0x0F),
        ] {
            assert_eq!(
                band_writes_dth30(khz, 0),
                vec![(0x02, 0x9E), (0x26, r26), (0x28, r28), (0x5D, r5d), (0x0A, 0x40), (0x0C, 0x20)],
                "{khz} kHz"
            );
        }
        assert_eq!(band_writes_dth30(449_999, 0), vec![(0x02, 0x1E), (0x0C, 0x30)]);
        assert_eq!(band_writes_dth30(59_299, 2), vec![(0x02, 0x5E), (0x0C, 0x37)]);
    }

    #[test]
    fn tune_needs_the_crystal() {
        let (t, mut dev) = tuner(VendorDriver::Dth30, &[]);
        std::assert_matches!(t.tune(&mut dev, 557_142_857, 6_000_000), Err(Error::Unsupported(_)));
        assert_eq!(ops(&dev), vec![]);
    }

    #[test]
    fn strength_formulas() {
        // (0x71, 0x72, 0x73, 0x74, series, DT-H30 level) from the model
        for (a, b, c, d, series, level) in [
            (0, 0, 0, 0, -75_000, -85),
            (1, 40, 3, 20, -16_000, -26),
            (6, 10, 7, 200, -36_000, -46),
            (5, 100, 2, 255, 4_000, -6),
            (0, 255, 0, 0, -76_000, 170),
            (3, 0, 7, 3, -15_000, -25),
        ] {
            assert_eq!(strength_series(a, b, c, d), series, "{a} {b} {c} {d}");
            assert_eq!(level_dth30(a, b, c, d), level, "{a} {b} {c} {d}");
        }
    }

    #[test]
    fn dth30_lna_follow_up() {
        for (level, r02, out, switch) in [
            (-10, 0xC0, -10, Some([0x9E, 0x10])),
            (-20, 0xC0, -20, None),
            (-61, 0x80, -26, Some([0xDE, 0x0A])),
            (-60, 0x80, -25, None),
            (-19, 0x40, -29, Some([0x1E, 0x0A])),
            (-20, 0x40, -30, None),
            (-71, 0x00, -36, Some([0x5E, 0x0A])),
            (-70, 0x00, -35, None),
        ] {
            assert_eq!(lna_dth30(level, r02), (out, switch), "{level} {r02:#x}");
        }
    }

    #[test]
    fn status_sequences() {
        let regs = [(0x71, 1), (0x72, 40), (0x73, 3), (0x74, 20), (0x02, 0x40)];
        let reads = [r(0x71), r(0x72), r(0x73), r(0x74), r(0x02)].concat();
        // series: the strength and the 03 / 24 writes
        let (t, mut dev) = tuner(VendorDriver::Series, &regs);
        let mut st = SignalStatus { mask: field::LOCK, strength: 5000, ..Default::default() };
        t.status(&mut dev, &mut st).unwrap();
        assert_eq!((st.strength, st.mask), (-16_000, field::LOCK | field::STRENGTH));
        assert_eq!(ops(&dev), [reads.clone(), w(&[0x03, 0x05]), w(&[0x24, 0x00])].concat());
        let (t, mut dev) = tuner(VendorDriver::Series, &[(0x71, 6)]);
        t.status(&mut dev, &mut st).unwrap();
        assert_eq!(&ops(&dev)[10..], &[w(&[0x03, 0x85]), w(&[0x24, 0xC0])].concat()[..]);
        // DT-H30, no LNA option: reads only
        let (mut t, mut dev) = tuner(VendorDriver::Dth30, &regs);
        t.status(&mut dev, &mut st).unwrap();
        assert_eq!((st.strength, ops(&dev)), (-26_000, reads.clone()));
        // with the option: LNA state 01 at -26 dBm reports -36, no switch
        t.init(&mut dev, &desc(2)).unwrap();
        ops(&dev);
        t.status(&mut dev, &mut st).unwrap();
        assert_eq!((st.strength, ops(&dev)), (-36_000, reads.clone()));
        // at -6 dBm it reports -16 and switches
        let (mut t2, mut dev2) =
            tuner(VendorDriver::Dth30, &[(0x71, 5), (0x72, 100), (0x73, 2), (0x74, 255), (0x02, 0x40)]);
        t2.init(&mut dev2, &desc(2)).unwrap();
        ops(&dev2);
        t2.status(&mut dev2, &mut st).unwrap();
        assert_eq!(st.strength, -16_000);
        assert_eq!(ops(&dev2), [reads.clone(), w(&[0x02, 0x1E]), w(&[0x5F, 0x0A])].concat());
        // parameters 5 and 8: a fixed level
        assert!(t.set_param(&mut dev, 8, (-50i32) as u32).unwrap());
        assert!(t.set_param(&mut dev, 5, 1).unwrap());
        t.status(&mut dev, &mut st).unwrap();
        assert_eq!(st.strength, -60_000);
        assert!(t.set_param(&mut dev, 5, 0x100).unwrap(), "only the low byte counts");
        t.status(&mut dev, &mut st).unwrap();
        assert_eq!(st.strength, -36_000);
    }

    #[test]
    fn parameters_wake_and_standby() {
        let (mut t, mut dev) = tuner(VendorDriver::Dth30, &[(0x02, 0x9F), (0x06, 0x12)]);
        t.init(&mut dev, &desc(2)).unwrap();
        ops(&dev);
        // off by default
        t.wake(&mut dev).unwrap();
        t.standby(&mut dev).unwrap();
        assert_eq!(ops(&dev), vec![]);
        assert!(t.set_param(&mut dev, 4, 1).unwrap());
        t.standby(&mut dev).unwrap();
        assert_eq!(ops(&dev), [r(0x02), w(&[0x02, 0x9F])].concat());
        t.wake(&mut dev).unwrap();
        assert_eq!(ops(&dev), [r(0x02), r(0x06), w(&[0x02, 0xDE]), w(&[0x06, 0xF6])].concat());
        // a negative byte switches them off again
        assert!(t.set_param(&mut dev, 4, 0x80).unwrap());
        t.wake(&mut dev).unwrap();
        assert_eq!(ops(&dev), vec![]);
        // parameter 1: register 0x06
        assert!(t.set_param(&mut dev, 1, 1).unwrap());
        assert!(t.set_param(&mut dev, 1, 0).unwrap());
        assert_eq!(ops(&dev), [w(&[0x06, 0xB6]), w(&[0x06, 0xF6])].concat());
        assert!(!t.set_param(&mut dev, 0, 1).unwrap());
        assert!(!t.set_param(&mut dev, 3, 1).unwrap());
        // the series driver has none of it
        let (mut t, mut dev) = tuner(VendorDriver::Series, &[]);
        assert!(!t.set_param(&mut dev, 1, 1).unwrap());
        assert!(!t.set_param(&mut dev, 4, 1).unwrap());
        t.wake(&mut dev).unwrap();
        t.standby(&mut dev).unwrap();
        assert_eq!(ops(&dev), vec![]);
    }
}
