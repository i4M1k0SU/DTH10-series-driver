//! MaxLinear MxL131RF / MxL135RF / MxL5007 tuner object, as the original
//! driver's kits drive it. See `docs/11-mxl13xrf.md`.
//!
//! All three tuner types share one code path in the original driver (the
//! type field is never read); behaviour differs only through `TunerDesc`
//! values (hardware version, IF, crystal, option bits). The register map
//! matches the Linux `mxl5007t` driver closely.

use core::time::Duration;
use std::thread::sleep;

use crate::device::Device;
use crate::error::{Error, Result};
use crate::protocol::UsbControl;
use crate::tlv::TunerDesc;
use crate::tuner_bus::TunerBus;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MxlType {
    MxL5007,
    MxL131RF,
    MxL135RF,
}

impl MxlType {
    /// Derive the type from the kit name suffix after `_`.
    pub fn from_kit_name(kit: &str) -> Option<Self> {
        let suffix = kit.split_once('_')?.1;
        if suffix.len() >= 7 && suffix[..7].eq_ignore_ascii_case("MxL5007") {
            Some(MxlType::MxL5007)
        } else if suffix.eq_ignore_ascii_case("MxL131RF") {
            Some(MxlType::MxL131RF)
        } else if suffix.eq_ignore_ascii_case("MxL135RF") {
            Some(MxlType::MxL135RF)
        } else {
            None
        }
    }
}

/// Initialisation table of the original driver (22 register pairs; `{0,0}` terminator omitted).
pub const INIT_TABLE: [(u8, u8); 22] = [
    (0x04, 0x00),
    (0x02, 0x00),
    (0x03, 0x08),
    (0x05, 0x00),
    (0x06, 0x10),
    (0x07, 0x0A),
    (0x08, 0x80),
    (0x09, 0x3F),
    (0x0A, 0x3F),
    (0x0B, 0x3F),
    (0x2E, 0x15),
    (0x30, 0x10),
    (0x45, 0x58),
    (0x48, 0x19),
    (0x52, 0x03),
    (0x53, 0x44),
    (0x6A, 0x4B),
    (0x76, 0x00),
    (0x78, 0x18),
    (0x7A, 0x17),
    (0x85, 0x06),
    (0x01, 0x01),
];

/// Fixed init table used by the `SMT_LJ101` hw 0x102 kit: no
/// TunerDesc patching; crystal code 9 (32 MHz) in 0x03/0x05, IF code 0
/// (4 MHz, not inverted) in 0x02, clock output enabled (0x03 bit3).
pub const INIT_TABLE_LJ101: [(u8, u8); 18] = [
    (0x00, 0x3F),
    (0x02, 0x00),
    (0x03, 0x98),
    (0x04, 0x00),
    (0x05, 0x09),
    (0x06, 0x10),
    (0x2E, 0x15),
    (0x30, 0x10),
    (0x45, 0x58),
    (0x48, 0x19),
    (0x52, 0x03),
    (0x53, 0x44),
    (0x6A, 0x4B),
    (0x76, 0x00),
    (0x78, 0x18),
    (0x7A, 0x17),
    (0x85, 0x06),
    (0x01, 0x01),
];

/// RF tune table (identical to Linux `reg_pair_rftune`).
pub const RFTUNE_TABLE: [(u8, u8); 10] = [
    (0x0F, 0x00),
    (0x0C, 0x15),
    (0x0D, 0x40),
    (0x0E, 0x0E),
    (0x1F, 0x87),
    (0x20, 0x1F),
    (0x21, 0x87),
    (0x22, 0x1F),
    (0x80, 0x01),
    (0x0F, 0x01),
];

fn patch(tab: &mut [(u8, u8)], reg: u8, f: impl Fn(u8) -> u8) {
    for p in tab.iter_mut() {
        if p.0 == reg {
            p.1 = f(p.1);
        }
    }
}

/// Build the patched init register list (the original driver's steps 1–7).
pub fn init_registers(desc: &TunerDesc) -> Result<[(u8, u8); 22]> {
    let mut t = INIT_TABLE;
    let p = desc.present;

    // 2. hardware version → reg 0x06
    if p & 0x1 != 0 {
        match desc.hw_version {
            0x101 => patch(&mut t, 0x06, |v| (v & 0xF2) | 0x12),
            0x102 => patch(&mut t, 0x06, |v| (v & 0xF0) | 0x10),
            _ => {}
        }
    }
    // 3. IF frequency → reg 0x02
    if p & 0x4000 != 0 {
        let (inv, if_hz) =
            if desc.if_freq_hz < 0 { (true, desc.if_freq_hz.unsigned_abs()) } else { (false, desc.if_freq_hz as u32) };
        if if_hz == 0 {
            return Err(Error::Unsupported("tuner IF frequency is 0".into()));
        }
        match if_hz {
            4_000_000 => patch(&mut t, 0x02, |v| v & 0xF0),
            5_000_000 => patch(&mut t, 0x02, |v| (v & 0xF4) | 0x04),
            6_000_000 => patch(&mut t, 0x02, |v| (v & 0xF6) | 0x06),
            44_000_000 => patch(&mut t, 0x02, |v| (v & 0xFB) | 0x0B),
            _ => {}
        }
        if inv {
            patch(&mut t, 0x02, |v| v | 0x10);
        } else {
            patch(&mut t, 0x02, |v| v & !0x10);
        }
    }
    // 4. crystal → reg 0x03[7:4], reg 0x05[3:0]
    if p & 0x2000 != 0 {
        match desc.tuner_xtal_hz {
            16_000_000 => {
                patch(&mut t, 0x03, |v| v & 0x0F);
                patch(&mut t, 0x05, |v| v & 0xF0);
            }
            24_000_000 => {
                patch(&mut t, 0x03, |v| (v & 0x0F) | 0x40);
                patch(&mut t, 0x05, |v| (v & 0xF4) | 0x04);
            }
            27_000_000 => {
                patch(&mut t, 0x03, |v| (v & 0x0F) | 0x70);
                patch(&mut t, 0x05, |v| (v & 0xF7) | 0x07);
            }
            32_000_000 => {
                patch(&mut t, 0x03, |v| (v & 0x0F) | 0x90);
                patch(&mut t, 0x05, |v| (v & 0xF9) | 0x09);
            }
            _ => {}
        }
    }
    // 5. option bit0 → reg 0x03 bit3 (clock output enable)
    if desc.tuner_opts & 0x1 != 0 {
        patch(&mut t, 0x03, |v| v | 0x08);
    } else {
        patch(&mut t, 0x03, |v| v & !0x08);
    }
    // 6. option bit1 or desc flags bit0 → reg 0x04 bit0
    let b = (desc.tuner_opts & 0x2 != 0) || (desc.flags & 0x1 != 0);
    patch(&mut t, 0x04, |v| (v & !0x01) | b as u8);
    // 7. option bit2 → reg 0x08 bit7 cleared, else set
    if desc.tuner_opts & 0x4 != 0 {
        patch(&mut t, 0x08, |v| v & !0x80);
    } else {
        patch(&mut t, 0x08, |v| v | 0x80);
    }
    Ok(t)
}

/// 10.6 fixed-point MHz frequency word (identical to Linux).
pub fn rf_tune_word(freq_hz: u32) -> u16 {
    let mut dig = freq_hz / 1_000_000;
    let mut rem = freq_hz % 1_000_000;
    let mut div = 1_000_000u32;
    for _ in 0..6 {
        div >>= 1;
        dig <<= 1;
        if rem > div {
            rem -= div;
            dig += 1;
        }
    }
    if rem > 7812 {
        dig += 1;
    }
    dig as u16
}

/// Build the RF tune register list for `freq_hz` / `bw_hz`.
pub fn rf_tune_registers(freq_hz: u32, bw_hz: u32) -> [(u8, u8); 10] {
    let mut t = RFTUNE_TABLE;
    match bw_hz {
        6_000_000 => patch(&mut t, 0x0C, |v| (v & 0xD5) | 0x15),
        8_000_000 => patch(&mut t, 0x0C, |v| v | 0x3F),
        _ => {}
    }
    let dig = rf_tune_word(freq_hz);
    patch(&mut t, 0x0D, |_| (dig & 0xFF) as u8);
    patch(&mut t, 0x0E, |_| (dig >> 8) as u8);
    if freq_hz >= 333_000_000 {
        patch(&mut t, 0x80, |v| v | 0x40);
    }
    t
}

fn flatten(pairs: &[(u8, u8)]) -> Vec<u8> {
    pairs.iter().flat_map(|&(r, v)| [r, v]).collect()
}

#[derive(Debug, Clone, Copy)]
pub struct Mxl13xRf {
    pub bus: TunerBus,
    pub kind: MxlType,
}

impl Mxl13xRf {
    pub fn new(bus: TunerBus, kind: MxlType) -> Self {
        Mxl13xRf { bus, kind }
    }

    /// Write bytes to the tuner.
    pub fn write<T: UsbControl>(&self, dev: &mut Device<T>, data: &[u8]) -> Result<()> {
        self.bus.write(dev, data)
    }

    /// Write `{0xFB, reg}` then read one byte.
    pub fn read_reg<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8) -> Result<u8> {
        self.bus.write(dev, &[0xFB, reg])?;
        let mut b = [0u8; 1];
        self.bus.read(dev, &mut b)?;
        Ok(b[0])
    }

    /// Open-time initialisation: soft reset (`0xFF`) then the
    /// patched table in one write. No delay in the original.
    pub fn init<T: UsbControl>(&self, dev: &mut Device<T>, desc: &TunerDesc) -> Result<()> {
        let regs = init_registers(desc)?;
        self.write(dev, &[0xFF])?;
        self.write(dev, &flatten(&regs))
    }

    /// Initialisation used by the `SMT_LJ101` hw 0x102 kit:
    /// soft reset then the fixed 18-pair table in one 36-byte write.
    pub fn init_lj101<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.write(dev, &[0xFF])?;
        self.write(dev, &flatten(&INIT_TABLE_LJ101))
    }

    /// Standby: reg 0x01 = 0, reg 0x0F = 0.
    pub fn standby<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.write(dev, &[0x01, 0x00, 0x0F, 0x00])
    }

    /// Wake: reg 0x01 = 1.
    pub fn wake<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.write(dev, &[0x01, 0x01])
    }

    /// Restart the synthesizer (present in, but unused by, the original driver).
    pub fn retune<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.write(dev, &[0x0F, 0x00, 0x0F, 0x01])
    }

    /// Set frequency: 10 register pairs in one write, 3 ms
    /// delay, then reg 0xD8 is read twice and discarded (no lock check).
    pub fn set_frequency<T: UsbControl>(&self, dev: &mut Device<T>, freq_hz: u32, bw_hz: u32) -> Result<()> {
        let regs = rf_tune_registers(freq_hz, bw_hz);
        self.write(dev, &flatten(&regs))?;
        sleep(Duration::from_millis(3));
        let _ = self.read_reg(dev, 0xD8);
        let _ = self.read_reg(dev, 0xD8);
        Ok(())
    }

    /// RF/REF lock bits from reg 0xD8 (Linux semantics: `0x0C` = RF lock,
    /// `0x03` = REF lock). Not evaluated by the original driver.
    pub fn lock_status<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<u8> {
        self.read_reg(dev, 0xD8)
    }

    /// Signal readout: `1000 * (113 - ba' - cc)` where
    /// `ba' = ba > 7 ? (ba + 0xF0) as u8 : ba` (zero-extended, as the original driver computes it).
    pub fn signal<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<i32> {
        let cc = self.read_reg(dev, 0xCC)?;
        let ba = self.read_reg(dev, 0xBA)?;
        Ok(signal_value(cc, ba))
    }
}

/// The original driver's conversion, reproduced exactly (zero-extends `ba + 0xF0`).
pub fn signal_value(cc: u8, ba: u8) -> i32 {
    let ba2 = if ba > 7 { ba.wrapping_add(0xF0) as u32 } else { ba as u32 };
    1000 * (113i32 - ba2 as i32 - cc as i32)
}

/// Probably-intended conversion: `ba` as a 4-bit two's-complement value.
pub fn signal_value_signed(cc: u8, ba: u8) -> i32 {
    let ba2 = if ba > 7 { ba as i32 - 16 } else { ba as i32 };
    1000 * (113 - ba2 - cc as i32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tune_word_example() {
        assert_eq!(rf_tune_word(473_142_857), 0x7649);
        let regs = rf_tune_registers(473_142_857, 6_000_000);
        assert_eq!(
            flatten(&regs),
            vec![
                0x0F, 0x00, 0x0C, 0x15, 0x0D, 0x49, 0x0E, 0x76, 0x1F, 0x87, 0x20, 0x1F, 0x21, 0x87, 0x22, 0x1F, 0x80,
                0x41, 0x0F, 0x01
            ]
        );
    }

    #[test]
    fn init_patches() {
        let desc = TunerDesc {
            present: 0x1 | 0x4000 | 0x2000,
            hw_version: 0x102,
            if_freq_hz: -4_000_000,
            tuner_xtal_hz: 24_000_000,
            ..Default::default()
        };
        let regs = init_registers(&desc).unwrap();
        let get = |r: u8| regs.iter().find(|p| p.0 == r).unwrap().1;
        assert_eq!(get(0x04), 0x00);
        assert_eq!(get(0x02), 0x10);
        assert_eq!(get(0x03), 0x40);
        assert_eq!(get(0x05), 0x04);
        assert_eq!(get(0x06), 0x10);
        assert_eq!(get(0x08), 0x80);
        assert_eq!(flatten(&regs).len(), 44);
    }

    #[test]
    fn kit_name_types() {
        assert_eq!(MxlType::from_kit_name("TC90517_MxL135RF"), Some(MxlType::MxL135RF));
        assert_eq!(MxlType::from_kit_name("TC90517_MxL131RF"), Some(MxlType::MxL131RF));
        assert_eq!(MxlType::from_kit_name("LGDT3305_MxL5007T"), Some(MxlType::MxL5007));
        assert_eq!(MxlType::from_kit_name("SMT_LJ101"), None);
    }

    #[test]
    fn signal_conversion() {
        assert_eq!(signal_value(10, 3), 100_000);
        assert_eq!(signal_value_signed(10, 8), 1000 * (113 + 8 - 10));
        assert_eq!(signal_value(10, 8), 1000 * (113 - 248 - 10));
    }
}
