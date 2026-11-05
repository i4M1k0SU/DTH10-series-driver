//! Toshiba TC90517 ISDB-T demodulator object, as the original driver's
//! terrestrial kits drive it. See `docs/10-tc90517.md`.
//!
//! This object only implements the ISDB-T side: it never looks at the tune
//! request (frequency/bandwidth go to the tuner), has no lock-wait loop, and
//! evaluates lock/C/N/BER on demand in [`Tc90517::status`].

use core::time::Duration;
use std::thread::sleep;

use crate::device::Device;
use crate::error::{Error, Result};
use crate::kit::{BER_UNUSED, SignalStatus, field, lock};
use crate::protocol::UsbControl;
use crate::tlv::TunerDesc;

/// Which terrestrial kit drives the demodulator; selects the register
/// sequences that differ between `TC90517_MxL13xRF` and `SMT_LJ101` hw 0x102
/// (the kit DT-H70/U2 reports for tuner #0).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerVariant {
    /// `TC90517_MxL13xRF`: its own Open configuration, TS-mode values and post-tune steps.
    Tc90517Mxl,
    /// `SMT_LJ101` hw 0x102: its own Open configuration, TS-mode values and post-tune steps.
    SmtLj101,
}

/// Demodulator instance bound to an I2C address (`TunerDesc::demod_i2c_addr`).
#[derive(Debug, Clone, Copy)]
pub struct Tc90517 {
    pub addr: u8,
    pub variant: TerVariant,
}

/// C/N in 0.001 dB from the 24-bit raw value of register 0x8B.
///
/// `X = log10(5505024 / cndat)`, `cnr = 240X⁴ − 1600X³ + 3900X² + 5491X + 3097.5`
/// (the original uses 8.24 fixed point; this float version matches within 1 mdB).
pub fn cnr_mdb_from_raw(cndat: u32) -> i32 {
    if cndat == 0 {
        return 0;
    }
    let x = (5_505_024.0f64 / cndat as f64).log10();
    let v = 240.0 * x.powi(4) - 1600.0 * x.powi(3) + 3900.0 * x.powi(2) + 5491.0 * x + 3097.5;
    v.trunc() as i32
}

/// BER ×1e-9 from an error count and a period in 204-byte packets.
pub fn ber_e9(err: u32, period: u32) -> u32 {
    if period == 0 {
        return 1_000_000_000;
    }
    ((1_000_000_000u64 * err as u64) / (1632u64 * period as u64)) as u32
}

impl Tc90517 {
    pub fn new(addr: u8) -> Self {
        Tc90517 { addr, variant: TerVariant::Tc90517Mxl }
    }

    pub fn with_variant(addr: u8, variant: TerVariant) -> Self {
        Tc90517 { addr, variant }
    }

    /// Open-time configuration, which differs between the MxL kit and the
    /// SMT_LJ101 kit.
    pub fn configure<T: UsbControl>(&self, dev: &mut Device<T>, desc: &TunerDesc) -> Result<()> {
        match self.variant {
            TerVariant::Tc90517Mxl => self.configure_mxl_kit(dev, desc),
            TerVariant::SmtLj101 => self.configure_lj101_kit(dev, desc),
        }
    }

    /// SMT_LJ101 hw 0x102, `TunerDesc.flags` bit5 clear: the terrestrial-side
    /// register blocks A1..A4 shared with the satellite kit's Open
    /// configuration, then `0x75 <- (~(X >> 3)) & 2` (X = `stream_param`).
    /// With bit5 set the original first rewrites the satellite side (0x19)
    /// as well; that path is not reproduced here.
    fn configure_lj101_kit<T: UsbControl>(&self, dev: &mut Device<T>, desc: &TunerDesc) -> Result<()> {
        if desc.flags & 0x20 != 0 {
            return Err(Error::Unsupported("SMT_LJ101 config with flags bit5 set".into()));
        }
        let a = self.addr;
        crate::lj101::demod::write_block(dev, a, &crate::lj101::demod::BLOCK_A1)?;
        crate::lj101::demod::write_block(dev, a, &crate::lj101::demod::BLOCK_A2)?;
        crate::lj101::demod::write_block(dev, a, &crate::lj101::demod::BLOCK_A3)?;
        crate::lj101::demod::write_block(dev, a, &crate::lj101::demod::BLOCK_A4)?;
        let x_bit = (!(desc.stream_param >> 3) & 0x02) as u8;
        dev.i2c_reg_write(a, 0x75, &[x_bit])
    }

    /// TC90517_MxL13xRF. Requires a 16 MHz crystal and an IF of
    /// 4, 44 or 57 MHz (sign/inversion handled by the tuner).
    ///
    /// The crystal/IF checks are done before queueing anything (the original
    /// leaves two batch entries queued when the crystal check fails).
    fn configure_mxl_kit<T: UsbControl>(&self, dev: &mut Device<T>, desc: &TunerDesc) -> Result<()> {
        if desc.demod_xtal_hz != 16_000_000 {
            return Err(Error::Unsupported(format!("TC90517 crystal {} Hz", desc.demod_xtal_hz)));
        }
        let if_hz = desc.if_freq_hz.unsigned_abs();
        let (reg16, reg30, reg31, reg38): (u8, u8, [u8; 4], [u8; 2]) = match if_hz {
            4_000_000 => (0x00, 0x20, [0x0D, 0xB0, 0x00, 0x0F], [0x00, 0x94]),
            44_000_000 => (0x23, 0x20, [0x10, 0xE1, 0x00, 0x3F], [0x01, 0x6F]),
            57_000_000 => (0x23, 0x28, [0x10, 0x00, 0x00, 0x3F], [0x01, 0x6F]),
            _ => return Err(Error::Unsupported(format!("TC90517 IF {} Hz", desc.if_freq_hz))),
        };
        let a = self.addr;
        dev.i2c_batch_append(a, 0x04, &[0x01])?;
        dev.i2c_batch_append(a, 0x09, &[0x10])?;
        // Four bytes from register 0x1C on, in this order: 28 A0 80 08.
        dev.i2c_batch_append(a, 0x1C, &[0x28, 0xA0, 0x80, 0x08])?;
        dev.i2c_batch_append(a, 0x16, &[reg16])?;
        dev.i2c_batch_append(a, 0x30, &[reg30])?;
        dev.i2c_batch_append(a, 0x31, &reg31)?;
        dev.i2c_batch_append(a, 0x38, &reg38)?;
        dev.i2c_batch_append(a, 0x3D, &[0x24])?;
        dev.i2c_batch_append(a, 0x4A, &[0x04])?;
        dev.i2c_batch_append(a, 0x54, &[0xD8, 0xE0])?;
        dev.i2c_batch_append(a, 0x59, &[0x52])?;
        dev.i2c_batch_append(a, 0x65, &[0x50, 0x5C, 0x00, 0x01])?;
        let b0 = if desc.stream_param & 0x10 == 0 { 0x06 } else { 0x04 };
        dev.i2c_batch_append(a, 0x75, &[b0, 0x32, 0x01])?;
        dev.i2c_batch_append(a, 0x7D, &[0xD2])?;
        dev.i2c_batch_append(a, 0xEF, &[0x01])?;
        dev.i2c_batch_append(a, 0xE5, &[0x04])?;
        dev.i2c_batch_append(a, 0x01, &[0x40])?;
        dev.i2c_batch_flush()
    }

    /// TS output / pin control: registers 0x1E/0x1F.
    /// MxL kit: mode 0 = `80/08`, 1 = `A8/A8`, 2 = `94/58`.
    /// SMT_LJ101 kit: mode 0 = `A2/08`, 1 = `AA/A8`, 2 = `A6/58`.
    pub fn set_ts_mode<T: UsbControl>(&self, dev: &mut Device<T>, mode: u8) -> Result<()> {
        let (r1e, r1f) = match (self.variant, mode) {
            (TerVariant::Tc90517Mxl, 0) => (0x80, 0x08),
            (TerVariant::Tc90517Mxl, 1) => (0xA8, 0xA8),
            (TerVariant::Tc90517Mxl, 2) => (0x94, 0x58),
            (TerVariant::SmtLj101, 0) => (0xA2, 0x08),
            (TerVariant::SmtLj101, 1) => (0xAA, 0xA8),
            (TerVariant::SmtLj101, 2) => (0xA6, 0x58),
            _ => return dev.i2c_batch_flush(),
        };
        dev.i2c_batch_append(self.addr, 0x1E, &[r1e])?;
        dev.i2c_batch_append(self.addr, 0x1F, &[r1f])?;
        dev.i2c_batch_flush()
    }

    fn read1<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8) -> Result<u8> {
        let mut b = [0u8; 1];
        dev.i2c_reg_read(self.addr, reg, &mut b)?;
        Ok(b[0])
    }

    /// Wake-up: `0x03 ← 00`, `0x1C ← reg & 0x3F` (immediate writes).
    pub fn wake<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        dev.i2c_reg_write(self.addr, 0x03, &[0x00])?;
        let v = self.read1(dev, 0x1C)?;
        dev.i2c_reg_write(self.addr, 0x1C, &[v & 0x3F])
    }

    /// Standby: TS stop, `0x1C |= 0x80`, `0x03 ← 0x90`.
    pub fn standby<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.set_ts_mode(dev, 1)?;
        let v = self.read1(dev, 0x1C)?;
        dev.i2c_batch_append(self.addr, 0x1C, &[(v & 0x3F) | 0x80])?;
        dev.i2c_batch_append(self.addr, 0x03, &[0x90])?;
        dev.i2c_batch_flush()
    }

    /// Pre-tune: TS stop, `0x25 ← 00`, `0x23 |= 1` (IF-AGC hold).
    pub fn pre_tune<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.set_ts_mode(dev, 1)?;
        let v = self.read1(dev, 0x23)?;
        dev.i2c_batch_append(self.addr, 0x25, &[0x00])?;
        dev.i2c_batch_append(self.addr, 0x23, &[v | 0x01])?;
        dev.i2c_batch_flush()
    }

    /// Post-tune. MxL kit: `0x01 <- 40`, `0x23 &= ~1`, `0x76 <- 8F`
    /// (batch), 20 ms, TS on. SMT_LJ101 kit: `0x01 <- 40`,
    /// `0x23 &= ~1` (immediate), 20 ms, TS on.
    pub fn post_tune<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        dev.i2c_reg_write(self.addr, 0x01, &[0x40])?;
        let v = self.read1(dev, 0x23)?;
        match self.variant {
            TerVariant::Tc90517Mxl => {
                dev.i2c_batch_append(self.addr, 0x23, &[v & !0x01])?;
                dev.i2c_batch_append(self.addr, 0x76, &[0x8F])?;
                dev.i2c_batch_flush()?;
            }
            TerVariant::SmtLj101 => {
                dev.i2c_reg_write(self.addr, 0x23, &[v & !0x01])?;
            }
        }
        sleep(Duration::from_millis(20));
        self.set_ts_mode(dev, 0)
    }

    /// Signal status. `st.mask` selects the BER layers to read;
    /// lock, strength, quality and C/N are always produced.
    pub fn status<T: UsbControl>(&self, dev: &mut Device<T>, st: &mut SignalStatus) -> Result<()> {
        let requested = st.mask;
        let b0 = self.read1(dev, 0xB0)?;
        let r80 = self.read1(dev, 0x80)?;
        let r81 = self.read1(dev, 0x81)?;
        let s = b0 & 0x0F;
        st.sync_state = s;
        let mut lk = 0u32;
        if s != 0 {
            lk |= lock::SYNC_STARTED;
        }
        if s > 2 {
            lk |= lock::SIGNAL;
        }
        if s >= 8 && r80 & 0x28 == 0 {
            lk |= lock::LOCKED;
        }
        if r81 & 0x80 != 0 {
            lk |= lock::REG81_BIT7;
        }
        st.lock = lk;
        let mut mask = field::LOCK;

        let mut q = 0u32;
        if lk & lock::SIGNAL != 0 {
            q = 20;
        }
        if lk & lock::LOCKED != 0 {
            q = 60;
        }
        let mut raw = [0u8; 3];
        dev.i2c_reg_read(self.addr, 0x8B, &mut raw)?;
        let cndat = ((raw[0] as u32) << 16) | ((raw[1] as u32) << 8) | raw[2] as u32;
        let cnr = cnr_mdb_from_raw(cndat);
        st.cnr_mdb = cnr;
        st.strength = 18000 - cnr;
        mask |= field::CNR | field::STRENGTH;
        if cnr > 22000 {
            q = 80;
        }
        if cnr > 23000 {
            q = 90;
        }
        if cnr > 24000 {
            q = if s >= 9 { 100 } else { 95 };
        }
        st.quality = q;
        mask |= field::QUALITY;

        // Layer A/B/C BER: "unused" when the 3-bit TMCC modulation field is 0b111.
        let layers: [(u32, u8, u8, u8, u8, usize); 3] = [
            (field::BER_A, 0xB3, 0xE0, 0x80, 0x9D, 0),
            (field::BER_B, 0xB4, 0x07, 0x04, 0xA0, 1),
            (field::BER_C, 0xB6, 0x38, 0x20, 0xA3, 2),
        ];
        let periods = [0xA6u8, 0xA8, 0xAA];
        for (bit, tmcc_reg, tmcc_mask, tmcc_unused, err_reg, idx) in layers {
            if requested & bit == 0 {
                continue;
            }
            let t = self.read1(dev, tmcc_reg)?;
            if (t & tmcc_mask) >= tmcc_unused {
                st.ber[idx] = BER_UNUSED;
            } else {
                let mut e = [0u8; 3];
                dev.i2c_reg_read(self.addr, err_reg, &mut e)?;
                let mut p = [0u8; 2];
                dev.i2c_reg_read(self.addr, periods[idx], &mut p)?;
                let err = ((e[0] as u32) << 16) | ((e[1] as u32) << 8) | e[2] as u32;
                let period = ((p[0] as u32) << 8) | p[1] as u32;
                st.ber[idx] = ber_e9(err, period);
            }
            mask |= bit;
        }
        st.mask = mask;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cnr_matches_reference_table() {
        for (raw, expect) in [(1000u32, 41453i32), (10000, 28044), (40000, 22049), (100000, 18237), (5505024, 3097)] {
            let got = cnr_mdb_from_raw(raw);
            assert!((got - expect).abs() <= 1, "raw {raw}: got {got}, expected {expect}");
        }
        assert_eq!(cnr_mdb_from_raw(0), 0);
    }

    #[test]
    fn ber_formula() {
        assert_eq!(ber_e9(0, 100), 0);
        assert_eq!(ber_e9(1632, 1), 1_000_000_000);
        assert_eq!(ber_e9(5, 0), 1_000_000_000);
    }
}
