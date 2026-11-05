//! The demodulator of the SMT_LJ101 module: the TC90517 satellite side as the
//! hw 0x402 kit drives it, and the module's register blocks (terrestrial
//! side A1-A4, also written by the hw 0x102 terrestrial kit, satellite side
//! S1/S2).

use crate::device::Device;
use crate::error::{Error, Result};
use crate::kit::{BER_UNUSED, SignalStatus, field, lock};
use crate::protocol::UsbControl;
use crate::tlv::TunerDesc;

/// Terrestrial-side I2C address, fixed in the original driver's Open configuration.
pub const ADDR_TER: u8 = 0x18;
/// Satellite-side I2C address, fixed in the original driver's Open configuration.
pub const ADDR_SAT: u8 = 0x19;

/// Block A1 (addr 0x18).
pub const BLOCK_A1: [(u8, u8); 25] = [
    (0x01, 0x00),
    (0x02, 0x00),
    (0x03, 0x00),
    (0x04, 0x02),
    (0x05, 0x00),
    (0x06, 0x80),
    (0x07, 0x00),
    (0x08, 0x00),
    (0x0C, 0x00),
    (0x0F, 0x34),
    (0x11, 0x27),
    (0x12, 0x0C),
    (0x13, 0x33),
    (0x14, 0x20),
    (0x15, 0x40),
    (0x16, 0x00),
    (0x17, 0x70),
    (0x18, 0x31),
    (0x19, 0x13),
    (0x1A, 0x31),
    (0x1B, 0x13),
    (0x1C, 0x28),
    (0x1D, 0x88),
    (0x1E, 0xA2),
    (0x1F, 0x08),
];
/// Block A2 (addr 0x18).
pub const BLOCK_A2: [(u8, u8); 28] = [
    (0x20, 0x00),
    (0x21, 0xFF),
    (0x22, 0x80),
    (0x23, 0x4C),
    (0x24, 0x4C),
    (0x25, 0x00),
    (0x26, 0x00),
    (0x27, 0x0C),
    (0x28, 0x60),
    (0x29, 0x6B),
    (0x2A, 0x40),
    (0x2B, 0x40),
    (0x2C, 0x5A),
    (0x2D, 0x05),
    (0x2E, 0xFF),
    (0x2F, 0x00),
    (0x30, 0x20),
    (0x31, 0x0D),
    (0x32, 0x21),
    (0x34, 0x0F),
    (0x38, 0x00),
    (0x39, 0xAA),
    (0x3A, 0x2D),
    (0x3B, 0x21),
    (0x3C, 0x3F),
    (0x3D, 0x10),
    (0x3E, 0x08),
    (0x3F, 0x0C),
];
/// Block A3 (addr 0x18).
pub const BLOCK_A3: [(u8, u8); 23] = [
    (0x40, 0x0C),
    (0x41, 0x00),
    (0x42, 0x00),
    (0x43, 0x4F),
    (0x44, 0xFF),
    (0x46, 0x20),
    (0x47, 0x00),
    (0x48, 0x90),
    (0x49, 0xE6),
    (0x4A, 0x02),
    (0x4B, 0x80),
    (0x4C, 0x00),
    (0x4F, 0x05),
    (0x50, 0x00),
    (0x51, 0x68),
    (0x52, 0x20),
    (0x54, 0x57),
    (0x55, 0xF1),
    (0x56, 0x20),
    (0x57, 0x70),
    (0x5C, 0x50),
    (0x5D, 0x00),
    (0x5F, 0x80),
];
/// Block A4 (addr 0x18).
pub const BLOCK_A4: [(u8, u8); 15] = [
    (0x70, 0x38),
    (0x71, 0x00),
    (0x72, 0x00),
    (0x75, 0x00),
    (0x76, 0x00),
    (0x77, 0x01),
    (0x7C, 0x00),
    (0x7D, 0xD2),
    (0xBA, 0x00),
    (0xBB, 0x00),
    (0xBC, 0x00),
    (0xC7, 0x00),
    (0xE4, 0xF4),
    (0xEC, 0x00),
    (0xEF, 0x01),
];
/// Block S1 (satellite side).
const BLOCK_S1: [(u8, u8); 24] = [
    (0x03, 0x00),
    (0x04, 0x00),
    (0x06, 0x40),
    (0x07, 0x41),
    (0x08, 0x00),
    (0x09, 0x00),
    (0x0A, 0xFF),
    (0x0C, 0x59),
    (0x0D, 0xF2),
    (0x0E, 0xF0),
    (0x0F, 0x50),
    (0x10, 0xB2),
    (0x11, 0x00),
    (0x12, 0x30),
    (0x13, 0x80),
    (0x14, 0x00),
    (0x15, 0x00),
    (0x17, 0x00),
    (0x1A, 0x00),
    (0x1B, 0x00),
    (0x1C, 0x0A),
    (0x1D, 0xAA),
    (0x1E, 0x20),
    (0x1F, 0x88),
];
/// Block S2 (satellite side).
const BLOCK_S2: [(u8, u8); 17] = [
    (0x20, 0x00),
    (0x38, 0x40),
    (0x39, 0x10),
    (0x3B, 0x90),
    (0x51, 0xB0),
    (0x52, 0x89),
    (0x53, 0xB3),
    (0x5A, 0x2D),
    (0x5B, 0xD3),
    (0x85, 0x69),
    (0x87, 0x00),
    (0x8D, 0x00),
    (0x8E, 0x00),
    (0xA3, 0x11),
    (0xA4, 0x00),
    (0xA5, 0x40),
    (0xA6, 0x04),
];

pub fn write_block<T: UsbControl>(dev: &mut Device<T>, addr: u8, block: &[(u8, u8)]) -> Result<()> {
    for &(r, v) in block {
        dev.i2c_batch_append(addr, r, &[v])?;
    }
    dev.i2c_batch_flush()
}

/// C/N in 0.001 dB from the 16-bit raw value of register 0xBC (identical
/// to Linux tc90522): `P = sqrt(cndat − 3000) / 64`,
/// `CNR = −1634.6P⁵ + 14341P⁴ − 50259P³ + 88977P² − 89565P + 58857`.
pub fn cnr_mdb_from_raw(cndat: u16) -> i32 {
    if cndat < 3000 {
        return 0;
    }
    let p = ((cndat - 3000) as f64).sqrt() / 64.0;
    let v =
        -1634.6 * p.powi(5) + 14341.0 * p.powi(4) - 50259.0 * p.powi(3) + 88977.0 * p.powi(2) - 89565.0 * p + 58857.0;
    v.trunc().max(0.0) as i32
}

/// Satellite-side TC90517 object. `addr` is the *current* demodulator
/// address (`TunerDesc::demod_i2c_addr`, replaced by 0x19 during `configure` when
/// `flags` bit5 is set).
#[derive(Debug, Clone, Copy)]
pub struct Tc90517Sat {
    pub addr: u8,
}

impl Tc90517Sat {
    pub fn new(addr: u8) -> Self {
        Tc90517Sat { addr }
    }

    /// Open/Init configuration.
    pub fn configure<T: UsbControl>(&mut self, dev: &mut Device<T>, desc: &TunerDesc) -> Result<()> {
        let x_bit = (!(desc.stream_param >> 3) & 0x02) as u8;
        if desc.flags & 0x20 != 0 {
            self.addr = ADDR_TER;
            write_block(dev, ADDR_TER, &BLOCK_A1)?;
            write_block(dev, ADDR_TER, &BLOCK_A2)?;
            write_block(dev, ADDR_TER, &BLOCK_A3)?;
            write_block(dev, ADDR_TER, &BLOCK_A4)?;
            let _ = dev.i2c_reg_write(ADDR_TER, 0x75, &[x_bit]);
            self.addr = ADDR_SAT;
        }
        dev.i2c_reg_write(self.addr, 0x01, &[0x90])?;
        write_block(dev, self.addr, &BLOCK_S1)?;
        write_block(dev, self.addr, &BLOCK_S2)?;
        dev.i2c_reg_write(self.addr, 0x04, &[x_bit])
    }

    fn read1<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8) -> Result<u8> {
        let mut b = [0u8; 1];
        dev.i2c_reg_read(self.addr, reg, &mut b)?;
        Ok(b[0])
    }

    /// TS pin control: 0 = output, 1 = stopped, 2 = unused.
    pub fn set_ts_pins<T: UsbControl>(&self, dev: &mut Device<T>, mode: u8) -> Result<()> {
        let v = self.read1(dev, 0x1C)? & 0x3F;
        let (r1c, r1f) = match mode {
            0 => (v, 0x88),
            1 => (v | 0x80, 0xAA),
            2 => (v | 0x40, 0x99),
            _ => return Ok(()),
        };
        dev.i2c_batch_append(self.addr, 0x1C, &[r1c])?;
        dev.i2c_batch_append(self.addr, 0x1F, &[r1f])?;
        dev.i2c_batch_flush()
    }

    /// `0x17 ← 0` (at Open).
    pub fn wake<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        dev.i2c_reg_write(self.addr, 0x17, &[0x00])
    }

    /// `0x17 ← 1` (end of the kit's sleep sequence).
    pub fn sleep<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        dev.i2c_reg_write(self.addr, 0x17, &[0x01])
    }

    /// Post-PLL demodulator restart (steps 4–5 of the original tune sequence): `0x03 ← 1`, `0xA5 |= 1`.
    pub fn restart<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        dev.i2c_reg_write(self.addr, 0x03, &[0x01])?;
        let v = self.read1(dev, 0xA5)?;
        dev.i2c_reg_write(self.addr, 0xA5, &[v | 0x01])
    }

    /// TSID selection: `0x8F ← hi`, `0x90 ← lo`.
    pub fn set_tsid<T: UsbControl>(&self, dev: &mut Device<T>, tsid: u16) -> Result<()> {
        dev.i2c_batch_append(self.addr, 0x8F, &[(tsid >> 8) as u8])?;
        dev.i2c_batch_append(self.addr, 0x90, &[(tsid & 0xFF) as u8])?;
        dev.i2c_batch_flush()
    }

    fn ber<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8) -> Result<u32> {
        let mut b = [0u8; 5];
        dev.i2c_reg_read(self.addr, reg, &mut b)?;
        let err = ((b[0] as u32) << 16) | ((b[1] as u32) << 8) | b[2] as u32;
        let period = ((b[3] as u32) << 8) | b[4] as u32;
        Ok(crate::tc90517::ber_e9(err, period))
    }

    /// Signal status. Request `field::TSID_TABLE` (0x10000) in
    /// `st.mask` to also read the TMCC TS-ID table (only when locked).
    pub fn status<T: UsbControl>(&self, dev: &mut Device<T>, st: &mut SignalStatus) -> Result<()> {
        let requested = st.mask;
        st.mask = 0;
        st.lock = 0;
        st.quality = 0;
        st.ber = [BER_UNUSED; 3];
        let c3 = self.read1(dev, 0xC3)?;
        if c3 & 0x10 == 0 {
            st.lock = lock::SYNC_STARTED | lock::SIGNAL | lock::LOCKED;
        }
        let mut mask = field::LOCK;
        if st.lock & lock::SIGNAL != 0 {
            st.quality = 20;
            mask |= field::QUALITY;
        }
        if st.lock & lock::LOCKED != 0 {
            st.quality = 60;
            mask |= field::QUALITY;
            dev.i2c_reg_read(self.addr, 0xC8, &mut st.tmcc_raw)?;
            if requested & field::TSID_TABLE != 0 {
                let mut ids = [0u8; 16];
                dev.i2c_reg_read(self.addr, 0xCE, &mut ids)?;
                let mut n = 0usize;
                st.tsids = [0; 8];
                for i in 0..8 {
                    let id = ((ids[2 * i] as u16) << 8) | ids[2 * i + 1] as u16;
                    if id != 0xFFFF {
                        st.tsids[n] = id;
                        n += 1;
                    }
                }
                st.tsid_count = n as u8;
                mask |= field::TSID_TABLE;
            }
        }
        let mut bc = [0u8; 2];
        dev.i2c_reg_read(self.addr, 0xBC, &mut bc)?;
        let cndat = ((bc[0] as u16) << 8) | bc[1] as u16;
        let cn = cnr_mdb_from_raw(cndat);
        mask |= field::CNR | field::STRENGTH;
        st.cnr_mdb = cn;
        st.strength = 8000 - cn;
        if requested & field::BER_B != 0 {
            st.ber[1] = self.ber(dev, 0xEB)?;
            mask |= field::BER_B;
        }
        if requested & field::BER_A != 0 {
            st.ber[0] = self.ber(dev, 0xF0)?;
            mask |= field::BER_A;
        }
        if cn > 5000 {
            st.quality = 80;
        }
        if cn > 6000 {
            st.quality = 90;
        }
        if cn > 7000 {
            st.quality = 95;
        }
        if cn > 8000 {
            st.quality = 100;
        }
        mask |= field::QUALITY;
        st.mask = mask;
        st.sync_state = c3;
        Ok(())
    }

    /// Demodulator-pin LNB control, mode 1: `0x1C[1:0] = 3` (on) / `2` (off)
    /// on whichever chip register `0x18:0x0F[5:4]` selects.
    pub fn lnb_enable<T: UsbControl>(&self, dev: &mut Device<T>, on: bool) -> Result<()> {
        let mut v = [0u8; 1];
        dev.i2c_reg_read(ADDR_TER, 0x0F, &mut v)?;
        let chip = match ((v[0] >> 4) & 3).wrapping_sub(1) {
            0 => ADDR_TER,
            2 => ADDR_SAT,
            _ => return Err(Error::Protocol("LNB control chip selection invalid")),
        };
        let mut c = [0u8; 1];
        dev.i2c_reg_read(chip, 0x1C, &mut c)?;
        dev.i2c_reg_write(chip, 0x1C, &[(c[0] & 0xFC) | if on { 0x03 } else { 0x02 }])
    }

    /// LNB state read-back, mode 1: `(0x1C & 3) == 3`.
    pub fn lnb_enabled<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<bool> {
        Ok(self.read1(dev, 0x1C)? & 0x03 == 0x03)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cnr_matches_reference_table() {
        for (raw, expect) in
            [(3000u16, 58857i32), (19384, 10712), (25000, 8004), (35000, 2607), (38215, 0), (2999, 0), (40000, 0)]
        {
            let got = cnr_mdb_from_raw(raw);
            assert!((got - expect).abs() <= 2, "raw {raw}: got {got}, expected {expect}");
        }
    }

    #[test]
    fn block_sizes_fit_batch() {
        for b in [BLOCK_A1.len(), BLOCK_A2.len(), BLOCK_A3.len(), BLOCK_A4.len(), BLOCK_S1.len(), BLOCK_S2.len()] {
            assert!(b * 6 <= crate::device::I2C_BATCH_MAX);
        }
    }
}
