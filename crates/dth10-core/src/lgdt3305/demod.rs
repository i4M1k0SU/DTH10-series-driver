//! LG LGDT3305 ATSC 8VSB / ITU-T J.83 Annex B QAM demodulator, as the
//! original driver's `LGDT3305_MxL5007T` kit drives it.
//!
//! Registers have 16-bit addresses and are accessed with raw I2C transfers
//! to the demodulator's 7-bit address (`TunerDesc::demod_i2c_addr`):
//!
//! * write: `[reg_hi, reg_lo, value]`
//! * read:  write `[reg_hi, reg_lo]`, then read one byte.
//!
//! The driver keeps no register shadow: every bit update is a read followed
//! by a write. It has no lock-wait loop either; lock, C/N, packet errors and
//! quality are evaluated on demand in [`Lgdt3305::status`].

use core::time::Duration;
use std::thread::sleep;

use crate::device::Device;
use crate::error::{Error, Result};
use crate::kit::{BER_UNUSED, SignalStatus, field};
use crate::protocol::UsbControl;

/// Register addresses used by the driver.
pub mod reg {
    /// General control 1: `bits[1:0]` = modulation (0 = 64QAM, 1 = 256QAM, 3 = 8VSB).
    pub const GEN_CTRL_1: u16 = 0x0000;
    /// General control 2: bit5 = I2C repeater to the tuner (0 = open, 1 = closed).
    pub const GEN_CTRL_2: u16 = 0x0001;
    /// General control 3: bit0 = soft reset (0 = held in reset).
    pub const GEN_CTRL_3: u16 = 0x0002;
    /// General status: bit2 = what the driver reports as full lock.
    pub const GEN_STATUS: u16 = 0x0003;
    /// IF control: bit6 set for an IF of 44 MHz or more, cleared below.
    pub const IF_CTRL: u16 = 0x0007;
    /// `bits[7:6]` cleared for a low IF or an IF of 44.0..44.2 MHz, set otherwise.
    pub const IF_RANGE: u16 = 0x000D;
    /// Digital AGC reference, high / low byte.
    pub const DGTL_AGC_REF_1: u16 = 0x0012;
    pub const DGTL_AGC_REF_2: u16 = 0x0013;
    /// Carrier-recovery centre frequency (IF NCO), 4 bytes, most significant first.
    pub const CR_CTR_FREQ_1: u16 = 0x0106;
    pub const CR_CTR_FREQ_2: u16 = 0x0107;
    pub const CR_CTR_FREQ_3: u16 = 0x0108;
    pub const CR_CTR_FREQ_4: u16 = 0x0109;
    /// bit7 cleared by the 8VSB set-up.
    pub const CR_CTRL_0116: u16 = 0x0116;
    /// Carrier-recovery mean square error (QAM C/N), high / low byte.
    pub const CR_MSE_1: u16 = 0x011B;
    pub const CR_MSE_2: u16 = 0x011C;
    /// Carrier lock: bit7 in 8VSB, bit1 in QAM.
    pub const CR_LOCK_STATUS: u16 = 0x011D;
    /// bit7 = spectral inversion (8VSB).
    pub const CR_CTRL_7: u16 = 0x0126;
    /// Read (QAM only) and discarded by the status readout.
    pub const STATUS_021B: u16 = 0x021B;
    /// IF bandwidth, two bytes.
    pub const IFBW_1: u16 = 0x0308;
    pub const IFBW_2: u16 = 0x0309;
    /// AGC control 1.
    pub const AGC_CTRL_1: u16 = 0x030C;
    /// bit2 set by both modulation set-ups.
    pub const AGC_CTRL_030D: u16 = 0x030D;
    /// AGC control 4.
    pub const AGC_CTRL_4: u16 = 0x0314;
    /// bit0 = what the driver reports as "signal" (first lock flag).
    pub const AGC_STATUS: u16 = 0x031E;
    /// Phase-tracker mean square error (8VSB C/N), 20 bits over three registers.
    pub const PT_MSE_1: u16 = 0x0417;
    pub const PT_MSE_2: u16 = 0x0418;
    pub const PT_MSE_3: u16 = 0x0419;
    /// Equaliser / filter set-up byte (0xAA for 8VSB, 0xAE for QAM).
    pub const FILTER_043F: u16 = 0x043F;
    /// FEC block control: bit1 = QAM spectrum setting, bit3 written 0 then 1.
    pub const FEC_BLOCK_CTRL: u16 = 0x0504;
    /// FEC lock status (read in QAM and discarded).
    pub const FEC_LOCK_STATUS: u16 = 0x050A;
    /// Uncorrectable packet counter, high / low byte.
    pub const FEC_PKT_ERR_1: u16 = 0x050C;
    pub const FEC_PKT_ERR_2: u16 = 0x050D;
    /// Transport-stream output control.
    pub const TP_CTRL_1: u16 = 0x050E;
}

/// Lock flag bits of [`SignalStatus::lock`] for this demodulator. The values
/// are those of [`crate::kit::lock`] with the same meaning (first flag,
/// signal / carrier, full lock), so callers can test them the same way.
pub mod lock {
    /// `0x031E` bit0.
    pub const AGC: u32 = 0x001;
    /// Carrier lock: `0x011D` bit7 (8VSB) / bit1 (QAM).
    pub const CARRIER: u32 = 0x002;
    /// `0x0003` bit2: locked, transport stream valid.
    pub const LOCKED: u32 = 0x010;
}

/// Modulation, as the value of `GEN_CTRL_1` `bits[1:0]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Modulation {
    Qam64 = 0,
    Qam256 = 1,
    Vsb8 = 3,
}

impl Modulation {
    /// From the tune request's modulation code: 4 = 64QAM, 5 = 256QAM,
    /// anything else (3 = 8VSB, 0 = not set) = 8VSB.
    pub fn from_request(code: u32) -> Self {
        match code {
            4 => Modulation::Qam64,
            5 => Modulation::Qam256,
            _ => Modulation::Vsb8,
        }
    }

    /// From `GEN_CTRL_1` `bits[1:0]`; the value 2 is not a mode the driver knows.
    pub fn from_reg(v: u8) -> Option<Self> {
        match v & 3 {
            0 => Some(Modulation::Qam64),
            1 => Some(Modulation::Qam256),
            3 => Some(Modulation::Vsb8),
            _ => None,
        }
    }

    pub fn is_qam(self) -> bool {
        self != Modulation::Vsb8
    }
}

/// Pilot offset from the channel centre used for the 8VSB NCO with a high IF:
/// 3 MHz − 309.441 kHz.
pub const VSB_PILOT_OFFSET_HZ: i64 = 2_690_559;

/// Default demodulator IF when the descriptor has none.
pub const DEFAULT_IF_HZ: i32 = 44_000_000;

/// Value of `TP_CTRL_1` (transport-stream output) from the descriptor's stream
/// parameter: base `0x12`; bit5 = NOT param bit0; bit3 = NOT param bit4;
/// bit2 = param bit5; bit0 = NOT param bit12.
pub fn tp_ctrl_1(stream_param: u32) -> u8 {
    let x = stream_param;
    let mut v = 0x12u8;
    if x & 0x01 == 0 {
        v |= 0x20;
    }
    if x & 0x10 == 0 {
        v |= 0x08;
    }
    if x & 0x20 != 0 {
        v |= 0x04;
    }
    if x & 0x1000 == 0 {
        v |= 0x01;
    }
    v
}

/// The four `CR_CTR_FREQ` bytes from a 32-bit NCO word:
/// `[(w >> 24) & 0x3F | 0x40, w >> 16, w >> 8, w]`.
pub fn nco_bytes(word: i64) -> [u8; 4] {
    let w = word as i32;
    [((w >> 24) as u8 & 0x3F) | 0x40, (w >> 16) as u8, (w >> 8) as u8, w as u8]
}

/// NCO word for the demodulator IF, as the driver computes it (integer
/// arithmetic, truncating division):
///
/// * QAM: `((|IF| / 1000) << 30) / 25000`
/// * 8VSB, IF below 44 MHz: `((|IF| / 1000) << 26) / 25000`
/// * 8VSB, IF of 44 MHz or more: `((±|IF| − 2690559) << 26) / 25000000`,
///   with `−|IF|` when the spectrum is inverted (negative IF).
pub fn nco_word(mode: Modulation, if_hz: i32) -> i64 {
    let abs = (if_hz as i64).abs();
    let high = abs >= 44_000_000;
    if mode.is_qam() {
        ((abs / 1000) << 30) / 25_000
    } else if !high {
        ((abs / 1000) << 26) / 25_000
    } else {
        let f = if if_hz < 0 { -VSB_PILOT_OFFSET_HZ - abs } else { abs - VSB_PILOT_OFFSET_HZ };
        (f << 26) / 25_000_000
    }
}

/// `log2(x) * 4096` in the driver's 4.12 fixed point (squaring method).
fn log2_q12(x: i64) -> i64 {
    let mut r = 0i64;
    let mut i = x << 12;
    while i < 4096 {
        i *= 2;
        r -= 4096;
    }
    while i >= 0x2000 {
        i /= 2;
        r += 4096;
    }
    let mut bit = 4096i64;
    loop {
        bit /= 2;
        i = i * i / 4096;
        if i >= 0x2000 {
            r += bit;
            i /= 2;
        }
        if bit < 1 {
            break;
        }
    }
    r
}

/// `10000 · log10(a / b)` (= C/N in 0.001 dB for a power ratio), with the
/// driver's fixed-point arithmetic reproduced exactly (it reads up to 3 below the exact value).
pub fn log10_ratio_e4(a: u32, b: u32) -> i32 {
    let d = log2_q12(a as i64) - log2_q12(b as i64);
    let t = 10_000 * ((d << 12) / 13_607) + 2048;
    (t / 4096) as i32
}

/// 8VSB C/N in 0.001 dB from the 20-bit phase-tracker MSE; 0 when the MSE is 0
/// or `0xFFFFF`, never negative.
pub fn cnr_mdb_vsb(mse: u32) -> i32 {
    if mse == 0 || mse > 0xFFFFE {
        return 0;
    }
    log10_ratio_e4(25_600, mse).max(0)
}

/// QAM C/N in 0.001 dB from the 16-bit carrier-recovery MSE
/// (`10·log10(688128 / mse)` for 64QAM, `696320 / mse` for 256QAM); 0 when
/// the MSE is 0 or `0xFFFF`, never negative.
pub fn cnr_mdb_qam(mode: Modulation, mse: u16) -> i32 {
    if mse == 0 || mse == 0xFFFF {
        return 0;
    }
    let c = if mode == Modulation::Qam256 { 696_320 } else { 688_128 };
    log10_ratio_e4(c, mse as u32).max(0)
}

/// Packets per second of the transport stream (19.39 / 26.97 / 38.81 Mbit/s
/// divided by 188 × 8), the divisor of [`packet_errors_e9`].
pub fn packets_per_second(mode: Modulation) -> u64 {
    match mode {
        Modulation::Qam64 => 17_932,
        Modulation::Qam256 => 25_805,
        Modulation::Vsb8 => 12_935,
    }
}

/// The value the driver reports in the layer-A BER field: the uncorrectable
/// packet counter × 1e9 / packets per second; `0xFFFF` (saturated) gives
/// [`BER_UNUSED`].
pub fn packet_errors_e9(mode: Modulation, count: u16) -> u32 {
    if count == 0xFFFF {
        return BER_UNUSED;
    }
    (1_000_000_000u64 * count as u64 / packets_per_second(mode)) as u32
}

/// Quality as the driver sets it from the C/N value. The thresholds are
/// compared with the C/N in 0.001 dB as is, so practically any real C/N
/// lands in the top bracket; `q` (20 = carrier, 60 = locked) is kept where no
/// bracket matches.
///
/// * 8VSB: 11..=16 → 80, 17..=24 → 95, > 24 → 100
/// * 64QAM: 21..=25 → 80, 26..=30 → 95, > 31 → 100
/// * 256QAM: 28..=30 → 80, 32 → 95, > 33 → 100
pub fn quality(mode: Modulation, cnr: i32, q: u32) -> u32 {
    let (b80, b95, top): (core::ops::RangeInclusive<i32>, core::ops::RangeInclusive<i32>, i32) = match mode {
        Modulation::Vsb8 => (11..=16, 17..=24, 24),
        Modulation::Qam64 => (21..=25, 26..=30, 31),
        Modulation::Qam256 => (28..=30, 32..=32, 33),
    };
    if b80.contains(&cnr) {
        80
    } else if b95.contains(&cnr) {
        95
    } else if cnr > top {
        100
    } else {
        q
    }
}

/// Base value from which the demodulator's provisional strength is derived
/// (`base − C/N`); the tuner overwrites it afterwards.
pub fn strength_base(mode: Modulation) -> i32 {
    match mode {
        Modulation::Qam64 => 20_000,
        Modulation::Qam256 => 27_000,
        Modulation::Vsb8 => 10_000,
    }
}

/// Demodulator instance.
#[derive(Debug, Clone, Copy)]
pub struct Lgdt3305 {
    /// 7-bit I2C address.
    pub addr: u8,
    /// IF the demodulator expects, Hz; negative = spectrum inverted.
    pub if_hz: i32,
    /// Modulation the chip was last set up for (8VSB after construction).
    pub mode: Modulation,
}

impl Lgdt3305 {
    pub fn new(addr: u8, if_hz: i32) -> Self {
        Lgdt3305 { addr, if_hz, mode: Modulation::Vsb8 }
    }

    pub fn read<T: UsbControl>(&self, dev: &mut Device<T>, reg: u16) -> Result<u8> {
        dev.i2c_write(self.addr, &reg.to_be_bytes())?;
        let mut b = [0u8; 1];
        dev.i2c_read(self.addr, &mut b)?;
        Ok(b[0])
    }

    pub fn write<T: UsbControl>(&self, dev: &mut Device<T>, reg: u16, value: u8) -> Result<()> {
        let [hi, lo] = reg.to_be_bytes();
        dev.i2c_write(self.addr, &[hi, lo, value])
    }

    /// Read, modify, write; returns the value written.
    fn update<T: UsbControl>(&self, dev: &mut Device<T>, reg: u16, f: impl FnOnce(u8) -> u8) -> Result<u8> {
        let v = f(self.read(dev, reg)?);
        self.write(dev, reg, v)?;
        Ok(v)
    }

    /// Soft reset: `0x0002` bit0 cleared, 20 ms, set again (one read, two writes).
    pub fn soft_reset<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        let v = self.read(dev, reg::GEN_CTRL_3)? & !0x01;
        self.write(dev, reg::GEN_CTRL_3, v)?;
        sleep(Duration::from_millis(20));
        self.write(dev, reg::GEN_CTRL_3, v | 0x01)
    }

    /// Tail shared by the two modulation set-ups: `0x030D` bit2 set, then a
    /// reset pulse that re-reads `0x0002` before releasing it.
    fn setup_tail<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.update(dev, reg::AGC_CTRL_030D, |v| v | 0x04)?;
        self.update(dev, reg::GEN_CTRL_3, |v| v & !0x01)?;
        sleep(Duration::from_millis(20));
        self.update(dev, reg::GEN_CTRL_3, |v| v | 0x01)?;
        Ok(())
    }

    /// 8VSB set-up.
    pub fn setup_vsb<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.update(dev, reg::GEN_CTRL_1, |v| v | 0x03)?;
        self.write(dev, reg::DGTL_AGC_REF_1, 0x32)?;
        self.write(dev, reg::DGTL_AGC_REF_2, 0xC4)?;
        self.write(dev, reg::AGC_CTRL_4, 0xE1)?;
        self.write(dev, reg::IFBW_1, 0x80)?;
        self.write(dev, reg::IFBW_2, 0x00)?;
        self.write(dev, reg::AGC_CTRL_1, 0x30)?;
        self.write(dev, reg::FILTER_043F, 0xAA)?;
        self.update(dev, reg::CR_CTRL_0116, |v| v & !0x80)?;
        self.setup_tail(dev)
    }

    /// QAM set-up. The driver reads `0x0000` first but then writes the bare
    /// mode value (0 or 1), not the merged one.
    pub fn setup_qam<T: UsbControl>(&self, dev: &mut Device<T>, mode: Modulation) -> Result<()> {
        let (gen1, agc_ref_2) = match mode {
            Modulation::Qam64 => (0x00, 0x00),
            Modulation::Qam256 => (0x01, 0x80),
            Modulation::Vsb8 => return Err(Error::Protocol("QAM set-up called for 8VSB")),
        };
        self.read(dev, reg::GEN_CTRL_1)?;
        self.write(dev, reg::GEN_CTRL_1, gen1)?;
        self.write(dev, reg::DGTL_AGC_REF_1, 0x2A)?;
        self.write(dev, reg::DGTL_AGC_REF_2, agc_ref_2)?;
        self.write(dev, reg::AGC_CTRL_4, 0xE3)?;
        self.write(dev, reg::IFBW_1, 0x88)?;
        self.write(dev, reg::IFBW_2, 0x88)?;
        self.write(dev, reg::AGC_CTRL_1, 0x34)?;
        self.write(dev, reg::FILTER_043F, 0xAE)?;
        self.setup_tail(dev)
    }

    /// The set-up block of `mode`.
    pub fn setup_mode<T: UsbControl>(&self, dev: &mut Device<T>, mode: Modulation) -> Result<()> {
        match mode {
            Modulation::Vsb8 => self.setup_vsb(dev),
            _ => self.setup_qam(dev, mode),
        }
    }

    /// Kit configuration (Init, or Open when Init skips the chips): set-up
    /// block of the current modulation, soft reset, TS output control.
    pub fn configure<T: UsbControl>(&self, dev: &mut Device<T>, stream_param: u32) -> Result<()> {
        self.setup_mode(dev, self.mode)?;
        self.soft_reset(dev)?;
        self.write(dev, reg::TP_CTRL_1, tp_ctrl_1(stream_param))
    }

    /// Switch to `mode` when it differs from the current one (set-up block,
    /// soft reset); the new mode is remembered only on success.
    pub fn set_modulation<T: UsbControl>(&mut self, dev: &mut Device<T>, mode: Modulation) -> Result<()> {
        if mode == self.mode {
            return Ok(());
        }
        self.setup_mode(dev, mode)?;
        self.soft_reset(dev)?;
        self.mode = mode;
        Ok(())
    }

    /// QAM spectrum setting: `0x0504` bits 1 and 3 cleared, bit1 = `bit1`,
    /// written, then written again with bit3 set.
    fn fec_block_ctrl<T: UsbControl>(&self, dev: &mut Device<T>, bit1: bool) -> Result<()> {
        let v = (self.read(dev, reg::FEC_BLOCK_CTRL)? & 0xF5) | if bit1 { 0x02 } else { 0x00 };
        self.write(dev, reg::FEC_BLOCK_CTRL, v)?;
        self.write(dev, reg::FEC_BLOCK_CTRL, v | 0x08)
    }

    /// IF set-up for the current modulation: IF range bits, spectral
    /// inversion and the carrier-recovery NCO.
    pub fn set_if<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        let abs = (self.if_hz as i64).abs();
        let high = abs >= 44_000_000;
        let inverted = self.if_hz < 0;
        let nco = nco_bytes(nco_word(self.mode, self.if_hz));
        if high {
            self.update(dev, reg::IF_CTRL, |v| v | 0x40)?;
            if self.mode.is_qam() {
                // The result of this step is not checked by the driver.
                let _ = self.fec_block_ctrl(dev, inverted);
            } else {
                self.update(dev, reg::CR_CTRL_7, |v| if inverted { v | 0x80 } else { v & 0x7F })?;
            }
            let near_nominal = (5_800_001..6_200_000).contains(&abs) || (43_800_001..=44_199_999).contains(&abs);
            self.update(dev, reg::IF_RANGE, |v| if near_nominal { v & 0x3F } else { v | 0xC0 })?;
        } else {
            self.update(dev, reg::IF_CTRL, |v| v & !0x40)?;
            self.update(dev, reg::IF_RANGE, |v| v & 0x3F)?;
            if self.mode.is_qam() {
                let _ = self.fec_block_ctrl(dev, true);
            } else {
                self.update(dev, reg::CR_CTRL_7, |v| if inverted { v | 0x80 } else { v & 0x7F })?;
            }
        }
        self.write(dev, reg::CR_CTR_FREQ_1, nco[0])?;
        self.write(dev, reg::CR_CTR_FREQ_2, nco[1])?;
        self.write(dev, reg::CR_CTR_FREQ_3, nco[2])?;
        self.write(dev, reg::CR_CTR_FREQ_4, nco[3])
    }

    /// I2C repeater to the tuner: 1 ms, `0x0001` bit5 cleared (open) or set
    /// (closed), 1 ms.
    pub fn gate<T: UsbControl>(&self, dev: &mut Device<T>, open: bool) -> Result<()> {
        sleep(Duration::from_millis(1));
        self.update(dev, reg::GEN_CTRL_2, |v| if open { v & !0x20 } else { (v & !0x20) | 0x20 })?;
        sleep(Duration::from_millis(1));
        Ok(())
    }

    /// Signal status. The modulation is read back from the chip; lock,
    /// strength (provisional), quality and C/N are always produced, the
    /// packet-error value (in `ber[0]`) when `st.mask` requests
    /// [`field::BER_A`].
    pub fn status<T: UsbControl>(&self, dev: &mut Device<T>, st: &mut SignalStatus) -> Result<()> {
        let requested = st.mask;
        st.mask = 0;
        st.lock = 0;
        st.quality = 0;
        let mode = Modulation::from_reg(self.read(dev, reg::GEN_CTRL_1)?)
            .ok_or(Error::Protocol("LGDT3305 reports modulation mode 2"))?;

        let mut lk = 0u32;
        if self.read(dev, reg::AGC_STATUS)? & 0x01 != 0 {
            lk |= lock::AGC;
        }
        let cr = self.read(dev, reg::CR_LOCK_STATUS)?;
        if mode.is_qam() {
            if cr & 0x02 != 0 {
                lk |= lock::CARRIER;
            }
            self.read(dev, reg::STATUS_021B)?;
        } else if cr & 0x80 != 0 {
            lk |= lock::CARRIER;
        }
        let gen_status = self.read(dev, reg::GEN_STATUS)?;
        // Not part of the driver's status block: the raw byte, for diagnostics.
        st.sync_state = gen_status;
        if gen_status & 0x04 != 0 {
            lk |= lock::LOCKED;
        }
        if mode.is_qam() {
            self.read(dev, reg::FEC_LOCK_STATUS)?;
        }
        st.lock = lk;
        let mut mask = field::LOCK;

        let mut q = 0u32;
        if lk & lock::CARRIER != 0 {
            q = 20;
            mask |= field::QUALITY;
        }
        if lk & lock::LOCKED != 0 {
            q = 60;
            mask |= field::QUALITY;
        }

        let cnr = if mode.is_qam() {
            let hi = self.read(dev, reg::CR_MSE_1)?;
            let lo = self.read(dev, reg::CR_MSE_2)?;
            cnr_mdb_qam(mode, u16::from_be_bytes([hi, lo]))
        } else {
            let b1 = self.read(dev, reg::PT_MSE_1)?;
            let b2 = self.read(dev, reg::PT_MSE_2)?;
            let b3 = self.read(dev, reg::PT_MSE_3)?;
            cnr_mdb_vsb(((b1 as u32 & 0x0F) << 16) | ((b2 as u32) << 8) | b3 as u32)
        };
        st.cnr_mdb = cnr;
        st.strength = strength_base(mode) - cnr;
        mask |= field::STRENGTH | field::CNR;

        if requested & field::BER_A != 0 {
            st.ber[0] = if lk & lock::LOCKED == 0 {
                BER_UNUSED
            } else {
                let hi = self.read(dev, reg::FEC_PKT_ERR_1)?;
                let lo = self.read(dev, reg::FEC_PKT_ERR_2)?;
                packet_errors_e9(mode, u16::from_be_bytes([hi, lo]))
            };
            mask |= field::BER_A;
        }

        st.quality = quality(mode, cnr, q);
        st.mask = mask | field::QUALITY;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn log10_matches_float_within_3() {
        for (a, b) in
            [(25_600u32, 1u32), (25_600, 100), (25_600, 256), (25_600, 25_600), (688_128, 1000), (696_320, 300)]
        {
            let exact = 10_000.0 * (a as f64 / b as f64).log10();
            let got = log10_ratio_e4(a, b);
            assert!((got as f64 - exact).abs() <= 3.0, "{a}/{b}: got {got}, exact {exact}");
        }
        // Exact values of the driver's fixed-point arithmetic.
        assert_eq!(log10_ratio_e4(25_600, 256), 19_998);
        assert_eq!(log10_ratio_e4(25_600, 100), 24_080);
        assert_eq!(log10_ratio_e4(688_128, 1000), 28_374);
        assert_eq!(log10_ratio_e4(25_600, 25_600), 0);
    }

    #[test]
    fn cnr_edges() {
        assert_eq!(cnr_mdb_vsb(0), 0);
        assert_eq!(cnr_mdb_vsb(0xFFFFF), 0);
        assert_eq!(cnr_mdb_vsb(0xFFFFE), 0); // negative → clamped
        assert_eq!(cnr_mdb_vsb(256), 19_998);
        assert_eq!(cnr_mdb_qam(Modulation::Qam64, 0xFFFF), 0);
        assert_eq!(cnr_mdb_qam(Modulation::Qam64, 1000), 28_374);
        assert_eq!(cnr_mdb_qam(Modulation::Qam256, 300), 33_655);
    }

    #[test]
    fn nco_words() {
        // 8VSB, IF 44 MHz: (44000000 - 2690559) << 26 / 25e6.
        assert_eq!(nco_word(Modulation::Vsb8, 44_000_000), 110_889_186);
        assert_eq!(nco_bytes(110_889_186), [0x46, 0x9C, 0x08, 0xE2]);
        assert_eq!(nco_bytes(nco_word(Modulation::Vsb8, -44_000_000)), [0x78, 0x87, 0x8E, 0x02]);
        // 8VSB, IF 6 MHz: 6000 << 26 / 25000.
        assert_eq!(nco_bytes(nco_word(Modulation::Vsb8, 6_000_000)), [0x40, 0xF5, 0xC2, 0x8F]);
        // QAM, IF 44 MHz / 6 MHz.
        assert_eq!(nco_bytes(nco_word(Modulation::Qam64, 44_000_000)), [0x70, 0xA3, 0xD7, 0x0A]);
        assert_eq!(nco_bytes(nco_word(Modulation::Qam256, 6_000_000)), [0x4F, 0x5C, 0x28, 0xF5]);
    }

    #[test]
    fn tp_ctrl() {
        assert_eq!(tp_ctrl_1(0), 0x3B);
        assert_eq!(tp_ctrl_1(0x01), 0x1B);
        assert_eq!(tp_ctrl_1(0x10), 0x33);
        assert_eq!(tp_ctrl_1(0x20), 0x3F);
        assert_eq!(tp_ctrl_1(0x1000), 0x3A);
    }

    #[test]
    fn modulation_codes() {
        assert_eq!(Modulation::from_request(3), Modulation::Vsb8);
        assert_eq!(Modulation::from_request(0), Modulation::Vsb8);
        assert_eq!(Modulation::from_request(4), Modulation::Qam64);
        assert_eq!(Modulation::from_request(5), Modulation::Qam256);
        assert_eq!(Modulation::from_reg(0xFB), Some(Modulation::Vsb8));
        assert_eq!(Modulation::from_reg(0x02), None);
    }

    #[test]
    fn quality_brackets() {
        assert_eq!(quality(Modulation::Vsb8, 10, 60), 60);
        assert_eq!(quality(Modulation::Vsb8, 16, 60), 80);
        assert_eq!(quality(Modulation::Vsb8, 24, 60), 95);
        assert_eq!(quality(Modulation::Vsb8, 20_000, 0), 100);
        assert_eq!(quality(Modulation::Qam64, 31, 20), 20);
        assert_eq!(quality(Modulation::Qam64, 32, 20), 100);
        assert_eq!(quality(Modulation::Qam256, 31, 20), 20);
        assert_eq!(quality(Modulation::Qam256, 32, 20), 95);
        assert_eq!(quality(Modulation::Qam256, 33, 20), 20);
        assert_eq!(quality(Modulation::Qam256, 0, 0), 0);
    }

    #[test]
    fn packet_errors() {
        assert_eq!(packet_errors_e9(Modulation::Vsb8, 0), 0);
        assert_eq!(packet_errors_e9(Modulation::Vsb8, 1), 77_309);
        assert_eq!(packet_errors_e9(Modulation::Qam64, 17_932), 1_000_000_000);
        assert_eq!(packet_errors_e9(Modulation::Qam256, 0xFFFF), BER_UNUSED);
    }
}
