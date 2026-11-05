//! PN2031 ISDB-T demodulator, as the vendor driver's `PN2031_MxL135RF` kit
//! drives it.
//!
//! The chip is not identified beyond the kit name. Its register file is
//! reached three ways, all on the demodulator's own I2C address:
//!
//! * direct registers (`0x08`, `0x0A`, `0x45`..`0x47`, `0x5F`..`0x6C`, ...);
//! * an 8-bit indirect window: address in `0x50`, data in `0x51`
//!   (and a second one for reads, address `0x6D`, data `0x6E`);
//! * 24-bit indirect words: index in `0x28`, data bytes in `0x29`..`0x2B`;
//!   another 8-bit window at `0x04`/`0x05`.
//!
//! The demodulator never looks at the tune request: the frequency and the
//! bandwidth go to the tuner. Lock, C/N and the BER of the three ISDB-T
//! layers are evaluated on demand in [`Pn2031::status`]; there is no
//! lock-wait loop.

use core::time::Duration;
use std::thread::sleep;

use crate::device::Device;
use crate::error::Result;
use crate::kit::{BER_UNUSED, SignalStatus, field, lock};
use crate::protocol::UsbControl;

/// I2C address the vendor driver uses when the descriptor has none.
pub const DEFAULT_ADDR: u8 = 0x10;

/// Configuration, part 1 (one batch): direct registers, the first
/// indirect-window values and two 24-bit words.
pub const CFG_BASE: [(u8, u8); 20] = [
    (0x08, 0x01),
    (0x73, 0x00),
    (0x08, 0x00),
    (0x06, 0xFF),
    (0x1F, 0x0F),
    (0x1C, 0x01),
    (0x09, 0x1A),
    (0x50, 0xD1),
    (0x51, 0x22),
    (0x71, 0x00),
    (0x28, 0x2A),
    (0x29, 0x00),
    (0x2A, 0xFF),
    (0x2B, 0x80),
    (0x3B, 0x21),
    (0x3C, 0x3A),
    (0x28, 0x20),
    (0x29, 0x3E),
    (0x2A, 0xE0),
    (0x2B, 0x8F),
];

/// Configuration, part 2 (one batch): register `0x01`, then the `0x04`/`0x05`
/// window.
pub const CFG_WINDOW04_A: [(u8, u8); 25] = [
    (0x01, 0x0B),
    (0x04, 0x08),
    (0x05, 0x03),
    (0x04, 0x0E),
    (0x05, 0x00),
    (0x04, 0x0F),
    (0x05, 0x32),
    (0x04, 0x0B),
    (0x05, 0x78),
    (0x04, 0x00),
    (0x05, 0x00),
    (0x04, 0x01),
    (0x05, 0x35),
    (0x04, 0x02),
    (0x05, 0x0E),
    (0x04, 0x03),
    (0x05, 0xD8),
    (0x04, 0x09),
    (0x05, 0x00),
    (0x04, 0x0A),
    (0x05, 0xC8),
    (0x04, 0x27),
    (0x05, 0x7D),
    (0x04, 0x28),
    (0x05, 0x00),
];

/// Configuration, part 3 (one batch): the rest of the `0x04`/`0x05` window.
pub const CFG_WINDOW04_B: [(u8, u8); 20] = [
    (0x04, 0x1E),
    (0x05, 0x00),
    (0x04, 0x29),
    (0x05, 0x64),
    (0x04, 0x14),
    (0x05, 0x00),
    (0x04, 0x04),
    (0x05, 0x00),
    (0x04, 0x05),
    (0x05, 0x22),
    (0x04, 0x06),
    (0x05, 0x0E),
    (0x04, 0x07),
    (0x05, 0xD8),
    (0x04, 0x12),
    (0x05, 0x00),
    (0x04, 0x13),
    (0x05, 0xFF),
    (0x04, 0x40),
    (0x05, 0x00),
];

/// Configuration, part 4 (one batch): `0x52`, indirect `0xA7`..`0xAF`.
pub const CFG_IND_A7: [(u8, u8); 19] = [
    (0x52, 0x01),
    (0x50, 0xA7),
    (0x51, 0x00),
    (0x50, 0xA8),
    (0x51, 0xFF),
    (0x50, 0xA9),
    (0x51, 0xFF),
    (0x50, 0xAA),
    (0x51, 0x00),
    (0x50, 0xAB),
    (0x51, 0xFF),
    (0x50, 0xAC),
    (0x51, 0xFF),
    (0x50, 0xAD),
    (0x51, 0x00),
    (0x50, 0xAE),
    (0x51, 0xFF),
    (0x50, 0xAF),
    (0x51, 0xFF),
];

/// Configuration, part 5 (one batch): `0x5E`, indirect `0xDC`..`0xE1`.
pub const CFG_IND_DC: [(u8, u8); 13] = [
    (0x5E, 0x07),
    (0x50, 0xDC),
    (0x51, 0x03),
    (0x50, 0xDD),
    (0x51, 0xE8),
    (0x50, 0xDE),
    (0x51, 0x03),
    (0x50, 0xDF),
    (0x51, 0xE8),
    (0x50, 0xE0),
    (0x51, 0xFF),
    (0x50, 0xE1),
    (0x51, 0xFF),
];

/// Configuration, part 6 (one batch): indirect `0xB0`, `0xB2`..`0xB7`.
pub const CFG_IND_B0: [(u8, u8); 14] = [
    (0x50, 0xB0),
    (0x51, 0x07),
    (0x50, 0xB2),
    (0x51, 0x03),
    (0x50, 0xB3),
    (0x51, 0xE8),
    (0x50, 0xB4),
    (0x51, 0x03),
    (0x50, 0xB5),
    (0x51, 0xE8),
    (0x50, 0xB6),
    (0x51, 0xFF),
    (0x50, 0xB7),
    (0x51, 0xFF),
];

/// Configuration, part 7 (one batch) up to indirect register `0xD5`, whose
/// value comes from the descriptor's stream parameter ([`ts_select`]).
pub const CFG_TS_HEAD: [(u8, u8); 7] =
    [(0x50, 0x51), (0x51, 0x04), (0x50, 0x50), (0x51, 0x02), (0x45, 0x04), (0x48, 0x04), (0x50, 0xD5)];

/// Configuration, part 7 continued after the stream-parameter byte.
pub const CFG_TS_TAIL: [(u8, u8); 6] =
    [(0x50, 0xD6), (0x51, 0x17), (0x50, 0xD2), (0x51, 0x03), (0x50, 0xD7), (0x51, 0xB7)];

/// Configuration, part 8 (one batch): `0x3B`/`0x3C`, indirect `0x1E`..`0x39`.
pub const CFG_IND_1E: [(u8, u8); 18] = [
    (0x3B, 0x21),
    (0x3C, 0x38),
    (0x50, 0x1E),
    (0x51, 0x5D),
    (0x50, 0x22),
    (0x51, 0x00),
    (0x50, 0x23),
    (0x51, 0xC8),
    (0x50, 0x24),
    (0x51, 0x00),
    (0x50, 0x25),
    (0x51, 0xF0),
    (0x50, 0x26),
    (0x51, 0x00),
    (0x50, 0x27),
    (0x51, 0xC3),
    (0x50, 0x39),
    (0x51, 0x02),
];

/// Configuration, part 9 (one batch): 24-bit words `0x74`, `0x46`, window
/// `0x04` index `0x40`, words `0x00`, `0x05`, `0x6A`, then `0x1C`.
pub const CFG_WORDS_A: [(u8, u8); 23] = [
    (0x28, 0x74),
    (0x29, 0x00),
    (0x2A, 0x00),
    (0x2B, 0xFF),
    (0x28, 0x46),
    (0x29, 0x00),
    (0x2A, 0x20),
    (0x2B, 0x0C),
    (0x04, 0x40),
    (0x05, 0x00),
    (0x28, 0x00),
    (0x29, 0x00),
    (0x2A, 0x00),
    (0x2B, 0x08),
    (0x28, 0x05),
    (0x29, 0x00),
    (0x2A, 0x00),
    (0x2B, 0x02),
    (0x28, 0x6A),
    (0x29, 0x00),
    (0x2A, 0x00),
    (0x2B, 0x3C),
    (0x1C, 0x01),
];

/// The 24-bit words `0x06`..`0x1E` the configuration writes through
/// `0x28`..`0x2B`, as `(index, value)`; sent in three batches of 8, 8 and 9
/// words ([`COEF_BATCHES`]).
pub const COEF_WORDS: [(u8, u32); 25] = [
    (0x06, 0x00001F),
    (0x07, 0x000018),
    (0x08, 0x000012),
    (0x09, 0x000030),
    (0x0A, 0x000037),
    (0x0B, 0x000002),
    (0x0C, 0x000009),
    (0x0D, 0x000006),
    (0x0E, 0x00007B),
    (0x0F, 0x000076),
    (0x10, 0x00007D),
    (0x11, 0x000008),
    (0x12, 0x00000B),
    (0x13, 0x000000),
    (0x14, 0x0001F2),
    (0x15, 0x0001F3),
    (0x16, 0x000005),
    (0x17, 0x000016),
    (0x18, 0x00000F),
    (0x19, 0x0007EF),
    (0x1A, 0x0007D8),
    (0x1B, 0x0007F1),
    (0x1C, 0x00003D),
    (0x1D, 0x000094),
    (0x1E, 0x0000BA),
];

/// Number of [`COEF_WORDS`] in each of the three batches.
pub const COEF_BATCHES: [usize; 3] = [8, 8, 9];

/// Raw write the tune sequence sends to the demodulator before programming
/// the tuner (as bytes on the bus: `71 00 39 01 44 00`).
pub const TUNE_RAW: [u8; 6] = [0x71, 0x00, 0x39, 0x01, 0x44, 0x00];

/// Byte the configuration writes to indirect register `0xD5`: 1 when bit0 of
/// the descriptor's stream parameter is clear, else 0 (TS output selection,
/// meaning unconfirmed).
pub fn ts_select(stream_param: u32) -> u8 {
    (!stream_param & 1) as u8
}

/// C/N table: `(raw threshold, C/N in 0.1 dB, step to the next higher threshold)`,
/// highest raw value (lowest C/N) first.
const CN_TABLE: [(i32, i32, i32); 30] = [
    (18187, 10, 1461),
    (16534, 20, 1653),
    (14823, 30, 1711),
    (13161, 40, 1662),
    (11622, 50, 1539),
    (10279, 60, 1343),
    (9089, 70, 1190),
    (8042, 80, 1047),
    (7137, 90, 905),
    (6342, 100, 795),
    (5641, 110, 701),
    (5030, 120, 611),
    (4474, 130, 556),
    (3988, 140, 486),
    (3556, 150, 432),
    (3180, 160, 376),
    (2841, 170, 339),
    (2541, 180, 300),
    (2276, 190, 265),
    (2038, 200, 238),
    (1800, 210, 238),
    (1625, 220, 175),
    (1462, 230, 163),
    (1324, 240, 138),
    (1175, 250, 149),
    (1063, 260, 112),
    (980, 270, 83),
    (907, 280, 73),
    (840, 290, 67),
    (788, 300, 52),
];

/// Raw value at and above which the C/N is reported as 0.
const CN_RAW_LIMIT: i32 = 19648;

/// C/N in 0.1 dB from the 16-bit raw value of registers `0x46`/`0x47`, as the
/// vendor driver computes it: a 1 dB-step table with linear interpolation in
/// between (integer arithmetic, division truncating toward zero). 0 and values
/// of 19648 or more give 0; values below 788 give 30.0 dB.
pub fn cnr_tenth_db_from_raw(raw: u16) -> i32 {
    let x = raw as i32;
    if x == 0 || x >= CN_RAW_LIMIT {
        return 0;
    }
    for &(t, cn, step) in CN_TABLE.iter() {
        if x >= t {
            return cn - (10 * (x - t) - step / 2) / step;
        }
    }
    300
}

/// C/N in 0.001 dB (what [`SignalStatus::cnr_mdb`] carries).
pub fn cnr_mdb_from_raw(raw: u16) -> i32 {
    100 * cnr_tenth_db_from_raw(raw)
}

/// BER ×1e-9 from a 24-bit error count: the vendor driver divides by a fixed
/// 1 632 000 (1000 packets of 204 bytes, in bits).
pub fn ber_e9(err: u32) -> u32 {
    (1_000_000_000u64 * err as u64 / 1_632_000) as u32
}

/// Per-layer BER registers: `(request bit, layer-active bit of 0x60, first error-count register, counter-restart bit of 0x5F)`.
const BER_LAYERS: [(u32, u8, u8, u8); 3] =
    [(field::BER_A, 0x01, 0x64, 0x01), (field::BER_B, 0x02, 0x67, 0x02), (field::BER_C, 0x04, 0x6A, 0x04)];

/// Demodulator instance bound to an I2C address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pn2031 {
    pub addr: u8,
    /// The descriptor's stream parameter (`TunerDesc::stream_param`), captured when the kit is built.
    pub stream_param: u32,
}

fn append<T: UsbControl>(dev: &mut Device<T>, addr: u8, pairs: &[(u8, u8)]) -> Result<()> {
    for &(r, v) in pairs {
        dev.i2c_batch_append(addr, r, &[v])?;
    }
    Ok(())
}

impl Pn2031 {
    /// `addr` 0 selects [`DEFAULT_ADDR`], as the vendor driver does.
    pub fn new(addr: u8, stream_param: u32) -> Self {
        Pn2031 { addr: if addr == 0 { DEFAULT_ADDR } else { addr }, stream_param }
    }

    fn write1<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8, v: u8) -> Result<()> {
        dev.i2c_reg_write(self.addr, reg, &[v])
    }

    fn read1<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8) -> Result<u8> {
        let mut b = [0u8; 1];
        dev.i2c_reg_read(self.addr, reg, &mut b)?;
        Ok(b[0])
    }

    fn batch<T: UsbControl>(&self, dev: &mut Device<T>, pairs: &[(u8, u8)]) -> Result<()> {
        append(dev, self.addr, pairs)?;
        dev.i2c_batch_flush()
    }

    /// Init / Open configuration: `0x70 ← 00`, 1 ms, `0x70 ← FF` (immediate),
    /// then twelve batches of single-byte register writes, then
    /// indirect `0x71 ← E5` through `0x6D`/`0x6E` (immediate). Every register
    /// write carries one data byte, also the 24-bit words (four writes each).
    pub fn configure<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.write1(dev, 0x70, 0x00)?;
        sleep(Duration::from_millis(1));
        self.write1(dev, 0x70, 0xFF)?;
        self.batch(dev, &CFG_BASE)?;
        self.batch(dev, &CFG_WINDOW04_A)?;
        self.batch(dev, &CFG_WINDOW04_B)?;
        self.batch(dev, &CFG_IND_A7)?;
        self.batch(dev, &CFG_IND_DC)?;
        self.batch(dev, &CFG_IND_B0)?;
        append(dev, self.addr, &CFG_TS_HEAD)?;
        dev.i2c_batch_append(self.addr, 0x51, &[ts_select(self.stream_param)])?;
        self.batch(dev, &CFG_TS_TAIL)?;
        self.batch(dev, &CFG_IND_1E)?;
        self.batch(dev, &CFG_WORDS_A)?;
        let mut words = COEF_WORDS.iter();
        for n in COEF_BATCHES {
            for &(idx, w) in words.by_ref().take(n) {
                let b = w.to_be_bytes();
                append(dev, self.addr, &[(0x28, idx), (0x29, b[1]), (0x2A, b[2]), (0x2B, b[3])])?;
            }
            dev.i2c_batch_flush()?;
        }
        self.write1(dev, 0x6D, 0x71)?;
        self.write1(dev, 0x6E, 0xE5)
    }

    /// Before the tuner is programmed: `0x08 ← 01` (immediate), then the raw
    /// write [`TUNE_RAW`].
    pub fn pre_tune<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.write1(dev, 0x08, 0x01)?;
        dev.i2c_write(self.addr, &TUNE_RAW)
    }

    /// Restart of the C/N measurement: `0x45 |= 0x10`, then `0x45 &= 0x6F`
    /// (both immediate, from one read of `0x45`).
    pub fn restart_cn<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        let v = self.read1(dev, 0x45)?;
        self.write1(dev, 0x45, v | 0x10)?;
        self.write1(dev, 0x45, v & 0x6F)
    }

    /// After the tuner is programmed: `0x08 ← 00`, C/N restart, `0x53 ← 00`
    /// then `07` (immediate), `0x5F ← 00` then `07` (one batch, all three BER
    /// counters restarted), indirect `0xB1 ← 07` then `00` (immediate). No
    /// delay, no lock wait.
    ///
    /// The vendor driver ignores failures after the first write; this
    /// version stops at the first failing transfer and reports it.
    pub fn post_tune<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.write1(dev, 0x08, 0x00)?;
        self.restart_cn(dev)?;
        self.write1(dev, 0x53, 0x00)?;
        self.write1(dev, 0x53, 0x07)?;
        self.batch(dev, &[(0x5F, 0x00), (0x5F, 0x07)])?;
        self.write1(dev, 0x50, 0xB1)?;
        self.write1(dev, 0x51, 0x07)?;
        self.write1(dev, 0x51, 0x00)
    }

    /// Read one layer's 24-bit error count (three one-byte reads) and restart
    /// its counter (`0x5F` bit cleared then set, one batch).
    fn layer_errors<T: UsbControl>(&self, dev: &mut Device<T>, first: u8, bit: u8) -> Result<u32> {
        let hi = self.read1(dev, first)? as u32;
        let mid = self.read1(dev, first + 1)? as u32;
        let lo = self.read1(dev, first + 2)? as u32;
        let r = self.read1(dev, 0x5F)?;
        self.batch(dev, &[(0x5F, r & !bit), (0x5F, (r & !bit) | bit)])?;
        Ok((hi << 16) | (mid << 8) | lo)
    }

    /// Signal status. `st.mask` selects the BER layers to read; lock, C/N,
    /// strength (`18000 − C/N`) and quality are always produced.
    ///
    /// * lock: `0x0A & 0x0F` ≥ 2 → [`lock::SYNC_STARTED`], ≥ 5 → [`lock::SIGNAL`],
    ///   ≥ 9 → [`lock::LOCKED`]; indirect `0x84` (window `0x6D`/`0x6E`) bit0 → 0x100;
    /// * C/N: raw `0x46`/`0x47`, then the measurement is restarted;
    /// * BER: layer in use when its bit of `0x60` is set, else [`BER_UNUSED`];
    /// * quality: 20 with signal, 60 when locked, then 80 / 90 / 95 above
    ///   22 / 23 / 24 dB C/N (100 instead of 95 when the sync state is 9 or more).
    pub fn status<T: UsbControl>(&self, dev: &mut Device<T>, st: &mut SignalStatus) -> Result<()> {
        let requested = st.mask;
        st.lock = 0;
        st.quality = 0;
        let s = self.read1(dev, 0x0A)? & 0x0F;
        self.write1(dev, 0x6D, 0x84)?;
        let r84 = self.read1(dev, 0x6E)?;
        st.sync_state = s;
        let mut lk = 0u32;
        if s >= 2 {
            lk |= lock::SYNC_STARTED;
        }
        if s >= 5 {
            lk |= lock::SIGNAL;
        }
        if s >= 9 {
            lk |= lock::LOCKED;
        }
        if r84 & 0x01 != 0 {
            lk |= lock::REG81_BIT7;
        }
        st.lock = lk;
        let mut mask = field::LOCK;
        if lk & lock::SIGNAL != 0 {
            st.quality = 20;
        }
        if lk & lock::LOCKED != 0 {
            st.quality = 60;
        }

        let raw = ((self.read1(dev, 0x46)? as u16) << 8) | self.read1(dev, 0x47)? as u16;
        self.restart_cn(dev)?;
        let cnr = cnr_mdb_from_raw(raw);
        st.cnr_mdb = cnr;
        st.strength = 18000 - cnr;
        mask |= field::CNR | field::STRENGTH;

        if requested & (field::BER_A | field::BER_B | field::BER_C) != 0 {
            let active = self.read1(dev, 0x60)?;
            for (idx, &(bit, active_bit, first, restart_bit)) in BER_LAYERS.iter().enumerate() {
                if requested & bit == 0 {
                    continue;
                }
                st.ber[idx] = if active & active_bit != 0 {
                    ber_e9(self.layer_errors(dev, first, restart_bit)?)
                } else {
                    BER_UNUSED
                };
                mask |= bit;
            }
        }

        if cnr > 22000 {
            st.quality = 80;
        }
        if cnr > 23000 {
            st.quality = 90;
        }
        if cnr > 24000 {
            st.quality = if s >= 9 { 100 } else { 95 };
        }
        st.mask = mask | field::QUALITY;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cnr_table_points() {
        assert_eq!(cnr_tenth_db_from_raw(0), 0);
        assert_eq!(cnr_tenth_db_from_raw(19648), 0);
        assert_eq!(cnr_tenth_db_from_raw(u16::MAX), 0);
        assert_eq!(cnr_tenth_db_from_raw(1), 300);
        assert_eq!(cnr_tenth_db_from_raw(787), 300);
        assert_eq!(cnr_tenth_db_from_raw(788), 300);
        assert_eq!(cnr_tenth_db_from_raw(3180), 160);
        assert_eq!(cnr_tenth_db_from_raw(18187), 10);
        assert_eq!(cnr_tenth_db_from_raw(19647), 1);
        // Interpolation: 160 - (10 * 188 - 188) / 376 = 160 - 4 (4.5 truncated).
        assert_eq!(cnr_tenth_db_from_raw(3368), 156);
        // Just below a threshold the lower entry is used: 170 - (10 * 338 - 169) / 339.
        assert_eq!(cnr_tenth_db_from_raw(3179), 161);
        assert_eq!(cnr_mdb_from_raw(3180), 16000);
    }

    #[test]
    fn cnr_table_is_continuous_and_monotonic() {
        let mut prev = cnr_tenth_db_from_raw(1);
        for raw in 2..CN_RAW_LIMIT as u16 {
            let v = cnr_tenth_db_from_raw(raw);
            assert!(v <= prev, "raw {raw}: {v} > {prev}");
            assert!(prev - v <= 1, "raw {raw}: step {prev} -> {v}");
            prev = v;
        }
        // Each step equals the gap to the next higher threshold.
        for w in CN_TABLE.windows(2) {
            assert_eq!(w[0].0 - w[1].0, w[1].2);
            assert_eq!(w[1].1 - w[0].1, 10);
        }
        assert_eq!(CN_RAW_LIMIT - CN_TABLE[0].0, CN_TABLE[0].2);
    }

    #[test]
    fn ber_formula() {
        assert_eq!(ber_e9(0), 0);
        assert_eq!(ber_e9(1632), 1_000_000);
        assert_eq!(ber_e9(1000), 612_745);
        assert_eq!(ber_e9(1_632_000), 1_000_000_000);
        // Counts above 7 008 306 overflow 32 bits; the vendor driver keeps the low 32 bits too.
        assert_eq!(ber_e9(0xFF_FFFF), (10_280_156_250u64 & 0xFFFF_FFFF) as u32);
    }

    #[test]
    fn ts_select_bit0() {
        assert_eq!(ts_select(0), 1);
        assert_eq!(ts_select(1), 0);
        assert_eq!(ts_select(0x10), 1);
    }

    #[test]
    fn default_address() {
        assert_eq!(Pn2031::new(0, 0).addr, 0x10);
        assert_eq!(Pn2031::new(0x12, 0).addr, 0x12);
    }

    #[test]
    fn batches_fit_the_bridge_buffer() {
        let biggest = [
            CFG_BASE.len(),
            CFG_WINDOW04_A.len(),
            CFG_WINDOW04_B.len(),
            CFG_IND_A7.len(),
            CFG_IND_DC.len(),
            CFG_IND_B0.len(),
            CFG_TS_HEAD.len() + 1 + CFG_TS_TAIL.len(),
            CFG_IND_1E.len(),
            CFG_WORDS_A.len(),
            4 * COEF_BATCHES.iter().max().unwrap(),
        ];
        for n in biggest {
            assert!(n * 6 <= crate::device::I2C_BATCH_MAX);
        }
        assert_eq!(COEF_BATCHES.iter().sum::<usize>(), COEF_WORDS.len());
    }
}
