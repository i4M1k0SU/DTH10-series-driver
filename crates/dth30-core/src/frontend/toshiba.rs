//! The TOSHIBA kit (DT-H50/U2 board 0xB4 and others): demodulator 0x20,
//! tuner 0xC0 written directly, and an EEPROM at 0xA0 whose byte 0 corrects
//! the first IF (docs/26 4-tuning §6).
//!
//! The tuner is an up-converting double conversion on a 16 MHz reference:
//! LO1 = RF + (1092 + E) MHz rounded up to the next 250 kHz step, LO2 =
//! LO1 − RF − 57 MHz rounded up to the next 62.5 kHz step, so the second IF
//! is about 57 MHz. Both dividers and the band go out in one 5-byte write to
//! registers 0x01–0x05. The demodulator sends 204-byte packets (inferred).
//!
//! Nothing here is verified on hardware; the worked examples in the tests
//! come from re-deriving the formulas (docs/26 `tune_ref.py`), not from
//! vendor output.

use crate::Result;

use super::Dev::{self, D, T};
use super::regs::{self, Bus};
use super::{Frontend, demod};

/// 8-bit I2C address of the EEPROM holding the IF correction.
pub const EEPROM_ADDR: u8 = 0xA0;
/// Attempts at reading the IF correction before a tune gives up.
const EEPROM_ATTEMPTS: u32 = 3;
/// Nominal first IF (MHz), before the EEPROM correction.
pub const IF1_BASE_MHZ: u32 = 1092;
/// Second IF (Hz).
pub const IF2_HZ: u32 = 57_000_000;
/// PLL reference (Hz).
pub const REF_HZ: u64 = 16_000_000;

/// Init table: 108 demodulator registers (0x20), then 10 tuner registers
/// (0xC0).
#[rustfmt::skip]
pub(super) const INIT: &[(Dev, u8, u8)] = &[
    (D, 0x02, 0x00), (D, 0x03, 0x00), (D, 0x04, 0x00), (D, 0x05, 0x00), (D, 0x06, 0x00), (D, 0x07, 0x00),
    (D, 0x08, 0x00), (D, 0x0C, 0x00), (D, 0x11, 0x21), (D, 0x12, 0x0B), (D, 0x13, 0x2C), (D, 0x14, 0x0A),
    (D, 0x15, 0x40), (D, 0x16, 0x02), (D, 0x17, 0x70), (D, 0x18, 0x31), (D, 0x19, 0x13), (D, 0x1A, 0x31),
    (D, 0x1B, 0x13), (D, 0x1C, 0x00), (D, 0x1D, 0xAA), (D, 0x1E, 0xA8), (D, 0x1F, 0xA8), (D, 0x20, 0x4D),
    (D, 0x21, 0xFF), (D, 0x22, 0x00), (D, 0x23, 0x4C), (D, 0x24, 0x4C), (D, 0x25, 0xFF), (D, 0x26, 0xFF),
    (D, 0x27, 0x0C), (D, 0x28, 0x60), (D, 0x29, 0x6B), (D, 0x2A, 0x40), (D, 0x2B, 0x40), (D, 0x2C, 0xFF),
    (D, 0x2D, 0x00), (D, 0x2E, 0xFF), (D, 0x2F, 0x00), (D, 0x30, 0x28), (D, 0x31, 0x0F), (D, 0x32, 0xA0),
    (D, 0x34, 0x3F), (D, 0x38, 0x01), (D, 0x39, 0x70), (D, 0x3A, 0x10), (D, 0x3B, 0x30), (D, 0x3C, 0x00),
    (D, 0x3D, 0x10), (D, 0x3E, 0x08), (D, 0x3F, 0x0C), (D, 0x40, 0x0C), (D, 0x41, 0x00), (D, 0x42, 0x00),
    (D, 0x43, 0x4F), (D, 0x44, 0xFF), (D, 0x46, 0x20), (D, 0x47, 0x00), (D, 0x48, 0x90), (D, 0x49, 0xE6),
    (D, 0x4A, 0x02), (D, 0x4B, 0x54), (D, 0x4C, 0x00), (D, 0x50, 0x04), (D, 0x51, 0x68), (D, 0x52, 0x20),
    (D, 0x54, 0x57), (D, 0x55, 0xF1), (D, 0x56, 0x20), (D, 0x57, 0x70), (D, 0x58, 0x20), (D, 0x5C, 0x50),
    (D, 0x5D, 0x00), (D, 0x70, 0x18), (D, 0x71, 0x08), (D, 0x72, 0x00), (D, 0x75, 0x00), (D, 0x76, 0xA6),
    (D, 0x77, 0xD8), (D, 0x7C, 0x00), (D, 0x7D, 0x52), (D, 0xB0, 0xA9), (D, 0xB1, 0x20), (D, 0xB2, 0x3C),
    (D, 0xB3, 0x68), (D, 0xB4, 0xEF), (D, 0xB5, 0xFF), (D, 0xB6, 0xFF), (D, 0xB7, 0xFF), (D, 0xB8, 0xFF),
    (D, 0xB9, 0xFC), (D, 0xBA, 0x00), (D, 0xBB, 0x00), (D, 0xBC, 0x00), (D, 0xC2, 0x10), (D, 0xC7, 0x00),
    (D, 0xC8, 0x00), (D, 0xC9, 0x00), (D, 0xCA, 0x00), (D, 0xCB, 0x00), (D, 0xCC, 0x00), (D, 0xCD, 0x00),
    (D, 0xCE, 0x00), (D, 0xCF, 0x00), (D, 0xE4, 0x1A), (D, 0xEC, 0x00), (D, 0xEF, 0x01), (D, 0xFE, 0x00),
    (T, 0x01, 0xB1), (T, 0x02, 0x52), (T, 0x03, 0x10), (T, 0x04, 0x64), (T, 0x05, 0x91), (T, 0x09, 0x20),
    (T, 0x0A, 0x1E), (T, 0x0B, 0x31), (T, 0x0C, 0xFF), (T, 0x0D, 0x80),
];

/// Standby (the kernel driver's start-up sequence for these boards too).
#[rustfmt::skip]
pub(super) const SLEEP: &[(Dev, u8, u8)] = &[(T, 0x09, 0xE0), (D, 0x03, 0x10), (D, 0x04, 0x20)];

/// `(LO1, LO2)` (Hz) for `hz` and the EEPROM correction `e` (added to
/// 1092 MHz as an unsigned byte, as the vendor does). Both are rounded *up*
/// to the next step even when already on one.
pub fn lo_hz(hz: u32, e: u8) -> (u64, u64) {
    let hz = u64::from(hz);
    let lo1 = 250_000 * ((hz + 1_000_000 * (u64::from(e) + u64::from(IF1_BASE_MHZ))) / 250_000 + 1);
    let lo2 = 62_500 * ((lo1 - hz - u64::from(IF2_HZ)) / 62_500 + 1);
    (lo1, lo2)
}

/// The band bits of register 0x01 (upper nibble), by RF frequency.
fn band(hz: u32) -> u8 {
    const BANDS: [(u32, u8); 10] = [
        (95_000_000, 0xB0),
        (180_000_000, 0xA0),
        (260_000_000, 0x90),
        (335_000_000, 0x80),
        (425_000_000, 0x70),
        (490_000_000, 0x60),
        (570_000_000, 0x50),
        (645_000_000, 0x40),
        (730_000_000, 0x30),
        (810_000_000, 0x20),
    ];
    BANDS.iter().find(|&&(th, _)| hz < th).map_or(0x10, |&(_, b)| b)
}

/// Integer part and the top `bits` bits of the fraction of `lo / 16 MHz`
/// (the vendor scales the fraction to 32 bits and shifts it down).
fn divide(lo: u64, bits: u32) -> (u64, u64) {
    let n = lo / REF_HZ;
    let frac32 = (u128::from(lo) << 32) / u128::from(REF_HZ) - (u128::from(n) << 32);
    (n, (frac32 >> (32 - bits)) as u64)
}

/// Tuner registers 0x01–0x05 for `hz` with IF correction `e`.
///
/// `0x01 = band | F1[5:2]`, `0x02 = N1`, `0x03 = F1[1:0] << 4 | F2[2:0]`,
/// `0x04 = F2[11:4]`, `0x05 = N2 << 1 | F2[12]`, where N1/F1 (6-bit
/// fraction) divide LO1 and N2/F2 (13-bit fraction) divide LO2 by 16 MHz.
/// F2 bit 3 goes nowhere; with LO2 on a 62.5 kHz (16 MHz / 256) raster the
/// low 5 bits of F2 are always 0 anyway.
pub fn tune_regs(hz: u32, e: u8) -> [u8; 5] {
    let (lo1, lo2) = lo_hz(hz, e);
    let (n1, f1) = divide(lo1, 6);
    let (n2, f13) = divide(lo2, 13);
    [
        band(hz) | ((f1 >> 2) & 0x0F) as u8,
        n1 as u8,
        (((f1 & 3) << 4) | (f13 & 7)) as u8,
        (f13 >> 4) as u8,
        ((n2 << 1) as u8) | ((f13 >> 12) & 1) as u8,
    ]
}

/// Read the IF correction byte (EEPROM 0xA0, offset 0), up to three times.
fn if_correction<B: Bus>(fe: &mut Frontend<B>) -> Result<u8> {
    let mut attempt = 1;
    loop {
        match regs::read_reg(&mut fe.bus, EEPROM_ADDR, 0x00) {
            Ok(e) => return Ok(e),
            Err(e) if attempt < EEPROM_ATTEMPTS => {
                log::debug!("TOSHIBA: IF correction read failed ({e}), retrying");
                attempt += 1;
            }
            Err(e) => return Err(e),
        }
    }
}

/// The tune sequence. The IF correction is read from the EEPROM on every
/// tune, as the vendor does.
pub(super) fn tune<B: Bus>(fe: &mut Frontend<B>, hz: u32) -> Result<()> {
    fe.demod.set_ts_output(&mut fe.bus, false)?;
    let e = if_correction(fe)?;
    fe.tw_n(0x01, &tune_regs(hz, e))?;
    fe.dw(demod::reg::RESYNC, 0x50)?;
    fe.finish_tune(hz, 0xA7);
    Ok(())
}

/// Signal level from `v` = demodulator 0x82 + 0x83 (the sum of the two
/// bytes), piecewise linear in signed integer arithmetic: 55 up to 77, then
/// `(1141500 − 8000v) / 10000` up to 116, `(589260 − 3320v) / 10000` up to
/// 250, `(−24516 − 832v) / 10000` up to 345, `(600030 − 2674v) / 10000` above.
pub fn level(v: u32) -> i32 {
    let v = v as i32;
    if v <= 77 {
        55
    } else if v <= 116 {
        (1_141_500 - 8000 * v) / 10_000
    } else if v <= 250 {
        (589_260 - 3320 * v) / 10_000
    } else if v <= 345 {
        (-24_516 - 832 * v) / 10_000
    } else {
        (600_030 - 2674 * v) / 10_000
    }
}

#[cfg(test)]
mod tests {
    use super::super::KitKind;
    use super::super::testutil::{Ev, Rig, Seq};
    use super::*;
    use crate::Error;

    const DA: u8 = 0x20;
    const TA: u8 = 0xC0;

    /// `(hz, E, LO1, LO2, registers 0x01..0x05)` from the re-derived formulas
    /// (docs/26 `tune_ref.py`).
    const VECTORS: &[(u32, u8, u64, u64, [u8; 5])] = &[
        (473_142_857, 0, 1_565_250_000, 1_035_125_000, [0x6D, 0x61, 0x10, 0x64, 0x81]),
        (557_142_857, 0, 1_649_250_000, 1_035_125_000, [0x51, 0x67, 0x10, 0x64, 0x81]),
        (767_142_857, 0, 1_859_250_000, 1_035_125_000, [0x23, 0x74, 0x10, 0x64, 0x81]),
        (93_142_857, 0, 1_185_250_000, 1_035_125_000, [0xB1, 0x4A, 0x10, 0x64, 0x81]),
        (111_142_857, 0, 1_203_250_000, 1_035_125_000, [0xA3, 0x4B, 0x10, 0x64, 0x81]),
        (219_142_857, 0, 1_311_250_000, 1_035_125_000, [0x9F, 0x51, 0x10, 0x64, 0x81]),
        (225_142_857, 0, 1_317_250_000, 1_035_125_000, [0x95, 0x52, 0x10, 0x64, 0x81]),
        (297_142_857, 0, 1_389_250_000, 1_035_125_000, [0x8D, 0x56, 0x10, 0x64, 0x81]),
        (335_142_857, 0, 1_427_250_000, 1_035_125_000, [0x73, 0x59, 0x10, 0x64, 0x81]),
        (647_142_857, 0, 1_739_250_000, 1_035_125_000, [0x3B, 0x6C, 0x10, 0x64, 0x81]),
        (473_142_857, 3, 1_568_250_000, 1_038_125_000, [0x60, 0x62, 0x10, 0xC4, 0x81]),
        (473_142_857, 0xFF, 1_820_250_000, 1_290_125_000, [0x6C, 0x71, 0x10, 0x44, 0xA1]),
        (93_142_857, 7, 1_192_250_000, 1_042_125_000, [0xB8, 0x4A, 0x10, 0x44, 0x82]),
        (767_142_857, 0x10, 1_875_250_000, 1_051_125_000, [0x23, 0x75, 0x10, 0x64, 0x83]),
    ];

    #[test]
    fn pll_matches_the_reference_vectors() {
        for &(hz, e, lo1, lo2, regs) in VECTORS {
            assert_eq!(lo_hz(hz, e), (lo1, lo2), "{hz} E={e}");
            assert_eq!(tune_regs(hz, e), regs, "{hz} E={e}");
        }
    }

    #[test]
    fn second_if_is_about_57_mhz() {
        for &(hz, e, ..) in VECTORS {
            let (lo1, lo2) = lo_hz(hz, e);
            let if2 = lo1 - u64::from(hz) - lo2;
            assert!((57_000_000 - 62_500..57_000_000).contains(&if2), "{hz}: {if2}");
        }
    }

    #[test]
    fn bands() {
        assert_eq!(band(94_999_999), 0xB0);
        assert_eq!(band(95_000_000), 0xA0);
        assert_eq!(band(809_999_999), 0x20);
        assert_eq!(band(810_000_000), 0x10);
    }

    fn tune_seq(regs: [u8; 5]) -> Seq {
        let mut tuner = vec![0x01];
        tuner.extend_from_slice(&regs);
        Seq::new()
            .w(DA, 0x1E, 0xA8)
            .w(DA, 0x1F, 0xA8)
            .read(EEPROM_ADDR, 0x00, 1)
            .raw(TA, &tuner)
            .w(DA, 0x01, 0x50)
            .w(DA, 0x76, 0xA7)
            .delay(30)
            .w(DA, 0x1E, 0x80)
            .w(DA, 0x1F, 0x08)
    }

    #[test]
    fn tune_sequences() {
        for &(hz, e, _, _, regs) in VECTORS {
            let mut fe = Frontend::new(Rig::new(), KitKind::Toshiba);
            fe.bus().mock.set(EEPROM_ADDR, 0x00, e);
            fe.tune(hz).unwrap();
            assert_eq!(fe.bus().ev, tune_seq(regs).0, "{hz} E={e}");
            assert_eq!(fe.frequency(), Some(hz));
        }
        // the spelled-out UHF 13 case
        let mut fe = Frontend::new(Rig::new(), KitKind::Toshiba);
        fe.tune(473_142_857).unwrap();
        assert_eq!(fe.bus().ev[3], Ev::W(0xC0, vec![0x01, 0x6D, 0x61, 0x10, 0x64, 0x81]));
    }

    #[test]
    fn eeprom_is_read_up_to_three_times() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Toshiba);
        fe.bus().fail_always(EEPROM_ADDR);
        assert!(matches!(fe.tune(473_142_857), Err(Error::I2c { .. })));
        let ev = &fe.bus().ev;
        assert_eq!(ev.iter().filter(|e| matches!(e, Ev::R(EEPROM_ADDR, ..))).count(), 3 * 10);
        assert!(!ev.iter().any(|e| matches!(e, Ev::W(TA, _))));
        assert_eq!(fe.frequency(), None);

        // a transient failure: the second read succeeds
        let mut fe = Frontend::new(Rig::new(), KitKind::Toshiba);
        fe.bus().mock.set(EEPROM_ADDR, 0x00, 3);
        fe.bus().fail_times(EEPROM_ADDR, 10);
        fe.tune(473_142_857).unwrap();
        assert!(fe.bus().ev.contains(&Ev::W(TA, vec![0x01, 0x60, 0x62, 0x10, 0xC4, 0x81])));
    }

    #[test]
    fn init_sequence() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Toshiba);
        fe.init().unwrap();
        let ev = &fe.bus().ev;
        assert_eq!(ev.len(), 118 + 1);
        assert_eq!(ev[0], Ev::W(DA, vec![0x02, 0x00]));
        assert_eq!(ev[107], Ev::W(DA, vec![0xFE, 0x00]));
        assert_eq!(ev[108], Ev::W(TA, vec![0x01, 0xB1]));
        assert_eq!(ev[117], Ev::W(TA, vec![0x0D, 0x80]));
        for (e, &(dev, reg, value)) in ev.iter().zip(INIT) {
            let addr = if dev == D { DA } else { TA };
            assert_eq!(*e, Ev::W(addr, vec![reg, value]));
        }
        assert_eq!(ev[118], Ev::W(DA, vec![0x02, 0x00]));
        assert!(fe.is_initialized());
    }

    #[test]
    fn init_after_sleep_retunes() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Toshiba);
        fe.init().unwrap();
        fe.tune(473_142_857).unwrap();
        fe.set_mode(2).unwrap();
        fe.bus().ev.clear();
        fe.sleep().unwrap();
        assert_eq!(fe.bus().ev, Seq::new().w(TA, 0x09, 0xE0).w(DA, 0x03, 0x10).w(DA, 0x04, 0x20).0);
        assert!(!fe.is_initialized());
        fe.bus().ev.clear();
        fe.init().unwrap();
        let ev = &fe.bus().ev;
        assert_eq!(ev[118], Ev::W(DA, vec![0x02, 0x80]));
        assert_eq!(ev[119..], tune_seq([0x6D, 0x61, 0x10, 0x64, 0x81]).0[..]);
    }

    #[test]
    fn modes() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Toshiba);
        assert!(matches!(fe.set_mode(1), Err(Error::Unsupported(_))));
        assert_eq!(fe.mode(), 0);
        fe.set_mode(2).unwrap();
        fe.set_mode(0).unwrap();
        assert_eq!(fe.bus().ev, Seq::new().w(DA, 0x02, 0x80).w(DA, 0x02, 0x00).0);
    }

    #[test]
    fn levels() {
        assert_eq!(level(0), 55);
        assert_eq!(level(77), 55);
        assert_eq!(level(78), (1_141_500 - 624_000) / 10_000);
        assert_eq!(level(116), 21);
        assert_eq!(level(117), 20);
        assert_eq!(level(250), -24);
        assert_eq!(level(251), -23);
        assert_eq!(level(345), -31);
        assert_eq!(level(346), -32);
        assert_eq!(level(510), -76);

        let mut fe = Frontend::new(Rig::new(), KitKind::Toshiba);
        fe.bus().mock.set(DA, 0x82, 0x80);
        fe.bus().mock.set(DA, 0x83, 0x10);
        assert_eq!(fe.signal_level().unwrap(), level(0x90));
        assert_eq!(fe.bus().ev, Seq::new().read(DA, 0x82, 2).0);
    }

    #[test]
    fn status_uses_per_layer_periods_and_both_flag_bits() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Toshiba);
        let m = &mut fe.bus().mock;
        for (i, v) in [0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 1, 0, 2, 0, 3].into_iter().enumerate() {
            m.set(DA, 0x9D + i as u8, v);
        }
        m.set(DA, 0x81, 0x80);
        let s = fe.status().unwrap();
        assert_eq!(s.ber.map(|b| (b.errors, b.period)), [(1, 1), (2, 2), (3, 3)]);
        assert!(!s.ews);
        assert_eq!(s.attenuator, None);
        assert_eq!(s.cnr_db, 0.0);
        fe.bus().mock.set(DA, 0xB2, 0x02);
        assert!(fe.ews().unwrap());
        assert!(fe.bus().ev.ends_with(&Seq::new().read(DA, 0x81, 1).read(DA, 0xB2, 1).0));
    }
}
