//! The SONY SMT-MJ102 and SMT-MJ110 kits (DT-H30/U2 board 0xB2, DT-H50/U2W
//! board 0xC6): the same tuner type behind the demodulator's 0xFE repeater,
//! MJ102 at demodulator 0x20 / tuner 0xC0, MJ110 at 0x30 / 0xC6
//! (docs/26 4-tuning §5). The tune sequence is shared; the init tables,
//! standby sequences, attenuator registers and level formulas differ.
//!
//! The tuner takes the frequency in 1/64 MHz, rounded, in registers
//! 0x14/0x15, between a stop (0x11 = 0x00) and a start (0x11 = 0x02).
//!
//! Nothing here is verified on hardware.

use crate::Result;
use crate::model::KitKind;

use super::Dev::{self, D, T};
use super::regs::Bus;
use super::{Frontend, demod};

/// MJ102 init table (19 tuner, 94 demodulator registers; tuner 0x05 = 0x01,
/// the last tuner write, wakes it up, inferred).
#[rustfmt::skip]
pub(super) const MJ102_INIT: &[(Dev, u8, u8)] = &[
    (T, 0x2C, 0x44), (T, 0x4D, 0x40), (T, 0x7F, 0x02), (T, 0x9A, 0x52), (T, 0x48, 0x5A), (T, 0x76, 0x1A),
    (T, 0x6A, 0x48), (T, 0x64, 0x28), (T, 0x66, 0xE6), (T, 0x35, 0x11), (T, 0x7E, 0x01), (T, 0x0B, 0x99),
    (T, 0x0C, 0x00), (T, 0x10, 0x00), (T, 0x12, 0xCA), (T, 0x16, 0x90), (T, 0x32, 0x36), (T, 0xD8, 0x18),
    (T, 0x05, 0x01), (D, 0x19, 0x13), (D, 0x16, 0x09), (D, 0x15, 0x42), (D, 0x14, 0xE0), (D, 0x11, 0x21),
    (D, 0x0C, 0x00), (D, 0x08, 0x00), (D, 0x07, 0x00), (D, 0x06, 0x00), (D, 0x12, 0x10), (D, 0x05, 0x00),
    (D, 0x04, 0x00), (D, 0x03, 0x00), (D, 0x13, 0x03), (D, 0x17, 0x20), (D, 0x18, 0x29), (D, 0x02, 0x00),
    (D, 0x01, 0x00), (D, 0x2B, 0x40), (D, 0x28, 0x60), (D, 0x27, 0x0C), (D, 0x26, 0x00), (D, 0x23, 0x4C),
    (D, 0x22, 0x80), (D, 0x21, 0xFF), (D, 0x20, 0x00), (D, 0x24, 0x4C), (D, 0x1E, 0xA8), (D, 0x1F, 0xA8),
    (D, 0x1D, 0xAA), (D, 0x1C, 0x3A), (D, 0x25, 0x80), (D, 0x29, 0x6B), (D, 0x2A, 0x40), (D, 0x1B, 0x13),
    (D, 0x1A, 0x29), (D, 0x41, 0x00), (D, 0x3E, 0x08), (D, 0x3D, 0x10), (D, 0x3C, 0x3F), (D, 0x39, 0x0F),
    (D, 0x38, 0x00), (D, 0x34, 0x0F), (D, 0x32, 0x84), (D, 0x31, 0x0F), (D, 0x3A, 0x20), (D, 0x30, 0x20),
    (D, 0x2F, 0x00), (D, 0x2E, 0xFF), (D, 0x3B, 0x21), (D, 0x3F, 0x0C), (D, 0x40, 0x0C), (D, 0x2D, 0x00),
    (D, 0x2C, 0xFF), (D, 0x5C, 0x50), (D, 0x5D, 0x00), (D, 0x58, 0x60), (D, 0x55, 0xF1), (D, 0x54, 0x57),
    (D, 0x52, 0x20), (D, 0x4C, 0x00), (D, 0x4B, 0x54), (D, 0x4A, 0x02), (D, 0x49, 0xE6), (D, 0x48, 0x90),
    (D, 0x50, 0x04), (D, 0x47, 0x00), (D, 0x46, 0x20), (D, 0x44, 0xFF), (D, 0x51, 0x58), (D, 0x56, 0x20),
    (D, 0x57, 0x70), (D, 0x43, 0x4F), (D, 0x42, 0x00), (D, 0xE6, 0x2F), (D, 0xE9, 0x08), (D, 0x5F, 0x80),
    (D, 0xEF, 0x00), (D, 0xC7, 0x00), (D, 0xC2, 0x10), (D, 0xBC, 0x00), (D, 0x7D, 0x52), (D, 0x7C, 0x00),
    (D, 0x77, 0xD8), (D, 0x76, 0x86), (D, 0x75, 0x20), (D, 0xBA, 0x00), (D, 0x72, 0x00), (D, 0x71, 0x08),
    (D, 0x70, 0x18), (D, 0xBB, 0x00), (D, 0xE4, 0x1A), (D, 0xEC, 0x00), (D, 0x5E, 0x01),
];

/// MJ110 init table (19 tuner, 26 demodulator registers; tuner 0x10 = 0x40
/// enables the RF loop-through).
#[rustfmt::skip]
pub(super) const MJ110_INIT: &[(Dev, u8, u8)] = &[
    (T, 0x2C, 0x34), (T, 0x4D, 0x40), (T, 0x7F, 0x02), (T, 0x9A, 0x52), (T, 0x48, 0x5A), (T, 0x76, 0x1A),
    (T, 0x6A, 0x48), (T, 0x64, 0x28), (T, 0x66, 0xE6), (T, 0x35, 0x0E), (T, 0x7E, 0x01), (T, 0x0B, 0x44),
    (T, 0x0C, 0x00), (T, 0x10, 0x40), (T, 0x12, 0xCA), (T, 0x16, 0x90), (T, 0x32, 0x36), (T, 0xD8, 0x18),
    (T, 0x05, 0x01), (D, 0x03, 0x80), (D, 0x09, 0x10), (D, 0x11, 0x26), (D, 0x12, 0x0C), (D, 0x13, 0x2B),
    (D, 0x14, 0x40), (D, 0x16, 0x00), (D, 0x1C, 0x2A), (D, 0x1D, 0xA0), (D, 0x1E, 0xA8), (D, 0x1F, 0xA8),
    (D, 0x30, 0x00), (D, 0x31, 0x0D), (D, 0x32, 0x79), (D, 0x34, 0x0F), (D, 0x38, 0x00), (D, 0x39, 0x94),
    (D, 0x3A, 0x20), (D, 0x3B, 0x21), (D, 0x3C, 0x3F), (D, 0x71, 0x08), (D, 0x75, 0x20), (D, 0x76, 0x86),
    (D, 0x77, 0xD8), (D, 0x7D, 0x80), (D, 0xEF, 0x01),
];

/// MJ102 standby.
#[rustfmt::skip]
pub(super) const MJ102_SLEEP: &[(Dev, u8, u8)] = &[
    (T, 0x05, 0x00), (D, 0x1D, 0xAA), (D, 0x1E, 0xAA), (D, 0x1F, 0xA8), (D, 0x1C, 0xAA), (D, 0x04, 0x20),
    (D, 0x03, 0x70),
];

/// MJ110 standby.
#[rustfmt::skip]
pub(super) const MJ110_SLEEP: &[(Dev, u8, u8)] = &[
    (T, 0x05, 0x00), (D, 0x1E, 0xA8), (D, 0x1F, 0xA8), (D, 0x1C, 0xAA), (D, 0x03, 0xF0),
];

/// The tuner's frequency word: `(hz·64 + 500000) / 10⁶` (1/64 MHz,
/// rounded; the same value as the MxL13xRF's tune word of the DTH10
/// series).
pub fn tune_word(hz: u32) -> u16 {
    ((u64::from(hz) * 64 + 500_000) / 1_000_000) as u16
}

/// The tune sequence (both kits).
pub(super) fn tune<B: Bus>(fe: &mut Frontend<B>, hz: u32) -> Result<()> {
    let w = tune_word(hz).to_le_bytes();
    fe.demod.set_ts_output(&mut fe.bus, false)?;
    fe.set_attenuator(true)?;
    fe.dw(0x25, 0x00)?;
    fe.dw(0x23, 0x4D)?; // IF AGC held
    fe.tw(0x11, 0x00)?;
    fe.tw(0x13, 0x15)?;
    fe.tw(0x14, w[0])?;
    fe.tw(0x15, w[1])?;
    fe.tw(0x11, 0x02)?;
    fe.dw(demod::reg::RESYNC, 0x40)?;
    fe.dw(0x23, 0x4C)?; // IF AGC running
    fe.finish_tune(hz, 0x87);
    Ok(())
}

/// Signal level from the AGC value (demodulator 0x82), signed integer
/// arithmetic. MJ102: `(62050 − 686v) / 1000` with the attenuator on,
/// `(83050 − 686v) / 1000` off. MJ110: `(65000 − 575v) / 1000` on,
/// `(78000 − 575v) / 1000` off. `kind` must be one of the two.
pub fn level(kind: KitKind, agc: u8, att_on: bool) -> i32 {
    let v = i32::from(agc);
    let (on, off, slope) = if kind == KitKind::Mj110 { (65_000, 78_000, 575) } else { (62_050, 83_050, 686) };
    ((if att_on { on } else { off }) - slope * v) / 1000
}

#[cfg(test)]
mod tests {
    use super::super::testutil::{Ev, Rig, Seq};
    use super::*;

    #[test]
    fn tune_word_is_the_rounded_64th_of_a_mhz() {
        assert_eq!(tune_word(473_142_857), 0x7649);
        assert_eq!(tune_word(93_142_857), 5961);
        assert_eq!(tune_word(767_142_857), 49097);
        let spec = |hz: u32| ((u64::from(hz) * 64 + 500_000) / 1_000_000) as u16;
        // every 1/64 MHz step boundary region from 40 to 1000 MHz, plus a coarse sweep
        for step in 40 * 64..1000 * 64u32 {
            let edge = step * 15_625 + 7_812; // just below the rounding point
            for hz in [edge - 1, edge, edge + 1, edge + 2, step * 15_625] {
                assert_eq!(tune_word(hz), spec(hz), "{hz}");
            }
        }
        for hz in (0..u32::MAX).step_by(9_999_991) {
            assert_eq!(tune_word(hz), spec(hz), "{hz}");
        }
    }

    fn tune_seq(d: u8, t: u8, att: u8, w: u16) -> Seq {
        Seq::new()
            .w(d, 0x1E, 0xA8)
            .w(d, 0x1F, 0xA8)
            .w(d, 0x1C, att)
            .w(d, 0x25, 0x00)
            .w(d, 0x23, 0x4D)
            .rep(d, t, 0x11, 0x00)
            .rep(d, t, 0x13, 0x15)
            .rep(d, t, 0x14, w as u8)
            .rep(d, t, 0x15, (w >> 8) as u8)
            .rep(d, t, 0x11, 0x02)
            .w(d, 0x01, 0x40)
            .w(d, 0x23, 0x4C)
            .w(d, 0x76, 0x87)
            .delay(30)
            .w(d, 0x1E, 0x80)
            .w(d, 0x1F, 0x08)
    }

    #[test]
    fn mj102_tune() {
        for (hz, w) in [(473_142_857, 0x7649), (93_142_857, 0x1749), (767_142_857, 0xBFC9)] {
            let mut fe = Frontend::new(Rig::new(), KitKind::Mj102);
            fe.tune(hz).unwrap();
            assert_eq!(fe.bus().ev, tune_seq(0x20, 0xC0, 0x3A, w).0, "{hz}");
        }
    }

    #[test]
    fn mj110_tune() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Mj110);
        fe.tune(473_142_857).unwrap();
        // the attenuator register stays 0x2A with ATTMode off
        assert_eq!(fe.bus().ev, tune_seq(0x30, 0xC6, 0x2A, 0x7649).0);
        assert_eq!(fe.attenuator(), Some(true));

        let mut fe = Frontend::new(Rig::new(), KitKind::Mj110);
        fe.set_att_mode(true);
        fe.tune(767_142_857).unwrap();
        assert_eq!(fe.bus().ev, tune_seq(0x30, 0xC6, 0x2E, 0xBFC9).0);
        fe.bus().ev.clear();
        fe.set_attenuator(false).unwrap();
        assert_eq!(fe.bus().ev, vec![Ev::W(0x30, vec![0x1C, 0x2E])]);
        assert_eq!(fe.attenuator(), Some(false));
    }

    #[test]
    fn mj102_init() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Mj102);
        fe.init().unwrap();
        let ev = &fe.bus().ev;
        assert_eq!(ev.len(), 113 + 1);
        assert_eq!(ev[0], Ev::W(0x20, vec![0xFE, 0xC0, 0x2C, 0x44]));
        assert_eq!(ev[13], Ev::W(0x20, vec![0xFE, 0xC0, 0x10, 0x00]));
        assert_eq!(ev[18], Ev::W(0x20, vec![0xFE, 0xC0, 0x05, 0x01]));
        assert_eq!(ev[19], Ev::W(0x20, vec![0x19, 0x13]));
        assert_eq!(ev[112], Ev::W(0x20, vec![0x5E, 0x01]));
        for (e, &(dev, reg, value)) in ev.iter().zip(MJ102_INIT) {
            let want = match dev {
                D => Ev::W(0x20, vec![reg, value]),
                T => Ev::W(0x20, vec![0xFE, 0xC0, reg, value]),
            };
            assert_eq!(*e, want);
        }
        assert_eq!(ev[113], Ev::W(0x20, vec![0x52, 0x20]));
    }

    #[test]
    fn mj110_init_retunes() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Mj110);
        fe.tune(473_142_857).unwrap();
        fe.set_mode(1).unwrap(); // writes nothing on this kit
        fe.bus().ev.clear();
        fe.init().unwrap();
        let ev = &fe.bus().ev;
        assert_eq!(ev.len(), 45 + tune_seq(0x30, 0xC6, 0x2A, 0x7649).0.len());
        assert_eq!(ev[0], Ev::W(0x30, vec![0xFE, 0xC6, 0x2C, 0x34]));
        assert_eq!(ev[13], Ev::W(0x30, vec![0xFE, 0xC6, 0x10, 0x40]));
        assert_eq!(ev[19], Ev::W(0x30, vec![0x03, 0x80]));
        assert_eq!(ev[44], Ev::W(0x30, vec![0xEF, 0x01]));
        for (e, &(dev, reg, value)) in ev.iter().zip(MJ110_INIT) {
            let want = match dev {
                D => Ev::W(0x30, vec![reg, value]),
                T => Ev::W(0x30, vec![0xFE, 0xC6, reg, value]),
            };
            assert_eq!(*e, want);
        }
        assert_eq!(ev[45..], tune_seq(0x30, 0xC6, 0x2A, 0x7649).0[..]);
    }

    #[test]
    fn sleeps() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Mj102);
        fe.tune(473_142_857).unwrap();
        fe.bus().ev.clear();
        fe.sleep().unwrap();
        let want = Seq::new()
            .rep(0x20, 0xC0, 0x05, 0x00)
            .w(0x20, 0x1D, 0xAA)
            .w(0x20, 0x1E, 0xAA)
            .w(0x20, 0x1F, 0xA8)
            .w(0x20, 0x1C, 0xAA)
            .w(0x20, 0x04, 0x20)
            .w(0x20, 0x03, 0x70);
        assert_eq!(fe.bus().ev, want.0);
        // these kits keep the frequency for the re-tune on wake-up
        assert_eq!(fe.frequency(), Some(473_142_857));

        let mut fe = Frontend::new(Rig::new(), KitKind::Mj110);
        fe.sleep().unwrap();
        let want = Seq::new()
            .rep(0x30, 0xC6, 0x05, 0x00)
            .w(0x30, 0x1E, 0xA8)
            .w(0x30, 0x1F, 0xA8)
            .w(0x30, 0x1C, 0xAA)
            .w(0x30, 0x03, 0xF0);
        assert_eq!(fe.bus().ev, want.0);
    }

    #[test]
    fn sleep_stops_at_the_first_failure() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Mj110);
        fe.bus().fail_reg(0x30, 0x1F);
        assert!(fe.sleep().is_err());
        assert!(!fe.bus().ev.iter().any(|e| *e == Ev::W(0x30, vec![0x1C, 0xAA])));
    }

    #[test]
    fn levels_and_mode() {
        assert_eq!(level(KitKind::Mj102, 0, true), 62);
        assert_eq!(level(KitKind::Mj102, 100, true), (62_050 - 68_600) / 1000);
        assert_eq!(level(KitKind::Mj102, 100, true), -6);
        assert_eq!(level(KitKind::Mj102, 100, false), 14);
        assert_eq!(level(KitKind::Mj110, 100, true), 7);
        assert_eq!(level(KitKind::Mj110, 200, false), -37);

        let mut fe = Frontend::new(Rig::new(), KitKind::Mj102);
        fe.set_mode(1).unwrap();
        fe.set_mode(2).unwrap();
        fe.set_mode(3).unwrap();
        assert_eq!(fe.bus().ev, Seq::new().w(0x20, 0x52, 0x62).w(0x20, 0x52, 0x62).w(0x20, 0x52, 0x20).0);
        assert_eq!(fe.mode(), 3);
    }
}
