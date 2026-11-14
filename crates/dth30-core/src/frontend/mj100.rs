//! The SONY SMT-MJ100 kit (DT-H30/U2 board 0xB3): demodulator 0x20 and a
//! register-map tuner at 0xC0, written directly with a trailing 0xFE byte
//! after the data (docs/26 4-tuning §4).
//!
//! The tuner has a fractional-N PLL on a 16 MHz reference (VCO = f·2^m in
//! 450–900 MHz: 7-bit integer, 18-bit fraction), a second synthesiser at
//! (f − 1.75 MHz)·M in 1200–1800 MHz and band-dependent tracking values.
//! The vendor computes them from the frequency in MHz as a single-precision
//! float, compares and multiplies in double and truncates toward zero;
//! [`tune_regs`] does the same (the float rounding of the frequency shows
//! in the PLL fraction).
//!
//! Nothing here is verified on hardware; the worked examples in the tests
//! come from re-deriving the formulas (docs/26 `tune_ref.py`), not from
//! vendor output.

use crate::Result;

use super::Dev::{self, D, T};
use super::regs::Bus;
use super::{Frontend, demod};

/// Init table, written after tuner 0x09 = 0xB1 and 10 ms (24 tuner, 94
/// demodulator registers).
#[rustfmt::skip]
pub(super) const INIT: &[(Dev, u8, u8)] = &[
    (T, 0x49, 0x80), (T, 0x35, 0xBB), (T, 0x3B, 0x00), (T, 0x16, 0x5C), (T, 0x19, 0x36), (T, 0x4C, 0xFB),
    (T, 0x2B, 0x2A), (T, 0x2C, 0x20), (T, 0x0B, 0x16), (T, 0x0C, 0x80), (T, 0x0D, 0x00), (T, 0x93, 0xDA),
    (T, 0x9C, 0x57), (T, 0x89, 0xD8), (T, 0xA6, 0x94), (T, 0xA7, 0x19), (T, 0xA8, 0xD2), (T, 0x86, 0x04),
    (T, 0x4D, 0x93), (T, 0x5B, 0x70), (T, 0x20, 0x80), (T, 0x38, 0x88), (T, 0x87, 0x01), (T, 0x95, 0xFB),
    (D, 0x52, 0x20), (D, 0x01, 0x00), (D, 0x02, 0x00), (D, 0x03, 0x00), (D, 0x04, 0x00), (D, 0x05, 0x00),
    (D, 0x06, 0x00), (D, 0x07, 0x00), (D, 0x08, 0x00), (D, 0x0C, 0x00), (D, 0x11, 0x21), (D, 0x12, 0x04),
    (D, 0x13, 0x03), (D, 0x14, 0xE0), (D, 0x15, 0x42), (D, 0x16, 0x09), (D, 0x17, 0x20), (D, 0x18, 0x29),
    (D, 0x19, 0x13), (D, 0x1A, 0x29), (D, 0x1B, 0x13), (D, 0x1C, 0x2A), (D, 0x1D, 0xAA), (D, 0x1E, 0xA8),
    (D, 0x1F, 0xA8), (D, 0x20, 0x00), (D, 0x21, 0xFF), (D, 0x22, 0x80), (D, 0x23, 0x4C), (D, 0x24, 0x4C),
    (D, 0x25, 0x80), (D, 0x26, 0x00), (D, 0x27, 0x0C), (D, 0x28, 0x60), (D, 0x29, 0x6B), (D, 0x2A, 0x40),
    (D, 0x2B, 0x40), (D, 0x2C, 0xFF), (D, 0x2D, 0x00), (D, 0x2E, 0xFF), (D, 0x2F, 0x00), (D, 0x30, 0x20),
    (D, 0x31, 0x0F), (D, 0x32, 0x84), (D, 0x34, 0x0F), (D, 0x38, 0x00), (D, 0x39, 0x0F), (D, 0x3A, 0x0A),
    (D, 0x3B, 0x21), (D, 0x3C, 0x3F), (D, 0x3D, 0x10), (D, 0x3E, 0x08), (D, 0x3F, 0x0C), (D, 0x40, 0x0C),
    (D, 0x41, 0x00), (D, 0x42, 0x00), (D, 0x43, 0x4F), (D, 0x44, 0xFF), (D, 0x46, 0x20), (D, 0x47, 0x00),
    (D, 0x48, 0x90), (D, 0x49, 0xE6), (D, 0x4A, 0x02), (D, 0x4B, 0x54), (D, 0x4C, 0x00), (D, 0x50, 0x04),
    (D, 0x51, 0x58), (D, 0x54, 0x57), (D, 0x55, 0xF1), (D, 0x56, 0x20), (D, 0x57, 0x70), (D, 0x58, 0x60),
    (D, 0x5C, 0x50), (D, 0x5D, 0x00), (D, 0x5E, 0x01), (D, 0x5F, 0x80), (D, 0x70, 0x18), (D, 0x71, 0x08),
    (D, 0x72, 0x00), (D, 0x75, 0x20), (D, 0x76, 0x86), (D, 0x77, 0xD8), (D, 0x7C, 0x00), (D, 0x7D, 0x52),
    (D, 0xBA, 0x08), (D, 0xBB, 0x00), (D, 0xBC, 0x00), (D, 0xC2, 0x10), (D, 0xC7, 0x00), (D, 0xE4, 0x1A),
    (D, 0xE6, 0x2F), (D, 0xE9, 0x08), (D, 0xEC, 0x00), (D, 0xEF, 0x00),
];

/// Standby: TS and demodulator down, attenuator register off, tuner 0x09 = 0x41.
#[rustfmt::skip]
pub(super) const SLEEP: &[(Dev, u8, u8)] = &[
    (D, 0x1D, 0xAA), (D, 0x1E, 0xAA), (D, 0x1F, 0xA8), (D, 0x1C, 0xAA), (D, 0x04, 0x20), (D, 0x03, 0x70),
    (T, 0x95, 0xFB), (T, 0x09, 0x41),
];

/// `value` of the first `(threshold, value)` with `f <= threshold`, else `default`.
fn ladder(f: f64, steps: &[(f64, u8)], default: u8) -> u8 {
    steps.iter().find(|&&(th, _)| f <= th).map_or(default, |&(_, v)| v)
}

/// Truncation toward zero, as the vendor's float-to-integer conversion.
fn ftol(x: f64) -> i64 {
    x as i64
}

/// Position of `f` (MHz) within its tracking segment (0 at the segment's
/// lower edge, 1 at its upper edge). The vendor multiplies by the
/// reciprocal of the width, kept here as a double constant.
fn segment_pos(f: f64) -> f64 {
    const SEGMENTS: [(f64, f64, f64); 9] = [
        (42.5, 28.0, 1.0 / 14.5),
        (56.0, 42.5, 1.0 / 13.5),
        (85.0, 56.0, 1.0 / 29.0),
        (112.0, 85.0, 1.0 / 27.0),
        (170.0, 112.0, 1.0 / 58.0),
        (225.0, 170.0, 1.0 / 55.0),
        (340.0, 225.0, 1.0 / 115.0),
        (450.0, 340.0, 1.0 / 110.0),
        (680.0, 450.0, 1.0 / 230.0),
    ];
    match SEGMENTS.iter().find(|&&(th, _, _)| f <= th) {
        Some(&(_, base, k)) => (f - base) * k,
        None => (f - 680.0) * (1.0 / 220.0),
    }
}

/// The frequency-dependent tuner registers of a tune to `hz`, in the order
/// the vendor writes them: 0x44, 0x45, 0x46, 0x49, 0x6F, 0x6E, 0x0E–0x11
/// (PLL), 0x70, 0x6A–0x6D (second synthesiser).
pub fn tune_regs(hz: u32) -> [(u8, u8); 15] {
    // MHz as f32, then everything in f64 from that value.
    let f = f64::from((f64::from(hz) * 1e-6) as f32);

    let r44 = ladder(f, &[(75.0, 0xB4), (100.0, 0xDB), (150.0, 0xD2), (200.0, 0xC1)], 0xC0);
    let r45 = ladder(f, &[(75.0, 0xE1), (100.0, 0xC1), (150.0, 0x61), (200.0, 0x21)], 0x01)
        | ladder(f, &[(112.0, 0x04), (225.0, 0x08), (300.0, 0x10)], 0x00);
    let r46 = ladder(
        f,
        &[
            (75.0, 0xE0),
            (100.0, 0xE2),
            (150.0, 0xE5),
            (200.0, 0xE6),
            (300.0, 0xE8),
            (400.0, 0x4A),
            (450.0, 0x4E),
            (500.0, 0x4F),
            (550.0, 0x53),
            (600.0, 0x55),
            (650.0, 0x56),
            (700.0, 0x98),
            (750.0, 0x9B),
            (800.0, 0x9D),
            (850.0, 0x9E),
        ],
        0x9F,
    );
    let r49 = ladder(f, &[(100.0, 0x90), (200.0, 0xA0)], 0xB0);
    let r6f = ladder(
        f,
        &[
            (42.5, 0x18),
            (56.0, 0x58),
            (85.0, 0x88),
            (112.0, 0xC8),
            (170.0, 0x88),
            (225.0, 0xC8),
            (300.0, 0x88),
            (340.0, 0x18),
            (450.0, 0x58),
            (680.0, 0xA8),
        ],
        0xE8,
    );

    let t = segment_pos(f);
    let k = ftol((1.0 - t) * 32.0 + 8.5);
    let r6e = (4 * k) as u8 | if f > 340.0 && f <= 450.0 { 2 } else { 1 };

    // PLL: N = trunc(f / 2^m), F = round(f / 2^m · 2^18) (integer part included).
    let (div, scale) = if f <= 56.0 {
        (1.0, 262_144.0)
    } else if f <= 112.0 {
        (0.5, 131_072.0)
    } else if f <= 225.0 {
        (0.25, 65_536.0)
    } else if f <= 450.0 {
        (0.125, 32_768.0)
    } else {
        (0.0625, 16_384.0)
    };
    let n = if f <= 56.0 { ftol(f) } else { ftol(f * div) };
    let frac = ftol(f * scale + 0.5);
    let r0e = (n & 0x7F) as u8 | if frac == 0 { 0x80 } else { 0 };
    let r0f = (frac >> 10) as u8;
    let r10 = (frac >> 2) as u8;
    let r11 = ((frac << 6) | 0x20) as u8;
    let r70 = (ftol((1.0 - t) * 4.0 + 1.5) | 0x80) as u8;

    // Second synthesiser at (f − 1.75 MHz)·M.
    let fp = f - 1.75;
    let mul = ladder(
        fp,
        &[(50.0, 36), (67.0, 24), (100.0, 18), (150.0, 12), (200.0, 8), (295.0, 6), (400.0, 4), (590.0, 3)],
        2,
    );
    let vco = fp * f64::from(mul);
    let vi = ftol(vco + 0.5);
    let k2 = ftol((1.0 - (vco - 1200.0) * (1.0 / 600.0)) * 32.0 + 8.5);
    let r6a = (4 * k2) as u8
        | if fp <= 67.0 {
            0
        } else if fp <= 400.0 {
            1
        } else {
            3
        };
    let r6b = ladder(
        fp,
        &[
            (50.0, 0x34),
            (67.0, 0x0C),
            (100.0, 0x64),
            (150.0, 0x44),
            (200.0, 0x04),
            (295.0, 0xC4),
            (400.0, 0x84),
            (590.0, 0xC4),
        ],
        0x84,
    );
    let r6c = ((vi >> 6) | 0x80) as u8;
    let r6d = ((vi << 2) | 2) as u8;

    [
        (0x44, r44),
        (0x45, r45),
        (0x46, r46),
        (0x49, r49),
        (0x6F, r6f),
        (0x6E, r6e),
        (0x0E, r0e),
        (0x0F, r0f),
        (0x10, r10),
        (0x11, r11),
        (0x70, r70),
        (0x6A, r6a),
        (0x6B, r6b),
        (0x6C, r6c),
        (0x6D, r6d),
    ]
}

/// The tune sequence.
pub(super) fn tune<B: Bus>(fe: &mut Frontend<B>, hz: u32) -> Result<()> {
    fe.demod.set_ts_output(&mut fe.bus, false)?;
    fe.set_attenuator(true)?;
    fe.tw(0x09, 0xB1)?;
    fe.tw(0x88, 0x7E)?;
    fe.tw(0x8A, 0x38)?;
    for (reg, value) in tune_regs(hz) {
        fe.tw(reg, value)?;
    }
    fe.tw(0x09, 0xF3)?;
    fe.bus.delay_ms(30);
    fe.tw(0x88, 0xFE)?;
    fe.tw(0x09, 0xF3)?;
    fe.dw(demod::reg::RESYNC, 0x40)?;
    fe.finish_tune(hz, 0x87);
    Ok(())
}

/// Signal level from the AGC value (demodulator 0x82), signed integer
/// arithmetic: attenuator on `(5455 − 70v) / 100`, off
/// `(200·(34 − v) / 3 + 4875) / 100`.
pub fn level(agc: u8, att_on: bool) -> i32 {
    let v = i32::from(agc);
    if att_on { (5455 - 70 * v) / 100 } else { (200 * (34 - v) / 3 + 4875) / 100 }
}

#[cfg(test)]
mod tests {
    use super::super::KitKind;
    use super::super::testutil::{Ev, Rig, Seq};
    use super::*;

    const DA: u8 = 0x20;
    const TA: u8 = 0xC0;

    /// The tune-dependent registers from the re-derived formulas (docs/26 `tune_ref.py`).
    const VECTORS: &[(u32, [u8; 15])] = &[
        // 0x44  0x45  0x46  0x49  0x6F  0x6E  0x0E  0x0F  0x10  0x11  0x70  0x6A  0x6B  0x6C  0x6D
        (473_142_857, [0xC0, 0x01, 0x4F, 0xB0, 0xA8, 0x95, 0x1D, 0x92, 0x49, 0x60, 0x85, 0x77, 0xC4, 0x96, 0x1A]),
        (557_142_857, [0xC0, 0x01, 0x55, 0xB0, 0xA8, 0x65, 0x22, 0xD2, 0x49, 0x60, 0x83, 0x3F, 0xC4, 0x9A, 0x0A]),
        (767_142_857, [0xC0, 0x01, 0x9D, 0xB0, 0xE8, 0x6D, 0x2F, 0xF2, 0x49, 0x60, 0x83, 0x5B, 0x84, 0x97, 0xEE]),
        (647_142_857, [0xC0, 0x01, 0x56, 0xB0, 0xA8, 0x35, 0x28, 0x72, 0x49, 0x60, 0x82, 0x8F, 0x84, 0x94, 0x2E]),
        (93_142_857, [0xDB, 0xC5, 0xE2, 0x90, 0xC8, 0x79, 0x2E, 0x92, 0x49, 0x60, 0x84, 0x41, 0x64, 0x99, 0xB6]),
        (117_142_857, [0xD2, 0x69, 0xE5, 0xA0, 0x88, 0x95, 0x1D, 0x49, 0x24, 0xE0, 0x85, 0x79, 0x44, 0x95, 0xA6]),
        (111_142_857, [0xD2, 0x65, 0xE5, 0xA0, 0xC8, 0x25, 0x37, 0x92, 0x49, 0x60, 0x81, 0x89, 0x44, 0x94, 0x86]),
        (167_142_857, [0xC1, 0x29, 0xE6, 0xA0, 0x88, 0x29, 0x29, 0xC9, 0x24, 0xA0, 0x81, 0x85, 0x04, 0x94, 0xAE]),
        (173_142_857, [0xC1, 0x29, 0xE6, 0xA0, 0xC8, 0x99, 0x2B, 0x49, 0x24, 0xA0, 0x85, 0x7D, 0x04, 0x95, 0x6E]),
        (219_142_857, [0xC0, 0x09, 0xE8, 0xB0, 0xC8, 0x2D, 0x36, 0xC9, 0x24, 0xA0, 0x81, 0x89, 0xC4, 0x94, 0x62]),
        (225_142_857, [0xC0, 0x11, 0xE8, 0xB0, 0x88, 0xA1, 0x1C, 0x24, 0x92, 0x60, 0x85, 0x85, 0xC4, 0x94, 0xF2]),
        (297_142_857, [0xC0, 0x11, 0xE8, 0xB0, 0x88, 0x51, 0x25, 0x24, 0x92, 0x60, 0x82, 0xA5, 0x84, 0x92, 0x7A]),
        (387_142_857, [0xC0, 0x01, 0x4A, 0xB0, 0x58, 0x6A, 0x30, 0x64, 0x92, 0x60, 0x83, 0x59, 0x84, 0x98, 0x1A]),
        (57_142_857, [0xB4, 0xE5, 0xE0, 0x90, 0x88, 0x9D, 0x1C, 0x92, 0x49, 0x60, 0x85, 0x84, 0x0C, 0x94, 0xC6]),
    ];

    const ORDER: [u8; 15] = [0x44, 0x45, 0x46, 0x49, 0x6F, 0x6E, 0x0E, 0x0F, 0x10, 0x11, 0x70, 0x6A, 0x6B, 0x6C, 0x6D];

    #[test]
    fn pll_matches_the_reference_vectors() {
        for &(hz, want) in VECTORS {
            let got = tune_regs(hz);
            let regs: Vec<u8> = got.iter().map(|&(r, _)| r).collect();
            let vals: Vec<u8> = got.iter().map(|&(_, v)| v).collect();
            assert_eq!(regs, ORDER);
            assert_eq!(vals, want, "{hz} Hz");
        }
    }

    #[test]
    fn frequency_is_rounded_to_f32_first() {
        // C14: 117.142857 MHz is 117.14286041 as f32, which moves the PLL
        // fraction by one LSB (0x752493 instead of 0x752492) and shows in 0x11.
        let f = f64::from((117_142_857f64 * 1e-6) as f32);
        assert_eq!((f * 65536.0 + 0.5) as i64, 0x75_2493);
        assert_eq!((117.142857f64 * 65536.0 + 0.5) as i64, 0x75_2492);
        assert_eq!(tune_regs(117_142_857)[9], (0x11, 0xE0));
    }

    fn tune_seq(hz: u32) -> Seq {
        let regs = VECTORS.iter().find(|v| v.0 == hz).unwrap().1;
        let mut s = Seq::new().w(DA, 0x1E, 0xA8).w(DA, 0x1F, 0xA8).wfe(TA, 0x95, 0x03);
        s = s.wfe(TA, 0x09, 0xB1).wfe(TA, 0x88, 0x7E).wfe(TA, 0x8A, 0x38);
        for (r, v) in ORDER.iter().zip(regs) {
            s = s.wfe(TA, *r, v);
        }
        s.wfe(TA, 0x09, 0xF3)
            .delay(30)
            .wfe(TA, 0x88, 0xFE)
            .wfe(TA, 0x09, 0xF3)
            .w(DA, 0x01, 0x40)
            .w(DA, 0x76, 0x87)
            .delay(30)
            .w(DA, 0x1E, 0x80)
            .w(DA, 0x1F, 0x08)
    }

    #[test]
    fn tune_sequences() {
        for hz in [473_142_857, 93_142_857, 767_142_857] {
            let mut fe = Frontend::new(Rig::new(), KitKind::Mj100);
            fe.tune(hz).unwrap();
            assert_eq!(fe.bus().ev, tune_seq(hz).0, "{hz} Hz");
            assert_eq!(fe.frequency(), Some(hz));
            assert_eq!(fe.attenuator(), Some(true));
        }
    }

    #[test]
    fn init_sequence() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Mj100);
        fe.init().unwrap();
        let ev = &fe.bus().ev;
        assert_eq!(ev.len(), 2 + 118 + 1);
        assert_eq!(ev[0], Ev::W(TA, vec![0x09, 0xB1, 0xFE]));
        assert_eq!(ev[1], Ev::D(10));
        // the table: 24 tuner registers with the trailing 0xFE, then 94 demodulator registers
        assert_eq!(ev[2], Ev::W(TA, vec![0x49, 0x80, 0xFE]));
        assert_eq!(ev[2 + 16], Ev::W(TA, vec![0xA8, 0xD2, 0xFE]));
        assert_eq!(ev[2 + 23], Ev::W(TA, vec![0x95, 0xFB, 0xFE]));
        assert_eq!(ev[2 + 24], Ev::W(DA, vec![0x52, 0x20]));
        assert_eq!(ev[2 + 117], Ev::W(DA, vec![0xEF, 0x00]));
        for (e, &(dev, reg, value)) in ev[2..120].iter().zip(INIT) {
            let want = match dev {
                D => Ev::W(DA, vec![reg, value]),
                T => Ev::W(TA, vec![reg, value, 0xFE]),
            };
            assert_eq!(*e, want);
        }
        // mode 0, no frequency yet: no tune
        assert_eq!(ev[120], Ev::W(DA, vec![0x52, 0x20]));
        assert!(fe.is_initialized());
    }

    #[test]
    fn init_retunes_and_sleep_forgets_the_frequency() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Mj100);
        fe.tune(473_142_857).unwrap();
        fe.set_mode(2).unwrap();
        fe.bus().ev.clear();
        fe.init().unwrap();
        let ev = &fe.bus().ev;
        assert_eq!(ev[120], Ev::W(DA, vec![0x52, 0x62]));
        assert_eq!(ev[121..], tune_seq(473_142_857).0[..]);

        fe.bus().ev.clear();
        fe.sleep().unwrap();
        let want = Seq::new()
            .w(DA, 0x1D, 0xAA)
            .w(DA, 0x1E, 0xAA)
            .w(DA, 0x1F, 0xA8)
            .w(DA, 0x1C, 0xAA)
            .w(DA, 0x04, 0x20)
            .w(DA, 0x03, 0x70)
            .wfe(TA, 0x95, 0xFB)
            .wfe(TA, 0x09, 0x41);
        assert_eq!(fe.bus().ev, want.0);
        assert!(!fe.is_initialized());
        assert_eq!(fe.frequency(), None);
    }

    #[test]
    fn failed_tune_stops_and_keeps_the_old_frequency() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Mj100);
        fe.tune(473_142_857).unwrap();
        fe.bus().fail_always(TA);
        assert!(fe.tune(93_142_857).is_err());
        assert_eq!(fe.frequency(), Some(473_142_857));
        // TS off went out, then the attenuator write failed ten times
        let ev = &fe.bus().ev;
        let n = ev.len();
        assert_eq!(ev[n - 1], Ev::W(TA, vec![0x95, 0x03, 0xFE]));
    }

    #[test]
    fn first_failure_stops_the_tune() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Mj100);
        fe.bus().fail_always(DA);
        // the TS-off write fails: nothing else is sent
        assert!(fe.tune(473_142_857).is_err());
        assert_eq!(fe.bus().ev.iter().filter(|e| matches!(e, Ev::W(TA, _))).count(), 0);
        assert_eq!(fe.frequency(), None);
    }

    #[test]
    fn late_failures_are_ignored() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Mj100);
        fe.bus().fail_reg(DA, 0x76);
        fe.tune(473_142_857).unwrap();
        assert_eq!(fe.frequency(), Some(473_142_857));
        let ev = &fe.bus().ev;
        assert_eq!(ev.iter().filter(|e| **e == Ev::W(DA, vec![0x76, 0x87])).count(), 10);
        assert_eq!(ev[ev.len() - 3..], Seq::new().delay(30).w(DA, 0x1E, 0x80).w(DA, 0x1F, 0x08).0[..]);
    }

    #[test]
    fn attenuator_and_level() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Mj100);
        fe.set_attenuator(false).unwrap();
        fe.set_attenuator(true).unwrap();
        assert_eq!(fe.bus().ev, Seq::new().wfe(TA, 0x95, 0xFB).wfe(TA, 0x95, 0x03).0);
        assert_eq!(level(0, true), 54);
        assert_eq!(level(100, true), (5455 - 7000) / 100); // -15, truncated toward zero
        assert_eq!(level(100, true), -15);
        assert_eq!(level(34, false), 48);
        assert_eq!(level(0, false), (2266 + 4875) / 100);
        assert_eq!(level(255, false), (-14733 + 4875) / 100);
        assert_eq!(level(255, false), -98);
        fe.bus().mock.set(DA, 0x82, 100);
        assert_eq!(fe.signal_level().unwrap(), -15);
    }

    #[test]
    fn vhf_toggle() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Mj100);
        // not initialised: nothing
        assert_eq!(fe.vhf_toggle_tick().unwrap(), None);
        fe.init().unwrap();
        fe.tune(93_142_857).unwrap();
        fe.bus().ev.clear();
        fe.bus().mock.set(DA, 0x94, 0x80);
        assert_eq!(fe.vhf_toggle_tick().unwrap(), Some(0xA2));
        assert_eq!(fe.vhf_toggle_tick().unwrap(), Some(0xD2));
        fe.bus().mock.set(DA, 0x94, 0x00);
        assert_eq!(fe.vhf_toggle_tick().unwrap(), None);
        let want =
            Seq::new().read(DA, 0x94, 1).wfe(TA, 0xA8, 0xA2).read(DA, 0x94, 1).wfe(TA, 0xA8, 0xD2).read(DA, 0x94, 1);
        assert_eq!(fe.bus().ev, want.0);
        // UHF: not even a read
        fe.tune(473_142_857).unwrap();
        fe.bus().ev.clear();
        fe.bus().mock.set(DA, 0x94, 0x80);
        assert_eq!(fe.vhf_toggle_tick().unwrap(), None);
        assert!(fe.bus().ev.is_empty());
    }
}
