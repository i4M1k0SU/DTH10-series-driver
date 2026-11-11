//! The Toshiba TC905xx-family ISDB-T demodulator all four kits carry, as
//! the vendor's user-mode filter drives it (docs/26 4-tuning §7, §8).
//!
//! The register layout matches the DTH10 series' TC90517
//! (`dth10_core::tc90517::demod`): TS output 0x1E/0x1F, lock 0x80/0xB0,
//! C/N 0x8B–0x8D, BER from 0x9D, repeater 0xFE. What differs is handled
//! here: the C/N polynomial (two coefficients) and its integer division,
//! the clamp of the MJ102/MJ110 kits, the BER periods (one shared period on
//! the SONY kits, one per layer on TOSHIBA) and that a zero BER period
//! reads as BER 0. TMCC is never read; the BER of all three layers is
//! reported whether a layer is in use or not.
//!
//! Nothing here is verified on hardware.

use crate::Result;

use super::regs::{self, Bus};

/// Demodulator registers the front-end code touches outside the tables.
pub mod reg {
    /// Resynchronisation after a tune (0x40 on SONY kits, 0x50 on TOSHIBA).
    pub const RESYNC: u8 = 0x01;
    /// TS output control pair: [`super::TS_OFF`] / [`super::TS_ON`].
    pub const TS_OUT: u8 = 0x1E;
    /// Lock state: bits 3 and 5 clear when locked.
    pub const STATUS_80: u8 = 0x80;
    /// Flags; bit 7 is read as an emergency-warning flag (inferred).
    pub const FLAGS_81: u8 = 0x81;
    /// AGC level, the input of the attenuator control and signal level.
    pub const AGC: u8 = 0x82;
    /// C/N raw value, 24 bits big-endian from here.
    pub const CNR: u8 = 0x8B;
    /// Bit 7 gates the MJ100 VHF toggle.
    pub const STATUS_94: u8 = 0x94;
    /// BER error counts and periods from here.
    pub const BER: u8 = 0x9D;
    /// TS-sync state in the low nibble (8 or more = locked).
    pub const SYNC_B0: u8 = 0xB0;
    /// TOSHIBA: bit 1 joins 0x81 bit 7 for the emergency-warning flag.
    pub const FLAGS_B2: u8 = 0xB2;
    /// The repeater: `[0xFE, tuner, reg, data...]` goes to the tuner.
    pub const REPEATER: u8 = 0xFE;
}

/// 0x1E/0x1F with the TS output stopped.
pub const TS_OFF: [u8; 2] = [0xA8, 0xA8];
/// 0x1E/0x1F with the TS output running.
pub const TS_ON: [u8; 2] = [0x80, 0x08];

/// Bits per 204-byte packet; the BER period counts packets.
pub const BER_PERIOD_BITS: u32 = 1632;

/// Whether the demodulator is locked: `(0x80 & 0x28) == 0 && (0xB0 & 0x0F) >= 8`.
pub fn is_locked(r80: u8, rb0: u8) -> bool {
    r80 & 0x28 == 0 && rb0 & 0x0F >= 8
}

/// Which C/N conversion a kit uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CnrCurve {
    /// MJ100, TOSHIBA: the polynomial, 0 dB for a raw value of 0.
    Plain,
    /// MJ102, MJ110: 32 dB for raw values below 0x300, else the polynomial.
    Clamped,
}

/// Raw C/N values below this read as [`CNR_CLAMP_DB`] on MJ102/MJ110.
pub const CNR_CLAMP_RAW: u32 = 0x300;
/// The C/N MJ102/MJ110 report for small raw values (dB).
pub const CNR_CLAMP_DB: f64 = 32.0;

/// C/N in dB from the 24-bit raw value of 0x8B–0x8D:
/// `P = 10·log10(0x540000 / raw)` with the quotient an integer, then
/// `0.000024P⁴ − 0.0016P³ + 0.0398P² + 0.5491P + 3.0965`.
///
/// The MJ102/MJ110 kits compute the same polynomial in fixed point with a
/// series logarithm; this f64 version is within a few hundredths of a dB
/// of it (estimate, docs/26 4-tuning §10.3). A raw value above 0x540000
/// makes the quotient 0, whose logarithm the vendor code does not guard;
/// it reads as 0 dB here.
pub fn cnr_db(raw: u32, curve: CnrCurve) -> f64 {
    if curve == CnrCurve::Clamped && raw < CNR_CLAMP_RAW {
        return CNR_CLAMP_DB;
    }
    if raw == 0 {
        return 0.0;
    }
    let q = 0x0054_0000 / raw;
    if q == 0 {
        return 0.0;
    }
    let p = 10.0 * f64::from(q).log10();
    0.000024 * p.powi(4) - 0.0016 * p.powi(3) + 0.0398 * p.powi(2) + 0.5491 * p + 3.0965
}

/// Where the BER period of each layer comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BerPeriods {
    /// SONY kits: 11 bytes from 0x9D, the period at 0xA6–0xA7 used for all
    /// three layers.
    Shared,
    /// TOSHIBA: 15 bytes from 0x9D, periods at 0xA6, 0xA8 and 0xAA.
    PerLayer,
}

impl BerPeriods {
    /// Bytes read from [`reg::BER`].
    pub const fn block_len(self) -> usize {
        match self {
            BerPeriods::Shared => 11,
            BerPeriods::PerLayer => 15,
        }
    }
}

/// One layer's error count and measurement period.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BerCount {
    /// 24-bit error count.
    pub errors: u32,
    /// 16-bit period in 204-byte packets.
    pub period: u16,
}

impl BerCount {
    /// `errors / (period × 1632)`, 0 when the period is 0.
    pub fn ber(&self) -> f64 {
        if self.period == 0 {
            0.0
        } else {
            f64::from(self.errors) / (f64::from(self.period) * f64::from(BER_PERIOD_BITS))
        }
    }

    /// `errors / period`, 0 when the period is 0.
    pub fn errors_per_period(&self) -> f64 {
        if self.period == 0 { 0.0 } else { f64::from(self.errors) / f64::from(self.period) }
    }
}

/// Split the BER block read from 0x9D (`periods.block_len()` bytes) into layers A/B/C.
pub fn parse_ber(buf: &[u8], periods: BerPeriods) -> [BerCount; 3] {
    let be24 = |i: usize| u32::from(buf[i]) << 16 | u32::from(buf[i + 1]) << 8 | u32::from(buf[i + 2]);
    let be16 = |i: usize| u16::from(buf[i]) << 8 | u16::from(buf[i + 1]);
    core::array::from_fn(|layer| BerCount {
        errors: be24(3 * layer),
        period: match periods {
            BerPeriods::Shared => be16(9),
            BerPeriods::PerLayer => be16(9 + 2 * layer),
        },
    })
}

/// The demodulator at one I2C address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Demod {
    pub addr: u8,
}

impl Demod {
    pub fn write<B: Bus + ?Sized>(&self, bus: &mut B, reg: u8, value: u8) -> Result<()> {
        regs::write_reg(bus, self.addr, reg, value)
    }

    pub fn read<B: Bus + ?Sized>(&self, bus: &mut B, reg: u8) -> Result<u8> {
        regs::read_reg(bus, self.addr, reg)
    }

    /// Start or stop the TS output (0x1E then 0x1F, one transfer each).
    pub fn set_ts_output<B: Bus + ?Sized>(&self, bus: &mut B, on: bool) -> Result<()> {
        let v = if on { TS_ON } else { TS_OFF };
        self.write(bus, reg::TS_OUT, v[0])?;
        self.write(bus, reg::TS_OUT + 1, v[1])
    }

    /// Lock state (two reads: 0x80, then 0xB0).
    pub fn locked<B: Bus + ?Sized>(&self, bus: &mut B) -> Result<bool> {
        let r80 = self.read(bus, reg::STATUS_80)?;
        let rb0 = self.read(bus, reg::SYNC_B0)?;
        Ok(is_locked(r80, rb0))
    }

    /// The 24-bit C/N raw value.
    pub fn cnr_raw<B: Bus + ?Sized>(&self, bus: &mut B) -> Result<u32> {
        let mut b = [0u8; 3];
        regs::read_regs(bus, self.addr, reg::CNR, &mut b)?;
        Ok(u32::from(b[0]) << 16 | u32::from(b[1]) << 8 | u32::from(b[2]))
    }

    /// Error counts and periods of layers A/B/C.
    pub fn ber_counts<B: Bus + ?Sized>(&self, bus: &mut B, periods: BerPeriods) -> Result<[BerCount; 3]> {
        let mut b = [0u8; 15];
        let buf = &mut b[..periods.block_len()];
        regs::read_regs(bus, self.addr, reg::BER, buf)?;
        Ok(parse_ber(buf, periods))
    }

    /// The raw AGC value (0x82).
    pub fn agc<B: Bus + ?Sized>(&self, bus: &mut B) -> Result<u8> {
        self.read(bus, reg::AGC)
    }
}

#[cfg(test)]
mod tests {
    use super::super::testutil::{Rig, Seq};
    use super::*;

    #[test]
    fn lock_condition() {
        assert!(is_locked(0x00, 0x08));
        assert!(is_locked(0x97, 0x0F)); // other bits of 0x80 do not matter
        assert!(!is_locked(0x08, 0x08));
        assert!(!is_locked(0x20, 0x09));
        assert!(!is_locked(0x00, 0x07));
        assert!(!is_locked(0x00, 0xF7));
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn cnr_values() {
        // Expected values from the formula in f64 (Python), with the integer quotient.
        for (raw, q_db) in [
            (10_000u32, 28.640137945177116), // q = 550
            (40_000, 22.394321748380612),    // q = 137
            (100_000, 18.47535479623564),    // q = 55
            (0x1234, 32.51682172227932),     // q = 1181
            (0x2A_0000, 5.068443905918704),  // q = 2
            (0x2A_0001, 3.0965),             // q = 1 (the division truncates)
            (0x54_0000, 3.0965),
            (0x300, 44.76029563644726),
        ] {
            assert!(close(cnr_db(raw, CnrCurve::Plain), q_db), "{raw:#x}: {}", cnr_db(raw, CnrCurve::Plain));
            assert!(close(cnr_db(raw, CnrCurve::Clamped), q_db), "{raw:#x}");
        }
        assert_eq!(cnr_db(0, CnrCurve::Plain), 0.0);
        assert_eq!(cnr_db(0x54_0001, CnrCurve::Plain), 0.0);
        assert_eq!(cnr_db(0, CnrCurve::Clamped), 32.0);
        assert_eq!(cnr_db(0x2FF, CnrCurve::Clamped), 32.0);
        assert!(close(cnr_db(1000, CnrCurve::Plain), 42.57229286546442));
    }

    #[test]
    fn ber_layouts() {
        let buf = [
            0x00, 0x00, 0x10, // A errors
            0x01, 0x02, 0x03, // B errors
            0xFF, 0xFF, 0xFF, // C errors
            0x00, 0x20, // period at 0xA6
            0x00, 0x40, // 0xA8
            0x00, 0x00, // 0xAA
        ];
        let shared = parse_ber(&buf[..11], BerPeriods::Shared);
        assert_eq!(
            shared,
            [
                BerCount { errors: 0x10, period: 0x20 },
                BerCount { errors: 0x010203, period: 0x20 },
                BerCount { errors: 0xFFFFFF, period: 0x20 },
            ]
        );
        let per = parse_ber(&buf, BerPeriods::PerLayer);
        assert_eq!(per[0].period, 0x20);
        assert_eq!(per[1].period, 0x40);
        assert_eq!(per[2].period, 0);
        assert!(close(per[0].ber(), 16.0 / (32.0 * 1632.0)));
        assert!(close(per[0].errors_per_period(), 0.5));
        assert_eq!(per[2].ber(), 0.0);
        assert_eq!(per[2].errors_per_period(), 0.0);
    }

    #[test]
    fn register_reads() {
        let d = Demod { addr: 0x30 };
        let mut rig = Rig::new();
        rig.mock.set(0x30, 0x80, 0x00);
        rig.mock.set(0x30, 0xB0, 0x08);
        for (i, v) in [0x01u8, 0x23, 0x45].into_iter().enumerate() {
            rig.mock.set(0x30, 0x8B + i as u8, v);
        }
        assert!(d.locked(&mut rig).unwrap());
        assert_eq!(d.cnr_raw(&mut rig).unwrap(), 0x012345);
        d.ber_counts(&mut rig, BerPeriods::Shared).unwrap();
        d.ber_counts(&mut rig, BerPeriods::PerLayer).unwrap();
        d.set_ts_output(&mut rig, true).unwrap();
        d.set_ts_output(&mut rig, false).unwrap();
        let want = Seq::new()
            .read(0x30, 0x80, 1)
            .read(0x30, 0xB0, 1)
            .read(0x30, 0x8B, 3)
            .read(0x30, 0x9D, 11)
            .read(0x30, 0x9D, 15)
            .w(0x30, 0x1E, 0x80)
            .w(0x30, 0x1F, 0x08)
            .w(0x30, 0x1E, 0xA8)
            .w(0x30, 0x1F, 0xA8);
        assert_eq!(rig.ev, want.0);
    }
}
