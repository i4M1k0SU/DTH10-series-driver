//! The two tuners of the SHARP_8011 module, write-only behind the
//! demodulator's 0xFE I2C repeater (or directly on the bridge bus when the
//! descriptor's flags bit3 is set), as the vendor driver programs them: one
//! PLL write per tune, no init, wake, standby or signal reading.
//!
//! * Terrestrial (hw 0x102): a 5-byte write, divider in 1/7 MHz steps with a
//!   57 MHz IF, band and charge pump by frequency.
//! * Satellite (hw 0x402): a 4-byte / 1-byte / 10 ms / 2-byte / 1-byte
//!   sequence, divider in 1 MHz steps with a 16 or 32 prescaler, band byte
//!   from nine frequency bands (edges 986 / 1073 / 1154 / 1291 / 1447 / 1615 /
//!   1791 / 1972 MHz).

use core::time::Duration;
use std::thread::sleep;

use crate::device::Device;
use crate::error::Result;
use crate::protocol::UsbControl;
use crate::tuner_bus::TunerBus;

/// The terrestrial PLL write for a tune request frequency in Hz:
/// `{N >> 8 & 7F, N, C1, CP | BAND, 80}` with
/// `N = (1000·f_kHz + 57000500) / 142857`. BAND is 1 below 170 MHz, 2
/// below 402 MHz, else 4; CP comes from a 10-step table. The vendor driver
/// tunes 743143 kHz as 743000 kHz (one divider step lower); the kHz value is
/// the request's Hz value truncated, so 743142857 Hz is not affected.
pub fn ter_pll_bytes(freq_hz: u64) -> [u8; 5] {
    let mut khz = (freq_hz / 1000) as u32;
    if khz == 743_143 {
        khz = 743_000;
    }
    let n = khz.wrapping_mul(1000).wrapping_add(57_000_500) / 142_857;
    let band = if khz < 170_000 {
        1
    } else if khz < 402_000 {
        2
    } else {
        4
    };
    let cp = match khz {
        0..140_000 => 0x80,
        140_000..170_000 => 0xA0,
        170_000..220_000 => 0x60,
        220_000..330_000 => 0xA0,
        330_000..402_000 => 0xE0,
        402_000..450_000 => 0x60,
        450_000..550_000 => 0x80,
        550_000..600_000 => 0xA0,
        600_000..700_000 => 0xC0,
        _ => 0xE0,
    };
    [((n >> 8) & 0x7F) as u8, n as u8, 0xC1, cp | band, 0x80]
}

/// The terrestrial tuner (hw 0x102 kit).
#[derive(Debug, Clone, Copy)]
pub struct TerTuner {
    pub bus: TunerBus,
}

impl TerTuner {
    /// Tune: the one PLL write ([`ter_pll_bytes`]).
    pub fn set_frequency<T: UsbControl>(&self, dev: &mut Device<T>, freq_hz: u64) -> Result<()> {
        self.bus.write(dev, &ter_pll_bytes(freq_hz))
    }
}

/// Lower edges (kHz) of the satellite bands 2..=9; band 1 is below 986 MHz.
const SAT_BANDS_KHZ: [u32; 8] = [986_000, 1_073_000, 1_154_000, 1_291_000, 1_447_000, 1_615_000, 1_791_000, 1_972_000];

/// The satellite PLL programming for a BS/CS IF in Hz.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SatPll {
    /// Divider word: `((f_MHz / P) << 5) | (f_MHz % P)`, P = 16 in bands 1-2, else 32.
    pub word: u16,
    /// Band byte: `VCO << 5 | DIV << 1 | P16 << 4 | 0x04`.
    pub cb: u8,
}

impl SatPll {
    /// The bytes of the four writes, in order.
    pub fn writes(&self) -> [Vec<u8>; 4] {
        let hi = ((self.word >> 8) & 0x1F) as u8;
        [vec![hi, self.word as u8, 0xE0, self.cb & 0xF3], vec![0xE4], vec![0xF4, self.cb], vec![hi]]
    }
}

pub fn sat_pll(if_hz: u64) -> SatPll {
    let khz = (if_hz / 1000) as u32;
    let band = 1 + SAT_BANDS_KHZ.iter().take_while(|&&edge| khz >= edge).count();
    // (divide-by-2, VCO) of the band
    let (div, vco): (u8, u8) = match band {
        1 => (1, 5),
        2 => (1, 6),
        3 => (1, 7),
        4 => (0, 1),
        5 => (0, 2),
        6 => (0, 3),
        7 => (0, 4),
        8 => (0, 5),
        _ => (0, 6),
    };
    let p16 = band <= 2;
    let cb = (vco << 5) | (div << 1) | if p16 { 0x10 } else { 0 } | 0x04;
    let mhz = khz.wrapping_mul(1000).wrapping_add(500) / 1_000_000;
    let p = if p16 { 16 } else { 32 };
    SatPll { word: (((mhz / p) << 5) | (mhz % p)) as u16, cb }
}

/// The satellite tuner (hw 0x402 kit).
#[derive(Debug, Clone, Copy)]
pub struct SatTuner {
    pub bus: TunerBus,
}

impl SatTuner {
    /// Tune: the four writes of [`SatPll::writes`] with 10 ms after the
    /// second. The first failed write ends the sequence and is returned (the
    /// 10 ms are waited all the same).
    pub fn set_frequency<T: UsbControl>(&self, dev: &mut Device<T>, if_hz: u64) -> Result<()> {
        let [w1, w2, w3, w4] = sat_pll(if_hz).writes();
        let r = self.bus.write(dev, &w1).and_then(|()| self.bus.write(dev, &w2));
        sleep(Duration::from_millis(10));
        r?;
        self.bus.write(dev, &w3)?;
        self.bus.write(dev, &w4)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regmock::{RegMock, Write};
    use crate::tuner_bus::TunerPath;

    #[test]
    fn terrestrial_pll_words() {
        // UHF 13 / 27 / 62 centres, as the channel table gives them
        assert_eq!(ter_pll_bytes(473_142_857), [0x0E, 0x7F, 0xC1, 0x84, 0x80]); // N = 3711
        assert_eq!(ter_pll_bytes(557_142_857), [0x10, 0xCB, 0xC1, 0xA4, 0x80]); // N = 4299
        assert_eq!(ter_pll_bytes(767_142_857), [0x16, 0x89, 0xC1, 0xE4, 0x80]); // N = 5769
        // the special case: 743143 kHz is tuned as 743000 kHz, 743142 kHz is not
        assert_eq!(ter_pll_bytes(743_143_000), [0x15, 0xE0, 0xC1, 0xE4, 0x80]); // N = 5600
        assert_eq!(ter_pll_bytes(743_000_000), [0x15, 0xE0, 0xC1, 0xE4, 0x80]);
        assert_eq!(ter_pll_bytes(743_142_857), [0x15, 0xE1, 0xC1, 0xE4, 0x80]); // N = 5601
        assert_eq!(ter_pll_bytes(743_143_999), [0x15, 0xE0, 0xC1, 0xE4, 0x80]);
    }

    #[test]
    fn terrestrial_band_and_charge_pump_edges() {
        for (khz, cp_band) in [
            (90_000, 0x81),
            (139_999, 0x81),
            (140_000, 0xA1),
            (169_999, 0xA1),
            (170_000, 0x62),
            (219_999, 0x62),
            (220_000, 0xA2),
            (329_999, 0xA2),
            (330_000, 0xE2),
            (401_999, 0xE2),
            (402_000, 0x64),
            (449_999, 0x64),
            (450_000, 0x84),
            (549_999, 0x84),
            (550_000, 0xA4),
            (599_999, 0xA4),
            (600_000, 0xC4),
            (699_999, 0xC4),
            (700_000, 0xE4),
            (770_000, 0xE4),
        ] {
            let b = ter_pll_bytes(khz as u64 * 1000);
            assert_eq!(b[3], cp_band, "{khz} kHz");
            let n = (khz * 1000 + 57_000_500) / 142_857;
            assert_eq!(((b[0] as u32) << 8) | b[1] as u32, n, "{khz} kHz");
        }
    }

    #[test]
    fn terrestrial_pll_goes_through_the_repeater() {
        let mut dev = Device::new(RegMock::default());
        let t = TerTuner { bus: TunerBus { tuner_addr: 0x61, path: TunerPath::ViaDemod { demod_addr: 0x18 } } };
        t.set_frequency(&mut dev, 557_142_857).unwrap();
        assert_eq!(
            dev.transport().take_writes(),
            vec![Write::Raw(0x18, vec![0xFE, 0xC2, 0x10, 0xCB, 0xC1, 0xA4, 0x80])]
        );
    }

    #[test]
    fn satellite_pll_words() {
        // BS-1 / BS-15 / ND-24 (IF after a 10.678 GHz LNB)
        assert_eq!(sat_pll(1_049_480_000), SatPll { word: (65 << 5) | 9, cb: 0xD6 }); // 1049 MHz, band 2
        assert_eq!(sat_pll(1_318_000_000), SatPll { word: (41 << 5) | 6, cb: 0x44 }); // 1318 MHz, band 5
        assert_eq!(sat_pll(2_053_000_000), SatPll { word: (64 << 5) | 5, cb: 0xC4 }); // 2053 MHz, band 9
        assert_eq!(
            sat_pll(1_049_480_000).writes(),
            [vec![0x08, 0x29, 0xE0, 0xD2], vec![0xE4], vec![0xF4, 0xD6], vec![0x08]]
        );
    }

    #[test]
    fn satellite_band_edges() {
        for (khz, cb) in [
            (950_000, 0xB6),
            (985_999, 0xB6),
            (986_000, 0xD6),
            (1_072_999, 0xD6),
            (1_073_000, 0xE6),
            (1_153_999, 0xE6),
            (1_154_000, 0x24),
            (1_290_999, 0x24),
            (1_291_000, 0x44),
            (1_446_999, 0x44),
            (1_447_000, 0x64),
            (1_614_999, 0x64),
            (1_615_000, 0x84),
            (1_790_999, 0x84),
            (1_791_000, 0xA4),
            (1_971_999, 0xA4),
            (1_972_000, 0xC4),
            (2_150_000, 0xC4),
        ] {
            let p = sat_pll(khz as u64 * 1000);
            assert_eq!(p.cb, cb, "{khz} kHz");
            let (mhz, d) = (khz / 1000, if cb & 0x10 != 0 { 16 } else { 32 });
            assert_eq!(p.word as u32, ((mhz / d) << 5) | (mhz % d), "{khz} kHz");
        }
    }

    #[test]
    fn satellite_pll_sequence() {
        let mut dev = Device::new(RegMock::default());
        let t = SatTuner { bus: TunerBus { tuner_addr: 0x61, path: TunerPath::ViaDemod { demod_addr: 0x19 } } };
        t.set_frequency(&mut dev, 1_318_000_000).unwrap();
        let w = (41u16 << 5) | 6;
        assert_eq!(
            dev.transport().take_writes(),
            vec![
                Write::Raw(0x19, vec![0xFE, 0xC2, (w >> 8) as u8, w as u8, 0xE0, 0x40]),
                Write::Raw(0x19, vec![0xFE, 0xC2, 0xE4]),
                Write::Raw(0x19, vec![0xFE, 0xC2, 0xF4, 0x44]),
                Write::Raw(0x19, vec![0xFE, 0xC2, (w >> 8) as u8]),
            ]
        );
    }
}
