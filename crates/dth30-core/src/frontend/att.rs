//! The two periodic controls the vendor's user-mode filter runs in threads
//! of its own (docs/26 4-tuning §3.3, §4.6), as pure decision logic. The
//! caller drives them on its own schedule through
//! [`super::Frontend::att_tick`] and [`super::Frontend::vhf_toggle_tick`];
//! nothing here sleeps or spawns.

use crate::model::KitKind;

/// How often the vendor samples the AGC for the attenuator control (ms).
pub const ATT_POLL_MS: u32 = 200;
/// Samples averaged per attenuator decision.
pub const ATT_SAMPLES: u32 = 10;
/// How often the vendor runs the MJ100 VHF toggle (ms).
pub const VHF_TOGGLE_MS: u32 = 1000;

/// `(lo, hi)` AGC thresholds of the attenuator control, `None` for a kit
/// without an attenuator (TOSHIBA).
pub const fn att_thresholds(kind: KitKind) -> Option<(u32, u32)> {
    match kind {
        KitKind::Mj100 => Some((50, 120)),
        KitKind::Mj102 => Some((80, 135)),
        KitKind::Mj110 => Some((110, 150)),
        KitKind::Toshiba => None,
    }
}

/// The attenuator decision for one averaged AGC value: switch on when
/// `avg > lo && avg >= hi`, off when `avg <= lo`, leave it in between. Only
/// returns a value when the state changes.
pub fn att_decision(avg: u32, (lo, hi): (u32, u32), att_on: bool) -> Option<bool> {
    if avg > lo && avg >= hi && !att_on {
        Some(true)
    } else if avg <= lo && att_on {
        Some(false)
    } else {
        None
    }
}

/// Attenuator control: collects raw AGC samples (demodulator 0x82), and
/// every [`ATT_SAMPLES`] samples averages them (integer division) and
/// applies [`att_decision`]. The sample set then starts over (the vendor
/// code is read as averaging disjoint sets of ten).
///
/// The vendor skips the loop and switches the attenuator off once when the
/// board's "ATT fixed" model flag and the `ATTMode` setting are both set;
/// no USB board of this family has the flag, so that case is not modelled.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AttControl {
    thresholds: (u32, u32),
    sum: u32,
    count: u32,
}

impl AttControl {
    pub fn new(thresholds: (u32, u32)) -> Self {
        AttControl { thresholds, sum: 0, count: 0 }
    }

    /// The control with a kit's thresholds; `None` for TOSHIBA.
    pub fn for_kit(kind: KitKind) -> Option<Self> {
        att_thresholds(kind).map(Self::new)
    }

    pub fn thresholds(&self) -> (u32, u32) {
        self.thresholds
    }

    /// Forget the collected samples.
    pub fn reset(&mut self) {
        self.sum = 0;
        self.count = 0;
    }

    /// Add one AGC sample; returns the new attenuator state when it should change.
    pub fn push(&mut self, agc: u8, att_on: bool) -> Option<bool> {
        self.sum += u32::from(agc);
        self.count += 1;
        if self.count < ATT_SAMPLES {
            return None;
        }
        let avg = self.sum / self.count;
        self.reset();
        att_decision(avg, self.thresholds, att_on)
    }
}

/// The MJ100 VHF toggle runs below this frequency (inclusive) (Hz).
pub const VHF_TOGGLE_MAX_HZ: u32 = 170_000_000;
/// The two values the toggle alternates tuner register 0xA8 between; the
/// first is the one the init table writes.
pub const VHF_A8: [u8; 2] = [0xD2, 0xA2];

/// The MJ100 VHF toggle: when tuned at or below 170 MHz and demodulator
/// 0x94 bit 7 is set, tuner register 0xA8 flips between 0xD2 and 0xA2.
/// Returns the value to write, if any. The purpose is unknown (a VHF AGC
/// or filter setting, inferred).
pub fn vhf_toggle(freq_hz: Option<u32>, reg94: u8, a8: u8) -> Option<u8> {
    match freq_hz {
        Some(f) if f <= VHF_TOGGLE_MAX_HZ && reg94 & 0x80 != 0 => {
            Some(if a8 == VHF_A8[0] { VHF_A8[1] } else { VHF_A8[0] })
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hysteresis() {
        let t = (50, 120);
        assert_eq!(att_decision(120, t, false), Some(true));
        assert_eq!(att_decision(200, t, false), Some(true));
        assert_eq!(att_decision(200, t, true), None);
        assert_eq!(att_decision(119, t, false), None);
        assert_eq!(att_decision(51, t, true), None);
        assert_eq!(att_decision(50, t, true), Some(false));
        assert_eq!(att_decision(0, t, true), Some(false));
        assert_eq!(att_decision(0, t, false), None);
    }

    #[test]
    fn averages_ten_samples() {
        let mut c = AttControl::for_kit(KitKind::Mj102).unwrap();
        assert_eq!(c.thresholds(), (80, 135));
        // nine samples decide nothing however large
        for _ in 0..9 {
            assert_eq!(c.push(255, false), None);
        }
        // the tenth: (9·255 + 0) / 10 = 229 >= 135
        assert_eq!(c.push(0, false), Some(true));
        // integer average: 9·81 + 80 = 809 → 80 <= 80 switches off
        for _ in 0..9 {
            assert_eq!(c.push(81, true), None);
        }
        assert_eq!(c.push(80, true), Some(false));
        // 9·81 + 89 = 818 → 81: between the thresholds
        for _ in 0..9 {
            c.push(81, true);
        }
        assert_eq!(c.push(89, true), None);
        assert_eq!(AttControl::for_kit(KitKind::Toshiba), None);
        assert_eq!(att_thresholds(KitKind::Mj100), Some((50, 120)));
        assert_eq!(att_thresholds(KitKind::Mj110), Some((110, 150)));
    }

    #[test]
    fn vhf_toggle_conditions() {
        assert_eq!(vhf_toggle(Some(93_142_857), 0x80, 0xD2), Some(0xA2));
        assert_eq!(vhf_toggle(Some(170_000_000), 0xFF, 0xA2), Some(0xD2));
        assert_eq!(vhf_toggle(Some(170_000_001), 0x80, 0xD2), None);
        assert_eq!(vhf_toggle(Some(93_142_857), 0x7F, 0xD2), None);
        assert_eq!(vhf_toggle(None, 0x80, 0xD2), None);
    }
}
