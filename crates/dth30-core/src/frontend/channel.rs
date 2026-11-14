//! ISDB-T channel → tuning frequency, as the vendor's property page lists
//! them (docs/26 4-tuning §9.10): the channel centre + 1/7 MHz, in Hz. The
//! driver itself takes frequencies; there is no per-board offset.

/// The `n/7 MHz` offset every channel carries (Hz).
const SEVENTH: u32 = 142_857;

/// A frequency band of the channel plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Band {
    /// VHF 1–12.
    Vhf,
    /// UHF 13–62.
    Uhf,
    /// CATV C13–C63.
    Catv,
}

/// VHF 1–12 centres (MHz).
const VHF_MHZ: [u32; 12] = [93, 99, 105, 173, 179, 185, 191, 195, 201, 207, 213, 219];

/// UHF channel 13–62.
pub fn uhf_hz(ch: u32) -> Option<u32> {
    (13..=62).contains(&ch).then(|| 473_000_000 + 6_000_000 * (ch - 13) + SEVENTH)
}

/// VHF channel 1–12.
pub fn vhf_hz(ch: u32) -> Option<u32> {
    let i = usize::try_from(ch).ok()?.checked_sub(1)?;
    VHF_MHZ.get(i).map(|m| m * 1_000_000 + SEVENTH)
}

/// CATV channel C13–C63. The raster is irregular: C13–C21 from 111 MHz in
/// 6 MHz steps, C22 at 167, C23 at 225, C24–C27 from 233, C28 onwards from
/// 255 MHz (4 MHz above C27).
pub fn catv_hz(ch: u32) -> Option<u32> {
    let mhz = match ch {
        13..=21 => 111 + 6 * (ch - 13),
        22 => 167,
        23 => 225,
        24..=27 => 233 + 6 * (ch - 24),
        28..=63 => 255 + 6 * (ch - 28),
        _ => return None,
    };
    Some(mhz * 1_000_000 + SEVENTH)
}

/// The tuning frequency of a channel.
pub fn channel_hz(band: Band, ch: u32) -> Option<u32> {
    match band {
        Band::Vhf => vhf_hz(ch),
        Band::Uhf => uhf_hz(ch),
        Band::Catv => catv_hz(ch),
    }
}

/// Parse a channel name as the vendor writes them: `1`–`12` (or `01`–`12`)
/// VHF, `13`–`62` UHF, `C13`–`C63` CATV (case-insensitive).
pub fn parse_channel(name: &str) -> Option<(Band, u32)> {
    let name = name.trim();
    if let Some(rest) = name.strip_prefix(['C', 'c']) {
        let ch = rest.parse().ok()?;
        return catv_hz(ch).map(|_| (Band::Catv, ch));
    }
    let ch: u32 = name.parse().ok()?;
    match ch {
        1..=12 => Some((Band::Vhf, ch)),
        13..=62 => Some((Band::Uhf, ch)),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uhf() {
        assert_eq!(uhf_hz(13), Some(473_142_857));
        assert_eq!(uhf_hz(27), Some(557_142_857));
        assert_eq!(uhf_hz(62), Some(767_142_857));
        assert_eq!(uhf_hz(12), None);
        assert_eq!(uhf_hz(63), None);
    }

    #[test]
    fn vhf() {
        assert_eq!(vhf_hz(1), Some(93_142_857));
        assert_eq!(vhf_hz(3), Some(105_142_857));
        assert_eq!(vhf_hz(4), Some(173_142_857));
        assert_eq!(vhf_hz(7), Some(191_142_857));
        assert_eq!(vhf_hz(8), Some(195_142_857));
        assert_eq!(vhf_hz(12), Some(219_142_857));
        assert_eq!(vhf_hz(0), None);
        assert_eq!(vhf_hz(13), None);
    }

    #[test]
    fn catv_matches_the_vendor_list() {
        // The full list from the vendor's property page, C13..C63 (MHz).
        let want = [
            111, 117, 123, 129, 135, 141, 147, 153, 159, 167, 225, 233, 239, 245, 251, 255, 261, 267, 273, 279, 285,
            291, 297, 303, 309, 315, 321, 327, 333, 339, 345, 351, 357, 363, 369, 375, 381, 387, 393, 399, 405, 411,
            417, 423, 429, 435, 441, 447, 453, 459, 465,
        ];
        for (i, mhz) in want.iter().enumerate() {
            assert_eq!(catv_hz(13 + i as u32), Some(mhz * 1_000_000 + 142_857), "C{}", 13 + i);
        }
        assert_eq!(catv_hz(12), None);
        assert_eq!(catv_hz(64), None);
    }

    #[test]
    fn names() {
        assert_eq!(parse_channel("13"), Some((Band::Uhf, 13)));
        assert_eq!(parse_channel("01"), Some((Band::Vhf, 1)));
        assert_eq!(parse_channel("c35"), Some((Band::Catv, 35)));
        assert_eq!(parse_channel("C63"), Some((Band::Catv, 63)));
        assert_eq!(parse_channel("C64"), None);
        assert_eq!(parse_channel("63"), None);
        assert_eq!(parse_channel("0"), None);
        assert_eq!(parse_channel("x"), None);
        let (b, ch) = parse_channel("C13").unwrap();
        assert_eq!(channel_hz(b, ch), Some(111_142_857));
    }
}
