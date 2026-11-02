//! Tuning systems and channel tables.
//!
//! ISDB-T: UHF 13–62ch, 6 MHz raster, centre = 473 + 6·(ch−13) MHz + 1/7 MHz.
//! ISDB-S: the transponder IF centre frequency as seen by the tuner (BS-IF /
//! CS-IF after a 10.678 GHz LNB) plus the TS to pick within the transponder,
//! either as its TSID or as its relative TS number in the TMCC table.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum System {
    IsdbT,
    IsdbS,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Channel {
    pub name: String,
    pub system: System,
    /// RF/IF frequency in Hz as programmed into the tuner.
    pub frequency_hz: u32,
    /// ISDB-S: the transport stream ID to select when `slot` is `None` or the
    /// TMCC table has no usable entry for it (ARIB numbering, e.g. 0x4010).
    pub tsid: u16,
    /// LNB voltage request for BS/CS (0 = backend default).
    pub lnb_volt: u32,
    /// ISDB-S: relative TS number within the transponder (index into the TMCC
    /// TS-ID table), when the channel is named that way.
    pub slot: Option<u8>,
}

/// UHF channel centre frequency in Hz (ARIB STD-B31: 473 + 1/7 MHz for ch13).
pub fn uhf_center_hz(ch: u32) -> Option<u32> {
    if !(13..=62).contains(&ch) {
        return None;
    }
    Some(473_000_000 + 6_000_000 * (ch - 13) + 142_857)
}

/// BS transponder IF centre frequencies (Hz) after a 10.678 GHz LNB, odd
/// transponders BS1..BS23 (ARIB STD-B20). BS-IF = RF − 10.678 GHz.
pub const BS_IF_HZ: &[(u8, u32)] = &[
    (1, 1_049_480_000),
    (3, 1_087_840_000),
    (5, 1_126_200_000),
    (7, 1_164_560_000),
    (9, 1_202_920_000),
    (11, 1_241_280_000),
    (13, 1_279_640_000),
    (15, 1_318_000_000),
    (17, 1_356_360_000),
    (19, 1_394_720_000),
    (21, 1_433_080_000),
    (23, 1_471_440_000),
];

/// CS110 transponder IF centre frequencies (Hz), even ND2..ND24.
pub const CS_IF_HZ: &[(u8, u32)] = &[
    (2, 1_613_000_000),
    (4, 1_653_000_000),
    (6, 1_693_000_000),
    (8, 1_733_000_000),
    (10, 1_773_000_000),
    (12, 1_813_000_000),
    (14, 1_853_000_000),
    (16, 1_893_000_000),
    (18, 1_933_000_000),
    (20, 1_973_000_000),
    (22, 2_013_000_000),
    (24, 2_053_000_000),
];

pub fn bs_if_hz(transponder: u8) -> Option<u32> {
    BS_IF_HZ.iter().find(|(n, _)| *n == transponder).map(|(_, f)| *f)
}

pub fn cs_if_hz(transponder: u8) -> Option<u32> {
    CS_IF_HZ.iter().find(|(n, _)| *n == transponder).map(|(_, f)| *f)
}

/// CS110 transport stream IDs: network_id << 12 | transponder << 4, the
/// network being 6 for ND02/ND08/ND10 and 7 for the other transponders (NIT
/// and TMCC TS-ID tables as received on 2026-11-01). A CS110 transponder
/// carries one TS, so tuning picks relative TS 0 and this TSID is only the
/// fallback and what the log shows.
pub const CS_TSID: &[(u8, u16)] = &[
    (2, 0x6020),
    (4, 0x7040),
    (6, 0x7060),
    (8, 0x6080),
    (10, 0x60A0),
    (12, 0x70C0),
    (14, 0x70E0),
    (16, 0x7100),
    (18, 0x7120),
    (20, 0x7140),
    (22, 0x7160),
    (24, 0x7180),
];

/// ARIB TSID of a BS transponder's relative TS (0x4000 | tp<<4 | ts; the
/// actual TSID of a BS TS may differ, which is why BS channels are selected
/// by relative TS number or an explicit TSID) or of a CS110 transponder
/// ([`CS_TSID`]; `ts` is ignored, a CS110 transponder has one TS).
pub fn arib_tsid(bs: bool, transponder: u8, ts: u8) -> u16 {
    if bs {
        0x4000 | ((transponder as u16) << 4) | (ts as u16 & 0xF)
    } else {
        CS_TSID.iter().find(|(n, _)| *n == transponder).map_or(0x6000 | ((transponder as u16) << 4), |(_, t)| *t)
    }
}

impl Channel {
    /// ISDB-T, UHF physical channel 13..62.
    pub fn terrestrial(uhf: u32) -> Option<Channel> {
        let f = uhf_center_hz(uhf)?;
        Some(Channel {
            name: format!("{uhf}ch"),
            system: System::IsdbT,
            frequency_hz: f,
            tsid: 0,
            lnb_volt: 0,
            slot: None,
        })
    }

    /// ISDB-T at an arbitrary centre frequency.
    pub fn terrestrial_hz(hz: u32) -> Channel {
        Channel { name: format!("{hz} Hz"), system: System::IsdbT, frequency_hz: hz, tsid: 0, lnb_volt: 0, slot: None }
    }

    /// BS transponder (odd 1..23) and relative TS number.
    pub fn bs(transponder: u8, slot: u8) -> Option<Channel> {
        let f = bs_if_hz(transponder)?;
        Some(Channel {
            name: format!("BS{transponder:02}/TS{slot}"),
            system: System::IsdbS,
            frequency_hz: f,
            tsid: arib_tsid(true, transponder, slot),
            lnb_volt: 0,
            slot: Some(slot),
        })
    }

    /// CS110 transponder (even 2..24) and relative TS number.
    pub fn cs(transponder: u8, slot: u8) -> Option<Channel> {
        let f = cs_if_hz(transponder)?;
        Some(Channel {
            name: format!("ND{transponder:02}/TS{slot}"),
            system: System::IsdbS,
            frequency_hz: f,
            tsid: arib_tsid(false, transponder, slot),
            lnb_volt: 0,
            slot: Some(slot),
        })
    }

    /// ISDB-S at an IF frequency with an explicit TSID (`None` = the first TS
    /// of the TMCC table).
    pub fn satellite_hz(if_hz: u32, tsid: Option<u16>) -> Channel {
        Channel {
            name: match tsid {
                Some(t) => format!("{if_hz} Hz TSID {t:04x}"),
                None => format!("{if_hz} Hz"),
            },
            system: System::IsdbS,
            frequency_hz: if_hz,
            tsid: tsid.unwrap_or(0),
            lnb_volt: 0,
            slot: if tsid.is_none() { Some(0) } else { None },
        }
    }

    /// Select a TSID explicitly instead of a relative TS number.
    pub fn with_tsid(mut self, tsid: u16) -> Channel {
        self.tsid = tsid;
        self.slot = None;
        self.name = format!("{} TSID {tsid:04x}", self.name);
        self
    }

    /// Parse a channel name:
    ///
    /// * `13`..`62`: UHF physical channel (ISDB-T);
    /// * `BS15`, `BS15_1`, `BS15/TS1`: BS transponder (odd) and relative TS number (default 0);
    /// * `ND02`, `ND02_0`: CS110 transponder (even) and relative TS number.
    ///
    /// Case-insensitive; a `ch` suffix on the UHF number is accepted.
    pub fn parse(name: &str) -> Option<Channel> {
        let s = name.trim();
        let upper = s.to_ascii_uppercase();
        if let Some(rest) = upper.strip_prefix("BS").or_else(|| upper.strip_prefix("ND")) {
            let bs = upper.starts_with("BS");
            let (tp, slot) = split_tp_slot(rest)?;
            return if bs { Channel::bs(tp, slot) } else { Channel::cs(tp, slot) };
        }
        let digits = upper.strip_suffix("CH").unwrap_or(&upper);
        let uhf: u32 = digits.parse().ok()?;
        Channel::terrestrial(uhf)
    }
}

/// `"15"`, `"15_1"`, `"15/TS1"` → (15, 1).
fn split_tp_slot(s: &str) -> Option<(u8, u8)> {
    let (tp, slot) = match s.split_once(['_', '/']) {
        Some((tp, rest)) => (tp, rest.strip_prefix("TS").unwrap_or(rest)),
        None => (s, "0"),
    };
    Some((tp.parse().ok()?, slot.parse().ok()?))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uhf_frequencies() {
        assert_eq!(uhf_center_hz(13), Some(473_142_857));
        assert_eq!(uhf_center_hz(27), Some(557_142_857));
        assert_eq!(uhf_center_hz(62), Some(767_142_857));
        assert_eq!(uhf_center_hz(12), None);
        assert_eq!(uhf_center_hz(63), None);
    }

    #[test]
    fn parses_names() {
        let c = Channel::parse("27").unwrap();
        assert_eq!((c.system, c.frequency_hz), (System::IsdbT, 557_142_857));
        assert_eq!(Channel::parse("27ch").unwrap(), c);
        let b = Channel::parse("bs15_1").unwrap();
        assert_eq!((b.system, b.frequency_hz, b.tsid, b.slot), (System::IsdbS, 1_318_000_000, 0x40F1, Some(1)));
        assert_eq!(Channel::parse("BS15/TS1").unwrap(), b);
        assert_eq!(Channel::parse("BS15").unwrap().slot, Some(0));
        let n = Channel::parse("ND02").unwrap();
        assert_eq!((n.frequency_hz, n.tsid), (1_613_000_000, 0x6020));
        let n = Channel::parse("ND16").unwrap();
        assert_eq!((n.frequency_hz, n.tsid, n.slot), (1_893_000_000, 0x7100, Some(0)));
        assert!(Channel::parse("BS16").is_none());
        assert!(Channel::parse("ND03").is_none());
        assert!(Channel::parse("99").is_none());
        assert!(Channel::parse("BS").is_none());
    }

    #[test]
    fn cs110_tables_agree() {
        // Every CS110 transponder has an IF and a TSID, both keyed by its number,
        // and the TSID carries the transponder number in bits 4..11.
        assert_eq!(CS_TSID.len(), CS_IF_HZ.len());
        for (&(nd, f), &(nd2, tsid)) in CS_IF_HZ.iter().zip(CS_TSID) {
            assert_eq!(nd, nd2);
            assert_eq!((tsid >> 4) & 0xFF, nd as u16);
            assert!(matches!(tsid >> 12, 6 | 7));
            assert_eq!(Channel::cs(nd, 0).map(|c| (c.frequency_hz, c.tsid)), Some((f, tsid)));
        }
        assert_eq!(arib_tsid(true, 15, 1), 0x40F1);
    }

    #[test]
    fn explicit_tsid() {
        let c = Channel::parse("BS15").unwrap().with_tsid(0x40F2);
        assert_eq!((c.tsid, c.slot), (0x40F2, None));
        let s = Channel::satellite_hz(1_613_000_000, None);
        assert_eq!(s.slot, Some(0));
    }
}
