//! Tuning spaces and channel tables exposed to TVTest.
//!
//! The tuning spaces: ISDB-T (UHF 13–62ch), BS and CS110 (ISDB-S, four
//! relative TS numbers per transponder), shown in that order (`[Device]
//! Spaces` may leave some out). The frequency tables live in
//! `dth_common::channels`.

use std::collections::HashMap;

pub use dth_common::channels::Channel;
use dth_common::channels::{BS_IF_HZ, CS_IF_HZ, UHF_FIRST};

/// A tuning space. TVTest numbers the spaces a DLL shows from 0 (see
/// `[Device] Spaces`), so its index is not a [`Space`] by itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Space {
    /// ISDB-T (UHF).
    Terrestrial,
    /// BS.
    Bs,
    /// CS110.
    Cs,
}

impl Space {
    /// Every tuning space, in the order they are shown.
    pub const ALL: [Space; 3] = [Space::Terrestrial, Space::Bs, Space::Cs];

    /// The name TVTest shows.
    pub fn name(self) -> &'static str {
        match self {
            Space::Terrestrial => "地デジ",
            Space::Bs => "BS",
            Space::Cs => "CS110",
        }
    }
}

/// Number of TS slots enumerated per satellite transponder.
const TS_PER_TRANSPONDER: u32 = 4;

pub fn channel(space: Space, ch: u32) -> Option<Channel> {
    match space {
        Space::Terrestrial => Channel::terrestrial(ch + UHF_FIRST),
        Space::Bs | Space::Cs => {
            let table = if space == Space::Bs { BS_IF_HZ } else { CS_IF_HZ };
            let tp = (ch / TS_PER_TRANSPONDER) as usize;
            let slot = (ch % TS_PER_TRANSPONDER) as u8;
            let (num, _) = *table.get(tp)?;
            if space == Space::Bs { Channel::bs(num, slot) } else { Channel::cs(num, slot) }
        }
    }
}

/// Keeps UTF-16 strings alive for the pointers handed to TVTest.
#[derive(Default)]
pub struct StringCache {
    map: HashMap<String, Box<[u16]>>,
}

impl StringCache {
    pub fn wide(&mut self, s: &str) -> *const u16 {
        if !self.map.contains_key(s) {
            let w: Vec<u16> = s.encode_utf16().chain(std::iter::once(0)).collect();
            self.map.insert(s.to_owned(), w.into_boxed_slice());
        }
        self.map[s].as_ptr()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dth_common::channels::System;

    #[test]
    fn spaces_map_to_channels() {
        let c = channel(Space::Terrestrial, 14).unwrap();
        assert_eq!((c.system, c.frequency_hz), (System::IsdbT, 557_142_857));
        let b = channel(Space::Bs, 4 * 7 + 1).unwrap(); // BS15, relative TS 1
        assert_eq!((b.frequency_hz, b.tsid, b.slot), (1_318_000_000, 0x40F1, Some(1)));
        assert_eq!(b.name, "BS15/TS1");
        let n = channel(Space::Cs, 0).unwrap();
        assert_eq!(n.name, "ND02/TS0");
        assert!(channel(Space::Bs, 4 * 12).is_none());
        assert_eq!(Space::ALL.map(Space::name), ["地デジ", "BS", "CS110"]);
    }
}
