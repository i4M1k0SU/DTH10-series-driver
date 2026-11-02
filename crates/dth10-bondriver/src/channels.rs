//! Tuning spaces and channel tables exposed to TVTest.
//!
//! Space 0: ISDB-T (UHF 13–62ch). Space 1: BS, Space 2: CS110 (ISDB-S), four
//! relative TS numbers per transponder. The frequency tables live in
//! `dth10_tuner::channels`.

use std::collections::HashMap;

pub use dth10_tuner::channels::Channel;
use dth10_tuner::channels::{BS_IF_HZ, CS_IF_HZ};

pub fn space_name(space: u32) -> Option<&'static str> {
    match space {
        0 => Some("地デジ"),
        1 => Some("BS"),
        2 => Some("CS110"),
        _ => None,
    }
}

/// Number of TS slots enumerated per satellite transponder.
const TS_PER_TRANSPONDER: u32 = 4;

pub fn channel(space: u32, ch: u32) -> Option<Channel> {
    match space {
        0 => Channel::terrestrial(ch + 13),
        1 | 2 => {
            let table = if space == 1 { BS_IF_HZ } else { CS_IF_HZ };
            let tp = (ch / TS_PER_TRANSPONDER) as usize;
            let slot = (ch % TS_PER_TRANSPONDER) as u8;
            let (num, _) = *table.get(tp)?;
            if space == 1 { Channel::bs(num, slot) } else { Channel::cs(num, slot) }
        }
        _ => None,
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
    use dth10_tuner::channels::System;

    #[test]
    fn spaces_map_to_channels() {
        let c = channel(0, 14).unwrap();
        assert_eq!((c.system, c.frequency_hz), (System::IsdbT, 557_142_857));
        let b = channel(1, 4 * 7 + 1).unwrap(); // BS15, relative TS 1
        assert_eq!((b.frequency_hz, b.tsid, b.slot), (1_318_000_000, 0x40F1, Some(1)));
        assert_eq!(b.name, "BS15/TS1");
        let n = channel(2, 0).unwrap();
        assert_eq!(n.name, "ND02/TS0");
        assert!(channel(1, 4 * 12).is_none());
        assert!(channel(3, 0).is_none());
    }
}
