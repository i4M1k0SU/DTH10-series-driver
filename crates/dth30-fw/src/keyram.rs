//! The key RAM that the descrambler and the transform engine take their
//! keys from: 64-bit entries, written through three registers.

use crate::hw::{mmr_r, mmr_w};

/// Number of entries.
const ENTRIES: u32 = 128;

/// Write one entry: `hi` and `lo` are the two words of the 64-bit value.
/// 0x1A2C bit 7 says the previous write has gone through; false when it
/// never did.
pub fn write(slot: u32, hi: u32, lo: u32) -> bool {
    if !(0..10_000).any(|_| mmr_r(0x1A2C) & 0x80 != 0) {
        return false;
    }
    mmr_w(0x1A28, hi);
    mmr_w(0x1A24, lo);
    mmr_w(0x1A2C, slot);
    true
}

/// Zero every entry: nothing of an earlier session stays behind. The host
/// leaves this to the firmware (doing it from the host takes a second).
/// Returns how many entries were written: all of them unless the RAM stopped
/// taking writes.
pub fn clear() -> u32 {
    (0..ENTRIES).take_while(|&slot| write(slot, 0, 0)).count() as u32
}
