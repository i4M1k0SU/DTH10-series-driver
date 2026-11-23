//! The key RAM that the descrambler and the transform engine take their
//! keys from: 64-bit entries, written through three registers.

use crate::hw::{mmr_r, mmr_w};

/// Number of entries.
const ENTRIES: u32 = 128;

/// The registers: the entry's two words, and the index whose write stores
/// them.
const DATA0: u32 = 0x1A24;
const DATA1: u32 = 0x1A28;
const INDEX: u32 = 0x1A2C;
/// INDEX: the previous write has gone through.
const INDEX_READY: u32 = 0x80;
/// How often INDEX is read for [`INDEX_READY`].
const READY_READS: u32 = 10_000;

/// Write one entry: `hi` and `lo` are the two words of the 64-bit value.
/// 0x1A2C bit 7 says the previous write has gone through; false when it
/// never did.
pub fn write(slot: u32, hi: u32, lo: u32) -> bool {
    if !(0..READY_READS).any(|_| mmr_r(INDEX) & INDEX_READY != 0) {
        return false;
    }
    mmr_w(DATA1, hi);
    mmr_w(DATA0, lo);
    mmr_w(INDEX, slot);
    true
}

/// Zero every entry: nothing of an earlier session stays behind. The host
/// leaves this to the firmware (doing it from the host takes a second).
/// Returns how many entries were written: all of them unless the RAM stopped
/// taking writes.
pub fn clear() -> u32 {
    (0..ENTRIES).take_while(|&slot| write(slot, 0, 0)).count() as u32
}
