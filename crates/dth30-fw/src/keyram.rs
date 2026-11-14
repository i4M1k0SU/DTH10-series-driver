//! The key RAM that the descrambler and the transform engine take their
//! keys from: 64-bit entries, written through three registers.

use crate::hw::{mmr_w, mmr_wait};

/// Write one entry: `hi` and `lo` are the two words of the 64-bit value.
/// 0x1A2C bit 7 says the previous write has gone through.
pub fn write(slot: u32, hi: u32, lo: u32) {
    mmr_wait(0x1A2C, 0x80, 0x80);
    mmr_w(0x1A28, hi);
    mmr_w(0x1A24, lo);
    mmr_w(0x1A2C, slot);
}
