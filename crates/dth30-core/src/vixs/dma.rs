//! Initialisation of the chip's own DMA engine ("XC_DMA", MMR 0xC10..0xC54)
//! and of the 128-entry table at MMR 0x1A24..0x1A2C
//! (docs/26 2-vixs-bringup.md §4.2 c).
//!
//! The table loop follows the vendor's polling of bit 7 of 0x1A2C exactly:
//! an entry is done after 30 reads with bit 7 set; while no read with bit 7
//! clear has been seen yet, up to five more rounds of 30 are spent (this
//! budget is shared by all entries). The vendor polls without bound; here
//! each entry gives up after [`READ_LIMIT`] reads.
//!
//! Not verified on hardware.

use super::mmr::{Chip, INT_ENABLE};
use crate::bus::RemotePci;
use crate::{Error, Result};

/// DMA interrupt status / enable registers.
pub const DMA_INT_STATUS: u32 = 0xC50;
pub const DMA_INT_MASK: u32 = 0xC54;
/// The bits the vendor enables in both (its two channel masks, 0xD | 1).
pub const DMA_INT_BITS: u32 = 0xD;
/// The table registers: two data words, then index (bit 7 = status).
pub const TABLE: [u32; 3] = [0x1A24, 0x1A28, 0x1A2C];
pub const TABLE_ENTRIES: u32 = 128;
/// Per-entry bound of status reads.
pub const READ_LIMIT: u32 = 10_000;

pub fn init<B: RemotePci>(chip: &mut Chip<B>) -> Result<()> {
    let v = chip.mmr_read_checked(0x000)? | 0x402;
    chip.mmr_write_checked(0x000, v)?;
    chip.mmr_write_checked(0x000, v & !0x402)?;

    let mut budget = 0u32;
    for idx in 0..TABLE_ENTRIES {
        chip.mmr_write_checked(TABLE[0], 0)?;
        chip.mmr_write_checked(TABLE[1], 0)?;
        chip.mmr_write_checked(TABLE[2], idx)?;
        let mut n = 0;
        let mut reads = 0;
        loop {
            reads += 1;
            if reads > READ_LIMIT {
                return Err(Error::Timeout);
            }
            if chip.mmr_read_checked(TABLE[2])? & 0x80 == 0 {
                budget = 100;
                continue;
            }
            n += 1;
            if n == 30 {
                if budget >= 10 {
                    break;
                }
                budget += 2;
                n = 0;
            }
        }
    }

    chip.mmr_modify_checked(DMA_INT_STATUS, |v| v | DMA_INT_BITS)?;
    chip.mmr_modify_checked(INT_ENABLE, |v| v | 0x100)?;
    chip.mmr_modify_checked(DMA_INT_MASK, |v| v | DMA_INT_BITS)
}

/// The vendor's DMA interrupt handling: acknowledge the enabled bits.
pub fn isr<B: RemotePci>(chip: &mut Chip<B>) -> Result<()> {
    if chip.mmr_read_checked(DMA_INT_STATUS)? & DMA_INT_BITS != 0 {
        chip.mmr_write_checked(DMA_INT_STATUS, DMA_INT_BITS)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::super::mmr::testutil::*;
    use super::*;
    use crate::bus::mock::Space;

    /// Status reads the vendor makes when bit 7 is always set: 6 rounds of
    /// 30 for the first entry (using up the budget), 30 for each other.
    pub(crate) const ALWAYS_SET_READS: usize = 180 + 127 * 30;

    #[test]
    fn sequence_with_status_always_set() {
        let mut c = chip(0x0300_00B2);
        c.bus().set(Space::Mem, MMR, 0x8000_0000);
        c.bus().queue(Space::Mem, MMR + 0x1A2C, &vec![0x80; ALWAYS_SET_READS]);
        c.bus().set(Space::Mem, MMR + 0xC50, 0x10);
        init(&mut c).unwrap();
        let acc = accesses(&c.bus().ops());
        use Acc::*;
        assert_eq!(&acc[..3], &[R(0), W(0, 0x8000_0402), W(0, 0x8000_0000)]);
        assert_eq!(&acc[3..6], &[W(0x1A24, 0), W(0x1A28, 0), W(0x1A2C, 0)]);
        let polls = acc.iter().filter(|a| **a == R(0x1A2C)).count();
        assert_eq!(polls, ALWAYS_SET_READS);
        assert!(acc.contains(&W(0x1A2C, 127)));
        assert_eq!(
            &acc[acc.len() - 6..],
            &[R(0xC50), W(0xC50, 0x1D), R(0x418), W(0x418, 0x100), R(0xC54), W(0xC54, 0xD)]
        );
    }

    #[test]
    fn a_clear_status_read_skips_the_extra_rounds() {
        let mut c = chip(0x0300_00B2);
        let mut q = vec![0u32, 0];
        q.extend(std::iter::repeat_n(0x80, 128 * 30));
        c.bus().queue(Space::Mem, MMR + 0x1A2C, &q);
        init(&mut c).unwrap();
        let polls = accesses(&c.bus().ops()).iter().filter(|a| **a == Acc::R(0x1A2C)).count();
        assert_eq!(polls, 2 + 128 * 30);
    }

    #[test]
    fn stuck_status_times_out() {
        let mut c = chip(0x0300_00B2);
        // The index written back reads with bit 7 clear forever.
        assert!(matches!(init(&mut c), Err(Error::Timeout)));
    }
}
