//! Register (MMR, BAR1) and SDRAM window (FB, BAR0) access to the ViXS chip.
//!
//! Every access is one 32-bit remote PCI memory cycle at `BAR + offset`.
//! The vendor driver has two kinds of accessors and uses them in fixed
//! places (docs/26, 2-vixs-bringup.md §1.3):
//!
//! * plain ones ([`Chip::mmr_read`], [`Chip::fb_write`], ...), and
//! * "checked" ones ([`Chip::mmr_read_checked`], ...) which first re-read
//!   the board-ID register 0x478 and compare it (masked with 0xF3FFFFFF) with
//!   the value read at start-up. Up to six reads may mismatch before the
//!   board is declared removed; after that every checked access fails.
//!
//! The sequences in this module tree use the same kind at the same place as
//! the vendor, so their bus traffic matches it read for read.
//!
//! The FB window is 1 MB of SDRAM selected by RBM_SWAP (0x43C) bits 14:8.
//!
//! Not verified on hardware.

use core::time::Duration;

use crate::bus::RemotePci;
use crate::net2280;
use crate::{Error, Result};

/// Board / chip ID ("XC_RBM_BOARD_ID").
pub const BOARD_ID: u32 = 0x478;
/// SDRAM window select and more ("RBM_SWAP"); bits 14:8 = 1 MB window.
pub const RBM_SWAP: u32 = 0x43C;
/// Mask applied to the board-ID register before comparing.
pub const BOARD_ID_CHECK_MASK: u32 = 0xF3FF_FFFF;
/// Chip-family bits of the board ID; Camino (XCode2) is [`CAMINO`].
pub const CHIP_MASK: u32 = 0xF3FF_0000;
pub const CAMINO: u32 = 0x0300_0000;
/// Free-running timer (used by the heartbeat check and I2C bus recovery).
pub const TIMER: u32 = 0x6130;
/// Top-level interrupt status / enable (bit7 = I2C/GPIO, bit8 = DMA, bit9 = MIPS).
pub const INT_STATUS: u32 = 0x414;
pub const INT_ENABLE: u32 = 0x418;

/// How many times the checked accessors read the board ID before giving up.
const CHECK_READS: u32 = 6;

/// The ViXS chip as seen through the remote PCI bus.
pub struct Chip<B> {
    bus: B,
    /// PCI address of BAR1 (registers).
    pub mmr: u32,
    /// PCI address of BAR0 (the 1 MB SDRAM window).
    pub fb: u32,
    /// The board-ID register as read at start-up (the reference for checks).
    pub board_id: u32,
    removed: bool,
    sleep: fn(Duration),
}

impl<B: RemotePci> Chip<B> {
    pub fn new(bus: B, mmr: u32, fb: u32, board_id: u32, sleep: fn(Duration)) -> Self {
        Self { bus, mmr, fb, board_id, removed: false, sleep }
    }

    pub fn bus(&self) -> &B {
        &self.bus
    }

    pub fn into_bus(self) -> B {
        self.bus
    }

    /// The low byte of the board ID: the board type.
    pub fn board(&self) -> u8 {
        self.board_id as u8
    }

    /// Whether a checked access found the board ID changed.
    pub fn is_removed(&self) -> bool {
        self.removed
    }

    /// Sleep for `d` (the function given at construction).
    pub fn sleep(&self, d: Duration) {
        (self.sleep)(d)
    }

    /// The vendor's `n`-tick busy wait (see [`net2280::ticks`]).
    pub fn wait_ticks(&self, n: u32) {
        self.sleep(net2280::ticks(n))
    }

    pub fn sleep_fn(&self) -> fn(Duration) {
        self.sleep
    }

    // ---- plain accessors ----

    pub fn mmr_read(&self, off: u32) -> Result<u32> {
        self.bus.mem_read(self.mmr + off)
    }

    pub fn mmr_write(&self, off: u32, value: u32) -> Result<()> {
        self.bus.mem_write(self.mmr + off, value)
    }

    pub fn fb_read(&self, off: u32) -> Result<u32> {
        self.bus.mem_read(self.fb + off)
    }

    pub fn fb_write(&self, off: u32, value: u32) -> Result<()> {
        self.bus.mem_write(self.fb + off, value)
    }

    /// Plain read-modify-write: `reg = f(reg)`.
    pub fn mmr_modify(&self, off: u32, f: impl FnOnce(u32) -> u32) -> Result<()> {
        let v = self.mmr_read(off)?;
        self.mmr_write(off, f(v))
    }

    // ---- checked accessors ----

    /// The board-ID check that precedes every checked access.
    pub fn check(&mut self) -> Result<()> {
        if self.removed {
            return Err(Error::Protocol("ViXS board removed"));
        }
        for _ in 0..CHECK_READS {
            let v = self.bus.mem_read(self.mmr + BOARD_ID)?;
            if v & BOARD_ID_CHECK_MASK == self.board_id {
                return Ok(());
            }
        }
        self.removed = true;
        log::error!("ViXS board ID no longer reads {:08x}: board removed", self.board_id);
        Err(Error::Protocol("ViXS board removed (board-ID register changed)"))
    }

    /// The single-read variant of the check the vendor uses before writing
    /// its host-side copy of a command slot.
    pub fn check_once(&mut self) -> Result<()> {
        if self.removed {
            return Err(Error::Protocol("ViXS board removed"));
        }
        if self.bus.mem_read(self.mmr + BOARD_ID)? & BOARD_ID_CHECK_MASK == self.board_id {
            return Ok(());
        }
        self.removed = true;
        Err(Error::Protocol("ViXS board removed (board-ID register changed)"))
    }

    pub fn mmr_read_checked(&mut self, off: u32) -> Result<u32> {
        self.check()?;
        self.mmr_read(off)
    }

    pub fn mmr_write_checked(&mut self, off: u32, value: u32) -> Result<()> {
        self.check()?;
        self.mmr_write(off, value)
    }

    pub fn fb_read_checked(&mut self, off: u32) -> Result<u32> {
        self.check()?;
        self.fb_read(off)
    }

    pub fn fb_write_checked(&mut self, off: u32, value: u32) -> Result<()> {
        self.check()?;
        self.fb_write(off, value)
    }

    /// Checked read-modify-write: `reg = f(reg)`.
    pub fn mmr_modify_checked(&mut self, off: u32, f: impl FnOnce(u32) -> u32) -> Result<()> {
        let v = self.mmr_read_checked(off)?;
        self.mmr_write_checked(off, f(v))
    }

    /// Select SDRAM window `n` (1 MB) in RBM_SWAP and read the register back,
    /// as the vendor does around every FB access during the firmware load.
    pub fn select_window(&mut self, n: u32) -> Result<()> {
        let v = self.mmr_read_checked(RBM_SWAP)?;
        self.mmr_write_checked(RBM_SWAP, (v & 0xFFFF_80FF) | ((n & 0x7F) << 8))?;
        self.mmr_read_checked(RBM_SWAP)?;
        Ok(())
    }
}

#[cfg(test)]
pub(crate) mod testutil {
    use super::*;
    use crate::bus::mock::{MockPci, Op, Space};

    pub const MMR: u32 = 0x2000_0000;
    pub const FB: u32 = 0x1000_0000;

    pub fn no_sleep(_: Duration) {}

    /// A chip on a mock bus whose board-ID register reads `board_id`.
    pub fn chip(board_id: u32) -> Chip<MockPci> {
        let m = MockPci::new();
        m.set(Space::Mem, MMR + BOARD_ID, board_id);
        Chip::new(m, MMR, FB, board_id, no_sleep)
    }

    /// An MMR register access in the op log, relative to BAR1, or an FB one
    /// relative to BAR0, with the board-ID checks left out.
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum Acc {
        R(u32),
        W(u32, u32),
        FbR(u32),
        FbW(u32, u32),
        Bulk(u8, usize),
        Csr(u16, u32),
    }

    pub fn accesses(ops: &[Op]) -> Vec<Acc> {
        ops.iter()
            .filter_map(|o| match *o {
                Op::MemRead(a) if a == MMR + BOARD_ID => None,
                Op::MemRead(a) if a >= MMR => Some(Acc::R(a - MMR)),
                Op::MemRead(a) => Some(Acc::FbR(a - FB)),
                Op::MemWrite(a, v) if a >= MMR => Some(Acc::W(a - MMR, v)),
                Op::MemWrite(a, v) => Some(Acc::FbW(a - FB, v)),
                Op::BulkOut(e, n) => Some(Acc::Bulk(e, n)),
                Op::CsrWrite(r, v) => Some(Acc::Csr(r, v)),
                _ => None,
            })
            .collect()
    }

    /// Only the writes (MMR and FB), without reads and checks.
    pub fn writes(ops: &[Op]) -> Vec<Acc> {
        accesses(ops).into_iter().filter(|a| matches!(a, Acc::W(..) | Acc::FbW(..))).collect()
    }

    /// Number of board-ID checks in the op log.
    pub fn checks(ops: &[Op]) -> usize {
        ops.iter().filter(|o| **o == Op::MemRead(MMR + BOARD_ID)).count()
    }
}

#[cfg(test)]
mod tests {
    use super::testutil::*;
    use crate::bus::mock::{Op, Space};

    #[test]
    fn checked_access_reads_board_id_first() {
        let mut c = chip(0x0300_00B2);
        c.mmr_write_checked(0x3020, 8).unwrap();
        c.fb_read_checked(0x10).unwrap();
        c.mmr_read(0x30E0).unwrap();
        assert_eq!(
            c.bus().ops(),
            vec![
                Op::MemRead(MMR + 0x478),
                Op::MemWrite(MMR + 0x3020, 8),
                Op::MemRead(MMR + 0x478),
                Op::MemRead(FB + 0x10),
                Op::MemRead(MMR + 0x30E0),
            ]
        );
    }

    #[test]
    fn checked_access_masks_bits_26_27_and_retries() {
        let mut c = chip(0x0300_00B2);
        c.bus().queue(Space::Mem, MMR + 0x478, &[0xFFFF_FFFF, 0, 0x0F00_00B2]);
        c.mmr_read_checked(0x414).unwrap();
        assert_eq!(checks(&c.bus().ops()), 3);
    }

    #[test]
    fn six_mismatches_mean_removed() {
        let mut c = chip(0x0300_00B2);
        c.bus().set(Space::Mem, MMR + 0x478, 0xFFFF_FFFF);
        assert!(c.mmr_write_checked(0x3020, 1).is_err());
        assert!(c.is_removed());
        assert_eq!(checks(&c.bus().ops()), 6);
        assert!(c.bus().writes().is_empty());
        // Further checked accesses fail without touching the bus.
        c.bus().clear();
        assert!(c.mmr_read_checked(0).is_err());
        assert!(c.bus().ops().is_empty());
    }

    #[test]
    fn window_select() {
        let mut c = chip(0x0300_00B2);
        c.bus().set(Space::Mem, MMR + 0x43C, 0x0002_7F55);
        c.select_window(2).unwrap();
        assert_eq!(accesses(&c.bus().ops()), vec![Acc::R(0x43C), Acc::W(0x43C, 0x0002_0255), Acc::R(0x43C)]);
        assert_eq!(checks(&c.bus().ops()), 3);
    }
}
