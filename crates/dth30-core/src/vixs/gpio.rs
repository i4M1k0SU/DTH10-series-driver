//! GPIO set-up and the LED / front-end power bits of the BUFFALO boards
//! (docs/26 2-vixs-bringup.md §2.3, 3-ioctl-frontend.md §4.3).
//!
//! Only boards 0xB4 and 0xC6 have the LED: bit 16 / bit 17 drive a two-colour
//! LED, bit 2 (0xB4) or bit 4 (0xC6) is set while the tuner is not in use.
//! Board 0xBB uses bit 2 to select one of its two front ends instead.
//!
//! Checked on a DT-H50/U2 (board 0xB4); the other boards are unverified.

use super::mmr::{BOARD_ID, Chip};
use crate::Result;
use crate::bus::RemotePci;

/// Output data, direction and function/input registers.
pub const DATA: u32 = 0x1088;
pub const DIR: u32 = 0x108C;
pub const FUNC: u32 = 0x1090;
/// PCI AD-to-GPIO remapping (only for board IDs with bit 27 set).
pub const REMAP: [u32; 2] = [0x498, 0x49C];

/// Application states, as the vendor's "app status" bits.
pub mod app_status {
    pub const RECORDING: u32 = 0x0100_0000;
    pub const PLAYING: u32 = 0x0000_0100;
    pub const WATCHING: u32 = 0x0001_0000;
    pub const NOT_USING: u32 = 0x0000_0001;
}

/// LED bits.
pub const LED_ON_AIR: u32 = 0x1_0000;
pub const LED_IDLE: u32 = 0x2_0000;

/// GPIO direction bits and initial data of a board.
pub fn board_gpio(board: u8) -> (u32, u32) {
    match board {
        0xB4 => (0x3_0005, 0x2_0005),
        0xC6 => (0x3_0013, 0x2_0007),
        _ => (0x1, 0x1),
    }
}

/// The "in use" bit of a board with an LED.
pub fn idle_bit(board: u8) -> Option<u32> {
    match board {
        0xB4 => Some(0x4),
        0xC6 => Some(0x10),
        _ => None,
    }
}

/// The vendor's GPIO initialisation.
pub fn init<B: RemotePci>(chip: &mut Chip<B>) -> Result<()> {
    if chip.mmr_read_checked(BOARD_ID)? & 0x0800_0000 != 0 {
        for r in REMAP {
            chip.mmr_write_checked(r, 0xFFFF_FFFF)?;
        }
    }
    let v = chip.mmr_read_checked(FUNC)?;
    chip.mmr_write(FUNC, v | 0x2_0000)?;
    chip.mmr_read_checked(FUNC)?;
    let (dir, data) = board_gpio(chip.board());
    let v = chip.mmr_read_checked(DIR)?;
    chip.mmr_write(DIR, v | dir)?;
    chip.mmr_write(DATA, data)
}

/// Set (`set`) or clear the bits `mask` of the output data register.
pub fn write_bits<B: RemotePci>(chip: &Chip<B>, mask: u32, set: bool) -> Result<()> {
    let v = chip.mmr_read(DATA)?;
    chip.mmr_write(DATA, (v & !mask) | if set { mask } else { 0 })
}

/// Update the LED and "in use" bits for an application state (a combination
/// of [`app_status`] bits; the highest-priority one wins). Boards without
/// the LED are left alone.
pub fn update_leds<B: RemotePci>(chip: &Chip<B>, status: u32) -> Result<()> {
    let Some(idle) = idle_bit(chip.board()) else { return Ok(()) };
    let busy = status & (app_status::RECORDING | app_status::PLAYING | app_status::WATCHING) != 0;
    if busy {
        write_bits(chip, idle, false)?;
        write_bits(chip, LED_ON_AIR, true)?;
        write_bits(chip, LED_IDLE, false)
    } else if status & app_status::NOT_USING != 0 {
        write_bits(chip, idle, true)?;
        write_bits(chip, LED_ON_AIR, false)?;
        write_bits(chip, LED_IDLE, true)
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::super::mmr::testutil::*;
    use super::*;
    use crate::bus::mock::Space;

    #[test]
    fn init_per_board() {
        for (board, dir, data) in [(0xB4u32, 0x3_0005, 0x2_0005), (0xC6, 0x3_0013, 0x2_0007), (0xB2, 1, 1)] {
            let mut c = chip(0x0300_0000 | board);
            c.bus().set(Space::Mem, MMR + DIR, 0x100);
            init(&mut c).unwrap();
            use Acc::*;
            assert_eq!(
                accesses(&c.bus().ops()),
                vec![R(0x1090), W(0x1090, 0x2_0000), R(0x1090), R(0x108C), W(0x108C, 0x100 | dir), W(0x1088, data)]
            );
            // Two board-ID reads for the bit-27 test (check + value), then
            // the three checked reads.
            assert_eq!(checks(&c.bus().ops()), 5);
        }
    }

    #[test]
    fn remap_for_bit27_boards() {
        let mut c = chip(0x0300_00B2);
        // Masked compare still passes with bit 27 set.
        c.bus().set(Space::Mem, MMR + BOARD_ID, 0x0B00_00B2);
        init(&mut c).unwrap();
        let w = writes(&c.bus().ops());
        assert_eq!(&w[..2], &[Acc::W(0x498, 0xFFFF_FFFF), Acc::W(0x49C, 0xFFFF_FFFF)]);
    }

    #[test]
    fn leds() {
        let c = chip(0x0300_00C6);
        c.bus().set(Space::Mem, MMR + DATA, 0x2_0007);
        update_leds(&c, app_status::WATCHING).unwrap();
        assert_eq!(
            writes(&c.bus().ops()),
            vec![Acc::W(DATA, 0x2_0007), Acc::W(DATA, 0x3_0007), Acc::W(DATA, 0x1_0007)]
        );
        c.bus().clear();
        update_leds(&c, app_status::NOT_USING).unwrap();
        assert_eq!(writes(&c.bus().ops()), vec![Acc::W(DATA, 0x1_0017), Acc::W(DATA, 0x17), Acc::W(DATA, 0x2_0017)]);
        c.bus().clear();
        update_leds(&c, 0).unwrap();
        assert!(c.bus().ops().is_empty());

        let c = chip(0x0300_00B4);
        update_leds(&c, app_status::RECORDING | app_status::NOT_USING).unwrap();
        assert_eq!(writes(&c.bus().ops()), vec![Acc::W(DATA, 0), Acc::W(DATA, 0x1_0000), Acc::W(DATA, 0x1_0000)]);
        let c = chip(0x0300_00B2);
        update_leds(&c, app_status::RECORDING).unwrap();
        assert!(c.bus().ops().is_empty());
    }
}
