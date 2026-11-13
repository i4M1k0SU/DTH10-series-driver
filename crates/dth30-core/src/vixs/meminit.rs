//! SDRAM controller and PLL initialisation (the vendor's separate memory-init
//! module, docs/26 2-vixs-bringup.md §3).
//!
//! The vendor selects a routine and parameters from a board table by the full
//! board-ID register value; all BUFFALO boards use the same routine, which is
//! only a list of register writes, DDR mode-register commands and a memory
//! test through registers. It uses plain (unchecked) register accesses.
//!
//! Not verified on hardware.

use core::time::Duration;

use super::mmr::Chip;
use crate::bus::RemotePci;
use crate::{Error, Result};

/// The vendor's default Power_Mode.
pub const DEFAULT_POWER_MODE: u32 = 6;
/// Core clock the memory init reports for these boards (kHz).
pub const CLOCK_KHZ: u32 = 182_250;
/// The firmware variant the driver carries; any other result is refused.
pub const FW_VARIANT: u32 = 4;

/// How often a DDR command-done bit is polled before giving up (the vendor
/// polls without bound).
const POLL_LIMIT: u32 = 10_000;

/// One row of the vendor's board table (the BUFFALO rows only).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoardRow {
    /// Full board-ID register value.
    pub board_id: u32,
    /// Power_Mode adjustment: below 7 the mode is raised by 5 (1 counts as 2).
    pub power_adj: bool,
    /// Memory size (bytes).
    pub mem_size: u32,
    /// Halve the memory size in Power_Mode 10/11.
    pub half_in_low_power: bool,
    /// Alternative PLL setting (register 0x58 bit 25).
    pub alt_pll: bool,
    /// Firmware variant the board needs.
    pub fw_variant: u32,
}

const fn row(board_id: u32, power_adj: bool, alt_pll: bool) -> BoardRow {
    BoardRow { board_id, power_adj, mem_size: 0x0400_0000, half_in_low_power: true, alt_pll, fw_variant: 4 }
}

/// The BUFFALO rows of the vendor's board table.
pub const BOARDS: &[BoardRow] = &[
    row(0x0300_00B2, false, false),
    row(0x0300_00B3, false, false),
    row(0x0300_00B4, false, false),
    row(0x0300_00B5, false, false),
    row(0x0300_00B6, false, false),
    row(0x0300_00B8, false, false),
    row(0x0300_00B9, false, false),
    row(0x0300_00BA, true, false),
    row(0x0300_00BB, false, false),
    row(0x0300_00BC, false, true),
    row(0x0300_00C6, false, false),
];

/// The table row of a board-ID register value.
pub fn board_row(board_id: u32) -> Option<&'static BoardRow> {
    BOARDS.iter().find(|r| r.board_id == board_id)
}

/// What the memory init produced (the values the firmware loader needs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MemInit {
    /// Power_Mode after the board's adjustment (written to FB 0x40).
    pub power_mode: u32,
    /// Core clock in kHz (FB 0x44).
    pub clock_khz: u32,
    /// SDRAM size in bytes (FB 0x48).
    pub mem_size: u32,
}

/// The DDR mode-register commands: (data 0x824, command 0x820).
const DDR_COMMANDS: [(u32, u32); 10] = [
    (0x0001_0000, 0x1171),
    (0x98EF_0000, 0x11F1),
    (0x0001_FFFF, 0x10A1),
    (0x0003_0000, 0x1081),
    (0x0003_0132, 0x0081),
    (0x0003_FFFF, 0x00A1),
    (0x0003_0000, 0x0091),
    (0x000E_0000, 0x0091),
    (0x000E_0032, 0x0081),
    (0x00C8_0000, 0x01F3),
];

/// Run the memory init for `board_id` (the full register value) with
/// `power_mode` (normally [`DEFAULT_POWER_MODE`]).
pub fn mem_init<B: RemotePci>(chip: &Chip<B>, board_id: u32, power_mode: u32) -> Result<MemInit> {
    let row = board_row(board_id)
        .ok_or_else(|| Error::Unsupported(format!("board ID {board_id:08x} is not a known BUFFALO board")))?;
    let mut power_mode = power_mode;
    if row.power_adj && power_mode < 7 {
        if power_mode == 1 {
            power_mode = 2;
        }
        power_mode += 5;
    }
    let low_power = matches!(power_mode, 10 | 11);
    let mem_size = if low_power { row.mem_size >> u32::from(row.half_in_low_power) } else { row.mem_size };

    let d10 = || chip.wait_ticks(1);
    let d2 = || chip.sleep(Duration::from_millis(2));
    let w = |off, v| chip.mmr_write(off, v);
    let rmw = |off, f: &dyn Fn(u32) -> u32| chip.mmr_modify(off, f);

    rmw(0x43C, &|t| t & 0xFFFD_FFFF)?;
    w(0x418, 0)?;
    w(0x420, 0)?;
    w(0x430, 0)?;
    w(0x000, 0xFFFF_FDFF)?;
    w(0x058, if row.alt_pll { 0x1300_0000 } else { 0x1100_0000 })?;
    for r in [0x010, 0x018, 0x030] {
        rmw(r, &|t| t | 0x00C0_0000)?;
    }
    w(0x05C, 0xF8FF_FF79)?;
    w(0x070, 0x0101_03FE)?;
    w(0x070, 0x0101_0002)?;
    for (r, v) in [(0x010, 0x3A02), (0x018, 0x3602), (0x030, 0x1800)] {
        rmw(r, &|t| (t & 0xFFFF_C0FD) | v)?;
    }
    for r in [0x010, 0x018, 0x030] {
        rmw(r, &|t| t & 0xFF7F_FFFF)?;
    }
    for r in [0x010, 0x018, 0x030] {
        rmw(r, &|t| t & 0xFFBF_FFFF)?;
    }
    d10();
    rmw(0x070, &|t| t & 0xFEFF_FFFF)?;
    w(0x050, 0x0F00_6704)?;
    w(0x054, 0xF6)?;
    rmw(0x058, &|t| t | 0x200)?;
    w(0x05C, 0x0F00_0000)?;
    w(0x060, 0)?;
    w(0x008, 0xFFFF_FFFF)?;
    w(0x068, 0xFFFF_FFFF)?;
    w(0x000, 0)?;
    rmw(0x890, &|t| t | 4)?;
    rmw(0x890, &|t| t & 0xFFFF_FFFD)?;
    rmw(0x860, &|t| (t & 0xFFFF_0FFF) | 0x1000)?;
    rmw(0x864, &|t| (t & 0xFFFF_0FFF) | 0x1000)?;
    w(0x85C, 0x0011_0000)?;
    rmw(0x860, &|t| t | 0x40)?;
    rmw(0x864, &|t| t | 0x40)?;
    rmw(0x860, &|t| t | 2)?;
    rmw(0x860, &|t| t & 0xFFFF_FFFE)?;
    if low_power {
        rmw(0x864, &|t| t | 1)?;
    } else {
        rmw(0x864, &|t| t | 2)?;
        rmw(0x864, &|t| t & 0xFFFF_FFFE)?;
    }
    rmw(0x880, &|t| t | 2)?;
    chip.mmr_read(0x880)?;
    d10();
    w(0x800, 0xF)?;
    w(0x800, 0)?;
    w(0x810, if low_power { 0x12EA_0936 } else { 0x02EA_0936 })?;
    w(0x814, 0x05F7_0B38)?;
    w(0x818, 0x3223_230E)?;
    w(0x81C, 0x31)?;
    w(0x808, 0x0E0F_00C8)?;
    w(0x80C, 0x024C_00C4)?;
    for (data, cmd) in DDR_COMMANDS {
        w(0x824, data)?;
        w(0x820, cmd)?;
        let mut n = 0;
        while chip.mmr_read(0x828)? & 1 == 0 {
            n += 1;
            if n >= POLL_LIMIT {
                return Err(Error::Timeout);
            }
        }
    }
    w(0x850, 0)?;
    w(0x840, 0xFFFF_FFFF)?;
    w(0x844, 0xFFFF_FFFF)?;
    w(0x8A0, 0x4444_4444)?;
    w(0x8A4, 0x4444_4444)?;
    w(0x848, 0)?;
    w(0x84C, 0xFFD)?;
    w(0x894, 0x8080_0880)?;
    w(0x898, 0x8080_8080)?;
    w(0x89C, 0x8080_8080)?;
    let strap = chip.mmr_read(0x47C)?;
    w(0x400, if strap != 0 { 0xFDC } else { 0x390 })?;
    rmw(0x058, &|t| t | 0xEFF)?;
    if low_power {
        w(0x810, 0xD2EA_0936)?;
        d10();
        w(0x810, 0x12EA_0936)?;
    } else {
        w(0x810, 0xC2EA_0936)?;
        d10();
        w(0x810, 0x02EA_0936)?;
    }
    d10();
    rmw(0x43C, &|t| t | 0x0002_0000)?;
    rmw(0x43C, &|t| t & 0xFFFF_80FF)?; // FB window 0

    // Memory test through the register interface.
    for (off, v) in [
        (0x874, 0x1234_5678),
        (0x878, 0x1F00),
        (0x870, 4),
        (0x874, 0x8765_4321),
        (0x878, 0x1F00),
        (0x870, 0x14),
        (0x870, 0),
        (0x878, 0),
    ] {
        w(off, v)?;
        d2();
    }
    let first = chip.mmr_read(0x874)?;
    d2();
    if first != 0x1234_5678 {
        return Err(Error::Protocol("ViXS SDRAM test failed (first word)"));
    }
    w(0x870, 0x10)?;
    d2();
    w(0x878, 0)?;
    d2();
    let second = chip.mmr_read(0x874)?;
    d2();
    if second != 0x8765_4321 {
        return Err(Error::Protocol("ViXS SDRAM test failed (second word)"));
    }

    // The vendor's check of the firmware variant against the memory size.
    let fw_variant = match mem_size {
        0x0200_0000 if !matches!(board_id, 0x0300_00BA | 0x0300_00B7) => 5,
        0x0100_0000 if board_id != 0x0300_00BE => 6,
        _ => row.fw_variant,
    };
    if fw_variant != FW_VARIANT {
        return Err(Error::Unsupported(format!(
            "board {board_id:08x} in Power_Mode {power_mode} needs firmware variant {fw_variant}"
        )));
    }
    Ok(MemInit { power_mode, clock_khz: CLOCK_KHZ, mem_size })
}

#[cfg(test)]
mod tests {
    use super::super::mmr::testutil::*;
    use super::*;
    use crate::bus::mock::{MockPci, Space};

    fn ready(c: &Chip<MockPci>) {
        c.bus().set(Space::Mem, MMR + 0x828, 1);
        c.bus().queue(Space::Mem, MMR + 0x874, &[0x1234_5678, 0x8765_4321]);
    }

    /// The expected writes of the default (Power_Mode 6) path with all
    /// registers reading 0 before the sequence and `strap` at 0x47C.
    fn expected_mode6(r58: u32, strap: u32) -> Vec<Acc> {
        use Acc::W;
        let mut v = vec![
            W(0x43C, 0),
            W(0x418, 0),
            W(0x420, 0),
            W(0x430, 0),
            W(0x000, 0xFFFF_FDFF),
            W(0x058, r58),
            W(0x010, 0x00C0_0000),
            W(0x018, 0x00C0_0000),
            W(0x030, 0x00C0_0000),
            W(0x05C, 0xF8FF_FF79),
            W(0x070, 0x0101_03FE),
            W(0x070, 0x0101_0002),
            W(0x010, 0x00C0_3A02),
            W(0x018, 0x00C0_3602),
            W(0x030, 0x00C0_1800),
            W(0x010, 0x0040_3A02),
            W(0x018, 0x0040_3602),
            W(0x030, 0x0040_1800),
            W(0x010, 0x3A02),
            W(0x018, 0x3602),
            W(0x030, 0x1800),
            W(0x070, 0x0001_0002),
            W(0x050, 0x0F00_6704),
            W(0x054, 0xF6),
            W(0x058, r58 | 0x200),
            W(0x05C, 0x0F00_0000),
            W(0x060, 0),
            W(0x008, 0xFFFF_FFFF),
            W(0x068, 0xFFFF_FFFF),
            W(0x000, 0),
            W(0x890, 4),
            W(0x890, 4),
            W(0x860, 0x1000),
            W(0x864, 0x1000),
            W(0x85C, 0x0011_0000),
            W(0x860, 0x1040),
            W(0x864, 0x1040),
            W(0x860, 0x1042),
            W(0x860, 0x1042),
            W(0x864, 0x1042),
            W(0x864, 0x1042),
            W(0x880, 2),
            W(0x800, 0xF),
            W(0x800, 0),
            W(0x810, 0x02EA_0936),
            W(0x814, 0x05F7_0B38),
            W(0x818, 0x3223_230E),
            W(0x81C, 0x31),
            W(0x808, 0x0E0F_00C8),
            W(0x80C, 0x024C_00C4),
        ];
        for (d, c) in DDR_COMMANDS {
            v.push(W(0x824, d));
            v.push(W(0x820, c));
        }
        v.extend([
            W(0x850, 0),
            W(0x840, 0xFFFF_FFFF),
            W(0x844, 0xFFFF_FFFF),
            W(0x8A0, 0x4444_4444),
            W(0x8A4, 0x4444_4444),
            W(0x848, 0),
            W(0x84C, 0xFFD),
            W(0x894, 0x8080_0880),
            W(0x898, 0x8080_8080),
            W(0x89C, 0x8080_8080),
            W(0x400, if strap != 0 { 0xFDC } else { 0x390 }),
            W(0x058, r58 | 0xEFF),
            W(0x810, 0xC2EA_0936),
            W(0x810, 0x02EA_0936),
            W(0x43C, 0x0002_0000),
            W(0x43C, 0x0002_0000),
            W(0x874, 0x1234_5678),
            W(0x878, 0x1F00),
            W(0x870, 4),
            W(0x874, 0x8765_4321),
            W(0x878, 0x1F00),
            W(0x870, 0x14),
            W(0x870, 0),
            W(0x878, 0),
            W(0x870, 0x10),
            W(0x878, 0),
        ]);
        v
    }

    #[test]
    fn default_board_write_list() {
        let c = chip(0x0300_00B2);
        ready(&c);
        let m = mem_init(&c, 0x0300_00B2, DEFAULT_POWER_MODE).unwrap();
        assert_eq!(m, MemInit { power_mode: 6, clock_khz: 182_250, mem_size: 0x0400_0000 });
        assert_eq!(writes(&c.bus().ops()), expected_mode6(0x1100_0000, 0));
        // Plain accesses only: no board-ID checks.
        assert_eq!(checks(&c.bus().ops()), 0);
    }

    #[test]
    fn alt_pll_board_and_strap() {
        let c = chip(0x0300_00BC);
        ready(&c);
        c.bus().set(Space::Mem, MMR + 0x47C, 1);
        mem_init(&c, 0x0300_00BC, DEFAULT_POWER_MODE).unwrap();
        assert_eq!(writes(&c.bus().ops()), expected_mode6(0x1300_0000, 1));
    }

    #[test]
    fn ba_board_low_power_branch() {
        let c = chip(0x0300_00BA);
        ready(&c);
        let m = mem_init(&c, 0x0300_00BA, DEFAULT_POWER_MODE).unwrap();
        assert_eq!(m.power_mode, 11);
        assert_eq!(m.mem_size, 0x0200_0000);
        let mut want = expected_mode6(0x1100_0000, 0);
        // 0x864 gets only |1, and 0x810 uses the 0x1xxxxxxx values.
        let i = want.iter().position(|a| *a == Acc::W(0x864, 0x1042)).unwrap();
        want.splice(i..i + 2, [Acc::W(0x864, 0x1041)]);
        for a in want.iter_mut() {
            match a {
                Acc::W(0x810, v @ 0x02EA_0936) => *v = 0x12EA_0936,
                Acc::W(0x810, v @ 0xC2EA_0936) => *v = 0xD2EA_0936,
                _ => {}
            }
        }
        assert_eq!(writes(&c.bus().ops()), want);
    }

    #[test]
    fn power_mode_adjustment() {
        for (given, got) in [(1, 7), (2, 7), (6, 11), (7, 7), (11, 11)] {
            let c = chip(0x0300_00BA);
            ready(&c);
            assert_eq!(mem_init(&c, 0x0300_00BA, given).unwrap().power_mode, got);
        }
    }

    #[test]
    fn low_power_on_other_boards_needs_another_firmware() {
        let c = chip(0x0300_00B2);
        ready(&c);
        std::assert_matches!(mem_init(&c, 0x0300_00B2, 10), Err(Error::Unsupported(_)));
    }

    #[test]
    fn memory_test_failure_and_unknown_board() {
        let c = chip(0x0300_00B2);
        c.bus().set(Space::Mem, MMR + 0x828, 1);
        c.bus().queue(Space::Mem, MMR + 0x874, &[0x1234_5678, 0]);
        std::assert_matches!(mem_init(&c, 0x0300_00B2, 6), Err(Error::Protocol(_)));
        std::assert_matches!(mem_init(&c, 0x0300_00A2, 6), Err(Error::Unsupported(_)));
    }

    #[test]
    fn ddr_command_poll_waits_for_done_bit() {
        let c = chip(0x0300_00B2);
        ready(&c);
        c.bus().queue(Space::Mem, MMR + 0x828, &[0, 0, 1]);
        mem_init(&c, 0x0300_00B2, 6).unwrap();
        let reads = c.bus().ops().iter().filter(|o| **o == crate::bus::mock::Op::MemRead(MMR + 0x828)).count();
        assert_eq!(reads, 12);
    }
}
