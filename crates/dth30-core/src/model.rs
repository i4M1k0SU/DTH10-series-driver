//! The products of the family and how a board maps to a front-end kit.
//!
//! The USB product ID names the product; which tuner board it carries is
//! only known from the ViXS board-ID register (see [`crate::vixs`]). All of
//! these but the DT-H50/U2 (board 0xB4) are unverified on hardware.

/// BUFFALO's USB vendor ID.
pub const VENDOR_ID: u16 = 0x0411;

/// The products the vendor driver (BufFsUSB 1.4.1.9) binds to, by USB product ID.
pub const PRODUCTS: &[(u16, &str)] =
    &[(0x0128, "DT-H30/U2"), (0x0129, "DT-H50/U2"), (0x012A, "DT-H50/U2W"), (0x0144, "DT-H30/U2")];

/// Whether `vid:pid` is a product of this family.
pub fn is_supported(vid: u16, pid: u16) -> bool {
    vid == VENDOR_ID && PRODUCTS.iter().any(|&(p, _)| p == pid)
}

/// The product name of a USB product ID of this family.
pub fn product_name(pid: u16) -> Option<&'static str> {
    PRODUCTS.iter().find(|&&(p, _)| p == pid).map(|&(_, n)| n)
}

/// The product name, or the ID in hex for an unknown one.
pub fn product_label(pid: u16) -> String {
    match product_name(pid) {
        Some(n) => n.to_owned(),
        None => format!("unknown product {pid:04x}"),
    }
}

/// The board ID: the low 8 bits of ViXS MMR 0x478 (`XC_RBM_BOARD_ID`).
/// Everything else in the register identifies the chip (see [`is_camino`]).
pub fn board_id(mmr_478: u32) -> u8 {
    mmr_478 as u8
}

/// Whether MMR 0x478 names an XCode2 ("Camino") chip.
pub fn is_camino(mmr_478: u32) -> bool {
    mmr_478 & 0xF3FF_0000 == 0x0300_0000
}

/// The demodulator + tuner kit on a board, as the vendor's user-mode filter
/// picks it (docs/26 4-tuning §2.1). All four share a Toshiba TC905xx-family
/// ISDB-T demodulator; see [`crate::frontend`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum KitKind {
    /// SONY SMT-MJ100: demodulator 0x20, register-map tuner 0xC0 written
    /// directly with a trailing 0xFE byte.
    Mj100,
    /// SONY SMT-MJ102: demodulator 0x20, tuner 0xC0 behind the demodulator's
    /// 0xFE repeater.
    Mj102,
    /// SONY SMT-MJ110: demodulator 0x30, tuner 0xC6 behind the repeater.
    Mj110,
    /// TOSHIBA: demodulator 0x20, up-converting double-conversion tuner 0xC0
    /// written directly, IF correction byte in an EEPROM at 0xA0.
    Toshiba,
}

impl KitKind {
    /// 8-bit I2C address of the demodulator.
    pub const fn demod_addr(self) -> u8 {
        match self {
            KitKind::Mj110 => 0x30,
            _ => 0x20,
        }
    }

    /// 8-bit I2C address of the tuner.
    pub const fn tuner_addr(self) -> u8 {
        match self {
            KitKind::Mj110 => 0xC6,
            _ => 0xC0,
        }
    }

    /// A short name for logs.
    pub const fn name(self) -> &'static str {
        match self {
            KitKind::Mj100 => "SONY SMT-MJ100",
            KitKind::Mj102 => "SONY SMT-MJ102",
            KitKind::Mj110 => "SONY SMT-MJ110",
            KitKind::Toshiba => "TOSHIBA",
        }
    }

    /// Whether the demodulator sends 204-byte packets (RS parity kept) to
    /// the ViXS chip. The vendor kernel driver sets the TS input up that way
    /// for TOSHIBA boards and for 188 bytes on SONY ones (inferred, docs/26
    /// 3-ioctl-frontend §3.2).
    pub const fn ts_packet_204(self) -> bool {
        matches!(self, KitKind::Toshiba)
    }
}

/// The subsystem IDs that select the MJ102 kit on board 0xB2.
pub const SUBSYSTEM_MJ102: (u16, u16) = (0xEEEE, 0xEEEE);

/// The kit on a board, from the board ID ([`board_id`]) and the two
/// subsystem ID words the board-info command reports (only board 0xB2 looks
/// at them: [`SUBSYSTEM_MJ102`] selects the MJ102 kit, anything else the
/// MJ110 kit). `None` for a board the vendor's model table does not list.
///
/// This follows the user-mode filter's model table (docs/26 4-tuning §0,
/// §2.1). The kernel driver's own board table (3-ioctl-frontend §3.2, see
/// [`board_class`]) differs in two places, and the user-mode table wins for
/// the kit choice since it is the one that tunes: board 0xB2 without the
/// MJ102 subsystem is an MJ110 kit (demodulator 0x30, tuner 0xC6) although
/// the kernel files it under MJ102, and the kernel's "anything else is
/// TOSHIBA" fallback is not applied.
pub fn kit_for_board(board_id: u8, subsystem: (u16, u16)) -> Option<KitKind> {
    match board_id {
        0xA2 | 0xA3 | 0xB3 | 0xBE => Some(KitKind::Mj100),
        0xB2 if subsystem == SUBSYSTEM_MJ102 => Some(KitKind::Mj102),
        0xB5 | 0xBF | 0xC1 => Some(KitKind::Mj102),
        0xB2 | 0xC6 => Some(KitKind::Mj110),
        0xB4 | 0xB7 | 0xB8 | 0xBA | 0xBB | 0xBC => Some(KitKind::Toshiba),
        _ => None,
    }
}

/// Number of front ends on a board: two on 0xC6 (DT-H50/U2W, two MJ110
/// kits) and 0xBB (dual TOSHIBA), one elsewhere. Only the first one is
/// supported by [`crate::frontend`] (the second one's addressing on 0xC6 is
/// unresolved, docs/26 4-tuning §10.5).
pub fn tuner_count(board_id: u8) -> u8 {
    match board_id {
        0xBB | 0xC6 => 2,
        _ => 1,
    }
}

/// The board classes of the vendor kernel driver (docs/26 3-ioctl-frontend
/// §3.2). They decide the TS packet size and the standby sequence the
/// kernel writes once the chip is up ([`crate::frontend::standby_steps`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BoardClass {
    /// 0xA2, 0xA3, 0xB3, 0xBE.
    SonyMj100,
    /// 0xB2, 0xB5, 0xBF, 0xC1 (but see [`kit_for_board`] for 0xB2).
    SonyMj102,
    /// 0xC6.
    SonyMj110Dual,
    /// 0xBB.
    DualToshiba,
    /// Every other board ID.
    Toshiba,
}

impl BoardClass {
    /// Whether the kernel sets the TS input up for 204-byte packets.
    pub const fn ts_packet_204(self) -> bool {
        matches!(self, BoardClass::Toshiba | BoardClass::DualToshiba)
    }
}

/// The kernel driver's class of a board ID.
pub fn board_class(board_id: u8) -> BoardClass {
    match board_id {
        0xA2 | 0xA3 | 0xB3 | 0xBE => BoardClass::SonyMj100,
        0xB2 | 0xB5 | 0xBF | 0xC1 => BoardClass::SonyMj102,
        0xC6 => BoardClass::SonyMj110Dual,
        0xBB => BoardClass::DualToshiba,
        _ => BoardClass::Toshiba,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn products() {
        assert!(is_supported(0x0411, 0x0129));
        assert!(!is_supported(0x0411, 0x0199));
        assert!(!is_supported(0x0525, 0x0128));
        assert_eq!(product_name(0x0144), Some("DT-H30/U2"));
        assert_eq!(product_label(0x1234), "unknown product 1234");
    }

    #[test]
    fn board_id_register() {
        assert_eq!(board_id(0x0300_00B4), 0xB4);
        assert!(is_camino(0x0300_00B4));
        assert!(is_camino(0x0700_00B4)); // bits 26/27 are masked off
        assert!(!is_camino(0x0100_00B4));
    }

    #[test]
    fn kits_by_board() {
        const ANY: (u16, u16) = (0x1234, 0x5678);
        assert_eq!(kit_for_board(0xB3, ANY), Some(KitKind::Mj100));
        assert_eq!(kit_for_board(0xB2, SUBSYSTEM_MJ102), Some(KitKind::Mj102));
        assert_eq!(kit_for_board(0xB2, ANY), Some(KitKind::Mj110));
        assert_eq!(kit_for_board(0xB2, (0xEEEE, 0)), Some(KitKind::Mj110));
        assert_eq!(kit_for_board(0xB4, ANY), Some(KitKind::Toshiba));
        assert_eq!(kit_for_board(0xB4, SUBSYSTEM_MJ102), Some(KitKind::Toshiba));
        assert_eq!(kit_for_board(0xC6, ANY), Some(KitKind::Mj110));
        for id in [0xA2, 0xA3, 0xBE] {
            assert_eq!(kit_for_board(id, ANY), Some(KitKind::Mj100));
        }
        for id in [0xB5, 0xBF, 0xC1] {
            assert_eq!(kit_for_board(id, ANY), Some(KitKind::Mj102));
        }
        for id in [0xB7, 0xB8, 0xBA, 0xBB, 0xBC] {
            assert_eq!(kit_for_board(id, ANY), Some(KitKind::Toshiba));
        }
        assert_eq!(kit_for_board(0x00, ANY), None);
        assert_eq!(kit_for_board(0xB6, ANY), None);
        assert_eq!(tuner_count(0xC6), 2);
        assert_eq!(tuner_count(0xBB), 2);
        assert_eq!(tuner_count(0xB4), 1);
    }

    #[test]
    fn kit_addresses() {
        assert_eq!((KitKind::Mj100.demod_addr(), KitKind::Mj100.tuner_addr()), (0x20, 0xC0));
        assert_eq!((KitKind::Mj102.demod_addr(), KitKind::Mj102.tuner_addr()), (0x20, 0xC0));
        assert_eq!((KitKind::Mj110.demod_addr(), KitKind::Mj110.tuner_addr()), (0x30, 0xC6));
        assert_eq!((KitKind::Toshiba.demod_addr(), KitKind::Toshiba.tuner_addr()), (0x20, 0xC0));
        assert!(KitKind::Toshiba.ts_packet_204());
        assert!(!KitKind::Mj110.ts_packet_204());
    }

    #[test]
    fn kernel_board_classes() {
        assert_eq!(board_class(0xB3), BoardClass::SonyMj100);
        assert_eq!(board_class(0xB2), BoardClass::SonyMj102);
        assert_eq!(board_class(0xC6), BoardClass::SonyMj110Dual);
        assert_eq!(board_class(0xBB), BoardClass::DualToshiba);
        assert_eq!(board_class(0xB4), BoardClass::Toshiba);
        assert_eq!(board_class(0x00), BoardClass::Toshiba);
        assert!(board_class(0xB4).ts_packet_204());
        assert!(board_class(0xBB).ts_packet_204());
        assert!(!board_class(0xB2).ts_packet_204());
    }
}
