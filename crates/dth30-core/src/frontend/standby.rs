//! The standby sequence the vendor kernel driver writes once the chip is up
//! (docs/26 3-ioctl-frontend §4.2): every front end on the board is put
//! into standby with its TS output stopped. Without the kernel driver the
//! host has to do this itself before the first [`super::Frontend::init`].
//!
//! The kernel writes these at 100 kHz, without retries, each only when the
//! one before succeeded. Board 0xB2 is written as MJ102 here as in the
//! kernel, even though its kit may be an MJ110 at other addresses
//! ([`crate::model::kit_for_board`]); those writes then fail harmlessly,
//! which is why [`run_standby`] stops at the first failure. Unverified on
//! hardware.

use crate::Result;
use crate::i2c::I2c;
use crate::model::BoardClass;

/// One step of a standby sequence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StandbyStep {
    /// One I2C write transfer: address, bytes (register first).
    Write(u8, &'static [u8]),
    /// Dual TOSHIBA: GPIO output 0x1088 bit 2 (mask 0x4), which selects
    /// the front end on the shared bus.
    GpioBit2(bool),
}

use StandbyStep::{GpioBit2, Write};

const TOSHIBA: &[StandbyStep] = &[Write(0xC0, &[0x09, 0xE0]), Write(0x20, &[0x03, 0x10]), Write(0x20, &[0x04, 0x20])];

const DUAL_TOSHIBA: &[StandbyStep] = &[
    GpioBit2(false),
    Write(0xC0, &[0x09, 0xE0]),
    Write(0x20, &[0x03, 0x10]),
    Write(0x20, &[0x04, 0x20]),
    GpioBit2(true),
    Write(0xC0, &[0x09, 0xE0]),
    Write(0x20, &[0x03, 0x10]),
    Write(0x20, &[0x04, 0x20]),
];

const MJ100: &[StandbyStep] = &[
    Write(0x20, &[0x1D, 0xAA]),
    Write(0x20, &[0x1E, 0xAA]),
    Write(0x20, &[0x1F, 0xAA]),
    Write(0x20, &[0x1C, 0xAA]),
    Write(0x20, &[0x04, 0x20]),
    Write(0x20, &[0x03, 0x70]),
    Write(0xC0, &[0x95, 0xFB, 0xFE]),
    Write(0xC0, &[0x09, 0x41, 0xFE]),
];

const MJ102: &[StandbyStep] = &[
    Write(0x20, &[0xFE, 0xC0, 0x05, 0x00]),
    Write(0x20, &[0x1D, 0xAA]),
    Write(0x20, &[0x1E, 0xAA]),
    Write(0x20, &[0x1F, 0xAA]),
    Write(0x20, &[0x1C, 0xAA]),
    Write(0x20, &[0x04, 0x20]),
    Write(0x20, &[0x03, 0x70]),
];

const MJ110_DUAL: &[StandbyStep] = &[
    Write(0x30, &[0xFE, 0xC6, 0x05, 0x00]),
    Write(0x30, &[0x1E, 0xA8]),
    Write(0x30, &[0x1F, 0xA8]),
    Write(0x30, &[0x1C, 0xAA]),
    Write(0x30, &[0x03, 0xF0]),
    // tuner 1: RF loop-through on
    Write(0x30, &[0xFE, 0xC6, 0x10, 0x40]),
    Write(0x3C, &[0xFE, 0xC0, 0x05, 0x00]),
    Write(0x3C, &[0x1E, 0xA8]),
    Write(0x3C, &[0x1F, 0xA8]),
    Write(0x3C, &[0x1C, 0xAA]),
    Write(0x3C, &[0x03, 0xF0]),
];

/// The kernel's start-up standby sequence for a board class.
pub fn standby_steps(class: BoardClass) -> &'static [StandbyStep] {
    match class {
        BoardClass::Toshiba => TOSHIBA,
        BoardClass::DualToshiba => DUAL_TOSHIBA,
        BoardClass::SonyMj100 => MJ100,
        BoardClass::SonyMj102 => MJ102,
        BoardClass::SonyMj110Dual => MJ110_DUAL,
    }
}

/// Run a standby sequence, stopping at the first failure. `gpio_bit2` sets
/// GPIO output bit 2 (only the dual TOSHIBA sequence uses it).
pub fn run_standby<B: I2c + ?Sized>(
    bus: &mut B,
    steps: &[StandbyStep],
    mut gpio_bit2: impl FnMut(bool) -> Result<()>,
) -> Result<()> {
    for step in steps {
        match *step {
            Write(addr, data) => bus.write(addr, data)?,
            GpioBit2(v) => gpio_bit2(v)?,
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::super::testutil::{Ev, Rig, Seq};
    use super::*;
    use crate::model::board_class;

    #[test]
    fn toshiba_board() {
        let mut rig = Rig::new();
        run_standby(&mut rig, standby_steps(board_class(0xB4)), |_| panic!("no GPIO")).unwrap();
        assert_eq!(rig.ev, Seq::new().w(0xC0, 0x09, 0xE0).w(0x20, 0x03, 0x10).w(0x20, 0x04, 0x20).0);
    }

    #[test]
    fn dual_toshiba_switches_the_bus() {
        let mut rig = Rig::new();
        let mut gpio = Vec::new();
        run_standby(&mut rig, standby_steps(BoardClass::DualToshiba), |v| {
            gpio.push(v);
            Ok(())
        })
        .unwrap();
        assert_eq!(gpio, [false, true]);
        assert_eq!(rig.ev.len(), 6);
    }

    #[test]
    fn mj100_and_mj110() {
        let mut rig = Rig::new();
        run_standby(&mut rig, standby_steps(board_class(0xB3)), |_| Ok(())).unwrap();
        assert_eq!(rig.ev[6], Ev::W(0xC0, vec![0x95, 0xFB, 0xFE]));
        assert_eq!(rig.ev[7], Ev::W(0xC0, vec![0x09, 0x41, 0xFE]));

        let mut rig = Rig::new();
        run_standby(&mut rig, standby_steps(board_class(0xC6)), |_| Ok(())).unwrap();
        assert_eq!(rig.ev[0], Ev::W(0x30, vec![0xFE, 0xC6, 0x05, 0x00]));
        assert_eq!(rig.ev[5], Ev::W(0x30, vec![0xFE, 0xC6, 0x10, 0x40]));
        assert_eq!(rig.ev[6], Ev::W(0x3C, vec![0xFE, 0xC0, 0x05, 0x00]));
        assert_eq!(rig.ev.len(), 11);
    }

    #[test]
    fn stops_at_the_first_failure_without_retrying() {
        let mut rig = Rig::new();
        rig.fail_always(0x20);
        assert!(run_standby(&mut rig, standby_steps(board_class(0xB2)), |_| Ok(())).is_err());
        assert_eq!(rig.ev, vec![Ev::W(0x20, vec![0xFE, 0xC0, 0x05, 0x00])]);
    }
}
