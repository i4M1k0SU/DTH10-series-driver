//! The ViXS chip's I2C master, driven by the host through registers
//! (docs/26 3-ioctl-frontend.md §2).
//!
//! Two channels at MMR 0x1000 (channel 1 at +0x10): TIMING, write FIFO,
//! read FIFO, and a control word that starts a transaction of up to 64 bytes
//! with optional START and STOP. The vendor waits for the completion
//! interrupt; this module polls the same status register (0x1094) and then
//! performs the vendor's interrupt-handler and completion steps in order:
//! acknowledge both channels' status bits, read the control register (bit31 =
//! no acknowledge), run the bus-recovery check, and after a NAK clear the
//! read FIFO and put a STOP on the bus if the transaction had none.
//!
//! Longer transfers are split into 64-byte transactions: START on the first,
//! STOP on the last, a repeated START before the read part of
//! [`I2c::write_read`]. The vendor itself never sends more than 64 bytes, so
//! the continuation transactions (no START) are unverified.
//!
//! Not verified on hardware.

use core::time::Duration;

use super::mmr::{Chip, INT_ENABLE, TIMER};
use crate::bus::RemotePci;
use crate::i2c::I2c;
use crate::{Error, Result};

/// Channel 0 registers; channel 1 is at +[`CHANNEL_STRIDE`].
pub const TIMING: u32 = 0x1000;
pub const WDATA: u32 = 0x1004;
pub const RDATA: u32 = 0x1008;
pub const CTRL: u32 = 0x100C;
pub const CHANNEL_STRIDE: u32 = 0x10;
/// Interrupt status (write 1 to clear) and enable of the I2C/GPIO block.
pub const STATUS: u32 = 0x1094;
pub const IRQ_ENABLE: u32 = 0x1098;
/// GPIO registers the bus recovery uses (function, data, direction, input).
pub const GPIO_FUNC: u32 = 0x1084;
pub const GPIO_DATA: u32 = 0x1088;
pub const GPIO_DIR: u32 = 0x108C;
pub const GPIO_IN: u32 = 0x1090;
/// Prescaler / control and reset registers of the block.
pub const PRESCALE: u32 = 0x10A0;
pub const RESET: u32 = 0x10A4;
pub const GLOBAL_DIV: u32 = 0x10A8;
/// Bus-recovery counter.
pub const RECOVERY_COUNT: u32 = 0x10FC;

/// Per-channel completion bits in [`STATUS`] (the vendor's masks; channel
/// 0's includes bit 7 and the error bits).
pub const DONE_MASK: [u32; 2] = [0x0030_00C0, 0x0000_0080];
/// Error bits in [`STATUS`].
pub const HW_ERROR: u32 = 0x0030_0000;
/// Control-register bit set after a failed command (no acknowledge).
pub const CMD_ERROR: u32 = 0x8000_0000;
/// Control word that only puts a STOP on the bus.
pub const STOP_ONLY: u32 = 0x1200;
/// Largest transaction.
pub const MAX_LEN: usize = 64;
/// Upper 16 bits of TIMING used by all the vendor's internal calls.
pub const DEFAULT_PARAM: u32 = 100;
/// The clock the divider is computed from (kHz).
pub const CORE_KHZ: u32 = 166_000;

/// How often the status register is polled for completion, and the pause
/// between polls after the first few.
const POLL_LIMIT: u32 = 200;
const POLL_FAST: u32 = 4;
const POLL_INTERVAL: Duration = Duration::from_millis(1);
/// Bound of the free-running-timer waits (the vendor's are unbounded).
const TIMER_READ_LIMIT: u32 = 10_000;

/// The TIMING divider for a bus clock in kHz (100 kHz -> 415, 400 kHz -> 103).
pub fn clkdiv(khz: u32) -> u32 {
    CORE_KHZ / (4 * khz.max(1))
}

/// The control word: `len | start<<8 | stop<<9 | dir<<12 | (addr8 & 0xFE)<<15`
/// with dir 1 = write, 0 = read.
pub fn ctrl_word(len: usize, start: bool, stop: bool, write: bool, addr8: u8) -> u32 {
    debug_assert!(len <= MAX_LEN);
    len as u32 | u32::from(start) << 8 | u32::from(stop) << 9 | u32::from(write) << 12 | u32::from(addr8 & 0xFE) << 15
}

fn reg(ch: u8, r: u32) -> u32 {
    r + CHANNEL_STRIDE * u32::from(ch)
}

/// The vendor's I2C block initialisation (3-ioctl-frontend.md §2.5).
pub fn init<B: RemotePci>(chip: &mut Chip<B>) -> Result<()> {
    let v = chip.mmr_read_checked(0x000)?;
    if v & 0x200 != 0 {
        chip.mmr_write_checked(0x000, v & !0x200)?;
    }
    chip.mmr_modify_checked(0x008, |v| v | 0x200)?;
    chip.mmr_write_checked(PRESCALE, 0x3C)?;
    chip.mmr_write_checked(RESET, 0xFFFF_FFFE)?;
    chip.wait_ticks(1);
    chip.mmr_write_checked(RESET, 1)?;
    chip.wait_ticks(1);
    chip.mmr_modify_checked(GPIO_FUNC, |v| v & !0x1E0)?;
    chip.wait_ticks(10);
    chip.mmr_write_checked(RDATA, 0)?;
    chip.mmr_write_checked(reg(1, RDATA), 0)?;
    let a = chip.mmr_read_checked(STATUS)?;
    let b = chip.mmr_read(STATUS)?;
    chip.mmr_write_checked(STATUS, a | 0x0030_00C0 | b)?;
    chip.mmr_modify_checked(IRQ_ENABLE, |v| v | 0x0030_00C0)?;
    chip.mmr_modify_checked(INT_ENABLE, |v| v | 0x80)?;
    chip.mmr_write_checked(GLOBAL_DIV, 0x67)
}

/// Wait until the free-running timer advances by `n` between two reads.
fn timer_wait<B: RemotePci>(chip: &mut Chip<B>, n: u32) -> Result<()> {
    let mut t0 = chip.mmr_read_checked(TIMER)?;
    for _ in 0..TIMER_READ_LIMIT {
        let t = chip.mmr_read_checked(TIMER)?;
        if t.wrapping_sub(t0) >= n {
            return Ok(());
        }
        t0 = t;
    }
    Err(Error::Timeout)
}

/// The vendor's bus-recovery check: when SCL reads high and SDA low four
/// times in a row, clock the bus nine times through GPIO 5/6 and reset the
/// block. Returns whether a recovery was done.
pub fn bus_recovery<B: RemotePci>(chip: &mut Chip<B>) -> Result<bool> {
    let mut stuck = true;
    for _ in 0..4 {
        if chip.mmr_read_checked(GPIO_IN)? & 0x60 != 0x20 {
            stuck = false;
            break;
        }
        timer_wait(chip, 1)?;
    }
    if !stuck {
        return Ok(false);
    }
    log::warn!("I2C bus recovery required");
    let n = chip.mmr_read_checked(RECOVERY_COUNT)?;
    if n != 0xFFFF_FFFF {
        chip.mmr_write(RECOVERY_COUNT, n.wrapping_add(1))?;
    }
    chip.mmr_modify_checked(GPIO_FUNC, |v| v | 0x60)?;
    chip.mmr_modify_checked(GPIO_DATA, |v| v & !0x60)?;
    chip.mmr_modify_checked(GPIO_DIR, |v| v | 0x60)?;
    timer_wait(chip, 1)?;
    for _ in 0..9 {
        for (mask, set) in [(0x20, true), (0x40, true), (0x20, false), (0x40, false)] {
            chip.mmr_modify_checked(GPIO_DATA, |v| (v & !mask) | if set { mask } else { 0 })?;
            timer_wait(chip, 1)?;
        }
    }
    chip.mmr_read_checked(GPIO_IN)?;
    chip.mmr_modify_checked(GPIO_FUNC, |v| v & !0x20)?;
    timer_wait(chip, 1)?;
    chip.mmr_modify_checked(GPIO_FUNC, |v| v & !0x40)?;
    chip.mmr_modify_checked(PRESCALE, |v| v | 0x80)?;
    timer_wait(chip, 1)?;
    chip.mmr_write(RDATA, 0)?;
    chip.mmr_modify_checked(PRESCALE, |v| v & !0x80)?;
    timer_wait(chip, 1)?;
    Ok(true)
}

/// Poll [`STATUS`] until channel `ch` reports completion, then do what the
/// vendor's interrupt handler does: read and acknowledge both channels.
/// Returns the status seen for `ch`.
fn wait_done<B: RemotePci>(chip: &mut Chip<B>, ch: u8) -> Result<Option<u32>> {
    let mask = DONE_MASK[usize::from(ch)];
    let mut polled = None;
    for i in 0..POLL_LIMIT {
        let st = chip.mmr_read(STATUS)?;
        if st & mask != 0 {
            polled = Some(st);
            break;
        }
        if i >= POLL_FAST {
            chip.sleep(POLL_INTERVAL);
        }
    }
    let Some(polled) = polled else { return Ok(None) };
    let mut seen = polled;
    for (c, m) in DONE_MASK.iter().enumerate() {
        let st = chip.mmr_read(STATUS)?;
        if c == usize::from(ch) && st & m != 0 {
            seen = st;
        }
        chip.mmr_write(STATUS, *m)?;
    }
    Ok(Some(seen))
}

/// After a NAK on a transaction without STOP, put a STOP on the bus.
fn put_stop<B: RemotePci>(chip: &mut Chip<B>, ch: u8) -> Result<()> {
    chip.mmr_modify_checked(INT_ENABLE, |v| v & !0x80)?;
    chip.mmr_write_checked(reg(ch, CTRL), STOP_ONLY)?;
    if wait_done(chip, ch)?.is_none() {
        log::warn!("I2C channel {ch}: STOP after NAK did not complete");
    }
    chip.mmr_modify_checked(INT_ENABLE, |v| v | 0x80)
}

/// One transaction of at most 64 bytes on channel `ch`. For a write, `buf`
/// holds the data; for a read, it receives it.
#[allow(clippy::too_many_arguments)]
pub fn transaction<B: RemotePci>(
    chip: &mut Chip<B>,
    ch: u8,
    timing: u32,
    addr8: u8,
    write: bool,
    start: bool,
    stop: bool,
    buf: &mut [u8],
) -> Result<()> {
    if ch > 1 {
        return Err(Error::Unsupported(format!("I2C channel {ch}")));
    }
    if buf.len() > MAX_LEN {
        return Err(Error::TooLarge { limit: MAX_LEN, actual: buf.len() });
    }
    let reg_hint = if write { buf.first().copied() } else { None };
    let fail = |msg| Error::I2c { addr: addr8, reg: reg_hint, msg };
    if write {
        for chunk in buf.chunks(4) {
            let mut w = [0u8; 4];
            w[..chunk.len()].copy_from_slice(chunk);
            chip.mmr_write(reg(ch, WDATA), u32::from_le_bytes(w))?;
        }
    }
    chip.mmr_write(reg(ch, TIMING), timing)?;
    chip.mmr_write(reg(ch, CTRL), ctrl_word(buf.len(), start, stop, write, addr8))?;
    let Some(st) = wait_done(chip, ch)? else {
        return Err(fail("completion timeout"));
    };

    let ctrl = chip.mmr_read(reg(ch, CTRL))?;
    let result = if st & HW_ERROR != 0 {
        bus_recovery(chip)?;
        chip.mmr_write(reg(ch, RDATA), 0)?;
        Err(fail("bus error"))
    } else if ctrl & CMD_ERROR == 0 {
        // The vendor runs the recovery check here too and ignores the result.
        bus_recovery(chip)?;
        Ok(())
    } else {
        bus_recovery(chip)?;
        chip.mmr_write(reg(ch, RDATA), 0)?;
        if !stop {
            put_stop(chip, ch)?;
        }
        Err(fail("no acknowledge"))
    };
    if !write {
        // The vendor drains the read FIFO in every outcome.
        for chunk in buf.chunks_mut(4) {
            let w = chip.mmr_read(reg(ch, RDATA))?.to_le_bytes();
            chunk.copy_from_slice(&w[..chunk.len()]);
        }
    }
    result
}

/// One channel of the I2C master at a fixed bus clock, as an [`I2c`] bus.
pub struct I2cMaster<'a, B> {
    chip: &'a mut Chip<B>,
    channel: u8,
    timing: u32,
}

impl<'a, B: RemotePci> I2cMaster<'a, B> {
    /// Channel `channel` (0 or 1) at `khz` with the vendor's TIMING upper half.
    pub fn new(chip: &'a mut Chip<B>, channel: u8, khz: u32) -> Self {
        Self { chip, channel, timing: clkdiv(khz) | DEFAULT_PARAM << 16 }
    }

    /// The TIMING register value used for every transaction.
    pub fn timing(&self) -> u32 {
        self.timing
    }

    fn xfer(&mut self, addr8: u8, write: bool, start: bool, stop: bool, buf: &mut [u8]) -> Result<()> {
        transaction(self.chip, self.channel, self.timing, addr8, write, start, stop, buf)
    }

    fn write_parts(&mut self, addr8: u8, data: &[u8], stop: bool) -> Result<()> {
        if data.is_empty() {
            return self.xfer(addr8, true, true, stop, &mut []);
        }
        let n = data.len().div_ceil(MAX_LEN);
        for (i, chunk) in data.chunks(MAX_LEN).enumerate() {
            let mut buf = chunk.to_vec();
            self.xfer(addr8, true, i == 0, stop && i + 1 == n, &mut buf)?;
        }
        Ok(())
    }
}

impl<B: RemotePci> I2c for I2cMaster<'_, B> {
    fn write(&mut self, addr8: u8, data: &[u8]) -> Result<()> {
        self.write_parts(addr8, data, true)
    }

    fn write_read(&mut self, addr8: u8, wr: &[u8], rd: &mut [u8]) -> Result<()> {
        if rd.is_empty() {
            return self.write(addr8, wr);
        }
        if !wr.is_empty() {
            self.write_parts(addr8, wr, false)?;
        }
        let n = rd.len().div_ceil(MAX_LEN);
        for (i, chunk) in rd.chunks_mut(MAX_LEN).enumerate() {
            self.xfer(addr8, false, i == 0, i + 1 == n, chunk).map_err(|e| match e {
                Error::I2c { addr, msg, .. } => Error::I2c { addr, reg: wr.first().copied(), msg },
                e => e,
            })?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::super::mmr::testutil::*;
    use super::*;
    use crate::bus::mock::{MockPci, Space};

    #[test]
    fn control_word_encoding() {
        assert_eq!(ctrl_word(3, true, true, true, 0x20), 3 | 0x100 | 0x200 | 0x1000 | 0x10 << 16);
        assert_eq!(ctrl_word(1, true, true, false, 0xC1), 1 | 0x300 | 0x60 << 16);
        assert_eq!(ctrl_word(64, false, false, true, 0x3C), 0x40 | 0x1000 | 0x1E << 16);
        assert_eq!(ctrl_word(0, false, true, true, 0), STOP_ONLY);
        assert_eq!(ctrl_word(2, true, false, true, 0xFE), 2 | 0x100 | 0x1000 | 0x7F << 16);
    }

    #[test]
    fn clock_divider() {
        assert_eq!(clkdiv(100), 415);
        assert_eq!(clkdiv(400), 103);
        assert_eq!(clkdiv(40), 1037);
    }

    fn done(c: &Chip<MockPci>, ch: u8) {
        c.bus().set(Space::Mem, MMR + STATUS, DONE_MASK[usize::from(ch)] & 0xC0);
    }

    #[test]
    fn write_transaction() {
        let mut c = chip(0x0300_00B2);
        done(&c, 0);
        c.bus().set(Space::Mem, MMR + GPIO_IN, 0x60);
        I2cMaster::new(&mut c, 0, 100).write(0x20, &[0x03, 0x10, 0xAB, 0xCD, 0xEF]).unwrap();
        use Acc::*;
        assert_eq!(
            accesses(&c.bus().ops()),
            vec![
                W(0x1004, 0xCDAB_1003),
                W(0x1004, 0x0000_00EF),
                W(0x1000, 415 | 100 << 16),
                W(0x100C, 5 | 0x300 | 0x1000 | 0x10 << 16),
                R(0x1094),
                R(0x1094),
                W(0x1094, 0x0030_00C0),
                R(0x1094),
                W(0x1094, 0x80),
                R(0x100C),
                R(0x1090),
            ]
        );
    }

    #[test]
    fn register_read_on_channel_1() {
        let mut c = chip(0x0300_00B2);
        // Status as the hardware shows it (the mock does not clear on write):
        // poll and the two handler reads, for both transactions.
        c.bus().queue(Space::Mem, MMR + STATUS, &[0x80; 6]);
        c.bus().set(Space::Mem, MMR + 0x1018, 0x4433_2211);
        let mut rd = [0u8; 3];
        I2cMaster::new(&mut c, 1, 400).write_read(0x30, &[0x80], &mut rd).unwrap();
        assert_eq!(rd, [0x11, 0x22, 0x33]);
        let w = writes(&c.bus().ops());
        use Acc::W;
        assert_eq!(
            w,
            vec![
                W(0x1014, 0x80),
                W(0x1010, 103 | 100 << 16),
                W(0x101C, 1 | 0x100 | 0x1000 | 0x18 << 16),
                W(0x1094, 0x0030_00C0),
                W(0x1094, 0x80),
                W(0x1010, 103 | 100 << 16),
                W(0x101C, 3 | 0x300 | 0x18 << 16),
                W(0x1094, 0x0030_00C0),
                W(0x1094, 0x80),
            ]
        );
        assert!(accesses(&c.bus().ops()).ends_with(&[Acc::R(0x101C), Acc::R(0x1090), Acc::R(0x1018)]));
    }

    #[test]
    fn nak_without_stop_puts_stop_on_bus() {
        let mut c = chip(0x0300_00B2);
        done(&c, 0);
        c.bus().queue(Space::Mem, MMR + 0x100C, &[CMD_ERROR]);
        let mut rd = [0u8; 1];
        let e = I2cMaster::new(&mut c, 0, 100).write_read(0xC0, &[0x09], &mut rd).unwrap_err();
        assert!(matches!(e, Error::I2c { addr: 0xC0, reg: Some(0x09), msg: "no acknowledge" }), "{e:?}");
        // The write part fails; the read part is never sent.
        let w = writes(&c.bus().ops());
        use Acc::W;
        assert_eq!(
            &w[3..],
            &[
                W(0x1094, 0x0030_00C0),
                W(0x1094, 0x80),
                W(0x1008, 0),
                W(0x418, 0),
                W(0x100C, STOP_ONLY),
                W(0x1094, 0x0030_00C0),
                W(0x1094, 0x80),
                W(0x418, 0x80),
            ]
        );
    }

    #[test]
    fn nak_with_stop_reads_fifo_anyway() {
        let mut c = chip(0x0300_00B2);
        done(&c, 0);
        c.bus().queue(Space::Mem, MMR + 0x100C, &[CMD_ERROR]);
        let mut rd = [0u8; 5];
        let e = transaction(&mut c, 0, 0, 0x20, false, true, true, &mut rd).unwrap_err();
        assert!(matches!(e, Error::I2c { msg: "no acknowledge", .. }));
        let acc = accesses(&c.bus().ops());
        assert!(!acc.contains(&Acc::W(0x100C, STOP_ONLY)));
        assert!(acc.ends_with(&[Acc::W(0x1008, 0), Acc::R(0x1008), Acc::R(0x1008)]));
    }

    #[test]
    fn hw_error_and_timeout() {
        let mut c = chip(0x0300_00B2);
        c.bus().set(Space::Mem, MMR + STATUS, 0x0010_0000);
        let e = I2cMaster::new(&mut c, 0, 100).write(0x20, &[1, 2]).unwrap_err();
        assert!(matches!(e, Error::I2c { msg: "bus error", .. }));
        let mut c = chip(0x0300_00B2);
        let e = I2cMaster::new(&mut c, 1, 100).write(0x20, &[1]).unwrap_err();
        assert!(matches!(e, Error::I2c { msg: "completion timeout", .. }));
        let polls = accesses(&c.bus().ops()).iter().filter(|a| **a == Acc::R(0x1094)).count();
        assert_eq!(polls, POLL_LIMIT as usize);
    }

    #[test]
    fn long_transfers_are_split() {
        let mut c = chip(0x0300_00B2);
        done(&c, 0);
        let data: Vec<u8> = (0..150).collect();
        I2cMaster::new(&mut c, 0, 100).write(0x20, &data).unwrap();
        let ctrls: Vec<u32> = writes(&c.bus().ops())
            .into_iter()
            .filter_map(|a| match a {
                Acc::W(0x100C, v) => Some(v),
                _ => None,
            })
            .collect();
        let a = 0x10 << 16 | 0x1000;
        assert_eq!(ctrls, vec![a | 0x100 | 64, a | 64, a | 0x200 | 22]);
        let mut c = chip(0x0300_00B2);
        done(&c, 0);
        let mut rd = vec![0u8; 100];
        I2cMaster::new(&mut c, 0, 100).write_read(0x20, &[], &mut rd).unwrap();
        let ctrls: Vec<u32> = writes(&c.bus().ops())
            .into_iter()
            .filter_map(|a| match a {
                Acc::W(0x100C, v) => Some(v),
                _ => None,
            })
            .collect();
        assert_eq!(ctrls, vec![0x10 << 16 | 0x100 | 64, 0x10 << 16 | 0x200 | 36]);
        assert!(transaction(&mut c, 0, 0, 0x20, true, true, true, &mut [0; 65]).is_err());
        assert!(transaction(&mut c, 2, 0, 0x20, true, true, true, &mut [0; 1]).is_err());
    }

    #[test]
    fn init_sequence() {
        let mut c = chip(0x0300_00B2);
        c.bus().set(Space::Mem, MMR, 0x0000_0201);
        c.bus().set(Space::Mem, MMR + 0x1084, 0xFFFF_FFFF);
        c.bus().queue(Space::Mem, MMR + STATUS, &[0x1, 0x2]);
        init(&mut c).unwrap();
        use Acc::*;
        assert_eq!(
            accesses(&c.bus().ops()),
            vec![
                R(0x000),
                W(0x000, 0x1),
                R(0x008),
                W(0x008, 0x200),
                W(0x10A0, 0x3C),
                W(0x10A4, 0xFFFF_FFFE),
                W(0x10A4, 1),
                R(0x1084),
                W(0x1084, 0xFFFF_FE1F),
                W(0x1008, 0),
                W(0x1018, 0),
                R(0x1094),
                R(0x1094),
                W(0x1094, 0x0030_00C3),
                R(0x1098),
                W(0x1098, 0x0030_00C0),
                R(0x418),
                W(0x418, 0x80),
                W(0x10A8, 0x67),
            ]
        );
        // Only the second 0x1094 read is plain.
        assert_eq!(checks(&c.bus().ops()), 18);
    }

    #[test]
    fn recovery_clocks_the_bus_nine_times() {
        let mut c = chip(0x0300_00B2);
        c.bus().set(Space::Mem, MMR + GPIO_IN, 0x20);
        c.bus().set(Space::Mem, MMR + RECOVERY_COUNT, 4);
        let ramp: Vec<u32> = (0..200).collect();
        c.bus().queue(Space::Mem, MMR + TIMER, &ramp);
        assert!(bus_recovery(&mut c).unwrap());
        let w = writes(&c.bus().ops());
        use Acc::W;
        assert_eq!(&w[..4], &[W(0x10FC, 5), W(0x1084, 0x60), W(0x1088, 0), W(0x108C, 0x60)]);
        let toggles: Vec<_> = w[4..4 + 36].to_vec();
        for k in 0..9 {
            assert_eq!(&toggles[4 * k..4 * k + 4], &[W(0x1088, 0x20), W(0x1088, 0x60), W(0x1088, 0x40), W(0x1088, 0)]);
        }
        assert_eq!(&w[40..], &[W(0x1084, 0x40), W(0x1084, 0), W(0x10A0, 0x80), W(0x1008, 0), W(0x10A0, 0)]);
        // 4 checks + 1 + 36 + 3 timer waits, two reads each.
        let timer = c.bus().ops().iter().filter(|o| **o == crate::bus::mock::Op::MemRead(MMR + TIMER)).count();
        assert_eq!(timer, 2 * (4 + 1 + 36 + 3));
    }

    #[test]
    fn no_recovery_when_bus_idle() {
        let mut c = chip(0x0300_00B2);
        c.bus().set(Space::Mem, MMR + GPIO_IN, 0x60);
        assert!(!bus_recovery(&mut c).unwrap());
        assert_eq!(accesses(&c.bus().ops()), vec![Acc::R(0x1090)]);
    }
}
