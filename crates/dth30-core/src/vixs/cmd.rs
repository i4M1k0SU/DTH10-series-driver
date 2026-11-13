//! Host-to-firmware commands and firmware-to-host messages
//! (docs/26 2-vixs-bringup.md §5.1, §5.3, §8.3).
//!
//! Commands live in a ring of 41 slots of 0x23C0 bytes in SDRAM (after the
//! DMA descriptor list in the pool the firmware assigned). A command is sent
//! by writing its whole slot with one DMA transfer and ringing the doorbells
//! (MMR 0x3030 = slot address, 0x3020 = function ID). The firmware signals
//! completion with bit 1 of MMR 0x30E0 and the function ID in 0x302C and the
//! slot address in 0x3048, and leaves the result in the slot at +0x10
//! (1 = success). Only one command is outstanding at a time, as in the
//! vendor driver.
//!
//! Messages from the firmware (bit 0 of 0x30E0) carry an ID in 0x3028 and
//! the SDRAM address of nine words in 0x3040; the host answers by writing
//! those two values back to 0x3038 / 0x3024.
//!
//! Not verified on hardware.

use std::time::{Duration, Instant};

use super::Vixs;
use super::mips::{self, M2H_DONE_FUNC, M2H_DONE_SLOT, M2H_INT_ENABLE, M2H_INT_STATUS};
use super::mmr::{Chip, INT_ENABLE, INT_STATUS, TIMER};
use crate::bus::RemotePci;
use crate::net2280::{self, IRQ_PCI_INTA};
use crate::{Error, Result};

/// Size of one command slot.
pub const SLOT_SIZE: usize = 0x23C0;
/// Most slots the vendor uses.
pub const MAX_SLOTS: u32 = 41;
/// Offset of the payload in a slot, and its largest size.
pub const PAYLOAD_OFFSET: usize = 0x24;
pub const MAX_PAYLOAD: usize = SLOT_SIZE - PAYLOAD_OFFSET;
/// Offset of the result word the firmware writes.
pub const RESULT_OFFSET: u32 = 0x10;
/// Result value of a successful command.
pub const RESULT_OK: u32 = 1;

/// Function IDs seen in the vendor driver.
pub mod func {
    pub const OPEN_STREAM: u32 = 8;
    pub const CLOSE_STREAM: u32 = 9;
    pub const CONTROL: u32 = 10;
    pub const INPUT: u32 = 11;
    pub const OUTPUT_REQUEST: u32 = 13;
    pub const INTERNAL_CONTROL: u32 = 20;
    pub const TS_CONTROL: u32 = 21;
    pub const I2C_DEVICE: u32 = 22;
    pub const TSD_CONFIG: u32 = 24;
    pub const OPEN_GENERIC: u32 = 25;
    pub const CLOSE_GENERIC: u32 = 26;
    pub const OTP_SIGNATURE: u32 = 27;
}

/// FB address of the OTP signature (command 27 uses no slot).
pub const OTP_FB: u32 = 0x5_DC40;
/// Size of the OTP signature in words.
pub const OTP_WORDS: usize = 28;
/// Message IDs at or above this are refused with status 5.
pub const M2H_ID_LIMIT: u32 = 0x1C;
/// Words of a firmware message.
pub const M2H_WORDS: usize = 9;
/// Heartbeat timer advance after which the vendor reports a hang.
pub const HANG_TICKS: u32 = 0x1C2;

/// The command ring.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CmdRing {
    /// FB address of slot 0.
    pub fb_base: u32,
    /// Number of slots.
    pub slots: u32,
    next: u32,
    /// Slots held by open streams (bit i = slot i), skipped by
    /// [`CmdRing::next_slot`].
    reserved: u64,
}

impl CmdRing {
    /// The ring over `size` bytes at `fb_base`: `min(size / 0x23C0, 41)`
    /// slots; fewer than two is an error, as in the vendor driver.
    pub fn new(fb_base: u32, size: u32) -> Result<Self> {
        let n = size / SLOT_SIZE as u32;
        if n < 2 {
            return Err(Error::Protocol("command list too small"));
        }
        Ok(Self { fb_base, slots: n.min(MAX_SLOTS), next: 0, reserved: 0 })
    }

    /// FB address of slot `i`.
    pub fn slot_fb(&self, i: u32) -> u32 {
        self.fb_base + SLOT_SIZE as u32 * i
    }

    /// The next slot that is not reserved, round robin.
    pub fn next_slot(&mut self) -> u32 {
        // `reserve` always leaves one slot free, so this ends.
        loop {
            let i = self.next;
            self.next = (i + 1) % self.slots;
            if !self.is_reserved(i) {
                return i;
            }
        }
    }

    /// Hold slot `i` until [`CmdRing::release`]. A stream keeps its OPEN
    /// slot because the CLOSE is sent in the same slot (docs/26
    /// 6-stream-payloads.md §2.5, §6.4); whether the firmware still reads
    /// the slot while the stream is open is not known, so other commands
    /// stay out of it. At least one slot always stays free.
    pub fn reserve(&mut self, i: u32) -> Result<()> {
        if i >= self.slots {
            return Err(Error::Protocol("command slot out of range"));
        }
        if self.is_reserved(i) {
            return Ok(());
        }
        if self.reserved.count_ones() + 2 > self.slots {
            return Err(Error::Protocol("no free command slot left"));
        }
        self.reserved |= 1 << i;
        Ok(())
    }

    /// Give a reserved slot back.
    pub fn release(&mut self, i: u32) {
        if i < self.slots {
            self.reserved &= !(1 << i);
        }
    }

    pub fn is_reserved(&self, i: u32) -> bool {
        i < self.slots && self.reserved & (1 << i) != 0
    }
}

/// The 0x23C0 bytes of a command slot.
pub fn build_slot(index: u32, slot_fb: u32, func: u32, payload: &[u8]) -> Result<Vec<u8>> {
    if payload.len() > MAX_PAYLOAD {
        return Err(Error::TooLarge { limit: MAX_PAYLOAD, actual: payload.len() });
    }
    let mut s = vec![0u8; SLOT_SIZE];
    let mut put = |off: usize, v: u32| s[off..off + 4].copy_from_slice(&v.to_le_bytes());
    put(0x08, 1);
    put(0x0C, func);
    put(0x10, 0);
    put(0x14, if payload.is_empty() { 0 } else { slot_fb + PAYLOAD_OFFSET as u32 });
    put(0x18, 0);
    put(0x1C, index);
    s[PAYLOAD_OFFSET..PAYLOAD_OFFSET + payload.len()].copy_from_slice(payload);
    Ok(s)
}

/// The outcome of a command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CmdResult {
    pub func: u32,
    pub slot: u32,
    pub slot_fb: u32,
    /// The firmware's result word (1 = success).
    pub status: u32,
}

impl CmdResult {
    pub fn ok(&self) -> bool {
        self.status == RESULT_OK
    }

    /// FB address of the payload (many commands write answers into it).
    pub fn payload_fb(&self) -> u32 {
        self.slot_fb + PAYLOAD_OFFSET as u32
    }
}

/// A message from the firmware.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct M2hMessage {
    pub id: u32,
    /// FB address of the message words.
    pub addr: u32,
    pub words: [u32; M2H_WORDS],
}

/// Handler for firmware messages: returns whether it handled the message.
/// Unhandled messages are refused the way the vendor refuses messages
/// without a registered handler. The handler must not acknowledge.
pub type M2hHandler<B> = Box<dyn FnMut(&mut Chip<B>, &M2hMessage) -> Result<bool> + Send>;

/// How command completion is waited for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Completion {
    /// Wait on the NET2280 interrupt endpoint (ViXS INTA) and run the
    /// interrupt handler; if no interrupt arrives within a slice, check the
    /// MIPS status registers directly.
    Interrupt,
    /// Only poll the MIPS status registers.
    Poll,
}

/// Result of a heartbeat check.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Heartbeat {
    /// The firmware's counter moved.
    Alive,
    /// It did not move, but not for long.
    Unchanged,
    /// It did not move for more than [`HANG_TICKS`] of the chip timer.
    Hung,
}

#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct HeartbeatState {
    last: Option<u32>,
    since: u32,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Pending {
    func: u32,
    slot: u32,
    slot_fb: u32,
    status: Option<u32>,
}

/// Slice of an interrupt wait before the status registers are polled.
const IRQ_SLICE: Duration = Duration::from_millis(20);
const POLL_INTERVAL: Duration = Duration::from_millis(2);
/// Bound of the top-level interrupt handler's loop.
const ISR_ROUNDS: u32 = 16;
/// Bound of the OTP completion poll (the vendor's is unbounded).
const OTP_POLL_LIMIT: u32 = 10_000;

impl<B: RemotePci> Vixs<B> {
    /// Send command `func` with `payload` in the next ring slot and wait
    /// for its completion.
    pub fn send_cmd(&mut self, func: u32, payload: &[u8]) -> Result<CmdResult> {
        let slot = self.ring.as_mut().ok_or(Error::Protocol("ViXS firmware not started"))?.next_slot();
        self.send_cmd_slot(slot, func, payload)
    }

    /// Send a command in a given slot (the vendor reuses the OPEN slot for
    /// the matching CLOSE).
    pub fn send_cmd_slot(&mut self, slot: u32, func: u32, payload: &[u8]) -> Result<CmdResult> {
        let ring = self.ring.as_ref().ok_or(Error::Protocol("ViXS firmware not started"))?;
        if slot >= ring.slots {
            return Err(Error::Protocol("command slot out of range"));
        }
        let slot_fb = ring.slot_fb(slot);
        if self.pending.is_some() {
            return Err(Error::Protocol("a ViXS command is already outstanding"));
        }
        if !self.alive {
            self.heartbeat()?;
            if !self.alive {
                return Err(Error::Protocol("ViXS firmware heartbeat not seen"));
            }
        }
        let data = build_slot(slot, slot_fb, func, payload)?;
        // The vendor fills five header words of its host-side copy through a
        // board-ID-checked writer.
        for _ in 0..5 {
            self.chip.check_once()?;
        }
        net2280::dma_write(self.chip.bus(), &self.layout, self.chip.fb + slot_fb, &data)?;
        self.chip.mmr_write_checked(mips::H2M_SLOT, slot_fb)?;
        self.chip.mmr_write_checked(mips::H2M_FUNC, func)?;
        self.pending = Some(Pending { func, slot, slot_fb, status: None });
        let r = self.wait_pending(self.opts.cmd_timeout);
        let p = self.pending.take();
        r?;
        let status = p.and_then(|p| p.status).ok_or(Error::Timeout)?;
        Ok(CmdResult { func, slot, slot_fb, status })
    }

    fn wait_pending(&mut self, timeout: Duration) -> Result<()> {
        let deadline = Instant::now() + timeout;
        loop {
            match self.opts.completion {
                Completion::Interrupt => {
                    let slice = deadline.saturating_duration_since(Instant::now()).min(IRQ_SLICE);
                    match self.irq.wait(self.chip.bus(), slice)? {
                        Some(stat) if stat & IRQ_PCI_INTA != 0 => self.service_interrupt()?,
                        Some(_) => {}
                        None => self.mips_isr()?,
                    }
                }
                Completion::Poll => self.mips_isr()?,
            }
            if self.pending.is_some_and(|p| p.status.is_some()) {
                return Ok(());
            }
            if Instant::now() >= deadline {
                log::error!("ViXS command {} timed out", self.pending.map_or_default(|p| p.func));
                return Err(Error::Timeout);
            }
            if self.opts.completion == Completion::Poll {
                self.chip.sleep(POLL_INTERVAL);
            }
        }
    }

    /// The vendor's top-level ViXS interrupt handler: dispatch the pending
    /// sources in MMR 0x414 & 0x418 until none is left.
    pub fn service_interrupt(&mut self) -> Result<()> {
        for _ in 0..ISR_ROUNDS {
            let s = self.chip.mmr_read_checked(INT_STATUS)? & self.chip.mmr_read_checked(INT_ENABLE)?;
            if s == 0 {
                return Ok(());
            }
            if s & 0x80 != 0 {
                let st = self.chip.mmr_read_checked(super::i2c::STATUS)?;
                if st & 0x0030_00C0 != 0 {
                    // No I2C transaction is in flight outside `i2c::transaction`;
                    // acknowledge as the vendor's handler does.
                    for m in super::i2c::DONE_MASK {
                        self.chip.mmr_read(super::i2c::STATUS)?;
                        self.chip.mmr_write(super::i2c::STATUS, m)?;
                    }
                }
                if st & 0x1F != 0 {
                    log::debug!("ViXS GPIO interrupt {st:#x} not handled");
                }
            }
            if s & 0x100 != 0 {
                super::dma::isr(&mut self.chip)?;
            }
            if s & 0x200 != 0 {
                self.mips_isr()?;
            }
        }
        log::warn!("ViXS interrupt sources still pending after {ISR_ROUNDS} rounds");
        Ok(())
    }

    /// The vendor's MIPS interrupt handler: command completion (bit 1) and
    /// firmware messages (bit 0) in MMR 0x30E0 & 0x30E4. Also usable as a
    /// poll without an interrupt.
    pub fn mips_isr(&mut self) -> Result<()> {
        if !self.alive || self.chip.is_removed() {
            return Ok(());
        }
        let s = self.chip.mmr_read_checked(M2H_INT_STATUS)? & self.chip.mmr_read_checked(M2H_INT_ENABLE)?;
        if s & 2 != 0 {
            let done = self.chip.mmr_read_checked(M2H_DONE_FUNC)?;
            if let Some(p) = self.pending
                && p.func == done
            {
                self.chip.mmr_write(M2H_INT_STATUS, 2)?;
                let slot = self.chip.mmr_read(M2H_DONE_SLOT)?;
                if slot == p.slot_fb {
                    let status = self.chip.fb_read(p.slot_fb + RESULT_OFFSET)?;
                    self.pending = Some(Pending { status: Some(status), ..p });
                } else {
                    log::warn!("ViXS command {done} completed for slot {slot:#x}, expected {:#x}", p.slot_fb);
                }
                log::trace!("ViXS command {} in slot {} done", p.func, p.slot);
            }
        }
        if s & 1 != 0 {
            self.chip.mmr_write(M2H_INT_STATUS, 1)?;
            self.handle_m2h()?;
        }
        Ok(())
    }

    /// Read, dispatch and acknowledge the current firmware message.
    fn handle_m2h(&mut self) -> Result<()> {
        if !self.alive || self.chip.is_removed() {
            return Ok(());
        }
        let id = self.chip.mmr_read_checked(mips::M2H_ID)?;
        let addr = self.chip.mmr_read_checked(mips::M2H_ADDR)?;
        let mut words = [0; M2H_WORDS];
        for (i, w) in words.iter_mut().enumerate() {
            *w = self.chip.fb_read(addr + 4 * i as u32)?;
        }
        let msg = M2hMessage { id, addr, words };
        if id < M2H_ID_LIMIT {
            let handled = match self.m2h_handler.as_mut() {
                Some(h) if words[3] == id => h(&mut self.chip, &msg)?,
                _ => false,
            };
            if handled {
                return mips::ack_m2h(&mut self.chip);
            }
            log::debug!("ViXS message {id:#x} not handled");
            self.chip.fb_write(addr + 0x10, 2)?;
        }
        self.chip.fb_write(addr + 0x10, 5)?;
        mips::ack_m2h(&mut self.chip)
    }

    /// Install the handler for firmware messages.
    pub fn set_m2h_handler(&mut self, handler: M2hHandler<B>) {
        self.m2h_handler = Some(handler);
    }

    /// Check the firmware heartbeat (FB 0x10) against the chip timer, as the
    /// vendor's timer thread does. A change marks the firmware alive.
    pub fn heartbeat(&mut self) -> Result<Heartbeat> {
        let off = if self.chip.board_id & super::mmr::CHIP_MASK != 0 { mips::fb::HEARTBEAT } else { 0x500 };
        let hb = self.chip.fb_read_checked(off)?;
        let now = self.chip.mmr_read_checked(TIMER)?;
        if self.hb.last == Some(hb) {
            if now.wrapping_sub(self.hb.since) > HANG_TICKS {
                log::warn!("ViXS firmware heartbeat stuck at {hb:#x}");
                return Ok(Heartbeat::Hung);
            }
            return Ok(Heartbeat::Unchanged);
        }
        self.alive = true;
        self.hb = HeartbeatState { last: Some(hb), since: now };
        Ok(Heartbeat::Alive)
    }

    /// Whether a heartbeat change has been seen (commands need it).
    pub fn is_alive(&self) -> bool {
        self.alive
    }

    /// Read the OTP signature (command 27, which bypasses the ring).
    pub fn read_otp_signature(&mut self) -> Result<[u32; OTP_WORDS]> {
        self.chip.mmr_write(mips::H2M_SLOT, OTP_FB)?;
        self.chip.mmr_write(mips::H2M_FUNC, func::OTP_SIGNATURE)?;
        let mut n = 0;
        while self.chip.mmr_read(mips::H2M_SLOT)? != 0 {
            n += 1;
            if n >= OTP_POLL_LIMIT {
                return Err(Error::Timeout);
            }
        }
        let mut sig = [0; OTP_WORDS];
        for (i, w) in sig.iter_mut().enumerate() {
            *w = self.chip.fb_read(OTP_FB + 4 * i as u32)?;
        }
        Ok(sig)
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use super::super::mmr::testutil::*;
    use super::super::tests::running;
    use super::*;
    use crate::bus::mock::{MockPci, Space};

    #[test]
    fn slot_layout() {
        let s = build_slot(3, 0x12_0000, 24, &[1, 2, 3, 4, 5]).unwrap();
        assert_eq!(s.len(), 0x23C0);
        let w = |o: usize| u32::from_le_bytes(s[o..o + 4].try_into().unwrap());
        assert_eq!(
            [w(0), w(4), w(8), w(0xC), w(0x10), w(0x14), w(0x18), w(0x1C), w(0x20)],
            [0, 0, 1, 24, 0, 0x12_0024, 0, 3, 0]
        );
        assert_eq!(&s[0x24..0x29], &[1, 2, 3, 4, 5]);
        assert!(s[0x29..].iter().all(|&b| b == 0));
        let s = build_slot(0, 0x12_0000, 9, &[]).unwrap();
        assert_eq!(u32::from_le_bytes(s[0x14..0x18].try_into().unwrap()), 0);
        assert!(build_slot(0, 0, 8, &vec![0; MAX_PAYLOAD + 1]).is_err());
        assert!(build_slot(0, 0, 8, &vec![0; MAX_PAYLOAD]).is_ok());
    }

    #[test]
    fn ring_geometry() {
        let mut r = CmdRing::new(0x10_0840, mips::COM_LIST_SIZE).unwrap();
        assert_eq!(r.slots, 41);
        assert_eq!(r.slot_fb(2), 0x10_0840 + 2 * 0x23C0);
        assert_eq!((0..42).map(|_| r.next_slot()).last(), Some(0));
        assert_eq!(CmdRing::new(0, 0x23C0 * 5).unwrap().slots, 5);
        assert!(CmdRing::new(0, 0x23C0).is_err());
    }

    #[test]
    fn reserved_slots_are_skipped() {
        let mut r = CmdRing::new(0, 0x23C0 * 4).unwrap();
        r.reserve(1).unwrap();
        r.reserve(2).unwrap();
        assert!(r.is_reserved(1) && !r.is_reserved(0));
        assert_eq!((0..4).map(|_| r.next_slot()).collect::<Vec<_>>(), [0, 3, 0, 3]);
        // One slot always stays free.
        r.reserve(3).unwrap();
        r.reserve(3).unwrap();
        assert!(r.reserve(0).is_err());
        assert!(r.reserve(4).is_err());
        assert_eq!((0..2).map(|_| r.next_slot()).collect::<Vec<_>>(), [0, 0]);
        r.release(1);
        r.release(3);
        assert_eq!((0..3).map(|_| r.next_slot()).collect::<Vec<_>>(), [1, 3, 0]);
    }

    fn script_completion(m: &MockPci, func: u32, slot_fb: u32, status: u32) {
        m.queue(Space::Mem, MMR + 0x30E0, &[2]);
        m.set(Space::Mem, MMR + 0x30E4, 3);
        m.set(Space::Mem, MMR + 0x302C, func);
        m.set(Space::Mem, MMR + 0x3048, slot_fb);
        m.set(Space::Mem, FB + slot_fb + 0x10, status);
    }

    #[test]
    fn send_cmd_polled() {
        let mut v = running(Completion::Poll);
        let slot_fb = v.ring.as_ref().unwrap().slot_fb(0);
        script_completion(v.bus(), 24, slot_fb, 1);
        v.bus().clear();
        let r = v.send_cmd(func::TSD_CONFIG, &[0xAA, 0, 0, 0]).unwrap();
        assert!(r.ok());
        assert_eq!(r, CmdResult { func: 24, slot: 0, slot_fb, status: 1 });
        assert_eq!(r.payload_fb(), slot_fb + 0x24);
        use Acc::*;
        assert_eq!(
            accesses(&v.bus().ops()),
            vec![
                Csr(0x194, FB + slot_fb),
                Csr(0x190, 0x23C0),
                Csr(0x180, 2),
                Csr(0x184, 1),
                Bulk(0x04, 0x23C0),
                W(0x3030, slot_fb),
                W(0x3020, 24),
                R(0x30E0),
                R(0x30E4),
                R(0x302C),
                W(0x30E0, 2),
                R(0x3048),
                FbR(slot_fb + 0x10),
            ]
        );
        let (ep, data) = v.bus().bulk().pop().unwrap();
        assert_eq!(ep, 0x04);
        assert_eq!(data, build_slot(0, slot_fb, 24, &[0xAA, 0, 0, 0]).unwrap());
        // Five header-word checks, two doorbells, four reads.
        assert_eq!(checks(&v.bus().ops()), 5 + 2 + 3);
        // The next command takes the next slot.
        let slot_fb1 = v.ring.as_ref().unwrap().slot_fb(1);
        script_completion(v.bus(), 9, slot_fb1, 0);
        let r = v.send_cmd(func::CLOSE_STREAM, &[]).unwrap();
        assert_eq!((r.slot, r.ok()), (1, false));
    }

    #[test]
    fn send_cmd_by_interrupt() {
        let mut v = running(Completion::Interrupt);
        let slot_fb = v.ring.as_ref().unwrap().slot_fb(0);
        script_completion(v.bus(), 8, slot_fb, 1);
        v.bus().set(Space::Mem, MMR + 0x414, 0x200);
        v.bus().queue(Space::Mem, MMR + 0x414, &[0x200, 0]);
        v.bus().set(Space::Mem, MMR + 0x418, 0x380);
        v.bus().push_interrupt(0x0100_0000);
        v.bus().clear();
        let r = v.send_cmd(func::OPEN_STREAM, &[0; 0x1198]).unwrap();
        assert!(r.ok());
        let acc = accesses(&v.bus().ops());
        use Acc::*;
        assert_eq!(
            &acc[7..],
            &[
                Csr(0x24, 0),
                Csr(0x2C, 0),
                Csr(0x24, 0),
                R(0x414),
                R(0x418),
                R(0x30E0),
                R(0x30E4),
                R(0x302C),
                W(0x30E0, 2),
                R(0x3048),
                FbR(slot_fb + 0x10),
                R(0x414),
                R(0x418),
            ]
        );
    }

    #[test]
    fn interrupt_timeout_falls_back_to_polling() {
        let mut v = running(Completion::Interrupt);
        let slot_fb = v.ring.as_ref().unwrap().slot_fb(0);
        script_completion(v.bus(), 21, slot_fb, 1);
        assert!(v.send_cmd(func::TS_CONTROL, &[1]).unwrap().ok());
    }

    #[test]
    fn command_timeout_and_wrong_function() {
        let mut v = running(Completion::Poll);
        v.opts.cmd_timeout = Duration::from_millis(5);
        // Completion for another function: not taken, bit stays.
        script_completion(v.bus(), 99, 0, 1);
        v.bus().set(Space::Mem, MMR + 0x30E0, 2);
        std::assert_matches!(v.send_cmd(10, &[]), Err(Error::Timeout));
        assert!(!writes(&v.bus().ops()).contains(&Acc::W(0x30E0, 2)));
        assert!(v.pending.is_none());
    }

    #[test]
    fn needs_heartbeat() {
        let mut v = running(Completion::Poll);
        v.alive = false;
        v.hb = HeartbeatState::default();
        v.bus().set(Space::Mem, FB + 0x10, 0);
        // First check sees a value: alive.
        let slot_fb = v.ring.as_ref().unwrap().slot_fb(0);
        script_completion(v.bus(), 10, slot_fb, 1);
        assert!(v.send_cmd(10, &[]).is_ok());
        assert!(v.is_alive());
    }

    #[test]
    fn heartbeat_states() {
        let mut v = running(Completion::Poll);
        v.hb = HeartbeatState::default();
        v.bus().set(Space::Mem, FB + 0x10, 7);
        v.bus().queue(Space::Mem, MMR + 0x6130, &[100, 200, 100 + 0x1C3]);
        assert_eq!(v.heartbeat().unwrap(), Heartbeat::Alive);
        assert_eq!(v.heartbeat().unwrap(), Heartbeat::Unchanged);
        assert_eq!(v.heartbeat().unwrap(), Heartbeat::Hung);
        v.bus().set(Space::Mem, FB + 0x10, 8);
        assert_eq!(v.heartbeat().unwrap(), Heartbeat::Alive);
    }

    fn script_message(m: &MockPci, id: u32, w3: u32) {
        m.queue(Space::Mem, MMR + 0x30E0, &[1]);
        m.set(Space::Mem, MMR + 0x30E4, 3);
        m.set(Space::Mem, MMR + 0x3028, id);
        m.set(Space::Mem, MMR + 0x3040, 0x20_0000);
        m.set(Space::Mem, FB + 0x20_000C, w3);
    }

    #[test]
    fn unhandled_message_is_refused_like_the_vendor() {
        let mut v = running(Completion::Poll);
        script_message(v.bus(), 0x0C, 0x0C);
        v.bus().clear();
        v.mips_isr().unwrap();
        use Acc::*;
        let mut want = vec![R(0x30E0), R(0x30E4), W(0x30E0, 1), R(0x3028), R(0x3040)];
        want.extend((0..9).map(|i| FbR(0x20_0000 + 4 * i)));
        want.extend([
            FbW(0x20_0010, 2),
            FbW(0x20_0010, 5),
            R(0x3040),
            R(0x3028),
            W(0x3038, 0x20_0000),
            W(0x3024, 0x0C),
        ]);
        assert_eq!(accesses(&v.bus().ops()), want);
    }

    #[test]
    fn handled_and_out_of_range_messages() {
        let mut v = running(Completion::Poll);
        let seen = Arc::new(Mutex::new(Vec::new()));
        let s2 = seen.clone();
        v.set_m2h_handler(Box::new(move |chip, msg| {
            s2.lock().unwrap().push(*msg);
            chip.fb_write(msg.words[5], 0x1234)?;
            Ok(true)
        }));
        script_message(v.bus(), 0x0E, 0x0E);
        v.bus().set(Space::Mem, FB + 0x20_0014, 0x30_0000);
        v.bus().clear();
        v.mips_isr().unwrap();
        assert_eq!(seen.lock().unwrap()[0].words[5], 0x30_0000);
        let w = writes(&v.bus().ops());
        assert_eq!(
            w,
            vec![Acc::W(0x30E0, 1), Acc::FbW(0x30_0000, 0x1234), Acc::W(0x3038, 0x20_0000), Acc::W(0x3024, 0x0E)]
        );

        // ID mismatch in word 3: refused without calling the handler.
        script_message(v.bus(), 0x0E, 0x0F);
        v.mips_isr().unwrap();
        assert_eq!(seen.lock().unwrap().len(), 1);

        // IDs from 0x1C up: status 5 only.
        script_message(v.bus(), 0x1C, 0x1C);
        v.bus().clear();
        v.mips_isr().unwrap();
        let w = writes(&v.bus().ops());
        assert_eq!(&w[1..2], &[Acc::FbW(0x20_0010, 5)]);
        assert_eq!(w.len(), 4);
    }

    #[test]
    fn otp_signature() {
        let mut v = running(Completion::Poll);
        v.bus().queue(Space::Mem, MMR + 0x3030, &[0x5DC40, 0]);
        v.bus().set(Space::Mem, FB + 0x5DC40, 0xDEAD);
        v.bus().clear();
        let sig = v.read_otp_signature().unwrap();
        assert_eq!(sig[0], 0xDEAD);
        let acc = accesses(&v.bus().ops());
        assert_eq!(&acc[..4], &[Acc::W(0x3030, 0x5DC40), Acc::W(0x3020, 27), Acc::R(0x3030), Acc::R(0x3030)]);
        assert_eq!(acc.len(), 4 + 28);
        assert_eq!(checks(&v.bus().ops()), 0);
    }

    #[test]
    fn top_level_isr_dispatch() {
        let mut v = running(Completion::Poll);
        v.bus().queue(Space::Mem, MMR + 0x414, &[0x380, 0]);
        v.bus().set(Space::Mem, MMR + 0x418, 0x380);
        v.bus().set(Space::Mem, MMR + 0x1094, 0x40);
        v.bus().set(Space::Mem, MMR + 0xC50, 0x4);
        v.bus().clear();
        v.service_interrupt().unwrap();
        use Acc::*;
        assert_eq!(
            accesses(&v.bus().ops()),
            vec![
                R(0x414),
                R(0x418),
                R(0x1094),
                R(0x1094),
                W(0x1094, 0x0030_00C0),
                R(0x1094),
                W(0x1094, 0x80),
                R(0xC50),
                W(0xC50, 0xD),
                R(0x30E0),
                R(0x30E4),
                R(0x414),
                R(0x418),
            ]
        );
    }
}
