//! The host interface: start-up handshake, commands, messages, heartbeat.
//!
//! Registers (offsets in the chip's block):
//!
//! | reg    | written by | meaning                                              |
//! |--------|------------|------------------------------------------------------|
//! | 0x3020 | host       | command doorbell (sets 0x30E8 bit 0); 27 = signature |
//! | 0x3030 | host       | physical address of the command slot                 |
//! | 0x3024 | host       | message acknowledge (sets 0x30E8 bit 1): the ID      |
//! | 0x3038 | host       | message acknowledge: the body's address              |
//! | 0x3028 | firmware   | message ID (sets the host's 0x30E0 bit 0)            |
//! | 0x3040 | firmware   | message body's physical address                      |
//! | 0x3044 | firmware   | size of the host's pool (start-up only)              |
//! | 0x302C | firmware   | completed command's ID (sets the host's 0x30E0 bit 1)|
//! | 0x3048 | firmware   | completed command's slot                             |
//! | 0x30E8 | —          | requests from the host, write 1 to clear             |
//!
//! A command slot: +0x0C command ID, +0x10 result (written here), +0x14
//! physical address of the payload. One command and one message are
//! outstanding at a time.

use crate::hw::{Clock, MMR_BASE, PHYS_MASK, count, fb_r, fb_w, mmr_r, mmr_w};

/// Requests from the host.
const H2M_STATUS: u32 = 0x30E8;
const DOORBELL: u32 = 1;
const MSG_ACK: u32 = 2;

/// The other registers of the table above.
const H2M_FUNC: u32 = 0x3020;
const H2M_ACK_ID: u32 = 0x3024;
const M2H_ID: u32 = 0x3028;
const M2H_DONE_FUNC: u32 = 0x302C;
const H2M_SLOT: u32 = 0x3030;
const H2M_ACK_ADDR: u32 = 0x3038;
const M2H_ADDR: u32 = 0x3040;
const M2H_POOL_SIZE: u32 = 0x3044;
const M2H_DONE_SLOT: u32 = 0x3048;
/// Interrupts towards the host (write 1 to clear); bit 31 is the watchdog's.
const M2H_INT_STATUS: u32 = 0x30E0;
const M2H_WATCHDOG: u32 = 1 << 31;

/// The parameter block (FB): where the MIPS sees the registers (written
/// here), the host's answer to it, the SDRAM size and the FLAG word (from the
/// host).
const FB_MMR_WINDOW: u32 = 0x20;
const FB_HOST_ANSWER: u32 = 0x28;
const FB_MEM_SIZE: u32 = 0x48;
const FB_FLAG: u32 = 0x4C;
/// The register window as the host is to describe it: 0x1FFF in the high
/// half, the high bits of the MIPS's address of the registers in the low.
const MMR_WINDOW: u32 = 0x1FFF_0000 | (MMR_BASE as u32 >> 16 & 0x1FFF);
/// The heartbeat (FB), and its period in ms.
const FB_HEARTBEAT: u32 = 0x10;
const HEARTBEAT_MS: u32 = 5;
/// How far the start-up has got (FB): the step numbers of docs/27 chapter 2
/// §1.2.
const FB_PROGRESS: u32 = 0xB0;

/// Start-up message: the host must set its areas up in the pool.
const MSG_ALLOC_FB: u32 = 5;
/// Start-up message: the firmware is ready for commands.
pub const MSG_READY: u32 = 0;

/// Results of a command (slot +0x10).
pub const OK: u32 = 1;
pub const FAILED: u32 = 2;
pub const UNSUPPORTED: u32 = 5;

/// The functions (slot +0x0C) this firmware handles: ping, open a stream,
/// close it, flush it before the close, a request to an object, set the TS
/// input up, open and close the CAS object.
pub const CMD_PING: u32 = 0;
pub const CMD_OPEN: u32 = 8;
pub const CMD_CLOSE: u32 = 9;
pub const CMD_FLUSH: u32 = 10;
pub const CMD_INPUT: u32 = 11;
pub const CMD_TS_CONFIG: u32 = 24;
pub const CMD_CAS_OPEN: u32 = 25;
pub const CMD_CAS_CLOSE: u32 = 26;
/// The command that bypasses the slot: 0x3030 is the destination instead.
pub const CMD_SIGNATURE: u32 = 27;

/// A command slot: the function, the result, the payload's address.
const SLOT_FUNC: u32 = 0x0C;
const SLOT_RESULT: u32 = 0x10;
const SLOT_PAYLOAD: u32 = 0x14;
/// Slots lie in the first megabyte (the host's window 0).
const SLOT_END: u32 = 0x0010_0000;

/// The pool handed to the host starts here.
const POOL_START: u32 = 0x1000;

/// Message bodies live above the host's pool, inside the first megabyte
/// (the host reads them through its window 0).
const MSG_BASE: u32 = 0x0006_1000;
const MSG_STRIDE: u32 = 0x280;
const MSG_COUNT: usize = 8;
/// Offset of the descriptor in a message body.
const MSG_DESC: u32 = 0x24;
/// Words of a message body: who sent it (2 = the firmware), the message ID,
/// the descriptor's physical address.
const BODY_SENDER: u32 = 0x08;
const SENDER_FIRMWARE: u32 = 2;
const BODY_ID: u32 = 0x0C;
const BODY_DESC_ADDR: u32 = 0x14;

/// A command taken from the host.
pub struct Command {
    pub id: u32,
    /// Physical address of the payload (0 when the host sent none).
    pub payload: u32,
    slot: u32,
}

impl Command {
    /// The signature command, which has no slot: `payload` is where the
    /// signature goes.
    pub fn is_signature(&self) -> bool {
        self.slot == 0
    }
}

pub struct Host {
    heartbeat: u32,
    tick: u32,
    next_tick: u32,
    /// Bodies in use.
    used: [bool; MSG_COUNT],
    /// Bodies waiting to be sent, oldest first.
    queue: [u8; MSG_COUNT],
    queued: usize,
    /// The body the host has not acknowledged yet.
    outstanding: Option<u8>,
}

impl Host {
    pub fn new(clock: &Clock) -> Self {
        let tick = clock.ticks(HEARTBEAT_MS);
        Self {
            heartbeat: 0,
            tick,
            next_tick: count().wrapping_add(tick),
            used: [false; MSG_COUNT],
            queue: [0; MSG_COUNT],
            queued: 0,
            outstanding: None,
        }
    }

    /// The start-up handshake up to the host's answer to ALLOC_FB.
    ///
    /// FB 0x20 tells the host where the MIPS sees the registers; the host
    /// answers in FB 0x28 (any non-zero value). Then the host is given its
    /// pool and answers through 0x3024. FB 0xB0 shows how far this got.
    pub fn handshake(&mut self) {
        fb_w(FB_PROGRESS, 1);
        // In the order the vendor's firmware clears them.
        for off in [0x30, 0x24, 0x38, 0x3C, 0x34, 0x2C, 0x28] {
            fb_w(off, 0);
        }
        fb_w(FB_MMR_WINDOW, MMR_WINDOW);
        while fb_r(FB_HOST_ANSWER) == 0 {}
        fb_w(FB_PROGRESS, 2);

        // The end of the pool by the SDRAM size and FLAG (docs/27 chapter 2
        // §1.5).
        let mem = fb_r(FB_MEM_SIZE) & PHYS_MASK;
        let pool_end = if mem < 0x0100_0000 {
            0x1_0000
        } else if fb_r(FB_FLAG) == 0 {
            0x2_0000
        } else {
            0x6_1000
        };
        mmr_w(H2M_STATUS, DOORBELL | MSG_ACK);
        mmr_w(M2H_ADDR, POOL_START);
        mmr_w(M2H_POOL_SIZE, pool_end - POOL_START);
        mmr_w(M2H_ID, MSG_ALLOC_FB);
        while mmr_r(H2M_STATUS) & MSG_ACK == 0 {}
        mmr_w(H2M_STATUS, MSG_ACK);
        // Drop a stale watchdog flag on the host's side; the watchdog itself
        // (0x3084) stays off.
        mmr_w(M2H_INT_STATUS, M2H_WATCHDOG);
        fb_w(FB_PROGRESS, 5);
        // Reads 0 from now on, as with the vendor's firmware.
        fb_w(0x14, 0);
    }

    /// Tell the host the firmware is ready. Doorbells rung before this are
    /// dropped.
    pub fn ready(&mut self) {
        mmr_w(H2M_STATUS, mmr_r(H2M_STATUS));
        fb_w(FB_PROGRESS, 10);
        if let Some(m) = self.message(MSG_READY) {
            self.send(m);
        }
    }

    /// The 5 ms heartbeat in FB 0x10.
    /// True when a tick has passed.
    pub fn poll_tick(&mut self) -> bool {
        let due = count().wrapping_sub(self.next_tick) < 0x8000_0000;
        if due {
            self.next_tick = self.next_tick.wrapping_add(self.tick);
            self.beat();
        }
        due
    }

    fn beat(&mut self) {
        self.heartbeat = self.heartbeat.wrapping_add(1);
        fb_w(FB_HEARTBEAT, self.heartbeat);
    }

    /// Handle a message acknowledge and return the next command, if the
    /// host rang.
    pub fn poll(&mut self) -> Option<Command> {
        let s = mmr_r(H2M_STATUS);
        if s & MSG_ACK != 0 {
            self.acknowledged();
        }
        if s & DOORBELL == 0 {
            return None;
        }
        mmr_w(H2M_STATUS, DOORBELL);
        let id = mmr_r(H2M_FUNC);
        let arg = mmr_r(H2M_SLOT) & PHYS_MASK;
        if id == CMD_SIGNATURE {
            return Some(Command { id, payload: arg, slot: 0 });
        }
        if arg == 0 || arg >= SLOT_END {
            return None;
        }
        Some(Command { id: fb_r(arg + SLOT_FUNC), payload: fb_r(arg + SLOT_PAYLOAD) & PHYS_MASK, slot: arg })
    }

    /// Finish a command with `result`.
    pub fn complete(&mut self, c: &Command, result: u32) {
        if c.is_signature() {
            // No completion: the host waits for 0x3030 to read 0.
            mmr_w(H2M_SLOT, 0);
            return;
        }
        fb_w(c.slot + SLOT_RESULT, result);
        self.beat();
        mmr_w(M2H_DONE_SLOT, c.slot);
        mmr_w(M2H_DONE_FUNC, c.id);
    }

    /// Take a message body for message `id`; `None` when all are in use.
    /// Returns the body's index for [`Host::descriptor`] and [`Host::send`].
    pub fn message(&mut self, id: u32) -> Option<u8> {
        let i = self.used.iter().position(|u| !u)?;
        self.used[i] = true;
        let body = Self::body(i as u8);
        for off in (0..MSG_STRIDE).step_by(4) {
            fb_w(body + off, 0);
        }
        fb_w(body + BODY_SENDER, SENDER_FIRMWARE);
        fb_w(body + BODY_ID, id);
        fb_w(body + BODY_DESC_ADDR, body + MSG_DESC);
        Some(i as u8)
    }

    fn body(i: u8) -> u32 {
        MSG_BASE + MSG_STRIDE * u32::from(i)
    }

    /// Physical address of a message's descriptor (0x25C bytes, zeroed).
    pub fn descriptor(m: u8) -> u32 {
        Self::body(m) + MSG_DESC
    }

    /// Queue a message; it goes out when the earlier ones are acknowledged.
    pub fn send(&mut self, m: u8) {
        self.queue[self.queued] = m;
        self.queued += 1;
        self.pump();
    }

    fn pump(&mut self) {
        if self.outstanding.is_some() || self.queued == 0 {
            return;
        }
        let m = self.queue[0];
        self.queue.copy_within(1..self.queued, 0);
        self.queued -= 1;
        let body = Self::body(m);
        self.outstanding = Some(m);
        self.beat();
        mmr_w(H2M_ACK_ADDR, 0xFFFF_FFFF);
        mmr_w(M2H_ADDR, body);
        mmr_w(M2H_ID, fb_r(body + BODY_ID));
    }

    /// The host acknowledged: 0x3038 = the body's address, 0x3024 = the ID.
    /// An acknowledge that does not match leaves the message outstanding.
    fn acknowledged(&mut self) {
        let id = mmr_r(H2M_ACK_ID);
        let addr = mmr_r(H2M_ACK_ADDR);
        mmr_w(H2M_STATUS, MSG_ACK);
        let Some(m) = self.outstanding else { return };
        let body = Self::body(m);
        if addr == body && id == fb_r(body + BODY_ID) {
            self.used[m as usize] = false;
            self.outstanding = None;
            self.pump();
        }
    }
}
