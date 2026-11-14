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

use crate::hw::{Clock, count, fb_r, fb_w, mmr_r, mmr_w};

/// Requests from the host.
const H2M_STATUS: u32 = 0x30E8;
const DOORBELL: u32 = 1;
const MSG_ACK: u32 = 2;

/// Start-up message: the host must set its areas up in the pool.
const MSG_ALLOC_FB: u32 = 5;
/// Start-up message: the firmware is ready for commands.
pub const MSG_READY: u32 = 0;

/// Results of a command (slot +0x10).
pub const OK: u32 = 1;
pub const FAILED: u32 = 2;
pub const UNSUPPORTED: u32 = 5;

/// The command that bypasses the slot: 0x3030 is the destination instead.
pub const CMD_SIGNATURE: u32 = 27;

/// The pool handed to the host starts here.
const POOL_START: u32 = 0x1000;

/// Message bodies live above the host's pool, inside the first megabyte
/// (the host reads them through its window 0).
const MSG_BASE: u32 = 0x0006_1000;
const MSG_STRIDE: u32 = 0x280;
const MSG_COUNT: usize = 8;
/// Offset of the descriptor in a message body.
const MSG_DESC: u32 = 0x24;

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
        let tick = clock.ticks(5);
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
        fb_w(0xB0, 1);
        for off in [0x30, 0x24, 0x38, 0x3C, 0x34, 0x2C, 0x28] {
            fb_w(off, 0);
        }
        fb_w(0x20, 0x1FFF_1004);
        while fb_r(0x28) == 0 {}
        fb_w(0xB0, 2);

        let mem = fb_r(0x48) & 0x1FFF_FFFF;
        let pool_end = if mem < 0x0100_0000 {
            0x1_0000
        } else if fb_r(0x4C) == 0 {
            0x2_0000
        } else {
            0x6_1000
        };
        mmr_w(H2M_STATUS, DOORBELL | MSG_ACK);
        mmr_w(0x3040, POOL_START);
        mmr_w(0x3044, pool_end - POOL_START);
        mmr_w(0x3028, MSG_ALLOC_FB);
        while mmr_r(H2M_STATUS) & MSG_ACK == 0 {}
        mmr_w(H2M_STATUS, MSG_ACK);
        // Drop a stale watchdog flag on the host's side; the watchdog itself
        // (0x3084) stays off.
        mmr_w(0x30E0, 0x8000_0000);
        fb_w(0xB0, 5);
        fb_w(0x14, 0);
    }

    /// Tell the host the firmware is ready. Doorbells rung before this are
    /// dropped.
    pub fn ready(&mut self) {
        mmr_w(H2M_STATUS, mmr_r(H2M_STATUS));
        fb_w(0xB0, 10);
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
        fb_w(0x10, self.heartbeat);
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
        let id = mmr_r(0x3020);
        let arg = mmr_r(0x3030) & 0x1FFF_FFFF;
        if id == CMD_SIGNATURE {
            return Some(Command { id, payload: arg, slot: 0 });
        }
        if arg == 0 || arg >= 0x0010_0000 {
            return None;
        }
        Some(Command { id: fb_r(arg + 0x0C), payload: fb_r(arg + 0x14) & 0x1FFF_FFFF, slot: arg })
    }

    /// Finish a command with `result`.
    pub fn complete(&mut self, c: &Command, result: u32) {
        if c.is_signature() {
            // No completion: the host waits for 0x3030 to read 0.
            mmr_w(0x3030, 0);
            return;
        }
        fb_w(c.slot + 0x10, result);
        self.beat();
        mmr_w(0x3048, c.slot);
        mmr_w(0x302C, c.id);
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
        fb_w(body + 0x08, 2);
        fb_w(body + 0x0C, id);
        fb_w(body + 0x14, body + MSG_DESC);
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
        mmr_w(0x3038, 0xFFFF_FFFF);
        mmr_w(0x3040, body);
        mmr_w(0x3028, fb_r(body + 0x0C));
    }

    /// The host acknowledged: 0x3038 = the body's address, 0x3024 = the ID.
    /// An acknowledge that does not match leaves the message outstanding.
    fn acknowledged(&mut self) {
        let id = mmr_r(0x3024);
        let addr = mmr_r(0x3038);
        mmr_w(H2M_STATUS, MSG_ACK);
        let Some(m) = self.outstanding else { return };
        let body = Self::body(m);
        if addr == body && id == fb_r(body + 0x0C) {
            self.used[m as usize] = false;
            self.outstanding = None;
            self.pump();
        }
    }
}
