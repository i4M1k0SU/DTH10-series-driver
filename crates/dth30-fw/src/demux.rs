//! The demultiplexer: PID filter entries, channels and their rings.
//!
//! The demultiplexer sorts the packets of an input port into channels
//! through a table of 32 PID filter entries. Each channel writes its
//! packets (188 bytes each) into a ring in SDRAM and keeps going whether
//! or not anything reads it. A packet goes to one channel only: that of
//! the matching entry with the lowest index. An "every PID" entry therefore
//! gets what no entry before it has taken.
//!
//! Registers (offsets in the chip's block):
//!
//! | reg             | meaning                                                        |
//! |-----------------|----------------------------------------------------------------|
//! | 0x6040/44/48    | PID filter: index / match word / destination                   |
//! | 0x6080/84       | descrambler setting of a channel: index / word                 |
//! | 0x6150          | channel run bits; writing 1 toggles a bit                      |
//! | 0x6400/04       | channel window: index (bit 16 = page window) / value           |
//! | 0x6408 + 16 n   | channel n's write position: address of the last byte written   |
//! | 0x6580 + 4(n/2) | channel event set-up, 9 bits per channel                       |
//! | 0x65E8/EC       | channel set-up: index / word                                   |
//! | 0x67D4, 0x67B0  | channel events, write 1 to clear                               |
//! | 0x67DC, 0x67B8  | channel event enables towards the CPU (left off: polling)      |
//!
//! Channels and filter entries are handed out statically:
//!
//! | channel | filter entries | use                                               |
//! |---------|----------------|---------------------------------------------------|
//! | 0       | 0..=25         | the Program stream's elementary streams and PCR   |
//! | 14      | 31 (every PID) | everything else: the Raw stream, and the rest of the Program stream |
//!
//! Several readers can follow one ring, each with its own [`Ring`].

use crate::hw::{mmr_r, mmr_rmw, mmr_w};

pub const PACKET: u32 = 188;

const SEMAPHORE: u32 = 0x440;
const SEM_DEMUX: u32 = 0x10;

/// Filter match word: the entry is valid; the input port goes to bits 14..13.
pub const FILTER_VALID: u32 = 0x8000_0000;
/// Filter destination: take every PID.
pub const FILTER_ALL: u32 = 0x100;

/// The demultiplexer's tables are changed under hardware semaphore 16.
pub fn lock() {
    for _ in 0..1000 {
        mmr_w(SEMAPHORE, SEM_DEMUX);
        if mmr_r(SEMAPHORE) == 0x8000_0000 | SEM_DEMUX {
            return;
        }
    }
}

pub fn unlock() {
    if mmr_r(SEMAPHORE) == 0x8000_0000 | SEM_DEMUX {
        mmr_w(SEMAPHORE, SEM_DEMUX);
    }
}

/// Write an index register until it reads back.
pub fn select(reg: u32, index: u32, mask: u32) {
    for _ in 0..1000 {
        mmr_w(reg, index);
        if mmr_r(reg) & mask == index & mask {
            return;
        }
    }
}

pub fn set_filter(index: u32, word: u32, dest: u32) {
    select(0x6040, index, 0x7F);
    mmr_w(0x6044, word);
    mmr_w(0x6048, dest);
}

/// Read-modify-write of a channel's descrambler word.
pub fn descrambler(channel: u32, f: impl FnOnce(u32) -> u32) {
    mmr_w(0x6080, channel);
    mmr_w(0x6080, channel);
    mmr_rmw(0x6084, f);
}

fn write_position(channel: u32) -> u32 {
    if channel < 4 { 0x6420 + 4 * channel } else { 0x6408 + 16 * channel }
}

pub fn stop_channel(channel: u32) {
    let bit = 1 << channel;
    if mmr_r(0x6150) & bit != 0 {
        mmr_w(0x6150, bit);
    }
    mmr_w(0x67D4, bit);
    mmr_w(0x67B0, bit);
}

/// Point a stopped channel at its ring (`len` a multiple of 188) and start
/// it.
pub fn start_channel(channel: u32, base: u32, len: u32) {
    let bit = 1 << channel;
    stop_channel(channel);
    select(0x65E8, channel, 0x1F);
    mmr_w(0x65EC, 0x7_0015);
    select(0x6400, 0x1_0000 | channel, 0x1_001F);
    mmr_w(0x6404, base >> 12 | (base + len - 1) >> 12 << 16);
    mmr_w(write_position(channel), base);
    select(0x6400, channel, 0x1_001F);
    mmr_w(0x6404, base + len - 1);
    // Events at packet ends, one per 2 KB written. Nothing waits for them
    // (the rings are polled), but with this setting the write position
    // always reads at a packet boundary.
    let shift = if channel & 1 != 0 { 16 } else { 0 };
    mmr_rmw(0x6580 + 4 * (channel >> 1), |v| (v & !(0x1F7 << shift)) | 0x006 << shift);
    mmr_w(0x67D4, bit);
    mmr_w(0x67B0, bit);
    mmr_w(0x6150, bit);
}

/// A channel's ring as the firmware follows it.
pub struct Ring {
    channel: u32,
    base: u32,
    len: u32,
    /// Next byte to send (physical).
    pub rp: u32,
    /// The position after the last byte the channel had written when last
    /// looked at.
    seen: u32,
    /// Bytes written and not yet sent.
    pub fill: u32,
    /// Bytes still to drop after the start.
    skip: u32,
    /// After an overrun the reader restarts on a multiple of this.
    align: u32,
    /// What the write position read when the channel had just been started
    /// (0, or where its last run ended): readings count once they differ.
    idle: Option<u32>,
    pub overruns: u32,
}

impl Ring {
    /// The reader of a channel that has just been started. Such a channel
    /// first delivers some packets it had kept from its last run: `skip`
    /// bytes are dropped (a multiple of `align`).
    pub fn new(channel: u32, base: u32, len: u32, align: u32, skip: u32) -> Self {
        let idle = Some(mmr_r(write_position(channel)));
        Self { channel, base, len, rp: base + skip, seen: base, fill: 0, skip, align, idle, overruns: 0 }
    }

    /// A reader that joins a channel that may already run: it starts
    /// `skip` bytes (a multiple of `align`) after the next multiple of
    /// `align` from the channel's current position.
    pub fn attach(channel: u32, base: u32, len: u32, align: u32, skip: u32) -> Self {
        let last = mmr_r(write_position(channel));
        if !(base..base + len).contains(&last) {
            return Self::new(channel, base, len, align, skip);
        }
        let now = (last + 1 - base) % len;
        let start = now.div_ceil(align) * align + skip;
        Self {
            channel,
            base,
            len,
            rp: base + start % len,
            seen: base + now,
            fill: 0,
            skip: start - now,
            align,
            idle: None,
            overruns: 0,
        }
    }

    /// Account for what the channel has written since the last call.
    pub fn follow(&mut self) {
        let last = mmr_r(write_position(self.channel));
        if Some(last) == self.idle || !(self.base..self.base + self.len).contains(&last) {
            // Nothing written yet since the start.
            return;
        }
        self.idle = None;
        let mut now = last + 1;
        if now == self.base + self.len {
            now = self.base;
        }
        let advance = if now >= self.seen { now - self.seen } else { now + self.len - self.seen };
        self.seen = now;
        let dropped = self.skip.min(advance);
        self.skip -= dropped;
        self.fill += advance - dropped;
        if self.fill > self.len - self.len / 8 {
            // The channel is about to lap the reader: start again near the
            // position being written.
            let offset = (now - self.base) / self.align * self.align;
            self.rp = self.base + offset;
            self.fill = now - self.rp;
            self.overruns += 1;
        }
    }

    /// The address `n` bytes on from `rp`.
    pub fn ahead(&self, n: u32) -> u32 {
        self.base + (self.rp - self.base + n) % self.len
    }

    /// The address `n` bytes back from `rp` (`n` at most the ring's length).
    pub fn behind(&self, n: u32) -> u32 {
        self.base + (self.rp - self.base + self.len - n) % self.len
    }

    /// The bytes ready at `rp` without wrapping.
    pub fn ready(&self) -> u32 {
        self.fill.min(self.base + self.len - self.rp)
    }

    pub fn consume(&mut self, n: u32) {
        self.fill -= n;
        self.rp += n;
        if self.rp >= self.base + self.len {
            self.rp -= self.len;
        }
    }
}
