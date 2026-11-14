//! Figures about the firmware's own timing, left in SDRAM where the host
//! can read them (through its window 0):
//!
//! | FB   | meaning                                                         |
//! |------|-----------------------------------------------------------------|
//! | 0xC8 | key RAM entries cleared at the start (128)                      |
//! | 0xCC | the time that took, in microseconds                             |
//! | 0xD0 | turns of the main loop in the last 5 ms                         |
//! | 0xD4 | the longest turn of the main loop so far, in microseconds       |
//! | 0xD8 | Program stream: ms from its open to the first ECM given to the card |
//! | 0xDC | ... to the first control words loaded                           |
//! | 0xE0 | ... to the first piece of the elementary streams sent           |
//! | 0xE4 | ... to the first piece of everything else sent                  |
//!
//! The Program stream's words read 0xFFFF_FFFF until the event has happened.

use crate::hw::{count, fb_w};

pub const ECM_SENT: u32 = 0xD8;
pub const KEYS: u32 = 0xDC;
pub const FIRST_ES: u32 = 0xE0;
pub const FIRST_REST: u32 = 0xE4;

/// The clearing of the key RAM at the start: `entries` written in `ticks`
/// of Count, of which there are `per_ms` in a millisecond.
pub fn key_ram_cleared(entries: u32, ticks: u32, per_ms: u32) {
    fb_w(0xC8, entries);
    fb_w(0xCC, ticks / (per_ms / 1000).max(1));
}

/// The main loop's turns.
pub struct Turns {
    per_us: u32,
    last: u32,
    turns: u32,
    longest: u32,
}

impl Turns {
    /// `per_ms`: Count ticks in a millisecond.
    pub fn new(per_ms: u32) -> Self {
        Self { per_us: (per_ms / 1000).max(1), last: count(), turns: 0, longest: 0 }
    }

    /// Once per turn of the main loop.
    pub fn turn(&mut self) {
        let now = count();
        self.longest = self.longest.max(now.wrapping_sub(self.last));
        self.last = now;
        self.turns += 1;
    }

    /// Once per 5 ms tick.
    pub fn publish(&mut self) {
        fb_w(0xD0, self.turns);
        fb_w(0xD4, self.longest / self.per_us);
        self.turns = 0;
    }
}

/// The times of a Program stream's first steps.
pub struct Since {
    start: u32,
    per_ms: u32,
    done: u8,
}

impl Since {
    pub fn now(per_ms: u32) -> Self {
        for off in [ECM_SENT, KEYS, FIRST_ES, FIRST_REST] {
            fb_w(off, u32::MAX);
        }
        Self { start: count(), per_ms: per_ms.max(1), done: 0 }
    }

    /// Record the first occurrence of the event that belongs at `off`.
    pub fn mark(&mut self, off: u32) {
        let bit = 1 << ((off - ECM_SENT) / 4);
        if self.done & bit == 0 {
            self.done |= bit;
            fb_w(off, count().wrapping_sub(self.start) / self.per_ms);
        }
    }
}
