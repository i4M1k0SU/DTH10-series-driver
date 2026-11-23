//! The chip's DMA engine: copies from SDRAM to a PCI address (the USB
//! bridge's endpoint FIFOs) or within SDRAM.
//!
//! A transfer is a list of 32-byte descriptors in SDRAM: source, 0,
//! destination, 0, control, 0, 0, 0. Control: bits 15..0 length (at most
//! 0x8000), bit 24 source is SDRAM, bit 25 destination is SDRAM, bit 29
//! report completion, bit 31 last descriptor. Writing the list's physical
//! address to 0xC04 starts it; 0xC34 bit 1 is set while it runs, 0xC58
//! bit 1 when it is done (write 1 to clear). A transfer to a FIFO the host
//! does not read simply stays busy.

use crate::hw::{count, fb_r, fb_w, mmr_r, mmr_rmw, mmr_w};

const LIST: u32 = 0xC04;
const BUSY: u32 = 0xC34;
const STATUS: u32 = 0xC58;
const ENABLE: u32 = 0xC5C;
/// STATUS and ENABLE: the transfer is done; BUSY: it runs.
const DONE: u32 = 2;
const RUNNING: u32 = 2;
/// Everything STATUS can report.
const STATUS_ALL: u32 = 0xF;

/// Where the firmware keeps the descriptor list (SDRAM, above the message
/// bodies).
const DESC_BASE: u32 = 0x0006_2800;
const DESC_MAX: u32 = 8;
const PIECE: u32 = 0x8000;
/// A descriptor: its size, and where the source, the destination and the
/// control word are in it.
const DESC_SIZE: u32 = 0x20;
const DESC_SRC: u32 = 0x00;
const DESC_DST: u32 = 0x08;
const DESC_CTRL: u32 = 0x10;
/// Control bits: the last descriptor, report completion.
const CTRL_LAST: u32 = 1 << 31;
const CTRL_REPORT: u32 = 1 << 29;

/// Control bit: the source is SDRAM (bit 25 would say so for the destination).
pub const SRC_LOCAL: u32 = 1 << 24;

pub struct Dma {
    busy: bool,
}

impl Dma {
    pub fn new() -> Self {
        mmr_w(ENABLE, 0);
        mmr_w(STATUS, STATUS_ALL);
        Self { busy: false }
    }

    pub fn idle(&self) -> bool {
        !self.busy
    }

    /// Start a transfer of `len` bytes (at most 0x40000); `flags` say which
    /// ends are SDRAM. The engine must be idle.
    pub fn start(&mut self, src: u32, dst: u32, len: u32, flags: u32) {
        let pieces = len.div_ceil(PIECE).clamp(1, DESC_MAX);
        for k in 0..pieces {
            let d = DESC_BASE + DESC_SIZE * k;
            let n = (len - k * PIECE).min(PIECE);
            let last = if k + 1 == pieces { CTRL_LAST | CTRL_REPORT } else { 0 };
            fb_w(d + DESC_SRC, src + k * PIECE);
            fb_w(d + 0x04, 0);
            fb_w(d + DESC_DST, dst + k * PIECE);
            fb_w(d + 0x0C, 0);
            fb_w(d + DESC_CTRL, last | flags | n);
            fb_w(d + 0x14, 0);
            fb_w(d + 0x18, 0);
            fb_w(d + 0x1C, 0);
        }
        // The descriptors must have reached the SDRAM before the engine
        // reads them.
        let _ = fb_r(DESC_BASE + DESC_SIZE * (pieces - 1) + DESC_CTRL);
        mmr_w(STATUS, DONE);
        mmr_rmw(ENABLE, |v| v | DONE);
        mmr_w(LIST, DESC_BASE);
        self.busy = true;
    }

    /// Let the running transfer finish, for at most `ticks` of CP0 Count.
    /// A stream is closed only after this, so that nothing of it reaches
    /// the host's endpoint after the host has emptied it.
    pub fn settle(&mut self, ticks: u32) {
        let start = count();
        while self.busy && count().wrapping_sub(start) < ticks {
            self.poll();
        }
    }

    /// True once when the running transfer has completed.
    pub fn poll(&mut self) -> bool {
        if !self.busy || mmr_r(STATUS) & DONE == 0 || mmr_r(BUSY) & RUNNING != 0 {
            return false;
        }
        mmr_w(STATUS, DONE);
        self.busy = false;
        true
    }
}
