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
const DONE: u32 = 2;

/// Where the firmware keeps the descriptor list (SDRAM, above the message
/// bodies).
const DESC_BASE: u32 = 0x0006_2800;
const DESC_MAX: u32 = 8;
const PIECE: u32 = 0x8000;

/// Control bit: the source is SDRAM (bit 25 would say so for the destination).
pub const SRC_LOCAL: u32 = 1 << 24;

pub struct Dma {
    busy: bool,
}

impl Dma {
    pub fn new() -> Self {
        mmr_w(ENABLE, 0);
        mmr_w(STATUS, 0xF);
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
            let d = DESC_BASE + 0x20 * k;
            let n = (len - k * PIECE).min(PIECE);
            let last = if k + 1 == pieces { 0xA000_0000 } else { 0 };
            fb_w(d, src + k * PIECE);
            fb_w(d + 0x04, 0);
            fb_w(d + 0x08, dst + k * PIECE);
            fb_w(d + 0x0C, 0);
            fb_w(d + 0x10, last | flags | n);
            fb_w(d + 0x14, 0);
            fb_w(d + 0x18, 0);
            fb_w(d + 0x1C, 0);
        }
        // The descriptors must have reached the SDRAM before the engine
        // reads them.
        let _ = fb_r(DESC_BASE + 0x20 * (pieces - 1) + 0x10);
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
        if !self.busy || mmr_r(STATUS) & DONE == 0 || mmr_r(BUSY) & DONE != 0 {
            return false;
        }
        mmr_w(STATUS, DONE);
        self.busy = false;
        true
    }
}
