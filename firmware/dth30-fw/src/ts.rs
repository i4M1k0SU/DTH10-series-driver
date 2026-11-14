//! TS input, the demultiplexer and the Raw stream.
//!
//! The demultiplexer sorts the packets of an input port into channels
//! through a table of 32 PID filter entries. Each channel writes its
//! packets (188 bytes each) into a ring in SDRAM and keeps going whether
//! or not anything reads it. The entries are tried in index order: an
//! "every PID" entry takes a copy and the search goes on, an entry for one
//! PID takes the packet and ends the search.
//!
//! The Raw stream is channel 14 with an "every PID" entry (index 31). The
//! firmware sends the ring to the host's endpoint in chunks of the size the
//! host asked for, straight from the ring (no copy).
//!
//! Registers used (offsets in the chip's block):
//!
//! | reg             | meaning                                                        |
//! |-----------------|----------------------------------------------------------------|
//! | 0x6000          | input ports: bit i = 204-byte packets, bit 24+i = enable, ...  |
//! | 0x6008          | bit 8+i = valid is active high, bit 24+i = falling clock edge  |
//! | 0x600C          | bit 8+i = drop the null packets of port i                      |
//! | 0x6030          | 0x503 (needed for reception)                                   |
//! | 0x6040/44/48    | PID filter: index / match word / destination                   |
//! | 0x6080/84       | descrambler setting of a channel: index / word                 |
//! | 0x6150          | channel run bits; writing 1 toggles a bit                      |
//! | 0x6400/04       | channel window: index (bit 16 = page window) / value           |
//! | 0x6408 + 16 n   | channel n's write position: address of the last byte written   |
//! | 0x6580 + 4(n/2) | channel event set-up, 9 bits per channel                       |
//! | 0x65E8/EC       | channel set-up: index / word                                   |
//! | 0x67D4, 0x67B0  | channel events, write 1 to clear                               |
//! | 0x67DC, 0x67B8  | channel event enables towards the CPU (left off: polling)      |

use crate::dma::{Dma, SRC_LOCAL};
use crate::host::{FAILED, OK};
use crate::hw::{fb_r, fb_w, mmr_r, mmr_rmw, mmr_w};

/// The Raw stream's channel and filter entry.
const RAW_CHANNEL: u32 = 14;
const RAW_FILTER: u32 = 31;
/// The Raw ring: 40 chunks of 0xBC00 bytes (= 10240 packets), outside the
/// image and the areas shared with the host.
const RAW_RING: u32 = 0x0080_0000;
const RAW_LEN: u32 = 0x001D_6000;
/// The only chunk size supported (256 packets, the host's transfer size).
const CHUNK: u32 = 0xBC00;
/// The handle returned for the Raw stream (the host passes it back).
const RAW_HANDLE: u32 = 0xA006_2A00;
/// Two pages that idle channels point at.
const SINK: u32 = 0x0006_4000;

const SEMAPHORE: u32 = 0x440;
const SEM_DEMUX: u32 = 0x10;

/// The stream types of an open (payload +0x58).
pub const KIND_RAW: u32 = 1;

struct Raw {
    /// PCI address of the host's endpoint FIFO.
    dest: u32,
    /// Next byte to send (physical).
    rp: u32,
    /// The position after the last byte the channel had written when last
    /// looked at.
    seen: u32,
    /// Bytes written and not yet sent.
    fill: u32,
    /// Bytes still to drop after the open.
    skip: u32,
}

pub struct Ts {
    raw: Option<Raw>,
    /// Times the Raw ring was overrun because the host did not read.
    pub overruns: u32,
}

/// The demultiplexer's tables are changed under hardware semaphore 16.
fn lock() {
    for _ in 0..1000 {
        mmr_w(SEMAPHORE, SEM_DEMUX);
        if mmr_r(SEMAPHORE) == 0x8000_0000 | SEM_DEMUX {
            return;
        }
    }
}

fn unlock() {
    if mmr_r(SEMAPHORE) == 0x8000_0000 | SEM_DEMUX {
        mmr_w(SEMAPHORE, SEM_DEMUX);
    }
}

/// Write an index register until it reads back.
fn select(reg: u32, index: u32, mask: u32) {
    for _ in 0..1000 {
        mmr_w(reg, index);
        if mmr_r(reg) & mask == index & mask {
            return;
        }
    }
}

fn set_filter(index: u32, word: u32, dest: u32) {
    select(0x6040, index, 0x7F);
    mmr_w(0x6044, word);
    mmr_w(0x6048, dest);
}

fn write_position(channel: u32) -> u32 {
    if channel < 4 { 0x6420 + 4 * channel } else { 0x6408 + 16 * channel }
}

fn stop_channel(channel: u32) {
    let bit = 1 << channel;
    if mmr_r(0x6150) & bit != 0 {
        mmr_w(0x6150, bit);
    }
}

impl Ts {
    /// Bring the demultiplexer to rest: no channel runs, no filter entry
    /// is valid, no event reaches the CPU.
    pub fn new() -> Self {
        mmr_w(0x67DC, 0);
        mmr_w(0x67B8, 0);
        mmr_w(0x6150, mmr_r(0x6150));
        for i in 0..32 {
            set_filter(i, 0, 0);
        }
        Self { raw: None, overruns: 0 }
    }

    /// Command 24: set the TS input up from the words the host left in
    /// FB 0x50..0x8C. Refused while a stream is open.
    ///
    /// | FB   | meaning (bit i = input port i)                          |
    /// |------|---------------------------------------------------------|
    /// | 0x50 | 1 = the words below are valid                           |
    /// | 0x54 | sync on channel -> 0x6000 bit 16+i                      |
    /// | 0x58 | port enable -> 0x6000 bit 24+i                          |
    /// | 0x5C | valid is active high -> 0x6008 bit 8+i                  |
    /// | 0x60 | falling clock edge -> 0x6008 bit 24+i                   |
    /// | 0x64 | serial input (ports 4..7: a clear bit sets 0x6000 4+i)  |
    /// | 0x68 | 204-byte packets -> 0x6000 bit i                        |
    /// | 0x6C | keep null packets (ports 0, 1; clear -> 0x600C bit 8+i) |
    /// | 0x70 | 1 = the output set-up below is valid                    |
    /// | 0x74.. | parallel output: data mode, clock invert, increment disable, valid high, clock select, null packets, spacing |
    pub fn configure(&mut self) -> u32 {
        if self.raw.is_some() {
            return FAILED;
        }
        let w = |off: u32| fb_r(off);
        let valid = w(0x50) == 1;

        // The parallel output's clock, with its block held in reset.
        mmr_rmw(0x000, |v| v | 0x0C);
        mmr_rmw(0x058, |v| v & !8);
        mmr_rmw(0x050, |v| v | 0xF00);
        mmr_rmw(0x058, |v| v | 8);
        mmr_rmw(0x000, |v| v & !0x0C);

        let (ports, edges) = if valid {
            let (sync, enable, high, falling, serial, size204) = (w(0x54), w(0x58), w(0x5C), w(0x60), w(0x64), w(0x68));
            let mut a = (size204 & 0xFF) | (sync & 0xFF) << 16 | (enable & 0xFF) << 24;
            for i in 4..8 {
                if serial & 1 << i == 0 {
                    a |= 1 << (i + 4);
                }
            }
            (a, (high & 0xFF) << 8 | (falling & 0xFF) << 24)
        } else {
            (0x0300_0100, 0xFF00_0000)
        };
        mmr_w(0x6000, ports);
        mmr_w(0x6008, 0x0032_0000 | edges);
        mmr_w(0x6030, 0x503);
        mmr_w(0x6168, 0);
        mmr_w(0x67B8, 0);
        mmr_w(0x67DC, 0);
        mmr_w(0x6150, mmr_r(0x6150));
        mmr_w(0x67D4, mmr_r(0x67D4));
        mmr_w(0x67B0, mmr_r(0x67B0));

        let sink = SINK >> 12 | ((SINK >> 12) + 1) << 16;
        for i in 0..16 {
            lock();
            set_filter(i, 0, 0);
            mmr_w(0x6080, i);
            mmr_w(0x6084, 0);
            select(0x6400, 0x1_0000 | i, 0x1_001F);
            mmr_w(0x6404, sink);
            unlock();
        }
        for i in 16..32 {
            set_filter(i, 0, 0);
        }
        for k in 0..8 {
            mmr_w(0x6580 + 4 * k, 0);
        }

        if valid {
            let keep = w(0x6C);
            mmr_rmw(0x600C, |v| (v & !0x300) | (!keep & 3) << 8);
        } else {
            mmr_rmw(0x600C, |v| v | 0x300);
        }
        mmr_w(0x6154, 0x3_3201);
        if w(0x70) == 1 {
            let (mode, clock) = (w(0x74), w(0x84));
            let spacing = match (mode, clock, w(0x8C)) {
                (0, _, _) => 0x5DF,
                (_, 0, 1) => 0x2F0,
                (_, 0 | 4, 2) => 0x5E0,
                (_, 0 | 4, 3) => 0x7FC,
                (_, 1, 1) => 0x177,
                (_, 1, 2) => 0x2EF,
                (_, 1, 3) => 0x5DF,
                (_, 4, 1) => 0x170,
                _ => 0,
            };
            let bit = |off: u32, n: u32| u32::from(w(off) != 0) << n;
            let v = bit(0x74, 0) | bit(0x78, 2) | bit(0x7C, 4) | bit(0x80, 6) | bit(0x88, 12);
            mmr_w(0x6090, v | if clock < 5 { clock << 7 } else { 0 } | spacing << 16);
        } else {
            mmr_rmw(0x6090, |v| (v | 1) & 0xF800_FFFF);
        }
        mmr_w(0x63FC, 0xBD);
        mmr_rmw(0x65E0, |v| v | 0x1_0000);
        OK
    }

    /// Command 8, kind 1: open the Raw stream.
    ///
    /// Payload: +0x08 input port, +0x18 the host's own handle (replaced by
    /// the firmware's), +0x60 flags (bit 20: TS to the host), +0x7C PID
    /// (0xFFFF = every PID), +0x314 number of host buffers, +0x318 chunk
    /// size, +0x31C.. their PCI addresses.
    pub fn open_raw(&mut self, p: u32) -> u32 {
        let (port, flags, pid) = (fb_r(p + 0x08), fb_r(p + 0x60), fb_r(p + 0x7C));
        let (chunk, dest) = (fb_r(p + 0x318), fb_r(p + 0x31C));
        if self.raw.is_some() || flags & 0x10_0000 == 0 || pid != 0xFFFF || chunk != CHUNK || port > 3 {
            return FAILED;
        }
        let n = RAW_CHANNEL;
        let bit = 1 << n;
        stop_channel(n);
        lock();
        set_filter(RAW_FILTER, 0x8000_0000 | port << 13, n | 0x100);
        mmr_w(0x6080, n);
        mmr_w(0x6080, n);
        mmr_rmw(0x6084, |v| v & 0x80);
        unlock();
        select(0x65E8, n, 0x1F);
        mmr_w(0x65EC, 0x7_0015);
        select(0x6400, 0x1_0000 | n, 0x1_001F);
        mmr_w(0x6404, RAW_RING >> 12 | (RAW_RING + RAW_LEN - 1) >> 12 << 16);
        mmr_w(write_position(n), RAW_RING);
        select(0x6400, n, 0x1_001F);
        mmr_w(0x6404, RAW_RING + RAW_LEN - 1);
        // Events at packet ends, one per 2 KB written. Nothing waits for
        // them (the ring is polled), but with this setting the write
        // position always reads at a packet boundary.
        mmr_rmw(0x6580 + 4 * (n >> 1), |v| (v & !0x1F7) | 0x006);
        mmr_w(0x67D4, bit);
        mmr_w(0x67B0, bit);
        mmr_w(0x6150, bit);

        // A channel that starts first delivers some packets it had kept
        // from its last run; the first chunk is dropped.
        self.raw = Some(Raw { dest, rp: RAW_RING + CHUNK, seen: RAW_RING, fill: 0, skip: CHUNK });
        fb_w(p + 0x18, RAW_HANDLE);
        fb_w(p + 0x1C, 0);
        OK
    }

    /// Command 10: the flush before a close. Nothing is held back, so
    /// there is nothing to do.
    pub fn flush(&mut self, handle: u32) -> u32 {
        if handle == RAW_HANDLE && self.raw.is_some() { OK } else { FAILED }
    }

    /// Command 9: close a stream.
    pub fn close(&mut self, handle: u32) -> u32 {
        if handle != RAW_HANDLE || self.raw.is_none() {
            return FAILED;
        }
        stop_channel(RAW_CHANNEL);
        lock();
        set_filter(RAW_FILTER, 0, 0);
        unlock();
        mmr_w(0x67D4, 1 << RAW_CHANNEL);
        mmr_w(0x67B0, 1 << RAW_CHANNEL);
        self.raw = None;
        OK
    }

    /// Send the next chunk of the Raw ring when one is complete and the
    /// DMA engine is free.
    pub fn poll(&mut self, dma: &mut Dma) {
        let Some(r) = self.raw.as_mut() else { return };
        let last = mmr_r(write_position(RAW_CHANNEL));
        if !(RAW_RING..RAW_RING + RAW_LEN).contains(&last) {
            // Reads 0 until the first packet has been written.
            return;
        }
        let mut now = last + 1;
        if now == RAW_RING + RAW_LEN {
            now = RAW_RING;
        }
        let advance = if now >= r.seen { now - r.seen } else { now + RAW_LEN - r.seen };
        r.seen = now;
        let dropped = r.skip.min(advance);
        r.skip -= dropped;
        r.fill += advance - dropped;
        if r.fill > RAW_LEN - CHUNK {
            // The channel has lapped the reader: start again at the chunk
            // being written (chunks hold whole packets).
            let offset = (now - RAW_RING) / CHUNK * CHUNK;
            r.rp = RAW_RING + offset;
            r.fill = now - r.rp;
            self.overruns += 1;
        }
        if r.fill >= CHUNK && dma.idle() {
            dma.start(r.rp, r.dest, CHUNK, SRC_LOCAL);
            r.fill -= CHUNK;
            r.rp += CHUNK;
            if r.rp == RAW_RING + RAW_LEN {
                r.rp = RAW_RING;
            }
        }
    }
}
