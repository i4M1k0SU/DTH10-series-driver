//! TS input set-up, the "every PID" channel and the Raw stream.
//!
//! Channel 14 with an "every PID" filter entry collects whatever no other
//! entry takes. The Raw stream is that channel's ring, sent to the host's
//! endpoint in chunks of the size the host asked for, straight from the
//! ring (no copy). The Program stream reads the same ring for the packets
//! that are not its service's.
//!
//! TS input registers (offsets in the chip's block):
//!
//! | reg    | meaning                                                         |
//! |--------|-----------------------------------------------------------------|
//! | 0x6000 | input ports: bit i = 204-byte packets, bit 24+i = enable, ...   |
//! | 0x6008 | bit 8+i = valid is active high, bit 24+i = falling clock edge   |
//! | 0x600C | bit 8+i = drop the null packets of port i                       |
//! | 0x6030 | 0x503 (needed for reception)                                    |

use crate::demux::{self, FILTER_ALL, FILTER_VALID, Ring};
use crate::dma::{Dma, SRC_LOCAL};
use crate::host::{FAILED, OK};
use crate::hw::{fb_r, fb_w, mmr_r, mmr_rmw, mmr_w};

/// The "every PID" channel and its filter entry (the last one: entries
/// for single PIDs come before it).
const RAW_CHANNEL: u32 = 14;
const RAW_FILTER: u32 = 31;
/// The Raw ring: 40 chunks of 0xBC00 bytes (= 10240 packets), outside the
/// image and the areas shared with the host.
const RAW_RING: u32 = 0x0080_0000;
const RAW_LEN: u32 = 0x001D_6000;
/// The only chunk size supported (256 packets, the host's transfer size).
pub const CHUNK: u32 = 0xBC00;
/// The handle returned for the Raw stream (the host passes it back).
const RAW_HANDLE: u32 = 0xA006_2A00;
/// Two pages that idle channels point at.
const SINK: u32 = 0x0006_4000;

/// The stream types of an open (payload +0x58).
pub const KIND_RAW: u32 = 1;
pub const KIND_PROGRAM: u32 = 5;

struct Raw {
    /// PCI address of the host's endpoint FIFO.
    dest: u32,
    ring: Ring,
}

pub struct Ts {
    raw: Option<Raw>,
    /// The Program stream reads the channel too.
    shared: bool,
    /// Follows the channel while it runs, to know how much of the ring it
    /// has written since its start (`written`, at most the ring's length).
    meter: Option<Ring>,
    written: u32,
}

impl Ts {
    /// Bring the demultiplexer to rest: no channel runs, no filter entry
    /// is valid, no event reaches the CPU.
    pub fn new() -> Self {
        mmr_w(0x67DC, 0);
        mmr_w(0x67B8, 0);
        mmr_w(0x6150, mmr_r(0x6150));
        for i in 0..32 {
            demux::set_filter(i, 0, 0);
        }
        Self { raw: None, shared: false, meter: None, written: 0 }
    }

    /// Start the "every PID" channel on `port` unless it runs already.
    /// Returns whether it ran.
    fn start(&mut self, port: u32) -> bool {
        if self.raw.is_some() || self.shared {
            return true;
        }
        demux::stop_channel(RAW_CHANNEL);
        demux::lock();
        demux::set_filter(RAW_FILTER, FILTER_VALID | port << 13, RAW_CHANNEL | FILTER_ALL);
        demux::descrambler(RAW_CHANNEL, |v| v & 0x80);
        demux::unlock();
        demux::start_channel(RAW_CHANNEL, RAW_RING, RAW_LEN);
        self.meter = Some(Ring::new(RAW_CHANNEL, RAW_RING, RAW_LEN, demux::PACKET, 0));
        self.written = 0;
        false
    }

    /// Stop the channel when nothing reads it any more.
    fn stop(&mut self) {
        if self.raw.is_some() || self.shared {
            return;
        }
        demux::stop_channel(RAW_CHANNEL);
        self.meter = None;
        self.written = 0;
        demux::lock();
        demux::set_filter(RAW_FILTER, 0, 0);
        demux::unlock();
    }

    /// A reader of the "every PID" ring for the Program stream, starting
    /// at the next packet, and how many bytes before that point the channel
    /// has written since it was started (what the reader may look back at).
    pub fn share(&mut self, port: u32) -> (Ring, u32) {
        self.start(port);
        self.shared = true;
        self.measure();
        (Ring::attach(RAW_CHANNEL, RAW_RING, RAW_LEN, demux::PACKET, 0), self.written)
    }

    fn measure(&mut self) {
        if let Some(m) = self.meter.as_mut() {
            m.follow();
            self.written = (self.written + m.fill).min(RAW_LEN - CHUNK);
            m.consume(m.fill);
        }
    }

    pub fn unshare(&mut self) {
        self.shared = false;
        self.stop();
    }

    /// Command 24: set the TS input up from the words the host left in
    /// FB 0x50..0x8C. Refused while a stream is open (`busy`: another
    /// module has one).
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
    pub fn configure(&mut self, busy: bool) -> u32 {
        if self.raw.is_some() || busy {
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
            demux::lock();
            demux::set_filter(i, 0, 0);
            mmr_w(0x6080, i);
            mmr_w(0x6084, 0);
            demux::select(0x6400, 0x1_0000 | i, 0x1_001F);
            mmr_w(0x6404, sink);
            demux::unlock();
        }
        for i in 16..32 {
            demux::set_filter(i, 0, 0);
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
        // A channel that starts first delivers some packets it had kept
        // from its last run: the first chunk is dropped.
        let ring = if self.start(port) {
            Ring::attach(RAW_CHANNEL, RAW_RING, RAW_LEN, CHUNK, 0)
        } else {
            Ring::new(RAW_CHANNEL, RAW_RING, RAW_LEN, CHUNK, CHUNK)
        };
        self.raw = Some(Raw { dest, ring });
        fb_w(p + 0x18, RAW_HANDLE);
        fb_w(p + 0x1C, 0);
        OK
    }

    /// Whether `handle` is the open Raw stream's.
    pub fn owns(&self, handle: u32) -> bool {
        handle == RAW_HANDLE && self.raw.is_some()
    }

    /// Command 9: close the Raw stream.
    pub fn close(&mut self) -> u32 {
        self.raw = None;
        self.stop();
        OK
    }

    /// Send the next chunk of the Raw ring when one is complete and the
    /// DMA engine is free.
    pub fn poll(&mut self, dma: &mut Dma) {
        self.measure();
        let Some(r) = self.raw.as_mut() else { return };
        r.ring.follow();
        if r.ring.ready() >= CHUNK && dma.idle() {
            dma.start(r.ring.rp, r.dest, CHUNK, SRC_LOCAL);
            r.ring.consume(CHUNK);
        }
    }
}
