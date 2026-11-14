//! The CAS object: what the host opens to follow the B-CAS card and to
//! ask for a card reset.
//!
//! The firmware reports to the host in blocks sent to the object's
//! endpoint: a 0x1EC-byte header, then nothing of interest up to the block
//! size the host gave (0x8000). Header words used:
//!
//! | offset | meaning                                                        |
//! |--------|----------------------------------------------------------------|
//! | +0x00  | the host's own handle (two words), as given in the open        |
//! | +0x30  | 0x10000 = card event                                           |
//! | +0x48  | 1                                                              |
//! | +0x84  | number of the block, counting through the host's buffers       |
//! | +0x88  | 1 \| 0x2000 inserted \| 0x4000 removed \| 0x8000 activated \| 0x800000 INIT done |
//! | +0x8C  | 1 with INIT done                                               |
//! | +0x1BC | with INIT done: the card's return code                         |
//!
//! The end of a reset request is told by message 0x0F ("input done").

use crate::card::{self, Card, Output};
use crate::dma::{Dma, SRC_LOCAL};
use crate::host::{FAILED, Host, OK};
use crate::hw::{fb_r, fb_w};

/// The handle returned to the host.
const HANDLE: u32 = 0xA006_2A40;
/// Where blocks are put together (SDRAM, above the firmware's other areas).
const BLOCK: u32 = 0x0006_8000;
const BLOCK_MAX: u32 = 0x8000;
const HEADER: u32 = 0x1EC;

const MSG_INPUT_DONE: u32 = 0x0F;
/// Input type (+0xA0) and request flag (+0xB8) of a card request.
const INPUT_CAS: u32 = 1 << 16;
const REQUEST_CARD: u32 = 1 << 4;

const EV_INSERTED: u32 = 0x2000;
const EV_REMOVED: u32 = 0x4000;
const EV_ACTIVATED: u32 = 0x8000;
const EV_INIT: u32 = 0x80_0000;

struct Object {
    cookie: [u32; 2],
    /// The host's buffers for event blocks: how many, their size, the PCI
    /// address (all the same endpoint FIFO).
    buffers: u32,
    size: u32,
    dest: u32,
    next: u32,
    /// The card status last reported.
    reported: u8,
    /// Words of the last request, repeated in "input done".
    request: [u32; 3],
}

pub struct Cas {
    object: Option<Object>,
    /// Events waiting for the DMA engine: +0x88 and +0x1BC.
    events: [(u32, u32); 8],
    count: usize,
}

impl Cas {
    pub fn new() -> Self {
        Self { object: None, events: [(0, 0); 8], count: 0 }
    }

    /// Command 25: open the object.
    ///
    /// Payload: +0x00 the host's handle (two words, replaced by the
    /// firmware's), +0x08 mode (bit 4 = B-CAS, bit 5 = B-CAS with the
    /// vendor's re-encryption, which is not supported and treated as bit
    /// 4), +0x0C number of event buffers, +0x10 their size, +0x14.. their
    /// PCI addresses.
    pub fn open(&mut self, p: u32) -> u32 {
        let (mode, buffers, size, dest) = (fb_r(p + 0x08), fb_r(p + 0x0C), fb_r(p + 0x10), fb_r(p + 0x14));
        if self.object.is_some()
            || mode & 0x30 == 0
            || !(1..=64).contains(&buffers)
            || !(HEADER..=BLOCK_MAX).contains(&size)
        {
            return FAILED;
        }
        for off in (0..BLOCK_MAX).step_by(4) {
            fb_w(BLOCK + off, 0);
        }
        self.object =
            Some(Object { cookie: [fb_r(p), fb_r(p + 4)], buffers, size, dest, next: 0, reported: 0, request: [0; 3] });
        self.count = 0;
        fb_w(p, HANDLE);
        fb_w(p + 4, 0);
        OK
    }

    /// Command 26: close the object. The card stays as it is.
    pub fn close(&mut self, handle: u32) -> u32 {
        if handle != HANDLE || self.object.is_none() {
            return FAILED;
        }
        self.object = None;
        self.count = 0;
        OK
    }

    /// Command 11: a request to the object. Only the card reset is
    /// supported (+0xA0 bit 16, +0xB8 bit 4, +0x1BC bit 0); the host always
    /// gets "done" for the command itself and the outcome in message 0x0F.
    pub fn input(&mut self, p: u32, card: &mut Card) -> u32 {
        fb_w(p + 0xA4, 0);
        let Some(o) = self.object.as_mut() else { return OK };
        let (kind, flags, what) = (fb_r(p + 0xA0), fb_r(p + 0xB8), fb_r(p + 0x1BC));
        if fb_r(p) != HANDLE || kind & INPUT_CAS == 0 || flags & REQUEST_CARD == 0 {
            return OK;
        }
        o.request = [kind, fb_r(p + 0xAC), flags];
        if what & 1 != 0 {
            card.request_reset();
        }
        OK
    }

    fn event(&mut self, bits: u32, value: u32) {
        if self.object.is_some() && self.count < self.events.len() {
            self.events[self.count] = (bits | 1, value);
            self.count += 1;
        }
    }

    /// Something the card code reports.
    pub fn card_output(&mut self, out: Output, host: &mut Host) {
        let Some(o) = self.object.as_ref() else { return };
        match out {
            Output::ResetDone(result) => {
                let Some(m) = host.message(MSG_INPUT_DONE) else { return };
                let d = Host::descriptor(m);
                fb_w(d, o.cookie[0]);
                fb_w(d + 0x04, o.cookie[1]);
                fb_w(d + 0x08, o.request[0]);
                fb_w(d + 0x1C, o.request[1]);
                fb_w(d + 0x20, result);
                fb_w(d + 0x24, o.request[2]);
                host.send(m);
            }
            Output::InitDone(ret) => self.event(EV_INIT, ret),
            // ECM answers concern the Program stream only.
            Output::Keys { .. } | Output::EcmRefused | Output::EcmLost => {}
        }
    }

    /// Report card status changes and send the next waiting block.
    pub fn poll(&mut self, status: u8, dma: &mut Dma) {
        let Some(o) = self.object.as_mut() else { return };
        if status != o.reported {
            let changed = status ^ o.reported;
            let mut bits = 0;
            if changed & card::PRESENT != 0 {
                bits |= if status & card::PRESENT != 0 { EV_INSERTED } else { EV_REMOVED };
            }
            if changed & status & card::ACTIVE != 0 {
                bits |= EV_ACTIVATED;
            }
            o.reported = status;
            if bits != 0 {
                self.event(bits, 0);
            }
        }
        if self.count == 0 || !dma.idle() {
            return;
        }
        let Some(o) = self.object.as_mut() else { return };
        let (bits, value) = self.events[0];
        self.events.copy_within(1..self.count, 0);
        self.count -= 1;
        for off in (0..HEADER).step_by(4) {
            fb_w(BLOCK + off, 0);
        }
        fb_w(BLOCK, o.cookie[0]);
        fb_w(BLOCK + 0x04, o.cookie[1]);
        fb_w(BLOCK + 0x30, 0x1_0000);
        fb_w(BLOCK + 0x48, 1);
        fb_w(BLOCK + 0x84, o.next);
        fb_w(BLOCK + 0x88, bits);
        if bits & EV_INIT != 0 {
            fb_w(BLOCK + 0x8C, 1);
            fb_w(BLOCK + 0x1BC, value);
        }
        o.next = (o.next + 1) % o.buffers;
        dma.start(BLOCK, o.dest, o.size, SRC_LOCAL);
    }
}
