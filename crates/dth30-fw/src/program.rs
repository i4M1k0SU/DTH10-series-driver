//! The Program stream: the whole TS without null packets, with the
//! elementary streams of one service descrambled.
//!
//! The service's elementary streams and PCR go to a channel of their own,
//! which descrambles them. Everything else stays in the "every PID"
//! channel (see `ts`), whose ring this stream reads as well. The two rings
//! are sent to the host's endpoint in turn, whole packets at a time, so
//! the host sees one packet stream. The service's ECMs are picked out of
//! the "every PID" ring and given to the card; its answer carries the two
//! control words, which go into the key RAM for the descrambler.
//!
//! The descrambler setting of a channel (0x6080 index, 0x6084 word):
//! bits 2..0 = 7 and bit 3 = 1 descramble MULTI2, bits 10..8 the key RAM
//! slot of the CBC initial value minus 64, bits 18..14 the first of the
//! four slots of the system key, bits 24..20 the slot of the even control
//! word, bits 30..26 the slot of the odd one. With bits 2..0 = 0 the
//! packets pass as they are.

use crate::card::{ACTIVE, Card, ECM_MAX, Output, SLOT_CBC, SLOT_SYSTEM_KEY};
use crate::demux::{self, FILTER_VALID, PACKET, Ring};
use crate::dma::{Dma, SRC_LOCAL};
use crate::host::{FAILED, OK};
use crate::hw::{count, fb_r, fb_w, mmr_rmw};
use crate::keyram;
use crate::stats::{self, Since};
use crate::ts::{CHUNK, Ts};

const ES_CHANNEL: u32 = 0;
/// Filter entries 0..=25 are the elementary streams and PCR.
const ES_FILTERS: usize = 26;
/// The elementary streams' ring: 10240 packets.
const ES_RING: u32 = 0x00C0_0000;
const ES_LEN: u32 = 0x001D_6000;

/// Key RAM slots of the control words.
const SLOT_ODD: u32 = 6;
const SLOT_EVEN: u32 = 7;

const HANDLE: u32 = 0xA006_2A80;

/// The most sent in one piece (whole packets, one DMA descriptor).
const PIECE: u32 = 174 * PACKET;
/// A ring's data is sent when this much is ready, or after 5 ms.
const ENOUGH: u32 = 12 * PACKET;
/// Packets dropped after the start (what a channel holds back).
const SKIP: u32 = 64 * PACKET;
/// How far back in the "every PID" ring the open looks for the service's
/// latest ECM (ECMs come about every 100 ms; this is 200 ms or more).
const LOOK_BACK: u32 = 4096 * PACKET;
/// The first word of a null packet (PID 0x1FFF, payload only).
const NULL_HEAD: u32 = u32::from_le_bytes([0x47, 0x1F, 0xFF, 0x10]);

struct Source {
    ring: Ring,
    last_sent: u32,
}

struct Open {
    /// PCI address of the host's endpoint FIFO.
    dest: u32,
    /// Count ticks in 5 ms.
    patience: u32,
    /// The elementary streams.
    es: Source,
    /// Everything else: a reader of the "every PID" ring.
    rest: Source,
    /// How much of what `rest` has ready has been looked at (bytes).
    looked: u32,
    rest_overruns: u32,
    /// Whether `rest` goes first when both have data.
    rest_first: bool,
    /// The PIDs that go to the elementary streams' channel.
    pids: [u16; ES_FILTERS],
    npids: usize,
    ecm_pid: Option<u32>,
    /// The ECM body given to the card last.
    last: [u8; ECM_MAX],
    last_len: usize,
    odd: Option<[u8; 8]>,
    even: Option<[u8; 8]>,
    card_active: bool,
    since: Since,
}

pub struct Program {
    open: Option<Open>,
}

fn be32(b: &[u8]) -> u32 {
    u32::from_be_bytes([b[0], b[1], b[2], b[3]])
}

/// The PID of the packet at physical address `at`, from its first word.
fn pid_at(at: u32) -> Option<u32> {
    let head = fb_r(at).to_le_bytes();
    (head[0] == 0x47).then(|| u32::from(head[1] & 0x1F) << 8 | u32::from(head[2]))
}

fn packet_at(at: u32) -> [u8; 188] {
    let mut p = [0u8; 188];
    for (i, w) in p.as_chunks_mut::<4>().0.iter_mut().enumerate() {
        *w = fb_r(at + 4 * i as u32).to_le_bytes();
    }
    p
}

/// The body of an ECM section that fits in this one packet.
fn ecm_body(p: &[u8; 188]) -> Option<&[u8]> {
    // No transport error, a section starts here.
    if p[1] & 0xC0 != 0x40 {
        return None;
    }
    let payload = match p[3] >> 4 & 3 {
        1 => 4,
        3 => 5 + usize::from(p[4]),
        _ => return None,
    };
    let section = payload + 1 + usize::from(*p.get(payload)?);
    let header = p.get(section..section + 3)?;
    let length = usize::from(header[1] & 0x0F) << 8 | usize::from(header[2]);
    // table_id 0x82; eight bytes of section header, then the body, then the CRC.
    if header[0] != 0x82 || length < 10 {
        return None;
    }
    let body = p.get(section + 8..section + 3 + length - 4)?;
    (body.len() <= ECM_MAX).then_some(body)
}

impl Open {
    /// Give an ECM to the card unless it is the one given last.
    fn ecm(&mut self, p: &[u8; 188], card: &mut Card) {
        let Some(body) = ecm_body(p) else { return };
        if body != &self.last[..self.last_len] && card.submit_ecm(body) {
            self.last[..body.len()].copy_from_slice(body);
            self.last_len = body.len();
            self.since.mark(stats::ECM_SENT);
        }
    }
}

impl Program {
    pub fn new() -> Self {
        Self { open: None }
    }

    pub fn is_open(&self) -> bool {
        self.open.is_some()
    }

    pub fn owns(&self, handle: u32) -> bool {
        handle == HANDLE && self.open.is_some()
    }

    /// Command 8, kind 5: open the Program stream.
    ///
    /// Payload: +0x08 input port, +0x18 the host's own handle (replaced by
    /// the firmware's), +0x60 flags (bit 20: TS to the host), +0x314
    /// number of host buffers, +0x318 chunk size, +0x31C.. their PCI
    /// addresses, +0xC08 PCR PID, +0xF2C mode (8 = descramble with the
    /// card; 16, the vendor's re-encryption, is refused), +0x1068 number
    /// of elementary streams, +0x106C.. their PIDs, +0x10CC.. their ECM
    /// PIDs (0x1FFF = not scrambled). One ECM PID is followed: the first
    /// one given.
    pub fn open(&mut self, p: u32, clock_5ms: u32, ts: &mut Ts, card: &mut Card) -> u32 {
        let (port, flags, mode) = (fb_r(p + 0x08), fb_r(p + 0x60), fb_r(p + 0xF2C));
        let (chunk, dest, n) = (fb_r(p + 0x318), fb_r(p + 0x31C), fb_r(p + 0x1068));
        if self.open.is_some() || flags & 0x10_0000 == 0 || mode & 0x18 != 8 || chunk != CHUNK || port > 3 || n > 24 {
            return FAILED;
        }
        let since = Since::now(clock_5ms / 5);
        // Null packets are dropped at the input from now on, for every
        // stream, until the TS input is set up again.
        mmr_rmw(0x600C, |v| v | 0x300);

        let is_pid = |v: u32| v != 0 && v < 0x1FFF;
        let pcr = fb_r(p + 0xC08);
        let (mut ecm_pid, mut pids, mut npids) = (None, [0u16; ES_FILTERS], 0);
        for i in 0..n {
            let (pid, ecm) = (fb_r(p + 0x106C + 4 * i), fb_r(p + 0x10CC + 4 * i));
            if ecm_pid.is_none() && is_pid(ecm) {
                ecm_pid = Some(ecm);
            }
            if is_pid(pid) && !pids[..npids].contains(&(pid as u16)) {
                pids[npids] = pid as u16;
                npids += 1;
            }
        }
        if is_pid(pcr) && !pids[..npids].contains(&(pcr as u16)) {
            pids[npids] = pcr as u16;
            npids += 1;
        }

        demux::stop_channel(ES_CHANNEL);
        demux::lock();
        for (i, pid) in pids[..npids].iter().enumerate() {
            demux::set_filter(i as u32, FILTER_VALID | port << 13 | u32::from(*pid), ES_CHANNEL);
        }
        // The keys' slots are set for the channel, but it passes the
        // packets as they are until the first control words have come.
        demux::descrambler(ES_CHANNEL, |v| {
            (v & 0x80) | 0x38 | (SLOT_CBC - 64) << 8 | SLOT_SYSTEM_KEY << 14 | SLOT_EVEN << 20 | SLOT_ODD << 26
        });
        demux::unlock();
        demux::start_channel(ES_CHANNEL, ES_RING, ES_LEN);

        let now = count();
        let (rest, history) = ts.share(port);
        let mut o = Open {
            dest,
            patience: clock_5ms,
            es: Source { ring: Ring::new(ES_CHANNEL, ES_RING, ES_LEN, PACKET, SKIP), last_sent: now },
            rest: Source { ring: rest, last_sent: now },
            looked: 0,
            rest_overruns: 0,
            rest_first: true,
            pids,
            npids,
            ecm_pid,
            last: [0; ECM_MAX],
            last_len: 0,
            odd: None,
            even: None,
            card_active: false,
            since,
        };
        // The service's latest ECM is usually in the ring already: with it
        // the card can work on the control words at once, instead of from
        // the next ECM that comes by.
        if let Some(pid) = ecm_pid {
            let back = history.min(LOOK_BACK) / PACKET * PACKET;
            let mut latest = None;
            for k in (0..back).step_by(PACKET as usize) {
                let at = o.rest.ring.behind(back - k);
                if pid_at(at) == Some(pid) {
                    let p = packet_at(at);
                    if ecm_body(&p).is_some() {
                        latest = Some(p);
                    }
                }
            }
            if let Some(p) = latest {
                o.ecm(&p, card);
            }
        }
        self.open = Some(o);
        fb_w(p + 0x18, HANDLE);
        fb_w(p + 0x1C, 0);
        OK
    }

    /// Command 9: close the Program stream.
    pub fn close(&mut self, ts: &mut Ts) -> u32 {
        demux::stop_channel(ES_CHANNEL);
        demux::lock();
        for i in 0..ES_FILTERS {
            demux::set_filter(i as u32, 0, 0);
        }
        demux::descrambler(ES_CHANNEL, |v| v & 0x80);
        demux::unlock();
        ts.unshare();
        self.open = None;
        OK
    }

    /// The card's answer to an ECM.
    pub fn card_output(&mut self, out: Output) {
        let Some(o) = self.open.as_mut() else { return };
        match out {
            Output::Keys { odd, even } => {
                // Only a control word that changed is written: the other
                // one is in use.
                if o.odd != Some(odd) {
                    keyram::write(SLOT_ODD, be32(&odd[0..4]), be32(&odd[4..8]));
                    o.odd = Some(odd);
                }
                if o.even != Some(even) {
                    keyram::write(SLOT_EVEN, be32(&even[0..4]), be32(&even[4..8]));
                    o.even = Some(even);
                }
                demux::descrambler(ES_CHANNEL, |v| v | 7);
                o.since.mark(stats::KEYS);
            }
            Output::EcmRefused => demux::descrambler(ES_CHANNEL, |v| v & !7),
            // Give the same ECM to the card again when it next comes by.
            Output::EcmLost => o.last_len = 0,
            _ => {}
        }
    }

    pub fn poll(&mut self, card: &mut Card, dma: &mut Dma) {
        let Some(o) = self.open.as_mut() else { return };
        let active = card.status() & ACTIVE != 0;
        if o.card_active && !active {
            // The card is gone or being reset: stop descrambling and start
            // afresh with the next ECM.
            demux::descrambler(ES_CHANNEL, |v| v & !7);
            o.last_len = 0;
            o.odd = None;
            o.even = None;
        }
        o.card_active = active;

        // Look at the header of every packet that has come into the
        // "every PID" ring, before it is sent.
        o.rest.ring.follow();
        if o.rest.ring.overruns != o.rest_overruns {
            o.rest_overruns = o.rest.ring.overruns;
            o.looked = 0;
        }
        while o.looked + PACKET <= o.rest.ring.fill {
            let at = o.rest.ring.ahead(o.looked);
            o.looked += PACKET;
            let Some(pid) = pid_at(at) else { continue };
            if Some(pid) == o.ecm_pid {
                o.ecm(&packet_at(at), card);
            } else if o.pids[..o.npids].contains(&(pid as u16)) {
                // A packet of the service that was on its way into this
                // ring when the filter entries took effect. It is older
                // than what the service's own channel delivers: blank it.
                fb_w(at, NULL_HEAD);
            }
        }

        o.es.ring.follow();
        if !dma.idle() {
            return;
        }
        let now = count();
        for rest in [o.rest_first, !o.rest_first] {
            let (s, limit) = if rest { (&mut o.rest, o.looked) } else { (&mut o.es, PIECE) };
            let n = s.ring.ready().min(PIECE).min(limit) / PACKET * PACKET;
            if n >= ENOUGH || (n > 0 && now.wrapping_sub(s.last_sent) >= o.patience) {
                dma.start(s.ring.rp, o.dest, n, SRC_LOCAL);
                s.ring.consume(n);
                s.last_sent = now;
                if rest {
                    o.looked -= n;
                }
                o.since.mark(if rest { stats::FIRST_REST } else { stats::FIRST_ES });
                o.rest_first = !rest;
                return;
            }
        }
    }
}
