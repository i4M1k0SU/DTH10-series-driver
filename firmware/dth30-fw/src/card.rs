//! The B-CAS card: detection, reset, ATR, the T=1 protocol and the INIT
//! command that yields the descrambler's system key.
//!
//! The chip's smart card interface (offsets in the register block):
//!
//! | reg    | meaning                                                          |
//! |--------|------------------------------------------------------------------|
//! | 0x1020 | bits 15..0: clocks per ETU (the block runs at the CPU clock)     |
//! | 0x1028 | transmit data (a 64-byte FIFO)                                   |
//! | 0x102C | receive data; each read takes one byte                           |
//! | 0x1030 | command: 0 = idle, 0x8100 = receive from a reset, 0x800001 \| n << 16 = send n bytes, then receive |
//! | 0x1034 | bits 6..0: bytes sent plus received since the last command       |
//! | 0x1080 | card clock: half periods minus one (21 -> CPU clock / 44)        |
//! | 0x1084 | pin functions, 0x108C pin directions                             |
//! | 0x1088 | pin outputs: bit 12 = RST, bit 13 = supply off                   |
//! | 0x1090 | pin inputs: bit 14 = a card is in the slot                       |
//!
//! Everything is polled: nothing here blocks for longer than it takes to
//! move the bytes of one block.

use crate::hw::{Clock, count, mmr_r, mmr_r1, mmr_rmw, mmr_w};
use crate::keyram;

/// ETU in clocks for F = 372, D = 1 (372 card clocks of 44 CPU clocks).
const ETU_DEFAULT: u32 = 372 * 44;

const PIN_RST: u32 = 1 << 12;
const PIN_OFF: u32 = 1 << 13;

/// Status bits as the host sees them.
pub const PRESENT: u8 = 1;
pub const ACTIVE: u8 = 2;

/// Key RAM slots of the system key (four in a row) and the CBC initial
/// value. Slots 0 and 1 belong to the chip's own fused key.
pub const SLOT_SYSTEM_KEY: u32 = 2;
pub const SLOT_CBC: u32 = 64;

/// What the card code reports to the CAS object.
#[derive(Clone, Copy)]
pub enum Output {
    /// A reset request has ended: 0 = the card answered its reset, 1 = not.
    ResetDone(u32),
    /// The INIT response has been taken in (the keys are loaded): its
    /// return code, or `u32::MAX` when the response was unusable.
    InitDone(u32),
}

#[derive(Clone, Copy, PartialEq)]
enum Job {
    /// S(RESYNCH request).
    Resync,
    /// S(IFS request): the card may send blocks of up to 254 bytes.
    Ifs,
    /// The INIT command.
    Init,
}

#[derive(Clone, Copy)]
enum Phase {
    Idle,
    /// Supply off until `until`.
    PowerOff {
        until: u32,
    },
    /// Supply on, RST low until `until`.
    PowerOn {
        until: u32,
    },
    /// RST high, waiting for the ATR.
    Atr {
        deadline: u32,
    },
    /// Guard time before the next block goes out.
    Gap {
        until: u32,
    },
    /// A block of `sent` bytes is out; the answer is coming in.
    Wait {
        sent: usize,
        deadline: u32,
    },
}

enum Atr {
    More,
    Bad,
    /// The ATR is complete: TA1 and TA3 (the card's block size), if given.
    Done {
        ta1: Option<u8>,
        ta3: Option<u8>,
    },
}

/// Follow the ATR's structure to learn where it ends.
fn parse_atr(b: &[u8]) -> Atr {
    if b.is_empty() {
        return Atr::More;
    }
    if b[0] != 0x3B {
        return Atr::Bad;
    }
    if b.len() < 2 {
        return Atr::More;
    }
    let historical = usize::from(b[1] & 0x0F);
    let (mut y, mut i, mut level, mut tck) = (b[1] >> 4, 2, 1, false);
    let (mut ta1, mut ta3) = (None, None);
    loop {
        if y & 1 != 0 {
            match level {
                1 => ta1 = b.get(i).copied(),
                3 => ta3 = b.get(i).copied(),
                _ => {}
            }
            i += 1;
        }
        i += usize::from(y & 2 != 0) + usize::from(y & 4 != 0);
        if y & 8 == 0 {
            break;
        }
        let Some(&td) = b.get(i) else { return Atr::More };
        tck |= td & 0x0F != 0;
        y = td >> 4;
        i += 1;
        level += 1;
        if level > 8 {
            return Atr::Bad;
        }
    }
    if b.len() < i + historical + usize::from(tck) { Atr::More } else { Atr::Done { ta1, ta3 } }
}

fn be32(b: &[u8]) -> u32 {
    u32::from_be_bytes([b[0], b[1], b[2], b[3]])
}

pub struct Card {
    /// Count ticks per millisecond.
    ms: u32,
    status: u8,
    /// The last reading of the slot's switch, for the debounce.
    seen: bool,
    next_detect: u32,
    phase: Phase,
    /// A reset request is in progress (its end is reported).
    resetting: bool,
    jobs: [Job; 4],
    njobs: usize,
    retries: u8,
    /// T=1 sequence numbers.
    ns: u8,
    nr: u8,
    rx: [u8; 260],
    rx_len: usize,
    out: [Option<Output>; 4],
}

impl Card {
    pub fn new(clock: &Clock) -> Self {
        mmr_rmw(0x109C, |v| v | 0xF_FF00);
        mmr_w(0x1080, 21 | 21 << 16);
        mmr_w(0x1020, 0xF_0000 | ETU_DEFAULT);
        mmr_w(0x1024, 3007);
        Self::pins();
        mmr_rmw(0x1088, |v| v | PIN_RST | PIN_OFF);
        mmr_w(0x1094, 0xF_FF00);
        mmr_w(0x1030, 0);
        Self {
            ms: clock.ticks(1),
            status: 0,
            seen: false,
            next_detect: count(),
            phase: Phase::Idle,
            resetting: false,
            jobs: [Job::Resync; 4],
            njobs: 0,
            retries: 0,
            ns: 0,
            nr: 0,
            rx: [0; 260],
            rx_len: 0,
            out: [None; 4],
        }
    }

    /// The card's pins: functions and directions (RST and supply are outputs).
    fn pins() {
        mmr_rmw(0x1084, |v| (v & !0x7800) | 0x7000);
        mmr_rmw(0x108C, |v| (v & !0x7800) | 0x3000);
    }

    /// [`PRESENT`] and [`ACTIVE`].
    pub fn status(&self) -> u8 {
        self.status
    }

    pub fn take_output(&mut self) -> Option<Output> {
        let o = self.out[0].take()?;
        self.out.rotate_left(1);
        Some(o)
    }

    fn report(&mut self, o: Output) {
        if let Some(slot) = self.out.iter_mut().find(|s| s.is_none()) {
            *slot = Some(o);
        }
    }

    fn after(&self, ms: u32) -> u32 {
        count().wrapping_add(self.ms * ms)
    }

    fn passed(t: u32) -> bool {
        count().wrapping_sub(t) < 0x8000_0000
    }

    /// The host asks for a card reset. The card is powered down and up
    /// again; its ATR, the protocol set-up and INIT follow in [`Card::poll`].
    pub fn request_reset(&mut self) {
        if self.resetting {
            self.report(Output::ResetDone(1));
        }
        self.njobs = 0;
        self.status &= !ACTIVE;
        if self.status & PRESENT == 0 {
            self.resetting = false;
            self.phase = Phase::Idle;
            self.report(Output::ResetDone(1));
            return;
        }
        self.resetting = true;
        Self::pins();
        mmr_rmw(0x1020, |v| (v & !0xFFFF) | ETU_DEFAULT);
        mmr_w(0x1030, 0);
        mmr_rmw(0x1088, |v| v | PIN_RST | PIN_OFF);
        self.phase = Phase::PowerOff { until: self.after(10) };
    }

    fn reset_failed(&mut self) {
        mmr_w(0x1030, 0);
        self.phase = Phase::Idle;
        if self.resetting {
            self.resetting = false;
            self.report(Output::ResetDone(1));
        }
    }

    /// Look at the slot every 50 ms; a change counts when seen twice.
    fn detect(&mut self) {
        if !Self::passed(self.next_detect) {
            return;
        }
        self.next_detect = self.after(50);
        let now = mmr_r(0x1090) & 1 << 14 != 0;
        let was = self.status & PRESENT != 0;
        if now == was || now != self.seen {
            self.seen = now;
            return;
        }
        if now {
            self.status |= PRESENT;
        } else {
            // Removed: power the contacts down and forget the session.
            self.status = 0;
            self.njobs = 0;
            mmr_rmw(0x1088, |v| v | PIN_RST | PIN_OFF);
            self.reset_failed();
        }
    }

    /// Empty the receive FIFO (reading it empty does no harm).
    fn drain() {
        for _ in 0..64 {
            mmr_r1(0x102C);
        }
    }

    /// Take the bytes received so far; `sent` bytes of the count are the
    /// block that went out.
    fn pull(&mut self, sent: usize) {
        let have = ((mmr_r(0x1034) & 0x7F) as usize).saturating_sub(sent);
        while self.rx_len < have && self.rx_len < self.rx.len() {
            self.rx[self.rx_len] = mmr_r1(0x102C) as u8;
            self.rx_len += 1;
        }
    }

    /// Send a T=1 block: NAD 0, PCB, LEN, the information field, LRC.
    fn send(&mut self, pcb: u8, inf: &[u8]) {
        Self::drain();
        let mut lrc = pcb ^ inf.len() as u8;
        mmr_w(0x1028, 0);
        mmr_w(0x1028, u32::from(pcb));
        mmr_w(0x1028, inf.len() as u32);
        for &b in inf {
            mmr_w(0x1028, u32::from(b));
            lrc ^= b;
        }
        mmr_w(0x1028, u32::from(lrc));
        let n = inf.len() + 4;
        mmr_w(0x1030, 0x0080_0001 | (n as u32) << 16);
        self.rx_len = 0;
        // The card may take up to about 1.4 s for a block.
        self.phase = Phase::Wait { sent: n, deadline: self.after(1500) };
    }

    fn send_job(&mut self) {
        match self.jobs[0] {
            Job::Resync => self.send(0xC0, &[]),
            Job::Ifs => self.send(0xC1, &[0xFE]),
            Job::Init => self.send(self.ns << 6, &[0x90, 0x30, 0x00, 0x00, 0x00]),
        }
    }

    fn job_done(&mut self) {
        self.jobs.copy_within(1.., 0);
        self.njobs -= 1;
        self.retries = 0;
        self.phase = if self.njobs > 0 { Phase::Gap { until: self.after(2) } } else { Phase::Idle };
    }

    /// The block was lost or damaged: send it again, twice at most.
    fn retry(&mut self) {
        self.retries += 1;
        if self.retries <= 2 {
            self.phase = Phase::Gap { until: self.after(5) };
            return;
        }
        if self.jobs[0] == Job::Init {
            self.report(Output::InitDone(u32::MAX));
        }
        self.job_done();
    }

    /// A complete block has come in.
    fn response(&mut self) {
        let len = usize::from(self.rx[2]);
        if self.rx[..len + 4].iter().fold(0, |a, b| a ^ b) != 0 || self.rx[0] != 0 {
            return self.retry();
        }
        let pcb = self.rx[1];
        if pcb & 0xC0 == 0xC0 {
            match pcb {
                // The card asks for more time: grant it and keep waiting.
                0xC3 => {
                    let wtx = [self.rx[3]];
                    self.send(0xE3, &wtx[..len.min(1)]);
                }
                0xE0 => {
                    self.ns = 0;
                    self.nr = 0;
                    self.job_done();
                }
                _ => self.job_done(),
            }
        } else if pcb & 0x80 != 0 {
            // R block: the card names the sequence number it expects.
            self.ns = pcb >> 4 & 1;
            self.retry();
        } else {
            self.ns ^= 1;
            self.nr ^= 1;
            // Chained answers are not supported (the IFS request makes them
            // unnecessary for this card).
            let ok = pcb & 0x20 == 0;
            if self.jobs[0] == Job::Init {
                let ret = if ok && len >= 56 { self.init_response() } else { u32::MAX };
                self.report(Output::InitDone(ret));
            }
            self.job_done();
        }
    }

    /// The INIT response: bytes 4..5 return code, 16..47 system key,
    /// 48..55 CBC initial value. Load the two into the key RAM.
    fn init_response(&mut self) -> u32 {
        let inf = &self.rx[3..];
        let (key, cbc) = (&inf[16..48], &inf[48..56]);
        keyram::write(SLOT_CBC, be32(&cbc[0..4]), be32(&cbc[4..8]));
        for n in 0..4 {
            let k = &key[8 * n..8 * n + 8];
            // The system key's words go in the other way round.
            keyram::write(SLOT_SYSTEM_KEY + n as u32, be32(&k[4..8]), be32(&k[0..4]));
        }
        u32::from(inf[4]) << 8 | u32::from(inf[5])
    }

    fn atr_done(&mut self, ta1: Option<u8>, ta3: Option<u8>) {
        // Only F = 372 is supported; D from TA1's low nibble.
        let d = match ta1.map(|t| (t >> 4, t & 0x0F)) {
            Some((0 | 1, 1)) => 1,
            Some((0 | 1, 2)) => 2,
            Some((0 | 1, 3)) => 4,
            Some((0 | 1, 4)) => 8,
            _ => 0,
        };
        if d == 0 || ta3.is_none() {
            return self.reset_failed();
        }
        mmr_rmw(0x1020, |v| (v & !0xFFFF) | (ETU_DEFAULT / d));
        self.status |= ACTIVE;
        self.ns = 0;
        self.nr = 0;
        self.resetting = false;
        self.report(Output::ResetDone(0));
        self.jobs[..3].copy_from_slice(&[Job::Resync, Job::Ifs, Job::Init]);
        self.njobs = 3;
        self.retries = 0;
        self.phase = Phase::Gap { until: self.after(5) };
    }

    pub fn poll(&mut self) {
        self.detect();
        match self.phase {
            Phase::Idle => {}
            Phase::PowerOff { until } => {
                if Self::passed(until) {
                    mmr_w(0x1030, 0x8100);
                    Self::drain();
                    mmr_rmw(0x1088, |v| v & !(PIN_RST | PIN_OFF));
                    self.phase = Phase::PowerOn { until: self.after(5) };
                }
            }
            Phase::PowerOn { until } => {
                if Self::passed(until) {
                    mmr_rmw(0x1088, |v| v | PIN_RST);
                    self.rx_len = 0;
                    self.phase = Phase::Atr { deadline: self.after(1000) };
                }
            }
            Phase::Atr { deadline } => {
                self.pull(0);
                match parse_atr(&self.rx[..self.rx_len]) {
                    Atr::Done { ta1, ta3 } => self.atr_done(ta1, ta3),
                    Atr::Bad => self.reset_failed(),
                    Atr::More if Self::passed(deadline) => self.reset_failed(),
                    Atr::More => {}
                }
            }
            Phase::Gap { until } => {
                if Self::passed(until) {
                    self.send_job();
                }
            }
            Phase::Wait { sent, deadline } => {
                self.pull(sent);
                if self.rx_len >= 4 && self.rx_len >= usize::from(self.rx[2]) + 4 {
                    self.response();
                } else if Self::passed(deadline) {
                    self.retry();
                }
            }
        }
    }
}
