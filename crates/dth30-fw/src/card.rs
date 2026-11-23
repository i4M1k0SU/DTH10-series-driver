//! The B-CAS card: detection, reset, ATR, the choice of the transmission
//! rate, the T=1 protocol and the INIT command that yields the descrambler's
//! system key.
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

/// The registers of the table above.
const SC_ETU: u32 = 0x1020;
const SC_TXDATA: u32 = 0x1028;
const SC_RXDATA: u32 = 0x102C;
const SC_CMD: u32 = 0x1030;
const SC_COUNT: u32 = 0x1034;
const SC_CLKDIV: u32 = 0x1080;
const GPIO_FUNC: u32 = 0x1084;
const GPIO_DATA: u32 = 0x1088;
const GPIO_DIR: u32 = 0x108C;
const GPIO_IN: u32 = 0x1090;
/// Interrupt status (write 1 to clear) and enables of the peripherals; the
/// card's are bits 19..8. Nothing waits for them (everything is polled).
const PERIPH_INT_STATUS: u32 = 0x1094;
const PERIPH_INT_ENABLE: u32 = 0x109C;
const CARD_INTS: u32 = 0xF_FF00;

/// SC_CMD: stop; receive the ATR after a reset; send `n << 16` bytes from
/// the transmit FIFO, then receive.
const CMD_IDLE: u32 = 0;
const CMD_ATR: u32 = 0x8100;
const CMD_SEND: u32 = 0x0080_0001;
/// SC_COUNT: bytes sent plus received.
const COUNT_MASK: u32 = 0x7F;
/// SC_ETU: clocks per ETU.
const ETU_MASK: u32 = 0xFFFF;
/// The size of the FIFOs.
const FIFO: u32 = 64;

/// CPU clocks per card clock; SC_CLKDIV takes half of it minus one, for
/// both half periods.
const CARD_CLOCK_DIV: u32 = 44;
const CLKDIV_HALF: u32 = CARD_CLOCK_DIV / 2 - 1;
/// ETU in clocks for F = 372, D = 1 (372 card clocks of 44 CPU clocks).
const ETU_DEFAULT: u32 = 372 * CARD_CLOCK_DIV;
/// D of the default rate.
const D_DEFAULT: u32 = 1;

const PIN_RST: u32 = 1 << 12;
const PIN_OFF: u32 = 1 << 13;
/// The card's two outputs, and their levels with the card switched off.
const PINS: u32 = PIN_RST | PIN_OFF;
const PINS_OFF: u32 = PIN_RST | PIN_OFF;
/// GPIO_IN: a card is in the slot.
const PIN_CARD: u32 = 1 << 14;
/// The card's pins in GPIO_FUNC and GPIO_DIR (bits 14..11).
const CARD_PIN_FIELD: u32 = 0x7800;

/// How often a reset is started over because the outputs had been written
/// over ([`Card::guard`]), until the host asks for the next reset.
const AGAIN_MAX: u8 = 3;
/// How often a block is sent again after the first time.
const RETRIES_MAX: u8 = 2;

/// Waits (ms): supply off before a reset; supply on with RST low; for the
/// ATR; between the ATR (or the PPS) and what is sent next; for the answer
/// to the PPS (the card has 9600 ETU, which is 0.87 s); for the answer to a
/// block (the card may take up to about 1.4 s); between two blocks; before a
/// block is sent again.
const POWER_OFF_MS: u32 = 10;
const RST_LOW_MS: u32 = 5;
const ATR_WAIT_MS: u32 = 1000;
const SETUP_GAP_MS: u32 = 5;
const PPS_WAIT_MS: u32 = 1000;
const BLOCK_WAIT_MS: u32 = 1500;
const BLOCK_GAP_MS: u32 = 2;
const RETRY_GAP_MS: u32 = 5;
/// How long the transmitter is given to come to rest after the card was
/// switched off in the middle of a block (ms).
const TX_STOP_MS: u32 = 5;
/// How often the slot's switch ([`Card::detect`]) and the card's outputs
/// ([`Card::guard`]) are looked at (ms).
const DETECT_MS: u32 = 50;
const GUARD_MS: u32 = 1;

/// Status bits as the host sees them.
pub const PRESENT: u8 = 1;
pub const ACTIVE: u8 = 2;

/// How a reset request ends ([`Output::ResetDone`]).
pub const RESET_ANSWERED: u32 = 0;
pub const RESET_FAILED: u32 = 1;

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
    /// return code.
    InitDone(u32),
    /// The card answered an ECM with the two control words.
    Keys { odd: [u8; 8], even: [u8; 8] },
    /// The card answered an ECM but does not let the service be viewed.
    EcmRefused,
    /// An ECM got no usable answer.
    EcmLost,
}

/// The longest ECM body taken: with its command header the block still
/// fits the transmit FIFO.
pub const ECM_MAX: usize = 54;

/// The longest block taken in.
const RX_MAX: usize = 260;
/// Requests waiting for the card, and reports waiting for the CAS object.
const JOBS_MAX: usize = 4;
const OUTPUTS_MAX: usize = 4;

/// TS of an ATR in the direct convention.
const TS_DIRECT: u8 = 0x3B;
/// T=1, as TD1 names the protocol (and TA2 and PPS0 too).
const T1: u8 = 1;
/// TA2 bit 4: the rate is not TA1's but one of the card's own choice.
const TA2_IMPLICIT: u8 = 0x10;
/// PPSS, and the bit of PPS0 that says PPS1 follows.
const PPSS: u8 = 0xFF;
const PPS0_PPS1: u8 = 0x10;

/// T=1 (ISO 7816-3): the NAD used; NAD, PCB, LEN and LRC around the
/// information field.
const NAD: u8 = 0;
const BLOCK_OVERHEAD: usize = 4;
/// PCB: an S-block (both bits), an R-block (the first only), the more-data
/// bit of an I-block, and where N(S) of an I-block and N(R) of an R-block
/// are.
const PCB_S: u8 = 0xC0;
const PCB_R: u8 = 0x80;
const PCB_MORE: u8 = 0x20;
const NS_BIT: u32 = 6;
const NR_BIT: u32 = 4;
/// S-blocks: RESYNCH, IFS and WTX requests, RESYNCH and WTX responses.
const S_RESYNCH: u8 = 0xC0;
const S_IFS: u8 = 0xC1;
const S_WTX: u8 = 0xC3;
const S_RESYNCH_RESPONSE: u8 = 0xE0;
const S_WTX_RESPONSE: u8 = 0xE3;
/// The block size asked of the card (IFSD).
const IFSD: u8 = 254;

/// The class of the B-CAS commands, and the INIT and ECM commands.
const CLA: u8 = 0x90;
const INS_INIT: u8 = 0x30;
const INS_ECM: u8 = 0x34;
/// The shortest information field of an INIT and of an ECM response.
const INIT_RESPONSE_LEN: usize = 56;
const ECM_RESPONSE_LEN: usize = 22;

#[derive(Clone, Copy, PartialEq)]
enum Job {
    /// S(RESYNCH request).
    Resync,
    /// S(IFS request): the card may send blocks of up to 254 bytes.
    Ifs,
    /// The INIT command.
    Init,
    /// The ECM command with the body in `Card::ecm`.
    Ecm,
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
    /// Guard time before the PPS request for the rate `ta1` goes out.
    PpsGap {
        until: u32,
        ta1: u8,
    },
    /// The PPS request is out; the answer is coming in.
    Pps {
        deadline: u32,
        ta1: u8,
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
    /// The ATR is complete: TA1 (the fastest rate the card offers), TA2
    /// (given by a card that needs no PPS) and TA3 (the card's block size),
    /// if given, and the protocol the card names first.
    Done {
        ta1: Option<u8>,
        ta2: Option<u8>,
        ta3: Option<u8>,
        protocol: u8,
    },
}

/// Follow the ATR's structure to learn where it ends.
fn parse_atr(b: &[u8]) -> Atr {
    if b.is_empty() {
        return Atr::More;
    }
    if b[0] != TS_DIRECT {
        return Atr::Bad;
    }
    if b.len() < 2 {
        return Atr::More;
    }
    let historical = usize::from(b[1] & 0x0F);
    let (mut y, mut i, mut level, mut tck) = (b[1] >> 4, 2, 1, false);
    let (mut ta1, mut ta2, mut ta3, mut protocol) = (None, None, None, 0);
    loop {
        if y & 1 != 0 {
            match level {
                1 => ta1 = b.get(i).copied(),
                2 => ta2 = b.get(i).copied(),
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
        if level == 1 {
            protocol = td & 0x0F;
        }
        y = td >> 4;
        i += 1;
        level += 1;
        if level > 8 {
            return Atr::Bad;
        }
    }
    if b.len() < i + historical + usize::from(tck) { Atr::More } else { Atr::Done { ta1, ta2, ta3, protocol } }
}

/// The rate TA1 names, as the divisor D of the ETU. Only F = 372 is
/// supported.
fn rate(ta1: u8) -> Option<u32> {
    match (ta1 >> 4, ta1 & 0x0F) {
        (0 | 1, 1) => Some(1),
        (0 | 1, 2) => Some(2),
        (0 | 1, 3) => Some(4),
        (0 | 1, 4) => Some(8),
        _ => None,
    }
}

/// The PPS request for T=1 at the rate `ta1`: PPSS, PPS0 (PPS1 follows,
/// T=1), PPS1, and the check byte that makes the XOR of all of them zero.
fn pps_request(ta1: u8) -> [u8; 4] {
    const PPS0: u8 = PPS0_PPS1 | T1;
    [PPSS, PPS0, ta1, PPSS ^ PPS0 ^ ta1]
}

/// The length of a PPS answer, once its PPS0 is in.
fn pps_len(b: &[u8]) -> Option<usize> {
    Some(3 + (b.get(1)? & 0x70).count_ones() as usize)
}

/// What a complete PPS answer to the request for `ta1` says: the rate the
/// card works at from now on (the request repeated: the one asked for; no
/// PPS1: the default one), or `None` when it is neither.
fn pps_answer(b: &[u8], ta1: u8) -> Option<u32> {
    if b == pps_request(ta1) {
        rate(ta1)
    } else if b == [PPSS, T1, PPSS ^ T1] {
        Some(D_DEFAULT)
    } else {
        None
    }
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
    /// The levels the card's two outputs are meant to have.
    levels: u32,
    next_guard: u32,
    /// Resets started over since the host last asked for one.
    again: u8,
    /// The card in the slot did not accept the PPS: it is used at the
    /// default rate.
    no_pps: bool,
    jobs: [Job; JOBS_MAX],
    njobs: usize,
    retries: u8,
    /// T=1 sequence numbers.
    ns: u8,
    nr: u8,
    rx: [u8; RX_MAX],
    rx_len: usize,
    ecm: [u8; ECM_MAX],
    ecm_len: usize,
    out: [Option<Output>; OUTPUTS_MAX],
}

impl Card {
    pub fn new(clock: &Clock) -> Self {
        mmr_rmw(PERIPH_INT_ENABLE, |v| v | CARD_INTS);
        mmr_w(SC_CLKDIV, CLKDIV_HALF | CLKDIV_HALF << 16);
        // Bits 19..16 of SC_ETU and register 0x1024 as the vendor's firmware
        // sets them (what they do is not known).
        mmr_w(SC_ETU, 0xF_0000 | ETU_DEFAULT);
        mmr_w(0x1024, 3007);
        Self::pins();
        mmr_rmw(GPIO_DATA, |v| v | PINS_OFF);
        mmr_w(PERIPH_INT_STATUS, CARD_INTS);
        mmr_w(SC_CMD, CMD_IDLE);
        Self {
            ms: clock.ticks(1),
            status: 0,
            seen: false,
            next_detect: count(),
            phase: Phase::Idle,
            resetting: false,
            levels: PINS_OFF,
            next_guard: count(),
            again: 0,
            no_pps: false,
            jobs: [Job::Resync; JOBS_MAX],
            njobs: 0,
            retries: 0,
            ns: 0,
            nr: 0,
            rx: [0; RX_MAX],
            rx_len: 0,
            ecm: [0; ECM_MAX],
            ecm_len: 0,
            out: [None; OUTPUTS_MAX],
        }
    }

    /// The card's pins: functions and directions (RST and supply are outputs).
    fn pins() {
        mmr_rmw(GPIO_FUNC, |v| (v & !CARD_PIN_FIELD) | PINS | PIN_CARD);
        mmr_rmw(GPIO_DIR, |v| (v & !CARD_PIN_FIELD) | PINS);
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

    /// Set the card's two outputs.
    fn drive(&mut self, levels: u32) {
        self.levels = levels;
        mmr_rmw(GPIO_DATA, |v| (v & !PINS) | levels);
    }

    /// The output register is not the firmware's alone: the host has its
    /// lamp and tuner bits in it and changes them by reading the register
    /// and writing it back, which undoes a change made here in between; and
    /// its GPIO set-up writes the whole register once after the firmware
    /// has started. So the two outputs are compared every millisecond with
    /// what they are meant to be, and put right.
    ///
    /// The wrong levels switch the card off or hold it in reset, so a card
    /// in use or being reset is started again. (And an empty slot whose
    /// supply is asked for goes on reporting "no card" after one has been
    /// put in.)
    fn guard(&mut self) {
        if !Self::passed(self.next_guard) {
            return;
        }
        self.next_guard = self.after(GUARD_MS);
        if mmr_r(GPIO_DATA) & PINS == self.levels {
            return;
        }
        if self.status & ACTIVE == 0 && matches!(self.phase, Phase::Idle) {
            self.drive(self.levels);
        } else if self.again < AGAIN_MAX {
            self.again += 1;
            self.power_cycle();
        } else {
            self.forget();
        }
    }

    /// Power the card down, and up again after 10 ms; its ATR, the protocol
    /// set-up and INIT follow in [`Card::poll`].
    fn power_cycle(&mut self) {
        self.njobs = 0;
        self.status &= !ACTIVE;
        Self::pins();
        mmr_w(SC_CMD, CMD_IDLE);
        self.drive(PINS_OFF);
        self.phase = Phase::PowerOff { until: self.after(POWER_OFF_MS) };
    }

    /// The host asks for a card reset.
    pub fn request_reset(&mut self) {
        if self.resetting {
            self.report(Output::ResetDone(RESET_FAILED));
        }
        if self.status & PRESENT == 0 {
            self.resetting = false;
            self.report(Output::ResetDone(RESET_FAILED));
            return;
        }
        self.resetting = true;
        self.again = 0;
        self.power_cycle();
    }

    /// The card is gone, or of no use as it is: power the contacts down and
    /// forget the session. A card that is still in the slot is found again
    /// by [`Card::detect`]; the host, told of a removal and an insertion,
    /// then asks for its reset.
    fn forget(&mut self) {
        self.status = 0;
        self.njobs = 0;
        self.no_pps = false;
        self.drive(PINS_OFF);
        mmr_w(SC_CMD, CMD_IDLE);
        self.phase = Phase::Idle;
        if self.resetting {
            self.resetting = false;
            self.report(Output::ResetDone(RESET_FAILED));
        }
    }

    fn reset_failed(&mut self) {
        if !self.resetting {
            // Not a reset the host asked for ([`Card::guard`]): it would
            // not learn that the card it takes for activated is not.
            return self.forget();
        }
        mmr_w(SC_CMD, CMD_IDLE);
        self.phase = Phase::Idle;
        self.resetting = false;
        self.report(Output::ResetDone(RESET_FAILED));
    }

    /// Look at the slot every 50 ms; a change counts when seen twice.
    fn detect(&mut self) {
        if !Self::passed(self.next_detect) {
            return;
        }
        self.next_detect = self.after(DETECT_MS);
        let now = mmr_r(GPIO_IN) & PIN_CARD != 0;
        let was = self.status & PRESENT != 0;
        if now == was || now != self.seen {
            self.seen = now;
            return;
        }
        if now {
            self.status |= PRESENT;
        } else {
            self.forget();
        }
    }

    /// Empty the receive FIFO (reading it empty does no harm).
    fn drain() {
        for _ in 0..FIFO {
            mmr_r1(SC_RXDATA);
        }
    }

    /// Take the bytes received so far; `sent` bytes of the count are the
    /// block that went out.
    fn pull(&mut self, sent: usize) {
        let have = ((mmr_r(SC_COUNT) & COUNT_MASK) as usize).saturating_sub(sent);
        while self.rx_len < have && self.rx_len < self.rx.len() {
            self.rx[self.rx_len] = mmr_r1(SC_RXDATA) as u8;
            self.rx_len += 1;
        }
    }

    /// Send a T=1 block: NAD 0, PCB, LEN, the information field, LRC.
    fn send(&mut self, pcb: u8, inf: &[u8]) {
        Self::drain();
        let mut lrc = NAD ^ pcb ^ inf.len() as u8;
        mmr_w(SC_TXDATA, u32::from(NAD));
        mmr_w(SC_TXDATA, u32::from(pcb));
        mmr_w(SC_TXDATA, inf.len() as u32);
        for &b in inf {
            mmr_w(SC_TXDATA, u32::from(b));
            lrc ^= b;
        }
        mmr_w(SC_TXDATA, u32::from(lrc));
        let n = inf.len() + BLOCK_OVERHEAD;
        mmr_w(SC_CMD, CMD_SEND | (n as u32) << 16);
        self.rx_len = 0;
        self.phase = Phase::Wait { sent: n, deadline: self.after(BLOCK_WAIT_MS) };
    }

    /// Send the PPS request for the rate `ta1`.
    fn send_pps(&mut self, ta1: u8) {
        Self::drain();
        let request = pps_request(ta1);
        for b in request {
            mmr_w(SC_TXDATA, u32::from(b));
        }
        mmr_w(SC_CMD, CMD_SEND | (request.len() as u32) << 16);
        self.rx_len = 0;
        self.phase = Phase::Pps { deadline: self.after(PPS_WAIT_MS), ta1 };
    }

    /// The card did not accept the PPS (or did not answer): reset it once
    /// more and use it at the default rate.
    fn pps_failed(&mut self) {
        self.no_pps = true;
        self.power_cycle();
    }

    fn send_job(&mut self) {
        match self.jobs[0] {
            Job::Resync => self.send(S_RESYNCH, &[]),
            Job::Ifs => self.send(S_IFS, &[IFSD]),
            Job::Init => self.send(self.ns << NS_BIT, &[CLA, INS_INIT, 0x00, 0x00, 0x00]),
            Job::Ecm => {
                // 90 34 00 00 Lc, the ECM body, Le.
                let mut apdu = [0u8; ECM_MAX + 6];
                let n = self.ecm_len;
                apdu[..5].copy_from_slice(&[CLA, INS_ECM, 0x00, 0x00, n as u8]);
                apdu[5..5 + n].copy_from_slice(&self.ecm[..n]);
                self.send(self.ns << NS_BIT, &apdu[..n + 6]);
            }
        }
    }

    fn job_done(&mut self) {
        self.jobs.copy_within(1.., 0);
        self.njobs -= 1;
        self.retries = 0;
        self.phase = if self.njobs > 0 { Phase::Gap { until: self.after(BLOCK_GAP_MS) } } else { Phase::Idle };
    }

    /// The block was lost or damaged: send it again, twice at most.
    fn retry(&mut self) {
        self.retries += 1;
        if self.retries <= RETRIES_MAX {
            self.phase = Phase::Gap { until: self.after(RETRY_GAP_MS) };
            return;
        }
        match self.jobs[0] {
            // Without the keys of INIT the card is of no use.
            Job::Init => return self.forget(),
            Job::Ecm => self.report(Output::EcmLost),
            _ => {}
        }
        self.job_done();
    }

    /// A complete block has come in.
    fn response(&mut self) {
        let len = usize::from(self.rx[2]);
        if self.rx[..len + BLOCK_OVERHEAD].iter().fold(0, |a, b| a ^ b) != 0 || self.rx[0] != NAD {
            return self.retry();
        }
        let pcb = self.rx[1];
        if pcb & PCB_S == PCB_S {
            match pcb {
                // The card asks for more time: grant it and keep waiting.
                S_WTX => {
                    let wtx = [self.rx[3]];
                    self.send(S_WTX_RESPONSE, &wtx[..len.min(1)]);
                }
                S_RESYNCH_RESPONSE => {
                    self.ns = 0;
                    self.nr = 0;
                    self.job_done();
                }
                _ => self.job_done(),
            }
        } else if pcb & PCB_R != 0 {
            // R block: the card names the sequence number it expects.
            self.ns = pcb >> NR_BIT & 1;
            self.retry();
        } else {
            self.ns ^= 1;
            self.nr ^= 1;
            // Chained answers are not supported (the IFS request makes them
            // unnecessary for this card).
            let ok = pcb & PCB_MORE == 0;
            if self.jobs[0] == Job::Init {
                if !ok || len < INIT_RESPONSE_LEN {
                    // Not the answer of a B-CAS card.
                    return self.forget();
                }
                let ret = self.init_response();
                self.report(Output::InitDone(ret));
            }
            if self.jobs[0] == Job::Ecm {
                let out = if ok && len >= ECM_RESPONSE_LEN { self.ecm_response() } else { Output::EcmLost };
                self.report(out);
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

    /// The ECM response: bytes 4..5 return code, 6..13 the odd control
    /// word, 14..21 the even one.
    fn ecm_response(&self) -> Output {
        let inf = &self.rx[3..];
        let ret = u32::from(inf[4]) << 8 | u32::from(inf[5]);
        // The return codes with which the service may be viewed.
        if !matches!(ret, 0x0200 | 0x0400 | 0x0800 | 0x4280 | 0x4480) {
            return Output::EcmRefused;
        }
        let (mut odd, mut even) = ([0; 8], [0; 8]);
        odd.copy_from_slice(&inf[6..14]);
        even.copy_from_slice(&inf[14..22]);
        Output::Keys { odd, even }
    }

    /// Give an ECM body to the card. False when it cannot be taken now
    /// (no active card, or the previous ECM is still with the card).
    pub fn submit_ecm(&mut self, body: &[u8]) -> bool {
        if self.status & ACTIVE == 0 || body.is_empty() || body.len() > ECM_MAX {
            return false;
        }
        match self.jobs[..self.njobs].iter().position(|j| *j == Job::Ecm) {
            // Already on its way to the card.
            Some(0) => return false,
            // Waiting behind other blocks: the newer ECM replaces it.
            Some(_) => {}
            None if self.njobs == self.jobs.len() => return false,
            None => {
                self.jobs[self.njobs] = Job::Ecm;
                self.njobs += 1;
                if self.njobs == 1 {
                    self.phase = Phase::Gap { until: self.after(BLOCK_GAP_MS) };
                }
            }
        }
        self.ecm[..body.len()].copy_from_slice(body);
        self.ecm_len = body.len();
        true
    }

    /// The ATR is in: choose the transmission rate.
    ///
    /// A card that gives TA2 ("specific mode", as B-CAS cards do) already
    /// works at the rate of its TA1. Any other card ("negotiable mode")
    /// stays at the default rate until it has accepted a PPS request, so
    /// TA1's rate is asked for when it is faster.
    fn atr_done(&mut self, ta1: Option<u8>, ta2: Option<u8>, ta3: Option<u8>, protocol: u8) {
        // Only T=1 is spoken, and the card has to say how long a block it
        // takes (the default of 32 bytes is too short for an ECM).
        if protocol != T1 || ta2.is_some_and(|t| t & 0x0F != T1) || ta3.is_none() {
            return self.reset_failed();
        }
        // No TA1 means the default rate.
        let offered = ta1.map_or(Some(D_DEFAULT), rate);
        match (ta2, offered, ta1) {
            // Not TA1's rate but one of the card's own choice, which is taken
            // to be the default.
            (Some(t), _, _) if t & TA2_IMPLICIT != 0 => self.activate(D_DEFAULT),
            (Some(_), Some(d), _) => self.activate(d),
            (Some(_), None, _) => self.reset_failed(),
            (None, Some(d), Some(ta1)) if d > D_DEFAULT && !self.no_pps => {
                self.phase = Phase::PpsGap { until: self.after(SETUP_GAP_MS), ta1 };
            }
            (None, _, _) => self.activate(D_DEFAULT),
        }
    }

    /// The card answers at the rate `d` (the divisor of the ETU): it is
    /// active, and the protocol set-up and INIT follow.
    fn activate(&mut self, d: u32) {
        mmr_rmw(SC_ETU, |v| (v & !ETU_MASK) | (ETU_DEFAULT / d));
        self.status |= ACTIVE;
        self.ns = 0;
        self.nr = 0;
        if self.resetting {
            self.resetting = false;
            self.report(Output::ResetDone(RESET_ANSWERED));
        }
        self.jobs[..3].copy_from_slice(&[Job::Resync, Job::Ifs, Job::Init]);
        self.njobs = 3;
        self.retries = 0;
        self.phase = Phase::Gap { until: self.after(SETUP_GAP_MS) };
    }

    pub fn poll(&mut self) {
        // In this order: a card given up by `guard` is seen as gone for at
        // least one turn before `detect` finds it again.
        self.detect();
        self.guard();
        match self.phase {
            Phase::Idle => {}
            Phase::PowerOff { until } => {
                if !Self::passed(until) {
                    // Wait.
                } else if mmr_r(SC_COUNT) & COUNT_MASK != 0 {
                    // The card was switched off while a block was going
                    // out. Stopping the command does not stop the
                    // transmitter: it goes on at the rate it had (a byte
                    // every 2 ms at the default one) until its count of
                    // bytes comes round to 128, and what it counts would be
                    // taken for the start of the ATR. Start the count again
                    // and wait until nothing has moved.
                    mmr_w(SC_CMD, CMD_IDLE);
                    self.phase = Phase::PowerOff { until: self.after(TX_STOP_MS) };
                } else {
                    mmr_rmw(SC_ETU, |v| (v & !ETU_MASK) | ETU_DEFAULT);
                    mmr_w(SC_CMD, CMD_ATR);
                    Self::drain();
                    self.drive(0);
                    self.phase = Phase::PowerOn { until: self.after(RST_LOW_MS) };
                }
            }
            Phase::PowerOn { until } => {
                if Self::passed(until) {
                    self.drive(PIN_RST);
                    self.rx_len = 0;
                    self.phase = Phase::Atr { deadline: self.after(ATR_WAIT_MS) };
                }
            }
            Phase::Atr { deadline } => {
                self.pull(0);
                match parse_atr(&self.rx[..self.rx_len]) {
                    Atr::Done { ta1, ta2, ta3, protocol } => self.atr_done(ta1, ta2, ta3, protocol),
                    Atr::Bad => self.reset_failed(),
                    Atr::More if Self::passed(deadline) => self.reset_failed(),
                    Atr::More => {}
                }
            }
            Phase::PpsGap { until, ta1 } => {
                if Self::passed(until) {
                    self.send_pps(ta1);
                }
            }
            Phase::Pps { deadline, ta1 } => {
                self.pull(pps_request(ta1).len());
                let answer = &self.rx[..self.rx_len];
                match pps_len(answer).filter(|&n| answer.len() >= n) {
                    Some(n) => match pps_answer(&answer[..n], ta1) {
                        Some(d) => self.activate(d),
                        None => self.pps_failed(),
                    },
                    None if Self::passed(deadline) => self.pps_failed(),
                    None => {}
                }
            }
            Phase::Gap { until } => {
                if Self::passed(until) {
                    self.send_job();
                }
            }
            Phase::Wait { sent, deadline } => {
                self.pull(sent);
                if self.rx_len >= BLOCK_OVERHEAD && self.rx_len >= usize::from(self.rx[2]) + BLOCK_OVERHEAD {
                    self.response();
                } else if Self::passed(deadline) {
                    self.retry();
                }
            }
        }
    }
}
