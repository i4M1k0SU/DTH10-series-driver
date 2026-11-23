//! Firmware for the ViXS XCode2 chip of the BUFFALO DTH30 series of USB TV
//! tuners (DT-H30/U2, DT-H45/U2, DT-H50/U2), written from scratch to stand in
//! for the one inside the vendor's driver.
//!
//! One core, no interrupts: after the start-up handshake the firmware
//! polls the host's requests, the DMA engine and the stream's ring in a
//! loop. See `README.md` for the memory map and what is implemented.
#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]

mod boot;
mod card;
mod cas;
mod demux;
mod dma;
mod host;
mod hw;
mod keyram;
mod otp;
mod program;
mod stats;
mod ts;

use core::panic::PanicInfo;

use boot::{FATAL_COUNT, FATAL_MARK, FATAL_RECORD};
use card::Card;
use cas::Cas;
use dma::Dma;
use host::{
    CMD_CAS_CLOSE, CMD_CAS_OPEN, CMD_CLOSE, CMD_FLUSH, CMD_INPUT, CMD_OPEN, CMD_PING, CMD_TS_CONFIG, Command, FAILED,
    Host, OK, UNSUPPORTED,
};
use hw::{Clock, fb_r, fb_w};
use program::Program;
use ts::Ts;

/// The place a panic gives in its fatal error record.
const PANIC_PLACE: u32 = u32::from_le_bytes(*b"PANC");
/// The CPU clock (kHz), from the host (FB).
const FB_CLOCK_KHZ: u32 = 0x44;
/// How long the close of a stream lets the running DMA transfer finish (ms).
const CLOSE_SETTLE_MS: u32 = 100;

/// The same record as for an exception, with "PANC" for the place.
#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    fb_w(FATAL_RECORD, FATAL_MARK);
    fb_w(FATAL_RECORD + 4, PANIC_PLACE);
    fb_w(FATAL_COUNT, fb_r(FATAL_COUNT).wrapping_add(1));
    loop {}
}

struct Firmware {
    clock: Clock,
    host: Host,
    dma: Dma,
    ts: Ts,
    card: Card,
    cas: Cas,
    program: Program,
}

impl Firmware {
    fn command(&mut self, c: &Command) -> u32 {
        let p = c.payload;
        if c.is_signature() {
            otp::signature(&self.clock, p);
            return OK;
        }
        match c.id {
            CMD_PING => OK,
            // The functions the vendor's firmware does not support either.
            1 | 2 | 3 | 5 => UNSUPPORTED,
            _ if p == 0 => FAILED,
            CMD_OPEN => match fb_r(p + ts::OPEN_KIND) {
                ts::KIND_RAW => self.ts.open_raw(p),
                ts::KIND_PROGRAM => {
                    self.program.open(p, self.clock.ticks(program::PATIENCE_MS), &mut self.ts, &mut self.card)
                }
                _ => FAILED,
            },
            CMD_CLOSE => {
                let result = match fb_r(p) {
                    h if self.ts.owns(h) => self.ts.close(),
                    h if self.program.owns(h) => self.program.close(&mut self.ts),
                    _ => FAILED,
                };
                self.dma.settle(self.clock.ticks(CLOSE_SETTLE_MS));
                result
            }
            // The flush before a close: nothing is held back, so there is
            // nothing to do.
            CMD_FLUSH if self.ts.owns(fb_r(p)) || self.program.owns(fb_r(p)) => OK,
            CMD_FLUSH => FAILED,
            CMD_INPUT => self.cas.input(p, &mut self.card),
            CMD_TS_CONFIG => self.ts.configure(self.program.is_open()),
            CMD_CAS_OPEN => self.cas.open(p),
            CMD_CAS_CLOSE => self.cas.close(fb_r(p)),
            _ => FAILED,
        }
    }
}

/// Entered from the boot code with the caches on, `.bss` cleared and a
/// stack.
fn fw_main() -> ! {
    hw::install_vectors();
    // The record of a fatal error (see `boot`): after power-on it holds
    // anything, later that of an earlier run.
    for off in boot::FATAL_AREA.step_by(4) {
        fb_w(off, 0);
    }
    fb_w(boot::FB_MAGIC, boot::MAGIC);
    fb_w(boot::FB_VERSION, boot::VERSION);
    // The host leaves the CPU clock (kHz) in FB 0x44 before the release.
    let khz = loop {
        let v = fb_r(FB_CLOCK_KHZ);
        if v != 0 {
            break v;
        }
    };
    let clock = Clock::new(khz);
    let mut host = Host::new(&clock);
    host.handshake();
    let before = hw::count();
    stats::key_ram_cleared(keyram::clear(), hw::count().wrapping_sub(before), clock.ticks(1));
    let card = Card::new(&clock);
    let mut fw =
        Firmware { clock, host, dma: Dma::new(), ts: Ts::new(), card, cas: Cas::new(), program: Program::new() };
    let mut raw_first = false;
    let mut turns = stats::Turns::new(fw.clock.ticks(1));
    fw.host.ready();
    loop {
        turns.turn();
        if fw.host.poll_tick() {
            turns.publish();
        }
        if let Some(c) = fw.host.poll() {
            let result = fw.command(&c);
            fw.host.complete(&c, result);
        }
        fw.card.poll();
        while let Some(out) = fw.card.take_output() {
            fw.cas.card_output(out, &mut fw.host);
            fw.program.card_output(out);
        }
        fw.dma.poll();
        fw.cas.poll(fw.card.status(), &mut fw.dma);
        // The streams share the DMA engine: take turns at being first.
        raw_first = !raw_first;
        if raw_first {
            fw.ts.poll(&mut fw.dma);
        }
        fw.program.poll(&mut fw.card, &mut fw.dma);
        fw.ts.poll(&mut fw.dma);
    }
}
