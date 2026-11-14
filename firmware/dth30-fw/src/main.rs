//! Firmware for the ViXS XCode2 chip of the BUFFALO DTH30 series of USB TV
//! tuners (DT-H30/U2, DT-H50/U2), written from scratch to stand in for the
//! one inside the vendor's driver.
//!
//! One core, no interrupts: after the start-up handshake the firmware
//! polls the host's requests, the DMA engine and the stream's ring in a
//! loop. See `../README.md` for the memory map and what is implemented.
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
mod ts;

use core::panic::PanicInfo;

use card::Card;
use cas::Cas;
use dma::Dma;
use host::{Command, FAILED, Host, OK, UNSUPPORTED};
use hw::{Clock, fb_r, fb_w};
use program::Program;
use ts::Ts;

/// The same record as for an exception, with "PANC" for the place.
#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    fb_w(0x530, 0xDEAD_C0DE);
    fb_w(0x534, 0x434E_4150);
    fb_w(0x520, fb_r(0x520).wrapping_add(1));
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
            // Ping.
            0 => OK,
            1 | 2 | 3 | 5 => UNSUPPORTED,
            _ if p == 0 => FAILED,
            // Open a stream.
            8 => match fb_r(p + 0x58) {
                ts::KIND_RAW => self.ts.open_raw(p),
                ts::KIND_PROGRAM => self.program.open(p, self.clock.ticks(5), &mut self.ts),
                _ => FAILED,
            },
            // Close a stream.
            9 => {
                let result = match fb_r(p) {
                    h if self.ts.owns(h) => self.ts.close(),
                    h if self.program.owns(h) => self.program.close(&mut self.ts),
                    _ => FAILED,
                };
                self.dma.settle(self.clock.ticks(100));
                result
            }
            // The flush before a close: nothing is held back, so there is
            // nothing to do.
            10 if self.ts.owns(fb_r(p)) || self.program.owns(fb_r(p)) => OK,
            10 => FAILED,
            11 => self.cas.input(p, &mut self.card),
            24 => self.ts.configure(self.program.is_open()),
            25 => self.cas.open(p),
            26 => self.cas.close(fb_r(p)),
            _ => FAILED,
        }
    }
}

/// Entered from the boot code with the caches on, `.bss` cleared and a
/// stack.
#[unsafe(no_mangle)]
pub extern "C" fn fw_main() -> ! {
    hw::install_vectors();
    fb_w(0xC0, boot::MAGIC);
    fb_w(0xC4, boot::VERSION);
    // The host leaves the CPU clock (kHz) in FB 0x44 before the release.
    let khz = loop {
        let v = fb_r(0x44);
        if v != 0 {
            break v;
        }
    };
    let clock = Clock::new(khz);
    let mut host = Host::new(&clock);
    host.handshake();
    let card = Card::new(&clock);
    let mut fw =
        Firmware { clock, host, dma: Dma::new(), ts: Ts::new(), card, cas: Cas::new(), program: Program::new() };
    let mut raw_first = false;
    fw.host.ready();
    loop {
        fw.host.poll_tick();
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
