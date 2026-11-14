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
mod dma;
mod host;
mod hw;
mod otp;
mod ts;

use core::panic::PanicInfo;

use dma::Dma;
use host::{Command, FAILED, Host, OK, UNSUPPORTED};
use hw::{Clock, fb_r, fb_w};
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
                _ => FAILED,
            },
            9 => self.ts.close(fb_r(p)),
            10 => self.ts.flush(fb_r(p)),
            24 => self.ts.configure(),
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
    let mut fw = Firmware { clock, host, dma: Dma::new(), ts: Ts::new() };
    fw.host.ready();
    loop {
        fw.host.poll_tick();
        if let Some(c) = fw.host.poll() {
            let result = fw.command(&c);
            fw.host.complete(&c, result);
        }
        fw.dma.poll();
        fw.ts.poll(&mut fw.dma);
    }
}
