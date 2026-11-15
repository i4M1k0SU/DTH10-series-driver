//! Access to the chip's registers, to the SDRAM shared with the host, and
//! to the CPU's own registers.
//!
//! Everything shared with the host or with the chip's hardware (the "FB":
//! SDRAM by physical address) is touched through kseg1, uncached; the
//! firmware's own code and data run cached in kseg0.

use core::arch::asm;
use core::ptr::{read_volatile, write_volatile};

use crate::boot::{JR_K0, LUI_K0, ORI_K0};

/// The register block (MMR) as the MIPS sees it.
pub const MMR_BASE: usize = 0xB004_0000;
/// SDRAM, uncached.
pub const FB_BASE: usize = 0xA000_0000;
/// Bound of the waits on a register (a stuck bit must not hang the firmware).
pub const WAIT_READS: u32 = 1_000_000;

/// Write a register.
#[inline(always)]
pub fn mmr_w(off: u32, value: u32) {
    // SAFETY: a register of the chip's block.
    unsafe { write_volatile((MMR_BASE + off as usize) as *mut u32, value) }
}

/// Read a register once (for registers whose read has an effect or whose
/// value keeps moving).
#[inline(always)]
pub fn mmr_r1(off: u32) -> u32 {
    // SAFETY: a register of the chip's block.
    unsafe { read_volatile((MMR_BASE + off as usize) as *const u32) }
}

/// Read a register until two reads in a row agree: a register read
/// occasionally returns a wrong value.
pub fn mmr_r(off: u32) -> u32 {
    let mut a = mmr_r1(off);
    for _ in 0..16 {
        let b = mmr_r1(off);
        if a == b {
            break;
        }
        a = b;
    }
    a
}

/// Read-modify-write of a register.
pub fn mmr_rmw(off: u32, f: impl FnOnce(u32) -> u32) {
    mmr_w(off, f(mmr_r(off)));
}

/// Wait until `(register & mask) == want`; false after [`WAIT_READS`] reads.
pub fn mmr_wait(off: u32, mask: u32, want: u32) -> bool {
    for _ in 0..WAIT_READS {
        if mmr_r(off) & mask == want {
            return true;
        }
    }
    false
}

/// Read a word of SDRAM by physical address.
#[inline(always)]
pub fn fb_r(phys: u32) -> u32 {
    // SAFETY: SDRAM through its uncached alias; the callers pass addresses
    // inside the 64 MB.
    unsafe { read_volatile((FB_BASE + (phys & 0x03FF_FFFC) as usize) as *const u32) }
}

/// Write a word of SDRAM by physical address.
#[inline(always)]
pub fn fb_w(phys: u32, value: u32) {
    // SAFETY: as `fb_r`.
    unsafe { write_volatile((FB_BASE + (phys & 0x03FF_FFFC) as usize) as *mut u32, value) }
}

/// The physical address of something in the image or its data.
pub fn phys_of<T>(p: *const T) -> u32 {
    p as u32 & 0x1FFF_FFFF
}

/// CP0 Count: runs at half the CPU clock.
#[inline(always)]
pub fn count() -> u32 {
    let v: u32;
    // SAFETY: reads a CP0 register.
    unsafe { asm!(".set noat", "mfc0 {0}, $9", ".set at", out(reg) v, options(nomem, nostack, preserves_flags)) };
    v
}

/// Write CP0 Status.
pub fn set_status(v: u32) {
    // SAFETY: the firmware runs in kernel mode with interrupts off.
    unsafe {
        asm!(".set noat", "mtc0 {0}, $12", "nop", "nop", ".set at", in(reg) v, options(nomem, nostack, preserves_flags))
    };
}

unsafe extern "C" {
    fn _exception();
}

/// Put the firmware's exception stub at the vectors (general and
/// interrupt, with and without BEV), then clear BEV. Until now the vectors
/// were the three words the host wrote, whose delay slot it leaves as it
/// was.
pub fn install_vectors() {
    let target = _exception as *const () as usize as u32;
    for v in [0x180, 0x200, 0x380, 0x400] {
        fb_w(v, LUI_K0 | target >> 16);
        fb_w(v + 4, ORI_K0 | target & 0xFFFF);
        fb_w(v + 8, JR_K0);
        fb_w(v + 12, 0);
    }
    set_status(0);
}

/// Time from CP0 Count.
pub struct Clock {
    /// Count ticks per millisecond (half the CPU clock in kHz).
    per_ms: u32,
}

impl Clock {
    pub fn new(clock_khz: u32) -> Self {
        Self { per_ms: (clock_khz / 2).max(1) }
    }

    pub fn ticks(&self, ms: u32) -> u32 {
        self.per_ms.saturating_mul(ms)
    }

    /// Busy-wait.
    pub fn delay_ms(&self, ms: u32) {
        let start = count();
        let n = self.ticks(ms);
        while count().wrapping_sub(start) < n {}
    }
}
