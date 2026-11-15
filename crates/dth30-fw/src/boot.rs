//! Image header, reset entry and the exception stub.
//!
//! The host loads the image at SDRAM physical 0x0010_0000, reads the vector
//! table from it (count at +0x110, entries of four words at +0x120: where
//! to put it, then three instruction words that land at +4, +8 and +12; the
//! host clears the first word) and releases core 0, which starts at
//! physical 0 in kseg1 with the caches off. The entries here send the reset
//! to the image's first instruction and the exception vectors to stubs at
//! the same offsets in the image, all through `k0` only.

use core::arch::global_asm;
use core::ops::Range;

use crate::hw::{FB_BASE, MMR_BASE};

/// "DTH3": tells this firmware from the vendor's (image +0x100, FB 0xC0).
pub const MAGIC: u32 = 0x3348_5444;
/// Firmware version (image +0x104, FB 0xC4): major << 16 | minor.
pub const VERSION: u32 = 0x0000_0001;

/// Offsets in the image: the mark and the version, the number of vector
/// table entries, the table.
const HEAD_MAGIC: u32 = 0x100;
const HEAD_VECTOR_COUNT: u32 = 0x110;
const HEAD_VECTOR_TABLE: u32 = 0x120;

/// The image as the reset finds it: SDRAM physical 0x0010_0000, uncached.
const IMAGE_KSEG1: u32 = FB_BASE as u32 | 0x0010_0000;
/// Cached addresses start here.
const KSEG0: u32 = 0x8000_0000;

/// The words of a vector: `lui k0, hi`, `ori k0, k0, lo`, `jr k0`.
pub const LUI_K0: u32 = 0x3C1A_0000;
pub const ORI_K0: u32 = 0x375A_0000;
pub const JR_K0: u32 = 0x0340_0008;

/// The exception vectors (physical addresses, and the offsets of their
/// stubs in the image). The table has the reset vector before them.
const VECTORS: [u32; 5] = [0x180, 0x200, 0x380, 0x400, 0x480];

/// CP0 Status while booting: BEV, kernel mode, interrupts off.
const STATUS_BOOT: u32 = 1 << 22;
/// CP0 Config: the cacheability fields K23 (bits 30..28), KU (27..25) and
/// K0 (2..0); 2 = uncached, 3 = cached.
const CONFIG_KEEP: u32 = !(7 << 28 | 7 << 25 | 7);
const CONFIG_SET: u32 = 2 << 28 | 2 << 25 | 3;

/// The caches: 16 KB of instructions, 8 KB of data, 16-byte lines.
const ICACHE_BYTES: u32 = 16 << 10;
const DCACHE_BYTES: u32 = 8 << 10;
const CACHE_LINE: u32 = 16;

/// The record of a fatal error, where the vendor's firmware leaves its own
/// for the host (SDRAM physical): a count of them; the mark, where, `sp`,
/// the core; the cause code, EPC, Status, Cause, BadVAddr, `ra`.
const FATAL_COUNT: u32 = 0x520;
const FATAL_RECORD: u32 = 0x530;
const FATAL_DETAIL: u32 = 0x560;
const FATAL_MARK: u32 = 0xDEAD_C0DE;
/// All of it.
pub const FATAL_AREA: Range<u32> = 0x520..0x580;

global_asm!(
    r#"
    .section .head, "ax"
    .set noreorder
    .set noat
    .globl _reset
_reset:
    nop
    b     _boot
    nop

    .org {head_magic}
    .word {magic}
    .word {version}

    .org {head_vector_count}
    .word {vector_count}

    // The reset vector, then the exception vectors.
    .org {head_vector_table}
    .word 0, {lui_k0_image}, {jr_k0}, 0
    .irp  off, {v0}, {v1}, {v2}, {v3}, {v4}
    .word \off, {lui_k0_image}, {ori_k0} + \off, {jr_k0}
    .endr

    // Exceptions before the firmware has installed its own vectors.
    .irp  off, {v0}, {v1}, {v2}, {v3}, {v4}
    .org  \off
    b     _exception
    nop
    .endr

_boot:
    // After power-on the watch registers and Cause hold anything. A watch
    // exception left pending in Cause (WP) is taken as soon as Status.ERL
    // goes, so all of that is cleared first.
    mtc0  $zero, $18                // WatchLo: no watchpoint
    mtc0  $zero, $19                // WatchHi
    mtc0  $zero, $13                // Cause
    lui   $t0, {status_boot_hi}
    mtc0  $t0, $12                  // Status
    mtc0  $zero, $11                // Compare: drops a pending timer interrupt
    // Only core 0 runs this firmware: register 0x3214 has the core's
    // number in its bits 31..28.
    lui   $t0, {mmr_hi}
    lw    $t1, 0x3214($t0)
    srl   $t1, $t1, 28
1:  bne   $t1, $zero, 1b
    nop
    // Caches: every line gets an empty tag.
    mtc0  $zero, $28                // TagLo
    mtc0  $zero, $28, 1             // DataLo
    lui   $t0, {kseg0_hi}
    ori   $t1, $t0, {icache_bytes}
2:  cache 0x08, 0($t0)              // Index Store Tag, instructions
    addiu $t0, $t0, {cache_line}
    bne   $t0, $t1, 2b
    nop
    lui   $t0, {kseg0_hi}
    ori   $t1, $t0, {dcache_bytes}
3:  cache 0x09, 0($t0)              // Index Store Tag, data
    addiu $t0, $t0, {cache_line}
    bne   $t0, $t1, 3b
    nop
    // Config: kseg0 cached, kuseg and kseg2/3 uncached.
    mfc0  $t0, $16
    lui   $t1, {config_keep_hi}
    ori   $t1, $t1, {config_keep_lo}
    and   $t0, $t0, $t1
    lui   $t1, {config_set_hi}
    ori   $t1, $t1, {config_set_lo}
    or    $t0, $t0, $t1
    mtc0  $t0, $16
    nop
    nop
    nop
    nop
    lui   $sp, %hi(__stack_top)
    addiu $sp, $sp, %lo(__stack_top)
    lui   $gp, %hi(_gp)
    addiu $gp, $gp, %lo(_gp)
    lui   $t0, %hi(_pre_main)
    addiu $t0, $t0, %lo(_pre_main)
    jr    $t0
    nop

    // Any exception is fatal (the firmware runs with interrupts off). It
    // leaves the record described at `FATAL_COUNT`. Uses k0/k1 only and
    // runs from either segment.
    .globl _exception
_exception:
    lui   $k0, {fb_hi}
    lui   $k1, {fatal_mark_hi}
    ori   $k1, $k1, {fatal_mark_lo}
    sw    $k1, {fatal_record}($k0)
    mfc0  $k1, $14                  // EPC
    sw    $k1, {fatal_record} + 4($k0)
    sw    $k1, {fatal_detail} + 4($k0)
    sw    $sp, {fatal_record} + 8($k0)
    sw    $zero, {fatal_record} + 12($k0)
    mfc0  $k1, $13                  // Cause, and its exception code
    sw    $k1, {fatal_detail} + 12($k0)
    srl   $k1, $k1, 2
    andi  $k1, $k1, 0x1F
    sw    $k1, {fatal_detail}($k0)
    mfc0  $k1, $12                  // Status
    sw    $k1, {fatal_detail} + 8($k0)
    mfc0  $k1, $8                   // BadVAddr
    sw    $k1, {fatal_detail} + 16($k0)
    sw    $ra, {fatal_detail} + 20($k0)
    lw    $k1, {fatal_count}($k0)
    addiu $k1, $k1, 1
    sw    $k1, {fatal_count}($k0)
4:  b     4b
    nop
    .set reorder
    .set at
    .text
"#,
    magic = const MAGIC,
    version = const VERSION,
    head_magic = const HEAD_MAGIC,
    head_vector_count = const HEAD_VECTOR_COUNT,
    head_vector_table = const HEAD_VECTOR_TABLE,
    vector_count = const VECTORS.len() + 1,
    lui_k0_image = const LUI_K0 | IMAGE_KSEG1 >> 16,
    ori_k0 = const ORI_K0,
    jr_k0 = const JR_K0,
    v0 = const VECTORS[0],
    v1 = const VECTORS[1],
    v2 = const VECTORS[2],
    v3 = const VECTORS[3],
    v4 = const VECTORS[4],
    status_boot_hi = const STATUS_BOOT >> 16,
    mmr_hi = const MMR_BASE >> 16,
    kseg0_hi = const KSEG0 >> 16,
    icache_bytes = const ICACHE_BYTES,
    dcache_bytes = const DCACHE_BYTES,
    cache_line = const CACHE_LINE,
    config_keep_hi = const CONFIG_KEEP >> 16,
    config_keep_lo = const CONFIG_KEEP & 0xFFFF,
    config_set_hi = const CONFIG_SET >> 16,
    config_set_lo = const CONFIG_SET & 0xFFFF,
    fb_hi = const FB_BASE >> 16,
    fatal_mark_hi = const FATAL_MARK >> 16,
    fatal_mark_lo = const FATAL_MARK & 0xFFFF,
    fatal_record = const FATAL_RECORD,
    fatal_detail = const FATAL_DETAIL,
    fatal_count = const FATAL_COUNT,
);

unsafe extern "C" {
    /// The bounds of `.bss` (from the linker script; both 16-byte aligned).
    static mut __bss_start: u32;
    static mut __bss_end: u32;
}

/// Entered from `_boot` with the caches on and a stack: zero `.bss`, then
/// start the firmware.
#[unsafe(no_mangle)]
extern "C" fn _pre_main() -> ! {
    // SAFETY: the two symbols bound `.bss`, in one piece of the image's
    // memory and word-aligned. Nothing has used `.bss` yet, and this
    // function keeps nothing there.
    unsafe {
        let (start, end) = (&raw mut __bss_start, &raw mut __bss_end);
        core::ptr::write_bytes(start, 0, end.offset_from_unsigned(start));
    }
    crate::fw_main()
}
