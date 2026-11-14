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

/// "DTH3": tells this firmware from the vendor's (image +0x100, FB 0xC0).
pub const MAGIC: u32 = 0x3348_5444;
/// Firmware version (image +0x104, FB 0xC4): major << 16 | minor.
pub const VERSION: u32 = 0x0000_0001;

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

    .org 0x100
    .word {magic}
    .word {version}

    .org 0x110
    .word 6

    // lui k0, 0xA010 / ori k0, k0, off / jr k0
    .org 0x120
    .word 0x000, 0x3C1AA010, 0x03400008, 0x00000000
    .word 0x180, 0x3C1AA010, 0x375A0180, 0x03400008
    .word 0x200, 0x3C1AA010, 0x375A0200, 0x03400008
    .word 0x380, 0x3C1AA010, 0x375A0380, 0x03400008
    .word 0x400, 0x3C1AA010, 0x375A0400, 0x03400008
    .word 0x480, 0x3C1AA010, 0x375A0480, 0x03400008

    // Exceptions before the firmware has installed its own vectors.
    .org 0x180
    b     _exception
    nop
    .org 0x200
    b     _exception
    nop
    .org 0x380
    b     _exception
    nop
    .org 0x400
    b     _exception
    nop
    .org 0x480
    b     _exception
    nop

    .org 0x500
_boot:
    // After power-on the watch registers and Cause hold anything. A watch
    // exception left pending in Cause (WP) is taken as soon as Status.ERL
    // goes, so all of that is cleared first.
    mtc0  $zero, $18                // WatchLo: no watchpoint
    mtc0  $zero, $19                // WatchHi
    mtc0  $zero, $13                // Cause
    lui   $t0, 0x0040               // Status: BEV = 1, kernel mode, interrupts off
    mtc0  $t0, $12
    mtc0  $zero, $11                // Compare: drops a pending timer interrupt
    lui   $t0, 0xB004               // Only core 0 runs this firmware.
    lw    $t1, 0x3214($t0)
    srl   $t1, $t1, 28
1:  bne   $t1, $zero, 1b
    nop
    // Caches: 16 KB of instructions, 8 KB of data, 16-byte lines. Every
    // line gets an empty tag.
    mtc0  $zero, $28                // TagLo
    mtc0  $zero, $28, 1             // DataLo
    lui   $t0, 0x8000
    ori   $t1, $t0, 0x4000
2:  cache 0x08, 0($t0)              // Index Store Tag, instructions
    addiu $t0, $t0, 16
    bne   $t0, $t1, 2b
    nop
    lui   $t0, 0x8000
    ori   $t1, $t0, 0x2000
3:  cache 0x09, 0($t0)              // Index Store Tag, data
    addiu $t0, $t0, 16
    bne   $t0, $t1, 3b
    nop
    // Config: kseg0 cached (K0 = 3), kuseg and kseg2/3 uncached (KU = K23 = 2).
    mfc0  $t0, $16
    lui   $t1, 0x81FF
    ori   $t1, $t1, 0xFFF8
    and   $t0, $t0, $t1
    lui   $t1, 0x2400
    ori   $t1, $t1, 0x0003
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
    lui   $t0, %hi(__bss_start)
    addiu $t0, $t0, %lo(__bss_start)
    lui   $t1, %hi(__bss_end)
    addiu $t1, $t1, %lo(__bss_end)
4:  beq   $t0, $t1, 5f
    nop
    sw    $zero, 0($t0)
    b     4b
    addiu $t0, $t0, 4
5:  lui   $t0, %hi(fw_main)
    addiu $t0, $t0, %lo(fw_main)
    jr    $t0
    nop

    // Any exception is fatal (the firmware runs with interrupts off). The
    // record is what the vendor's firmware leaves for the host: FB 0x530 =
    // 0xDEADC0DE, where, sp, core; FB 0x560 = cause code, EPC, Status,
    // Cause, then BadVAddr and ra; FB 0x520 counts. Uses k0/k1 only and
    // runs from either segment.
    .globl _exception
_exception:
    lui   $k0, 0xA000
    lui   $k1, 0xDEAD
    ori   $k1, $k1, 0xC0DE
    sw    $k1, 0x530($k0)
    mfc0  $k1, $14
    sw    $k1, 0x534($k0)
    sw    $k1, 0x564($k0)
    sw    $sp, 0x538($k0)
    sw    $zero, 0x53C($k0)
    mfc0  $k1, $13
    sw    $k1, 0x56C($k0)
    srl   $k1, $k1, 2
    andi  $k1, $k1, 0x1F
    sw    $k1, 0x560($k0)
    mfc0  $k1, $12
    sw    $k1, 0x568($k0)
    mfc0  $k1, $8
    sw    $k1, 0x570($k0)
    sw    $ra, 0x574($k0)
    lw    $k1, 0x520($k0)
    addiu $k1, $k1, 1
    sw    $k1, 0x520($k0)
6:  b     6b
    nop
    .set reorder
    .set at
    .text
"#,
    magic = const MAGIC,
    version = const VERSION,
);
