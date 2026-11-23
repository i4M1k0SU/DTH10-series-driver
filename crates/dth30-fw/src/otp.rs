//! Command 27: the "OTP signature" the host shows as the unit's identity.
//!
//! The chip computes it: the firmware loads the chip's fused key into key
//! slots 0 and 1, then has the transform engine process one fixed 16-byte
//! block with seven key numbers. The result (7 x 16 bytes) is whatever the
//! hardware makes of it; the firmware handles no key.

use crate::hw::{Clock, PHYS_MASK, mmr_w, mmr_wait, phys_of};

/// The engine's input block. The vendor's firmware feeds this text; the
/// same input is needed for the host to see the same signature.
#[repr(align(16))]
struct Block([u8; 16]);
static INPUT: Block = Block(*b"VIXSCONFIDENTIAL");

/// Transform engine: busy flag, source, destination, key number, command.
const ENGINE_BUSY: (u32, u32) = (0xC34, 4);
const ENGINE_SRC: u32 = 0xC80;
const ENGINE_DST: u32 = 0xC88;
const ENGINE_KEY: u32 = 0xC94;
const ENGINE_CMD: u32 = 0xC90;
/// Transform 16 bytes, both ends in SDRAM.
const CMD_BLOCK: u32 = 0x0313_0010;
/// The key numbers the block is processed with (0, 2, 4, ...): one 16-byte
/// result each.
const KEYS: u32 = 7;
const BLOCK: u32 = 16;

/// The fuse block: status (bit 30: ready), control, selection.
const OTP_STATUS: u32 = 0x1A40;
const OTP_READY: u32 = 1 << 30;
const OTP_CTRL: u32 = 0x1A30;
const OTP_SELECT: u32 = 0x1A48;
/// How long the fused key takes to load (ms).
const KEY_LOAD_MS: u32 = 30;

/// Write the 0x70-byte signature to physical address `dst`.
pub fn signature(clock: &Clock, dst: u32) {
    mmr_wait(OTP_STATUS, OTP_READY, OTP_READY);
    // Load the fused key into key slots 0 and 1; the load takes a while.
    mmr_w(OTP_SELECT, 0x0000_FFFF);
    mmr_w(OTP_CTRL, 0);
    mmr_w(OTP_CTRL, 3);
    clock.delay_ms(KEY_LOAD_MS);
    for i in 0..KEYS {
        mmr_wait(ENGINE_BUSY.0, ENGINE_BUSY.1, 0);
        mmr_w(ENGINE_SRC, phys_of(INPUT.0.as_ptr()));
        mmr_w(ENGINE_SRC + 4, 0);
        mmr_w(ENGINE_DST, (dst & PHYS_MASK) + BLOCK * i);
        mmr_w(ENGINE_DST + 4, 0);
        mmr_w(ENGINE_KEY, 2 * i);
        mmr_w(ENGINE_CMD, CMD_BLOCK);
        mmr_wait(ENGINE_BUSY.0, ENGINE_BUSY.1, 0);
    }
}
