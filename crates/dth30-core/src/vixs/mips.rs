//! Firmware download, MIPS start and the start-up handshake
//! (docs/26 2-vixs-bringup.md §4.3, §4.4).
//!
//! Follows the vendor's Camino path (embedded firmware, core 1 held in reset
//! first, then both cores run) access for access, including which accesses
//! are board-ID checked. The firmware goes to SDRAM window 1 and up, 1 MB per
//! DMA transfer; the vector table is then copied to window 0, which stays
//! selected from then on.
//!
//! Not verified on hardware.

use super::firmware::{Firmware, VECTOR_COUNT_OFFSET, VECTOR_TABLE_OFFSET};
use super::meminit::MemInit;
use super::mmr::{Chip, INT_ENABLE, RBM_SWAP};
use crate::bus::RemotePci;
use crate::net2280::{self, Layout};
use crate::{Error, Result};

/// MIPS-to-host interrupt status (write 1 to clear): bit0 message, bit1 command done.
pub const M2H_INT_STATUS: u32 = 0x30E0;
/// MIPS-to-host interrupt enable.
pub const M2H_INT_ENABLE: u32 = 0x30E4;
/// MIPS state (debug only).
pub const MIPS_DEBUG: u32 = 0x30E8;
/// Host-to-MIPS doorbell: function ID.
pub const H2M_FUNC: u32 = 0x3020;
/// Host-to-MIPS: acknowledged message ID (4 answers ALLOC_FB at start-up).
pub const H2M_ACK_ID: u32 = 0x3024;
/// MIPS-to-host: message ID / start-up state.
pub const M2H_ID: u32 = 0x3028;
/// MIPS-to-host: function ID of the completed command.
pub const M2H_DONE_FUNC: u32 = 0x302C;
/// Host-to-MIPS: FB address of the command slot.
pub const H2M_SLOT: u32 = 0x3030;
/// Host-to-MIPS: FB address of the acknowledged message.
pub const H2M_ACK_ADDR: u32 = 0x3038;
/// MIPS-to-host: FB address of the message (at start-up: the pool offset).
pub const M2H_ADDR: u32 = 0x3040;
/// MIPS-to-host at start-up: pool size.
pub const M2H_POOL_SIZE: u32 = 0x3044;
/// MIPS-to-host: FB address of the completed command slot.
pub const M2H_DONE_SLOT: u32 = 0x3048;
/// MIPS core control registers (bit0/bit1 = core 0/1 runs).
pub const MIPS_CTRL: [u32; 3] = [0x30F8, 0x34F8, 0x38F8];

/// Start-up states in [`M2H_ID`].
pub const STATE_OK: u32 = 0;
pub const STATE_ALLOC_FB: u32 = 5;
pub const STATE_INIT_FAILED: u32 = 7;

/// FB parameter words the host fills before the MIPS starts.
pub mod fb {
    /// Firmware heartbeat (Camino).
    pub const HEARTBEAT: u32 = 0x10;
    /// MIPS-side register address, written by the firmware.
    pub const MIPS_MMR_OUT: u32 = 0x20;
    /// Host-side (PCI) register address, written by the host.
    pub const MMR_APERTURE_IN: u32 = 0x28;
    pub const POWER_MODE: u32 = 0x40;
    pub const CLOCK_KHZ: u32 = 0x44;
    pub const MEM_SIZE: u32 = 0x48;
    pub const PARAMS_VALID: u32 = 0x4C;
}

/// Size of the DMA descriptor list at the start of the firmware's pool.
pub const DMA_DESC_SIZE: u32 = 0x840;
/// Size of the command list following it.
pub const COM_LIST_SIZE: u32 = 0x8_F000;

/// The vendor's handshake bounds: 5 idle reads per tick, 100 ticks.
const IDLE_READS_PER_TICK: u32 = 5;
const TICK_LIMIT: u32 = 100;

/// What the firmware reported at start-up and where the host put its areas.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MipsInfo {
    /// The MIPS-side register address (FB 0x20, copied to MMR 0x3008).
    pub mips_mmr: u32,
    /// FB pool offset and size reported with ALLOC_FB.
    pub pool_offset: u32,
    pub pool_size: u32,
    /// FB address of the DMA descriptor list (= pool offset).
    pub dma_desc_fb: u32,
    /// FB address of the command ring (pool offset + 0x840).
    pub cmd_ring_fb: u32,
    /// The rest of the pool as the vendor computes it.
    pub free_fb: u32,
    pub free_size: u32,
}

/// Download the firmware, start the MIPS and run the start-up handshake.
pub fn start<B: RemotePci>(chip: &mut Chip<B>, layout: &Layout, fw: &Firmware, mem: &MemInit) -> Result<MipsInfo> {
    // 1. Reset pulse.
    chip.mmr_modify_checked(0x080, |v| v | 0x80)?;
    chip.mmr_modify_checked(0x080, |v| v & !0x80)?;
    // 2. MIPS interrupts off, pending ones cleared.
    disable_interrupts(chip)?;
    // 3. Hold all cores.
    for r in MIPS_CTRL {
        chip.mmr_modify_checked(r, |v| v & 0x7FFF_FFF8)?;
    }
    // 4. Clock/reset control.
    chip.mmr_modify_checked(0x000, |v| v | 0xE0)?;
    chip.mmr_modify_checked(0x000, |v| v & !0x20)?;
    // 5. The vendor clears these FB words (the same offsets as the doorbells).
    chip.fb_write_checked(0x3024, 0)?;
    chip.fb_write_checked(0x3028, 0)?;
    // 6./7. Window 0, then window 1 for the download.
    chip.select_window(0)?;
    chip.select_window(1)?;
    // 8. Download, 1 MB per window starting at window 1.
    download(chip, layout, fw)?;
    // 9.-11. Vector table: read from window 1, write to window 0.
    chip.select_window(0)?;
    chip.select_window(1)?;
    let count = chip.fb_read_checked(VECTOR_COUNT_OFFSET)?;
    if count == 0xFFFF_FFFF {
        // The vendor returns success here without starting the MIPS.
        return Err(Error::Protocol("firmware vector table not readable from SDRAM"));
    }
    for k in 0..count {
        let e = VECTOR_TABLE_OFFSET + 16 * k;
        chip.select_window(1)?;
        let dst = chip.fb_read_checked(e)?;
        let w1 = chip.fb_read_checked(e + 4)?;
        chip.select_window(0)?;
        chip.fb_write_checked(dst.wrapping_add(4), w1)?;
        for i in [2, 3] {
            chip.select_window(1)?;
            let w = chip.fb_read_checked(e + 4 * i)?;
            chip.select_window(0)?;
            chip.fb_write_checked(dst.wrapping_add(4 * i), w)?;
        }
        chip.fb_write_checked(dst, 0)?;
    }
    // 12. Parameters, with the vendor's write test.
    chip.fb_write_checked(fb::MEM_SIZE, mem.mem_size)?;
    chip.fb_write_checked(fb::CLOCK_KHZ, mem.clock_khz)?;
    chip.fb_write_checked(fb::PARAMS_VALID, 1)?;
    let back = chip.fb_read(fb::CLOCK_KHZ)?;
    if back != mem.clock_khz {
        log::error!("FB is not writable: wrote {:#x} to 0x44, read back {back:#x}", mem.clock_khz);
        return Err(Error::Protocol("ViXS SDRAM window not writable"));
    }
    // 13. Power mode.
    chip.fb_write(fb::POWER_MODE, mem.power_mode)?;

    // 14. Release the MIPS: core 1 stays in reset first, then both run.
    chip.mmr_write_checked(0x3000, 0x0FFF_FFFF)?;
    chip.mmr_write_checked(0x3008, 0x1FFF_0004)?;
    chip.mmr_write_checked(0x3010, 0x110)?;
    chip.mmr_write_checked(0x3088, 0)?;
    let ctrl = chip.mmr_read_checked(MIPS_CTRL[0])? & !2;
    chip.fb_write_checked(fb::MMR_APERTURE_IN, 0)?;
    chip.fb_write_checked(fb::MIPS_MMR_OUT, 0)?;
    chip.mmr_write_checked(MIPS_CTRL[0], ctrl)?;
    let ctrl = chip.mmr_read_checked(MIPS_CTRL[0])? | 3;
    chip.mmr_write_checked(MIPS_CTRL[0], ctrl)?;
    chip.wait_ticks(1);

    // 15. Wait for the firmware to report its register address.
    wait_mips_mmr(chip)?;
    // 16. Exchange the register addresses.
    let mips_mmr = chip.fb_read(fb::MIPS_MMR_OUT)?;
    chip.mmr_write(0x3008, mips_mmr)?;
    chip.fb_write(fb::MMR_APERTURE_IN, chip.mmr)?;

    // 17. ALLOC_FB.
    wait_state(chip, true, |s| (s != STATE_OK).then_some(s == STATE_ALLOC_FB), "ALLOC_FB")?;
    let pool_offset = chip.mmr_read_checked(M2H_ADDR)?;
    let pool_size = chip.mmr_read_checked(M2H_POOL_SIZE)?;
    chip.mmr_write_checked(H2M_ACK_ID, 4)?;
    // 18. The host's areas in the pool (the vendor's arithmetic).
    let cmd_ring_fb = pool_offset.wrapping_add(DMA_DESC_SIZE);
    let end = cmd_ring_fb.wrapping_add(COM_LIST_SIZE);
    let free_size = pool_size.wrapping_sub(DMA_DESC_SIZE + COM_LIST_SIZE + (end & 0xFFF));
    let free_fb = end.wrapping_add(0xFFF) & 0xFFFF_F000;
    log::debug!("MIPS pool {pool_offset:#x}+{pool_size:#x}; command ring at {cmd_ring_fb:#x}");

    // 19. The last start-up message: 0 (OK) or 7 (INIT_FAILED).
    let last = wait_state(
        chip,
        false,
        |s| matches!(s, STATE_OK | STATE_INIT_FAILED).then_some(true),
        "TEST_INT || INIT_FAILED",
    )?;
    // 20. Acknowledge it.
    ack_m2h(chip)?;
    // 21.
    if last == STATE_INIT_FAILED {
        return Err(Error::Protocol("ViXS firmware reported INIT_FAILED"));
    }
    enable_interrupts(chip)?;
    Ok(MipsInfo { mips_mmr, pool_offset, pool_size, dma_desc_fb: pool_offset, cmd_ring_fb, free_fb, free_size })
}

/// The download: chunk `n` goes to window `n + 1` through BAR0 + 0.
fn download<B: RemotePci>(chip: &mut Chip<B>, layout: &Layout, fw: &Firmware) -> Result<()> {
    for (n, chunk) in fw.image().chunks(net2280::DMA_MAX).enumerate() {
        if n == 0 {
            chip.mmr_read(RBM_SWAP)?; // logged by the vendor
        } else {
            let v = chip.mmr_read(RBM_SWAP)?;
            chip.mmr_write_checked(RBM_SWAP, (((n as u32) + 1) << 8) | (v & 0xFFFF_80FF))?;
            chip.mmr_read(RBM_SWAP)?;
        }
        net2280::dma_write(chip.bus(), layout, chip.fb, chunk)?;
    }
    Ok(())
}

/// Step 15: FB 0x20 must read non-zero and the same twice in a row, three
/// times; every 5 zero reads cost one tick, 100 ticks is a timeout.
fn wait_mips_mmr<B: RemotePci>(chip: &mut Chip<B>) -> Result<()> {
    let (mut stable, mut zeros, mut ticks) = (0, 0, 0);
    loop {
        let v = chip.fb_read(fb::MIPS_MMR_OUT)?;
        zeros += 1;
        if v != 0 {
            zeros = 0;
            if chip.fb_read(fb::MIPS_MMR_OUT)? == v {
                stable += 1;
            }
        }
        if zeros == IDLE_READS_PER_TICK {
            zeros = 0;
            ticks += 1;
            chip.wait_ticks(1);
        }
        if ticks == TICK_LIMIT {
            log::error!("MIPS start: no register address from the firmware (MIPS_M2H_MMR)");
            return Err(Error::Timeout);
        }
        if stable >= 3 {
            return Ok(());
        }
    }
}

/// Steps 17 and 19: poll the start-up state in MMR 0x3028 (checked) until
/// `classify` has accepted three reads. `classify` returns `None` for an
/// idle read, `Some(true)` for an accepted one and `Some(false)` for a read
/// that neither counts nor idles. With `debug`, MMR 0x30E8 is read before
/// each poll as the vendor does during ALLOC_FB.
fn wait_state<B: RemotePci>(
    chip: &mut Chip<B>,
    debug: bool,
    classify: impl Fn(u32) -> Option<bool>,
    what: &str,
) -> Result<u32> {
    let (mut hits, mut idle, mut ticks) = (0, 0, 0);
    loop {
        if debug {
            let s = chip.mmr_read(MIPS_DEBUG)?;
            log::trace!("MIPS state 0x30E8 = {s:#x}");
        }
        let v = chip.mmr_read_checked(M2H_ID)?;
        idle += 1;
        match classify(v) {
            Some(true) => {
                idle = 0;
                hits += 1;
            }
            Some(false) => idle = 0,
            None => {}
        }
        if hits >= 3 {
            return Ok(v);
        }
        if idle == IDLE_READS_PER_TICK {
            idle = 0;
            ticks += 1;
            chip.wait_ticks(1);
        }
        if ticks == TICK_LIMIT {
            log::error!("MIPS start: timeout waiting for {what}");
            return Err(Error::Timeout);
        }
    }
}

/// Acknowledge the current MIPS-to-host message.
pub fn ack_m2h<B: RemotePci>(chip: &mut Chip<B>) -> Result<()> {
    let addr = chip.mmr_read_checked(M2H_ADDR)?;
    let id = chip.mmr_read_checked(M2H_ID)?;
    chip.mmr_write_checked(H2M_ACK_ADDR, addr)?;
    chip.mmr_write_checked(H2M_ACK_ID, id)
}

pub fn disable_interrupts<B: RemotePci>(chip: &mut Chip<B>) -> Result<()> {
    chip.mmr_modify_checked(INT_ENABLE, |v| v & !0x200)?;
    chip.mmr_modify_checked(M2H_INT_ENABLE, |v| v & !3)?;
    chip.mmr_modify_checked(M2H_INT_STATUS, |v| v | 0x8000_0007)
}

pub fn enable_interrupts<B: RemotePci>(chip: &mut Chip<B>) -> Result<()> {
    chip.mmr_modify_checked(M2H_INT_STATUS, |v| v | 0x8000_0007)?;
    chip.mmr_modify_checked(INT_ENABLE, |v| v | 0x200)?;
    chip.mmr_modify_checked(M2H_INT_ENABLE, |v| v | 3)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::super::mmr::testutil::*;
    use super::super::tests::layout;
    use super::*;
    use crate::bus::mock::{MockPci, Space};

    pub(crate) fn fw(len: usize) -> Firmware {
        let mut img = vec![0u8; len];
        img[0x110..0x114].copy_from_slice(&2u32.to_le_bytes());
        Firmware::unverified(img)
    }

    const MEM: MemInit = MemInit { power_mode: 6, clock_khz: 182_250, mem_size: 0x0400_0000 };

    /// Script the firmware's side of a successful start.
    pub(crate) fn script_firmware(m: &MockPci) {
        // Vector table as it reads back from SDRAM window 1.
        m.set(Space::Mem, FB + 0x110, 2);
        for (k, e) in [[0x180u32, 0xA1, 0xA2, 0xA3], [0x200, 0xB1, 0xB2, 0xB3]].iter().enumerate() {
            for (i, w) in e.iter().enumerate() {
                m.set(Space::Mem, FB + 0x120 + 16 * k as u32 + 4 * i as u32, *w);
            }
        }
        // MIPS register address: one zero, then stable; one more read after.
        m.queue(Space::Mem, FB + 0x20, &[0, 0x1FFF_0004, 0x1FFF_0004, 0x1FFF_0004, 0x1FFF_0004]);
        m.queue(Space::Mem, FB + 0x20, &[0x1FFF_0004, 0x1FFF_0004, 0x1FFF_0004]);
        // ALLOC_FB three times, then OK three times, then the ack read.
        m.queue(Space::Mem, MMR + 0x3028, &[0, 5, 5, 5, 0, 0, 0, 0]);
        m.set(Space::Mem, MMR + 0x3040, 0x0010_0000);
        m.set(Space::Mem, MMR + 0x3044, 0x0080_0000);
    }

    #[test]
    fn full_start_sequence() {
        let mut c = chip(0x0300_00B2);
        c.bus().set(Space::Mem, MMR + 0x30F8, 0x8000_0007);
        c.bus().set(Space::Mem, MMR + 0x43C, 0x0002_0000);
        script_firmware(c.bus());
        let info = start(&mut c, &layout(), &fw(0x18_0000), &MEM).unwrap();
        assert_eq!(
            info,
            MipsInfo {
                mips_mmr: 0x1FFF_0004,
                pool_offset: 0x10_0000,
                pool_size: 0x80_0000,
                dma_desc_fb: 0x10_0000,
                cmd_ring_fb: 0x10_0840,
                free_fb: 0x19_0000,
                free_size: 0x80_0000 - 0x840 - 0x8F000 - 0x840,
            }
        );
        let acc = accesses(&c.bus().ops());
        use Acc::*;
        let win = |n: u32| [R(0x43C), W(0x43C, 0x0002_0000 | n << 8), R(0x43C)];
        let mut want = vec![
            R(0x080),
            W(0x080, 0x80),
            R(0x080),
            W(0x080, 0),
            R(0x418),
            W(0x418, 0),
            R(0x30E4),
            W(0x30E4, 0),
            R(0x30E0),
            W(0x30E0, 0x8000_0007),
            R(0x30F8),
            W(0x30F8, 0),
            R(0x34F8),
            W(0x34F8, 0),
            R(0x38F8),
            W(0x38F8, 0),
            R(0x000),
            W(0x000, 0xE0),
            R(0x000),
            W(0x000, 0xC0),
            FbW(0x3024, 0),
            FbW(0x3028, 0),
        ];
        want.extend(win(0));
        want.extend(win(1));
        // Chunk 0 to window 1, chunk 1 to window 2.
        want.extend([
            R(0x43C),
            Csr(0x194, FB),
            Csr(0x190, 0x10_0000),
            Csr(0x180, 2),
            Csr(0x184, 1),
            Bulk(0x04, 0x10_0000),
            R(0x43C),
            W(0x43C, 0x0002_0200),
            R(0x43C),
            Csr(0x194, FB),
            Csr(0x190, 0x8_0000),
            Csr(0x180, 2),
            Csr(0x184, 1),
            Bulk(0x04, 0x8_0000),
        ]);
        want.extend(win(0));
        want.extend(win(1));
        want.push(FbR(0x110));
        for (k, dst, w) in [(0u32, 0x180u32, [0xA1, 0xA2, 0xA3]), (1, 0x200, [0xB1, 0xB2, 0xB3])] {
            let e = 0x120 + 16 * k;
            want.extend(win(1));
            want.extend([FbR(e), FbR(e + 4)]);
            want.extend(win(0));
            want.push(FbW(dst + 4, w[0]));
            want.extend(win(1));
            want.push(FbR(e + 8));
            want.extend(win(0));
            want.push(FbW(dst + 8, w[1]));
            want.extend(win(1));
            want.push(FbR(e + 12));
            want.extend(win(0));
            want.extend([FbW(dst + 12, w[2]), FbW(dst, 0)]);
        }
        want.extend([
            FbW(0x48, 0x0400_0000),
            FbW(0x44, 182_250),
            FbW(0x4C, 1),
            FbR(0x44),
            FbW(0x40, 6),
            W(0x3000, 0x0FFF_FFFF),
            W(0x3008, 0x1FFF_0004),
            W(0x3010, 0x110),
            W(0x3088, 0),
            R(0x30F8),
            FbW(0x28, 0),
            FbW(0x20, 0),
            W(0x30F8, 0),
            R(0x30F8),
            W(0x30F8, 3),
            // M2H_MMR: zero, (v, v) x3.
            FbR(0x20),
            FbR(0x20),
            FbR(0x20),
            FbR(0x20),
            FbR(0x20),
            FbR(0x20),
            FbR(0x20),
            // Exchange.
            FbR(0x20),
            W(0x3008, 0x1FFF_0004),
            FbW(0x28, MMR),
        ]);
        for _ in 0..4 {
            want.extend([R(0x30E8), R(0x3028)]);
        }
        want.extend([R(0x3040), R(0x3044), W(0x3024, 4)]);
        want.extend([R(0x3028), R(0x3028), R(0x3028)]);
        want.extend([R(0x3040), R(0x3028), W(0x3038, 0x10_0000), W(0x3024, 0)]);
        want.extend([R(0x30E0), W(0x30E0, 0x8000_0007), R(0x418), W(0x418, 0x200), R(0x30E4), W(0x30E4, 3)]);
        assert_eq!(acc, want);
        // Everything but the plain accesses is board-ID checked.
        // Plain: RBM_SWAP reads around the download (3), the FB write test and
        // Power_Mode (2), FB 0x20 reads (8), the address exchange (2) and the
        // 0x30E8 debug reads (4).
        let plain = 3 + 2 + 8 + 2 + 4;
        let all = acc.iter().filter(|a| !matches!(a, Csr(..) | Bulk(..))).count();
        assert_eq!(checks(&c.bus().ops()), all - plain);
    }

    #[test]
    fn init_failed_is_an_error() {
        let mut c = chip(0x0300_00B2);
        c.bus().set(Space::Mem, FB + 0x110, 0);
        c.bus().queue(Space::Mem, FB + 0x20, &[1; 8]);
        c.bus().queue(Space::Mem, MMR + 0x3028, &[5, 5, 5, 7, 7, 7, 7]);
        let e = start(&mut c, &layout(), &fw(0x200), &MEM).unwrap_err();
        std::assert_matches!(e, Error::Protocol(m) if m.contains("INIT_FAILED"));
        // The message was still acknowledged, and interrupts stay off.
        let w = writes(&c.bus().ops());
        assert!(w.contains(&Acc::W(0x3024, 7)));
        assert!(!w.contains(&Acc::W(0x30E4, 3)));
    }

    #[test]
    fn mips_mmr_timeout_after_100_ticks() {
        let mut c = chip(0x0300_00B2);
        c.bus().set(Space::Mem, FB + 0x110, 0);
        std::assert_matches!(start(&mut c, &layout(), &fw(0x200), &MEM), Err(Error::Timeout));
        let reads = accesses(&c.bus().ops()).iter().filter(|a| **a == Acc::FbR(0x20)).count();
        assert_eq!(reads, 500);
    }

    #[test]
    fn fb_write_test_failure() {
        let mut c = chip(0x0300_00B2);
        c.bus().queue(Space::Mem, FB + 0x44, &[0]);
        let e = start(&mut c, &layout(), &fw(0x200), &MEM).unwrap_err();
        std::assert_matches!(e, Error::Protocol(m) if m.contains("not writable"));
    }

    #[test]
    fn unreadable_vector_count() {
        let mut c = chip(0x0300_00B2);
        c.bus().set(Space::Mem, FB + 0x110, 0xFFFF_FFFF);
        assert!(start(&mut c, &layout(), &fw(0x200), &MEM).is_err());
    }
}
