//! The PLX NET2280 in PCI host mode: register map and the bring-up the vendor driver performs.
//!
//! The vendor stack does no hardware set-up in its USB layer; the function
//! driver programs the NET2280 itself when the device starts (docs/26,
//! 1-usb-net2280.md §7). [`hw_init`] repeats that sequence in the same order
//! with the same values, [`dma_write`] is the host-to-ViXS memory path (the
//! NET2280 DMA engine fed by bulk OUT), and [`IrqState::wait`] implements the
//! one-shot interrupt endpoint with the vendor's re-arm sequence.
//!
//! None of this is verified on hardware.

use core::time::Duration;

use crate::bus::RemotePci;
use crate::{Error, Result};

/// NET2280 memory-mapped registers (its BAR0 space, reached with `csr_*`).
pub mod reg {
    /// DEVINIT: PCI reset / soft reset / PCI enable.
    pub const DEVINIT: u16 = 0x00;
    /// USBIRQENB1: interrupt enables reported on the status endpoint.
    pub const USBIRQENB1: u16 = 0x24;
    /// IRQSTAT1: interrupt status (write 1 to clear).
    pub const IRQSTAT1: u16 = 0x2C;
    /// FIFOCTL: PCI BASE2 range (the FIFO window size).
    pub const FIFOCTL: u16 = 0x38;
    /// GPIOCTL (only touched for the NetChip evaluation product IDs).
    pub const GPIOCTL: u16 = 0x50;
    /// DMA channel A registers; channels B..D follow at +0x20 each.
    pub const DMACTL: u16 = 0x180;
    pub const DMASTAT: u16 = 0x184;
    pub const DMACOUNT: u16 = 0x190;
    pub const DMAADDR: u16 = 0x194;
    /// EP_CFG of programmable endpoint `n` (1..=4 = A..D) is at
    /// `EP_CFG + 0x20 * n`, with EP_RSP at +4 and EP_STAT at +0xC.
    pub const EP_BASE: u16 = 0x300;
    pub const EP_RSP: u16 = 0x04;
    pub const EP_STAT: u16 = 0x0C;
    /// EP_CFG of endpoint A..D (the registers [`super::hw_init`] reads back).
    pub const EP_CFG_A: u16 = 0x320;
    /// The endpoint register blocks whose halt the vendor clears after a PCI
    /// abort (the two dedicated PCI endpoints and endpoints A..D).
    pub const HALT_CLEAR_BLOCKS: [u16; 6] = [0x220, 0x230, 0x320, 0x340, 0x360, 0x380];
}

/// NET2280's own PCI configuration registers (`local_cfg_*`).
pub mod cfg {
    pub const COMMAND: u16 = 0x04;
    pub const BAR2: u16 = 0x18;
}

/// Configuration addresses of the remote (ViXS) device are `REMOTE_CFG | register`.
pub const REMOTE_CFG: u32 = 0x1_0000;
/// USBIRQENB1 value: USB interrupt enable (bit 31) and PCI INTA (bit 24).
pub const IRQ_ENABLE: u32 = 0x8100_0000;
/// Interrupt status bit of the remote INTA# (the ViXS chip).
pub const IRQ_PCI_INTA: u32 = 0x0100_0000;
/// Interrupt status bits of a PCI target / master abort.
pub const IRQ_PCI_ABORT: u32 = 0x0018_0000;
/// The PCI address of the NET2280 FIFO window (its BAR2, 1 MB).
pub const FIFO_WINDOW: u32 = 0x3000_0000;
/// Each of FIFO A..D gets 256 KB of the window.
pub const FIFO_STRIDE: u32 = 0x4_0000;
/// The vendor's largest single DMA transfer.
pub const DMA_MAX: usize = 0x10_0000;
/// First PCI address the BAR assignment hands out.
pub const BAR_BASE: u32 = 0x1000_0000;
/// The vendor's wait unit (its interrupt-time divider): 10 ms.
pub const TICK: Duration = Duration::from_millis(10);

/// The product IDs for which the vendor pulses GPIOCTL first (not BUFFALO's).
const GPIO_RESET_PIDS: [u16; 4] = [0xA872, 0xA873, 0x2202, 0x2201];
/// The NetChip evaluation product IDs, which use another endpoint set.
const NETCHIP_PIDS: [u16; 2] = [0xA870, 0xA872];

/// The vendor's busy-wait of `n` ticks: it waits until the 10 ms counter has
/// advanced by more than `n`, so at most `n + 1` ticks.
pub fn ticks(n: u32) -> Duration {
    TICK * (n + 1)
}

/// What [`hw_init`] set up.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Layout {
    /// The ViXS chip's vendor/device ID (configuration register 0, logged only).
    pub vixs_id: u32,
    /// BAR assignment: `(config offset, sizing read-back, assigned address)`
    /// for every BAR that was assigned.
    pub bars: Vec<(u16, u32, u32)>,
    /// ViXS BAR0 (the 1 MB SDRAM window, "FB").
    pub fb: u32,
    /// ViXS BAR1 (the register space, "MMR").
    pub mmr: u32,
    /// The endpoint addresses of FIFO A..D.
    pub endpoints: [u8; 4],
    /// PCI addresses of FIFO A..D.
    pub fifo: [u32; 4],
    /// EP_CFG of endpoints A..D as read back after configuration (logged only).
    pub ep_cfg: [u32; 4],
    /// Register offset of the DMA channel serving endpoint A (0x00/0x20/0x40/0x60).
    pub dma_offset: u16,
    /// Whether a DMA channel was mapped to endpoint A's address. If not, the
    /// vendor stack still uses offset 0 (channel A), and so does [`dma_write`].
    pub dma_mapped: bool,
}

impl Layout {
    /// The bulk OUT endpoint that feeds the DMA engine (FIFO A).
    pub fn ep_out(&self) -> u8 {
        self.endpoints[0]
    }
}

/// The endpoint addresses of FIFO A..D for a product ID, and whether each is
/// configured as OUT (the vendor's per-endpoint flag).
pub fn endpoints_for(pid: u16) -> ([u8; 4], [bool; 4]) {
    if NETCHIP_PIDS.contains(&pid) {
        ([0x01, 0x02, 0x03, 0x81], [true, true, true, false])
    } else {
        ([0x04, 0x82, 0x83, 0x84], [true, false, false, false])
    }
}

/// Bring up the NET2280 and the remote PCI bus as the vendor driver does
/// when the device starts (docs/26 1-usb-net2280.md §7, steps 1 to 15).
///
/// `pipes` are the endpoint addresses present in the device's active
/// configuration (the DMA channel of an endpoint is only mapped when its pipe
/// exists, as in the vendor's USB layer); `sleep` performs the waits.
pub fn hw_init<B: RemotePci + ?Sized>(bus: &B, pid: u16, pipes: &[u8], sleep: fn(Duration)) -> Result<Layout> {
    // 1. NetChip evaluation boards only: GPIO power sequence.
    if GPIO_RESET_PIDS.contains(&pid) {
        let v = bus.csr_read(reg::GPIOCTL)? & !0xF;
        bus.csr_write(reg::GPIOCTL, v)?;
        sleep(Duration::from_millis(80));
        for (bit, wait) in [(1, 50), (2, 50), (8, 0)] {
            let v = bus.csr_read(reg::GPIOCTL)? | bit;
            bus.csr_write(reg::GPIOCTL, v)?;
            if wait != 0 {
                sleep(Duration::from_millis(wait));
            }
        }
    }

    // 2. DEVINIT pulse: PCI reset + soft reset with PCI disabled, then enable.
    let devinit = bus.csr_read(reg::DEVINIT)?;
    bus.csr_write(reg::DEVINIT, (devinit & !0x20) | 0x84)?;
    sleep(ticks(10));
    bus.csr_write(reg::DEVINIT, (devinit & !0x84) | 0x20)?;

    // 3./4. Command registers: NET2280 MEM|MASTER|MWI, ViXS IO|MEM|MASTER.
    bus.local_cfg_write(cfg::COMMAND, 0x16)?;
    bus.cfg_write(REMOTE_CFG | 0x04, 7)?;
    // 5. ViXS vendor/device ID (not checked by the vendor).
    let vixs_id = bus.cfg_read(REMOTE_CFG)?;

    // 6. BAR sizing and assignment on 256 MB boundaries.
    let mut base = BAR_BASE;
    let mut bars = Vec::new();
    for off in [0x10u16, 0x14, 0x18, 0x1C, 0x20, 0x24, 0x30] {
        let want_bit0 = u32::from(off == 0x30);
        let addr = REMOTE_CFG | u32::from(off);
        bus.cfg_write(addr, 0xFFFF_FFFF)?;
        let v = bus.cfg_read(addr)?;
        if v >= 0x10 && (v & 1) == want_bit0 {
            bus.cfg_write(addr, base)?;
            bars.push((off, v, base));
            let mut n = base.wrapping_sub(v);
            if n & 0x0FFF_FFFF != 0 {
                n = n.wrapping_add(0x1000_0000);
            }
            base = n & 0xF000_0000;
        }
    }

    // 7. Map DMA channels to the endpoints configured from the EEPROM.
    let (endpoints, out) = endpoints_for(pid);
    let mut dma_offset = 0;
    let mut dma_mapped = false;
    for ep_cfg in [0x320u16, 0x340, 0x360, 0x380] {
        let v = bus.csr_read(ep_cfg)?;
        let ep = (v & 0x0F) as u8 | (v & 0x80) as u8;
        if pipes.contains(&ep) {
            let off = ep_cfg - reg::EP_CFG_A;
            if ep == endpoints[0] {
                dma_offset = off;
                dma_mapped = true;
            }
            bus.csr_write(reg::DMASTAT + off, 2)?; // DMA abort
            bus.csr_write(ep_cfg + reg::EP_STAT, 0x200)?; // FIFO flush
        }
    }

    // 8./9. ViXS command again, then the interrupt enables.
    bus.cfg_write(REMOTE_CFG | 0x04, 7)?;
    bus.csr_write(reg::USBIRQENB1, IRQ_ENABLE)?;
    // 10. Dummy read at the first BAR address.
    bus.mem_read(BAR_BASE)?;
    // 11./12. FIFO window: 1 MB at FIFO_WINDOW.
    bus.csr_write(reg::FIFOCTL, 0xFFF0_0000)?;
    bus.local_cfg_write(cfg::BAR2, FIFO_WINDOW)?;

    // 13. Endpoints A..D.
    for (i, (&ep, &is_out)) in endpoints.iter().zip(out.iter()).enumerate() {
        let blk = reg::EP_BASE + 0x20 * (i as u16 + 1);
        bus.csr_write(blk, u32::from(ep) | 0x600)?;
        bus.csr_write(blk + reg::EP_STAT, 0x003F_127F)?;
        bus.csr_write(blk + reg::EP_RSP, 0x80)?;
        if is_out {
            bus.csr_write(blk + reg::EP_RSP, 0x04)?;
        }
    }
    // 14. Read back EP_CFG (logged by the vendor).
    let mut ep_cfg = [0; 4];
    for (i, v) in ep_cfg.iter_mut().enumerate() {
        *v = bus.csr_read(reg::EP_CFG_A + 0x20 * i as u16)?;
    }
    // 15. Where the BARs ended up.
    let fb = bus.cfg_read(REMOTE_CFG | 0x10)? & !0xF;
    let mmr = bus.cfg_read(REMOTE_CFG | 0x14)? & !0xF;

    let fifo = [0, 1, 2, 3].map(|i| FIFO_WINDOW + FIFO_STRIDE * i);
    if !dma_mapped {
        log::warn!("no NET2280 DMA channel serves endpoint {:#04x}; using channel A", endpoints[0]);
    }
    log::debug!("NET2280 up: ViXS id {vixs_id:08x}, FB {fb:08x}, MMR {mmr:08x}, EP_CFG {ep_cfg:08x?}");
    Ok(Layout { vixs_id, bars, fb, mmr, endpoints, fifo, ep_cfg, dma_offset, dma_mapped })
}

/// Write `data` (at most [`DMA_MAX`] bytes) to the remote PCI address
/// `pci_addr` through the NET2280 DMA engine: program the channel serving
/// endpoint A, then send the data on that bulk OUT endpoint.
pub fn dma_write<B: RemotePci + ?Sized>(bus: &B, layout: &Layout, pci_addr: u32, data: &[u8]) -> Result<()> {
    if data.len() > DMA_MAX {
        return Err(Error::TooLarge { limit: DMA_MAX, actual: data.len() });
    }
    let ch = layout.dma_offset;
    bus.csr_write(reg::DMAADDR + ch, pci_addr)?;
    bus.csr_write(reg::DMACOUNT + ch, data.len() as u32)?; // bit 30 clear: USB to PCI
    bus.csr_write(reg::DMACTL + ch, 2)?; // DMA enable
    bus.csr_write(reg::DMASTAT + ch, 1)?; // DMA start
    bus.bulk_out(layout.ep_out(), data)
}

/// The re-arm sequence of the one-shot status endpoint: clear the previously
/// received status bits in IRQSTAT1 with the interrupt enables off.
pub fn rearm_interrupt<B: RemotePci + ?Sized>(bus: &B, last_status: u32) -> Result<()> {
    let save = bus.csr_read(reg::USBIRQENB1)?;
    bus.csr_write(reg::USBIRQENB1, 0)?;
    bus.csr_write(reg::IRQSTAT1, last_status)?;
    bus.csr_write(reg::USBIRQENB1, save)
}

/// After a PCI abort, clear the device-side halt of the PCI endpoints and
/// endpoints A..D (the vendor does not reset the host-side pipes).
pub fn clear_abort_halts<B: RemotePci + ?Sized>(bus: &B) -> Result<()> {
    for blk in reg::HALT_CLEAR_BLOCKS {
        let rsp = blk + reg::EP_RSP;
        if bus.csr_read(rsp)? & 1 != 0 {
            bus.csr_write(rsp, 1)?;
        }
    }
    Ok(())
}

/// State of the interrupt (status) endpoint.
///
/// The vendor arms the endpoint once, and again after each status word it
/// handled; the transport keeps a timed-out read pending, so a timeout does
/// not re-arm.
#[derive(Debug, Default, Clone)]
pub struct IrqState {
    /// The status word received last (written back to IRQSTAT1 on re-arm).
    pub last: u32,
    armed: bool,
}

impl IrqState {
    pub fn new() -> Self {
        Self::default()
    }

    /// Wait up to `timeout` for one status word, arming first if needed.
    /// A PCI abort in the status clears the endpoint halts before returning.
    pub fn wait<B: RemotePci + ?Sized>(&mut self, bus: &B, timeout: Duration) -> Result<Option<u32>> {
        if !self.armed {
            rearm_interrupt(bus, self.last)?;
            self.armed = true;
        }
        let Some(stat) = bus.interrupt_status(timeout)? else {
            return Ok(None);
        };
        self.last = stat;
        self.armed = false;
        if stat & IRQ_PCI_ABORT != 0 {
            log::warn!("NET2280 reports a PCI abort (status {stat:08x})");
            clear_abort_halts(bus)?;
        }
        Ok(Some(stat))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::mock::{MockPci, Op, Space};

    fn no_sleep(_: Duration) {}

    const PIPES: [u8; 9] = [0x0D, 0x8D, 0x0E, 0x8E, 0x8F, 0x04, 0x82, 0x83, 0x84];

    /// Script a ViXS chip with a 1 MB BAR0, a 64 KB BAR1 and nothing else.
    fn scripted() -> MockPci {
        let m = MockPci::new();
        m.set(Space::Csr, 0x00, 0x0000_0201);
        m.set(Space::Cfg, 0x1_0000, 0x0002_1745);
        m.queue(Space::Cfg, 0x1_0010, &[0xFFF0_0000]);
        m.queue(Space::Cfg, 0x1_0014, &[0xFFFF_0000]);
        for off in [0x18, 0x1C, 0x20, 0x24, 0x30] {
            m.queue(Space::Cfg, 0x1_0000 + off, &[0]);
        }
        // EEPROM defaults of EP_CFG A..D (endpoint numbers 4, 2, 3, 4|IN).
        m.set(Space::Csr, 0x320, 0x0000_0004);
        m.set(Space::Csr, 0x340, 0x0000_0082);
        m.set(Space::Csr, 0x360, 0x0000_0083);
        m.set(Space::Csr, 0x380, 0x0000_0084);
        m
    }

    #[test]
    fn hw_init_buffalo_sequence() {
        let m = scripted();
        let l = hw_init(&m, 0x0128, &PIPES, no_sleep).unwrap();
        use Op::*;
        let mut want = vec![
            CsrRead(0x00),
            CsrWrite(0x00, 0x0000_0285),
            CsrWrite(0x00, 0x0000_0221),
            LocalCfgWrite(0x04, 0x16),
            CfgWrite(0x1_0004, 7),
            CfgRead(0x1_0000),
            CfgWrite(0x1_0010, 0xFFFF_FFFF),
            CfgRead(0x1_0010),
            CfgWrite(0x1_0010, 0x1000_0000),
            CfgWrite(0x1_0014, 0xFFFF_FFFF),
            CfgRead(0x1_0014),
            CfgWrite(0x1_0014, 0x2000_0000),
        ];
        for off in [0x18, 0x1C, 0x20, 0x24, 0x30] {
            want.push(CfgWrite(0x1_0000 + off, 0xFFFF_FFFF));
            want.push(CfgRead(0x1_0000 + off));
        }
        for (i, r) in [0x320u16, 0x340, 0x360, 0x380].into_iter().enumerate() {
            want.push(CsrRead(r));
            want.push(CsrWrite(0x184 + 0x20 * i as u16, 2));
            want.push(CsrWrite(r + 0xC, 0x200));
        }
        want.extend([
            CfgWrite(0x1_0004, 7),
            CsrWrite(0x24, 0x8100_0000),
            MemRead(0x1000_0000),
            CsrWrite(0x38, 0xFFF0_0000),
            LocalCfgWrite(0x18, 0x3000_0000),
            CsrWrite(0x320, 0x604),
            CsrWrite(0x32C, 0x3F_127F),
            CsrWrite(0x324, 0x80),
            CsrWrite(0x324, 0x04),
            CsrWrite(0x340, 0x682),
            CsrWrite(0x34C, 0x3F_127F),
            CsrWrite(0x344, 0x80),
            CsrWrite(0x360, 0x683),
            CsrWrite(0x36C, 0x3F_127F),
            CsrWrite(0x364, 0x80),
            CsrWrite(0x380, 0x684),
            CsrWrite(0x38C, 0x3F_127F),
            CsrWrite(0x384, 0x80),
            CsrRead(0x320),
            CsrRead(0x340),
            CsrRead(0x360),
            CsrRead(0x380),
            CfgRead(0x1_0010),
            CfgRead(0x1_0014),
        ]);
        assert_eq!(m.ops(), want);
        assert_eq!(l.fb, 0x1000_0000);
        assert_eq!(l.mmr, 0x2000_0000);
        assert_eq!(l.vixs_id, 0x0002_1745);
        assert_eq!(l.bars, vec![(0x10, 0xFFF0_0000, 0x1000_0000), (0x14, 0xFFFF_0000, 0x2000_0000)]);
        assert_eq!(l.endpoints, [0x04, 0x82, 0x83, 0x84]);
        assert_eq!(l.fifo, [0x3000_0000, 0x3004_0000, 0x3008_0000, 0x300C_0000]);
        assert_eq!((l.dma_offset, l.dma_mapped), (0, true));
        assert_eq!(l.ep_cfg, [0x604, 0x682, 0x683, 0x684]);
    }

    #[test]
    fn hw_init_maps_dma_channel_from_eeprom_and_pipes() {
        let m = scripted();
        // EEPROM puts endpoint 4 on channel C; only 0x04 has a pipe.
        m.set(Space::Csr, 0x320, 0x82);
        m.set(Space::Csr, 0x360, 0x04);
        let l = hw_init(&m, 0x0129, &[0x04], no_sleep).unwrap();
        assert_eq!((l.dma_offset, l.dma_mapped), (0x40, true));
        let w = m.writes();
        assert!(w.contains(&Op::CsrWrite(0x1C4, 2)));
        assert!(w.contains(&Op::CsrWrite(0x36C, 0x200)));
        assert!(!w.contains(&Op::CsrWrite(0x184, 2)));
    }

    #[test]
    fn bar_assignment_rounds_to_256mb_and_takes_rom() {
        let m = MockPci::new();
        m.queue(Space::Cfg, 0x1_0010, &[0xFFF0_0008]); // 1 MB, prefetchable
        m.queue(Space::Cfg, 0x1_0014, &[0xE000_0000]); // 512 MB
        m.queue(Space::Cfg, 0x1_0018, &[0x0000_0001]); // I/O BAR: skipped
        for off in [0x1C, 0x20, 0x24] {
            m.queue(Space::Cfg, 0x1_0000 + off, &[0]);
        }
        m.queue(Space::Cfg, 0x1_0030, &[0xFFFF_0001]); // ROM
        let l = hw_init(&m, 0x0128, &PIPES, no_sleep).unwrap();
        assert_eq!(
            l.bars,
            vec![(0x10, 0xFFF0_0008, 0x1000_0000), (0x14, 0xE000_0000, 0x2000_0000), (0x30, 0xFFFF_0001, 0x4000_0000)]
        );
    }

    #[test]
    fn hw_init_netchip_gpio_and_endpoints() {
        let m = scripted();
        m.set(Space::Csr, 0x50, 0xFF);
        let l = hw_init(&m, 0xA872, &PIPES, no_sleep).unwrap();
        let ops = m.ops();
        assert_eq!(
            &ops[..8],
            &[
                Op::CsrRead(0x50),
                Op::CsrWrite(0x50, 0xF0),
                Op::CsrRead(0x50),
                Op::CsrWrite(0x50, 0xF1),
                Op::CsrRead(0x50),
                Op::CsrWrite(0x50, 0xF3),
                Op::CsrRead(0x50),
                Op::CsrWrite(0x50, 0xFB),
            ]
        );
        assert_eq!(l.endpoints, [0x01, 0x02, 0x03, 0x81]);
        let w = m.writes();
        // Endpoints B and C are OUT on these boards, D is not.
        assert!(w.contains(&Op::CsrWrite(0x344, 4)));
        assert!(w.contains(&Op::CsrWrite(0x364, 4)));
        assert!(!w.contains(&Op::CsrWrite(0x384, 4)));
        // Endpoint 0x01 has no pipe here, so no channel was mapped.
        assert!(!l.dma_mapped);
    }

    fn layout(off: u16) -> Layout {
        Layout {
            vixs_id: 0,
            bars: vec![],
            fb: 0x1000_0000,
            mmr: 0x2000_0000,
            endpoints: [0x04, 0x82, 0x83, 0x84],
            fifo: [0; 4],
            ep_cfg: [0; 4],
            dma_offset: off,
            dma_mapped: true,
        }
    }

    #[test]
    fn dma_write_programs_channel_then_bulk() {
        let m = MockPci::new();
        let data = vec![0xA5; 0x23C0];
        dma_write(&m, &layout(0x20), 0x1004_0000, &data).unwrap();
        assert_eq!(
            m.ops(),
            vec![
                Op::CsrWrite(0x1B4, 0x1004_0000),
                Op::CsrWrite(0x1B0, 0x23C0),
                Op::CsrWrite(0x1A0, 2),
                Op::CsrWrite(0x1A4, 1),
                Op::BulkOut(0x04, 0x23C0),
            ]
        );
        assert_eq!(m.bulk()[0].1, data);
        assert!(matches!(
            dma_write(&m, &layout(0), 0, &vec![0; DMA_MAX + 1]),
            Err(Error::TooLarge { limit: DMA_MAX, .. })
        ));
    }

    #[test]
    fn interrupt_rearm_only_after_a_status_word() {
        let m = MockPci::new();
        m.set(Space::Csr, 0x24, 0x8100_0000);
        let mut irq = IrqState::new();
        // First wait arms with last = 0, times out.
        assert_eq!(irq.wait(&m, Duration::ZERO).unwrap(), None);
        // A timeout keeps the read pending: no second arming.
        assert_eq!(irq.wait(&m, Duration::ZERO).unwrap(), None);
        m.push_interrupt(0x0100_0000);
        assert_eq!(irq.wait(&m, Duration::ZERO).unwrap(), Some(0x0100_0000));
        // The next wait clears the received bits first.
        assert_eq!(irq.wait(&m, Duration::ZERO).unwrap(), None);
        use Op::*;
        assert_eq!(
            m.ops(),
            vec![
                CsrRead(0x24),
                CsrWrite(0x24, 0),
                CsrWrite(0x2C, 0),
                CsrWrite(0x24, 0x8100_0000),
                Interrupt,
                Interrupt,
                Interrupt,
                CsrRead(0x24),
                CsrWrite(0x24, 0),
                CsrWrite(0x2C, 0x0100_0000),
                CsrWrite(0x24, 0x8100_0000),
                Interrupt,
            ]
        );
    }

    #[test]
    fn pci_abort_clears_device_halts() {
        let m = MockPci::new();
        m.set(Space::Csr, 0x234, 1);
        m.set(Space::Csr, 0x364, 0x81);
        m.push_interrupt(0x0010_0000);
        let mut irq = IrqState::new();
        m.clear();
        irq.wait(&m, Duration::ZERO).unwrap();
        let w = m.writes();
        assert_eq!(
            &w[3..],
            &[Op::CsrWrite(0x234, 1), Op::CsrWrite(0x364, 1)],
            "only the halted endpoints get CLEAR_ENDPOINT_HALT"
        );
        let reads: Vec<_> = m.ops().into_iter().filter(|o| matches!(o, Op::CsrRead(r) if *r != 0x24)).collect();
        assert_eq!(reads.len(), 6);
    }

    #[test]
    fn tick_rounding() {
        assert_eq!(ticks(10), Duration::from_millis(110));
        assert_eq!(ticks(1), Duration::from_millis(20));
    }
}
