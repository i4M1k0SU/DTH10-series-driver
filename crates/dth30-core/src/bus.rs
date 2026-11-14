//! Access to the PLX NET2280 and, through it, to the remote PCI bus.
//!
//! The NET2280 runs in PCI host mode: its dedicated endpoints carry register
//! accesses as small bulk packets (`dth30-usb` builds them):
//!
//! * EP 0x0D / 0x8D: the NET2280's own registers (its BAR0 space) and its own
//!   PCI configuration space.
//! * EP 0x0E / 0x8E: master cycles on the remote PCI bus (configuration,
//!   memory and I/O space of the ViXS chip).
//! * EP 0x8F: interrupt status (4 bytes, little endian).
//! * Programmable endpoints: bulk OUT 0x04 feeds the NET2280 DMA engine
//!   (host to ViXS memory), bulk IN 0x82 / 0x83 / 0x84 drain FIFO B / C / D.
//!
//! All accesses are 32-bit and little endian. Implementations must keep each
//! read's OUT and IN packets together (no other access in between), which the
//! vendor stack does with one lock shared by both endpoint pairs.

use core::time::Duration;

use crate::Result;

/// Register-level access to the NET2280 and the remote PCI bus.
///
/// Methods take `&self` so that a reader thread and the control path can
/// share one transport; implementations serialise internally.
pub trait RemotePci {
    /// Read a NET2280 register (its BAR0 space, e.g. `0x24` USBIRQENB1).
    fn csr_read(&self, offset: u16) -> Result<u32>;
    /// Write a NET2280 register.
    fn csr_write(&self, offset: u16, value: u32) -> Result<()>;
    /// Read the NET2280's own PCI configuration space.
    fn local_cfg_read(&self, offset: u16) -> Result<u32>;
    /// Write the NET2280's own PCI configuration space.
    fn local_cfg_write(&self, offset: u16, value: u32) -> Result<()>;
    /// Configuration read cycle on the remote bus (`addr` as the NET2280
    /// expects it; the ViXS chip's space is `0x1_0000 | register`).
    fn cfg_read(&self, addr: u32) -> Result<u32>;
    /// Configuration write cycle on the remote bus.
    fn cfg_write(&self, addr: u32, value: u32) -> Result<()>;
    /// Memory read cycle on the remote bus (a PCI address inside a BAR).
    fn mem_read(&self, addr: u32) -> Result<u32>;
    /// Memory write cycle on the remote bus.
    fn mem_write(&self, addr: u32, value: u32) -> Result<()>;
    /// Send `data` on a bulk OUT endpoint (the NET2280 DMA engine must have
    /// been programmed for it first, see [`crate::net2280`]).
    fn bulk_out(&self, endpoint: u8, data: &[u8]) -> Result<()>;
    /// Wait for one interrupt status word on EP 0x8F; `Ok(None)` on timeout.
    /// The caller re-arms the interrupt first (see [`crate::net2280`]).
    fn interrupt_status(&self, timeout: Duration) -> Result<Option<u32>>;
}

/// A recording [`RemotePci`] for tests: every access is logged, reads are
/// answered from a register map (unknown registers read as 0).
#[cfg(any(test, feature = "mock"))]
pub mod mock {
    use std::collections::HashMap;
    use std::sync::Mutex;

    use super::*;

    /// One logged access.
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum Op {
        CsrRead(u16),
        CsrWrite(u16, u32),
        LocalCfgRead(u16),
        LocalCfgWrite(u16, u32),
        CfgRead(u32),
        CfgWrite(u32, u32),
        MemRead(u32),
        MemWrite(u32, u32),
        BulkOut(u8, usize),
        Interrupt,
    }

    /// Which register space a scripted value belongs to.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub enum Space {
        Csr,
        LocalCfg,
        Cfg,
        Mem,
    }

    #[derive(Default)]
    struct State {
        ops: Vec<Op>,
        regs: HashMap<(Space, u32), u32>,
        /// Values returned by successive reads before falling back to `regs`.
        queued: HashMap<(Space, u32), Vec<u32>>,
        interrupts: Vec<u32>,
        bulk: Vec<(u8, Vec<u8>)>,
    }

    /// See the module documentation.
    #[derive(Default)]
    pub struct MockPci {
        state: Mutex<State>,
    }

    impl MockPci {
        pub fn new() -> Self {
            Self::default()
        }

        /// Let reads of `addr` in `space` return `value` (writes also update it).
        pub fn set(&self, space: Space, addr: u32, value: u32) {
            self.state.lock().unwrap().regs.insert((space, addr), value);
        }

        /// Queue values for the next reads of `addr` (then the map value).
        pub fn queue(&self, space: Space, addr: u32, values: &[u32]) {
            self.state.lock().unwrap().queued.entry((space, addr)).or_default().extend_from_slice(values);
        }

        /// Queue an interrupt status word.
        pub fn push_interrupt(&self, status: u32) {
            self.state.lock().unwrap().interrupts.push(status);
        }

        /// The accesses so far.
        pub fn ops(&self) -> Vec<Op> {
            self.state.lock().unwrap().ops.clone()
        }

        /// The writes so far (reads left out).
        pub fn writes(&self) -> Vec<Op> {
            self.ops()
                .into_iter()
                .filter(|o| matches!(o, Op::CsrWrite(..) | Op::LocalCfgWrite(..) | Op::CfgWrite(..) | Op::MemWrite(..)))
                .collect()
        }

        /// The bulk OUT payloads so far.
        pub fn bulk(&self) -> Vec<(u8, Vec<u8>)> {
            self.state.lock().unwrap().bulk.clone()
        }

        pub fn clear(&self) {
            let mut s = self.state.lock().unwrap();
            s.ops.clear();
            s.bulk.clear();
        }

        fn read(&self, space: Space, addr: u32, op: Op) -> u32 {
            let mut s = self.state.lock().unwrap();
            s.ops.push(op);
            if let Some(q) = s.queued.get_mut(&(space, addr))
                && !q.is_empty()
            {
                return q.remove(0);
            }
            s.regs.get(&(space, addr)).copied().unwrap_or(0)
        }

        fn write(&self, space: Space, addr: u32, value: u32, op: Op) {
            let mut s = self.state.lock().unwrap();
            s.ops.push(op);
            s.regs.insert((space, addr), value);
        }
    }

    impl RemotePci for MockPci {
        fn csr_read(&self, offset: u16) -> Result<u32> {
            Ok(self.read(Space::Csr, offset as u32, Op::CsrRead(offset)))
        }
        fn csr_write(&self, offset: u16, value: u32) -> Result<()> {
            self.write(Space::Csr, offset as u32, value, Op::CsrWrite(offset, value));
            Ok(())
        }
        fn local_cfg_read(&self, offset: u16) -> Result<u32> {
            Ok(self.read(Space::LocalCfg, offset as u32, Op::LocalCfgRead(offset)))
        }
        fn local_cfg_write(&self, offset: u16, value: u32) -> Result<()> {
            self.write(Space::LocalCfg, offset as u32, value, Op::LocalCfgWrite(offset, value));
            Ok(())
        }
        fn cfg_read(&self, addr: u32) -> Result<u32> {
            Ok(self.read(Space::Cfg, addr, Op::CfgRead(addr)))
        }
        fn cfg_write(&self, addr: u32, value: u32) -> Result<()> {
            self.write(Space::Cfg, addr, value, Op::CfgWrite(addr, value));
            Ok(())
        }
        fn mem_read(&self, addr: u32) -> Result<u32> {
            Ok(self.read(Space::Mem, addr, Op::MemRead(addr)))
        }
        fn mem_write(&self, addr: u32, value: u32) -> Result<()> {
            self.write(Space::Mem, addr, value, Op::MemWrite(addr, value));
            Ok(())
        }
        fn bulk_out(&self, endpoint: u8, data: &[u8]) -> Result<()> {
            let mut s = self.state.lock().unwrap();
            s.ops.push(Op::BulkOut(endpoint, data.len()));
            s.bulk.push((endpoint, data.to_vec()));
            Ok(())
        }
        fn interrupt_status(&self, _timeout: Duration) -> Result<Option<u32>> {
            let mut s = self.state.lock().unwrap();
            s.ops.push(Op::Interrupt);
            Ok(if s.interrupts.is_empty() { None } else { Some(s.interrupts.remove(0)) })
        }
    }
}
