//! I2C as the front-end drivers see it.
//!
//! On this hardware the host drives the ViXS chip's I2C master itself
//! (`crate::vixs::i2c`), not the firmware. Addresses are 8-bit (the 7-bit
//! address shifted left; the R/W bit is ignored), as in the vendor code.

use crate::Result;

/// One I2C bus.
pub trait I2c {
    /// Write `data` to `addr8` (start, data, stop).
    fn write(&mut self, addr8: u8, data: &[u8]) -> Result<()>;
    /// Write `wr` without a stop, then read `rd.len()` bytes with a repeated
    /// start and a stop (a register read when `wr` is the register number).
    fn write_read(&mut self, addr8: u8, wr: &[u8], rd: &mut [u8]) -> Result<()>;
}

impl<T: I2c + ?Sized> I2c for &mut T {
    fn write(&mut self, addr8: u8, data: &[u8]) -> Result<()> {
        (**self).write(addr8, data)
    }
    fn write_read(&mut self, addr8: u8, wr: &[u8], rd: &mut [u8]) -> Result<()> {
        (**self).write_read(addr8, wr, rd)
    }
}

/// A recording [`I2c`] for tests: writes are logged and also stored as
/// register values (first byte = register, auto-increment), reads return the
/// stored registers (0 when unknown) unless a value was queued.
#[cfg(any(test, feature = "mock"))]
pub mod mock {
    use std::collections::{HashMap, VecDeque};

    use super::*;

    /// One logged transfer.
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum Xfer {
        Write(u8, Vec<u8>),
        WriteRead(u8, Vec<u8>, usize),
    }

    #[derive(Default)]
    pub struct MockI2c {
        pub log: Vec<Xfer>,
        regs: HashMap<(u8, u8), u8>,
        queued: HashMap<(u8, u8), VecDeque<u8>>,
    }

    impl MockI2c {
        pub fn new() -> Self {
            Self::default()
        }

        /// Set the value a register read returns.
        pub fn set(&mut self, addr8: u8, reg: u8, value: u8) {
            self.regs.insert((addr8, reg), value);
        }

        /// Queue values for the next reads of one register.
        pub fn queue(&mut self, addr8: u8, reg: u8, values: &[u8]) {
            self.queued.entry((addr8, reg)).or_default().extend(values.iter().copied());
        }

        /// The writes to one device, as `(register, value)` pairs in order
        /// (multi-byte writes split with auto-increment).
        pub fn reg_writes(&self, addr8: u8) -> Vec<(u8, u8)> {
            let mut out = Vec::new();
            for x in &self.log {
                if let Xfer::Write(a, d) = x
                    && *a == addr8
                    && let Some((&reg, data)) = d.split_first()
                {
                    for (i, v) in data.iter().enumerate() {
                        out.push((reg.wrapping_add(i as u8), *v));
                    }
                }
            }
            out
        }
    }

    impl I2c for MockI2c {
        fn write(&mut self, addr8: u8, data: &[u8]) -> Result<()> {
            self.log.push(Xfer::Write(addr8, data.to_vec()));
            if let Some((&reg, rest)) = data.split_first() {
                for (i, v) in rest.iter().enumerate() {
                    self.regs.insert((addr8, reg.wrapping_add(i as u8)), *v);
                }
            }
            Ok(())
        }

        fn write_read(&mut self, addr8: u8, wr: &[u8], rd: &mut [u8]) -> Result<()> {
            self.log.push(Xfer::WriteRead(addr8, wr.to_vec(), rd.len()));
            let reg = wr.first().copied().unwrap_or(0);
            for (i, b) in rd.iter_mut().enumerate() {
                let r = reg.wrapping_add(i as u8);
                *b = match self.queued.get_mut(&(addr8, r)).and_then(|q| q.pop_front()) {
                    Some(v) => v,
                    None => self.regs.get(&(addr8, r)).copied().unwrap_or(0),
                };
            }
            Ok(())
        }
    }
}
