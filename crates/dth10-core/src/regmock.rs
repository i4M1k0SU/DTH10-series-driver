//! Test transport: a register file per I2C address behind the bridge's
//! register commands, with a log of the single-register accesses.

use core::cell::RefCell;
use core::time::Duration;
use std::collections::HashMap;

use crate::error::Result;
use crate::protocol::{UsbControl, cmd};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Op {
    /// Register read: address, register.
    R(u8, u8),
    /// One-byte register write: address, register, value.
    W(u8, u8, u8),
    /// Anything else (GPIO, multi-byte, batch, raw I2C).
    Other,
}

#[derive(Default)]
pub struct RegMock {
    ops: RefCell<Vec<Op>>,
    regs: RefCell<HashMap<(u8, u8), u8>>,
}

impl RegMock {
    pub fn with_regs(init: &[(u8, u8, u8)]) -> Self {
        let m = RegMock::default();
        for &(a, r, v) in init {
            m.regs.borrow_mut().insert((a, r), v);
        }
        m
    }

    /// The accesses since the last call.
    pub fn take(&self) -> Vec<Op> {
        std::mem::take(&mut *self.ops.borrow_mut())
    }

    /// A register's current value (0 when never written).
    pub fn reg(&self, addr: u8, reg: u8) -> u8 {
        *self.regs.borrow().get(&(addr, reg)).unwrap_or(&0)
    }
}

impl UsbControl for RegMock {
    fn control_out(&self, _: u8, value: u16, index: u16, data: &[u8], _: Duration) -> Result<usize> {
        let op = if value & 0xFF00 == cmd::I2C_REG {
            let addr = (value & 0xFF) as u8;
            for (k, &b) in data.iter().enumerate() {
                self.regs.borrow_mut().insert((addr, index as u8 + k as u8), b);
            }
            if data.len() == 1 { Op::W(addr, index as u8, data[0]) } else { Op::Other }
        } else {
            Op::Other
        };
        self.ops.borrow_mut().push(op);
        Ok(data.len())
    }

    fn control_in(&self, _: u8, value: u16, index: u16, buf: &mut [u8], _: Duration) -> Result<usize> {
        if value & 0xFF00 == cmd::I2C_REG {
            let addr = (value & 0xFF) as u8;
            for (k, x) in buf.iter_mut().enumerate() {
                *x = self.reg(addr, index as u8 + k as u8);
            }
            self.ops.borrow_mut().push(if buf.len() == 1 { Op::R(addr, index as u8) } else { Op::Other });
        } else {
            buf.fill(0);
            self.ops.borrow_mut().push(Op::Other);
        }
        Ok(buf.len())
    }
}
