//! A recording bus for the front-end tests: I2C transfers and delays in one
//! ordered log, register reads served by [`MockI2c`], and failure injection.

use std::collections::HashMap;

use crate::i2c::I2c;
use crate::i2c::mock::MockI2c;
use crate::{Error, Result};

use super::regs::Delay;

/// One logged event. Failed attempts are logged too.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Ev {
    /// Write transfer.
    W(u8, Vec<u8>),
    /// Write-then-read transfer: address, written bytes, read length.
    R(u8, Vec<u8>, usize),
    /// Delay (ms).
    D(u32),
}

enum Fail {
    Times(u32),
    Always,
    Usb,
}

#[derive(Default)]
pub struct Rig {
    pub mock: MockI2c,
    pub ev: Vec<Ev>,
    fail: HashMap<(u8, Option<u8>), Fail>,
}

impl Rig {
    pub fn new() -> Self {
        Self::default()
    }

    /// The next `n` transfers to `addr8` fail with an I2C error.
    pub fn fail_times(&mut self, addr8: u8, n: u32) {
        self.fail.insert((addr8, None), Fail::Times(n));
    }

    /// Every transfer to `addr8` fails with an I2C error.
    pub fn fail_always(&mut self, addr8: u8) {
        self.fail.insert((addr8, None), Fail::Always);
    }

    /// Every transfer to `addr8` fails with a transport error.
    pub fn fail_with_usb(&mut self, addr8: u8) {
        self.fail.insert((addr8, None), Fail::Usb);
    }

    /// Every transfer to `addr8` whose first byte is `reg` fails with an I2C error.
    pub fn fail_reg(&mut self, addr8: u8, reg: u8) {
        self.fail.insert((addr8, Some(reg)), Fail::Always);
    }

    fn check(&mut self, addr8: u8, first: Option<u8>) -> Result<()> {
        let err = Error::I2c { addr: addr8, reg: first, msg: "injected" };
        let key = if self.fail.contains_key(&(addr8, first)) { (addr8, first) } else { (addr8, None) };
        match self.fail.get_mut(&key) {
            None => Ok(()),
            Some(Fail::Always) => Err(err),
            Some(Fail::Usb) => Err(Error::Usb("injected".into())),
            Some(Fail::Times(n)) => {
                *n -= 1;
                if *n == 0 {
                    self.fail.remove(&key);
                }
                Err(err)
            }
        }
    }
}

impl I2c for Rig {
    fn write(&mut self, addr8: u8, data: &[u8]) -> Result<()> {
        self.ev.push(Ev::W(addr8, data.to_vec()));
        self.check(addr8, data.first().copied())?;
        self.mock.write(addr8, data)
    }

    fn write_read(&mut self, addr8: u8, wr: &[u8], rd: &mut [u8]) -> Result<()> {
        self.ev.push(Ev::R(addr8, wr.to_vec(), rd.len()));
        self.check(addr8, wr.first().copied())?;
        self.mock.write_read(addr8, wr, rd)
    }
}

impl Delay for Rig {
    fn delay_ms(&mut self, ms: u32) {
        self.ev.push(Ev::D(ms));
    }
}

/// Builds an expected event list.
#[derive(Default)]
pub struct Seq(pub Vec<Ev>);

impl Seq {
    pub fn new() -> Self {
        Self::default()
    }
    /// Direct register write.
    pub fn w(mut self, addr8: u8, reg: u8, value: u8) -> Self {
        self.0.push(Ev::W(addr8, vec![reg, value]));
        self
    }
    /// Direct tuner write with a trailing 0xFE.
    pub fn wfe(mut self, addr8: u8, reg: u8, value: u8) -> Self {
        self.0.push(Ev::W(addr8, vec![reg, value, 0xFE]));
        self
    }
    /// Tuner write through the demodulator's repeater.
    pub fn rep(mut self, demod: u8, tuner: u8, reg: u8, value: u8) -> Self {
        self.0.push(Ev::W(demod, vec![0xFE, tuner, reg, value]));
        self
    }
    pub fn raw(mut self, addr8: u8, data: &[u8]) -> Self {
        self.0.push(Ev::W(addr8, data.to_vec()));
        self
    }
    pub fn read(mut self, addr8: u8, reg: u8, len: usize) -> Self {
        self.0.push(Ev::R(addr8, vec![reg], len));
        self
    }
    pub fn delay(mut self, ms: u32) -> Self {
        self.0.push(Ev::D(ms));
        self
    }
}
