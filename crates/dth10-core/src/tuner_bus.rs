//! I2C path to the tuner: either directly on the bridge bus or through the
//! TC90517's I2C repeater (register 0xFE).
//!
//! * Direct (`TunerDesc.flags` bit3 set): raw I2C write/read with the tuner address.
//! * Via demodulator (default): through the original driver's repeater proxy,
//!   which turns each tuner access into demodulator transfers:
//!   - write: raw write to the demod address of `[0xFE, tuner_addr<<1, data...]`
//!   - read:  raw write to the demod address of `[0xFE, (tuner_addr<<1)|1]`,
//!     then raw read of `len` bytes from the demod address.

use crate::device::Device;
use crate::error::Result;
use crate::protocol::UsbControl;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TunerPath {
    Direct,
    ViaDemod { demod_addr: u8 },
}

/// Tuner I2C endpoint (address + path).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TunerBus {
    pub tuner_addr: u8,
    pub path: TunerPath,
}

impl TunerBus {
    pub fn write<T: UsbControl>(&self, dev: &mut Device<T>, data: &[u8]) -> Result<()> {
        match self.path {
            TunerPath::Direct => dev.i2c_write(self.tuner_addr, data),
            TunerPath::ViaDemod { demod_addr } => {
                let mut buf = Vec::with_capacity(data.len() + 2);
                buf.push(0xFE);
                buf.push(self.tuner_addr << 1);
                buf.extend_from_slice(data);
                dev.i2c_write(demod_addr, &buf)
            }
        }
    }

    pub fn read<T: UsbControl>(&self, dev: &mut Device<T>, buf: &mut [u8]) -> Result<()> {
        match self.path {
            TunerPath::Direct => dev.i2c_read(self.tuner_addr, buf),
            TunerPath::ViaDemod { demod_addr } => {
                dev.i2c_write(demod_addr, &[0xFE, (self.tuner_addr << 1) | 1])?;
                dev.i2c_read(demod_addr, buf)
            }
        }
    }
}
