//! The MxL135RF tuner of the `PN2031_MxL135RF` kit and its I2C path.
//!
//! The tuner object is the shared MaxLinear one ([`Mxl13xRf`]): same tables,
//! same transfer sequence. Only the path differs from the TC90517 kits:
//!
//! * `TunerDesc.flags` bit3 set: raw I2C on the bridge bus with the tuner
//!   address ([`Mxl13xRf`] with [`TunerPath::Direct`]).
//! * bit3 clear (default): the PN2031's I2C gate. Before **every** tuner
//!   transfer (write or read) the vendor driver writes `0x00` to demodulator
//!   register `0xFE` (register write, immediate); the transfer itself then
//!   goes to the tuner address on the bridge bus, unwrapped. The TC90517 kits
//!   instead wrap the tuner bytes into a demodulator write behind `0xFE`.

use core::time::Duration;
use std::thread::sleep;

use crate::device::Device;
use crate::error::Result;
use crate::mxl13xrf::{self, Mxl13xRf, MxlType};
use crate::protocol::UsbControl;
use crate::tlv::TunerDesc;
use crate::tuner_bus::{TunerBus, TunerPath};

/// Demodulator register written before each tuner transfer to open the gate.
pub const GATE_REG: u8 = 0xFE;
/// Value written to [`GATE_REG`].
pub const GATE_OPEN: u8 = 0x00;

/// The kit's tuner: the shared MxL object plus the gate, when used.
#[derive(Debug, Clone, Copy)]
pub struct Pn2031Tuner {
    /// The MxL object, addressed directly (its bus path is always [`TunerPath::Direct`]).
    pub mxl: Mxl13xRf,
    /// Demodulator address of the I2C gate; `None` = tuner directly on the bus.
    pub gate: Option<u8>,
}

fn flatten(pairs: &[(u8, u8)]) -> Vec<u8> {
    pairs.iter().flat_map(|&(r, v)| [r, v]).collect()
}

impl Pn2031Tuner {
    pub fn new(tuner_addr: u8, gate: Option<u8>) -> Self {
        Pn2031Tuner { mxl: Mxl13xRf::new(TunerBus { tuner_addr, path: TunerPath::Direct }, MxlType::MxL135RF), gate }
    }

    fn open_gate<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        match self.gate {
            Some(demod) => dev.i2c_reg_write(demod, GATE_REG, &[GATE_OPEN]),
            None => Ok(()),
        }
    }

    /// One tuner write (gate first when used).
    pub fn write<T: UsbControl>(&self, dev: &mut Device<T>, data: &[u8]) -> Result<()> {
        self.open_gate(dev)?;
        self.mxl.write(dev, data)
    }

    /// One tuner read (gate first when used).
    pub fn read<T: UsbControl>(&self, dev: &mut Device<T>, buf: &mut [u8]) -> Result<()> {
        self.open_gate(dev)?;
        self.mxl.bus.read(dev, buf)
    }

    /// `{0xFB, reg}` then one byte back (two gated transfers).
    pub fn read_reg<T: UsbControl>(&self, dev: &mut Device<T>, reg: u8) -> Result<u8> {
        self.write(dev, &[0xFB, reg])?;
        let mut b = [0u8; 1];
        self.read(dev, &mut b)?;
        Ok(b[0])
    }

    /// Init: soft reset `0xFF`, then the descriptor-patched init table in one write.
    pub fn init<T: UsbControl>(&self, dev: &mut Device<T>, desc: &TunerDesc) -> Result<()> {
        if self.gate.is_none() {
            return self.mxl.init(dev, desc);
        }
        let regs = mxl13xrf::init_registers(desc)?;
        self.write(dev, &[0xFF])?;
        self.write(dev, &flatten(&regs))
    }

    /// Standby: reg 0x01 = 0, reg 0x0F = 0.
    pub fn standby<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.write(dev, &[0x01, 0x00, 0x0F, 0x00])
    }

    /// Wake: reg 0x01 = 1.
    pub fn wake<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.write(dev, &[0x01, 0x01])
    }

    /// Synthesizer restart (`0x0F ← 0`, `0x0F ← 1`). The kit hands this to
    /// the demodulator object as a callback, which never calls it.
    pub fn retune<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<()> {
        self.write(dev, &[0x0F, 0x00, 0x0F, 0x01])
    }

    /// Set frequency: the 10 RF-tune register pairs in one write, 3 ms, then
    /// reg 0xD8 read twice and discarded.
    pub fn set_frequency<T: UsbControl>(&self, dev: &mut Device<T>, freq_hz: u32, bw_hz: u32) -> Result<()> {
        if self.gate.is_none() {
            return self.mxl.set_frequency(dev, freq_hz, bw_hz);
        }
        let regs = mxl13xrf::rf_tune_registers(freq_hz, bw_hz);
        self.write(dev, &flatten(&regs))?;
        sleep(Duration::from_millis(3));
        let _ = self.read_reg(dev, 0xD8);
        let _ = self.read_reg(dev, 0xD8);
        Ok(())
    }

    /// Signal readout from regs 0xCC / 0xBA (the shared MxL conversion).
    pub fn signal<T: UsbControl>(&self, dev: &mut Device<T>) -> Result<i32> {
        let cc = self.read_reg(dev, 0xCC)?;
        let ba = self.read_reg(dev, 0xBA)?;
        Ok(mxl13xrf::signal_value(cc, ba))
    }
}
