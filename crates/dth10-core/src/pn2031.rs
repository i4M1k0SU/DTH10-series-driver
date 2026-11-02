//! The `PN2031_MxL135RF` tuner kit: PN2031 demodulator with a MxL135RF tuner.
//!
//! Not implemented yet: [`Pn2031Kit::new`] refuses the kit, so a unit that reports
//! it fails at open with a clear message.

use crate::device::Device;
use crate::error::{Error, Result};
use crate::protocol::UsbControl;
use crate::tc90517::{SignalStatus, TuneRequest};
use crate::tlv::TunerDesc;

pub struct Pn2031Kit<T: UsbControl> {
    pub dev: Device<T>,
    pub desc: TunerDesc,
}

fn unsupported(desc: &TunerDesc) -> Error {
    Error::Unsupported(format!(
        "terrestrial tuner kit {} hw 0x{:x} is not implemented yet",
        desc.kit_name, desc.hw_version
    ))
}

impl<T: UsbControl> Pn2031Kit<T> {
    pub fn new(dev: Device<T>, desc: TunerDesc) -> Result<Self> {
        let _ = dev;
        Err(unsupported(&desc))
    }

    pub fn init(&mut self) -> Result<()> {
        Err(unsupported(&self.desc))
    }

    pub fn open(&mut self) -> Result<()> {
        Err(unsupported(&self.desc))
    }

    pub fn close(&mut self) -> Result<()> {
        Ok(())
    }

    pub fn is_open(&self) -> bool {
        false
    }

    pub fn tune(&mut self, _req: &TuneRequest) -> Result<()> {
        Err(unsupported(&self.desc))
    }

    pub fn signal_status(&mut self, _mask: u32) -> Result<SignalStatus> {
        Err(unsupported(&self.desc))
    }
}
