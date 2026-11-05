//! The kit of tuner #0 (the terrestrial filter), chosen by the kit name of the
//! device's descriptor exactly as the vendor driver chooses its kit object:
//!
//! | kit name | kit |
//! |---|---|
//! | `TC90517_MxL131RF`, `TC90517_MxL135RF`, `SMT_LJ101` hw 0x102 | [`Tc90517Kit`] |
//! | `PN2031_MxL135RF` | [`Pn2031Kit`] |
//! | `BQ200X_MxL135RF`, `BQ200X_E4005` | [`Bq200xKit`] |
//! | `LGDT3305_MxL5007T` | [`Lgdt3305Kit`] |
//!
//! Every kit offers the same operations (init, open / close, tune, signal
//! status); the bridge (`dev`) is shared by all of them.

use crate::bq200x::Bq200xKit;
use crate::device::Device;
use crate::error::{Error, Result};
use crate::lgdt3305::Lgdt3305Kit;
use crate::pn2031::Pn2031Kit;
use crate::protocol::UsbControl;
use crate::tc90517::Tc90517Kit;
use crate::tc90517::{SignalStatus, TuneRequest};
use crate::tlv::TunerDesc;

pub enum TerrestrialKit<T: UsbControl> {
    Tc90517(Tc90517Kit<T>),
    Pn2031(Pn2031Kit<T>),
    Bq200x(Bq200xKit<T>),
    Lgdt3305(Lgdt3305Kit<T>),
}

/// Run the same expression on whichever kit it is.
macro_rules! each {
    ($self:expr, $k:ident => $e:expr) => {
        match $self {
            TerrestrialKit::Tc90517($k) => $e,
            TerrestrialKit::Pn2031($k) => $e,
            TerrestrialKit::Bq200x($k) => $e,
            TerrestrialKit::Lgdt3305($k) => $e,
        }
    };
}

impl<T: UsbControl> TerrestrialKit<T> {
    /// The kit for `desc` (tuner #0's descriptor).
    pub fn new(dev: Device<T>, desc: TunerDesc) -> Result<Self> {
        let name = desc.kit_name.as_str();
        Ok(match name {
            "TC90517_MxL131RF" | "TC90517_MxL135RF" => TerrestrialKit::Tc90517(Tc90517Kit::new(dev, desc)?),
            "SMT_LJ101" if desc.hw_version == 0x102 => TerrestrialKit::Tc90517(Tc90517Kit::new(dev, desc)?),
            "PN2031_MxL135RF" => TerrestrialKit::Pn2031(Pn2031Kit::new(dev, desc)?),
            "BQ200X_MxL135RF" | "BQ200X_E4005" => TerrestrialKit::Bq200x(Bq200xKit::new(dev, desc)?),
            "LGDT3305_MxL5007T" => TerrestrialKit::Lgdt3305(Lgdt3305Kit::new(dev, desc)?),
            _ => {
                return Err(Error::Unsupported(format!(
                    "terrestrial tuner kit {name} hw 0x{:x} is not one the vendor driver knows",
                    desc.hw_version
                )));
            }
        })
    }

    /// Which kit it is, for the log.
    pub fn kind(&self) -> &'static str {
        match self {
            TerrestrialKit::Tc90517(_) => "TC90517",
            TerrestrialKit::Pn2031(_) => "PN2031",
            TerrestrialKit::Bq200x(_) => "BQ200X",
            TerrestrialKit::Lgdt3305(_) => "LGDT3305",
        }
    }

    /// The bridge, for the stream and GPIO commands of tuner #0.
    pub fn dev(&mut self) -> &mut Device<T> {
        each!(self, k => &mut k.dev)
    }

    /// The bridge, read-only.
    pub fn dev_ref(&self) -> &Device<T> {
        each!(self, k => &k.dev)
    }

    pub fn desc(&self) -> &TunerDesc {
        each!(self, k => &k.desc)
    }

    pub fn init(&mut self) -> Result<()> {
        each!(self, k => k.init())
    }

    pub fn open(&mut self) -> Result<()> {
        each!(self, k => k.open())
    }

    pub fn close(&mut self) -> Result<()> {
        each!(self, k => k.close())
    }

    pub fn is_open(&self) -> bool {
        each!(self, k => k.is_open())
    }

    pub fn tune(&mut self, req: &TuneRequest) -> Result<()> {
        each!(self, k => k.tune(req))
    }

    pub fn signal_status(&mut self, mask: u32) -> Result<SignalStatus> {
        each!(self, k => k.signal_status(mask))
    }
}
