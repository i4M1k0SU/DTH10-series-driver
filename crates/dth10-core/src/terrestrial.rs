//! The kit of a terrestrial tuner (tuner #0, and tuner #1 of the two-tuner
//! models), chosen by the kit name of that tuner's descriptor exactly as the
//! vendor driver chooses its kit object:
//!
//! | kit name | kit |
//! |---|---|
//! | `TC90517_MxL131RF`, `TC90517_MxL135RF`, `SMT_LJ101` hw 0x102 | [`Tc90517Kit`] |
//! | `PN2031_MxL135RF` | [`Pn2031Kit`] |
//! | `BQ200X_MxL135RF`, `BQ200X_E4005` | [`Bq200xKit`] |
//! | `LGDT3305_MxL5007T` | [`Lgdt3305Kit`] |
//!
//! Every kit offers the same operations (init, open / close, tune, signal
//! status); the bridge (`dev`) is shared by all of them. The model (USB
//! product ID) selects the register sequences where its vendor driver has
//! its own ([`Pn2031Variant`]).

use crate::bq200x::Bq200xKit;
use crate::device::Device;
use crate::error::{Error, Result};
use crate::kit::{SignalStatus, TuneRequest};
use crate::lgdt3305::Lgdt3305Kit;
use crate::pn2031::{Pn2031Kit, Pn2031Variant};
use crate::protocol::UsbControl;
use crate::tc90517::Tc90517Kit;
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
    /// Whether `desc` names one of these kits (whose variant [`Self::new`] may
    /// still refuse): on tuner #1 such a descriptor makes the tuner a second
    /// terrestrial tuner (the vendor driver builds each filter's kit from that
    /// filter's own descriptor).
    pub fn supports(desc: &TunerDesc) -> bool {
        matches!(
            desc.kit_name.as_str(),
            "TC90517_MxL131RF"
                | "TC90517_MxL135RF"
                | "PN2031_MxL135RF"
                | "BQ200X_MxL135RF"
                | "BQ200X_E4005"
                | "LGDT3305_MxL5007T"
        ) || (desc.kit_name == "SMT_LJ101" && desc.hw_version == 0x102)
    }

    /// The kit for `desc` (the descriptor of the tuner it drives) on the
    /// model with USB product ID `pid`.
    pub fn new(dev: Device<T>, desc: TunerDesc, pid: u16) -> Result<Self> {
        let name = desc.kit_name.as_str();
        Ok(match name {
            "TC90517_MxL131RF" | "TC90517_MxL135RF" => TerrestrialKit::Tc90517(Tc90517Kit::new(dev, desc)?),
            "SMT_LJ101" if desc.hw_version == 0x102 => TerrestrialKit::Tc90517(Tc90517Kit::new(dev, desc)?),
            "PN2031_MxL135RF" => TerrestrialKit::Pn2031(Pn2031Kit::new(dev, desc, Pn2031Variant::for_product(pid))?),
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

    /// The bridge (on tuner #0 also the one of its stream commands).
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

    /// Whether Init leaves power-up and configuration to Open (descriptor
    /// `flags` bit5, unless the kit's variant does not look at it); the vendor
    /// driver then powers the chips down after the device start.
    pub fn skips_init_config(&self) -> bool {
        match self {
            TerrestrialKit::Pn2031(k) => k.skips_init_config(),
            k => k.desc().skip_init_config(),
        }
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

    /// Whether the kit has the attenuator switch (the vendor application's
    /// "use attenuator"): `SMT_LJ101` hw 0x102 only; for every other kit the
    /// vendor driver does nothing with the setting.
    pub fn has_attenuator(&self) -> bool {
        match self {
            TerrestrialKit::Tc90517(k) => k.has_attenuator(),
            _ => false,
        }
    }

    /// Switch the attenuator. `Ok(false)`: not switched (the kit has none,
    /// or the module selects no chip for it), as in the vendor driver.
    pub fn set_attenuator(&mut self, on: bool) -> Result<bool> {
        match self {
            TerrestrialKit::Tc90517(k) => k.set_attenuator(on),
            _ => Ok(false),
        }
    }

    /// Whether the attenuator is on; `None` when it cannot be read (no attenuator).
    pub fn attenuator(&mut self) -> Result<Option<bool>> {
        match self {
            TerrestrialKit::Tc90517(k) => k.attenuator(),
            _ => Ok(None),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regmock::RegMock;

    #[test]
    fn supports_names_the_kits_new_builds() {
        for (name, model, hw) in [
            ("TC90517_MxL131RF", "", 0x101),
            ("TC90517_MxL135RF", "", 0x101),
            ("SMT_LJ101", "h70", 0x102),
            ("SMT_LJ101", "h70", 0x402),
            ("PN2031_MxL135RF", "", 0x104),
            ("BQ200X_MxL135RF", "", 0x102),
            ("BQ200X_MxL135RF", "h55", 0x102),
            ("LGDT3305_MxL5007T", "", 0x101),
            ("SOMETHING_ELSE", "", 0x102),
        ] {
            let desc = TunerDesc { kit_name: name.into(), model: model.into(), hw_version: hw, ..Default::default() };
            let built = TerrestrialKit::new(Device::new(RegMock::default()), desc.clone(), 0x0199).is_ok();
            assert_eq!(TerrestrialKit::<RegMock>::supports(&desc), built, "{name} hw 0x{hw:x}");
        }
    }

    #[test]
    fn pn2031_variant_by_product() {
        let desc = TunerDesc { kit_name: "PN2031_MxL135RF".into(), flags: 0x20, ..Default::default() };
        for (pid, variant) in [(0x017B, Pn2031Variant::Dth10U7), (0x015F, Pn2031Variant::Standard)] {
            let k = TerrestrialKit::new(Device::new(RegMock::default()), desc.clone(), pid).unwrap();
            let TerrestrialKit::Pn2031(p) = &k else { panic!("not a PN2031 kit") };
            assert_eq!(p.variant, variant);
            // Flags bit5 counts on the standard variant only.
            assert_eq!(k.skips_init_config(), variant == Pn2031Variant::Standard);
        }
    }
}
