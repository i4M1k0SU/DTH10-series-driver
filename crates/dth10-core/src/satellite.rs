//! The kit of the satellite tuner (tuner #1), chosen by the kit name and hw
//! version of that tuner's descriptor as the vendor driver chooses its kit
//! object:
//!
//! | kit name | hw | kit |
//! |---|---|---|
//! | `SMT_LJ101` | 0x402 | [`Lj101Kit`] |
//! | `SHARP_8011` | 0x402 | [`Sharp8011SatKit`] |
//! | `TC90522_MTV600`, `TC90532_MTV600`, `TC90532_MTV808` | 0x402 | [`Tc9052xSatKit`] |
//!
//! Every kit offers the same operations (init, open / close, tune to a BS/CS
//! IF, TSID selection, signal status, antenna supply, booster); the bridge
//! (`dev`) is shared by all of them. A new kit goes into [`Which`] (the
//! table above), the enum and the `each!` dispatch.

use crate::device::Device;
use crate::error::{Error, Result};
use crate::kit::SignalStatus;
use crate::lj101::Lj101Kit;
use crate::protocol::UsbControl;
use crate::sharp8011::Sharp8011SatKit;
use crate::tc9052x::{self, Tc9052xSatKit};
use crate::tlv::TunerDesc;

pub enum SatelliteKit<T: UsbControl> {
    Lj101(Lj101Kit<T>),
    Sharp8011(Sharp8011SatKit<T>),
    Tc9052x(Tc9052xSatKit<T>),
}

/// The satellite kits by descriptor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Which {
    Lj101,
    Sharp8011,
    Tc9052x,
}

impl Which {
    fn of<T: UsbControl>(desc: &TunerDesc) -> Option<Self> {
        if Lj101Kit::<T>::supports(desc) {
            Some(Which::Lj101)
        } else if Sharp8011SatKit::<T>::supports(desc) {
            Some(Which::Sharp8011)
        } else if Tc9052xSatKit::<T>::supports(desc) {
            Some(Which::Tc9052x)
        } else {
            None
        }
    }
}

/// Run the same expression on whichever kit it is.
macro_rules! each {
    ($self:expr, $k:ident => $e:expr) => {
        match $self {
            SatelliteKit::Lj101($k) => $e,
            SatelliteKit::Sharp8011($k) => $e,
            SatelliteKit::Tc9052x($k) => $e,
        }
    };
}

impl<T: UsbControl> SatelliteKit<T> {
    /// Whether `desc` names one of these kits.
    pub fn supports(desc: &TunerDesc) -> bool {
        Which::of::<T>(desc).is_some()
    }

    /// The kit for `desc` (the descriptor of the tuner it drives) on the
    /// model with USB product ID `pid` (as [`TerrestrialKit::new`](crate::terrestrial::TerrestrialKit::new):
    /// the TC90522 / TC90532 kit follows the model's vendor driver where the
    /// drivers differ, [`tc9052x::driver_for`]).
    pub fn new(dev: Device<T>, desc: TunerDesc, pid: u16) -> Result<Self> {
        Ok(match Which::of::<T>(&desc) {
            Some(Which::Lj101) => SatelliteKit::Lj101(Lj101Kit::new(dev, desc)?),
            Some(Which::Sharp8011) => SatelliteKit::Sharp8011(Sharp8011SatKit::new(dev, desc)?),
            Some(Which::Tc9052x) => {
                let driver = tc9052x::driver_for(&desc, pid).unwrap_or_default();
                SatelliteKit::Tc9052x(Tc9052xSatKit::new(dev, desc, driver)?)
            }
            None => {
                // Other satellite kits of the series are not implemented yet (no
                // such unit was available to check them).
                return Err(Error::Unsupported(format!(
                    "satellite tuner kit {} hw 0x{:x} is not implemented yet (implemented: SMT_LJ101 hw 0x402, SHARP_8011 hw 0x402, TC90522_MTV600 / TC90532_MTV600 / TC90532_MTV808 hw 0x402)",
                    desc.kit_name, desc.hw_version
                )));
            }
        })
    }

    /// Which kit it is, for the log.
    pub fn kind(&self) -> &'static str {
        match self {
            SatelliteKit::Lj101(_) => "SMT_LJ101",
            SatelliteKit::Sharp8011(_) => "SHARP_8011",
            SatelliteKit::Tc9052x(_) => "TC9052x",
        }
    }

    /// The bridge of the kit's I2C and GPIO.
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

    /// Tune to a BS/CS IF (RF − 10.678 GHz) in Hz.
    pub fn tune(&mut self, if_hz: u64) -> Result<()> {
        each!(self, k => k.tune(if_hz))
    }

    /// Select the transport stream by TSID (separate from tuning).
    pub fn set_tsid(&mut self, tsid: u16) -> Result<()> {
        each!(self, k => k.set_tsid(tsid))
    }

    /// Signal status; `field::TSID_TABLE` in `mask` also reads the TMCC TS-ID table.
    pub fn signal_status(&mut self, mask: u32) -> Result<SignalStatus> {
        each!(self, k => k.signal_status(mask))
    }

    /// Antenna (LNB) power (0 / 110 / 150 = off / 11 V / 15 V). Returns
    /// whether the bridge reported the supply as healthy.
    pub fn lnb_power(&mut self, volt: u32) -> Result<bool> {
        each!(self, k => k.lnb_power(volt))
    }

    /// The booster. `Ok(false)`: not switched.
    pub fn set_booster(&mut self, on: bool) -> Result<bool> {
        each!(self, k => k.set_booster(on))
    }

    /// Whether the booster is on.
    pub fn booster(&mut self) -> Result<bool> {
        each!(self, k => k.booster())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regmock::RegMock;
    use crate::terrestrial::TerrestrialKit;

    fn desc(name: &str, hw: u32) -> TunerDesc {
        TunerDesc { kit_name: name.into(), hw_version: hw, demod_i2c_addr: 0x19, ..Default::default() }
    }

    #[test]
    fn kit_by_name_and_hw_version() {
        for (name, hw, kind) in [
            ("SMT_LJ101", 0x402, Some("SMT_LJ101")),
            ("SHARP_8011", 0x402, Some("SHARP_8011")),
            ("SMT_LJ101", 0x102, None),
            ("SHARP_8011", 0x102, None),
            ("SHARP_8011", 0x401, None),
            ("SHARP_801", 0x402, None),
            ("TC90532_MTV600", 0x402, Some("TC9052x")),
            ("TC90522_MTV600", 0x402, Some("TC9052x")),
            ("TC90532_MTV808", 0x402, Some("TC9052x")),
            ("TC90522_MTV808", 0x402, None),
            ("TC90532_MTV600", 0x102, None),
            ("TC90527_FC5500", 0x402, None),
            ("", 0x402, None),
        ] {
            let d = desc(name, hw);
            let kit = SatelliteKit::new(Device::new(RegMock::default()), d.clone(), 0x0199);
            assert_eq!(kit.as_ref().ok().map(|k| k.kind()), kind, "{name} hw 0x{hw:x}");
            assert_eq!(SatelliteKit::<RegMock>::supports(&d), kind.is_some(), "{name} hw 0x{hw:x}");
            if kind.is_none() {
                let e = kit.err().unwrap();
                assert!(matches!(e, Error::Unsupported(_)), "{e}");
                assert!(e.to_string().contains(&format!("satellite tuner kit {name} hw 0x{hw:x}")), "{e}");
            }
        }
    }

    #[test]
    fn no_descriptor_is_both_terrestrial_and_satellite() {
        for name in ["SMT_LJ101", "SHARP_8011", "TC90532_MTV600", "TC90532_MTV808"] {
            for hw in [0x102, 0x402] {
                let d = desc(name, hw);
                assert_ne!(TerrestrialKit::<RegMock>::supports(&d), SatelliteKit::<RegMock>::supports(&d), "{name}");
            }
        }
    }

    #[test]
    fn dispatch_reaches_the_kit() {
        let mut k = SatelliteKit::new(Device::new(RegMock::default()), desc("SHARP_8011", 0x402), 0x0199).unwrap();
        assert!(!k.is_open());
        assert!(k.tune(1_049_480_000).is_err(), "not open");
        k.open().unwrap();
        assert!(k.is_open());
        assert_eq!(k.desc().kit_name, "SHARP_8011");
        k.close().unwrap();
        assert!(!k.is_open());
    }
}
