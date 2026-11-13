//! The TC90522 / TC90532 tuner kits: a TC90517-class demodulator with an
//! FC5500, MTV808 or MTV600 tuner behind its I2C repeater (or directly on the
//! bridge bus when the descriptor's flags bit3 says so). The vendor drivers
//! build them by kit name and hw version:
//!
//! | kit name | hw | kit | tuner | vendor driver |
//! |---|---|---|---|---|
//! | `TC90522_FC5500`, `TC90532_FC5500` | 0x102 | [`Tc9052xKit`] | [`Fc5500`] | DT-H30/PCIEM-TS |
//! | `TC90522_MTV808` | 0x102 | [`Tc9052xKit`] | [`Mtv808`] | DT-H30/PCIEM-TS |
//! | `TC90532_MTV808` | 0x102 | [`Tc9052xKit`] | [`Mtv808`] | both |
//! | `TC90532_MTV600` | 0x102 | [`Tc9052xKit`] | [`Mtv808`] | series |
//! | `TC90522_MTV600` | 0x402 | [`Tc9052xSatKit`] | [`mtv600::Mtv600`] | DT-H30/PCIEM-TS |
//! | `TC90532_MTV600` | 0x402 | [`Tc9052xSatKit`] | [`mtv600::Mtv600`] | both |
//! | `TC90532_MTV808` | 0x402 | [`Tc9052xSatKit`] | [`mtv600::Mtv600`] | series |
//!
//! The series driver (5.1.9.33) builds its two kits by hw version alone, so
//! both names give the MTV808 kit at 0x102 and the MTV600 kit at 0x402; the
//! DT-H30/PCIEM-TS driver (5.1.9.46) builds them by name and refuses the other
//! hw version. A combination that both drivers know follows the driver of the
//! model ([`driver_for`]); the two drivers' register sequences differ.
//!
//! The terrestrial kit drives the demodulator with the TC90517 code (wake,
//! standby, pre- / post-tune, status) and its own configuration and TS pins
//! ([`demod`]). It does not read the descriptor's demodulator crystal; the
//! DT-H30/PCIEM-TS driver reads its demodulator IF (`val_62`).

pub mod demod;
pub mod mtv600;
pub mod sat;

pub use sat::Tc9052xSatKit;

use crate::device::Device;
use crate::error::{Error, Result};
use crate::fc5500::Fc5500;
use crate::kit::{SignalStatus, TuneRequest, VendorDriver};
use crate::mtv808::Mtv808;
use crate::protocol::UsbControl;
use crate::tc90517::{Tc90517, TerVariant};
use crate::tlv::TunerDesc;
use crate::tuner_bus::{TunerBus, TunerPath};

/// The vendor driver whose code the kit for `desc` follows on the model with
/// USB product ID `pid`, `None` when no vendor driver builds a kit for it. A
/// kit name and hw version that only one driver knows follows that driver; one
/// that both know follows the driver of the model.
pub fn driver_for(desc: &TunerDesc, pid: u16) -> Option<VendorDriver> {
    match (desc.kit_name.as_str(), desc.hw_version) {
        ("TC90522_FC5500" | "TC90532_FC5500" | "TC90522_MTV808", 0x102) => Some(VendorDriver::Dth30),
        ("TC90522_MTV600", 0x402) => Some(VendorDriver::Dth30),
        ("TC90532_MTV808", 0x102) | ("TC90532_MTV600", 0x402) => Some(VendorDriver::for_product(pid)),
        ("TC90532_MTV600", 0x102) | ("TC90532_MTV808", 0x402) => Some(VendorDriver::Series),
        _ => None,
    }
}

/// The tuner's I2C path: through the repeater of the demodulator, or
/// directly when the descriptor's flags bit3 says so.
pub(crate) fn tuner_bus(desc: &TunerDesc) -> TunerBus {
    let path = if desc.tuner_direct_i2c() {
        TunerPath::Direct
    } else {
        TunerPath::ViaDemod { demod_addr: desc.demod_i2c_addr }
    };
    TunerBus { tuner_addr: desc.tuner_i2c_addr, path }
}

/// The tuner of the terrestrial kit.
#[derive(Debug, Clone, Copy)]
pub enum TerTuner {
    Fc5500(Fc5500),
    Mtv808(Mtv808),
}

/// The terrestrial kit (hw 0x102).
pub struct Tc9052xKit<T: UsbControl> {
    pub dev: Device<T>,
    pub desc: TunerDesc,
    pub driver: VendorDriver,
    pub demod: Tc90517,
    pub tuner: TerTuner,
    open_count: u32,
    last_request: Option<TuneRequest>,
}

impl<T: UsbControl> Tc9052xKit<T> {
    /// Whether `desc` names this kit (a TC90522 / TC90532 name with hw 0x102).
    pub fn supports(desc: &TunerDesc) -> bool {
        desc.hw_version == 0x102 && driver_for(desc, 0).is_some()
    }

    /// The kit for `desc` on the model with USB product ID `pid`
    /// ([`driver_for`]). The kit name's suffix selects the tuner: FC5500, or
    /// MTV808 (the series driver's kit also for `TC90532_MTV600`).
    pub fn new(dev: Device<T>, desc: TunerDesc, pid: u16) -> Result<Self> {
        let driver = match driver_for(&desc, pid) {
            Some(d) if desc.hw_version == 0x102 => d,
            _ => {
                return Err(Error::Unsupported(format!(
                    "terrestrial tuner kit {} hw 0x{:x} is not a TC90522 / TC90532 hw 0x102 kit",
                    desc.kit_name, desc.hw_version
                )));
            }
        };
        let bus = tuner_bus(&desc);
        let tuner = if desc.kit_name.ends_with("_FC5500") {
            TerTuner::Fc5500(Fc5500::new(bus, driver))
        } else {
            TerTuner::Mtv808(Mtv808::new(bus, driver))
        };
        Ok(Tc9052xKit {
            demod: Tc90517::with_variant(desc.demod_i2c_addr, TerVariant::Tc9052x(driver)),
            tuner,
            dev,
            desc,
            driver,
            open_count: 0,
            last_request: None,
        })
    }

    /// Demodulator configuration, then tuner init.
    fn configure(&mut self) -> Result<()> {
        let r = self.demod.configure(&mut self.dev, &self.desc).and_then(|()| match &mut self.tuner {
            TerTuner::Fc5500(t) => t.init(&mut self.dev, &self.desc),
            TerTuner::Mtv808(t) => t.init(&mut self.dev, &self.desc),
        });
        self.last_request = None;
        r
    }

    fn tuner_wake(&mut self) -> Result<()> {
        match &self.tuner {
            TerTuner::Fc5500(t) => t.wake(&mut self.dev),
            TerTuner::Mtv808(t) => t.wake(&mut self.dev),
        }
    }

    /// Standby: tuner, then demodulator. The tuners have no standby with the
    /// series driver.
    fn standby(&mut self) -> Result<()> {
        match &self.tuner {
            TerTuner::Fc5500(t) => t.standby(&mut self.dev)?,
            TerTuner::Mtv808(t) => t.standby(&mut self.dev)?,
        }
        self.demod.standby(&mut self.dev)
    }

    /// Init: power-up, demodulator configuration, tuner init, then tuner and
    /// demodulator standby (brought up again by [`open`](Self::open)). With
    /// flags bit5 set only the power-down, as in the vendor drivers.
    pub fn init(&mut self) -> Result<()> {
        if self.desc.skip_init_config() {
            return self.dev.power_down(&self.desc);
        }
        self.dev.power_up(&self.desc)?;
        self.configure()?;
        self.standby()
    }

    /// Open (reference counted): demodulator wake, then tuner wake (with flags
    /// bit5 set: power-up, configuration and tuner init), then GPIO mask C <-
    /// open value, which the vendor drivers write whether or not the steps
    /// before succeeded.
    pub fn open(&mut self) -> Result<()> {
        self.open_count += 1;
        if self.open_count != 1 {
            return Ok(());
        }
        let r = if self.desc.skip_init_config() {
            self.dev.power_up(&self.desc).and_then(|()| self.configure())
        } else {
            self.demod.wake(&mut self.dev).and_then(|()| self.tuner_wake())
        };
        let g = self.dev.gpio_write(self.desc.gpio_mask_c, self.desc.gpio_c_open);
        let r = r.and(g);
        if r.is_err() {
            self.open_count = 0;
        }
        r
    }

    /// Close: tuner and demodulator standby (with flags bit5 set:
    /// power-down). The GPIO of mask C is left as Open set it.
    pub fn close(&mut self) -> Result<()> {
        if self.open_count == 0 {
            return Ok(());
        }
        self.open_count -= 1;
        if self.open_count != 0 {
            return Ok(());
        }
        if self.desc.skip_init_config() { self.dev.power_down(&self.desc) } else { self.standby() }
    }

    pub fn is_open(&self) -> bool {
        self.open_count > 0
    }

    /// Tune: demodulator pre-tune, the tuner, demodulator post-tune.
    pub fn tune(&mut self, req: &TuneRequest) -> Result<()> {
        if !self.is_open() {
            return Err(Error::Protocol("kit is not open"));
        }
        self.demod.pre_tune(&mut self.dev)?;
        match &mut self.tuner {
            TerTuner::Fc5500(t) => t.tune(&mut self.dev, req.frequency_u32(), req.bandwidth_hz)?,
            TerTuner::Mtv808(t) => t.tune(&mut self.dev, req.frequency_u32())?,
        }
        self.demod.post_tune(&mut self.dev)?;
        self.last_request = Some(*req);
        Ok(())
    }

    /// Signal status: the demodulator's, then the tuner's (which replaces the
    /// strength).
    pub fn signal_status(&mut self, mask: u32) -> Result<SignalStatus> {
        let mut st = SignalStatus { mask, ..Default::default() };
        self.demod.status(&mut self.dev, &mut st)?;
        match &mut self.tuner {
            TerTuner::Fc5500(t) => t.status(&mut self.dev, &mut st)?,
            TerTuner::Mtv808(t) => t.status(&mut self.dev, &mut st)?,
        }
        Ok(st)
    }

    pub fn last_request(&self) -> Option<&TuneRequest> {
        self.last_request.as_ref()
    }

    /// Whether the kit has the attenuator switch: with the series driver only
    /// (the DT-H30/PCIEM-TS driver hands the setting to the bridge, which does
    /// nothing with it).
    pub fn has_attenuator(&self) -> bool {
        self.driver == VendorDriver::Series
    }

    /// The attenuator ([`demod::set_attenuator`]). `Ok(false)`: not switched.
    pub fn set_attenuator(&mut self, on: bool) -> Result<bool> {
        if !self.has_attenuator() {
            return Ok(false);
        }
        demod::set_attenuator(&mut self.dev, self.demod.addr, on)
    }

    /// The attenuator read-back ([`demod::attenuator`]); `None` when there is none.
    pub fn attenuator(&mut self) -> Result<Option<bool>> {
        if !self.has_attenuator() {
            return Ok(None);
        }
        demod::attenuator(&mut self.dev, self.demod.addr)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kit::PID_DTH30_PCIEM_TS;
    use crate::regmock::{Op, RegMock, Write};

    fn desc(name: &str, hw: u32, flags: u32) -> TunerDesc {
        TunerDesc {
            kit_name: name.into(),
            hw_version: hw,
            flags,
            demod_i2c_addr: 0x10,
            tuner_i2c_addr: 0x61,
            tuner_xtal_hz: 16_000_000,
            gpio_mask_c: 0x10,
            gpio_c_open: 0x10,
            ..Default::default()
        }
    }

    #[test]
    fn driver_by_name_hw_and_model() {
        for (name, hw, h30, other) in [
            ("TC90522_FC5500", 0x102, Some(VendorDriver::Dth30), Some(VendorDriver::Dth30)),
            ("TC90532_FC5500", 0x102, Some(VendorDriver::Dth30), Some(VendorDriver::Dth30)),
            ("TC90522_MTV808", 0x102, Some(VendorDriver::Dth30), Some(VendorDriver::Dth30)),
            ("TC90532_MTV808", 0x102, Some(VendorDriver::Dth30), Some(VendorDriver::Series)),
            ("TC90532_MTV600", 0x102, Some(VendorDriver::Series), Some(VendorDriver::Series)),
            ("TC90522_MTV600", 0x402, Some(VendorDriver::Dth30), Some(VendorDriver::Dth30)),
            ("TC90532_MTV600", 0x402, Some(VendorDriver::Dth30), Some(VendorDriver::Series)),
            ("TC90532_MTV808", 0x402, Some(VendorDriver::Series), Some(VendorDriver::Series)),
            ("TC90522_MTV600", 0x102, None, None),
            ("TC90522_MTV808", 0x402, None, None),
            ("TC90522_FC5500", 0x402, None, None),
            ("TC90532_FC5500", 0x104, None, None),
            ("TC90527_FC5500", 0x102, None, None),
            ("TC90532_MTV80", 0x102, None, None),
        ] {
            let d = desc(name, hw, 0);
            assert_eq!(driver_for(&d, PID_DTH30_PCIEM_TS), h30, "{name} hw 0x{hw:x}");
            assert_eq!(driver_for(&d, 0x01FF), other, "{name} hw 0x{hw:x}");
            assert_eq!(Tc9052xKit::<RegMock>::supports(&d), hw == 0x102 && h30.is_some(), "{name} hw 0x{hw:x}");
            assert_eq!(Tc9052xSatKit::<RegMock>::supports(&d), hw == 0x402 && h30.is_some(), "{name} hw 0x{hw:x}");
        }
    }

    #[test]
    fn tuner_by_kit_name() {
        for (name, pid, fc5500, driver) in [
            ("TC90522_FC5500", 0x01FF, true, VendorDriver::Dth30),
            ("TC90532_FC5500", 0x0218, true, VendorDriver::Dth30),
            ("TC90522_MTV808", 0x01FF, false, VendorDriver::Dth30),
            ("TC90532_MTV808", 0x0218, false, VendorDriver::Dth30),
            ("TC90532_MTV808", 0x01FF, false, VendorDriver::Series),
            ("TC90532_MTV600", 0x0218, false, VendorDriver::Series),
        ] {
            let k = Tc9052xKit::new(Device::new(RegMock::default()), desc(name, 0x102, 0), pid).unwrap();
            assert_eq!(matches!(k.tuner, TerTuner::Fc5500(_)), fc5500, "{name}");
            assert_eq!(k.driver, driver, "{name}");
            assert_eq!(k.demod.variant, TerVariant::Tc9052x(driver), "{name}");
            assert_eq!(k.has_attenuator(), driver == VendorDriver::Series, "{name}");
        }
        assert!(Tc9052xKit::new(Device::new(RegMock::default()), desc("TC90532_MTV600", 0x402, 0), 0x0218).is_err());
    }

    /// The demodulator part of the standby, after the TS pins have stopped the
    /// output (0x1C = 20 from the configuration).
    fn demod_standby(addr: u8) -> Write {
        Write::Batch(vec![(addr, 0x1C, vec![0xA0]), (addr, 0x03, vec![0x90])])
    }

    /// The demodulator wake after a standby.
    fn demod_wake(addr: u8) -> [Write; 2] {
        [Write::Reg(addr, 0x03, vec![0]), Write::Reg(addr, 0x1C, vec![0x20])]
    }

    /// The writes of the TS pin control in `mode` from a zeroed register file
    /// that the configuration left at `1D=02 1E=28 1F=AA`.
    fn ts_pins(addr: u8, mode: u8) -> Vec<Write> {
        let r1d = [0x02, 0xAA, 0x56][mode as usize];
        vec![
            Write::Batch(vec![(addr, 0x1D, vec![r1d])]),
            Write::Batch(vec![(addr, 0x1E, vec![0x28])]),
            Write::Batch(vec![(addr, 0x1F, vec![0xAA])]),
        ]
    }

    #[test]
    fn series_mtv808_init_open_tune_close() {
        let mut k = Tc9052xKit::new(Device::new(RegMock::default()), desc("TC90532_MTV808", 0x102, 0), 0x01FF).unwrap();
        k.init().unwrap();
        let w = k.dev.transport().take_writes();
        // the configuration (4 batches + 0x75)
        std::assert_matches!(&w[0], Write::Batch(b) if b.len() == 23 && b[9] == (0x10, 0x0F, vec![0x10]), "{:?}", w[0]);
        assert_eq!(w[4], Write::Reg(0x10, 0x75, vec![0x02]));
        // the tuner's init table follows through the repeater
        assert_eq!(
            w[5],
            Write::Raw(
                0x10,
                vec![0xFE, 0xC2, crate::mtv808::INIT_TABLE_SERIES[0].0, crate::mtv808::INIT_TABLE_SERIES[0].1]
            )
        );
        // ends with the demodulator standby only (no tuner standby in this driver)
        let n = w.len();
        assert_eq!(&w[n - 4..n - 1], &ts_pins(0x10, 1)[..]);
        assert_eq!(w[n - 1], demod_standby(0x10));
        assert!(w[5..n - 4].iter().all(|w| matches!(w, Write::Raw(0x10, d) if d[0] == 0xFE)), "{w:?}");

        k.open().unwrap();
        let w = k.dev.transport().take_writes();
        assert_eq!(w, demod_wake(0x10), "no tuner wake");

        k.tune(&TuneRequest::isdb_t(557_142_857, 6_000_000)).unwrap();
        let w = k.dev.transport().take_writes();
        let mut pre = ts_pins(0x10, 1);
        pre.push(Write::Batch(vec![(0x10, 0x25, vec![0x00]), (0x10, 0x23, vec![0x4D])]));
        assert_eq!(&w[..4], &pre[..]);
        let n = w.len();
        let mut post = vec![Write::Reg(0x10, 0x01, vec![0x40]), Write::Reg(0x10, 0x23, vec![0x4C])];
        post.extend(ts_pins(0x10, 0));
        assert_eq!(&w[n - 5..], &post[..]);
        assert!(w[4..n - 5].iter().all(|w| matches!(w, Write::Raw(0x10, _))), "{w:?}");
        assert_eq!(k.last_request().map(|r| r.frequency_hz), Some(557_142_857));

        let st = k.signal_status(crate::kit::field::ALL_BASIC).unwrap();
        assert_ne!(st.mask & crate::kit::field::STRENGTH, 0);
        // the strength reads go through the repeater
        assert_eq!(
            k.dev.transport().take_writes()[..2],
            [Write::Raw(0x10, vec![0xFE, 0xC2, 3]), Write::Raw(0x10, vec![0xFE, 0xC3])]
        );

        k.close().unwrap();
        let mut standby = ts_pins(0x10, 1);
        standby.push(demod_standby(0x10));
        assert_eq!(k.dev.transport().take_writes(), standby);
    }

    #[test]
    fn dth30_kits_put_the_tuner_to_standby() {
        // FC5500: its standby does nothing until parameter 4 is set; MTV808:
        // register 0x33 bit6 through the repeater
        for (name, tuner_standby) in [("TC90522_FC5500", false), ("TC90522_MTV808", true)] {
            let mut k = Tc9052xKit::new(Device::new(RegMock::default()), desc(name, 0x102, 0), 0x0218).unwrap();
            k.init().unwrap();
            let w = k.dev.transport().take_writes();
            std::assert_matches!(&w[0], Write::Batch(b) if b.len() == 16 && b[4] == (0x10, 0x0F, vec![0x17]), "{name}");
            let n = w.len();
            assert_eq!(w[n - 1], demod_standby(0x10), "{name}");
            assert_eq!(&w[n - 4..n - 1], &ts_pins(0x10, 1)[..], "{name}");
            let before = &w[n - 7..n - 4];
            let mtv808_standby = [
                Write::Raw(0x10, vec![0xFE, 0xC2, 0x33]),
                Write::Raw(0x10, vec![0xFE, 0xC3]),
                Write::Raw(0x10, vec![0xFE, 0xC2, 0x33, 0x40]),
            ];
            assert_eq!(before == mtv808_standby, tuner_standby, "{name}: {before:?}");
            k.open().unwrap();
            let w = k.dev.transport().take_writes();
            assert_eq!(&w[..2], &demod_wake(0x10), "{name}");
            assert_eq!(w.len() > 2, tuner_standby, "{name}: tuner wake {w:?}");
            k.close().unwrap();
            let w = k.dev.transport().take_writes();
            assert_eq!(w.last(), Some(&demod_standby(0x10)), "{name}");
        }
    }

    #[test]
    fn flags_bit5_configures_at_open_and_powers_down_at_close() {
        let mut k = Tc9052xKit::new(Device::new(RegMock::default()), desc("TC90522_MTV808", 0x102, 0x20), 0).unwrap();
        k.init().unwrap();
        assert_eq!(k.dev.transport().take_writes(), vec![], "Init only powers down");
        k.open().unwrap();
        let w = k.dev.transport().take_writes();
        std::assert_matches!(&w[0], Write::Batch(b) if b.len() == 16, "{:?}", w[0]);
        assert_eq!(w[4], Write::Reg(0x10, 0x75, vec![0x02]));
        assert!(k.is_open());
        k.close().unwrap();
        assert_eq!(k.dev.transport().take_writes(), vec![], "Close only powers down");
    }

    #[test]
    fn open_writes_the_gpio_after_a_failed_configuration() {
        // an IF the DT-H30/PCIEM-TS driver refuses
        let mut d = desc("TC90522_FC5500", 0x102, 0x20);
        d.val_62 = 44_000_000;
        let mut k = Tc9052xKit::new(Device::new(RegMock::default()), d, 0x0218).unwrap();
        assert!(k.open().is_err());
        assert!(!k.is_open());
        let ops = k.dev.transport().take();
        std::assert_matches!(ops.last(), Some(Op::Other), "{ops:?}");
    }

    #[test]
    fn attenuator_with_the_series_driver_only() {
        let mut k = Tc9052xKit::new(Device::new(RegMock::default()), desc("TC90532_MTV808", 0x102, 0), 0x01FF).unwrap();
        k.init().unwrap();
        k.dev.transport().take();
        // the configuration's 0x0E = 77 selects 0x11 for the switch, its 0x0F = 10 0x10 for the read-back
        assert!(k.set_attenuator(true).unwrap());
        assert_eq!(k.dev.transport().take(), vec![Op::R(0x10, 0x0E), Op::R(0x11, 0x1C), Op::W(0x11, 0x1C, 0x0C)]);
        assert_eq!(k.attenuator().unwrap(), Some(false));

        let mut k = Tc9052xKit::new(Device::new(RegMock::default()), desc("TC90532_MTV808", 0x102, 0), 0x0218).unwrap();
        assert!(!k.set_attenuator(true).unwrap());
        assert_eq!(k.attenuator().unwrap(), None);
        assert_eq!(k.dev.transport().take(), vec![]);
    }
}
