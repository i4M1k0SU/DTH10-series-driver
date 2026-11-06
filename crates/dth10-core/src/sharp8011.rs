//! The `SHARP_8011` tuner kits: a variant of the SMT_LJ101 module with the
//! same TC90517 demodulator and I2C repeater, its own register values and
//! two PLL tuners of its own, and no MaxLinear tuner. The vendor driver
//! builds them by the descriptor's hw version:
//!
//! * hw 0x102: [`Sharp8011Kit`], the terrestrial kit (ISDB-T). It drives the
//!   demodulator with the SMT_LJ101 hw 0x102 kit's code (wake, standby,
//!   pre- / post-tune, status, attenuator; [`TerVariant::SmtLj101`]) and has
//!   no tuner init, wake, standby or signal reading: the signal strength is
//!   the demodulator's.
//! * hw 0x402: [`Sharp8011SatKit`], the satellite kit (ISDB-S). It drives the
//!   demodulator's satellite side with the SMT_LJ101 hw 0x402 kit's code (TS
//!   pins, sleep / wake, restart, status, TSID, booster, antenna supply) and
//!   tunes in the same order; its sleep is the demodulator's alone.
//!
//! Neither kit reads the descriptor's tuner crystal, IF, tuner options or
//! demodulator crystal.

pub mod demod;
pub mod tuner;

pub use tuner::{SatPll, SatTuner, TerTuner, sat_pll, ter_pll_bytes};

use core::time::Duration;
use std::thread::sleep;

use crate::device::{self, Device};
use crate::error::{Error, Result};
use crate::kit::{SignalStatus, TuneRequest};
use crate::lj101::Tc90517Sat;
use crate::protocol::UsbControl;
use crate::tc90517::{Tc90517, TerVariant};
use crate::tlv::TunerDesc;
use crate::tuner_bus::{TunerBus, TunerPath};

/// The kit name both kits report.
pub const KIT_NAME: &str = "SHARP_8011";

/// The tuner's I2C path: through the repeater of the demodulator at
/// `demod_addr`, or directly when the descriptor's flags bit3 says so.
fn tuner_bus(desc: &TunerDesc) -> TunerBus {
    let path = if desc.tuner_direct_i2c() {
        TunerPath::Direct
    } else {
        TunerPath::ViaDemod { demod_addr: desc.demod_i2c_addr }
    };
    TunerBus { tuner_addr: desc.tuner_i2c_addr, path }
}

/// The terrestrial kit (`SHARP_8011` hw 0x102).
pub struct Sharp8011Kit<T: UsbControl> {
    pub dev: Device<T>,
    pub desc: TunerDesc,
    pub demod: Tc90517,
    pub tuner: TerTuner,
    open_count: u32,
    last_request: Option<TuneRequest>,
}

impl<T: UsbControl> Sharp8011Kit<T> {
    /// Whether `desc` names this kit.
    pub fn supports(desc: &TunerDesc) -> bool {
        desc.kit_name == KIT_NAME && desc.hw_version == 0x102
    }

    pub fn new(dev: Device<T>, desc: TunerDesc) -> Result<Self> {
        if !Self::supports(&desc) {
            return Err(Error::Unsupported(format!(
                "terrestrial tuner kit {} hw 0x{:x} is not the SHARP_8011 hw 0x102 kit",
                desc.kit_name, desc.hw_version
            )));
        }
        Ok(Sharp8011Kit {
            demod: Tc90517::with_variant(desc.demod_i2c_addr, TerVariant::SmtLj101),
            tuner: TerTuner { bus: tuner_bus(&desc) },
            dev,
            desc,
            open_count: 0,
            last_request: None,
        })
    }

    /// Init: power-up, demodulator configuration, demodulator standby
    /// (brought up again by [`open`](Self::open)). With flags bit5 set only
    /// the power-down, as in the vendor driver.
    pub fn init(&mut self) -> Result<()> {
        if self.desc.skip_init_config() {
            return self.dev.power_down(&self.desc);
        }
        self.dev.power_up(&self.desc)?;
        demod::configure_ter(&mut self.dev, self.demod.addr)?;
        self.last_request = None;
        self.demod.standby(&mut self.dev)
    }

    /// Open (reference counted): demodulator wake (with flags bit5 set:
    /// power-up and configuration), then GPIO mask C <- open value.
    pub fn open(&mut self) -> Result<()> {
        self.open_count += 1;
        if self.open_count != 1 {
            return Ok(());
        }
        let r = (|| {
            if self.desc.skip_init_config() {
                self.dev.power_up(&self.desc)?;
                demod::configure_ter(&mut self.dev, self.demod.addr)?;
                self.last_request = None;
            } else {
                self.demod.wake(&mut self.dev)?;
            }
            self.dev.gpio_write(self.desc.gpio_mask_c, self.desc.gpio_c_open)
        })();
        if r.is_err() {
            self.open_count = 0;
        }
        r
    }

    /// Close: demodulator standby (with flags bit5 set: power-down). The
    /// vendor driver leaves the GPIO of mask C as Open set it.
    pub fn close(&mut self) -> Result<()> {
        if self.open_count == 0 {
            return Ok(());
        }
        self.open_count -= 1;
        if self.open_count != 0 {
            return Ok(());
        }
        if self.desc.skip_init_config() { self.dev.power_down(&self.desc) } else { self.demod.standby(&mut self.dev) }
    }

    pub fn is_open(&self) -> bool {
        self.open_count > 0
    }

    /// Tune: demodulator pre-tune, the PLL write, demodulator post-tune.
    /// The bandwidth is not used.
    pub fn tune(&mut self, req: &TuneRequest) -> Result<()> {
        if !self.is_open() {
            return Err(Error::Protocol("kit is not open"));
        }
        self.demod.pre_tune(&mut self.dev)?;
        self.tuner.set_frequency(&mut self.dev, req.frequency_hz)?;
        self.demod.post_tune(&mut self.dev)?;
        self.last_request = Some(*req);
        Ok(())
    }

    /// Signal status: the demodulator's alone (its strength is `18000 − C/N`).
    pub fn signal_status(&mut self, mask: u32) -> Result<SignalStatus> {
        let mut st = SignalStatus { mask, ..Default::default() };
        self.demod.status(&mut self.dev, &mut st)?;
        Ok(st)
    }

    pub fn last_request(&self) -> Option<&TuneRequest> {
        self.last_request.as_ref()
    }

    /// The attenuator (the SMT_LJ101 hw 0x102 kit's switch, the same code in
    /// the vendor driver). The configuration sets `0x0F = 0x5F`, which selects
    /// the chip at 0x19 for it. `Ok(false)`: not switched.
    pub fn set_attenuator(&mut self, on: bool) -> Result<bool> {
        self.demod.set_attenuator(&mut self.dev, on)
    }

    /// Whether the attenuator is on; `None` when the module selects no chip for it.
    pub fn attenuator(&mut self) -> Result<Option<bool>> {
        self.demod.attenuator(&mut self.dev)
    }
}

/// The satellite kit (`SHARP_8011` hw 0x402).
pub struct Sharp8011SatKit<T: UsbControl> {
    pub dev: Device<T>,
    pub desc: TunerDesc,
    pub demod: Tc90517Sat,
    pub tuner: SatTuner,
    open_count: u32,
}

impl<T: UsbControl> Sharp8011SatKit<T> {
    /// Whether `desc` names this kit.
    pub fn supports(desc: &TunerDesc) -> bool {
        desc.kit_name == KIT_NAME && desc.hw_version == 0x402
    }

    pub fn new(dev: Device<T>, desc: TunerDesc) -> Result<Self> {
        if !Self::supports(&desc) {
            return Err(Error::Unsupported(format!(
                "satellite tuner kit {} hw 0x{:x} is not the SHARP_8011 hw 0x402 kit",
                desc.kit_name, desc.hw_version
            )));
        }
        Ok(Sharp8011SatKit {
            demod: Tc90517Sat::new(desc.demod_i2c_addr),
            tuner: SatTuner { bus: tuner_bus(&desc) },
            dev,
            desc,
            open_count: 0,
        })
    }

    /// Sleep: TS output stopped, `0x17 <- 1`.
    fn sleep(&mut self) -> Result<()> {
        self.demod.set_ts_pins(&mut self.dev, 1)?;
        self.demod.sleep(&mut self.dev)
    }

    /// Init: power-up, demodulator configuration, sleep. With flags bit5 set
    /// only the power-down, as in the vendor driver.
    pub fn init(&mut self) -> Result<()> {
        if self.desc.skip_init_config() {
            return self.dev.power_down(&self.desc);
        }
        self.dev.power_up(&self.desc)?;
        demod::configure_sat(&mut self.dev, self.demod.addr, &self.desc)?;
        self.sleep()
    }

    /// Open (reference counted): TS output on and `0x17 <- 0` (with flags
    /// bit5 set: power-up and configuration), then GPIO mask C <- open value.
    /// As in the vendor driver, a failure of the TS pin step does not stop
    /// the Open.
    pub fn open(&mut self) -> Result<()> {
        self.open_count += 1;
        if self.open_count != 1 {
            return Ok(());
        }
        let r = (|| {
            if self.desc.skip_init_config() {
                self.dev.power_up(&self.desc)?;
                demod::configure_sat(&mut self.dev, self.demod.addr, &self.desc)?;
            } else {
                if let Err(e) = self.demod.set_ts_pins(&mut self.dev, 0) {
                    log::debug!("SHARP_8011 satellite Open, TS pins: {e}");
                }
                self.demod.wake(&mut self.dev)?;
            }
            self.dev.gpio_write(self.desc.gpio_mask_c, self.desc.gpio_c_open)
        })();
        if r.is_err() {
            self.open_count = 0;
        }
        r
    }

    /// Close: sleep (with flags bit5 set: power-down).
    pub fn close(&mut self) -> Result<()> {
        if self.open_count == 0 {
            return Ok(());
        }
        self.open_count -= 1;
        if self.open_count != 0 {
            return Ok(());
        }
        if self.desc.skip_init_config() { self.dev.power_down(&self.desc) } else { self.sleep() }
    }

    pub fn is_open(&self) -> bool {
        self.open_count > 0
    }

    /// Tune to a BS/CS IF (RF − 10.678 GHz) in Hz, in the SMT_LJ101 hw 0x402
    /// kit's order: TS stop, PLL, 100 ms, demodulator restart, 150 ms, TS on.
    pub fn tune(&mut self, if_hz: u64) -> Result<()> {
        if !self.is_open() {
            return Err(Error::Protocol("kit is not open"));
        }
        self.demod.set_ts_pins(&mut self.dev, 1)?;
        self.tuner.set_frequency(&mut self.dev, if_hz)?;
        sleep(Duration::from_millis(100));
        let r = self.demod.restart(&mut self.dev);
        sleep(Duration::from_millis(150));
        r?;
        self.demod.set_ts_pins(&mut self.dev, 0)
    }

    /// Select the transport stream by TSID.
    pub fn set_tsid(&mut self, tsid: u16) -> Result<()> {
        self.demod.set_tsid(&mut self.dev, tsid)
    }

    pub fn signal_status(&mut self, mask: u32) -> Result<SignalStatus> {
        let mut st = SignalStatus { mask, ..Default::default() };
        self.demod.status(&mut self.dev, &mut st)?;
        Ok(st)
    }

    /// Antenna (LNB) power, as with the SMT_LJ101 hw 0x402 kit: the bridge
    /// GPIO supply of the models that switch it there, no effect on others.
    pub fn lnb_power(&mut self, volt: u32) -> Result<bool> {
        if let Some(gpio) = device::LnbGpio::of(&self.desc) {
            return device::lnb_set(&self.dev, gpio, volt);
        }
        Ok(true)
    }

    /// The booster, as with the SMT_LJ101 hw 0x402 kit (the same code in the
    /// vendor driver). `Ok(false)`: the module selects no chip for it.
    pub fn set_booster(&mut self, on: bool) -> Result<bool> {
        self.demod.set_booster(&mut self.dev, on)
    }

    /// Whether the booster is on (read back from the demodulator).
    pub fn booster(&mut self) -> Result<bool> {
        self.demod.booster(&mut self.dev)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regmock::{Op, RegMock, Write};

    fn desc(hw: u32, flags: u32) -> TunerDesc {
        TunerDesc {
            kit_name: KIT_NAME.into(),
            hw_version: hw,
            flags,
            demod_i2c_addr: if hw == 0x402 { 0x19 } else { 0x18 },
            tuner_i2c_addr: 0x61,
            gpio_mask_c: 0x10,
            gpio_c_open: 0x10,
            gpio_c_close: 0,
            ..Default::default()
        }
    }

    #[test]
    fn kits_accept_their_hw_version_only() {
        let dev = || Device::new(RegMock::default());
        assert!(Sharp8011Kit::new(dev(), desc(0x102, 0)).is_ok());
        assert!(Sharp8011Kit::new(dev(), desc(0x402, 0)).is_err());
        assert!(Sharp8011SatKit::new(dev(), desc(0x402, 0)).is_ok());
        assert!(Sharp8011SatKit::new(dev(), desc(0x102, 0)).is_err());
        let smt = TunerDesc { kit_name: "SMT_LJ101".into(), ..desc(0x102, 0) };
        assert!(!Sharp8011Kit::<RegMock>::supports(&smt));
    }

    #[test]
    fn terrestrial_init_open_tune_close() {
        let mut k = Sharp8011Kit::new(Device::new(RegMock::default()), desc(0x102, 0)).unwrap();
        k.init().unwrap();
        let w = k.dev.transport().take_writes();
        // the configuration (4 batches + 0x75), then the standby batches
        assert_eq!(w.len(), 5 + 2, "{w:?}");
        assert_eq!(w[4], Write::Reg(0x18, 0x75, vec![0x02]));
        assert_eq!(w[5], Write::Batch(vec![(0x18, 0x1E, vec![0xAA]), (0x18, 0x1F, vec![0xA8])]));
        assert_eq!(w[6], Write::Batch(vec![(0x18, 0x1C, vec![0x80]), (0x18, 0x03, vec![0x90])]));
        assert!(k.tune(&TuneRequest::isdb_t(557_142_857, 6_000_000)).is_err(), "not open");

        k.open().unwrap();
        assert_eq!(
            k.dev.transport().take_writes(),
            vec![Write::Reg(0x18, 0x03, vec![0]), Write::Reg(0x18, 0x1C, vec![0])]
        );
        k.tune(&TuneRequest::isdb_t(557_142_857, 6_000_000)).unwrap();
        assert_eq!(
            k.dev.transport().take_writes(),
            vec![
                // pre-tune: TS stop, 0x25 <- 0, 0x23 |= 1 (0x23 = 0x78 from the configuration)
                Write::Batch(vec![(0x18, 0x1E, vec![0xAA]), (0x18, 0x1F, vec![0xA8])]),
                Write::Batch(vec![(0x18, 0x25, vec![0x00]), (0x18, 0x23, vec![0x79])]),
                // the PLL through the repeater
                Write::Raw(0x18, vec![0xFE, 0xC2, 0x10, 0xCB, 0xC1, 0xA4, 0x80]),
                // post-tune: 0x01 <- 40, 0x23 &= ~1, TS on
                Write::Reg(0x18, 0x01, vec![0x40]),
                Write::Reg(0x18, 0x23, vec![0x78]),
                Write::Batch(vec![(0x18, 0x1E, vec![0xA2]), (0x18, 0x1F, vec![0x08])]),
            ]
        );
        assert_eq!(k.last_request().map(|r| r.frequency_hz), Some(557_142_857));
        k.close().unwrap();
        assert_eq!(
            k.dev.transport().take_writes(),
            vec![
                Write::Batch(vec![(0x18, 0x1E, vec![0xAA]), (0x18, 0x1F, vec![0xA8])]),
                Write::Batch(vec![(0x18, 0x1C, vec![0x80]), (0x18, 0x03, vec![0x90])]),
            ]
        );
    }

    #[test]
    fn terrestrial_attenuator_is_on_the_chip_the_configuration_selects() {
        let mut k = Sharp8011Kit::new(Device::new(RegMock::default()), desc(0x102, 0)).unwrap();
        k.init().unwrap();
        k.dev.transport().take();
        assert!(k.set_attenuator(true).unwrap());
        assert_eq!(k.dev.transport().take(), vec![Op::R(0x18, 0x0F), Op::R(0x19, 0x1C), Op::W(0x19, 0x1C, 0x0C)]);
        assert_eq!(k.attenuator().unwrap(), Some(true));
    }

    #[test]
    fn terrestrial_with_flags_bit5_configures_at_open() {
        let mut k = Sharp8011Kit::new(Device::new(RegMock::default()), desc(0x102, 0x20)).unwrap();
        k.init().unwrap();
        assert_eq!(k.dev.transport().take_writes(), vec![], "Init only powers down");
        k.open().unwrap();
        let w = k.dev.transport().take_writes();
        assert_eq!(w.len(), 5);
        assert_eq!(w[4], Write::Reg(0x18, 0x75, vec![0x02]));
        k.close().unwrap();
        assert_eq!(k.dev.transport().take_writes(), vec![], "Close only powers down");
    }

    #[test]
    fn satellite_init_open_tune_close() {
        let mut k = Sharp8011SatKit::new(Device::new(RegMock::default()), desc(0x402, 0)).unwrap();
        k.init().unwrap();
        let w = k.dev.transport().take_writes();
        assert_eq!(w.len(), 4 + 2, "{w:?}");
        assert_eq!(w[0], Write::Reg(0x19, 0x01, vec![0x90]));
        assert_eq!(w[3], Write::Reg(0x19, 0x04, vec![0x02]));
        // sleep: TS stopped (0x1C = 0 from the configuration), 0x17 <- 1
        assert_eq!(w[4], Write::Batch(vec![(0x19, 0x1C, vec![0x80]), (0x19, 0x1F, vec![0xAA])]));
        assert_eq!(w[5], Write::Reg(0x19, 0x17, vec![0x01]));

        k.open().unwrap();
        assert_eq!(
            k.dev.transport().take_writes(),
            vec![
                Write::Batch(vec![(0x19, 0x1C, vec![0x00]), (0x19, 0x1F, vec![0x88])]),
                Write::Reg(0x19, 0x17, vec![0])
            ]
        );
        k.tune(1_318_000_000).unwrap();
        let w = (41u16 << 5) | 6;
        assert_eq!(
            k.dev.transport().take_writes(),
            vec![
                Write::Batch(vec![(0x19, 0x1C, vec![0x80]), (0x19, 0x1F, vec![0xAA])]),
                Write::Raw(0x19, vec![0xFE, 0xC2, (w >> 8) as u8, w as u8, 0xE0, 0x40]),
                Write::Raw(0x19, vec![0xFE, 0xC2, 0xE4]),
                Write::Raw(0x19, vec![0xFE, 0xC2, 0xF4, 0x44]),
                Write::Raw(0x19, vec![0xFE, 0xC2, (w >> 8) as u8]),
                Write::Reg(0x19, 0x03, vec![0x01]),
                Write::Reg(0x19, 0xA5, vec![0x01]),
                Write::Batch(vec![(0x19, 0x1C, vec![0x00]), (0x19, 0x1F, vec![0x88])]),
            ]
        );
        k.set_tsid(0x40F1).unwrap();
        assert_eq!(
            k.dev.transport().take_writes(),
            vec![Write::Batch(vec![(0x19, 0x8F, vec![0x40]), (0x19, 0x90, vec![0xF1])])]
        );
        k.close().unwrap();
        assert_eq!(
            k.dev.transport().take_writes(),
            vec![
                Write::Batch(vec![(0x19, 0x1C, vec![0x80]), (0x19, 0x1F, vec![0xAA])]),
                Write::Reg(0x19, 0x17, vec![0x01])
            ]
        );
    }

    #[test]
    fn satellite_with_flags_bit5_keeps_the_descriptor_address() {
        let mut k = Sharp8011SatKit::new(Device::new(RegMock::default()), desc(0x402, 0x20)).unwrap();
        k.init().unwrap();
        assert_eq!(k.dev.transport().take_writes(), vec![]);
        k.open().unwrap();
        let w = k.dev.transport().take_writes();
        assert_eq!(w.len(), 4);
        assert!(
            w.iter().all(|w| match w {
                Write::Reg(a, ..) => *a == 0x19,
                Write::Batch(e) => e.iter().all(|(a, ..)| *a == 0x19),
                Write::Raw(..) => false,
            }),
            "{w:?}"
        );
        assert_eq!(k.demod.addr, 0x19);
    }

    #[test]
    fn satellite_lnb_power_follows_the_model() {
        let mut k = Sharp8011SatKit::new(Device::new(RegMock::default()), desc(0x402, 0)).unwrap();
        assert!(k.lnb_power(150).unwrap());
        assert_eq!(k.dev.transport().take(), vec![]);
        let mut d = desc(0x402, 0);
        d.model = "h70".into();
        let mut k = Sharp8011SatKit::new(Device::new(RegMock::default()), d).unwrap();
        let _ = k.lnb_power(150).unwrap();
        let ops = k.dev.transport().take();
        assert!(!ops.is_empty() && ops.iter().all(|op| matches!(op, Op::Other)), "{ops:?}");
    }
}
