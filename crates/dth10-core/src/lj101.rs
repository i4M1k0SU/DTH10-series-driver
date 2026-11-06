//! ISDB-S (BS / CS110) tuner kit "SMT_LJ101" with hardware version 0x402.
//! See `docs/12-isdbs-kit.md`.
//!
//! * Demodulator: TC90517 satellite side (TC90522-compatible register map),
//!   I2C address 0x19 (0x18 = terrestrial side, rewritten too when
//!   `TunerDesc.flags` bit5 is set).
//! * Tuner: unidentified module, write-only through the demodulator's 0xFE
//!   I2C repeater; PLL word = IF in MHz (×2 below 1075 MHz) + VCO band.
//! * TSID selection is separate from tuning (`set_tsid`, registers 0x8F/0x90).

pub mod demod;
pub mod tuner;

pub use demod::Tc90517Sat;
pub use tuner::{Lj101Tuner, PllWords, pll_words};

use core::time::Duration;
use std::thread::sleep;

use crate::device::{self, Device};
use crate::error::{Error, Result};
use crate::kit::SignalStatus;
use crate::protocol::UsbControl;
use crate::tlv::TunerDesc;

/// The ISDB-S kit: open/close/tune/status orchestration.
pub struct Lj101Kit<T: UsbControl> {
    pub dev: Device<T>,
    pub desc: TunerDesc,
    pub demod: Tc90517Sat,
    pub tuner: Lj101Tuner,
    open_count: u32,
}

impl<T: UsbControl> Lj101Kit<T> {
    pub fn new(dev: Device<T>, desc: TunerDesc) -> Result<Self> {
        if !desc.kit_name.starts_with("SMT_LJ101") || desc.hw_version != 0x402 {
            // Other satellite kits of the series are not implemented yet (no
            // such unit was available to check the vendor driver's tables).
            return Err(Error::Unsupported(format!(
                "satellite tuner kit {} hw 0x{:x} is not implemented yet (implemented: SMT_LJ101 hw 0x402)",
                desc.kit_name, desc.hw_version
            )));
        }
        Ok(Lj101Kit {
            demod: Tc90517Sat::new(desc.demod_i2c_addr),
            tuner: Lj101Tuner { addr: desc.tuner_i2c_addr, direct: desc.tuner_direct_i2c() },
            dev,
            desc,
            open_count: 0,
        })
    }

    /// Sleep sequence: tuner standby then demodulator sleep.
    fn standby(&mut self) -> Result<()> {
        self.tuner.standby(&mut self.dev, self.demod.addr)?;
        self.demod.sleep(&mut self.dev)
    }

    /// Init.
    pub fn init(&mut self) -> Result<()> {
        if self.desc.skip_init_config() {
            return self.dev.power_down(&self.desc);
        }
        self.dev.power_up(&self.desc)?;
        self.demod.configure(&mut self.dev, &self.desc)?;
        self.tuner.init(&mut self.dev, self.demod.addr)?;
        self.standby()
    }

    /// Open.
    pub fn open(&mut self) -> Result<()> {
        self.open_count += 1;
        if self.open_count != 1 {
            return Ok(());
        }
        let r = (|| {
            if self.desc.skip_init_config() {
                self.dev.power_up(&self.desc)?;
                self.demod.configure(&mut self.dev, &self.desc)?;
                self.tuner.init(&mut self.dev, self.demod.addr)?;
            } else {
                self.demod.wake(&mut self.dev)?;
                self.tuner.wake(&mut self.dev, self.demod.addr)?;
            }
            self.dev.gpio_write(self.desc.gpio_mask_c, self.desc.gpio_c_open)
        })();
        if r.is_err() {
            self.open_count = 0;
        }
        r
    }

    /// Close.
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

    /// Tune to a BS/CS IF (RF − 10.678 GHz) in Hz.
    pub fn tune(&mut self, if_hz: u64) -> Result<()> {
        if !self.is_open() {
            return Err(Error::Protocol("kit is not open"));
        }
        self.demod.set_ts_pins(&mut self.dev, 1)?;
        self.tuner.set_frequency(&mut self.dev, self.demod.addr, if_hz)?;
        sleep(Duration::from_millis(100));
        self.demod.restart(&mut self.dev)?;
        sleep(Duration::from_millis(150));
        self.demod.set_ts_pins(&mut self.dev, 0)
    }

    /// Select the transport stream by TSID (a kit operation separate from tuning).
    pub fn set_tsid(&mut self, tsid: u16) -> Result<()> {
        self.demod.set_tsid(&mut self.dev, tsid)
    }

    /// Diagnostic read-back of the demodulator's TS/output-related registers:
    /// `[0x03, 0x04, 0x17, 0x1C, 0x1D, 0x1E, 0x1F, 0xA5]`.
    pub fn debug_regs(&mut self) -> Result<[u8; 8]> {
        let mut out = [0u8; 8];
        for (i, r) in [0x03u8, 0x04, 0x17, 0x1C, 0x1D, 0x1E, 0x1F, 0xA5].iter().enumerate() {
            let mut b = [0u8; 1];
            self.dev.i2c_reg_read(self.demod.addr, *r, &mut b)?;
            out[i] = b[0];
        }
        Ok(out)
    }

    /// Diagnostic: re-run the TS pin control (0 = output, 1 = stopped).
    pub fn ts_pins(&mut self, mode: u8) -> Result<()> {
        self.demod.set_ts_pins(&mut self.dev, mode)
    }

    pub fn signal_status(&mut self, mask: u32) -> Result<SignalStatus> {
        let mut st = SignalStatus { mask, ..Default::default() };
        self.demod.status(&mut self.dev, &mut st)?;
        Ok(st)
    }

    /// Antenna (LNB) power: for model "h70" the bridge GPIO supply (0 / 110 /
    /// 150 = off / 11 V / 15 V); no effect on other models, as in the vendor
    /// driver. Returns whether the bridge reported the supply as healthy
    /// (`false` = switched off again by the over-current check, see
    /// `device::lnb_set`). The booster is a separate switch ([`set_booster`](Self::set_booster)).
    pub fn lnb_power(&mut self, volt: u32) -> Result<bool> {
        if self.desc.model_is("h70") {
            return device::lnb_set(&self.dev, volt);
        }
        Ok(true)
    }

    /// The booster (the vendor application's "use booster" for BS/CS): the
    /// module's switch bits in the demodulator register 0x1C. It stays set
    /// until the next kit Init. `Ok(false)`: the module selects no chip for
    /// it, nothing was written.
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
    use crate::regmock::{Op, RegMock};

    fn kit(model: &str, regs: &[(u8, u8, u8)]) -> Lj101Kit<RegMock> {
        let desc = TunerDesc {
            kit_name: "SMT_LJ101".into(),
            hw_version: 0x402,
            demod_i2c_addr: 0x19,
            model: model.into(),
            ..Default::default()
        };
        Lj101Kit::new(Device::new(RegMock::with_regs(regs)), desc).unwrap()
    }

    #[test]
    fn booster_switches_bits_1_0_of_the_selected_chip() {
        // 0x0F = 0x34 (the Open configuration): bits 5:4 = 11 -> 0x19
        let mut k = kit("h70", &[(0x18, 0x0F, 0x34), (0x19, 0x1C, 0xFA), (0x18, 0x1C, 0x28)]);
        assert!(k.set_booster(true).unwrap());
        assert_eq!(k.dev.transport().take(), vec![Op::R(0x18, 0x0F), Op::R(0x19, 0x1C), Op::W(0x19, 0x1C, 0xFB)]);
        assert!(k.booster().unwrap());
        assert!(k.set_booster(false).unwrap());
        assert_eq!(k.dev.transport().reg(0x19, 0x1C), 0xFA);
        assert!(!k.booster().unwrap());
        assert_eq!(k.dev.transport().reg(0x18, 0x1C), 0x28, "the other chip is left alone");

        // bits 5:4 = 01 -> 0x18
        let mut k = kit("h70", &[(0x18, 0x0F, 0x14), (0x18, 0x1C, 0x28)]);
        assert!(k.set_booster(true).unwrap());
        assert_eq!(k.dev.transport().take(), vec![Op::R(0x18, 0x0F), Op::R(0x18, 0x1C), Op::W(0x18, 0x1C, 0x2B)]);
    }

    #[test]
    fn booster_is_left_alone_when_no_chip_is_selected() {
        for sel in [0x04u8, 0x24] {
            let mut k = kit("h70", &[(0x18, 0x0F, sel), (0x19, 0x1C, 0x0A)]);
            assert!(!k.set_booster(true).unwrap(), "0x0F = {sel:#04x}");
            assert_eq!(k.dev.transport().take(), vec![Op::R(0x18, 0x0F)]);
        }
    }

    #[test]
    fn lnb_power_does_not_touch_the_booster() {
        // another model: no effect at all
        let mut k = kit("", &[(0x18, 0x0F, 0x34), (0x19, 0x1C, 0x0A)]);
        assert!(k.lnb_power(150).unwrap());
        assert!(k.lnb_power(0).unwrap());
        assert_eq!(k.dev.transport().take(), vec![]);
        // model "h70": the bridge GPIO only, no I2C
        let mut k = kit("h70", &[(0x18, 0x0F, 0x34), (0x19, 0x1C, 0x0A)]);
        let _ = k.lnb_power(150).unwrap();
        let ops = k.dev.transport().take();
        assert!(!ops.is_empty() && ops.iter().all(|op| matches!(op, Op::Other)), "{ops:?}");
        assert_eq!(k.dev.transport().reg(0x19, 0x1C), 0x0A);
    }
}
