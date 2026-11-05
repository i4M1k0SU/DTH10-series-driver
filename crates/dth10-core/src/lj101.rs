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

    /// LNB power: demodulator pin (mode 1) and, for model "h70", the bridge
    /// GPIO supply (mode 2, 0 / 110 / 150 = off / 11 V / 15 V). Returns
    /// whether the bridge reported the supply as healthy (`false` = switched
    /// off again by the over-current check, see `device::lnb_set`).
    pub fn lnb_power(&mut self, volt: u32) -> Result<bool> {
        if let Err(e) = self.demod.lnb_enable(&mut self.dev, volt != 0) {
            log::debug!("demod LNB pin: {e}");
        }
        if self.desc.model_is("h70") {
            return device::lnb_set(&self.dev, volt);
        }
        Ok(true)
    }
}
