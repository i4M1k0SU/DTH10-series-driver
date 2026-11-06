//! The `TC90517_MxL13xRF` tuner kit (and `SMT_LJ101` hw 0x102, which drives
//! the same demodulator): open/close/tune/status orchestration over the
//! bridge, the demodulator ([`demod`]) and the MaxLinear tuner.

pub mod demod;

pub use demod::{Tc90517, TerVariant, ber_e9, cnr_mdb_from_raw};

use crate::device::Device;
use crate::error::{Error, Result};
use crate::kit::{SignalStatus, TuneRequest};
use crate::mxl13xrf::{Mxl13xRf, MxlType};
use crate::protocol::UsbControl;
use crate::tlv::TunerDesc;
use crate::tuner_bus::{TunerBus, TunerPath};

pub struct Tc90517Kit<T: UsbControl> {
    pub dev: Device<T>,
    pub desc: TunerDesc,
    pub demod: Tc90517,
    pub tuner: Mxl13xRf,
    open_count: u32,
    last_request: Option<TuneRequest>,
}

impl<T: UsbControl> Tc90517Kit<T> {
    /// Build the kit from the device descriptor: `TC90517_MxL13xRF`
    /// or `SMT_LJ101` with hw version 0x102 (what the DT-H70/U2 reports for
    /// tuner #0); both build the same MxL tuner object.
    pub fn new(dev: Device<T>, desc: TunerDesc) -> Result<Self> {
        let variant = if desc.kit_name.starts_with("TC90517_") {
            TerVariant::Tc90517Mxl
        } else if desc.kit_name.starts_with("SMT_LJ101") && desc.hw_version == 0x102 {
            TerVariant::SmtLj101
        } else {
            // The other kits have their own types ([`crate::terrestrial`]).
            return Err(Error::Unsupported(format!(
                "terrestrial tuner kit {} hw 0x{:x} is not a TC90517 kit",
                desc.kit_name, desc.hw_version
            )));
        };
        // The SMT_LJ101 kit name has no tuner suffix; its MxL object is type -1 (unused).
        let kind = MxlType::from_kit_name(&desc.kit_name).unwrap_or(MxlType::MxL5007);
        let path = if desc.tuner_direct_i2c() {
            TunerPath::Direct
        } else {
            TunerPath::ViaDemod { demod_addr: desc.demod_i2c_addr }
        };
        Ok(Tc90517Kit {
            demod: Tc90517::with_variant(desc.demod_i2c_addr, variant),
            tuner: Mxl13xRf::new(TunerBus { tuner_addr: desc.tuner_i2c_addr, path }, kind),
            dev,
            desc,
            open_count: 0,
            last_request: None,
        })
    }

    /// Init: power-up, demod configure, tuner init, then both to
    /// standby (the chips are brought up again by [`open`](Self::open)).
    pub fn init(&mut self) -> Result<()> {
        if self.desc.skip_init_config() {
            self.dev.power_down(&self.desc)?;
            return Ok(());
        }
        self.dev.power_up(&self.desc)?;
        self.demod.configure(&mut self.dev, &self.desc)?;
        self.tuner_init()?;
        self.tuner.standby(&mut self.dev)?;
        self.demod.standby(&mut self.dev)
    }

    fn tuner_init(&mut self) -> Result<()> {
        match self.demod.variant {
            TerVariant::Tc90517Mxl => self.tuner.init(&mut self.dev, &self.desc),
            TerVariant::SmtLj101 => self.tuner.init_lj101(&mut self.dev),
            // Built by the TC90522 / TC90532 kits only, never by this one.
            TerVariant::Tc9052x(_) => Err(Error::Protocol("not a TC90517 kit variant")),
        }
    }

    /// Open (reference counted).
    pub fn open(&mut self) -> Result<()> {
        self.open_count += 1;
        if self.open_count != 1 {
            return Ok(());
        }
        let r = self.open_inner();
        if r.is_err() {
            self.open_count = 0;
        }
        r
    }

    fn open_inner(&mut self) -> Result<()> {
        if self.desc.skip_init_config() {
            self.dev.power_up(&self.desc)?;
            self.demod.configure(&mut self.dev, &self.desc)?;
            self.tuner_init()?;
            self.last_request = None;
        } else {
            self.demod.wake(&mut self.dev)?;
            self.tuner.wake(&mut self.dev)?;
        }
        // GPIO output: mask C ← open value
        self.dev.gpio_write(self.desc.gpio_mask_c, self.desc.gpio_c_open)
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
        self.dev.gpio_write(self.desc.gpio_mask_c, self.desc.gpio_c_close)?;
        if self.desc.skip_init_config() {
            self.dev.power_down(&self.desc)
        } else {
            self.tuner.standby(&mut self.dev)?;
            self.demod.standby(&mut self.dev)
        }
    }

    pub fn is_open(&self) -> bool {
        self.open_count > 0
    }

    /// Tune.
    pub fn tune(&mut self, req: &TuneRequest) -> Result<()> {
        if !self.is_open() {
            return Err(Error::Protocol("kit is not open"));
        }
        // The original driver's per-model tune hook only acts on model "f200"; the DT-H70 path is a no-op.
        self.demod.pre_tune(&mut self.dev)?;
        self.tuner.set_frequency(&mut self.dev, req.frequency_u32(), req.bandwidth_hz)?;
        self.demod.post_tune(&mut self.dev)?;
        self.last_request = Some(*req);
        Ok(())
    }

    /// Signal status (no caching here): demodulator
    /// first, then the tuner overwrites `strength` with its own reading.
    pub fn signal_status(&mut self, mask: u32) -> Result<SignalStatus> {
        let mut st = SignalStatus { mask, ..Default::default() };
        self.demod.status(&mut self.dev, &mut st)?;
        st.strength = self.tuner.signal(&mut self.dev)?;
        st.mask |= crate::kit::field::STRENGTH;
        Ok(st)
    }

    pub fn last_request(&self) -> Option<&TuneRequest> {
        self.last_request.as_ref()
    }

    /// Whether the kit has the attenuator switch: the `SMT_LJ101` hw 0x102
    /// kit only (the vendor driver does nothing for `TC90517_MxL13xRF`).
    pub fn has_attenuator(&self) -> bool {
        self.demod.variant == TerVariant::SmtLj101
    }

    /// The attenuator (the vendor application's "use attenuator"). It stays
    /// set until the next kit Init. `Ok(false)`: not switched (no attenuator
    /// on this kit, or the module selects no chip for it).
    pub fn set_attenuator(&mut self, on: bool) -> Result<bool> {
        if !self.has_attenuator() {
            return Ok(false);
        }
        self.demod.set_attenuator(&mut self.dev, on)
    }

    /// Whether the attenuator is on; `None` when the kit has none or the
    /// module selects no chip for it.
    pub fn attenuator(&mut self) -> Result<Option<bool>> {
        if !self.has_attenuator() {
            return Ok(None);
        }
        self.demod.attenuator(&mut self.dev)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regmock::{Op, RegMock};

    fn kit(name: &str, regs: &[(u8, u8, u8)]) -> Tc90517Kit<RegMock> {
        let desc = TunerDesc {
            kit_name: name.into(),
            hw_version: 0x102,
            demod_i2c_addr: 0x18,
            tuner_i2c_addr: 0x60,
            ..Default::default()
        };
        Tc90517Kit::new(Device::new(RegMock::with_regs(regs)), desc).unwrap()
    }

    #[test]
    fn attenuator_switches_bits_3_2_of_the_selected_chip() {
        // 0x0F = 0x34 (the Open configuration): bits 3:2 = 01 -> 0x18
        let mut k = kit("SMT_LJ101", &[(0x18, 0x0F, 0x34), (0x18, 0x1C, 0x28), (0x19, 0x1C, 0x0A)]);
        assert!(k.set_attenuator(true).unwrap());
        assert_eq!(k.dev.transport().take(), vec![Op::R(0x18, 0x0F), Op::R(0x18, 0x1C), Op::W(0x18, 0x1C, 0x2C)]);
        assert_eq!(k.attenuator().unwrap(), Some(true));
        assert!(k.set_attenuator(false).unwrap());
        assert_eq!(k.dev.transport().reg(0x18, 0x1C), 0x28);
        assert_eq!(k.attenuator().unwrap(), Some(false));
        assert_eq!(k.dev.transport().reg(0x19, 0x1C), 0x0A, "the other chip is left alone");

        // bits 3:2 = 11 -> 0x19; the other bits of 0x1C are kept
        let mut k = kit("SMT_LJ101", &[(0x18, 0x0F, 0x3C), (0x19, 0x1C, 0xF3)]);
        assert!(k.set_attenuator(false).unwrap());
        assert_eq!(k.dev.transport().take(), vec![Op::R(0x18, 0x0F), Op::R(0x19, 0x1C), Op::W(0x19, 0x1C, 0xFB)]);
        assert_eq!(k.attenuator().unwrap(), Some(false));
        assert!(k.set_attenuator(true).unwrap());
        assert_eq!(k.dev.transport().reg(0x19, 0x1C), 0xFF);
    }

    #[test]
    fn attenuator_is_left_alone_when_no_chip_is_selected() {
        for sel in [0x30u8, 0x38] {
            let mut k = kit("SMT_LJ101", &[(0x18, 0x0F, sel), (0x18, 0x1C, 0x28)]);
            assert!(!k.set_attenuator(true).unwrap(), "0x0F = {sel:#04x}");
            assert_eq!(k.dev.transport().take(), vec![Op::R(0x18, 0x0F)]);
            assert_eq!(k.attenuator().unwrap(), None);
        }
    }

    #[test]
    fn tc90517_mxl_kit_has_no_attenuator() {
        let mut k = kit("TC90517_MxL135RF", &[(0x18, 0x0F, 0x34), (0x18, 0x1C, 0x28)]);
        assert!(!k.has_attenuator());
        assert!(!k.set_attenuator(true).unwrap());
        assert_eq!(k.attenuator().unwrap(), None);
        assert_eq!(k.dev.transport().take(), vec![]);
    }
}
