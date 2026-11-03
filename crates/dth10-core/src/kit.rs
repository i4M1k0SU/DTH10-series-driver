//! The `TC90517_MxL13xRF` tuner kit: open/close/tune/status
//! orchestration over the bridge, the demodulator and the tuner.

use crate::device::Device;
use crate::error::{Error, Result};
use crate::mxl13xrf::{Mxl13xRf, MxlType};
use crate::protocol::UsbControl;
use crate::tc90517::{SignalStatus, Tc90517, TerVariant, TuneRequest};
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
        st.mask |= crate::tc90517::field::STRENGTH;
        Ok(st)
    }

    pub fn last_request(&self) -> Option<&TuneRequest> {
        self.last_request.as_ref()
    }
}
