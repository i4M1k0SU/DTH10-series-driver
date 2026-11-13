//! The `TC90527_*` terrestrial (ISDB-T) tuner kits: a TC90527 demodulator
//! ([`demod`]) with an FC5500 or MTV808 tuner behind its I2C repeater (or
//! directly on the bridge bus when the descriptor's flags bit3 says so). The
//! vendor drivers build them by kit name alone (the hw version and model are
//! not looked at):
//!
//! | kit name | tuner | vendor driver |
//! |---|---|---|
//! | `TC90527_FC5500` | [`Fc5500`] | both (the kit follows the model's driver) |
//! | `TC90527_MTV808` | [`Mtv808`] | DT-H30/PCIEM-TS |
//!
//! The demodulator needs a 16 / 18 / 24 / 25.4 / 32 / 36 MHz crystal and a
//! 0 / 3 / 4 MHz IF in the descriptor. The two drivers differ in the tuner
//! code, in whether Init and Close put the tuner to standby and Open wakes it
//! (the DT-H30/PCIEM-TS driver does, the series driver does not) and in a few
//! demodulator details (see [`demod`]).

pub mod demod;

use crate::device::Device;
use crate::error::{Error, Result};
use crate::fc5500::Fc5500;
use crate::kit::{SignalStatus, TuneRequest, VendorDriver};
use crate::mtv808::Mtv808;
use crate::protocol::UsbControl;
use crate::tc90517::{Tc90517, TerVariant};
use crate::tlv::TunerDesc;
use crate::tuner_bus::{TunerBus, TunerPath};

/// The kit names.
pub const KIT_NAMES: [&str; 2] = ["TC90527_FC5500", "TC90527_MTV808"];

/// The tuner of the kit.
#[derive(Debug, Clone, Copy)]
pub enum Tc90527Tuner {
    Fc5500(Fc5500),
    Mtv808(Mtv808),
}

/// A `TC90527_*` kit.
pub struct Tc90527Kit<T: UsbControl> {
    pub dev: Device<T>,
    pub desc: TunerDesc,
    pub driver: VendorDriver,
    pub demod: Tc90517,
    pub tuner: Tc90527Tuner,
    /// Whether a configuration has passed its crystal and IF checks (the
    /// status preamble is written only then, as in the vendor drivers).
    configured: bool,
    open_count: u32,
    last_request: Option<TuneRequest>,
}

impl<T: UsbControl> Tc90527Kit<T> {
    /// Whether `desc` names one of these kits.
    pub fn supports(desc: &TunerDesc) -> bool {
        KIT_NAMES.contains(&desc.kit_name.as_str())
    }

    /// The kit for `desc` on the model with USB product ID `pid`:
    /// `TC90527_FC5500` follows the model's vendor driver
    /// ([`VendorDriver::for_product`]), `TC90527_MTV808` the DT-H30/PCIEM-TS
    /// driver, the only one that knows it.
    pub fn new(dev: Device<T>, desc: TunerDesc, pid: u16) -> Result<Self> {
        if !Self::supports(&desc) {
            return Err(Error::Unsupported(format!(
                "terrestrial tuner kit {} hw 0x{:x} is not a TC90527 kit",
                desc.kit_name, desc.hw_version
            )));
        }
        let path = if desc.tuner_direct_i2c() {
            TunerPath::Direct
        } else {
            TunerPath::ViaDemod { demod_addr: desc.demod_i2c_addr }
        };
        let bus = TunerBus { tuner_addr: desc.tuner_i2c_addr, path };
        let (driver, tuner) = if desc.kit_name == "TC90527_FC5500" {
            let driver = VendorDriver::for_product(pid);
            (driver, Tc90527Tuner::Fc5500(Fc5500::new(bus, driver)))
        } else {
            (VendorDriver::Dth30, Tc90527Tuner::Mtv808(Mtv808::new(bus, VendorDriver::Dth30)))
        };
        Ok(Tc90527Kit {
            demod: Tc90517::with_variant(desc.demod_i2c_addr, TerVariant::Tc90527(driver)),
            tuner,
            dev,
            desc,
            driver,
            configured: false,
            open_count: 0,
            last_request: None,
        })
    }

    /// Demodulator configuration, then tuner init.
    fn configure(&mut self) -> Result<()> {
        let r = self.demod.configure(&mut self.dev, &self.desc);
        // The vendor drivers keep the descriptor once the checks are passed,
        // whether or not the batch went through.
        if !matches!(r, Err(Error::Unsupported(_))) {
            self.configured = true;
        }
        let r = r.and_then(|()| match &mut self.tuner {
            Tc90527Tuner::Fc5500(t) => t.init(&mut self.dev, &self.desc),
            Tc90527Tuner::Mtv808(t) => t.init(&mut self.dev, &self.desc),
        });
        self.last_request = None;
        r
    }

    fn demod_wake(&mut self) -> Result<()> {
        demod::wake(&mut self.dev, self.demod.addr, self.driver, &self.desc.kit_name)
    }

    fn demod_standby(&mut self) -> Result<()> {
        demod::standby(&mut self.dev, self.demod.addr, self.driver, &self.desc.kit_name)
    }

    /// The tuner's standby: the DT-H30/PCIEM-TS driver's only (the series
    /// driver leaves the tuner on).
    fn tuner_standby(&mut self) -> Result<()> {
        if self.driver == VendorDriver::Series {
            return Ok(());
        }
        match &self.tuner {
            Tc90527Tuner::Fc5500(t) => t.standby(&mut self.dev),
            Tc90527Tuner::Mtv808(t) => t.standby(&mut self.dev),
        }
    }

    /// The tuner's wake: the DT-H30/PCIEM-TS driver's only.
    fn tuner_wake(&mut self) -> Result<()> {
        if self.driver == VendorDriver::Series {
            return Ok(());
        }
        match &self.tuner {
            Tc90527Tuner::Fc5500(t) => t.wake(&mut self.dev),
            Tc90527Tuner::Mtv808(t) => t.wake(&mut self.dev),
        }
    }

    /// Init: power-up, demodulator configuration, tuner init, tuner standby
    /// (DT-H30/PCIEM-TS driver) and demodulator standby (brought up again by
    /// [`open`](Self::open)). With flags bit5 set only the power-down, as in
    /// the vendor drivers.
    pub fn init(&mut self) -> Result<()> {
        if self.desc.skip_init_config() {
            return self.dev.power_down(&self.desc);
        }
        self.dev.power_up(&self.desc)?;
        self.configure()?;
        self.tuner_standby()?;
        self.demod_standby()
    }

    /// Open (reference counted): demodulator wake, tuner wake
    /// (DT-H30/PCIEM-TS driver) — with flags bit5 set: power-up,
    /// configuration and tuner init —, then GPIO mask C <- open value, which
    /// the vendor drivers write whether or not the steps before succeeded.
    pub fn open(&mut self) -> Result<()> {
        self.open_count += 1;
        if self.open_count != 1 {
            return Ok(());
        }
        let r = if self.desc.skip_init_config() {
            self.dev.power_up(&self.desc).and_then(|()| self.configure())
        } else {
            self.demod_wake().and_then(|()| self.tuner_wake())
        };
        let g = self.dev.gpio_write(self.desc.gpio_mask_c, self.desc.gpio_c_open);
        let r = r.and(g);
        if r.is_err() {
            self.open_count = 0;
        }
        r
    }

    /// Close: GPIO mask C <- close value, then tuner standby
    /// (DT-H30/PCIEM-TS driver) and demodulator standby (with flags bit5 set:
    /// power-down).
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
            return self.dev.power_down(&self.desc);
        }
        self.tuner_standby()?;
        self.demod_standby()
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
            Tc90527Tuner::Fc5500(t) => t.tune(&mut self.dev, req.frequency_u32(), req.bandwidth_hz)?,
            Tc90527Tuner::Mtv808(t) => t.tune(&mut self.dev, req.frequency_u32())?,
        }
        self.demod.post_tune(&mut self.dev)?;
        self.last_request = Some(*req);
        Ok(())
    }

    /// Signal status: the demodulator's (after its preamble, once configured),
    /// then the tuner's (which replaces the strength).
    pub fn signal_status(&mut self, mask: u32) -> Result<SignalStatus> {
        if self.configured {
            demod::status_preamble(&mut self.dev, self.demod.addr)?;
        }
        let mut st = SignalStatus { mask, ..Default::default() };
        self.demod.status(&mut self.dev, &mut st)?;
        match &mut self.tuner {
            Tc90527Tuner::Fc5500(t) => t.status(&mut self.dev, &mut st)?,
            Tc90527Tuner::Mtv808(t) => t.status(&mut self.dev, &mut st)?,
        }
        Ok(st)
    }

    pub fn last_request(&self) -> Option<&TuneRequest> {
        self.last_request.as_ref()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kit::PID_DTH30_PCIEM_TS;
    use crate::regmock::{Op, RegMock, Write};

    fn desc(name: &str, flags: u32) -> TunerDesc {
        TunerDesc {
            kit_name: name.into(),
            hw_version: 0x104,
            flags,
            demod_i2c_addr: 0x10,
            tuner_i2c_addr: 0x61,
            tuner_xtal_hz: 16_000_000,
            demod_xtal_hz: 16_000_000,
            if_freq_hz: 4_000_000,
            gpio_mask_c: 0x10,
            gpio_c_open: 0x10,
            gpio_c_close: 0,
            ..Default::default()
        }
    }

    #[test]
    fn kit_by_name_alone() {
        for (name, ok) in [
            ("TC90527_FC5500", true),
            ("TC90527_MTV808", true),
            ("TC90527_MTV600", false),
            ("TC90522_FC5500", false),
            ("TC90527", false),
        ] {
            for hw in [0x101, 0x102, 0x104, 0x402] {
                let d = TunerDesc { hw_version: hw, ..desc(name, 0) };
                assert_eq!(Tc90527Kit::<RegMock>::supports(&d), ok, "{name} hw 0x{hw:x}");
                assert_eq!(Tc90527Kit::new(Device::new(RegMock::default()), d, 0x0202).is_ok(), ok, "{name}");
            }
        }
    }

    #[test]
    fn driver_by_kit_and_model() {
        for (name, pid, driver) in [
            ("TC90527_FC5500", 0x0202, VendorDriver::Series),
            ("TC90527_FC5500", PID_DTH30_PCIEM_TS, VendorDriver::Dth30),
            ("TC90527_MTV808", 0x0202, VendorDriver::Dth30),
            ("TC90527_MTV808", PID_DTH30_PCIEM_TS, VendorDriver::Dth30),
        ] {
            let k = Tc90527Kit::new(Device::new(RegMock::default()), desc(name, 0), pid).unwrap();
            assert_eq!(k.driver, driver, "{name} {pid:#06x}");
            assert_eq!(k.demod.variant, TerVariant::Tc90527(driver));
            assert_eq!(matches!(k.tuner, Tc90527Tuner::Fc5500(_)), name.ends_with("FC5500"));
        }
    }

    /// The demodulator standby after the configuration, from a zeroed register file.
    fn demod_standby() -> Vec<Write> {
        vec![
            Write::Batch(vec![(0x10, 0x1D, vec![0xA9]), (0x10, 0x1E, vec![0x95])]),
            Write::Reg(0x10, 0x03, vec![0x10]),
            Write::Reg(0x10, 0x04, vec![0x2A]),
            Write::Reg(0x10, 0x1F, vec![0x00]),
        ]
    }

    #[test]
    fn series_fc5500_kit_leaves_the_tuner_on() {
        let mut k = Tc90527Kit::new(Device::new(RegMock::default()), desc("TC90527_FC5500", 0), 0x0202).unwrap();
        k.init().unwrap();
        let w = k.dev.transport().take_writes();
        std::assert_matches!(&w[0], Write::Batch(b) if b.len() == 9 && b[4] == (0x10, 0x11, vec![0x2D]), "{:?}", w[0]);
        // the FC5500 init (register 0x01 read, then the table) through the repeater
        assert_eq!(&w[1..3], &[Write::Raw(0x10, vec![0xFE, 0xC2, 0x01]), Write::Raw(0x10, vec![0xFE, 0xC3])]);
        assert_eq!(w[3], Write::Raw(0x10, vec![0xFE, 0xC2, 0x00, 0x00]));
        let n = w.len();
        assert_eq!(&w[n - 4..], &demod_standby()[..], "no tuner standby before the demodulator's");
        assert_eq!(n, 1 + 3 + 84 + 4, "{w:?}");

        k.open().unwrap();
        assert_eq!(
            k.dev.transport().take_writes(),
            vec![
                Write::Reg(0x10, 0x1F, vec![0x0C]),
                Write::Reg(0x10, 0x04, vec![0x02]),
                Write::Reg(0x10, 0x03, vec![0])
            ]
        );
        k.tune(&TuneRequest::isdb_t(557_142_857, 6_000_000)).unwrap();
        let w = k.dev.transport().take_writes();
        assert_eq!(
            &w[..2],
            &[
                Write::Batch(vec![(0x10, 0x1D, vec![0xA9]), (0x10, 0x1E, vec![0x95])]),
                Write::Batch(vec![(0x10, 0x25, vec![0x00]), (0x10, 0x23, vec![0x01])]),
            ]
        );
        let n = w.len();
        assert_eq!(
            &w[n - 3..],
            &[
                Write::Reg(0x10, 0x01, vec![0x40]),
                Write::Batch(vec![(0x10, 0x23, vec![0x00]), (0x10, 0x76, vec![0x8F])]),
                Write::Batch(vec![(0x10, 0x1D, vec![0x01]), (0x10, 0x1E, vec![0x15])]),
            ]
        );
        assert!(w[2..n - 3].iter().all(|w| matches!(w, Write::Raw(0x10, d) if d[0] == 0xFE)), "{w:?}");
        // the series PLL writes the 6 MHz bandwidth into register 0x32
        assert!(w.contains(&Write::Raw(0x10, vec![0xFE, 0xC2, 0x32, 0x54])), "{w:?}");

        k.dev.transport().take();
        let st = k.signal_status(crate::kit::field::ALL_BASIC).unwrap();
        assert_ne!(st.mask & crate::kit::field::STRENGTH, 0);
        let ops = k.dev.transport().take();
        assert_eq!(&ops[..3], &[Op::R(0x10, 0xB3), Op::R(0x10, 0xB4), Op::W(0x10, 0xFA, 0x04)]);
        assert_eq!(ops[3], Op::R(0x10, 0xB0), "then the TC90517 status");

        k.dev.transport().take_writes();
        k.close().unwrap();
        let ops = k.dev.transport().take();
        assert_eq!(ops[0], Op::Other, "GPIO first");
        let mut w = vec![Write::Batch(vec![(0x10, 0x1D, vec![0xA9]), (0x10, 0x1E, vec![0x95])])];
        w.extend([
            Write::Reg(0x10, 0x03, vec![0x10]),
            Write::Reg(0x10, 0x04, vec![0x2A]),
            Write::Reg(0x10, 0x1F, vec![0x00]),
        ]);
        assert_eq!(k.dev.transport().take_writes(), w);
    }

    #[test]
    fn dth30_mtv808_kit_puts_the_tuner_to_standby() {
        let mut k = Tc90527Kit::new(Device::new(RegMock::default()), desc("TC90527_MTV808", 0), 0x0202).unwrap();
        k.init().unwrap();
        let w = k.dev.transport().take_writes();
        let n = w.len();
        assert_eq!(&w[n - 4..], &demod_standby()[..]);
        assert_eq!(
            &w[n - 7..n - 4],
            &[
                Write::Raw(0x10, vec![0xFE, 0xC2, 0x33]),
                Write::Raw(0x10, vec![0xFE, 0xC3]),
                Write::Raw(0x10, vec![0xFE, 0xC2, 0x33, 0x40]),
            ],
            "tuner standby first"
        );
        k.open().unwrap();
        let w = k.dev.transport().take_writes();
        // 0x1F is written back as read for this kit
        assert_eq!(
            &w[..3],
            &[Write::Reg(0x10, 0x1F, vec![0x00]), Write::Reg(0x10, 0x04, vec![0x02]), Write::Reg(0x10, 0x03, vec![0])]
        );
        assert_eq!(w.len(), 3 + 3, "tuner wake: {w:?}");
        k.close().unwrap();
        let w = k.dev.transport().take_writes();
        assert_eq!(w.len(), 3 + 4, "{w:?}");
    }

    #[test]
    fn flags_bit5_configures_at_open_and_powers_down_at_close() {
        let mut k = Tc90527Kit::new(Device::new(RegMock::default()), desc("TC90527_FC5500", 0x20), 0x0218).unwrap();
        k.init().unwrap();
        assert_eq!(k.dev.transport().take_writes(), vec![], "Init only powers down");
        // no configuration yet: no status preamble
        k.dev.transport().take();
        k.signal_status(crate::kit::field::LOCK).unwrap();
        assert_eq!(k.dev.transport().take()[0], Op::R(0x10, 0xB0));
        k.dev.transport().take_writes();
        k.open().unwrap();
        let w = k.dev.transport().take_writes();
        std::assert_matches!(&w[0], Write::Batch(b) if b.len() == 9, "{:?}", w[0]);
        k.close().unwrap();
        assert_eq!(k.dev.transport().take_writes(), vec![], "Close only powers down");
    }

    #[test]
    fn unsupported_crystal_or_if_fails_the_init() {
        for (xtal, if_hz, what) in
            [(20_000_000u32, 4_000_000i32, "crystal 20000000 Hz"), (16_000_000, 5_000_000, "IF 5000000 Hz")]
        {
            let d = TunerDesc { demod_xtal_hz: xtal, if_freq_hz: if_hz, ..desc("TC90527_FC5500", 0) };
            let mut k = Tc90527Kit::new(Device::new(RegMock::default()), d, 0x0202).unwrap();
            let e = k.init().unwrap_err();
            std::assert_matches!(e, Error::Unsupported(_) if e.to_string().contains(what), "{e}");
            assert!(!k.configured);
        }
    }
}
