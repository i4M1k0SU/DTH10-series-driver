//! The `BQ200X_*` tuner kits: BQ200X ISDB-T demodulator ([`demod`]) with a
//! MaxLinear MxL135RF ([`crate::mxl13xrf`]) or an Elonics E4005 ([`e4005`]) tuner.
//!
//! The vendor driver builds one of five kit variants from the kit name, the
//! descriptor's model string, its flags bit7 and a "diversity" request that
//! comes from the filter layer (see [`Bq200xKit::with_diversity`]):
//!
//! | kit name | model | diversity requested, flags bit7 set | otherwise |
//! |---|---|---|---|
//! | `BQ200X_MxL135RF` | `h55`, `f200` | [`Variant::MxlDiversity`] | [`Variant::MxlSwitched`] |
//! | `BQ200X_MxL135RF` | `f201` | [`Variant::MxlDiversity`] (bit7 alone suffices) | refused (bit7 clear) |
//! | `BQ200X_MxL135RF` | any other | [`Variant::MxlPlain`] | [`Variant::MxlPlain`] |
//! | `BQ200X_E4005` | `f200` | [`Variant::E4005Diversity`] | [`Variant::E4005Single`] |
//! | `BQ200X_E4005` | any other | refused | refused |
//!
//! In a diversity variant the kit drives two demodulators (master = demodulator
//! address, slave = TLV tag 'F') and two tuners (tuner address and TLV tag 'G')
//! on the same channel; the status comes from the master.
//!
//! The tuners sit behind the master's I2C gate (register 0x02 bit1) unless the
//! descriptor's flags bit3 puts them directly on the bridge bus.
//!
//! On model `f200` every tune starts by switching GPIO 0xC0 by frequency band
//! (see [`f200_band_bits`]).

pub mod demod;
pub mod e4005;

use core::time::Duration;
use std::thread::sleep;

use crate::device::{Device, FLAG_TUNER1};
use crate::error::{Error, Result};
use crate::kit::{SignalStatus, TuneRequest, field};
use crate::mxl13xrf::{self, Mxl13xRf, MxlType};
use crate::protocol::{self, UsbControl, cmd};
use crate::tlv::TunerDesc;
use crate::tuner_bus::{TunerBus, TunerPath};

pub use demod::{Bq200x, TunerGate, lock};
pub use e4005::E4005;

/// Which kit object the vendor driver would build.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// BQ200X + MxL135RF, models `h55` / `f200`: standard power sequence, GPIO
    /// mask C written at Open / Close.
    MxlSwitched,
    /// BQ200X + MxL135RF, other models: masks A|B raised together, no mask C.
    MxlPlain,
    /// Two BQ200X (master / slave) + two MxL135RF.
    MxlDiversity,
    /// BQ200X + E4005 (model `f200`).
    E4005Single,
    /// Two BQ200X + two E4005 (model `f200`).
    E4005Diversity,
}

impl Variant {
    pub fn is_diversity(self) -> bool {
        matches!(self, Variant::MxlDiversity | Variant::E4005Diversity)
    }
}

fn refuse(desc: &TunerDesc, why: &str) -> Error {
    Error::Unsupported(format!(
        "terrestrial tuner kit {} (model {}, flags 0x{:x}): {why}",
        desc.kit_name, desc.model, desc.flags
    ))
}

/// The variant for a descriptor, as the vendor's kit factory chooses it.
/// `diversity` is the filter layer's request (false at device start).
pub fn select_variant(desc: &TunerDesc, diversity: bool) -> Result<Variant> {
    let bit7 = desc.flags & 0x80 != 0;
    match desc.kit_name.as_str() {
        "BQ200X_MxL135RF" => {
            if desc.model_is("h55") || desc.model_is("f200") {
                Ok(if diversity && bit7 { Variant::MxlDiversity } else { Variant::MxlSwitched })
            } else if desc.model_is("f201") {
                if bit7 { Ok(Variant::MxlDiversity) } else { Err(refuse(desc, "model f201 needs flags bit7")) }
            } else {
                Ok(Variant::MxlPlain)
            }
        }
        "BQ200X_E4005" => {
            if !desc.model_is("f200") {
                Err(refuse(desc, "the E4005 kit exists only for model f200"))
            } else if diversity && bit7 {
                Ok(Variant::E4005Diversity)
            } else {
                Ok(Variant::E4005Single)
            }
        }
        _ => Err(refuse(desc, "not a BQ200X kit")),
    }
}

/// GPIO 0xC0 value of the model `f200` tune step: below 540 MHz 0x00,
/// 540–580 MHz 0x80, 580–680 MHz 0x40, from 680 MHz 0xC0.
pub fn f200_band_bits(freq_hz: u64) -> u32 {
    if freq_hz < 540_000_000 {
        0x00
    } else if freq_hz < 580_000_000 {
        0x80
    } else if freq_hz < 680_000_000 {
        0x40
    } else {
        0xC0
    }
}

#[derive(Debug, Clone, Copy)]
enum KitTuner {
    Mxl(Mxl13xRf),
    E4005(E4005),
}

pub struct Bq200xKit<T: UsbControl> {
    pub dev: Device<T>,
    pub desc: TunerDesc,
    pub variant: Variant,
    pub demod: Bq200x,
    pub gate: TunerGate,
    /// Primary tuner (tuner address) first, then the diversity tuner (tag 'G').
    tuners: Vec<KitTuner>,
    open_count: u32,
    last_request: Option<TuneRequest>,
}

impl<T: UsbControl> Bq200xKit<T> {
    /// The kit for `desc` outside diversity mode (what the driver builds at device start).
    pub fn new(dev: Device<T>, desc: TunerDesc) -> Result<Self> {
        Self::with_diversity(dev, desc, false)
    }

    /// The kit with the filter layer's diversity request. The vendor driver
    /// rebuilds both filters' kits with this request set when an application
    /// sets the vendor-specific digital-demodulator property 0xC0BDA001 to a
    /// non-zero value (only on devices with two filters whose first descriptor
    /// has flags bit7, and only while both are stopped); a filter whose own
    /// descriptor lacks bit7 then gets no kit at all.
    pub fn with_diversity(dev: Device<T>, desc: TunerDesc, diversity: bool) -> Result<Self> {
        let variant = select_variant(&desc, diversity)?;
        let gate = TunerGate { demod: if desc.tuner_direct_i2c() { None } else { Some(desc.demod_i2c_addr) } };
        let slave = if variant.is_diversity() { Some(desc.byte_46) } else { None };
        let mut addrs = vec![desc.tuner_i2c_addr];
        if variant.is_diversity() {
            addrs.push(desc.byte_47);
        }
        let tuners = addrs
            .into_iter()
            .map(|a| match variant {
                Variant::E4005Single | Variant::E4005Diversity => KitTuner::E4005(E4005::new(a)),
                _ => {
                    KitTuner::Mxl(Mxl13xRf::new(TunerBus { tuner_addr: a, path: TunerPath::Direct }, MxlType::MxL135RF))
                }
            })
            .collect();
        Ok(Bq200xKit {
            demod: Bq200x::new(desc.demod_i2c_addr, slave),
            dev,
            desc,
            variant,
            gate,
            tuners,
            open_count: 0,
            last_request: None,
        })
    }

    /// Whether the diversity kits act on GPIO / power (only the kit of tuner id 0 does).
    fn owns_gpio(&self) -> bool {
        !self.variant.is_diversity() || self.desc.id == 0
    }

    fn power_up(&self) -> Result<()> {
        let d = &self.desc;
        match self.variant {
            Variant::MxlSwitched | Variant::E4005Single => self.dev.power_up(d),
            Variant::MxlPlain => {
                let ab = d.gpio_mask_a | d.gpio_mask_b;
                let r = self.dev.gpio_enable(ab).and_then(|_| self.dev.gpio_set_bits(ab));
                sleep(Duration::from_millis(10));
                r
            }
            Variant::MxlDiversity | Variant::E4005Diversity => {
                if !self.owns_gpio() {
                    return Ok(());
                }
                self.dev.power_up(d)?;
                self.dev.power_up(d)
            }
        }
    }

    fn power_down(&self) -> Result<()> {
        let d = &self.desc;
        match self.variant {
            Variant::MxlSwitched | Variant::E4005Single => self.dev.power_down(d),
            Variant::MxlPlain => {
                let ab = d.gpio_mask_a | d.gpio_mask_b;
                self.dev.gpio_enable(ab)?;
                self.dev.gpio_clear_bits(ab)
            }
            Variant::MxlDiversity | Variant::E4005Diversity => {
                if !self.owns_gpio() {
                    return Ok(());
                }
                self.dev.power_down(d)?;
                self.dev.power_down(d)
            }
        }
    }

    fn tuner_init(&mut self, i: usize) -> Result<()> {
        let gate = self.gate;
        match self.tuners[i] {
            KitTuner::Mxl(t) => {
                // The table is checked before the gate is opened.
                mxl13xrf::init_registers(&self.desc)?;
                let desc = &self.desc;
                gate.with(&mut self.dev, |d| t.init(d, desc))
            }
            KitTuner::E4005(mut t) => {
                let r = t.init(&mut self.dev, &self.desc, &gate);
                self.tuners[i] = KitTuner::E4005(t);
                r
            }
        }
    }

    fn tuner_standby(&mut self, i: usize) -> Result<()> {
        let gate = self.gate;
        match self.tuners[i] {
            KitTuner::Mxl(t) => gate.with(&mut self.dev, |d| t.standby(d)),
            KitTuner::E4005(t) => t.standby(&mut self.dev, &gate),
        }
    }

    fn tuner_wake(&mut self, i: usize) -> Result<()> {
        let gate = self.gate;
        match self.tuners[i] {
            KitTuner::Mxl(t) => gate.with(&mut self.dev, |d| t.wake(d)),
            KitTuner::E4005(t) => t.wake(&mut self.dev, &gate),
        }
    }

    fn tuner_tune(&mut self, i: usize, req: &TuneRequest) -> Result<()> {
        let gate = self.gate;
        let (f, bw) = (req.frequency_u32(), req.bandwidth_hz);
        match self.tuners[i] {
            KitTuner::Mxl(t) => gate.with(&mut self.dev, |d| t.set_frequency(d, f, bw)),
            KitTuner::E4005(t) => t.set_frequency(&mut self.dev, &gate, f, bw),
        }
    }

    /// Demodulator configuration and tuner initialisation (the diversity tuner first).
    fn configure_chips(&mut self) -> Result<()> {
        let r = self.demod.configure(&mut self.dev, &self.desc).and_then(|_| {
            for i in (0..self.tuners.len()).rev() {
                self.tuner_init(i)?;
            }
            Ok(())
        });
        self.last_request = None;
        r
    }

    /// Init: power-up, demodulator configuration, tuner initialisation, tuners to
    /// standby (the demodulator keeps running). With flags bit5 only the power-down.
    pub fn init(&mut self) -> Result<()> {
        if self.desc.skip_init_config() {
            // The original ignores the power-down result here.
            let _ = self.power_down();
            return Ok(());
        }
        // The original ignores the power-up result here.
        let _ = self.power_up();
        self.configure_chips()?;
        for i in 0..self.tuners.len() {
            self.tuner_standby(i)?;
        }
        Ok(())
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
        let r = if self.desc.skip_init_config() {
            self.power_up().and_then(|_| self.configure_chips())
        } else {
            (0..self.tuners.len()).rev().try_for_each(|i| self.tuner_wake(i))
        };
        // Written whatever the result, as in the original.
        let d = &self.desc;
        match self.variant {
            Variant::MxlSwitched | Variant::E4005Single => {
                let _ = self.dev.gpio_write(d.gpio_mask_c, d.gpio_c_open);
            }
            Variant::MxlDiversity | Variant::E4005Diversity if self.owns_gpio() => {
                let _ = self.dev.gpio_write(d.gpio_mask_c, d.val_67);
            }
            _ => {}
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
        let write_c = match self.variant {
            Variant::MxlPlain => false,
            Variant::MxlSwitched | Variant::E4005Single => true,
            Variant::MxlDiversity | Variant::E4005Diversity => self.owns_gpio(),
        };
        if write_c {
            let _ = self.dev.gpio_write(self.desc.gpio_mask_c, self.desc.gpio_c_close);
        }
        if self.desc.skip_init_config() {
            let _ = self.power_down();
            return Ok(());
        }
        for i in 0..self.tuners.len() {
            self.tuner_standby(i)?;
        }
        Ok(())
    }

    pub fn is_open(&self) -> bool {
        self.open_count > 0
    }

    /// Model `f200`: GPIO 0xC0 by band. The command goes out with the tuner-1
    /// flag when the descriptor id is 0 (the driver wraps its command channel in
    /// the second-tuner proxy then), with this tuner's own flags otherwise.
    fn f200_band_switch(&self, req: &TuneRequest) -> Result<()> {
        if !self.desc.model_is("f200") {
            return Ok(());
        }
        let mut p = [0u8; 12];
        p[4..8].copy_from_slice(&0xC0u32.to_le_bytes());
        p[8..12].copy_from_slice(&f200_band_bits(req.frequency_hz).to_le_bytes());
        let flags = if self.desc.id == 0 { FLAG_TUNER1 } else { self.dev.flags(0) };
        protocol::send(self.dev.transport(), flags, cmd::SYS, cmd::SYS_GPIO_SET, &p, None).map(|_| ())
    }

    /// Tune: the `f200` band switch, the tuners (the diversity tuner first), then
    /// the demodulator's acquisition restart.
    pub fn tune(&mut self, req: &TuneRequest) -> Result<()> {
        if !self.is_open() {
            return Err(Error::Protocol("kit is not open"));
        }
        self.f200_band_switch(req)?;
        for i in (0..self.tuners.len()).rev() {
            self.tuner_tune(i, req)?;
        }
        self.demod.restart(&mut self.dev)?;
        self.last_request = Some(*req);
        Ok(())
    }

    /// Signal status (no caching here): the demodulator, then, in the single
    /// MxL135RF variants, the tuner's reading replaces `strength`.
    pub fn signal_status(&mut self, mask: u32) -> Result<SignalStatus> {
        let mut st = SignalStatus { mask, ..Default::default() };
        self.demod.status(&mut self.dev, &mut st)?;
        if let (false, KitTuner::Mxl(t)) = (self.variant.is_diversity(), self.tuners[0]) {
            st.strength = self.gate.with(&mut self.dev, |d| t.signal(d))?;
            st.mask |= field::STRENGTH;
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
    use std::cell::RefCell;
    use std::collections::HashMap;

    /// One logged operation, decoded from the control transfers.
    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Op {
        /// Register write: address, register, value(s).
        W(u8, u8, Vec<u8>),
        /// Register read: address, register.
        R(u8, u8),
        /// Raw I2C write: address, bytes.
        Raw(u8, Vec<u8>),
        /// Raw I2C read: address, length.
        RawRead(u8, usize),
        /// GPIO: bRequest, wIndex (0x1004 / 0x1006), mask, value.
        Gpio(u8, u16, u32, u32),
    }

    /// Bridge mock with a register file per I2C address.
    #[derive(Default)]
    struct Mock {
        ops: RefCell<Vec<Op>>,
        regs: RefCell<HashMap<(u8, u8), u8>>,
    }

    impl Mock {
        fn with_regs(init: &[(u8, u8, u8)]) -> Self {
            let m = Mock::default();
            for &(a, r, v) in init {
                m.regs.borrow_mut().insert((a, r), v);
            }
            m
        }
    }

    impl UsbControl for Mock {
        fn control_out(&self, rq: u8, v: u16, i: u16, d: &[u8], _: Duration) -> Result<usize> {
            let addr = (v & 0xFF) as u8;
            let op = match v & 0xFF00 {
                0x2D00 => {
                    for (k, &b) in d.iter().enumerate() {
                        self.regs.borrow_mut().insert((addr, i as u8 + k as u8), b);
                    }
                    Op::W(addr, i as u8, d.to_vec())
                }
                0x2C00 => Op::Raw(addr, d.to_vec()),
                _ => {
                    let mask = u32::from_le_bytes([d[4], d[5], d[6], d[7]]);
                    let val = u32::from_le_bytes([d[8], d[9], d[10], d[11]]);
                    Op::Gpio(rq, i, mask, val)
                }
            };
            self.ops.borrow_mut().push(op);
            Ok(d.len())
        }

        fn control_in(&self, _: u8, v: u16, i: u16, b: &mut [u8], _: Duration) -> Result<usize> {
            let addr = (v & 0xFF) as u8;
            match v & 0xFF00 {
                0x2D00 => {
                    for (k, x) in b.iter_mut().enumerate() {
                        *x = *self.regs.borrow().get(&(addr, i as u8 + k as u8)).unwrap_or(&0);
                    }
                    self.ops.borrow_mut().push(Op::R(addr, i as u8));
                }
                _ => {
                    b.fill(0);
                    self.ops.borrow_mut().push(Op::RawRead(addr, b.len()));
                }
            }
            Ok(b.len())
        }
    }

    const DEMOD: u8 = 0x18;
    const SLAVE: u8 = 0x19;
    const TUNER: u8 = 0x60;
    const TUNER2: u8 = 0x61;

    fn desc(kit: &str, model: &str, flags: u32) -> TunerDesc {
        TunerDesc {
            present: 0x1 | 0x2000 | 0x4000,
            kit_name: kit.into(),
            id: 0,
            flags,
            hw_version: 0x102,
            demod_i2c_addr: DEMOD,
            byte_46: SLAVE,
            tuner_i2c_addr: TUNER,
            byte_47: TUNER2,
            tuner_xtal_hz: 16_000_000,
            if_freq_hz: 4_000_000,
            demod_xtal_hz: 16_000_000,
            stream_param: 0x10,
            gpio_mask_a: 0x01,
            gpio_mask_b: 0x02,
            gpio_mask_c: 0x0C,
            gpio_c_open: 0x04,
            gpio_c_close: 0x08,
            val_67: 0x0C,
            model: model.into(),
            ..Default::default()
        }
    }

    /// The acquisition flag (0x3A bit6) is preset so that the 30-poll wait ends at once.
    fn kit(d: TunerDesc, diversity: bool) -> Bq200xKit<Mock> {
        let m = Mock::with_regs(&[(DEMOD, 0x3A, 0x40), (SLAVE, 0x3A, 0x40)]);
        Bq200xKit::with_diversity(Device::new(m), d, diversity).unwrap()
    }

    fn take_ops(k: &Bq200xKit<Mock>) -> Vec<Op> {
        std::mem::take(&mut *k.dev.transport().ops.borrow_mut())
    }

    fn w(a: u8, r: u8, v: u8) -> Op {
        Op::W(a, r, vec![v])
    }

    /// Expected configuration of one chip with an all-zero register file except
    /// 0x3A (16 MHz crystal, IF 4 MHz, role bits `ra` / `rb`).
    fn chip_config_ops(a: u8, ra: u8, rb: u8) -> Vec<Op> {
        use Op::R;
        let mut v = vec![
            R(a, 0x03),
            w(a, 0x03, 0x20),
            R(a, 0x02),
            w(a, 0x02, 0x04),
            R(a, 0x56),
            w(a, 0x56, 0x00),
            R(a, 0x02),
            w(a, 0x02, 0x0C),
            w(a, 0xB6, 0xC0),
            w(a, 0x2F, 0x04),
            w(a, 0x90, 0xE0),
            w(a, 0x61, 0xA9),
            R(a, 0x21),
            w(a, 0x62, 0x35),
            w(a, 0x63, 0x55),
            w(a, 0x79, 0x01),
            w(a, 0x7A, 0x06),
            w(a, 0x7B, 0x24),
            w(a, 0x7C, 0xDD),
            R(a, 0x64),
            w(a, 0x64, 0x01),
            w(a, 0x65, 0xFB),
            w(a, 0x66, 0xEF),
            w(a, 0x6E, 0x01),
            w(a, 0x6F, 0xFF),
            R(a, 0x6C),
            w(a, 0x6C, 0x00),
            w(a, 0x6D, 0x28),
            R(a, 0x56),
            w(a, 0x56, 0x80),
            R(a, 0x75),
            w(a, 0x75, 0x10),
            w(a, 0xF3, 0x80),
            R(a, 0x04),
            w(a, 0x04, 0x00),
            w(a, 0x1E, 0xCF),
            w(a, 0x1F, 0x20),
            w(a, 0x5A, 0x08),
            w(a, 0xA2, 0x40),
            w(a, 0xA3, 0x32),
            w(a, 0x97, 0x00),
            R(a, 0xA5),
            w(a, 0xA5, 0x00),
            w(a, 0x9E, 0x00),
            w(a, 0x22, 0x0F),
            R(a, 0x00),
            w(a, 0x00, 0x00),
            R(a, 0x00),
            w(a, 0x00, ra),
            R(a, 0x00),
            w(a, 0x00, ra | (rb << 2)),
            R(a, 0x68),
            w(a, 0x68, 0x07),
            w(a, 0x69, 0xD0),
            w(a, 0x6A, 0x80),
            w(a, 0x6B, 0x00),
            R(a, 0x00),
            w(a, 0x00, (ra | (rb << 2)) & 0xCF),
            R(a, 0x64),
            w(a, 0x64, 0x01),
            w(a, 0x65, 0xFB),
            w(a, 0x66, 0xEF),
            R(a, 0x5E),
            w(a, 0x5E, 0x80),
            R(a, 0x71),
            w(a, 0x71, 0x10),
            w(a, 0x95, 0x03),
            R(a, 0x56),
            w(a, 0x56, 0x80),
            w(a, 0x85, 0x02),
            w(a, 0xBA, 0x12),
            w(a, 0xBB, 0x11),
            R(a, 0x01),
            w(a, 0x01, 0x1F),
            R(a, 0x6C),
            w(a, 0x6C, 0x0C),
            w(a, 0x88, 0x08),
            w(a, 0x89, 0x00),
            w(a, 0x8A, 0x00),
            w(a, 0x8B, 0x08),
            w(a, 0x8C, 0x00),
            w(a, 0x8D, 0x00),
            R(a, 0x1F),
            w(a, 0x1F, 0x20),
            R(a, 0x30),
            w(a, 0x30, 0x00),
            w(a, 0x29, 0x68),
            w(a, 0x2A, 0xAB),
            w(a, 0x2D, 0x20),
            w(a, 0x40, 0x2C),
            w(a, 0x75, 0xD4),
            w(a, 0xA7, 0x01),
            w(a, 0xB4, 0x50),
            w(a, 0xB5, 0x08),
            w(a, 0xF3, 0x80),
            R(a, 0x45),
            w(a, 0x45, 0x80),
        ];
        v.extend(sync_start_ops(a, 0x1F, 0x80));
        v
    }

    /// Acquisition restart with register 0x01 = `r01` and 0x45 = `r45` before it.
    fn sync_start_ops(a: u8, r01: u8, r45: u8) -> Vec<Op> {
        vec![
            Op::R(a, 0x01),
            w(a, 0x29, 0x68),
            w(a, 0x2A, 0xAB),
            w(a, 0x01, r01 & 0xEF),
            w(a, 0x01, r01 | 0x10),
            Op::R(a, 0x3A),
            Op::R(a, 0x45),
            w(a, 0x45, r45 | 0x20),
            w(a, 0x29, 0x66),
            w(a, 0x2A, 0x66),
        ]
    }

    fn gate_ops(open: bool, before: u8) -> Vec<Op> {
        vec![Op::R(DEMOD, 0x02), w(DEMOD, 0x02, if open { before & 0xFD } else { before | 0x02 })]
    }

    fn power_up_ops() -> Vec<Op> {
        vec![
            Op::Gpio(0x02, 0x1004, 0x0F, 0x0F),
            Op::Gpio(0x02, 0x1006, 0x03, 0x00),
            Op::Gpio(0x02, 0x1006, 0x01, 0x01),
            Op::Gpio(0x02, 0x1006, 0x02, 0x02),
        ]
    }

    fn mxl_tune_ops(addr: u8) -> Vec<Op> {
        let regs = mxl13xrf::rf_tune_registers(557_142_857, 6_000_000);
        let data: Vec<u8> = regs.iter().flat_map(|&(r, v)| [r, v]).collect();
        vec![
            Op::Raw(addr, data),
            Op::Raw(addr, vec![0xFB, 0xD8]),
            Op::RawRead(addr, 1),
            Op::Raw(addr, vec![0xFB, 0xD8]),
            Op::RawRead(addr, 1),
        ]
    }

    #[test]
    fn variant_selection() {
        let d = |kit: &str, model: &str, flags: u32| desc(kit, model, flags);
        assert_eq!(select_variant(&d("BQ200X_MxL135RF", "h55", 0), false).unwrap(), Variant::MxlSwitched);
        assert_eq!(select_variant(&d("BQ200X_MxL135RF", "f200", 0x80), false).unwrap(), Variant::MxlSwitched);
        assert_eq!(select_variant(&d("BQ200X_MxL135RF", "f200", 0x80), true).unwrap(), Variant::MxlDiversity);
        assert_eq!(select_variant(&d("BQ200X_MxL135RF", "h55", 0), true).unwrap(), Variant::MxlSwitched);
        assert_eq!(select_variant(&d("BQ200X_MxL135RF", "f201", 0x80), false).unwrap(), Variant::MxlDiversity);
        assert!(select_variant(&d("BQ200X_MxL135RF", "f201", 0), true).is_err());
        assert_eq!(select_variant(&d("BQ200X_MxL135RF", "h70", 0x80), true).unwrap(), Variant::MxlPlain);
        assert_eq!(select_variant(&d("BQ200X_E4005", "f200", 0x80), false).unwrap(), Variant::E4005Single);
        assert_eq!(select_variant(&d("BQ200X_E4005", "f200", 0x80), true).unwrap(), Variant::E4005Diversity);
        assert!(select_variant(&d("BQ200X_E4005", "h55", 0x80), false).is_err());
        assert!(select_variant(&d("TC90517_MxL135RF", "h70", 0), false).is_err());
    }

    #[test]
    fn f200_band_thresholds() {
        assert_eq!(f200_band_bits(539_999_999), 0x00);
        assert_eq!(f200_band_bits(540_000_000), 0x80);
        assert_eq!(f200_band_bits(579_999_999), 0x80);
        assert_eq!(f200_band_bits(580_000_000), 0x40);
        assert_eq!(f200_band_bits(679_999_999), 0x40);
        assert_eq!(f200_band_bits(680_000_000), 0xC0);
    }

    #[test]
    fn mxl_init_sequence() {
        let mut k = kit(desc("BQ200X_MxL135RF", "h55", 0), false);
        k.init().unwrap();
        let ops = take_ops(&k);
        let mut exp = power_up_ops();
        exp.extend(chip_config_ops(DEMOD, 0, 0));
        exp.extend([w(DEMOD, 0xF3, 0x90), Op::R(DEMOD, 0x1D), w(DEMOD, 0x1D, 0x00), w(DEMOD, 0x1D, 0x20)]);
        // MxL initialisation through the gate.
        exp.extend(gate_ops(true, 0x0C));
        let regs = mxl13xrf::init_registers(&k.desc).unwrap();
        exp.push(Op::Raw(TUNER, vec![0xFF]));
        exp.push(Op::Raw(TUNER, regs.iter().flat_map(|&(r, v)| [r, v]).collect()));
        exp.extend(gate_ops(false, 0x0C));
        // MxL standby through the gate.
        exp.extend(gate_ops(true, 0x0E));
        exp.push(Op::Raw(TUNER, vec![0x01, 0x00, 0x0F, 0x00]));
        exp.extend(gate_ops(false, 0x0C));
        assert_eq!(ops, exp);
    }

    #[test]
    fn mxl_open_tune_close_f200() {
        let mut k = kit(desc("BQ200X_MxL135RF", "f200", 0), false);
        k.init().unwrap();
        take_ops(&k);
        k.open().unwrap();
        let ops = take_ops(&k);
        let mut exp = gate_ops(true, 0x0E);
        exp.push(Op::Raw(TUNER, vec![0x01, 0x01]));
        exp.extend(gate_ops(false, 0x0C));
        exp.push(Op::Gpio(0x02, 0x1006, 0x0C, 0x04));
        assert_eq!(ops, exp);

        let req = TuneRequest::isdb_t(557_142_857, 6_000_000);
        k.tune(&req).unwrap();
        let ops = take_ops(&k);
        // f200: band switch on GPIO 0xC0 with the second-tuner flag (descriptor id 0).
        let mut exp = vec![Op::Gpio(0x82, 0x1006, 0xC0, 0x80)];
        exp.extend(gate_ops(true, 0x0E));
        exp.extend(mxl_tune_ops(TUNER));
        exp.extend(gate_ops(false, 0x0C));
        exp.extend(sync_start_ops(DEMOD, 0x1F, 0xA0));
        assert_eq!(ops, exp);
        assert_eq!(k.last_request(), Some(&req));

        // Below 540 MHz the band bits are 0; id 1 sends with the tuner's own flags.
        k.desc.id = 1;
        k.tune(&TuneRequest::isdb_t(533_142_857, 6_000_000)).unwrap();
        assert_eq!(take_ops(&k)[0], Op::Gpio(0x02, 0x1006, 0xC0, 0x00));
        k.tune(&TuneRequest::isdb_t(767_142_857, 6_000_000)).unwrap();
        assert_eq!(take_ops(&k)[0], Op::Gpio(0x02, 0x1006, 0xC0, 0xC0));
        k.tune(&TuneRequest::isdb_t(593_142_857, 6_000_000)).unwrap();
        assert_eq!(take_ops(&k)[0], Op::Gpio(0x02, 0x1006, 0xC0, 0x40));

        k.close().unwrap();
        let ops = take_ops(&k);
        let mut exp = vec![Op::Gpio(0x02, 0x1006, 0x0C, 0x08)];
        exp.extend(gate_ops(true, 0x0E));
        exp.push(Op::Raw(TUNER, vec![0x01, 0x00, 0x0F, 0x00]));
        exp.extend(gate_ops(false, 0x0C));
        assert_eq!(ops, exp);
        assert!(!k.is_open());
    }

    #[test]
    fn plain_variant_power_and_no_gpio_c() {
        let mut d = desc("BQ200X_MxL135RF", "h11", 0x08);
        d.flags = 0x08; // tuner directly on the bus: no gate
        let mut k = kit(d, false);
        k.init().unwrap();
        let ops = take_ops(&k);
        assert_eq!(ops[0], Op::Gpio(0x02, 0x1004, 0x03, 0x03));
        assert_eq!(ops[1], Op::Gpio(0x02, 0x1006, 0x03, 0x03));
        k.open().unwrap();
        assert_eq!(take_ops(&k), vec![Op::Raw(TUNER, vec![0x01, 0x01])]);
        k.tune(&TuneRequest::isdb_t(557_142_857, 6_000_000)).unwrap();
        let ops = take_ops(&k);
        // Not model f200: no band switch; tuner written directly.
        assert_eq!(&ops[..5], &mxl_tune_ops(TUNER)[..]);
        k.close().unwrap();
        assert_eq!(take_ops(&k), vec![Op::Raw(TUNER, vec![0x01, 0x00, 0x0F, 0x00])]);
    }

    #[test]
    fn diversity_init_and_tune() {
        let mut k = kit(desc("BQ200X_MxL135RF", "f201", 0x80), false);
        assert_eq!(k.variant, Variant::MxlDiversity);
        k.init().unwrap();
        let ops = take_ops(&k);
        let mut exp = power_up_ops();
        exp.extend(power_up_ops());
        exp.extend(chip_config_ops(SLAVE, 1, 1));
        exp.extend(chip_config_ops(DEMOD, 1, 2));
        exp.extend([w(DEMOD, 0xC2, 0x2C), w(DEMOD, 0xC3, 0x0F), w(DEMOD, 0xF3, 0x90)]);
        exp.extend([Op::R(SLAVE, 0x1D), w(SLAVE, 0x1D, 0x00), w(SLAVE, 0x1D, 0x20)]);
        exp.extend([Op::R(DEMOD, 0x1D), w(DEMOD, 0x1D, 0x00), w(DEMOD, 0x1D, 0x20)]);
        assert_eq!(&ops[..exp.len()], &exp[..]);
        // Then: tuner 2 init, tuner 1 init, tuner 1 standby, tuner 2 standby.
        let raws: Vec<(u8, usize)> = ops[exp.len()..]
            .iter()
            .filter_map(|o| if let Op::Raw(a, d) = o { Some((*a, d.len())) } else { None })
            .collect();
        assert_eq!(raws, vec![(TUNER2, 1), (TUNER2, 44), (TUNER, 1), (TUNER, 44), (TUNER, 4), (TUNER2, 4)]);

        k.open().unwrap();
        let ops = take_ops(&k);
        assert_eq!(ops.last(), Some(&Op::Gpio(0x02, 0x1006, 0x0C, 0x0C)));
        k.tune(&TuneRequest::isdb_t(557_142_857, 6_000_000)).unwrap();
        let ops = take_ops(&k);
        // Model f201: no band switch. Tuner 2 first, then tuner 1, then slave and master restart.
        let raws: Vec<u8> = ops
            .iter()
            .filter_map(|o| if let Op::Raw(a, d) = o { (d.len() == 20).then_some(*a) } else { None })
            .collect();
        assert_eq!(raws, vec![TUNER2, TUNER]);
        let n = ops.len();
        let mut tail = sync_start_ops(SLAVE, 0x1F, 0xA0);
        tail.extend(sync_start_ops(DEMOD, 0x1F, 0xA0));
        assert_eq!(&ops[n - tail.len()..], &tail[..]);
    }

    #[test]
    fn diversity_kit_of_tuner_id_1_leaves_gpio_alone() {
        let mut d = desc("BQ200X_MxL135RF", "f201", 0x80);
        d.id = 1;
        let mut k = kit(d, false);
        k.init().unwrap();
        k.open().unwrap();
        k.close().unwrap();
        assert!(!take_ops(&k).iter().any(|o| matches!(o, Op::Gpio(..))));
    }

    #[test]
    fn e4005_init_sequence() {
        let mut k = kit(desc("BQ200X_E4005", "f200", 0), false);
        k.init().unwrap();
        let ops = take_ops(&k);
        let start = ops.iter().position(|o| matches!(o, Op::W(TUNER, ..))).unwrap();
        let mut exp = vec![w(TUNER, 0x09, 0x00), w(TUNER, 0x05, 0x00), w(TUNER, 0x09, 0x80)];
        exp.extend(e4005::INIT_TABLE_A.iter().chain(e4005::INIT_TABLE_B.iter()).map(|&(r, v)| w(TUNER, r, v)));
        exp.extend([w(TUNER, 0x06, 0x20), w(TUNER, 0x8A, 0x96)]);
        exp.extend(gate_ops(false, 0x0C));
        exp.extend(gate_ops(true, 0x0E));
        exp.extend([w(TUNER, 0x00, 0x00)]);
        exp.extend(gate_ops(false, 0x0C));
        assert_eq!(&ops[start - 2..], &[gate_ops(true, 0x0C), exp].concat()[..]);
    }

    #[test]
    fn e4005_tune_557() {
        let mut k = kit(desc("BQ200X_E4005", "f200", 0), false);
        k.init().unwrap();
        k.open().unwrap();
        take_ops(&k);
        k.tune(&TuneRequest::isdb_t(557_142_857, 6_000_000)).unwrap();
        let ops = take_ops(&k);
        assert_eq!(ops[0], Op::Gpio(0x82, 0x1006, 0xC0, 0x80));
        let writes: Vec<(u8, u8)> =
            ops.iter().filter_map(|o| if let Op::W(TUNER, r, v) = o { Some((*r, v[0])) } else { None }).collect();
        // LNA calibration: 15 + 100 requests in 0x1F, then the 0x1B average back to 0x1B.
        assert_eq!(writes.iter().take_while(|&&x| x == (0x1F, 0x1F)).count(), 115);
        let after_cal = &writes[115..];
        assert_eq!(
            &after_cal[..7],
            &[(0x1F, 0x0F), (0xFA, 0x08), (0x1B, 0x00), (0x00, 0x02), (0x1A, 0x1C), (0x20, 0x04), (0x8D, 0x00)]
        );
        // 0x8D bits 2:1 = 0b10, divider code 2 (×6), sel 0; IF step: 0x43 = 0x0B, 0x44 = 34, 0x96 = 0xFF.
        assert_eq!(&after_cal[7..12], &[(0x8D, 0x04), (0x0D, 0x02), (0x43, 0x0B), (0x44, 34), (0x96, 0xFF)]);
        // Band 3 → 0x07 bits 2:1 = 0b10, 0x88 = 3; PLL N 0x74, fraction 0x123D; tracking filter 7.
        assert_eq!(
            &after_cal[12..20],
            &[
                (0x07, 0x04),
                (0x88, 0x03),
                (0x8D, 0x04),
                (0x13, 0x00),
                (0x09, 0x74),
                (0x0A, 0x3D),
                (0x0B, 0x12),
                (0x10, 7)
            ]
        );
        let gain: Vec<(u8, u8)> = e4005::GAIN_TABLE.iter().enumerate().map(|(i, &v)| (0x9A + i as u8, v)).collect();
        assert_eq!(&after_cal[20..110], &gain[..]);
        assert_eq!(
            &after_cal[110..],
            &[(0x29, 0x01), (0x2D, 0x1C), (0x20, 0x64), (0x11, 13), (0x12, 5), (0x13, 0), (0x43, 0x1F)]
        );
        // Gate closed after the tuner, then the demodulator restart.
        let n = ops.len();
        let tail = sync_start_ops(DEMOD, 0x1F, 0xA0);
        assert_eq!(&ops[n - tail.len()..], &tail[..]);
        assert_eq!(&ops[n - tail.len() - 2..n - tail.len()], &gate_ops(false, 0x0C)[..]);
    }

    #[test]
    fn e4005_bands_as_the_vendor_sets_them() {
        let mut k = kit(desc("BQ200X_E4005", "f200", 0), false);
        k.init().unwrap();
        k.open().unwrap();
        take_ops(&k);
        // (0x0D, then 0x07, 0x88, 0x8D, 0x13, 0x09, 0x0A, 0x0B, 0x10) of the PLL section.
        let mut pll = |f: u64| -> Vec<(u8, u8)> {
            k.tune(&TuneRequest::isdb_t(f, 6_000_000)).unwrap();
            let writes: Vec<(u8, u8)> = take_ops(&k)
                .iter()
                .filter_map(|o| if let Op::W(TUNER, r, v) = o { Some((*r, v[0])) } else { None })
                .collect();
            let i = writes.iter().position(|w| w.0 == 0x07).unwrap();
            let d = writes[..i].iter().rev().find(|w| w.0 == 0x0D).copied().unwrap();
            [vec![d], writes[i..i + 8].to_vec()].concat()
        };
        // 100 MHz, band 1: VHF2 code, 3-phase mixing with divider code 5 (×32), tracking filter 1.
        assert_eq!(
            pll(100_000_000),
            [
                (0x0D, 0x0D),
                (0x07, 0x00),
                (0x88, 3),
                (0x8D, 0x04),
                (0x13, 0),
                (0x09, 111),
                (0x0A, 0x71),
                (0x0B, 0x1C),
                (0x10, 1)
            ]
        );
        // 200 MHz, band 3: the vendor uses the UHF code here, no 3-phase mixing, code 5 (×16).
        assert_eq!(
            pll(200_000_000),
            [
                (0x0D, 0x05),
                (0x07, 0x04),
                (0x88, 3),
                (0x8D, 0x04),
                (0x13, 0),
                (0x09, 111),
                (0x0A, 0x71),
                (0x0B, 0x1C),
                (0x10, 0)
            ]
        );
        // 400 MHz, band 5: bit3 set with the VHF2 code, 0x88 = 0, code 3 (×8), tracking filter 15.
        assert_eq!(
            pll(400_000_000),
            [
                (0x0D, 0x03),
                (0x07, 0x08),
                (0x88, 0),
                (0x8D, 0x04),
                (0x13, 0),
                (0x09, 111),
                (0x0A, 0x71),
                (0x0B, 0x1C),
                (0x10, 15)
            ]
        );
        // SYNTH1 is read back, so bit3 stays set on the next tune (as in the vendor driver).
        assert_eq!(pll(557_142_857)[1], (0x07, 0x0C));
    }

    #[test]
    fn e4005_negative_if_harmonic_step() {
        let mut d = desc("BQ200X_E4005", "f200", 0);
        d.if_freq_hz = -4_000_000;
        let mut k = kit(d, false);
        k.init().unwrap();
        k.open().unwrap();
        take_ops(&k);
        k.tune(&TuneRequest::isdb_t(547_000_000, 6_000_000)).unwrap();
        let writes: Vec<(u8, u8)> = take_ops(&k)
            .iter()
            .filter_map(|o| if let Op::W(TUNER, r, v) = o { Some((*r, v[0])) } else { None })
            .collect();
        // Zero-IF filter set, then 0x07 = 0x0D and 0x05 = 3.
        assert_eq!(
            &writes[writes.len() - 6..],
            &[(0x11, 13), (0x12, 5), (0x13, 0), (0x43, 28), (0x07, 0x0D), (0x05, 3)]
        );
        assert!(writes.contains(&(0x44, 0x24)));
    }

    #[test]
    fn status_locked_and_cnr() {
        let mut k = kit(desc("BQ200X_MxL135RF", "h55", 0), false);
        k.init().unwrap();
        k.open().unwrap();
        k.tune(&TuneRequest::isdb_t(557_142_857, 6_000_000)).unwrap();
        {
            let mut r = k.dev.transport().regs.borrow_mut();
            for (reg, v) in [
                (0x3A, 0x40),
                (0x70, 0x80),
                (0x92, 0x01),
                (0xDB, 0x40),
                (0xDE, 0x01),
                (0xDA, 0x1B), // layer A 64QAM-class code 3, layer B code 3
                (0xAE, 0x0B),
                (0xAF, 0x00), // layer B raw 0x0B00 → 24000
                (0xC6, 0x00),
                (0xC7, 0x10),
                (0xCD, 0x00),
                (0xCE, 0x00),
                (0xCF, 0x10),
            ] {
                r.insert((DEMOD, reg), v);
            }
        }
        take_ops(&k);
        let st = k.signal_status(field::ALL).unwrap();
        assert_eq!(st.lock, lock::REG3A_BIT7_CLEAR | lock::SIGNAL | lock::LOCKED);
        assert_eq!(st.cnr_mdb, 24000);
        assert_eq!(st.quality, 90);
        assert_eq!(st.ber[1], ber_check(0x10, 0x10));
        assert_eq!(st.ber[2], crate::kit::BER_UNUSED);
        assert_eq!(st.sync_state, 1);
        // The tuner's reading replaces the strength (all-zero MxL registers: 113 dB).
        assert_eq!(st.strength, 113_000);
        assert_eq!(st.mask, field::ALL);
        // First status after the configuration captured the carrier offset: no 0x69/0x6A writes.
        let ops = take_ops(&k);
        assert!(!ops.iter().any(|o| matches!(o, Op::W(DEMOD, 0x69 | 0x6A, _))));
        // Later, an offset drifting by more than 256 steps 0x6A.
        k.dev.transport().regs.borrow_mut().insert((DEMOD, 0x71), 0x30);
        k.dev.transport().regs.borrow_mut().insert((DEMOD, 0x6A), 0x80);
        k.signal_status(field::ALL_BASIC).unwrap();
        let ops = take_ops(&k);
        assert!(ops.contains(&w(DEMOD, 0x6A, 0x81)));
    }

    fn ber_check(err: u32, period: u32) -> u32 {
        demod::ber_e9(err, period)
    }

    #[test]
    fn status_unlocked_after_timeout_reports_lock_only() {
        let mut k = kit(desc("BQ200X_E4005", "f200", 0), false);
        k.init().unwrap();
        take_ops(&k);
        // Never tuned: the 3 s window has passed, so only the lock flags come back.
        let st = k.signal_status(field::ALL).unwrap();
        assert_eq!(st.mask, field::LOCK);
        assert_eq!(st.lock, lock::REG3A_BIT7_CLEAR);
        let regs: Vec<u8> =
            take_ops(&k).iter().filter_map(|o| if let Op::R(_, r) = o { Some(*r) } else { None }).collect();
        assert_eq!(regs, vec![0x3A, 0x70, 0x92]);
    }
}
