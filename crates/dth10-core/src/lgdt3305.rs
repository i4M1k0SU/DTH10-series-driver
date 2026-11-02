//! The `LGDT3305_MxL5007T` tuner kit: LG LGDT3305 demodulator (ATSC 8VSB,
//! J.83 Annex B 64/256QAM) with a MaxLinear MxL5007T tuner. See
//! `docs/15-lgdt3305-kit.md`.
//!
//! * The tuner is the MaxLinear object shared with the ISDB-T kits
//!   ([`Mxl13xRf`] with [`MxlType::MxL5007`]); its register traffic is the
//!   same, the descriptor's hardware version 0x101 selects the ATSC mode value
//!   of tuner register 0x06.
//! * The tuner is addressed with its own I2C address. Unless the descriptor
//!   says the tuner is directly on the bridge bus (`flags` bit3), every tuner
//!   transaction is bracketed by opening and closing the demodulator's I2C
//!   repeater (register 0x0001 bit5).
//! * The tune request carries the channel centre frequency: the conversion
//!   from an analog video-carrier frequency (+1.75 MHz) happens in the
//!   original driver's filter layer, before the kit is called.
//! * The modulation comes from the request's modulation code (4 = 64QAM,
//!   5 = 256QAM, anything else = 8VSB); the demodulator is set up again only
//!   when it changes.

pub mod demod;

pub use demod::{Lgdt3305, Modulation, lock};

use core::time::Duration;
use std::thread::sleep;

use crate::device::Device;
use crate::error::{Error, Result};
use crate::mxl13xrf::{Mxl13xRf, MxlType};
use crate::protocol::UsbControl;
use crate::tc90517::{SignalStatus, TuneRequest, field};
use crate::tlv::TunerDesc;
use crate::tuner_bus::{TunerBus, TunerPath};

/// Kit name the original driver builds this kit for (exact match).
pub const KIT_NAME: &str = "LGDT3305_MxL5007T";

/// Presence bit of the descriptor's demodulator IF value (tag 0x62).
const PRESENT_DEMOD_IF: u32 = 0x20000;

pub struct Lgdt3305Kit<T: UsbControl> {
    pub dev: Device<T>,
    pub desc: TunerDesc,
    pub demod: Lgdt3305,
    pub tuner: Mxl13xRf,
    /// Tuner transactions go through the demodulator's I2C repeater.
    gated: bool,
    open_count: u32,
    last_request: Option<TuneRequest>,
}

impl<T: UsbControl> Lgdt3305Kit<T> {
    /// Build the kit from the device descriptor. The demodulator IF is the
    /// descriptor's tag-0x62 value (`val_62`) when present, else 44 MHz; the
    /// tuner keeps using its own IF (`if_freq_hz`) for its init table.
    pub fn new(dev: Device<T>, desc: TunerDesc) -> Result<Self> {
        if desc.kit_name != KIT_NAME {
            return Err(Error::Unsupported(format!(
                "terrestrial tuner kit {} hw 0x{:x} is not the LGDT3305 kit",
                desc.kit_name, desc.hw_version
            )));
        }
        let demod_if = if desc.present & PRESENT_DEMOD_IF != 0 { desc.val_62 as i32 } else { demod::DEFAULT_IF_HZ };
        let bus = TunerBus { tuner_addr: desc.tuner_i2c_addr, path: TunerPath::Direct };
        Ok(Lgdt3305Kit {
            demod: Lgdt3305::new(desc.demod_i2c_addr, demod_if),
            tuner: Mxl13xRf::new(bus, MxlType::MxL5007),
            gated: !desc.tuner_direct_i2c(),
            dev,
            desc,
            open_count: 0,
            last_request: None,
        })
    }

    /// Run one tuner transaction, with the repeater opened before and closed
    /// after it when the tuner sits behind the demodulator. As in the
    /// original, a failed open skips the transaction and the result of the
    /// close is not checked.
    fn with_tuner<R>(&mut self, f: impl FnOnce(&Mxl13xRf, &mut Device<T>) -> Result<R>) -> Result<R> {
        if self.gated {
            self.demod.gate(&mut self.dev, true)?;
        }
        let r = f(&self.tuner, &mut self.dev);
        if self.gated {
            let _ = self.demod.gate(&mut self.dev, false);
        }
        r
    }

    /// Demodulator configuration and tuner initialisation (the kit's
    /// configure step): set-up block of the current modulation, soft reset,
    /// TS output control, then the tuner's soft reset and init table.
    /// Forgets the last tune request.
    fn configure(&mut self) -> Result<()> {
        self.last_request = None;
        self.demod.configure(&mut self.dev, self.desc.stream_param)?;
        let desc = self.desc.clone();
        self.with_tuner(|t, dev| t.init(dev, &desc))
    }

    /// Init: power-up, configure, tuner to standby. The demodulator is not
    /// put into a low-power state. With `flags` bit5 the chips are only
    /// powered down (Open brings them up).
    pub fn init(&mut self) -> Result<()> {
        if self.desc.skip_init_config() {
            return self.dev.power_down(&self.desc);
        }
        self.dev.power_up(&self.desc)?;
        self.configure()?;
        self.with_tuner(|t, dev| t.standby(dev))
    }

    /// Open (reference counted): tuner wake (or power-up + configure with
    /// `flags` bit5), then GPIO mask C ← open value.
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
            self.dev.power_up(&self.desc).and_then(|_| self.configure())
        } else {
            self.with_tuner(|t, dev| t.wake(dev))
        };
        // The original writes the GPIO open value whatever the outcome.
        let g = self.dev.gpio_write(self.desc.gpio_mask_c, self.desc.gpio_c_open);
        r.and(g)
    }

    /// Close: GPIO mask C ← close value, then tuner standby (or power-down
    /// with `flags` bit5).
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
            self.with_tuner(|t, dev| t.standby(dev))
        }
    }

    pub fn is_open(&self) -> bool {
        self.open_count > 0
    }

    /// Tune: 20 ms, modulation switch when needed, demodulator IF / NCO
    /// set-up, tuner frequency (`req.frequency_hz` = channel centre, low 32
    /// bits; bandwidth 6 / 8 MHz changes the tuner's filter, any other value
    /// keeps the 6 MHz setting), soft reset.
    ///
    /// The original's per-model tune hook acts on model "f200" only and is
    /// not reproduced (that model is not an ATSC unit).
    pub fn tune(&mut self, req: &TuneRequest) -> Result<()> {
        if !self.is_open() {
            return Err(Error::Protocol("kit is not open"));
        }
        sleep(Duration::from_millis(20));
        self.demod.set_modulation(&mut self.dev, Modulation::from_request(req.modulation))?;
        self.demod.set_if(&mut self.dev)?;
        let (freq, bw) = (req.frequency_u32(), req.bandwidth_hz);
        self.with_tuner(|t, dev| t.set_frequency(dev, freq, bw))?;
        self.demod.soft_reset(&mut self.dev)?;
        self.last_request = Some(*req);
        Ok(())
    }

    /// Signal status (no caching here): demodulator first (lock, provisional
    /// strength, quality, C/N in 0.001 dB, packet-error value in `ber[0]`
    /// when [`field::BER_A`] is requested), then the tuner overwrites
    /// `strength` with its own reading.
    pub fn signal_status(&mut self, mask: u32) -> Result<SignalStatus> {
        let mut st = SignalStatus { mask, ..Default::default() };
        self.demod.status(&mut self.dev, &mut st)?;
        st.strength = self.with_tuner(|t, dev| t.signal(dev))?;
        st.mask |= field::STRENGTH;
        Ok(st)
    }

    pub fn last_request(&self) -> Option<&TuneRequest> {
        self.last_request.as_ref()
    }

    /// Modulation the demodulator is currently set up for.
    pub fn modulation(&self) -> Modulation {
        self.demod.mode
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mxl13xrf::init_registers;
    use std::cell::RefCell;
    use std::collections::HashMap;

    const D: u8 = 0x0E;
    const TU: u8 = 0x60;

    /// One control transfer: OUT (wValue, wIndex, data) or IN (wValue, wIndex, length).
    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Op {
        Out(u16, u16, Vec<u8>),
        In(u16, u16, usize),
    }

    /// Records transfers and emulates the register files of both chips:
    /// a 2-byte write to the demodulator (or `FB reg` to the tuner) sets the
    /// read pointer, a 3-byte write to the demodulator stores a value.
    #[derive(Default)]
    struct Mock {
        log: RefCell<Vec<Op>>,
        regs: RefCell<HashMap<(u8, u16), u8>>,
        ptr: RefCell<HashMap<u8, u16>>,
    }

    impl UsbControl for Mock {
        fn control_out(&self, _r: u8, v: u16, i: u16, d: &[u8], _: Duration) -> Result<usize> {
            self.log.borrow_mut().push(Op::Out(v, i, d.to_vec()));
            if v & 0xFF00 == 0x2C00 && i == 0x30 {
                let addr = v as u8;
                match (addr, d) {
                    (D, [hi, lo]) => {
                        self.ptr.borrow_mut().insert(addr, u16::from_be_bytes([*hi, *lo]));
                    }
                    (D, [hi, lo, val]) => {
                        self.regs.borrow_mut().insert((addr, u16::from_be_bytes([*hi, *lo])), *val);
                    }
                    (TU, [0xFB, r]) => {
                        self.ptr.borrow_mut().insert(addr, *r as u16);
                    }
                    _ => {}
                }
            }
            Ok(d.len())
        }
        fn control_in(&self, _r: u8, v: u16, i: u16, b: &mut [u8], _: Duration) -> Result<usize> {
            self.log.borrow_mut().push(Op::In(v, i, b.len()));
            let addr = v as u8;
            let p = self.ptr.borrow().get(&addr).copied().unwrap_or(0);
            b.fill(self.regs.borrow().get(&(addr, p)).copied().unwrap_or(0));
            Ok(b.len())
        }
    }

    /// Hand-written expected transfer list.
    #[derive(Default)]
    struct Exp(Vec<Op>);

    impl Exp {
        fn rd(&mut self, reg: u16) -> &mut Self {
            self.0.push(Op::Out(0x2C00 | D as u16, 0x30, reg.to_be_bytes().to_vec()));
            self.0.push(Op::In(0x2C00 | D as u16, 0x33, 1));
            self
        }
        fn wr(&mut self, reg: u16, v: u8) -> &mut Self {
            let [hi, lo] = reg.to_be_bytes();
            self.0.push(Op::Out(0x2C00 | D as u16, 0x30, vec![hi, lo, v]));
            self
        }
        /// Read-modify-write; `v` is the value written.
        fn rmw(&mut self, reg: u16, v: u8) -> &mut Self {
            self.rd(reg).wr(reg, v)
        }
        fn tw(&mut self, data: &[u8]) -> &mut Self {
            self.0.push(Op::Out(0x2C00 | TU as u16, 0x30, data.to_vec()));
            self
        }
        fn tr(&mut self, reg: u8) -> &mut Self {
            self.0.push(Op::Out(0x2C00 | TU as u16, 0x30, vec![0xFB, reg]));
            self.0.push(Op::In(0x2C00 | TU as u16, 0x33, 1));
            self
        }
        fn gate_open(&mut self) -> &mut Self {
            self.rmw(0x0001, 0x00)
        }
        fn gate_close(&mut self) -> &mut Self {
            self.rmw(0x0001, 0x20)
        }
        fn gpio(&mut self, index: u16, mask: u32, value: u32) -> &mut Self {
            let mut p = vec![0u8; 4];
            p.extend_from_slice(&mask.to_le_bytes());
            p.extend_from_slice(&value.to_le_bytes());
            self.0.push(Op::Out(0x0000, index, p));
            self
        }
    }

    /// An ATSC unit: hw version 0x101, MxL5007T at 0x60 with a 24 MHz crystal
    /// and a 44 MHz IF, LGDT3305 at 0x0E with the default (44 MHz) IF.
    fn atsc_desc() -> TunerDesc {
        TunerDesc {
            present: 0x1 | 0x4000 | 0x2000,
            kit_name: KIT_NAME.into(),
            hw_version: 0x101,
            demod_i2c_addr: D,
            tuner_i2c_addr: TU,
            tuner_xtal_hz: 24_000_000,
            if_freq_hz: 44_000_000,
            gpio_mask_a: 0x1,
            gpio_mask_b: 0x2,
            gpio_mask_c: 0x4,
            gpio_c_open: 0x4,
            gpio_c_close: 0x0,
            ..Default::default()
        }
    }

    fn kit() -> Lgdt3305Kit<Mock> {
        Lgdt3305Kit::new(Device::new(Mock::default()), atsc_desc()).unwrap()
    }

    fn take(k: &Lgdt3305Kit<Mock>) -> Vec<Op> {
        std::mem::take(&mut *k.dev.transport().log.borrow_mut())
    }

    fn flatten(p: &[(u8, u8)]) -> Vec<u8> {
        p.iter().flat_map(|&(r, v)| [r, v]).collect()
    }

    fn assert_ops(got: &[Op], exp: &[Op]) {
        for (i, (g, e)) in got.iter().zip(exp).enumerate() {
            assert_eq!(g, e, "transfer #{i}");
        }
        assert_eq!(got.len(), exp.len(), "number of transfers");
    }

    fn expected_init() -> Vec<Op> {
        let mut e = Exp::default();
        // Power-up: enable A|B|C, clear A|B, set A, set B.
        e.gpio(0x1004, 0x7, 0x7).gpio(0x1006, 0x3, 0).gpio(0x1006, 0x1, 0x1).gpio(0x1006, 0x2, 0x2);
        // 8VSB set-up.
        e.rmw(0x0000, 0x03);
        e.wr(0x0012, 0x32).wr(0x0013, 0xC4).wr(0x0314, 0xE1).wr(0x0308, 0x80).wr(0x0309, 0x00);
        e.wr(0x030C, 0x30).wr(0x043F, 0xAA);
        e.rmw(0x0116, 0x00).rmw(0x030D, 0x04).rmw(0x0002, 0x00).rmw(0x0002, 0x01);
        // Soft reset (one read, two writes), TS output control.
        e.rd(0x0002).wr(0x0002, 0x00).wr(0x0002, 0x01);
        e.wr(0x050E, 0x3B);
        // Tuner soft reset + init table, then standby, each behind the repeater.
        let table = init_registers(&atsc_desc()).unwrap();
        e.gate_open().tw(&[0xFF]).tw(&flatten(&table)).gate_close();
        e.gate_open().tw(&[0x01, 0x00, 0x0F, 0x00]).gate_close();
        e.0
    }

    #[test]
    fn rejects_other_kits() {
        let desc = TunerDesc { kit_name: "TC90517_MxL135RF".into(), ..atsc_desc() };
        assert!(Lgdt3305Kit::new(Device::new(Mock::default()), desc).is_err());
    }

    #[test]
    fn tuner_init_table_uses_atsc_mode() {
        let t = init_registers(&atsc_desc()).unwrap();
        let get = |r: u8| t.iter().find(|p| p.0 == r).unwrap().1;
        assert_eq!(get(0x06), 0x12); // hw 0x101
        assert_eq!(get(0x02), 0x0B); // IF 44 MHz, not inverted
        assert_eq!(get(0x03), 0x40); // 24 MHz crystal, no clock out
        assert_eq!(get(0x05), 0x04);
    }

    #[test]
    fn init_sequence() {
        let mut k = kit();
        k.init().unwrap();
        assert_ops(&take(&k), &expected_init());
        assert!(!k.is_open());
        assert_eq!(k.modulation(), Modulation::Vsb8);
    }

    #[test]
    fn open_close_sequence() {
        let mut k = kit();
        k.init().unwrap();
        take(&k);
        k.open().unwrap();
        k.open().unwrap();
        let mut e = Exp::default();
        e.gate_open().tw(&[0x01, 0x01]).gate_close();
        e.gpio(0x1006, 0x4, 0x4);
        assert_ops(&take(&k), &e.0);
        k.close().unwrap();
        assert!(take(&k).is_empty());
        assert!(k.is_open());
        k.close().unwrap();
        let mut e = Exp::default();
        e.gpio(0x1006, 0x4, 0x0);
        e.gate_open().tw(&[0x01, 0x00, 0x0F, 0x00]).gate_close();
        assert_ops(&take(&k), &e.0);
        assert!(!k.is_open());
    }

    #[test]
    fn tune_atsc_channel_14() {
        let mut k = kit();
        k.init().unwrap();
        k.open().unwrap();
        take(&k);
        // US channel 14: video carrier 471.25 MHz, centre 473 MHz, 6 MHz wide, 8VSB.
        let req =
            TuneRequest { frequency_hz: 473_000_000, bandwidth_hz: 6_000_000, modulation: 3, ..Default::default() };
        k.tune(&req).unwrap();
        let mut e = Exp::default();
        // Modulation unchanged (8VSB): no set-up block. IF 44 MHz → high-IF path.
        e.rmw(0x0007, 0x40).rmw(0x0126, 0x00).rmw(0x000D, 0x00);
        e.wr(0x0106, 0x46).wr(0x0107, 0x9C).wr(0x0108, 0x08).wr(0x0109, 0xE2);
        // 473 MHz = 0x7640 in 1/64 MHz; >= 333 MHz → 0x80 = 0x41.
        e.gate_open();
        e.tw(&[
            0x0F, 0x00, 0x0C, 0x15, 0x0D, 0x40, 0x0E, 0x76, 0x1F, 0x87, 0x20, 0x1F, 0x21, 0x87, 0x22, 0x1F, 0x80, 0x41,
            0x0F, 0x01,
        ]);
        e.tr(0xD8).tr(0xD8);
        e.gate_close();
        e.rd(0x0002).wr(0x0002, 0x00).wr(0x0002, 0x01);
        assert_ops(&take(&k), &e.0);
        assert_eq!(k.last_request(), Some(&req));
    }

    #[test]
    fn tune_switches_to_qam256() {
        let mut k = kit();
        k.init().unwrap();
        k.open().unwrap();
        take(&k);
        let req =
            TuneRequest { frequency_hz: 117_000_000, bandwidth_hz: 6_000_000, modulation: 5, ..Default::default() };
        k.tune(&req).unwrap();
        let log = take(&k);
        let mut e = Exp::default();
        e.rd(0x0000).wr(0x0000, 0x01);
        e.wr(0x0012, 0x2A).wr(0x0013, 0x80).wr(0x0314, 0xE3).wr(0x0308, 0x88).wr(0x0309, 0x88);
        e.wr(0x030C, 0x34).wr(0x043F, 0xAE);
        e.rmw(0x030D, 0x04).rmw(0x0002, 0x00).rmw(0x0002, 0x01);
        e.rd(0x0002).wr(0x0002, 0x00).wr(0x0002, 0x01);
        // High IF, QAM: 0x0007 bit6, spectrum bits in 0x0504, IF range, NCO.
        e.rmw(0x0007, 0x40).rmw(0x0504, 0x00).wr(0x0504, 0x08).rmw(0x000D, 0x00);
        e.wr(0x0106, 0x70).wr(0x0107, 0xA3).wr(0x0108, 0xD7).wr(0x0109, 0x0A);
        assert_ops(&log[..e.0.len()], &e.0);
        assert_eq!(k.modulation(), Modulation::Qam256);
        // Same modulation again: no set-up block, the IF step comes first.
        take(&k);
        k.tune(&req).unwrap();
        assert_eq!(take(&k)[..3], Exp::default().rmw(0x0007, 0x40).0[..]);
    }

    #[test]
    fn tune_requires_open() {
        let mut k = kit();
        assert!(k.tune(&TuneRequest::isdb_t(473_000_000, 6_000_000)).is_err());
    }

    #[test]
    fn direct_tuner_has_no_repeater_steps() {
        let desc = TunerDesc { flags: 0x8, ..atsc_desc() };
        let mut k = Lgdt3305Kit::new(Device::new(Mock::default()), desc).unwrap();
        k.init().unwrap();
        take(&k);
        k.open().unwrap();
        let mut e = Exp::default();
        e.tw(&[0x01, 0x01]).gpio(0x1006, 0x4, 0x4);
        assert_ops(&take(&k), &e.0);
    }

    #[test]
    fn status_vsb_locked() {
        let mut k = kit();
        {
            let mut r = k.dev.transport().regs.borrow_mut();
            r.insert((D, 0x0000), 0x03);
            r.insert((D, 0x031E), 0x01);
            r.insert((D, 0x011D), 0x80);
            r.insert((D, 0x0003), 0x04);
            r.insert((D, 0x0418), 0x01); // PT MSE = 0x00100 = 256 → 20 dB
            r.insert((D, 0x050D), 0x01); // one uncorrectable packet
            r.insert((TU, 0xCC), 10);
            r.insert((TU, 0xBA), 3);
        }
        let st = k.signal_status(field::ALL).unwrap();
        assert_eq!(st.lock, lock::AGC | lock::CARRIER | lock::LOCKED);
        assert_eq!(st.cnr_mdb, 19_998);
        assert_eq!(st.quality, 100);
        assert_eq!(st.ber[0], 77_309);
        assert_eq!(st.strength, 100_000);
        assert_eq!(st.mask, field::LOCK | field::STRENGTH | field::QUALITY | field::CNR | field::BER_A);

        let mut e = Exp::default();
        e.rd(0x0000).rd(0x031E).rd(0x011D).rd(0x0003).rd(0x0417).rd(0x0418).rd(0x0419).rd(0x050C).rd(0x050D);
        e.gate_open().tr(0xCC).tr(0xBA).gate_close();
        assert_ops(&take(&k), &e.0);
    }

    #[test]
    fn status_qam_unlocked() {
        let mut k = kit();
        k.dev.transport().regs.borrow_mut().insert((D, 0x0000), 0x00);
        let st = k.signal_status(field::BER_A).unwrap();
        assert_eq!(st.lock, 0);
        assert_eq!(st.cnr_mdb, 0);
        assert_eq!(st.quality, 0);
        assert_eq!(st.ber[0], crate::tc90517::BER_UNUSED);
        let mut e = Exp::default();
        e.rd(0x0000).rd(0x031E).rd(0x011D).rd(0x021B).rd(0x0003).rd(0x050A).rd(0x011B).rd(0x011C);
        e.gate_open().tr(0xCC).tr(0xBA).gate_close();
        assert_ops(&take(&k), &e.0);
    }

    #[test]
    fn status_rejects_mode_2() {
        let mut k = kit();
        k.dev.transport().regs.borrow_mut().insert((D, 0x0000), 0x02);
        assert!(k.signal_status(field::ALL).is_err());
    }
}
