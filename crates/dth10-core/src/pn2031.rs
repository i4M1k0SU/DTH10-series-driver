//! The `PN2031_MxL135RF` tuner kit: PN2031 ISDB-T demodulator with a
//! MaxLinear MxL135RF tuner (terrestrial, tuner/filter #0).
//!
//! The kit follows the same pattern as the TC90517 kit ([`crate::kit`]):
//!
//! * Init: power-up, demodulator configuration, tuner init, tuner standby.
//!   Unlike the TC90517 kit the demodulator is not put to standby.
//! * Open (reference counted): tuner wake, GPIO mask C ← open value. The
//!   demodulator stays configured between Close and Open.
//! * Close: GPIO mask C ← close value, tuner standby.
//! * With `TunerDesc.flags` bit5 set, Init only powers down and Open / Close
//!   do the full power-up + configuration / power-down instead.
//! * Tune: `0x08 ← 01` and a fixed raw write to the demodulator, tuner
//!   frequency / bandwidth, then the demodulator's restart sequence.
//! * Signal status: demodulator (lock, C/N, quality, BER A/B/C), then the
//!   tuner overwrites the strength with its own reading.
//!
//! The vendor driver also calls a per-model tune hook first, which only acts
//! on model "f200" (as in the TC90517 kit); it is not reproduced here.

pub mod demod;
pub mod tuner;

pub use demod::{Pn2031, ber_e9, cnr_mdb_from_raw, cnr_tenth_db_from_raw};
pub use tuner::Pn2031Tuner;

use crate::device::Device;
use crate::error::{Error, Result};
use crate::protocol::UsbControl;
use crate::tc90517::{SignalStatus, TuneRequest, field};
use crate::tlv::TunerDesc;

pub struct Pn2031Kit<T: UsbControl> {
    pub dev: Device<T>,
    pub desc: TunerDesc,
    pub demod: Pn2031,
    pub tuner: Pn2031Tuner,
    open_count: u32,
    last_request: Option<TuneRequest>,
}

impl<T: UsbControl> Pn2031Kit<T> {
    /// Build the kit from the device descriptor (kit name `PN2031_MxL135RF`).
    ///
    /// The demodulator address defaults to 0x10 when the descriptor has none.
    /// The tuner is reached through the demodulator's I2C gate unless
    /// `flags` bit3 says it sits directly on the bridge bus.
    pub fn new(dev: Device<T>, desc: TunerDesc) -> Result<Self> {
        if desc.kit_name != "PN2031_MxL135RF" {
            return Err(Error::Unsupported(format!(
                "terrestrial tuner kit {} hw 0x{:x} is not a PN2031 kit",
                desc.kit_name, desc.hw_version
            )));
        }
        let demod = Pn2031::new(desc.demod_i2c_addr, desc.stream_param);
        let gate = if desc.tuner_direct_i2c() { None } else { Some(demod.addr) };
        Ok(Pn2031Kit {
            tuner: Pn2031Tuner::new(desc.tuner_i2c_addr, gate),
            demod,
            dev,
            desc,
            open_count: 0,
            last_request: None,
        })
    }

    /// Init: power-up, demodulator configuration, tuner init, tuner standby.
    /// With `flags` bit5 set: power-down only.
    pub fn init(&mut self) -> Result<()> {
        if self.desc.skip_init_config() {
            return self.dev.power_down(&self.desc);
        }
        self.dev.power_up(&self.desc)?;
        self.demod.configure(&mut self.dev)?;
        self.tuner.init(&mut self.dev, &self.desc)?;
        self.last_request = None;
        self.tuner.standby(&mut self.dev)
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
            self.demod.configure(&mut self.dev)?;
            self.tuner.init(&mut self.dev, &self.desc)?;
            self.last_request = None;
        } else {
            self.tuner.wake(&mut self.dev)?;
        }
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
        if self.desc.skip_init_config() { self.dev.power_down(&self.desc) } else { self.tuner.standby(&mut self.dev) }
    }

    pub fn is_open(&self) -> bool {
        self.open_count > 0
    }

    /// Tune: demodulator pre-tune, tuner frequency (low 32 bits of the
    /// request's frequency) and bandwidth, demodulator post-tune.
    pub fn tune(&mut self, req: &TuneRequest) -> Result<()> {
        if !self.is_open() {
            return Err(Error::Protocol("kit is not open"));
        }
        self.demod.pre_tune(&mut self.dev)?;
        self.tuner.set_frequency(&mut self.dev, req.frequency_u32(), req.bandwidth_hz)?;
        self.demod.post_tune(&mut self.dev)?;
        self.last_request = Some(*req);
        Ok(())
    }

    /// Signal status (no caching here): demodulator first, then the tuner
    /// overwrites `strength` with its own reading.
    pub fn signal_status(&mut self, mask: u32) -> Result<SignalStatus> {
        let mut st = SignalStatus { mask, ..Default::default() };
        self.demod.status(&mut self.dev, &mut st)?;
        st.strength = self.tuner.signal(&mut self.dev)?;
        st.mask |= field::STRENGTH;
        Ok(st)
    }

    pub fn last_request(&self) -> Option<&TuneRequest> {
        self.last_request.as_ref()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tc90517::{BER_UNUSED, lock};
    use core::time::Duration;
    use std::cell::RefCell;
    use std::collections::HashMap;

    const DEMOD: u8 = 0x10;
    const TUNER: u8 = 0x60;
    /// wValue of a register access to the demodulator.
    const REG: u16 = 0x2D00 | DEMOD as u16;
    /// wValue of a raw transfer to the demodulator / the tuner.
    const RAW_DEMOD: u16 = 0x2C00 | DEMOD as u16;
    const RAW_TUNER: u16 = 0x2C00 | TUNER as u16;

    /// One control transfer: (OUT?, wValue, wIndex, data or IN length).
    type Call = (bool, u16, u16, Vec<u8>);

    /// Records every control transfer; IN transfers of register reads are
    /// answered from `regs` (wValue, register) → byte, anything else with zeros.
    #[derive(Default)]
    struct Mock {
        log: RefCell<Vec<Call>>,
        regs: HashMap<(u16, u16), u8>,
    }

    impl UsbControl for Mock {
        fn control_out(&self, _r: u8, v: u16, i: u16, d: &[u8], _: Duration) -> Result<usize> {
            self.log.borrow_mut().push((true, v, i, d.to_vec()));
            Ok(d.len())
        }
        fn control_in(&self, _r: u8, v: u16, i: u16, b: &mut [u8], _: Duration) -> Result<usize> {
            self.log.borrow_mut().push((false, v, i, vec![b.len() as u8]));
            b.fill(0);
            if let Some(&x) = self.regs.get(&(v, i)) {
                b[0] = x;
            }
            Ok(b.len())
        }
    }

    fn desc() -> TunerDesc {
        TunerDesc {
            kit_name: "PN2031_MxL135RF".into(),
            present: 0x1 | 0x4000 | 0x2000,
            hw_version: 0x102,
            demod_i2c_addr: DEMOD,
            tuner_i2c_addr: TUNER,
            tuner_xtal_hz: 24_000_000,
            if_freq_hz: 4_000_000,
            gpio_mask_a: 0x1,
            gpio_mask_b: 0x2,
            gpio_mask_c: 0x4,
            gpio_c_open: 0x4,
            gpio_c_close: 0x0,
            ..Default::default()
        }
    }

    fn kit_with(regs: HashMap<(u16, u16), u8>, d: TunerDesc) -> Pn2031Kit<Mock> {
        Pn2031Kit::new(Device::new(Mock { regs, ..Default::default() }), d).unwrap()
    }

    fn take_log(k: &Pn2031Kit<Mock>) -> Vec<Call> {
        k.dev.transport().log.borrow_mut().drain(..).collect()
    }

    fn w(reg: u8, v: u8) -> Call {
        (true, REG, reg as u16, vec![v])
    }
    fn r(reg: u8) -> Call {
        (false, REG, reg as u16, vec![1])
    }
    fn gate() -> Call {
        w(0xFE, 0x00)
    }
    fn tuner_w(data: &[u8]) -> Call {
        (true, RAW_TUNER, 0x30, data.to_vec())
    }
    fn tuner_r() -> Call {
        (false, RAW_TUNER, 0x33, vec![1])
    }
    /// A batch of single-byte register writes to the demodulator.
    fn batch(pairs: &[(u8, u8)]) -> Call {
        let mut p = Vec::new();
        for &(reg, v) in pairs {
            p.extend_from_slice(&[0, 2, 0, DEMOD << 1, reg, v]);
        }
        (true, 0x2B00, 0, p)
    }
    fn gpio(index: u16, mask: u32, value: u32) -> Call {
        let mut p = vec![0u8; 4];
        p.extend_from_slice(&mask.to_le_bytes());
        p.extend_from_slice(&value.to_le_bytes());
        (true, 0, index, p)
    }
    /// Gated `{0xFB, reg}` read of one tuner register.
    fn tuner_reg_read(reg: u8) -> [Call; 4] {
        [gate(), tuner_w(&[0xFB, reg]), gate(), tuner_r()]
    }

    #[test]
    fn rejects_other_kits() {
        let d = TunerDesc { kit_name: "TC90517_MxL135RF".into(), ..desc() };
        assert!(Pn2031Kit::new(Device::new(Mock::default()), d).is_err());
    }

    #[test]
    fn init_sequence() {
        let mut k = kit_with(HashMap::new(), desc());
        k.init().unwrap();
        let log = take_log(&k);

        // Power-up: enable A|B|C, clear A|B, set A, set B.
        assert_eq!(log[0], gpio(0x1004, 0x7, 0x7));
        assert_eq!(log[1], gpio(0x1006, 0x3, 0));
        assert_eq!(log[2], gpio(0x1006, 0x1, 0x1));
        assert_eq!(log[3], gpio(0x1006, 0x2, 0x2));
        // Demodulator reset pulse.
        assert_eq!(log[4], w(0x70, 0x00));
        assert_eq!(log[5], w(0x70, 0xFF));
        // First configuration batch, transcribed.
        assert_eq!(
            log[6],
            batch(&[
                (0x08, 0x01),
                (0x73, 0x00),
                (0x08, 0x00),
                (0x06, 0xFF),
                (0x1F, 0x0F),
                (0x1C, 0x01),
                (0x09, 0x1A),
                (0x50, 0xD1),
                (0x51, 0x22),
                (0x71, 0x00),
                (0x28, 0x2A),
                (0x29, 0x00),
                (0x2A, 0xFF),
                (0x2B, 0x80),
                (0x3B, 0x21),
                (0x3C, 0x3A),
                (0x28, 0x20),
                (0x29, 0x3E),
                (0x2A, 0xE0),
                (0x2B, 0x8F),
            ])
        );
        // Twelve batches in all.
        let batches: Vec<&Call> = log.iter().filter(|c| c.1 == 0x2B00).collect();
        assert_eq!(batches.len(), 12);
        assert!(batches.iter().all(|c| c.3.len() % 6 == 0 && c.3.len() <= 256));
        // The stream-parameter batch: stream_param bit0 clear → indirect 0xD5 = 1.
        assert_eq!(
            *batches[6],
            batch(&[
                (0x50, 0x51),
                (0x51, 0x04),
                (0x50, 0x50),
                (0x51, 0x02),
                (0x45, 0x04),
                (0x48, 0x04),
                (0x50, 0xD5),
                (0x51, 0x01),
                (0x50, 0xD6),
                (0x51, 0x17),
                (0x50, 0xD2),
                (0x51, 0x03),
                (0x50, 0xD7),
                (0x51, 0xB7),
            ])
        );
        // The last coefficient batch starts with word 0x16 = 0x000005 and ends with word 0x1E = 0x0000BA.
        let last = &batches[11].3;
        assert_eq!(last.len(), 9 * 4 * 6);
        assert_eq!(
            &last[..24],
            &[0, 2, 0, 0x20, 0x28, 0x16, 0, 2, 0, 0x20, 0x29, 0, 0, 2, 0, 0x20, 0x2A, 0, 0, 2, 0, 0x20, 0x2B, 0x05]
        );
        assert_eq!(&last[last.len() - 6..], &[0, 2, 0, 0x20, 0x2B, 0xBA]);
        // After the batches: indirect write through 0x6D/0x6E, then the tuner.
        let i = log.iter().rposition(|c| c.1 == 0x2B00).unwrap();
        assert_eq!(log[i + 1], w(0x6D, 0x71));
        assert_eq!(log[i + 2], w(0x6E, 0xE5));
        assert_eq!(log[i + 3], gate());
        assert_eq!(log[i + 4], tuner_w(&[0xFF]));
        assert_eq!(log[i + 5], gate());
        assert_eq!(log[i + 6].1, RAW_TUNER);
        assert_eq!(log[i + 6].3.len(), 44);
        // Tuner standby; the demodulator is left running.
        assert_eq!(log[i + 7], gate());
        assert_eq!(log[i + 8], tuner_w(&[0x01, 0x00, 0x0F, 0x00]));
        assert_eq!(log.len(), i + 9);
    }

    #[test]
    fn stream_param_bit0_selects_d5() {
        let d = TunerDesc { stream_param: 0x1, ..desc() };
        let mut k = kit_with(HashMap::new(), d);
        k.init().unwrap();
        let log = take_log(&k);
        let batches: Vec<&Call> = log.iter().filter(|c| c.1 == 0x2B00).collect();
        assert_eq!(&batches[6].3[7 * 6..8 * 6], &[0, 2, 0, 0x20, 0x51, 0x00]);
    }

    #[test]
    fn open_tune_close_sequence() {
        let mut regs = HashMap::new();
        regs.insert((REG, 0x45), 0x9A);
        let mut k = kit_with(regs, desc());
        assert!(k.tune(&TuneRequest::isdb_t(557_142_857, 6_000_000)).is_err());

        k.open().unwrap();
        k.open().unwrap();
        assert_eq!(take_log(&k), vec![gate(), tuner_w(&[0x01, 0x01]), gpio(0x1006, 0x4, 0x4)]);

        k.tune(&TuneRequest::isdb_t(557_142_857, 6_000_000)).unwrap();
        let mut expect = vec![
            w(0x08, 0x01),
            (true, RAW_DEMOD, 0x30, vec![0x71, 0x00, 0x39, 0x01, 0x44, 0x00]),
            gate(),
            // 557.142857 MHz → word 0x8B49, 6 MHz bandwidth, ≥ 333 MHz.
            tuner_w(&[
                0x0F, 0x00, 0x0C, 0x15, 0x0D, 0x49, 0x0E, 0x8B, 0x1F, 0x87, 0x20, 0x1F, 0x21, 0x87, 0x22, 0x1F, 0x80,
                0x41, 0x0F, 0x01,
            ]),
        ];
        expect.extend(tuner_reg_read(0xD8));
        expect.extend(tuner_reg_read(0xD8));
        expect.extend([
            w(0x08, 0x00),
            r(0x45),
            w(0x45, 0x9A | 0x10),
            w(0x45, 0x9A & 0x6F),
            w(0x53, 0x00),
            w(0x53, 0x07),
            batch(&[(0x5F, 0x00), (0x5F, 0x07)]),
            w(0x50, 0xB1),
            w(0x51, 0x07),
            w(0x51, 0x00),
        ]);
        assert_eq!(take_log(&k), expect);
        assert_eq!(k.last_request().unwrap().frequency_hz, 557_142_857);

        k.close().unwrap();
        assert!(k.is_open());
        assert!(take_log(&k).is_empty());
        k.close().unwrap();
        assert!(!k.is_open());
        assert_eq!(take_log(&k), vec![gpio(0x1006, 0x4, 0x0), gate(), tuner_w(&[0x01, 0x00, 0x0F, 0x00])]);
    }

    #[test]
    fn direct_tuner_path_has_no_gate() {
        let d = TunerDesc { flags: 0x8, ..desc() };
        let mut k = kit_with(HashMap::new(), d);
        k.open().unwrap();
        assert_eq!(take_log(&k), vec![tuner_w(&[0x01, 0x01]), gpio(0x1006, 0x4, 0x4)]);
    }

    #[test]
    fn skip_init_config_powers_at_open_close() {
        let d = TunerDesc { flags: 0x20, ..desc() };
        let mut k = kit_with(HashMap::new(), d);
        k.init().unwrap();
        assert_eq!(take_log(&k), vec![gpio(0x1004, 0x7, 0x7), gpio(0x1006, 0x3, 0)]);
        k.open().unwrap();
        let log = take_log(&k);
        assert_eq!(log[4], w(0x70, 0x00));
        assert_eq!(*log.last().unwrap(), gpio(0x1006, 0x4, 0x4));
        k.close().unwrap();
        assert_eq!(take_log(&k), vec![gpio(0x1006, 0x4, 0x0), gpio(0x1004, 0x7, 0x7), gpio(0x1006, 0x3, 0)]);
    }

    #[test]
    fn signal_status_locked_layer_a() {
        let mut regs = HashMap::new();
        regs.insert((REG, 0x0A), 0x29); // sync state 9
        regs.insert((REG, 0x6E), 0x01);
        regs.insert((REG, 0x46), 0x0C); // raw 0x0C6C = 3180 → 16.0 dB
        regs.insert((REG, 0x47), 0x6C);
        regs.insert((REG, 0x45), 0x00);
        regs.insert((REG, 0x60), 0x01); // only layer A in use
        regs.insert((REG, 0x64), 0x00);
        regs.insert((REG, 0x65), 0x03);
        regs.insert((REG, 0x66), 0xE8); // 1000 errors
        regs.insert((REG, 0x5F), 0x07);
        let mut k = kit_with(regs, desc());
        let st = k.signal_status(field::ALL).unwrap();
        assert_eq!(st.lock, lock::SYNC_STARTED | lock::SIGNAL | lock::LOCKED | lock::REG81_BIT7);
        assert_eq!(st.sync_state, 9);
        assert_eq!(st.cnr_mdb, 16000);
        assert_eq!(st.quality, 60);
        assert_eq!(st.ber, [612_745, BER_UNUSED, BER_UNUSED]);
        // Tuner reading with both MxL registers 0: 1000 × 113.
        assert_eq!(st.strength, 113_000);
        assert_eq!(st.mask, field::ALL);

        let log = take_log(&k);
        let mut expect = vec![r(0x0A), w(0x6D, 0x84), r(0x6E), r(0x46), r(0x47), r(0x45), w(0x45, 0x10), w(0x45, 0x00)];
        expect.extend([r(0x60), r(0x64), r(0x65), r(0x66), r(0x5F), batch(&[(0x5F, 0x06), (0x5F, 0x07)])]);
        expect.extend(tuner_reg_read(0xCC));
        expect.extend(tuner_reg_read(0xBA));
        assert_eq!(log, expect);
    }

    #[test]
    fn signal_status_quality_from_cnr_without_ber() {
        let mut regs = HashMap::new();
        regs.insert((REG, 0x0A), 0x05); // signal, not locked
        regs.insert((REG, 0x46), 0x03); // raw 788 → 30.0 dB
        regs.insert((REG, 0x47), 0x14);
        let mut k = kit_with(regs, desc());
        let st = k.signal_status(field::ALL_BASIC).unwrap();
        assert_eq!(st.lock, lock::SYNC_STARTED | lock::SIGNAL);
        assert_eq!(st.cnr_mdb, 30000);
        assert_eq!(st.quality, 95);
        assert_eq!(st.mask, field::ALL_BASIC);
        assert!(!take_log(&k).contains(&r(0x60)));
    }
}
