//! Front-end kits (demodulator + tuner) driven over the ViXS I2C master.
//!
//! The host drives the front end itself, as the vendor's user-mode filter
//! does (docs/26 4-tuning). Every kit pairs a Toshiba TC905xx-family ISDB-T
//! demodulator ([`demod`]) with one of three tuners ([`KitKind`], picked
//! from the board ID by [`crate::model::kit_for_board`]):
//!
//! * MJ100 ([`mj100`]): a register-map tuner with a fractional-N PLL and a
//!   second synthesiser, set up from single-precision float arithmetic;
//! * MJ102 / MJ110 ([`mj10x`]): a tuner that takes the frequency in
//!   1/64 MHz, behind the demodulator's repeater;
//! * TOSHIBA ([`toshiba`]): an up-converting double-conversion tuner with
//!   an IF correction byte in an EEPROM.
//!
//! [`Frontend`] runs the vendor's sequences: [`Frontend::init`] (init
//! table, mode, re-tune to the remembered frequency), [`Frontend::tune`]
//! (TS off, attenuator on, tuner, demodulator resync, 30 ms, TS on — no
//! lock wait, see [`Frontend::wait_lock`]), [`Frontend::sleep`] and the
//! signal readouts. The attenuator control and the MJ100 VHF toggle the
//! vendor runs in threads are driven by the caller ([`att`]). The kernel
//! driver's standby writes at start-up are in [`standby_steps`].
//!
//! Frequencies are in Hz, the channel centre + 1/7 MHz ([`channel`]). The
//! first initialisation of the vendor tunes to 0xFFFFFFFF Hz because no
//! frequency is set yet; here the re-tune is skipped until a frequency is
//! known, so the TS output stays off until the first [`Frontend::tune`].
//!
//! Nothing here is verified on hardware. The register sequences come from
//! static analysis of the vendor filter; the worked PLL examples in the
//! tests were obtained by re-deriving the formulas (docs/26 `tune_ref.py`),
//! not from vendor output or I2C captures.

pub mod att;
pub mod channel;
pub mod demod;
pub mod mj100;
pub mod mj10x;
pub mod regs;
mod standby;
#[cfg(test)]
mod testutil;
pub mod toshiba;

pub use att::{AttControl, att_thresholds};
pub use channel::{Band, channel_hz, parse_channel};
pub use demod::{BerCount, CnrCurve, Demod};
pub use regs::{Bus, Delay, StdSleep, TunerPath};
pub use standby::{StandbyStep, run_standby, standby_steps};

pub use crate::model::KitKind;
use crate::model::kit_for_board;
use crate::{Error, Result};

/// The device a table entry or sequence step goes to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dev {
    /// Demodulator.
    D,
    /// Tuner (along the kit's [`TunerPath`]).
    T,
}

/// Wait after the tuner and demodulator are set before the TS output is
/// opened (ms).
const TUNE_SETTLE_MS: u32 = 30;

/// One reading of everything the demodulator reports.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SignalStatus {
    pub locked: bool,
    /// 24-bit C/N raw value.
    pub cnr_raw: u32,
    /// C/N (dB) ([`demod::cnr_db`]).
    pub cnr_db: f64,
    /// Layers A/B/C, whether in use or not (TMCC is not read).
    pub ber: [BerCount; 3],
    /// The kit's integer signal level, from the AGC value and the
    /// attenuator state (unit as the vendor reports it, roughly dB-like).
    pub level: i32,
    /// Attenuator state; `None` on TOSHIBA, which has none.
    pub attenuator: Option<bool>,
    /// Demodulator 0x81 bit 7 (TOSHIBA: and 0xB2 bit 1), read as the
    /// emergency-warning flag (inferred).
    pub ews: bool,
}

/// One front end: a kit on a bus and the state the vendor keeps with it.
#[derive(Debug)]
pub struct Frontend<B> {
    bus: B,
    kind: KitKind,
    demod: Demod,
    tuner: u8,
    /// Frequency of the last successful tune.
    freq: Option<u32>,
    mode: u32,
    /// Attenuator state as last set (the MJ110 kit only records it, see
    /// [`Frontend::set_attenuator`]).
    att: bool,
    /// The vendor's `ATTMode` setting (MJ110 only).
    att_mode: bool,
    /// MJ100: current value of tuner register 0xA8 (VHF toggle).
    vhf_a8: u8,
    initialized: bool,
}

impl<B: Bus> Frontend<B> {
    /// A front end of a known kit; nothing is written until [`Frontend::init`].
    pub fn new(bus: B, kind: KitKind) -> Self {
        Frontend {
            bus,
            kind,
            demod: Demod { addr: kind.demod_addr() },
            tuner: kind.tuner_addr(),
            freq: None,
            mode: 0,
            att: false,
            att_mode: false,
            vhf_a8: att::VHF_A8[0],
            initialized: false,
        }
    }

    /// The front end of a board ([`crate::model::kit_for_board`]).
    pub fn for_board(bus: B, board_id: u8, subsystem: (u16, u16)) -> Result<Self> {
        match kit_for_board(board_id, subsystem) {
            Some(kind) => Ok(Self::new(bus, kind)),
            None => Err(Error::Unsupported(format!("board ID {board_id:#04x}"))),
        }
    }

    pub fn kind(&self) -> KitKind {
        self.kind
    }

    pub fn bus(&mut self) -> &mut B {
        &mut self.bus
    }

    pub fn into_bus(self) -> B {
        self.bus
    }

    /// The frequency of the last successful tune.
    pub fn frequency(&self) -> Option<u32> {
        self.freq
    }

    /// Whether [`Frontend::init`] succeeded and no [`Frontend::sleep`] followed.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    pub fn mode(&self) -> u32 {
        self.mode
    }

    /// The vendor's `ATTMode` setting (off by default). Only the MJ110 kit
    /// uses it: its attenuator register is set to 0x2E when it is on and
    /// 0x2A when it is off, whatever [`Frontend::set_attenuator`] asks.
    /// That looks like a vendor bug but is reproduced as-is, since that is
    /// how the shipped product behaves (docs/26 4-tuning §5.4).
    pub fn set_att_mode(&mut self, on: bool) {
        self.att_mode = on;
    }

    /// Wake up and initialise: the kit's init table, the mode, and a re-tune
    /// to the remembered frequency, if any.
    pub fn init(&mut self) -> Result<()> {
        self.initialized = false;
        match self.kind {
            KitKind::Mj100 => {
                self.tw(0x09, 0xB1)?;
                self.bus.delay_ms(10);
                self.write_seq(mj100::INIT)?;
            }
            KitKind::Mj102 => self.write_seq(mj10x::MJ102_INIT)?,
            KitKind::Mj110 => self.write_seq(mj10x::MJ110_INIT)?,
            KitKind::Toshiba => self.write_seq(toshiba::INIT)?,
        }
        self.write_mode(self.mode)?;
        if let Some(f) = self.freq {
            self.tune_force(f)?;
        }
        self.vhf_a8 = att::VHF_A8[0];
        self.initialized = true;
        Ok(())
    }

    /// Tune to `hz`; does nothing when the last tune was to the same
    /// frequency (use [`Frontend::tune_force`] to repeat it).
    pub fn tune(&mut self, hz: u32) -> Result<()> {
        if self.freq == Some(hz) {
            return Ok(());
        }
        self.tune_force(hz)
    }

    /// Tune to `hz` unconditionally. Stops at the first failed transfer up
    /// to the demodulator resync; like the vendor, the failure of the last
    /// writes (0x76 and the TS output) is only logged.
    pub fn tune_force(&mut self, hz: u32) -> Result<()> {
        match self.kind {
            KitKind::Mj100 => mj100::tune(self, hz),
            KitKind::Mj102 | KitKind::Mj110 => mj10x::tune(self, hz),
            KitKind::Toshiba => toshiba::tune(self, hz),
        }
    }

    /// Put the kit into standby; [`Frontend::init`] wakes it up. The MJ100
    /// kit forgets its frequency here (as the vendor's does), the others
    /// re-tune to it on wake-up.
    pub fn sleep(&mut self) -> Result<()> {
        self.initialized = false;
        match self.kind {
            KitKind::Mj100 => {
                self.write_seq(mj100::SLEEP)?;
                self.freq = None;
            }
            KitKind::Mj102 => self.write_seq(mj10x::MJ102_SLEEP)?,
            KitKind::Mj110 => self.write_seq(mj10x::MJ110_SLEEP)?,
            KitKind::Toshiba => self.write_seq(toshiba::SLEEP)?,
        }
        Ok(())
    }

    /// Set the mode the vendor's private interface exposes (meaning
    /// unknown; 0 by default). MJ100/MJ102: demodulator 0x52 = 0x62 for
    /// modes 1 and 2, else 0x20. MJ110: nothing. TOSHIBA: 0x02 = 0x80 for
    /// mode 2, 0x00 otherwise; mode 1 is refused.
    pub fn set_mode(&mut self, mode: u32) -> Result<()> {
        self.write_mode(mode)?;
        self.mode = mode;
        Ok(())
    }

    /// Switch the attenuator. MJ100: tuner 0x95 = 0x03 (on) / 0xFB (off).
    /// MJ102: demodulator 0x1C = 0x3A / 0x2A. MJ110: see
    /// [`Frontend::set_att_mode`]; the state is recorded for the level
    /// formula only. TOSHIBA has no attenuator.
    pub fn set_attenuator(&mut self, on: bool) -> Result<()> {
        match self.kind {
            KitKind::Mj100 => self.tw(0x95, if on { 0x03 } else { 0xFB })?,
            KitKind::Mj102 => self.dw(0x1C, if on { 0x3A } else { 0x2A })?,
            KitKind::Mj110 => self.dw(0x1C, if self.att_mode { 0x2E } else { 0x2A })?,
            KitKind::Toshiba => return Err(Error::Unsupported("the TOSHIBA kit has no attenuator".into())),
        }
        self.att = on;
        Ok(())
    }

    /// The attenuator state; `None` on TOSHIBA.
    pub fn attenuator(&self) -> Option<bool> {
        (self.kind != KitKind::Toshiba).then_some(self.att)
    }

    pub fn is_locked(&mut self) -> Result<bool> {
        self.demod.locked(&mut self.bus)
    }

    pub fn cnr_raw(&mut self) -> Result<u32> {
        self.demod.cnr_raw(&mut self.bus)
    }

    /// C/N (dB) with the kit's conversion.
    pub fn cnr_db(&mut self) -> Result<f64> {
        let raw = self.cnr_raw()?;
        Ok(demod::cnr_db(raw, self.cnr_curve()))
    }

    /// Error counts and periods of layers A/B/C.
    pub fn ber(&mut self) -> Result<[BerCount; 3]> {
        let periods =
            if self.kind == KitKind::Toshiba { demod::BerPeriods::PerLayer } else { demod::BerPeriods::Shared };
        self.demod.ber_counts(&mut self.bus, periods)
    }

    /// The raw AGC value (demodulator 0x82), the attenuator control's input.
    pub fn agc(&mut self) -> Result<u8> {
        self.demod.agc(&mut self.bus)
    }

    /// The kit's signal level.
    pub fn signal_level(&mut self) -> Result<i32> {
        Ok(match self.kind {
            KitKind::Mj100 => mj100::level(self.agc()?, self.att),
            KitKind::Mj102 | KitKind::Mj110 => mj10x::level(self.kind, self.agc()?, self.att),
            KitKind::Toshiba => {
                let mut b = [0u8; 2];
                regs::read_regs(&mut self.bus, self.demod.addr, demod::reg::AGC, &mut b)?;
                toshiba::level(u32::from(b[0]) + u32::from(b[1]))
            }
        })
    }

    /// The emergency-warning flag (inferred): demodulator 0x81 bit 7, and
    /// on TOSHIBA also 0xB2 bit 1.
    pub fn ews(&mut self) -> Result<bool> {
        let r81 = self.demod.read(&mut self.bus, demod::reg::FLAGS_81)?;
        if self.kind != KitKind::Toshiba {
            return Ok(r81 & 0x80 != 0);
        }
        let rb2 = self.demod.read(&mut self.bus, demod::reg::FLAGS_B2)?;
        Ok(r81 & 0x80 != 0 && rb2 & 0x02 != 0)
    }

    /// Lock, C/N, BER, level and flags in one go.
    pub fn status(&mut self) -> Result<SignalStatus> {
        let locked = self.is_locked()?;
        let cnr_raw = self.cnr_raw()?;
        let ber = self.ber()?;
        let level = self.signal_level()?;
        let ews = self.ews()?;
        Ok(SignalStatus {
            locked,
            cnr_raw,
            cnr_db: demod::cnr_db(cnr_raw, self.cnr_curve()),
            ber,
            level,
            attenuator: self.attenuator(),
            ews,
        })
    }

    /// Poll the lock every `poll_ms` for up to `timeout_ms`. The vendor
    /// never waits for lock after a tune; this is an addition.
    pub fn wait_lock(&mut self, timeout_ms: u32, poll_ms: u32) -> Result<bool> {
        let poll_ms = poll_ms.max(1);
        let mut waited = 0;
        loop {
            if self.is_locked()? {
                return Ok(true);
            }
            if waited >= timeout_ms {
                return Ok(false);
            }
            self.bus.delay_ms(poll_ms);
            waited += poll_ms;
        }
    }

    /// One step of the attenuator control (the vendor runs it every
    /// [`att::ATT_POLL_MS`]): read the AGC, feed `ctl`, switch the
    /// attenuator when it decides so. Returns the new state when it changed.
    /// Does nothing on TOSHIBA.
    pub fn att_tick(&mut self, ctl: &mut AttControl) -> Result<Option<bool>> {
        if self.kind == KitKind::Toshiba {
            return Ok(None);
        }
        let agc = self.agc()?;
        match ctl.push(agc, self.att) {
            Some(on) => {
                self.set_attenuator(on)?;
                Ok(Some(on))
            }
            None => Ok(None),
        }
    }

    /// One step of the MJ100 VHF toggle (the vendor runs it every
    /// [`att::VHF_TOGGLE_MS`] while initialised): at or below 170 MHz,
    /// when demodulator 0x94 bit 7 is set, flip tuner 0xA8 between 0xD2 and
    /// 0xA2. Returns the value written. Does nothing on other kits.
    pub fn vhf_toggle_tick(&mut self) -> Result<Option<u8>> {
        if self.kind != KitKind::Mj100 || !self.initialized {
            return Ok(None);
        }
        if !matches!(self.freq, Some(f) if f <= att::VHF_TOGGLE_MAX_HZ) {
            return Ok(None);
        }
        let r94 = self.demod.read(&mut self.bus, demod::reg::STATUS_94)?;
        match att::vhf_toggle(self.freq, r94, self.vhf_a8) {
            Some(v) => {
                self.tw(0xA8, v)?;
                self.vhf_a8 = v;
                Ok(Some(v))
            }
            None => Ok(None),
        }
    }

    fn cnr_curve(&self) -> CnrCurve {
        match self.kind {
            KitKind::Mj100 | KitKind::Toshiba => CnrCurve::Plain,
            KitKind::Mj102 | KitKind::Mj110 => CnrCurve::Clamped,
        }
    }

    fn tuner_path(&self) -> TunerPath {
        match self.kind {
            KitKind::Mj100 => TunerPath::DirectTrailingFe,
            KitKind::Mj102 | KitKind::Mj110 => TunerPath::ViaDemod,
            KitKind::Toshiba => TunerPath::Direct,
        }
    }

    /// Write one demodulator register.
    fn dw(&mut self, reg: u8, value: u8) -> Result<()> {
        self.demod.write(&mut self.bus, reg, value)
    }

    /// Write one tuner register.
    fn tw(&mut self, reg: u8, value: u8) -> Result<()> {
        self.tw_n(reg, &[value])
    }

    /// Write consecutive tuner registers.
    fn tw_n(&mut self, reg: u8, data: &[u8]) -> Result<()> {
        let path = self.tuner_path();
        regs::write_tuner(&mut self.bus, path, self.demod.addr, self.tuner, reg, data)
    }

    /// Write a table or sequence, one register per transfer, stopping at
    /// the first failure.
    fn write_seq(&mut self, seq: &[(Dev, u8, u8)]) -> Result<()> {
        for &(dev, reg, value) in seq {
            match dev {
                Dev::D => self.dw(reg, value)?,
                Dev::T => self.tw(reg, value)?,
            }
        }
        Ok(())
    }

    fn write_mode(&mut self, mode: u32) -> Result<()> {
        match self.kind {
            KitKind::Mj100 | KitKind::Mj102 => self.dw(0x52, if mode == 1 || mode == 2 { 0x62 } else { 0x20 }),
            KitKind::Mj110 => Ok(()),
            KitKind::Toshiba => match mode {
                1 => Err(Error::Unsupported("mode 1 on the TOSHIBA kit".into())),
                2 => self.dw(0x02, 0x80),
                _ => self.dw(0x02, 0x00),
            },
        }
    }

    /// The end every tune shares, after the demodulator resync (0x01): 0x76,
    /// 30 ms, TS output on, remember the frequency. Write failures here are
    /// logged and otherwise ignored, as in the vendor code.
    fn finish_tune(&mut self, hz: u32, reg76: u8) {
        if let Err(e) = self.dw(0x76, reg76) {
            log::warn!("{}: demodulator 0x76 after tune: {e}", self.kind.name());
        }
        self.bus.delay_ms(TUNE_SETTLE_MS);
        if let Err(e) = self.demod.set_ts_output(&mut self.bus, true) {
            log::warn!("{}: TS output on after tune: {e}", self.kind.name());
        }
        self.freq = Some(hz);
    }
}

#[cfg(test)]
mod tests {
    use super::testutil::{Ev, Rig, Seq};
    use super::*;

    #[test]
    fn for_board_picks_the_kit() {
        assert_eq!(Frontend::for_board(Rig::new(), 0xB4, (0, 0)).unwrap().kind(), KitKind::Toshiba);
        assert_eq!(Frontend::for_board(Rig::new(), 0xB2, (0xEEEE, 0xEEEE)).unwrap().kind(), KitKind::Mj102);
        assert_eq!(Frontend::for_board(Rig::new(), 0xB2, (0, 0)).unwrap().kind(), KitKind::Mj110);
        assert!(matches!(Frontend::for_board(Rig::new(), 0x12, (0, 0)), Err(Error::Unsupported(_))));
    }

    #[test]
    fn tune_to_the_same_frequency_is_skipped() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Mj102);
        fe.tune(473_142_857).unwrap();
        let n = fe.bus().ev.len();
        fe.tune(473_142_857).unwrap();
        assert_eq!(fe.bus().ev.len(), n);
        fe.tune_force(473_142_857).unwrap();
        assert_eq!(fe.bus().ev.len(), 2 * n);
    }

    #[test]
    fn wait_lock_polls() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Toshiba);
        fe.bus().mock.queue(0x20, 0xB0, &[0x02, 0x05]);
        fe.bus().mock.set(0x20, 0xB0, 0x08);
        assert!(fe.wait_lock(500, 100).unwrap());
        let want = Seq::new()
            .read(0x20, 0x80, 1)
            .read(0x20, 0xB0, 1)
            .delay(100)
            .read(0x20, 0x80, 1)
            .read(0x20, 0xB0, 1)
            .delay(100)
            .read(0x20, 0x80, 1)
            .read(0x20, 0xB0, 1);
        assert_eq!(fe.bus().ev, want.0);

        let mut fe = Frontend::new(Rig::new(), KitKind::Toshiba);
        assert!(!fe.wait_lock(250, 100).unwrap());
        assert_eq!(fe.bus().ev.iter().filter(|e| **e == Ev::D(100)).count(), 3);
    }

    #[test]
    fn status_reads_everything() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Mj110);
        let m = &mut fe.bus().mock;
        m.set(0x30, 0xB0, 0x09);
        for (i, v) in [0x00u8, 0x27, 0x10].into_iter().enumerate() {
            m.set(0x30, 0x8B + i as u8, v); // 10000
        }
        for (i, v) in [0, 0, 0x10, 0, 0, 0, 0, 0, 0, 0, 0x20].into_iter().enumerate() {
            m.set(0x30, 0x9D + i as u8, v);
        }
        m.set(0x30, 0x82, 100);
        m.set(0x30, 0x81, 0x80);
        let s = fe.status().unwrap();
        assert!(s.locked);
        assert_eq!(s.cnr_raw, 10_000);
        assert!((s.cnr_db - 28.640137945177116).abs() < 1e-9);
        assert_eq!(s.ber[0], BerCount { errors: 0x10, period: 0x20 });
        assert_eq!(s.ber[2], BerCount { errors: 0, period: 0x20 });
        assert_eq!(s.level, (78_000 - 575 * 100) / 1000);
        assert_eq!(s.attenuator, Some(false));
        assert!(s.ews);
        let want = Seq::new()
            .read(0x30, 0x80, 1)
            .read(0x30, 0xB0, 1)
            .read(0x30, 0x8B, 3)
            .read(0x30, 0x9D, 11)
            .read(0x30, 0x82, 1)
            .read(0x30, 0x81, 1);
        assert_eq!(fe.bus().ev, want.0);
    }

    #[test]
    fn att_tick_drives_the_attenuator() {
        let mut fe = Frontend::new(Rig::new(), KitKind::Mj102);
        let mut ctl = AttControl::for_kit(KitKind::Mj102).unwrap();
        fe.bus().mock.set(0x20, 0x82, 200);
        for _ in 0..9 {
            assert_eq!(fe.att_tick(&mut ctl).unwrap(), None);
        }
        assert_eq!(fe.att_tick(&mut ctl).unwrap(), Some(true));
        assert_eq!(fe.attenuator(), Some(true));
        assert_eq!(fe.bus().ev.last(), Some(&Ev::W(0x20, vec![0x1C, 0x3A])));
        fe.bus().mock.set(0x20, 0x82, 10);
        for _ in 0..9 {
            fe.att_tick(&mut ctl).unwrap();
        }
        assert_eq!(fe.att_tick(&mut ctl).unwrap(), Some(false));
        assert_eq!(fe.bus().ev.last(), Some(&Ev::W(0x20, vec![0x1C, 0x2A])));

        let mut fe = Frontend::new(Rig::new(), KitKind::Toshiba);
        assert_eq!(fe.att_tick(&mut ctl).unwrap(), None);
        assert!(fe.bus().ev.is_empty());
        assert!(matches!(fe.set_attenuator(true), Err(Error::Unsupported(_))));
        assert_eq!(fe.attenuator(), None);
    }
}
