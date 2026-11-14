//! One DT-H30/U2-family device: the ViXS chip, the front end of its board's
//! kit and the three streams of the vendor's filter.
//!
//! [`Dth30::bring_up`] (or [`Dth30::start`] on a chip that is already up)
//! follows the vendor's start-up: chip and firmware ([`Vixs::bring_up`]),
//! the kernel driver's front-end standby writes, the TS input set-up
//! (docs/26 6-stream-payloads.md §5.1, sent while no stream is open) and the
//! front-end initialisation of the user-mode filter. Then [`Dth30::tune`],
//! and the streams in the filter's order (5-usermode.md §3.3, 6-stream-
//! payloads.md §9):
//!
//! 1. [`Dth30::open_cas`]: the CAS object; read its endpoint continuously
//!    ([`StreamInfo`], blocks with a header, see
//!    [`crate::vixs::stream::parse_generic_block`]);
//! 2. [`Dth30::open_raw`]: the Raw stream (EP 0x83); find the service in it
//!    with [`crate::psi::ProgramScanner`];
//! 3. [`Dth30::open_program`]: the Program stream of that service (EP 0x84).
//!
//! A channel change closes Program and Raw, tunes, and opens Raw and
//! Program again; the CAS object stays open. Stopping closes Program, Raw,
//! then the CAS object.
//!
//! This crate does no USB I/O beyond [`RemotePci`]: the caller reads the
//! endpoints each open returns. Keep reading an endpoint while its stream
//! is closed (the firmware drains it during the flush), then cancel the IN
//! transfers and call [`Dth30::flush_endpoint_fifo`].
//!
//! With [`CasMode::Bcas`] (the default) the Program stream comes out in the
//! clear once the card is activated. Checked on a DT-H50/U2 (board 0xB4,
//! TOSHIBA kit); the other boards are unverified.

use core::time::Duration;

use crate::bus::RemotePci;
use crate::frontend::{Delay, Frontend, SignalStatus, StandbyStep, standby_steps};
use crate::i2c::I2c;
use crate::model::{self, BoardClass, KitKind};
use crate::psi::ProgramSpec;
use crate::vixs::stream::{CasMode, ProgramParams, StreamHandle, StreamKind, TsdConfig};
use crate::vixs::{self, Firmware, Vixs, gpio};
use crate::{Error, Result};

/// I2C clock of the kernel driver's standby writes (kHz).
pub const STANDBY_KHZ: u32 = 100;
/// I2C clock the vendor's user-mode filter uses for the front end (kHz).
pub const FRONTEND_KHZ: u32 = 400;
/// GPIO output bit that selects the front end on the dual TOSHIBA board.
pub const DUAL_SELECT_GPIO: u32 = 0x4;

/// Device settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// The two subsystem ID words the vendor's filter reads with the board
    /// information; only board 0xB2 uses them to pick its kit
    /// ([`model::kit_for_board`]). Where they come from on this hardware is
    /// not known (docs/26 4-tuning.md §2.1), so they are given here; `None`
    /// counts as "other" (the MJ110 kit on board 0xB2).
    pub subsystem: Option<(u16, u16)>,
    /// The CAS set-up used by [`Dth30::open_cas`] and
    /// [`Dth30::open_program`].
    pub cas_mode: CasMode,
    /// TS port of the Raw and Program streams (0 on single-tuner boards).
    pub ts_port: u32,
    /// The TS input port of the second front end on a dual board (1: the
    /// second kit is taken to feed port 1; inferred).
    pub ts_port_second: u32,
    /// I2C channel and clock of the front end.
    pub i2c_channel: u8,
    pub i2c_khz: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            subsystem: None,
            cas_mode: CasMode::default(),
            ts_port: 0,
            ts_port_second: 1,
            i2c_channel: 0,
            i2c_khz: FRONTEND_KHZ,
        }
    }
}

/// The front end's bus: the ViXS chip's I2C master, owned together with the
/// chip so that the front end keeps its state between calls.
pub struct FeBus<B: RemotePci> {
    vixs: Vixs<B>,
    channel: u8,
    khz: u32,
}

impl<B: RemotePci> FeBus<B> {
    pub fn vixs(&self) -> &Vixs<B> {
        &self.vixs
    }

    pub fn vixs_mut(&mut self) -> &mut Vixs<B> {
        &mut self.vixs
    }
}

impl<B: RemotePci> I2c for FeBus<B> {
    fn write(&mut self, addr8: u8, data: &[u8]) -> Result<()> {
        self.vixs.i2c(self.channel, self.khz).write(addr8, data)
    }

    fn write_read(&mut self, addr8: u8, wr: &[u8], rd: &mut [u8]) -> Result<()> {
        self.vixs.i2c(self.channel, self.khz).write_read(addr8, wr, rd)
    }
}

impl<B: RemotePci> Delay for FeBus<B> {
    fn delay_ms(&mut self, ms: u32) {
        self.vixs.chip().sleep(Duration::from_millis(ms.into()))
    }
}

/// Where an open stream's data comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StreamInfo {
    pub kind: StreamKind,
    /// The bulk IN endpoint to read.
    pub endpoint: u8,
    /// The vendor's transfer length on it.
    pub transfer_len: usize,
    /// Header bytes at the start of each block (generic channel only).
    pub header_len: usize,
    /// The firmware's handle of the stream.
    pub phys: u32,
}

impl StreamInfo {
    fn of(h: &StreamHandle, endpoint: u8) -> Self {
        let kind = h.kind();
        Self { kind, endpoint, transfer_len: kind.transfer_len(), header_len: kind.header_len(), phys: h.phys() }
    }
}

/// A DT-H30/U2-family device (see the module documentation).
pub struct Dth30<B: RemotePci> {
    fe: Frontend<FeBus<B>>,
    /// The front end in use (0, or 1 on a dual board).
    tuner: usize,
    board_id: u32,
    kit: KitKind,
    cfg: Config,
    cas: Option<StreamHandle>,
    raw: Option<StreamHandle>,
    program: Option<StreamHandle>,
}

impl<B: RemotePci> Dth30<B> {
    /// Bring up the NET2280, read the board ID and pick the kit
    /// ([`Vixs::open`]); nothing else is started.
    pub fn open(bus: B, pid: u16, pipes: &[u8], opts: vixs::Options, cfg: Config) -> Result<Self> {
        Self::new(Vixs::open(bus, pid, pipes, opts)?, cfg)
    }

    /// The device around an opened (or already brought-up) chip; fails for
    /// a board without a known kit.
    pub fn new(vixs: Vixs<B>, cfg: Config) -> Result<Self> {
        let board_id = vixs.board_id();
        let board = model::board_id(board_id);
        if board == 0xB2 && cfg.subsystem.is_none() {
            log::warn!("board 0xB2 without subsystem IDs: assuming the MJ110 kit");
        }
        let kit = model::kit_for_board(board, cfg.subsystem.unwrap_or((0, 0)))
            .ok_or_else(|| Error::Unsupported(format!("board ID {board:#04x}")))?;
        log::info!("board {board:#04x}: {} front end", kit.name());
        let bus = FeBus { vixs, channel: cfg.i2c_channel, khz: cfg.i2c_khz };
        Ok(Self { fe: Frontend::new(bus, kit), tuner: 0, board_id, kit, cfg, cas: None, raw: None, program: None })
    }

    /// Start the chip and the firmware ([`Vixs::bring_up`]), then
    /// [`Dth30::start`].
    pub fn bring_up(&mut self, fw: &Firmware) -> Result<()> {
        self.vixs_mut().bring_up(fw)?;
        self.start()
    }

    /// On a chip whose firmware runs: the front-end standby
    /// ([`Dth30::standby`], a failure is only logged as the writes may not
    /// match the board's kit), the TS input set-up and the front-end
    /// initialisation.
    pub fn start(&mut self) -> Result<()> {
        if let Err(e) = self.standby() {
            log::warn!("front-end standby: {e}");
        }
        self.tsd_config()?;
        self.fe.init()
    }

    /// The kernel driver's start-up standby writes for the board class
    /// (channel 0, 100 kHz, stopping at the first failure). On the dual
    /// TOSHIBA board the select GPIO is put back to the first front end
    /// afterwards (inferred: the kernel leaves the second one selected).
    pub fn standby(&mut self) -> Result<()> {
        let class = model::board_class(model::board_id(self.board_id));
        let v = self.vixs_mut();
        let r = run_standby(v, standby_steps(class));
        if class == BoardClass::DualToshiba {
            gpio::write_bits(v.chip(), DUAL_SELECT_GPIO, false)?;
        }
        r
    }

    /// Show the unit as in use (the vendor application's "watching") or
    /// idle ("not using") on its lamps, as the application does through
    /// the driver while it views or records. Only boards 0xB4 and 0xC6
    /// have the lamps; on a DT-H50/U2 in use is power blue and status red,
    /// idle is power red (standby) and status green. Bring-up leaves the
    /// unit idle.
    pub fn set_in_use(&mut self, in_use: bool) -> Result<()> {
        let s = if in_use { gpio::app_status::WATCHING } else { gpio::app_status::NOT_USING };
        self.vixs_mut().set_app_status(s)
    }

    /// The front ends this crate drives on the board: 2 on a dual board
    /// with a known second front end ([`model::second_front_end`]), else 1.
    pub fn tuner_count(&self) -> usize {
        if model::second_front_end(self.board()).is_some() { 2 } else { 1 }
    }

    /// The front end in use.
    pub fn tuner(&self) -> usize {
        self.tuner
    }

    /// Switch to front end `tuner` (one at a time, as the vendor stack: one
    /// Raw and one Program stream per device). Refused while the Raw or
    /// Program stream is open. Both front ends go through the board's
    /// standby (which also sets the first tuner's loop-through, the second
    /// kit's RF input, inferred), then the chosen one is initialised; the
    /// Raw and Program streams then use its TS port.
    pub fn select_tuner(&mut self, tuner: usize) -> Result<()> {
        if tuner == self.tuner {
            return Ok(());
        }
        if tuner >= self.tuner_count() {
            return Err(Error::Unsupported(format!("no front end #{tuner} on this board")));
        }
        if self.raw.is_some() || self.program.is_some() {
            return Err(Error::Protocol("close the Raw and Program streams before switching front ends"));
        }
        if let Err(e) = self.standby() {
            log::warn!("front-end standby: {e}");
        }
        let (demod, tuner_addr) = match tuner {
            0 => (self.kit.demod_addr(), self.kit.tuner_addr()),
            _ => model::second_front_end(self.board()).ok_or(Error::Unsupported("no second front end".into()))?,
        };
        self.fe.set_addresses(demod, tuner_addr);
        self.tuner = tuner;
        log::info!("front end #{tuner}: demodulator 0x{demod:02x}, tuner 0x{tuner_addr:02x}");
        self.fe.init()
    }

    /// The TS input port of the front end in use.
    fn ts_port(&self) -> u32 {
        if self.tuner == 0 { self.cfg.ts_port } else { self.cfg.ts_port_second }
    }

    /// The TS input set-up for the board's kit. Refused while a stream is
    /// open (the firmware would answer status 30).
    pub fn tsd_config(&mut self) -> Result<()> {
        if self.any_open() {
            return Err(Error::Protocol("the TS input set-up needs all streams closed"));
        }
        let c = TsdConfig::for_kit(self.kit);
        self.vixs_mut().tsd_config(&c)
    }

    /// Tune the front end to `hz` (channel centre + 1/7 MHz, see
    /// [`crate::frontend::channel_hz`]). On a channel change close Program
    /// and Raw first and open them again afterwards, as the vendor does.
    pub fn tune(&mut self, hz: u32) -> Result<()> {
        self.fe.tune(hz)
    }

    /// Lock, C/N, BER and level.
    pub fn status(&mut self) -> Result<SignalStatus> {
        self.fe.status()
    }

    /// Open the CAS object with [`Config::cas_mode`] and ask for a card
    /// reset ([`Vixs::reset_card`]). Read the returned endpoint
    /// continuously while it is open, and wait for a block with
    /// [`card_state::ACTIVATED`](crate::vixs::stream::card_state) before
    /// expecting a descrambled Program stream.
    pub fn open_cas(&mut self) -> Result<StreamInfo> {
        if self.cas.is_some() {
            return Err(Error::Protocol("the CAS object is already open"));
        }
        let mode = self.cfg.cas_mode;
        let h = self.vixs_mut().open_generic(mode)?;
        let info = self.opened(h.clone());
        self.vixs_mut().reset_card(&h)?;
        Ok(info)
    }

    /// Ask for a card reset again on the open CAS object: the firmware
    /// does not reset a card that is put back after a removal (its block
    /// reports it inserted but not activated, and the TS stays scrambled).
    pub fn reset_card(&mut self) -> Result<()> {
        let h = self.cas.clone().ok_or(Error::Protocol("the CAS object is not open"))?;
        self.vixs_mut().reset_card(&h)
    }

    /// Open the Raw stream (PSI / TS pass-through) on [`Config::ts_port`].
    pub fn open_raw(&mut self) -> Result<StreamInfo> {
        if self.raw.is_some() {
            return Err(Error::Protocol("the Raw stream is already open"));
        }
        let port = self.ts_port();
        let h = self.vixs_mut().open_psi(port)?;
        Ok(self.opened(h))
    }

    /// Open the Program stream of a service, with the CAS object's and the
    /// Raw stream's handles when they are open (the vendor always has
    /// both) and the secured mode of [`Config::cas_mode`].
    pub fn open_program(&mut self, spec: &ProgramSpec) -> Result<StreamInfo> {
        if self.program.is_some() {
            return Err(Error::Protocol("the Program stream is already open"));
        }
        let cas = self.cas.as_ref().map_or_default(StreamHandle::phys);
        let raw = self.raw.as_ref().map_or_default(StreamHandle::phys);
        if self.cas.is_none() || self.raw.is_none() {
            log::warn!("Program open without the CAS object or the Raw stream open");
        }
        let es = spec.es_entries();
        let (port, mode) = (self.ts_port(), self.cfg.cas_mode);
        let p = ProgramParams::new(port, cas, raw, spec.sid, spec.pmt_pid, spec.pcr_pid, &es, mode);
        let h = self.vixs_mut().open_program(&p)?;
        Ok(self.opened(h))
    }

    fn opened(&mut self, h: StreamHandle) -> StreamInfo {
        let ep = self.vixs_mut().stream_endpoint(h.kind());
        let info = StreamInfo::of(&h, ep);
        let kind = h.kind();
        *self.slot_mut(kind) = Some(h);
        info
    }

    fn slot_mut(&mut self, kind: StreamKind) -> &mut Option<StreamHandle> {
        match kind {
            StreamKind::Generic => &mut self.cas,
            StreamKind::Raw => &mut self.raw,
            StreamKind::Program => &mut self.program,
        }
    }

    /// Flush and close a stream (nothing when it is not open); the handle
    /// is kept when the firmware refuses, so the close can be repeated.
    fn close(&mut self, kind: StreamKind) -> Result<()> {
        let Some(h) = self.slot_mut(kind).take() else { return Ok(()) };
        match self.vixs_mut().stop_stream(&h) {
            Ok(()) => Ok(()),
            Err(e) => {
                *self.slot_mut(kind) = Some(h);
                Err(e)
            }
        }
    }

    /// Flush and close the Program stream; then cancel the reads on its
    /// endpoint and call [`Dth30::flush_endpoint_fifo`].
    pub fn close_program(&mut self) -> Result<()> {
        self.close(StreamKind::Program)
    }

    /// Flush and close the Raw stream (as [`Dth30::close_program`]).
    pub fn close_raw(&mut self) -> Result<()> {
        self.close(StreamKind::Raw)
    }

    /// Close the CAS object (no flush); then cancel the reads on its
    /// endpoint and call [`Dth30::flush_endpoint_fifo`].
    pub fn close_cas(&mut self) -> Result<()> {
        self.close(StreamKind::Generic)
    }

    /// Close Program, Raw and the CAS object in that order, going on after
    /// a failure; returns the first error.
    pub fn close_all(&mut self) -> Result<()> {
        let mut first = Ok(());
        for kind in [StreamKind::Program, StreamKind::Raw, StreamKind::Generic] {
            if let Err(e) = self.close(kind)
                && first.is_ok()
            {
                first = Err(e);
            }
        }
        first
    }

    /// Empty the NET2280 FIFO of `ep` once its IN transfers are cancelled
    /// ([`crate::vixs::stream::flush_endpoint_fifo`]).
    pub fn flush_endpoint_fifo(&mut self, ep: u8) -> Result<bool> {
        self.vixs_mut().flush_endpoint_fifo(ep)
    }

    /// Use another CAS set-up for the next opens; refused while the CAS
    /// object or the Program stream is open.
    pub fn set_cas_mode(&mut self, mode: CasMode) -> Result<()> {
        if self.cas.is_some() || self.program.is_some() {
            return Err(Error::Protocol("close the CAS object and the Program stream first"));
        }
        self.cfg.cas_mode = mode;
        Ok(())
    }

    /// The open stream of a kind.
    pub fn stream(&self, kind: StreamKind) -> Option<&StreamHandle> {
        match kind {
            StreamKind::Generic => self.cas.as_ref(),
            StreamKind::Raw => self.raw.as_ref(),
            StreamKind::Program => self.program.as_ref(),
        }
    }

    fn any_open(&self) -> bool {
        self.cas.is_some() || self.raw.is_some() || self.program.is_some()
    }

    pub fn kit(&self) -> KitKind {
        self.kit
    }

    /// The board-ID register as read when the device was opened.
    pub fn board_id(&self) -> u32 {
        self.board_id
    }

    /// The board type (low byte of the board ID).
    pub fn board(&self) -> u8 {
        model::board_id(self.board_id)
    }

    pub fn config(&self) -> &Config {
        &self.cfg
    }

    pub fn vixs_mut(&mut self) -> &mut Vixs<B> {
        self.fe.bus().vixs_mut()
    }

    /// The front end, for what [`Dth30`] does not wrap (attenuator, sleep,
    /// lock wait...).
    pub fn frontend_mut(&mut self) -> &mut Frontend<FeBus<B>> {
        &mut self.fe
    }

    /// Give the chip back (open streams are not closed).
    pub fn into_vixs(self) -> Vixs<B> {
        self.fe.into_bus().vixs
    }
}

/// Run a standby sequence on channel 0 at [`STANDBY_KHZ`], stopping at the
/// first failure (as [`crate::frontend::run_standby`], with the GPIO on the
/// same chip).
fn run_standby<B: RemotePci>(v: &mut Vixs<B>, steps: &[StandbyStep]) -> Result<()> {
    for step in steps {
        match *step {
            StandbyStep::Write(addr, data) => v.i2c(0, STANDBY_KHZ).write(addr, data)?,
            StandbyStep::GpioBit2(on) => gpio::write_bits(v.chip(), DUAL_SELECT_GPIO, on)?,
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::mock::{MockPci, Space};
    use crate::psi::EsSpec;
    use crate::vixs::cmd::{Completion, MAX_SLOTS};
    use crate::vixs::i2c::{CMD_ERROR, ctrl_word};
    use crate::vixs::mmr::testutil::*;
    use crate::vixs::stream::tests::{script_cmds, sent_payload};
    use crate::vixs::stream::{build_card_reset, build_open_generic, build_open_psi};
    use crate::vixs::tests::{layout, opts};

    const T100: u32 = 415 | 100 << 16;
    const T400: u32 = 103 | 100 << 16;

    /// A device on board `board` whose firmware runs and whose I2C
    /// transfers all succeed.
    fn device(board: u8, cfg: Config) -> Dth30<MockPci> {
        let mut v = Vixs::with_chip(chip(0x0300_0000 | u32::from(board)), layout(), opts(Completion::Poll));
        v.ring = Some(vixs::cmd::CmdRing::new(0x10_0840, vixs::mips::COM_LIST_SIZE).unwrap());
        v.alive = true;
        v.bus().set(Space::Mem, MMR + 0x1094, 0xC0);
        v.bus().set(Space::Mem, MMR + 0x1090, 0x60);
        Dth30::new(v, cfg).unwrap()
    }

    fn bus(d: &mut Dth30<MockPci>) -> &MockPci {
        d.vixs_mut().bus()
    }

    fn pos(acc: &[Acc], a: Acc) -> usize {
        acc.iter().position(|x| *x == a).unwrap_or_else(|| panic!("{a:?} missing"))
    }

    fn w(b: &[u8], off: usize) -> u32 {
        u32::from_le_bytes(b[off..off + 4].try_into().unwrap())
    }

    #[test]
    fn kit_selection() {
        assert_eq!(device(0xB4, Config::default()).kit(), KitKind::Toshiba);
        assert_eq!(device(0xB2, Config::default()).kit(), KitKind::Mj110);
        let cfg = Config { subsystem: Some(model::SUBSYSTEM_MJ102), ..Config::default() };
        let d = device(0xB2, cfg);
        assert_eq!((d.kit(), d.board(), d.board_id()), (KitKind::Mj102, 0xB2, 0x0300_00B2));
        assert_eq!(d.config().i2c_khz, 400);
        let v = Vixs::with_chip(chip(0x0300_0012), layout(), opts(Completion::Poll));
        std::assert_matches!(Dth30::new(v, Config::default()).err(), Some(Error::Unsupported(_)));
    }

    #[test]
    fn second_front_end_of_the_dual_board() {
        let mut d = device(0xC6, Config::default());
        assert_eq!((d.tuner_count(), d.tuner()), (2, 0));
        assert_eq!(d.frontend_mut().addresses(), (0x30, 0xC6));
        bus(&mut d).clear();
        d.select_tuner(1).unwrap();
        assert_eq!((d.tuner(), d.frontend_mut().addresses()), (1, (0x3C, 0xC0)));
        // Standby on both kits (100 kHz), then the second kit's init.
        let acc = accesses(&bus(&mut d).ops());
        let standby_second = pos(&acc, Acc::W(0x100C, ctrl_word(4, true, true, true, 0x3C)));
        assert!(
            acc.iter().rposition(|a| *a == Acc::W(0x100C, ctrl_word(4, true, true, true, 0x3C))).unwrap()
                > standby_second
        );
        // The Raw stream of front end #1 reads TS port 1.
        script_cmds(d.vixs_mut(), &[(8, 0, 1)]);
        bus(&mut d).clear();
        d.open_raw().unwrap();
        assert_eq!(w(&sent_payload(bus(&mut d), 0), 0x08), 1);
        // No switch while a stream is open; back to #0 once it is closed.
        assert!(d.select_tuner(0).is_err());
        script_cmds(d.vixs_mut(), &[(10, 1, 1), (9, 0, 1)]);
        d.close_raw().unwrap();
        d.select_tuner(0).unwrap();
        assert_eq!((d.tuner(), d.frontend_mut().addresses()), (0, (0x30, 0xC6)));
        assert!(d.select_tuner(2).is_err());
    }

    #[test]
    fn single_front_end_boards() {
        let mut d = device(0xB4, Config::default());
        assert_eq!(d.tuner_count(), 1);
        d.select_tuner(0).unwrap();
        std::assert_matches!(d.select_tuner(1), Err(Error::Unsupported(_)));
    }

    #[test]
    fn start_order() {
        let mut d = device(0xB4, Config::default());
        script_cmds(d.vixs_mut(), &[(24, 0, 1)]);
        bus(&mut d).clear();
        d.start().unwrap();
        let acc = accesses(&bus(&mut d).ops());
        // Standby at 100 kHz (TOSHIBA: tuner 0xC0 first), TS input set-up,
        // then the front-end init at 400 kHz.
        let standby = pos(&acc, Acc::W(0x100C, ctrl_word(2, true, true, true, 0xC0)));
        let first_100 = pos(&acc, Acc::W(0x1000, T100));
        let tsd = pos(&acc, Acc::FbW(0x50, 1));
        let doorbell = pos(&acc, Acc::W(0x3020, 24));
        let first_400 = pos(&acc, Acc::W(0x1000, T400));
        assert!(first_100 < standby && standby < tsd && tsd < doorbell && doorbell < first_400);
        assert_eq!(acc.iter().filter(|a| **a == Acc::W(0x1000, T100)).count(), 3);
        assert!(acc[first_400..].iter().all(|a| *a != Acc::W(0x1000, T100)));
        assert!(acc.contains(&Acc::FbW(0x68, 3)));
        assert!(d.frontend_mut().is_initialized());
    }

    #[test]
    fn failed_standby_does_not_stop_the_start() {
        let mut d = device(0xB2, Config::default());
        script_cmds(d.vixs_mut(), &[(24, 0, 1)]);
        // The first transfer (MJ102 standby to 0x20) is not acknowledged.
        bus(&mut d).queue(Space::Mem, MMR + 0x100C, &[CMD_ERROR]);
        bus(&mut d).clear();
        d.start().unwrap();
        let acc = accesses(&bus(&mut d).ops());
        assert_eq!(acc.iter().filter(|a| **a == Acc::W(0x1000, T100)).count(), 1);
        assert!(acc.contains(&Acc::FbW(0x68, 0)));
        assert!(pos(&acc, Acc::W(0x3020, 24)) < pos(&acc, Acc::W(0x1000, T400)));
        // MJ110 kit: the front end is at 0x30.
        assert!(acc.contains(&Acc::W(0x100C, ctrl_word(2, true, true, true, 0x30))));
    }

    #[test]
    fn dual_toshiba_standby_selects_the_first_front_end_again() {
        let mut d = device(0xBB, Config::default());
        bus(&mut d).clear();
        d.standby().unwrap();
        let gpio: Vec<bool> = writes(&bus(&mut d).ops())
            .into_iter()
            .filter_map(|a| match a {
                Acc::W(0x1088, v) => Some(v & 4 != 0),
                _ => None,
            })
            .collect();
        assert_eq!(gpio, [false, true, false]);
    }

    fn spec() -> ProgramSpec {
        ProgramSpec {
            sid: 0x408,
            pmt_pid: 0x1F0,
            pcr_pid: 0x1FF,
            ecm_pid: Some(0x1F01),
            es: vec![
                EsSpec { pid: 0x100, stream_type: 2, ecm_pid: None },
                EsSpec { pid: 0x110, stream_type: 0x0F, ecm_pid: None },
            ],
        }
    }

    fn script_handles(d: &mut Dth30<MockPci>, slots: &[(u32, u32, u32)]) {
        let ring = d.vixs_mut().ring().unwrap().clone();
        for &(slot, off, phys) in slots {
            bus(d).set(Space::Mem, FB + ring.slot_fb(slot) + 0x24 + off, phys);
        }
    }

    #[test]
    fn stream_lifecycle() {
        let mut d = device(0xB4, Config::default());
        script_cmds(
            d.vixs_mut(),
            &[
                (25, 0, 1),
                (11, 1, 1),
                (8, 2, 1),
                (8, 3, 1),
                (10, 4, 1),
                (9, 3, 1),
                (10, 5, 1),
                (9, 2, 1),
                (26, 0, 1),
                (24, 6, 1),
            ],
        );
        script_handles(&mut d, &[(0, 0, 0xCA5), (2, 0x18, 0xAA7), (3, 0x18, 0x9E0)]);
        bus(&mut d).clear();

        let cas = d.open_cas().unwrap();
        assert_eq!(
            cas,
            StreamInfo {
                kind: StreamKind::Generic,
                endpoint: 0x82,
                transfer_len: 0x8000,
                header_len: 0x1EC,
                phys: 0xCA5
            }
        );
        let raw = d.open_raw().unwrap();
        assert_eq!((raw.endpoint, raw.transfer_len, raw.header_len, raw.phys), (0x83, 48128, 0, 0xAA7));
        let pn = d.open_program(&spec()).unwrap();
        assert_eq!((pn.endpoint, pn.transfer_len, pn.phys), (0x84, 48128, 0x9E0));

        let m = bus(&mut d);
        assert_eq!(sent_payload(m, 0)[..0x530], build_open_generic(1, CasMode::Bcas));
        // The card reset right after the CAS open.
        assert_eq!(sent_payload(m, 1)[..0x2DC], build_card_reset(0xCA5));
        assert_eq!(sent_payload(m, 2)[..0x1198], build_open_psi(2, 0));
        let p = sent_payload(m, 3);
        assert_eq!((w(&p, 0x18), w(&p, 0x20), w(&p, 0x28)), (3, 0xCA5, 0xAA7));
        assert_eq!((w(&p, 0x7C), w(&p, 0xC08), w(&p, 0xC40), w(&p, 0xC70)), (0x100, 0x1FF, 0x1F0, 0x408));
        assert_eq!((w(&p, 0xEE4), w(&p, 0xEE8)), (1, 0x110));
        assert_eq!(w(&p, 0xF2C), 8);
        assert_eq!((w(&p, 0x1068), w(&p, 0x106C), w(&p, 0x1070)), (2, 0x100, 0x110));
        assert_eq!((w(&p, 0x10CC), w(&p, 0x10D0)), (0x1F01, 0x1F01));
        assert_eq!((w(&p, 0x112C), w(&p, 0x1130)), (2, 0x0F));

        // One of each; the TS input set-up and a CAS-mode change wait.
        let n = bus(&mut d).bulk().len();
        assert!(d.open_cas().is_err());
        assert!(d.open_raw().is_err());
        assert!(d.open_program(&spec()).is_err());
        assert!(d.tsd_config().is_err());
        assert!(d.set_cas_mode(CasMode::BcasDrm).is_err());
        assert_eq!(bus(&mut d).bulk().len(), n);
        assert_eq!(d.stream(StreamKind::Program).unwrap().phys(), 0x9E0);

        // Stop in the vendor's order.
        bus(&mut d).clear();
        d.close_all().unwrap();
        let funcs: Vec<(u32, u32)> = bus(&mut d).bulk().iter().map(|(_, s)| (w(s, 0x0C), w(s, 0x1C))).collect();
        assert_eq!(funcs, [(10, 4), (9, 3), (10, 5), (9, 2), (26, 0)]);
        assert!([StreamKind::Generic, StreamKind::Raw, StreamKind::Program].iter().all(|&k| d.stream(k).is_none()));
        assert!((0..MAX_SLOTS).all(|i| !d.vixs_mut().ring().unwrap().is_reserved(i)));
        d.close_program().unwrap(); // nothing open: nothing sent
        assert_eq!(bus(&mut d).bulk().len(), 5);

        // Afterwards the FIFOs are flushed and the set-up is allowed again.
        bus(&mut d).set(Space::Csr, 0x390, 0x40);
        assert!(d.flush_endpoint_fifo(0x84).unwrap());
        d.tsd_config().unwrap();
    }

    #[test]
    fn refused_close_keeps_the_stream() {
        let mut d = device(0xB4, Config::default());
        script_cmds(d.vixs_mut(), &[(8, 0, 1), (10, 1, 0), (10, 2, 1), (9, 0, 1)]);
        d.open_raw().unwrap();
        assert!(d.close_raw().is_err());
        assert!(d.stream(StreamKind::Raw).is_some());
        d.close_raw().unwrap();
        assert!(d.stream(StreamKind::Raw).is_none());
    }

    #[test]
    fn drm_mode() {
        let cfg = Config { cas_mode: CasMode::BcasDrm, ts_port: 1, ..Config::default() };
        let mut d = device(0xB4, cfg);
        script_cmds(d.vixs_mut(), &[(25, 0, 1), (11, 1, 1), (8, 2, 1), (26, 0, 1)]);
        d.open_cas().unwrap();
        d.open_program(&spec()).unwrap();
        let m = bus(&mut d);
        assert_eq!(w(&sent_payload(m, 0), 0x08), 32);
        let p = sent_payload(m, 2);
        assert_eq!((w(&p, 0xF2C), w(&p, 0x08), w(&p, 0x10), w(&p, 0x28)), (16, 1, 1, 0));
        // Switching back needs the CAS object closed (the Program stream too).
        d.close_cas().unwrap();
        assert!(d.set_cas_mode(CasMode::Bcas).is_err());
    }
}
