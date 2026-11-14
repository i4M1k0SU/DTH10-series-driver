//! The DT-H30/U2 family (DT-H30/U2, DT-H50/U2, DT-H50/U2W: a ViXS XCode2
//! behind a PLX NET2280, `dth30-core`) behind [`Session`], for
//! `dth10 serve`.
//!
//! The family has one terrestrial tuner. Its two TS outputs map to the
//! stream modes:
//!
//! * `raw` ([`StreamModeCfg::Raw`]): the Raw stream (EP 0x83), the
//!   broadcast TS as received, scrambled; no card is involved.
//! * `device` ([`StreamModeCfg::Device`]): the Program stream (EP 0x84) of
//!   one service, which the firmware descrambles with the built-in B-CAS
//!   card: a partial TS with the service's PMT, ES and ECM and the
//!   channel's PSI/SI (PAT as broadcast, NIT, SDT, EIT, TOT...). The Raw
//!   stream feeds it and is read (and discarded) alongside. The CAS object
//!   is opened (and the card reset) on the first device-mode tune and stays
//!   open until [`Session::close`].
//!
//! A unit is named by its USB serial number; the DT-H50/U2 has none, so it
//! is named `DTH30-<product ID>` instead (`-2`, `-3`, ... for further units
//! of the same product, in USB location order) ([`list`]).

use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use dth10_core::{Error, Result};
use dth30_core::device::{Config as DevConfig, Dth30, StreamInfo};
use dth30_core::psi::{ProgramScanner, ProgramSpec};
use dth30_core::vixs::stream::{card_state, is_cdxm_packet, parse_generic_block};
use dth30_core::vixs::{Firmware, Options};
use dth30_usb::{Found, Net2280Usb, StreamReader};

use crate::session::Session;
use crate::{Channel, DeviceProgram, StreamModeCfg, System, TsQueue};

/// File name of the vendor driver the firmware is taken from.
pub const FIRMWARE_FILE: &str = "BufFsUSB.sys";
/// Prefix of the name of a unit without a USB serial number.
pub const UNIT_PREFIX: &str = "DTH30-";

/// How long the card may take to report itself activated after the CAS
/// open (it takes about a second).
const CARD_WAIT: Duration = Duration::from_secs(5);
/// How long the Raw stream is read for the service's PAT and PMT.
const PSI_WAIT: Duration = Duration::from_secs(5);
/// How much longer the lock wait goes on while there is a signal.
const SIGNAL_GRACE: Duration = Duration::from_secs(2);
/// Reads that fail in a row before a stream gives itself up.
const FAILED_READS: u32 = 20;

const PACKET: usize = 188;

/// The family's devices present now, with their unit names.
pub fn list() -> Result<Vec<(String, Found)>> {
    let mut found = Net2280Usb::list_all()?;
    found.sort_by(|a, b| a.location.cmp(&b.location));
    Ok(name_units(found))
}

fn name_units(found: Vec<Found>) -> Vec<(String, Found)> {
    let mut out: Vec<(String, Found)> = Vec::new();
    for f in found {
        let name = match f.serial.as_deref().filter(|s| !s.is_empty()) {
            Some(s) => s.to_owned(),
            None => {
                let base = format!("{UNIT_PREFIX}{:04X}", f.product_id);
                let n = out.iter().filter(|(n, _)| *n == base || n.starts_with(&format!("{base}-"))).count();
                if n == 0 { base } else { format!("{base}-{}", n + 1) }
            }
        };
        out.push((name, f));
    }
    out
}

/// The firmware source: `explicit`, else `$DTH30_FIRMWARE`, else
/// [`FIRMWARE_FILE`] in `state_dir`, else next to this executable.
pub fn firmware_path(explicit: Option<&Path>, state_dir: &Path) -> Option<std::path::PathBuf> {
    if let Some(p) = explicit {
        return Some(p.to_path_buf());
    }
    if let Some(p) = std::env::var_os("DTH30_FIRMWARE") {
        return Some(p.into());
    }
    let candidates = [
        Some(state_dir.join(FIRMWARE_FILE)),
        std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.join(FIRMWARE_FILE))),
    ];
    candidates.into_iter().flatten().find(|p| p.is_file())
}

/// One opened unit of the family.
pub struct Dth30Tuner {
    name: String,
    dev: Dth30<Net2280Usb>,
    queue: TsQueue,
    mode: StreamModeCfg,
    program: DeviceProgram,
    lock_wait: Duration,
    read_depth: usize,
    current: Option<Channel>,
    cas: Option<CasWatch>,
    pump: Option<Pump>,
}

impl Dth30Tuner {
    /// Open unit `name` (see [`list`]) and bring its chip up with `fw`.
    pub fn open(name: &str, fw: &Firmware, read_depth: usize, queue: TsQueue) -> Result<Self> {
        let found = list()?
            .into_iter()
            .find(|(n, _)| n == name)
            .map(|(_, f)| f)
            .ok_or_else(|| Error::Usb(format!("no DT-H30/U2-family unit {name}")))?;
        let t = found.open()?;
        let pid = t.product_id();
        let pipes: Vec<u8> = t.endpoints().iter().map(|e| e.address).collect();
        let mut dev = Dth30::open(t, pid, &pipes, Options::default(), DevConfig::default())?;
        dev.bring_up(fw)?;
        log::info!(
            "unit {name}: {} board 0x{:02x}, {} front end",
            found.product_label(),
            dev.board(),
            dev.kit().name()
        );
        Ok(Self {
            name: name.to_owned(),
            dev,
            queue,
            mode: StreamModeCfg::Device,
            program: DeviceProgram::First,
            lock_wait: Duration::from_millis(1200),
            read_depth: read_depth.max(1),
            current: None,
            cas: None,
            pump: None,
        })
    }

    fn reader(&mut self, s: &StreamInfo) -> Result<StreamReader> {
        let mut r = self.dev.vixs_mut().bus().stream_reader(s.endpoint, s.transfer_len, self.read_depth)?;
        r.start();
        Ok(r)
    }

    /// Wait for lock; a little longer while there is a signal.
    fn wait_lock(&mut self) -> Result<bool> {
        let start = Instant::now();
        let mut deadline = start + self.lock_wait;
        let mut extended = false;
        loop {
            let st = self.dev.status()?;
            if st.locked {
                log::info!("unit {}: locked after {:?}, C/N {:.1} dB", self.name, start.elapsed(), st.cnr_db);
                return Ok(true);
            }
            if Instant::now() >= deadline {
                if st.cnr_db > 0.0 && !extended {
                    extended = true;
                    deadline += SIGNAL_GRACE;
                } else {
                    log::info!("unit {}: no lock within {:?}", self.name, start.elapsed());
                    return Ok(false);
                }
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    /// Open the CAS object (resetting the card) unless it is open, and wait
    /// for the card to report itself activated.
    fn ensure_cas(&mut self) -> Result<()> {
        if self.cas.is_none() {
            let info = self.dev.open_cas()?;
            let r = self.reader(&info)?;
            self.cas = Some(CasWatch::start(info, r));
        }
        let Some(cas) = self.cas.as_ref() else { return Ok(()) };
        let (card, activated) = (cas.card.clone(), cas.activated.clone());
        let deadline = Instant::now() + CARD_WAIT;
        while !activated.load(Ordering::Relaxed) {
            if Instant::now() >= deadline {
                log::warn!(
                    "unit {}: the B-CAS card did not activate (state 0x{:x}); the TS stays scrambled",
                    self.name,
                    card.load(Ordering::Relaxed)
                );
                return Ok(());
            }
            std::thread::sleep(Duration::from_millis(20));
        }
        Ok(())
    }

    /// Read the Raw stream until the PAT and PMT of the service are seen.
    fn scan(&self, raw: &mut StreamReader) -> Result<ProgramSpec> {
        let sid = match self.program {
            DeviceProgram::Id(n) => Some(n),
            DeviceProgram::First => None,
            DeviceProgram::All => {
                log::warn!("unit {}: program=all is not supported, streaming the first service", self.name);
                None
            }
        };
        let mut scan = ProgramScanner::new(sid);
        let deadline = Instant::now() + PSI_WAIT;
        while Instant::now() < deadline {
            if let Some(b) = raw.next(Duration::from_millis(200))?
                && let Some(spec) = scan.push(&b)
            {
                return Ok(spec);
            }
        }
        Err(Error::Protocol(match sid {
            Some(_) => "no such service in the PAT, or no PMT, within 5 s",
            None => "no PAT/PMT in the Raw stream within 5 s",
        }))
    }

    /// Start streaming the tuned channel.
    fn start_stream(&mut self) -> Result<()> {
        let raw_info = self.dev.open_raw()?;
        let mut raw = self.reader(&raw_info)?;
        if self.mode == StreamModeCfg::Raw {
            self.pump = Some(Pump::start(self.queue.clone(), (raw_info, raw), None));
            return Ok(());
        }
        let spec = match self.scan(&mut raw) {
            Ok(s) => s,
            Err(e) => {
                self.close_stream(&raw_info, raw);
                return Err(e);
            }
        };
        log::info!("unit {}: service {} (PMT 0x{:x}, {} streams)", self.name, spec.sid, spec.pmt_pid, spec.es.len());
        let prog_info = match self.dev.open_program(&spec) {
            Ok(i) => i,
            Err(e) => {
                self.close_stream(&raw_info, raw);
                return Err(e);
            }
        };
        let prog = self.reader(&prog_info)?;
        self.pump = Some(Pump::start(self.queue.clone(), (raw_info, raw), Some((prog_info, prog))));
        Ok(())
    }

    /// Close one stream (the reads stay pending during the close, as in the
    /// vendor driver), then cancel the reads and empty the FIFO.
    fn close_stream(&mut self, info: &StreamInfo, mut r: StreamReader) {
        let closed = match info.kind {
            dth30_core::vixs::stream::StreamKind::Program => self.dev.close_program(),
            dth30_core::vixs::stream::StreamKind::Raw => self.dev.close_raw(),
            dth30_core::vixs::stream::StreamKind::Generic => self.dev.close_cas(),
        };
        if let Err(e) = closed {
            log::warn!("unit {}: closing the {:?} stream: {e}", self.name, info.kind);
        }
        r.stop();
        if let Err(e) = self.dev.flush_endpoint_fifo(info.endpoint) {
            log::debug!("unit {}: FIFO of 0x{:02x}: {e}", self.name, info.endpoint);
        }
    }

    /// Stop the stream: Program, then Raw.
    fn stop_stream(&mut self) {
        let Some(p) = self.pump.take() else { return };
        let (raw, prog) = p.stop();
        if let Some((info, r)) = prog {
            self.close_stream(&info, r);
        }
        if let Some((info, r)) = raw {
            self.close_stream(&info, r);
        }
    }
}

impl Session for Dth30Tuner {
    fn set_queue(&mut self, tuner: usize, queue: TsQueue) {
        if tuner == 0 {
            self.queue = queue;
        }
    }
    /// The firmware descrambles with the card: no link decryption.
    fn needs_app_keys(&self, _raw: bool, _system: System) -> bool {
        false
    }
    fn has_second_terrestrial(&self) -> bool {
        false
    }
    fn set_stream_mode(&mut self, mode: StreamModeCfg) {
        self.mode = mode;
    }
    fn set_device_program(&mut self, program: DeviceProgram) {
        self.program = program;
    }
    fn set_lock_wait_ms(&mut self, system: System, ms: u64) {
        if system == System::IsdbT {
            self.lock_wait = Duration::from_millis(ms);
        }
    }

    fn set_channel_on(&mut self, tuner: usize, ch: &Channel) -> Result<()> {
        match (ch.system, tuner) {
            (System::IsdbT, 0) => {}
            (System::IsdbS, _) => return Err(Error::Unsupported("this unit has no satellite tuner".into())),
            _ => return Err(Error::Unsupported(format!("no terrestrial tuner #{tuner} on this unit"))),
        }
        log::info!("unit {}: tune {} ({} Hz), {:?}", self.name, ch.name, ch.frequency_hz, self.mode);
        self.stop_stream();
        self.current = None;
        self.dev.tune(ch.frequency_hz)?;
        if !self.wait_lock()? {
            return Err(Error::Protocol("ISDB-T: no lock (no signal on this channel)"));
        }
        if self.mode == StreamModeCfg::Device {
            self.ensure_cas()?;
        }
        self.start_stream()?;
        self.current = Some(ch.clone());
        Ok(())
    }

    fn signal_level_of(&mut self, tuner: usize) -> f32 {
        if tuner != 0 {
            return 0.0;
        }
        match self.dev.status() {
            Ok(st) => st.cnr_db as f32,
            Err(e) => {
                log::debug!("unit {}: signal status: {e}", self.name);
                0.0
            }
        }
    }

    fn release(&mut self, tuner: usize) {
        if tuner == 0 {
            self.stop_stream();
            self.current = None;
        }
    }

    fn stream_failed(&self, tuner: usize) -> bool {
        tuner == 0 && self.pump.as_ref().is_some_and(|p| p.failed.load(Ordering::Relaxed))
    }

    fn poll_status(&mut self) -> [Option<f32>; 2] {
        let ter = self.current.is_some().then(|| self.signal_level_of(0));
        if let Some(c) = ter {
            log::debug!("unit {}: status: C/N {c:.1} dB", self.name);
        }
        [ter, None]
    }

    fn close(&mut self) {
        self.stop_stream();
        self.current = None;
        if let Some(c) = self.cas.take() {
            let (info, r) = c.stop();
            self.close_stream(&info, r);
        }
        if let Err(e) = self.dev.standby() {
            log::debug!("unit {}: standby: {e}", self.name);
        }
    }
}

impl Drop for Dth30Tuner {
    fn drop(&mut self) {
        self.stop_stream();
        if let Some(c) = self.cas.take() {
            let (info, r) = c.stop();
            self.close_stream(&info, r);
        }
    }
}

/// The CAS endpoint read in the background for as long as the CAS object
/// is open (an unread FIFO may stall the firmware). A block's card state
/// names the event it reports (inserted, activated, an EMM...), so
/// `activated` is kept from the activation until the card is removed.
struct CasWatch {
    card: Arc<AtomicU32>,
    activated: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    thread: JoinHandle<(StreamInfo, StreamReader)>,
}

impl CasWatch {
    fn start(info: StreamInfo, mut r: StreamReader) -> Self {
        let card = Arc::new(AtomicU32::new(0));
        let activated = Arc::new(AtomicBool::new(false));
        let stop = Arc::new(AtomicBool::new(false));
        let (c, a, s) = (card.clone(), activated.clone(), stop.clone());
        let thread = std::thread::spawn(move || {
            while !s.load(Ordering::Relaxed) {
                match r.next(Duration::from_millis(100)) {
                    Ok(Some(b)) => {
                        let Some((h, _)) = parse_generic_block(&b) else { continue };
                        if h.card & card_state::REMOVED != 0 {
                            a.store(false, Ordering::Relaxed);
                        } else if h.card & card_state::ACTIVATED != 0 {
                            a.store(true, Ordering::Relaxed);
                        }
                        if c.swap(h.card, Ordering::Relaxed) != h.card {
                            log::info!("B-CAS card event 0x{:x}", h.card);
                        }
                    }
                    Ok(None) => {}
                    Err(e) => {
                        log::debug!("CAS endpoint: {e}");
                        std::thread::sleep(Duration::from_millis(100));
                    }
                }
            }
            (info, r)
        });
        Self { card, activated, stop, thread }
    }

    fn stop(self) -> (StreamInfo, StreamReader) {
        self.stop.store(true, Ordering::Relaxed);
        self.thread.join().expect("CAS reader thread")
    }
}

type Reader = (StreamInfo, StreamReader);

/// The thread moving a stream into the queue.
struct Pump {
    stop: Arc<AtomicBool>,
    failed: Arc<AtomicBool>,
    thread: JoinHandle<(Option<Reader>, Option<Reader>)>,
}

impl Pump {
    /// Raw alone: its packets go to the queue. With a Program stream: the
    /// Program packets, Raw being read and discarded. Both through
    /// [`TsFilter`].
    fn start(queue: TsQueue, raw: Reader, prog: Option<Reader>) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let failed = Arc::new(AtomicBool::new(false));
        let (s, f) = (stop.clone(), failed.clone());
        let thread = std::thread::spawn(move || {
            let (raw_info, mut raw) = raw;
            let mut errors = 0u32;
            let prog = match prog {
                None => {
                    // Whole packets only: a stream may start mid-packet.
                    let mut filter = TsFilter::default();
                    while !s.load(Ordering::Relaxed) && !f.load(Ordering::Relaxed) {
                        if let Some(b) = count(&mut errors, raw.next(Duration::from_millis(100))) {
                            let out = filter.push(&b);
                            if !out.is_empty() {
                                queue.push(out);
                            }
                        }
                        if errors >= FAILED_READS {
                            log::warn!("TS reads keep failing; the stream gives up");
                            f.store(true, Ordering::Relaxed);
                        }
                    }
                    None
                }
                Some((info, mut p)) => {
                    let mut filter = TsFilter::default();
                    while !s.load(Ordering::Relaxed) && !f.load(Ordering::Relaxed) {
                        if let Some(b) = count(&mut errors, p.next(Duration::from_millis(50))) {
                            let out = filter.push(&b);
                            if !out.is_empty() {
                                queue.push(out);
                            }
                        }
                        // The Raw stream feeds the Program stream: keep it drained.
                        while count(&mut errors, raw.next(Duration::from_millis(1))).is_some() {}
                        if errors >= FAILED_READS {
                            log::warn!("TS reads keep failing; the stream gives up");
                            f.store(true, Ordering::Relaxed);
                        }
                    }
                    Some((info, p))
                }
            };
            (Some((raw_info, raw)), prog)
        });
        Self { stop, failed, thread }
    }

    /// End the thread and hand the readers back (still started, so their
    /// reads stay pending during the closes).
    fn stop(self) -> (Option<Reader>, Option<Reader>) {
        self.stop.store(true, Ordering::Relaxed);
        self.thread.join().unwrap_or_else(|_| {
            log::error!("TS thread panicked");
            (None, None)
        })
    }
}

/// A read's data; a failed read counts towards [`FAILED_READS`].
fn count(errors: &mut u32, r: Result<Option<Vec<u8>>>) -> Option<Vec<u8>> {
    match r {
        Ok(b) => {
            *errors = 0;
            b
        }
        Err(e) => {
            *errors += 1;
            log::debug!("TS read: {e}");
            std::thread::sleep(Duration::from_millis(20));
            None
        }
    }
}

/// A stream as output: whole 188-byte packets (a stream may start
/// mid-packet), without the vendor's "CDXM" packets (which the firmware
/// adds to the Program stream in the DRM mode).
#[derive(Default)]
pub struct TsFilter {
    carry: Vec<u8>,
}

impl TsFilter {
    /// The next bytes of the stream; returns the packets to output
    /// (an incomplete tail waits for the next call).
    pub fn push(&mut self, data: &[u8]) -> Vec<u8> {
        let mut out = Vec::with_capacity(data.len());
        for_packets(&mut self.carry, data, |p| {
            if !is_cdxm_packet(p) {
                out.extend_from_slice(p);
            }
        });
        out
    }
}

/// Call `f` with each 188-byte packet of `carry` + `data`, resyncing on the
/// sync byte; an incomplete tail stays in `carry`.
fn for_packets(carry: &mut Vec<u8>, data: &[u8], mut f: impl FnMut(&[u8])) {
    carry.extend_from_slice(data);
    let mut i = 0;
    while i + PACKET <= carry.len() {
        if carry[i] != 0x47 {
            i += 1;
            continue;
        }
        f(&carry[i..i + PACKET]);
        i += PACKET;
    }
    carry.drain(..i);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn packet(pid: u16, cc: u8, payload: &[u8]) -> Vec<u8> {
        let mut p = vec![0xFFu8; PACKET];
        p[..4].copy_from_slice(&[0x47, 0x40 | (pid >> 8) as u8, pid as u8, 0x10 | cc]);
        p[4..4 + payload.len()].copy_from_slice(payload);
        p
    }

    fn pid_of(p: &[u8]) -> u16 {
        u16::from(p[1] & 0x1F) << 8 | u16::from(p[2])
    }

    #[test]
    fn program_packets_pass_without_cdxm() {
        let mut m = TsFilter::default();
        let mut cdxm = packet(0x100, 0, &[]);
        cdxm[7..11].copy_from_slice(b"CDXM");
        let mut data = packet(0x100, 1, &[1, 2, 3]);
        data.extend(&cdxm);
        data.extend(packet(0x110, 0, &[]));
        // Split mid-packet: the tail waits for the next chunk.
        let (a, b) = data.split_at(200);
        let mut out = m.push(a);
        out.extend(m.push(b));
        assert_eq!(out.chunks(PACKET).map(pid_of).collect::<Vec<_>>(), [0x100, 0x110]);
    }

    #[test]
    fn units_without_serial_are_named_by_product() {
        let f = |pid: u16, serial: Option<&str>| Found {
            vendor_id: 0x0411,
            product_id: pid,
            serial: serial.map(str::to_owned),
            location: String::new(),
            info: None,
            #[cfg(any(target_os = "linux", target_os = "android"))]
            path: None,
        };
        let names: Vec<String> =
            name_units(vec![f(0x129, None), f(0x129, Some("")), f(0x128, Some("ABC")), f(0x128, None)])
                .into_iter()
                .map(|(n, _)| n)
                .collect();
        assert_eq!(names, ["DTH30-0129", "DTH30-0129-2", "ABC", "DTH30-0128"]);
    }
}
