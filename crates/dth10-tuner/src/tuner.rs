//! [`Tuner`]: owns the USB transport, the tuner kits, the reader thread and
//! the TS queue. Behaviour is driven by [`TunerConfig`].

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use dth10_core::LockExt;
use dth10_core::cepm::{self, AppKeys, Decryptor, Handshake};
use dth10_core::kit::{SignalStatus, TuneRequest, field, lock};
use dth10_core::lj101::Lj101Kit;
use dth10_core::terrestrial::TerrestrialKit;
use dth10_core::tlv::{DeviceInformation, TunerDesc};
use dth10_core::{Device, Error, Result, StreamMode};
use dth10_usb::{StreamReader, UsbTransport};

use crate::channels::{Channel, System};
use crate::config::{DeviceProgram, StreamModeCfg, TunerConfig};
use crate::psi::{Program, PsiCollector};

const TS_PACKET: usize = 188;
/// Upper bound on queued bytes before the oldest chunks are dropped.
const QUEUE_LIMIT_BYTES: usize = 32 * 1024 * 1024;
/// How long to collect PAT/PMT in raw mode before switching to device mode.
const PSI_LEARN_TIMEOUT: Duration = Duration::from_millis(2500);
/// PSI learn through the link decryption (satellite): a stale handshake is
/// renewed on the way (several seconds) before the first content key arrives.
const PSI_LEARN_TIMEOUT_ENCRYPTED: Duration = Duration::from_secs(12);

struct QueueInner {
    chunks: VecDeque<Vec<u8>>,
    bytes: usize,
    /// Bytes thrown away because the consumer did not keep up (the queue is
    /// bounded; the oldest chunks go first).
    dropped: u64,
}

/// Thread-safe queue of 188-byte-aligned TS chunks.
#[derive(Clone)]
pub struct TsQueue {
    inner: Arc<(Mutex<QueueInner>, Condvar)>,
}

impl Default for TsQueue {
    fn default() -> Self {
        Self::new()
    }
}

impl TsQueue {
    pub fn new() -> Self {
        TsQueue {
            inner: Arc::new((Mutex::new(QueueInner { chunks: VecDeque::new(), bytes: 0, dropped: 0 }), Condvar::new())),
        }
    }

    pub fn push(&self, chunk: Vec<u8>) {
        let (m, cv) = &*self.inner;
        let mut q = m.locked();
        q.bytes += chunk.len();
        q.chunks.push_back(chunk);
        while q.bytes > QUEUE_LIMIT_BYTES {
            if let Some(old) = q.chunks.pop_front() {
                q.bytes -= old.len();
                q.dropped += old.len() as u64;
            }
        }
        cv.notify_all();
    }

    /// Bytes dropped so far because the consumer did not keep up.
    pub fn dropped_bytes(&self) -> u64 {
        self.inner.0.locked().dropped
    }

    pub fn pop(&self) -> Option<Vec<u8>> {
        let (m, _) = &*self.inner;
        let mut q = m.locked();
        let c = q.chunks.pop_front()?;
        q.bytes -= c.len();
        Some(c)
    }

    pub fn len(&self) -> usize {
        self.inner.0.locked().chunks.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn purge(&self) {
        let mut q = self.inner.0.locked();
        q.chunks.clear();
        q.bytes = 0;
    }

    /// Wait until a chunk is available. `timeout_ms == 0` means "do not wait".
    pub fn wait(&self, timeout_ms: u32) -> bool {
        let (m, cv) = &*self.inner;
        let q = m.locked();
        if !q.chunks.is_empty() {
            return true;
        }
        if timeout_ms == 0 {
            return false;
        }
        let (q, _) = cv.wait_timeout(q, Duration::from_millis(timeout_ms as u64)).unwrap();
        !q.chunks.is_empty()
    }
}

/// Extracts TS packets from the bulk format of the bridge: the
/// pipe carries 256-byte blocks whose first 188 bytes are one TS packet and
/// whose remaining 68 bytes are padding. A transfer tail shorter than 256
/// bytes is dropped, as in the original driver.
pub struct Aligner {
    dropped_tail: u64,
    /// Blocks whose first byte was not a sync byte (seen at stream start on
    /// some hosts: the first blocks arrive with zeroed TS headers).
    unsynced: u64,
    drop_filler: bool,
}

pub const BLOCK: usize = 256;

impl Aligner {
    pub fn new(drop_filler: bool) -> Self {
        Aligner { dropped_tail: 0, unsynced: 0, drop_filler }
    }

    /// Return the TS packets contained in `data`.
    ///
    /// With `drop_filler`, packets with the transport-error indicator set
    /// *and* PID 0x1FFF are dropped: the bridge emits them (`47 9F FF 1F ...`)
    /// while the demodulator output is not valid yet, about 1.5 s after a
    /// tune, and a host would count them as errors.
    pub fn feed(&mut self, data: &[u8]) -> Vec<u8> {
        let blocks = data.len() / BLOCK;
        let tail = data.len() % BLOCK;
        if tail != 0 {
            self.dropped_tail += tail as u64;
            if self.dropped_tail == tail as u64 {
                log::warn!("bulk transfer length {} is not a multiple of {BLOCK}; dropping tail", data.len());
            }
        }
        let mut out = Vec::with_capacity(blocks * TS_PACKET);
        for b in 0..blocks {
            let pkt = &data[b * BLOCK..b * BLOCK + TS_PACKET];
            if pkt[0] != 0x47 {
                self.unsynced += 1;
                if self.unsynced == 1 || self.unsynced.is_multiple_of(10000) {
                    log::debug!("dropping block without sync byte ({} so far): {:02x?}", self.unsynced, &pkt[..8]);
                }
                continue;
            }
            if self.drop_filler && pkt[1] == 0x9F && pkt[2] == 0xFF {
                continue;
            }
            out.extend_from_slice(pkt);
        }
        out
    }
}

/// CEPM handshakes per request path, shared with the reader thread.
type CepmSet = Arc<Mutex<Vec<(u8, Handshake)>>>;

/// Reader thread over one bulk IN endpoint.
struct Stream {
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
    system: System,
    /// The reader thread gave up: the bulk reads kept failing (the device is gone).
    failed: Arc<AtomicBool>,
}

/// Bulk read errors in a row (50 ms apart) after which a stream's reader
/// thread gives up: an unplugged device never answers again, and a device in
/// trouble is better reopened by the next request than hammered.
const READ_FAILURES_BEFORE_GIVING_UP: u32 = 20;

/// What the reader thread needs to restart its stream after a new CEPM
/// handshake (docs/22 §6c): the device keeps wrapping the content keys for the
/// old session until the stream is disabled and enabled again, and it forgets
/// the program PID list on disable.
struct StreamRestart {
    dev: Device<UsbTransport>,
    desc: TunerDesc,
    mode: StreamMode,
    pids: Vec<(u32, Vec<u16>)>,
}

impl StreamRestart {
    fn run(&self) -> Result<()> {
        self.dev.stream_disable()?;
        for (prog, list) in &self.pids {
            self.dev.set_program_pids(*prog, list)?;
        }
        self.dev.stream_enable_mode(&self.desc, self.mode)?;
        send_program_lists(&self.dev, &self.pids);
        Ok(())
    }
}

/// Send the program lists again right after a stream enable. Firmware differs
/// in when it takes the list: the DT-H70/U2 (1.0.18) keeps one sent before the
/// enable, the DT-F100/U2 (0.8.17) clears it on the enable and then outputs
/// only PSI + 1seg, so the list goes to the device both before and after.
fn send_program_lists(dev: &Device<UsbTransport>, lists: &[(u32, Vec<u16>)]) {
    for (prog, pids) in lists {
        if let Err(e) = dev.set_program_pids(*prog, pids) {
            log::warn!("set_program_pids 0x{prog:04x} after the stream enable: {e}");
        }
    }
}

/// A raw (byte0=1) stream enable that the device refuses with a STALL: this
/// firmware has no raw mode (seen on the DT-F100/U2, 0.8.17). Reported as
/// unsupported rather than as a USB failure, so a server keeps the device open.
fn stream_enable_error(mode: StreamMode, e: Error) -> Error {
    match e {
        Error::Usb(m) if mode == StreamMode::RawFullTs && m.contains("stall") => Error::Unsupported(
            "this device's firmware has no raw mode (it rejects the raw stream enable); use device mode".into(),
        ),
        e => e,
    }
}

/// How often a running stream logs its counters.
const STATS_INTERVAL: Duration = Duration::from_secs(60);

/// Counters of one stream's reader thread, logged every [`STATS_INTERVAL`]
/// and when the stream ends: what came in over USB, what went out, and how
/// many continuity-counter errors each side had (errors on the input mean
/// data lost before the host saw it; errors on the output that the input did
/// not have mean packets dropped here, e.g. for want of a key).
#[derive(Default)]
struct StreamStats {
    bytes_in: u64,
    packets_in: u64,
    packets_out: u64,
    cc_errors_in: u64,
    cc_errors_out: u64,
    read_errors: u64,
}

impl StreamStats {
    fn log(&self, label: &str, elapsed: Duration, cepm: Option<&cepm::Stats>) {
        let c = cepm
            .map(|c| {
                format!(
                    ", cepm: decrypted {} clear {} no-key {} key-pkts {} unwraps {}/{} errors {}",
                    c.decrypted,
                    c.clear,
                    c.dropped_no_key,
                    c.key_packets,
                    c.session_unwraps,
                    c.content_unwraps,
                    c.errors
                )
            })
            .unwrap_or_default();
        log::info!(
            "{label} stats t+{:.0}s: in {} pkts ({:.1} MB) cc-err {}, out {} pkts cc-err {}, read errors {}{c}",
            elapsed.as_secs_f64(),
            self.packets_in,
            self.bytes_in as f64 / 1e6,
            self.cc_errors_in,
            self.packets_out,
            self.cc_errors_out,
            self.read_errors
        );
    }
}

/// Continuity-counter check over 188-byte packets (ARIB STD-B32 / ISO 13818-1):
/// the counter of a PID steps by one with every packet that carries a payload;
/// null packets and the first packet of a PID are not errors.
#[derive(Default)]
struct CcCheck {
    last: std::collections::HashMap<u16, u8>,
}

impl CcCheck {
    /// Number of discontinuities in `chunk`.
    fn feed(&mut self, chunk: &[u8]) -> u64 {
        let mut errors = 0;
        for p in chunk.as_chunks::<TS_PACKET>().0 {
            if p[0] != 0x47 || p[1] & 0x80 != 0 {
                continue;
            }
            let pid = ((p[1] as u16 & 0x1F) << 8) | p[2] as u16;
            if pid == 0x1FFF || p[3] & 0x10 == 0 {
                continue;
            }
            let cc = p[3] & 0x0F;
            if let Some(prev) = self.last.insert(pid, cc)
                && (prev + 1) & 0x0F != cc
                && prev != cc
            {
                errors += 1;
            }
        }
        errors
    }
}

impl Stream {
    #[allow(clippy::too_many_arguments)]
    fn spawn(
        mut reader: StreamReader,
        queue: TsQueue,
        system: System,
        drop_filler: bool,
        name: &str,
        decrypt: Option<(Handshake, UsbTransport, Arc<AppKeys>)>,
        cepm_flags: u8,
        state: Option<(std::path::PathBuf, CepmSet)>,
        restart: Option<StreamRestart>,
    ) -> Result<Self> {
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = stop.clone();
        let failed = Arc::new(AtomicBool::new(false));
        let failed2 = failed.clone();
        let label = format!("[{name}]");
        let handle = std::thread::Builder::new()
            .name(name.into())
            .spawn(move || {
                let mut aligner = Aligner::new(drop_filler);
                let mut dec = decrypt.map(|(hs, t, keys)| {
                    let mut d = Decryptor::new(hs, keys);
                    d.label = label.clone();
                    (d, t)
                });
                let mut st = StreamStats::default();
                let (mut cc_in, mut cc_out) = (CcCheck::default(), CcCheck::default());
                let t0 = Instant::now();
                let mut last_stats = Instant::now();
                let mut failures_in_a_row = 0u32;
                reader.start();
                while !stop2.load(Ordering::Relaxed) {
                    if last_stats.elapsed() >= STATS_INTERVAL {
                        last_stats = Instant::now();
                        st.log(&label, t0.elapsed(), dec.as_ref().map(|(d, _)| &d.stats));
                    }
                    match reader.next(Duration::from_millis(200)) {
                        Ok(Some(data)) => {
                            failures_in_a_row = 0;
                            st.bytes_in += data.len() as u64;
                            let mut chunk = aligner.feed(&data);
                            st.packets_in += (chunk.len() / TS_PACKET) as u64;
                            st.cc_errors_in += cc_in.feed(&chunk);
                            if let Some((d, t)) = dec.as_mut() {
                                let mut out = Vec::with_capacity(chunk.len());
                                for pkt in chunk.as_chunks_mut::<TS_PACKET>().0 {
                                    match d.process(t, cepm_flags, pkt) {
                                        Ok(true) => out.extend_from_slice(pkt),
                                        Ok(false) => {}
                                        Err(e) => log::warn!("cepm: {e}"),
                                    }
                                }
                                chunk = out;
                                if let Some(hs) = d.take_new_handshake() {
                                    if let Some((path, shared)) = state.as_ref() {
                                        let mut set = shared.locked();
                                        set.retain(|(f, _)| *f != cepm_flags);
                                        set.push((cepm_flags, hs));
                                        if let Err(e) = cepm::save_set(path, &set) {
                                            log::warn!("cepm: cannot save handshake state to {}: {e}", path.display());
                                        }
                                    }
                                    if let Some(r) = restart.as_ref() {
                                        match r.run() {
                                            Ok(()) => log::info!("cepm: stream restarted for the new session"),
                                            Err(e) => log::warn!("cepm: stream restart failed: {e}"),
                                        }
                                    }
                                }
                            }
                            st.packets_out += (chunk.len() / TS_PACKET) as u64;
                            st.cc_errors_out += cc_out.feed(&chunk);
                            if !chunk.is_empty() {
                                queue.push(chunk);
                            }
                        }
                        Ok(None) => failures_in_a_row = 0,
                        Err(e) => {
                            st.read_errors += 1;
                            failures_in_a_row += 1;
                            if failures_in_a_row >= READ_FAILURES_BEFORE_GIVING_UP {
                                log::error!("{label} {failures_in_a_row} bulk read errors in a row ({e}): the device is gone; giving this stream up");
                                failed2.store(true, Ordering::Relaxed);
                                break;
                            }
                            log::warn!("{label} bulk read error: {e}");
                            let _ = reader.clear_halt();
                            std::thread::sleep(Duration::from_millis(50));
                        }
                    }
                }
                reader.stop();
                st.log(&label, t0.elapsed(), dec.as_ref().map(|(d, _)| &d.stats));
            })
            .map_err(|e| Error::Usb(format!("spawn reader thread: {e}")))?;
        Ok(Stream { stop, handle: Some(handle), system, failed })
    }

    fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

struct Satellite {
    /// Kit whose `dev` uses `TunerConfig::sat_i2c_path` for I2C/GPIO.
    kit: Lj101Kit<UsbTransport>,
    /// Device handle on `TunerConfig::sat_stream_path`, used for the stream-enable command.
    stream_dev: Device<UsbTransport>,
    info: DeviceInformation,
    endpoint: u8,
    lnb_volt: u32,
    /// `Lj101Kit::init` has run (at open; on first use when that failed).
    initialised: bool,
    /// The command path of the satellite kit's I2C and GPIO.
    i2c_path: u8,
}

/// The device's report that the antenna supply may be over-current: event
/// 0x0000/0x1011 for the satellite filter with bit 0 of its fifth payload byte set.
fn lnb_overcurrent_report(ev: &dth10_core::events::Event) -> bool {
    (ev.class, ev.code) == dth10_core::events::code::LNB_CHECK
        && ev.targets & 0b10 != 0
        && ev.payload.get(4).is_some_and(|b| b & 1 != 0)
}

/// After an over-current report: 10 ms later (the vendor driver's delay), the
/// supply's state bit in the GPIO status block; not good means switch the
/// supply off (both its bits).
fn check_lnb_supply(dev: &Device<UsbTransport>, cut: &AtomicBool) {
    std::thread::sleep(Duration::from_millis(10));
    match dev.gpio_read() {
        Ok(st) if st[12] & 1 == 0 => match dev.gpio_write(0x2020, 0) {
            Ok(()) => {
                cut.store(true, Ordering::Relaxed);
                log::warn!("antenna (LNB) supply over-current: switched off; the next BS/CS tune switches it on again");
            }
            Err(e) => log::warn!("antenna (LNB) supply over-current, but switching it off failed: {e}"),
        },
        Ok(_) => log::info!("antenna (LNB) supply over-current report, but the supply is good"),
        Err(e) => log::warn!("antenna (LNB) supply over-current report, GPIO status: {e}"),
    }
}

/// An opened device: terrestrial kit on tuner #0, optional satellite kit.
pub struct Tuner {
    cfg: TunerConfig,
    transport: UsbTransport,
    terrestrial: TerrestrialKit<UsbTransport>,
    info0: DeviceInformation,
    satellite: Option<Satellite>,
    /// The running streams by system: 0 = terrestrial (tuner #0), 1 = satellite.
    streams: [Option<Stream>; 2],
    /// Where each system's TS goes: both the queue given at open unless
    /// [`Tuner::set_queue`] changed one.
    queues: [TsQueue; 2],
    /// The channel each system is tuned to.
    current: [Option<Channel>; 2],
    /// The system of the last `set_channel`.
    last: Option<System>,
    /// CEPM handshake results per request path (device mode with link decryption),
    /// loaded from `TunerConfig::cepm_state` or done on demand; shared with the
    /// reader thread, which replaces one when the device no longer honours it.
    cepm: CepmSet,
    /// Application credentials for CEPM, loaded from `TunerConfig::app_keys`
    /// on first use (or given with [`Tuner::with_app_keys`]).
    keys: Option<Arc<AppKeys>>,
    /// Set by the event thread when it switched the antenna supply off after
    /// an over-current report; the next satellite tune switches it on again
    /// (with its own health check).
    lnb_cut: Arc<AtomicBool>,
    /// The interrupt-endpoint thread, if one runs. Last field: dropped after
    /// [`Tuner::close`], it stops the thread and waits for it.
    _events: Option<EventThread>,
}

/// The interrupt-endpoint reader. It holds the endpoint and with it the
/// device handle, so it is stopped and joined when the tuner goes: otherwise
/// the device could not be opened again (WinUSB refuses a second open).
struct EventThread {
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl Drop for EventThread {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

impl Tuner {
    /// Open the device selected by `cfg` (`serial`, `recipient`) and initialise it.
    pub fn open(cfg: &TunerConfig, queue: TsQueue) -> Result<Self> {
        let t_start = Instant::now();
        let serial = if cfg.serial.is_empty() { None } else { Some(cfg.serial.as_str()) };
        let transport = UsbTransport::open_matching(cfg.recipient, serial)?;
        log::info!("usb open {:?} (recipient {:?})", t_start.elapsed(), cfg.recipient);
        Self::open_with(transport, cfg, queue)
    }

    /// Initialise an already opened device: device information of both tuner
    /// paths, terrestrial kit init, satellite kit init, device start (the
    /// original driver's order).
    pub fn open_with(transport: UsbTransport, cfg: &TunerConfig, queue: TsQueue) -> Result<Self> {
        let t_start = Instant::now();
        let pipes = transport.pipes();
        let pid = transport.product_id();
        log::info!("model {} ({pid:04x})", dth10_core::product_label(pid));
        log::info!("endpoints: {pipes:?}");

        let dev0 = Device::new(transport.clone());
        let info0 = dev0.read_device_information()?;
        log::info!(
            "tuner #0 kit {:?} model {:?} hw 0x{:x}",
            info0.desc.kit_name,
            info0.desc.model,
            info0.desc.hw_version
        );
        if let Ok((a, b, c)) = dev0.firmware_version() {
            log::info!("firmware {a}.{b}.{c}");
        }

        // Second tuner/filter exists when TunerDesc flags bit0 is set (as in the original driver).
        let satellite = if info0.desc.flags & 0x1 != 0 {
            let dev1 = Device::new(transport.clone()).with_tuner_index(1);
            match dev1.read_device_information() {
                Ok(info1) => {
                    log::info!(
                        "tuner #1 kit {:?} model {:?} hw 0x{:x}",
                        info1.desc.kit_name,
                        info1.desc.model,
                        info1.desc.hw_version
                    );
                    // The vendor driver sends the satellite kit's I2C over the tuner #0
                    // path when the descriptor's flags bit2 is set, else over the proxy
                    // path #1 (docs/20 §5.2).
                    let i2c_path = cfg.sat_i2c_path.unwrap_or(if info1.desc.flags & 0x4 != 0 { 0 } else { 1 });
                    let i2c_dev = Device::new(transport.clone()).with_tuner_index(i2c_path);
                    match Lj101Kit::new(i2c_dev, info1.desc.clone()) {
                        Ok(kit) => {
                            let endpoint = if cfg.sat_endpoint != 0 {
                                cfg.sat_endpoint
                            } else {
                                pipes.ts1.map(|(ep, _)| ep).unwrap_or(0x82)
                            };
                            log::info!(
                                "satellite: I2C via path {} (descriptor flags 0x{:x}), stream via path {} on EP 0x{endpoint:02x}",
                                i2c_path,
                                info1.desc.flags,
                                cfg.sat_stream_path
                            );
                            Some(Satellite {
                                kit,
                                stream_dev: Device::new(transport.clone()).with_tuner_index(cfg.sat_stream_path),
                                info: info1,
                                endpoint,
                                lnb_volt: 0,
                                initialised: false,
                                i2c_path,
                            })
                        }
                        Err(e) => {
                            log::warn!("tuner #1 not supported: {}", dth10_core::unverified_kit_error(pid, 1, e));
                            None
                        }
                    }
                }
                Err(e) => {
                    log::warn!("tuner #1 device information failed: {e}");
                    None
                }
            }
        } else {
            None
        };

        let mut terrestrial =
            TerrestrialKit::new(dev0, info0.desc.clone()).map_err(|e| dth10_core::unverified_kit_error(pid, 0, e))?;
        terrestrial.init()?;
        log::info!("terrestrial kit ({}) init done {:?}", terrestrial.kind(), t_start.elapsed());
        // The original driver initialises both kits at start-up, in order, and
        // only then sends the device start (USB trace of the original driver,
        // docs/40). A failure here is retried on the kit's first use.
        let mut satellite = satellite;
        if let Some(sat) = satellite.as_mut() {
            let t = Instant::now();
            match sat.kit.init() {
                Ok(()) => {
                    sat.initialised = true;
                    log::info!("satellite kit init done in {:?}", t.elapsed());
                }
                Err(e) => log::warn!("satellite kit init failed ({e}); retried on first use"),
            }
        }
        // As in the original driver, the device start (wValue 0 / wIndex 0x10) comes
        // after kit Init, followed by the kit's power-down when desc flags bit5 is set.
        terrestrial.dev().start()?;
        if info0.desc.skip_init_config() {
            terrestrial.dev().power_down(&info0.desc)?;
        }

        let log_events = cfg.log_events || std::env::var_os("DTH10_EVENTS").is_some();
        // The antenna supply of the model whose GPIO switches it ("h70"): the
        // device reports a possible over-current on the interrupt endpoint, and
        // the vendor driver then reads the supply's state 10 ms later and
        // switches it off when it is not good. Watched here the same way.
        let lnb_cut = Arc::new(AtomicBool::new(false));
        let lnb_watch = satellite
            .as_ref()
            .filter(|s| s.kit.desc.model_is("h70"))
            .map(|s| (Device::new(transport.clone()).with_tuner_index(s.i2c_path), lnb_cut.clone()));
        let mut events = None;
        if (log_events || lnb_watch.is_some())
            && let Some(addr) = pipes.interrupt_in
        {
            // Log the interrupt-endpoint events (card events 0xCBDA/0xCACD and
            // the like) while running, when asked to. A storm of them (the
            // device sends one every 0.5 s when its card engine is in trouble,
            // docs/40) is summarised once a minute instead of flooding the log.
            match transport.interrupt_in(addr) {
                Ok(ep) => {
                    let mut rd = dth10_usb::EventReader::new(ep);
                    let t0 = Instant::now();
                    let stop = Arc::new(AtomicBool::new(false));
                    let stop_thread = stop.clone();
                    let handle = std::thread::Builder::new()
                        .name("dth10-events".into())
                        .spawn(move || {
                            let (mut window, mut in_window, mut suppressed) = (Instant::now(), 0u32, 0u64);
                            while !stop_thread.load(Ordering::Relaxed) {
                                match rd.next(Duration::from_millis(1000)) {
                                    Ok(Some(ev)) => {
                                        if let Some((dev, cut)) = lnb_watch.as_ref()
                                            && lnb_overcurrent_report(&ev)
                                        {
                                            check_lnb_supply(dev, cut);
                                        }
                                        if !log_events {
                                            continue;
                                        }
                                        in_window += 1;
                                        if in_window <= 5 {
                                            log::info!(
                                                "event +{:.1}s targets {:#04b} class 0x{:04x} code 0x{:04x} payload {:02x?}",
                                                t0.elapsed().as_secs_f32(),
                                                ev.targets,
                                                ev.class,
                                                ev.code,
                                                ev.payload
                                            );
                                        } else {
                                            suppressed += 1;
                                        }
                                    }
                                    Ok(None) => {}
                                    Err(e) => {
                                        log::warn!("event read: {e}");
                                        std::thread::sleep(Duration::from_millis(500));
                                    }
                                }
                                if window.elapsed() >= Duration::from_secs(60) {
                                    if suppressed > 0 {
                                        log::warn!("events: {in_window} in the last minute ({suppressed} not shown): the device keeps raising card events");
                                    }
                                    window = Instant::now();
                                    in_window = 0;
                                    suppressed = 0;
                                }
                            }
                        })
                        .ok();
                    events = handle.map(|h| EventThread { stop, handle: Some(h) });
                }
                Err(e) => log::warn!("event endpoint: {e}"),
            }
        }
        let saved = cfg.cepm_state.as_ref().map(|p| cepm::load_set(p)).unwrap_or_default();
        for (f, hs) in &saved {
            log::info!("cepm: loaded saved handshake for path flags 0x{f:02x} (session id {:02x?})", hs.session_id);
        }
        Ok(Tuner {
            cfg: cfg.clone(),
            transport,
            terrestrial,
            info0,
            satellite,
            streams: [None, None],
            queues: [queue.clone(), queue],
            current: [None, None],
            last: None,
            cepm: Arc::new(Mutex::new(saved)),
            keys: None,
            lnb_cut,
            _events: events,
        })
    }

    /// Use already loaded application credentials instead of reading
    /// `TunerConfig::app_keys` on first use.
    pub fn with_app_keys(&mut self, keys: Arc<AppKeys>) {
        self.keys = Some(keys);
    }

    /// Send one system's TS to its own queue (both go to the queue given at
    /// open otherwise). Takes effect at that system's next `set_channel`.
    pub fn set_queue(&mut self, system: System, queue: TsQueue) {
        self.queues[slot(system)] = queue;
    }

    /// Change the stream mode for the following `set_channel` calls.
    pub fn set_stream_mode(&mut self, mode: StreamModeCfg) {
        self.cfg.stream_mode = mode;
    }

    /// Change the program the device descrambles for the following `set_channel` calls.
    pub fn set_device_program(&mut self, program: DeviceProgram) {
        self.cfg.device_program = program;
    }

    /// Change the antenna (LNB) supply for the following satellite tunes:
    /// 0 = off, else the voltage (110 / 150). A running satellite stream keeps
    /// its supply; the next satellite tune switches it.
    pub fn set_lnb(&mut self, volt: u32) {
        self.cfg.lnb_power = volt != 0;
        if volt != 0 {
            self.cfg.lnb_voltage = volt;
        }
    }

    /// Change how long the following tunes of `system` wait for lock.
    pub fn set_lock_wait_ms(&mut self, system: System, ms: u64) {
        match system {
            System::IsdbT => self.cfg.ter_lock_wait_ms = ms,
            System::IsdbS => self.cfg.sat_lock_wait_ms = ms,
        }
    }

    /// Stop one system's stream and forget its channel. With both systems
    /// idle the kits go to standby ([`Tuner::close`]); otherwise the other
    /// stream and its kit are left alone.
    pub fn release(&mut self, system: System) {
        self.stop_stream_of(system);
        self.current[slot(system)] = None;
        if self.last == Some(system) {
            self.last = [System::IsdbT, System::IsdbS].into_iter().find(|s| self.current[slot(*s)].is_some());
        }
        if self.streams.iter().all(Option::is_none) {
            self.close();
        }
    }

    /// The application credentials, read from `TunerConfig::app_keys` on first use.
    fn app_keys(&mut self) -> Result<Arc<AppKeys>> {
        if let Some(k) = self.keys.as_ref() {
            return Ok(k.clone());
        }
        let Some(path) = self.cfg.app_keys.as_ref() else {
            return Err(Error::AppKeys(
                "no application key file configured (the vendor application's CoBDACom.dll)".into(),
            ));
        };
        let t = Instant::now();
        let keys = Arc::new(AppKeys::load(path)?);
        log::info!("cepm: application credentials read from {} in {:?}", path.display(), t.elapsed());
        self.keys = Some(keys.clone());
        Ok(keys)
    }

    /// Request path of the CEPM context serving a stream: the stream of tuner
    /// #n is bound to the handshake done on tuner #n's request path (docs/22).
    fn cepm_flags_for(&self, system: System) -> u8 {
        match system {
            System::IsdbT => cepm::path_flags(0),
            System::IsdbS => cepm::path_flags(self.cfg.sat_stream_path),
        }
    }

    fn start_stream(&mut self, ep: u8, system: System, decrypt: bool, restart: Option<StreamRestart>) -> Result<()> {
        let reader = StreamReader::new(
            self.transport.bulk_in(ep)?,
            StreamReader::DEFAULT_TRANSFER_LEN,
            self.cfg.read_depth.max(1),
        )?;
        let name = if system == System::IsdbT { "dth10-ts0" } else { "dth10-ts1" };
        let flags = self.cepm_flags_for(system);
        let dec = if decrypt {
            let keys = self.app_keys()?;
            self.cepm
                .locked()
                .iter()
                .find(|(f, _)| *f == flags)
                .map(|(_, hs)| (hs.clone(), self.transport.clone(), keys.clone()))
        } else {
            None
        };
        let state = self.cfg.cepm_state.clone().map(|p| (p, self.cepm.clone()));
        self.streams[slot(system)] = Some(Stream::spawn(
            reader,
            self.queues[slot(system)].clone(),
            system,
            self.cfg.drop_filler,
            name,
            dec,
            flags,
            state,
            restart,
        )?);
        Ok(())
    }

    /// Device mode with link decryption: make sure a CEPM handshake exists for the
    /// stream's request path (docs/22). Failure is logged and the stream stays encrypted.
    fn ensure_cepm(&mut self, system: System) -> bool {
        if !self.cfg.device_decrypt {
            return false;
        }
        let flags = self.cepm_flags_for(system);
        let keys = match self.app_keys() {
            Ok(k) => k,
            Err(e) => {
                log::error!("cepm: {e}; device-mode stream will stay encrypted");
                return false;
            }
        };
        let mut set = self.cepm.locked();
        if !set.iter().any(|(f, _)| *f == flags) {
            let t = Instant::now();
            match Handshake::run_for_path(&self.transport, flags, &keys) {
                Ok(hs) => {
                    log::info!(
                        "cepm handshake (path flags 0x{flags:02x}) done in {:?}, session id {:02x?}",
                        t.elapsed(),
                        hs.session_id
                    );
                    set.push((flags, hs));
                    if let Some(p) = self.cfg.cepm_state.as_ref()
                        && let Err(e) = cepm::save_set(p, &set)
                    {
                        log::warn!("cepm: cannot save handshake state to {}: {e}", p.display());
                    }
                }
                Err(e) => log::warn!("cepm handshake failed ({e}); device-mode stream will stay encrypted"),
            }
        }
        set.iter().any(|(f, _)| *f == flags)
    }

    /// Stop one system's stream: the 0xCBDA/2 disable command first
    /// (RUN→PAUSE), then the bulk reads are cancelled (PAUSE→ACQUIRE), the
    /// original driver's order (docs/21 §4.1).
    fn stop_stream_of(&mut self, system: System) {
        let i = slot(system);
        if let Some(mut s) = self.streams[i].take() {
            debug_assert_eq!(s.system, system);
            let r = match system {
                System::IsdbT => self.terrestrial.dev().stream_disable(),
                System::IsdbS => self.satellite.as_ref().map_or(Ok(()), |sat| sat.stream_dev.stream_disable()),
            };
            if let Err(e) = r {
                log::warn!("stream disable: {e}");
            }
            s.stop();
        }
        self.queues[i].purge();
    }

    /// Stop both streams.
    fn stop_streams(&mut self) {
        self.stop_stream_of(System::IsdbT);
        self.stop_stream_of(System::IsdbS);
    }

    /// Stop streaming and put the kits to standby (LNB supply off).
    pub fn close(&mut self) {
        self.stop_streams();
        if let Err(e) = self.terrestrial.close() {
            log::warn!("terrestrial kit close: {e}");
        }
        if let Some(sat) = self.satellite.as_mut() {
            if sat.initialised
                && let Err(e) = sat.kit.close()
            {
                log::warn!("satellite kit close: {e}");
            }
            if sat.lnb_volt != 0 {
                let _ = sat.kit.lnb_power(0);
                sat.lnb_volt = 0;
            }
        }
        self.current = [None, None];
        self.last = None;
    }

    /// Device-side descrambling mode on the terrestrial stream: run the stream
    /// in raw mode briefly to learn PAT/PMT, then push the selected program's
    /// PID list (returned so the stream can be restarted with it later).
    fn learn_and_push_pids(
        &mut self,
        ep: u8,
        dev_for_stream: &Device<UsbTransport>,
        desc: &TunerDesc,
    ) -> Result<Vec<(u32, Vec<u16>)>> {
        let mut reader = StreamReader::new(
            self.transport.bulk_in(ep)?,
            StreamReader::DEFAULT_TRANSFER_LEN,
            self.cfg.read_depth.max(1),
        )?;
        let mut psi = PsiCollector::new();
        let mut aligner = Aligner::new(true);
        reader.start();
        dev_for_stream.stream_enable_mode(desc, StreamMode::RawFullTs)?;
        let deadline = Instant::now() + PSI_LEARN_TIMEOUT;
        while Instant::now() < deadline && !psi.complete() {
            if let Some(data) = reader.next(Duration::from_millis(200))? {
                psi.feed(&aligner.feed(&data));
            }
        }
        dev_for_stream.stream_disable()?;
        reader.stop();
        Ok(self.push_program_lists(dev_for_stream, psi.programs()))
    }

    /// PSI learn in the device's own mode (link-encrypted, one program
    /// descrambled by the built-in card): the PSI is read through the link
    /// decryption with the handshake of the stream's request path. A stale
    /// handshake is renewed on the way and the new one stored. Used for the
    /// satellite stream (which has no raw mode) and, by default, for the
    /// terrestrial one too, because a raw enable on stream #0 stops the
    /// satellite stream until the next power cycle (docs/40).
    fn learn_and_push_pids_encrypted(
        &mut self,
        ep: u8,
        dev_for_stream: &Device<UsbTransport>,
        desc: &TunerDesc,
        flags: u8,
    ) -> Result<Vec<(u32, Vec<u16>)>> {
        let keys = self.app_keys()?;
        let hs = self.cepm.locked().iter().find(|(f, _)| *f == flags).map(|(_, hs)| hs.clone());
        let Some(hs) = hs else {
            return Err(Error::Protocol("no CEPM handshake for this stream path; cannot read its PSI"));
        };
        let mut dec = Decryptor::new(hs, keys);
        let mut reader = StreamReader::new(
            self.transport.bulk_in(ep)?,
            StreamReader::DEFAULT_TRANSFER_LEN,
            self.cfg.read_depth.max(1),
        )?;
        let mut psi = PsiCollector::new();
        let mut aligner = Aligner::new(true);
        reader.start();
        dev_for_stream.stream_enable_mode(desc, StreamMode::EncryptedPartial)?;
        let t = Instant::now();
        let deadline = t + PSI_LEARN_TIMEOUT_ENCRYPTED;
        while Instant::now() < deadline && !psi.complete() {
            if let Some(data) = reader.next(Duration::from_millis(200))? {
                let mut chunk = aligner.feed(&data);
                for pkt in chunk.as_chunks_mut::<TS_PACKET>().0 {
                    match dec.process(&self.transport, flags, pkt) {
                        Ok(true) => psi.feed(pkt),
                        Ok(false) => {}
                        Err(e) => log::debug!("cepm (psi learn): {e}"),
                    }
                }
                if let Some(hs) = dec.take_new_handshake() {
                    // The device keeps wrapping content keys for the session that
                    // was current when the stream was enabled (docs/22 §6c): restart
                    // it so the keys come under the new session.
                    self.store_handshake(flags, hs);
                    dev_for_stream.stream_disable()?;
                    dev_for_stream.stream_enable_mode(desc, StreamMode::EncryptedPartial)?;
                    log::info!("cepm: stream restarted for the new session (PSI learn)");
                }
            }
        }
        dev_for_stream.stream_disable()?;
        reader.stop();
        log::info!("PSI learned through the link decryption in {:?} (complete: {})", t.elapsed(), psi.complete());
        Ok(self.push_program_lists(dev_for_stream, psi.programs()))
    }

    /// Replace the stored handshake of a request path and persist the set.
    fn store_handshake(&self, flags: u8, hs: Handshake) {
        let mut set = self.cepm.locked();
        set.retain(|(f, _)| *f != flags);
        set.push((flags, hs));
        if let Some(p) = self.cfg.cepm_state.as_ref()
            && let Err(e) = cepm::save_set(p, &set)
        {
            log::warn!("cepm: cannot save handshake state to {}: {e}", p.display());
        }
    }

    /// Push the PID list of the program(s) selected by `TunerConfig::device_program`.
    /// The device descrambles exactly one program: it reads the program number
    /// of the list and ignores the PIDs themselves (docs/20 §6), so `all` ends
    /// up with the last program pushed.
    fn push_program_lists(
        &self,
        dev_for_stream: &Device<UsbTransport>,
        programs: Vec<Program>,
    ) -> Vec<(u32, Vec<u16>)> {
        if programs.is_empty() {
            log::warn!("device mode: no PAT/PMT learned, streaming without a PID list");
        }
        let (selected, note) = select_programs(&programs, self.cfg.device_program);
        if let Some((level, msg)) = note {
            log::log!(level, "{msg}");
        }
        let mut lists = Vec::new();
        for p in selected {
            let pids = p.pid_list();
            log::info!("device mode: program {} (0x{:04x}) pids {:04x?}", p.number, p.number, pids);
            if let Err(e) = dev_for_stream.set_program_pids(p.number as u32, &pids) {
                log::warn!("set_program_pids 0x{:04x}: {e}", p.number);
            }
            lists.push((p.number as u32, pids));
        }
        lists
    }

    fn wait_lock<F: FnMut() -> Result<SignalStatus>>(
        mut status: F,
        timeout: Duration,
        what: &str,
    ) -> Option<SignalStatus> {
        let deadline = Instant::now() + timeout;
        loop {
            match status() {
                Ok(st) if st.lock & lock::LOCKED != 0 => return Some(st),
                Ok(_) => {}
                Err(e) => {
                    log::debug!("status during lock wait: {e}");
                    return None;
                }
            }
            if Instant::now() >= deadline {
                log::info!("{what}: no lock within {timeout:?}");
                return None;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    /// Tune to `ch` and start streaming into the queue.
    pub fn set_channel(&mut self, ch: &Channel) -> Result<()> {
        log::info!("set_channel {:?} {} Hz tsid 0x{:04x} slot {:?}", ch.system, ch.frequency_hz, ch.tsid, ch.slot);
        let pipes = self.transport.pipes();
        let device_mode = self.cfg.stream_mode == StreamModeCfg::Device;
        let i = slot(ch.system);
        if self.cfg.exclusive {
            // One stream at a time: the other system's stream ends here and
            // its kit goes to standby below.
            let other = if ch.system == System::IsdbT { System::IsdbS } else { System::IsdbT };
            self.stop_stream_of(other);
            self.current[slot(other)] = None;
        }
        if device_mode {
            // The PID list is per channel: the stream is always started over.
            self.stop_stream_of(ch.system);
        }
        match ch.system {
            System::IsdbT => {
                let (ep, _) = pipes.ts0.ok_or(Error::Protocol("no TS endpoint for tuner #0"))?;
                if let Some(sat) = self.satellite.as_mut()
                    && sat.initialised
                    && self.cfg.exclusive
                    && self.streams.iter().all(Option::is_none)
                {
                    let _ = sat.kit.close();
                }
                if !self.terrestrial.is_open() {
                    self.terrestrial.open()?;
                }
                let req = TuneRequest::isdb_t(ch.frequency_hz as u64, self.cfg.ter_bandwidth_hz);
                self.terrestrial.tune(&req)?;
                let wait = Duration::from_millis(self.cfg.ter_lock_wait_ms);
                let ter = &mut self.terrestrial;
                if Self::wait_lock(|| ter.signal_status(field::LOCK), wait, "ISDB-T").is_none() {
                    // Without lock there is nothing to learn or stream; failing here
                    // keeps a channel without signal from costing the PSI learn's
                    // full wait (and TVTest's channel scan from crawling).
                    return Err(Error::Protocol("ISDB-T: no lock (no signal on this channel)"));
                }
                if self.streams[0].is_none() {
                    let desc = self.info0.desc.clone();
                    let mode = if device_mode { StreamMode::EncryptedPartial } else { StreamMode::RawFullTs };
                    if !device_mode && self.satellite.is_some() {
                        // docs/40: once stream #0 ran in raw mode, the next satellite
                        // enable hangs the device until it is power-cycled.
                        log::warn!(
                            "terrestrial raw mode: BS/CS channels will not work on this device until it is power-cycled"
                        );
                    }
                    let mut restart = None;
                    if device_mode {
                        let dev = Device::new(self.transport.clone());
                        let pids = if self.cfg.device_decrypt {
                            // Learn the PSI in the device's own mode through the link
                            // decryption: a raw (byte0=1) enable on stream #0 would stop
                            // the satellite stream until the next power cycle (docs/40),
                            // so a failed handshake fails the tune rather than fall back.
                            if !self.ensure_cepm(System::IsdbT) {
                                return Err(Error::Protocol(
                                    "CEPM handshake for the terrestrial stream failed (application key file?); no raw fallback, that would stop BS/CS until a power cycle",
                                ));
                            }
                            let flags = self.cepm_flags_for(System::IsdbT);
                            self.learn_and_push_pids_encrypted(ep, &dev, &desc, flags)?
                        } else {
                            if self.satellite.is_some() {
                                log::warn!(
                                    "terrestrial PSI learn in raw mode: BS/CS channels will not work on this device until it is power-cycled"
                                );
                            }
                            self.learn_and_push_pids(ep, &dev, &desc)?
                        };
                        restart = Some(StreamRestart { dev, desc: desc.clone(), mode, pids });
                    }
                    let decrypt = device_mode && self.ensure_cepm(System::IsdbT);
                    let lists = restart.as_ref().map(|r| r.pids.clone()).unwrap_or_default();
                    // KS pin order (docs/21): bulk reads start before the stream-enable command.
                    self.start_stream(ep, System::IsdbT, decrypt, restart)?;
                    let dev = self.terrestrial.dev();
                    dev.stream_enable_mode(&desc, mode).map_err(|e| stream_enable_error(mode, e))?;
                    send_program_lists(dev, &lists);
                }
            }
            System::IsdbS => {
                if self.cfg.exclusive {
                    let _ = self.terrestrial.close();
                }
                let lnb_volt = if self.cfg.lnb_power { self.cfg.lnb_voltage } else { 0 };
                let sat_wait = Duration::from_millis(self.cfg.sat_lock_wait_ms);
                let lnb_was_cut = self.lnb_cut.swap(false, Ordering::Relaxed);
                let sat = self.satellite.as_mut().ok_or(Error::Unsupported("no ISDB-S tuner on this device".into()))?;
                if lnb_was_cut {
                    // switched off after an over-current report: on again below
                    sat.lnb_volt = 0;
                }
                let ep = sat.endpoint;
                if !sat.initialised {
                    let t = Instant::now();
                    sat.kit.init()?;
                    sat.initialised = true;
                    log::info!("satellite kit init done in {:?}", t.elapsed());
                }
                if !sat.kit.is_open() {
                    sat.kit.open()?;
                }
                if sat.lnb_volt != lnb_volt {
                    if !sat.kit.lnb_power(lnb_volt)? {
                        log::warn!("LNB supply reported unhealthy (over-current?), switched off");
                    }
                    sat.lnb_volt = lnb_volt;
                }
                sat.kit.tune(ch.frequency_hz as u64)?;
                // Wait for lock, then pick the TSID: the TMCC table entry of the
                // relative TS number when the channel names one and the table
                // has it, else the channel's own TSID.
                let mut tsid = ch.tsid;
                let kit = &mut sat.kit;
                if let Some(st) =
                    Self::wait_lock(|| kit.signal_status(field::ALL_BASIC | field::TSID_TABLE), sat_wait, "ISDB-S")
                {
                    let ids = &st.tsids[..(st.tsid_count as usize).min(st.tsids.len())];
                    log::info!("ISDB-S locked, C/N {:.1} dB, TMCC TS-IDs {:04x?}", st.cnr_mdb as f64 / 1000.0, ids);
                    if st.mask & field::TSID_TABLE != 0 {
                        if let Some(slot) = ch.slot {
                            match ids.get(slot as usize) {
                                Some(&id) if id != 0 && id != 0xFFFF => tsid = id,
                                _ => log::warn!(
                                    "ISDB-S: relative TS {slot} not in the TMCC table, selecting TSID 0x{tsid:04x}"
                                ),
                            }
                        } else if !ids.contains(&tsid) {
                            log::warn!("ISDB-S: TSID 0x{tsid:04x} is not in the TMCC table {ids:04x?}");
                        }
                    }
                } else {
                    // No lock at all (no antenna, no signal): fail at once rather than
                    // spend the PSI learn's full wait on an empty stream.
                    return Err(Error::Protocol("ISDB-S: no lock (no signal, or no antenna)"));
                }
                sat.kit.set_tsid(tsid)?;
                log::info!("ISDB-S: TSID 0x{tsid:04x} selected");
                if self.streams[1].is_none() {
                    let desc = sat.info.desc.clone();
                    log::info!("satellite enable payload: desc id {} stream_param 0x{:x}", desc.id, desc.stream_param);
                    // The device outputs the satellite stream only in the original
                    // driver's mode (link-encrypted, one program descrambled by the
                    // built-in card); no first byte of the enable command gives a raw
                    // stream (docs/40). Satellite channels therefore always run in
                    // device mode with link decryption, whatever `stream_mode` says.
                    if !device_mode {
                        log::warn!(
                            "satellite: the device has no raw mode on this stream; using device mode (one program, descrambled by the device)"
                        );
                    }
                    if !self.cfg.device_decrypt {
                        return Err(Error::Unsupported(
                            "satellite channels need link decryption (device mode with the application key file)"
                                .into(),
                        ));
                    }
                    let mode = StreamMode::EncryptedPartial;
                    let flags = self.cepm_flags_for(System::IsdbS);
                    if !self.ensure_cepm(System::IsdbS) {
                        return Err(Error::Protocol("CEPM handshake for the satellite stream failed"));
                    }
                    let dev = Device::new(self.transport.clone()).with_tuner_index(self.cfg.sat_stream_path);
                    let pids = self.learn_and_push_pids_encrypted(ep, &dev, &desc, flags)?;
                    let lists = pids.clone();
                    let restart = Some(StreamRestart { dev, desc: desc.clone(), mode, pids });
                    let decrypt = true;
                    self.start_stream(ep, System::IsdbS, decrypt, restart)?;
                    let sat = self.satellite.as_mut().expect("checked above");
                    sat.stream_dev.stream_enable_mode(&desc, mode)?;
                    send_program_lists(&sat.stream_dev, &lists);
                }
            }
        }
        self.current[i] = Some(ch.clone());
        self.last = Some(ch.system);
        Ok(())
    }

    /// C/N in dB of the last tuned system's channel (0.0 when unavailable).
    pub fn signal_level(&mut self) -> f32 {
        match self.last {
            Some(system) => self.signal_level_of(system),
            None => 0.0,
        }
    }

    /// C/N in dB of one system's demodulator (0.0 when unavailable).
    pub fn signal_level_of(&mut self, system: System) -> f32 {
        let r = match system {
            System::IsdbT => self.terrestrial.signal_status(field::ALL_BASIC),
            System::IsdbS => match self.satellite.as_mut() {
                Some(sat) => sat.kit.signal_status(field::ALL_BASIC),
                None => return 0.0,
            },
        };
        match r {
            Ok(st) => st.cnr_mdb as f32 / 1000.0,
            Err(e) => {
                log::debug!("signal status: {e}");
                0.0
            }
        }
    }

    /// Full signal status of the last tuned system's demodulator.
    pub fn status(&mut self) -> Result<SignalStatus> {
        match self.last {
            Some(System::IsdbS) => match self.satellite.as_mut() {
                Some(sat) => sat.kit.signal_status(field::ALL | field::TSID_TABLE),
                None => Err(Error::Unsupported("no ISDB-S tuner".into())),
            },
            _ => self.terrestrial.signal_status(field::ALL),
        }
    }

    /// Device information of tuner #0.
    pub fn info(&self) -> &DeviceInformation {
        &self.info0
    }

    /// Device information of tuner #1, when the device has one.
    pub fn info1(&self) -> Option<&DeviceInformation> {
        self.satellite.as_ref().map(|s| &s.info)
    }

    /// Whether the device advertises a satellite tuner this crate can drive.
    pub fn has_satellite(&self) -> bool {
        self.satellite.is_some()
    }

    pub fn transport(&self) -> &UsbTransport {
        &self.transport
    }

    /// The queue one system's TS goes to.
    pub fn queue(&self, system: System) -> &TsQueue {
        &self.queues[slot(system)]
    }

    /// The channel of the last `set_channel`.
    pub fn current(&self) -> Option<&Channel> {
        self.last.and_then(|s| self.current[slot(s)].as_ref())
    }

    /// One system's stream has given itself up (its bulk reads kept failing:
    /// the device is gone). The stream counts as running until released.
    pub fn stream_failed(&self, system: System) -> bool {
        self.streams[slot(system)].as_ref().is_some_and(|s| s.failed.load(Ordering::Relaxed))
    }

    /// The channel one system is tuned to.
    pub fn current_of(&self, system: System) -> Option<&Channel> {
        self.current[slot(system)].as_ref()
    }
}

/// The program(s) [`Tuner::push_program_lists`] hands to the device for a
/// [`DeviceProgram`] choice, plus what the log should say about the others.
///
/// The device descrambles one program per tuning and outputs only that
/// program's elementary streams (with the PSI of the whole TS), so a client
/// that asks for a channel without naming a program (Mirakurun's
/// `/api/channels/<type>/<channel>/stream`, a channels.yml entry without a
/// `.<service id>` suffix) gets the first program and no video/audio for the
/// others: the note names them, so that the log shows why a service of the
/// channel stays black.
fn select_programs(programs: &[Program], choice: DeviceProgram) -> (Vec<&Program>, Option<(log::Level, String)>) {
    let others = |skip: u16| {
        programs.iter().filter(|p| p.number != skip).map(|p| p.number.to_string()).collect::<Vec<_>>().join(", ")
    };
    match choice {
        DeviceProgram::All => (programs.iter().collect(), None),
        DeviceProgram::First => {
            let note = match programs.first() {
                Some(p) if programs.len() > 1 => {
                    let n = p.number;
                    let msg = format!(
                        "device mode: no program named, descrambling the first one, {n} (0x{n:04x}); programs {} have no video/audio in this stream",
                        others(n)
                    );
                    Some((
                        log::Level::Info,
                        format!("{msg} (name one with --program <service id> or a .<service id> channel suffix)"),
                    ))
                }
                _ => None,
            };
            (programs.iter().take(1).collect(), note)
        }
        DeviceProgram::Id(id) => {
            let v: Vec<&Program> = programs.iter().filter(|p| p.number == id).collect();
            if v.is_empty() && !programs.is_empty() {
                let note = format!(
                    "device mode: program {id} (0x{id:04x}) is not in the PAT (programs {}), descrambling the first one",
                    others(id)
                );
                (programs.iter().take(1).collect(), Some((log::Level::Warn, note)))
            } else {
                (v, None)
            }
        }
    }
}

/// Index of a system's stream, queue and channel.
fn slot(system: System) -> usize {
    match system {
        System::IsdbT => 0,
        System::IsdbS => 1,
    }
}

impl Drop for Tuner {
    fn drop(&mut self) {
        self.close();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aligner_extracts_188_of_256() {
        let mut a = Aligner::new(true);
        let mut data = vec![0xEEu8; 2 * BLOCK + 10];
        data[0] = 0x47;
        data[BLOCK] = 0x47;
        data[187] = 1;
        data[188] = 2; // padding, must not appear
        let out = a.feed(&data);
        assert_eq!(out.len(), 2 * TS_PACKET);
        assert_eq!(out[0], 0x47);
        assert_eq!(out[187], 1);
        assert_eq!(out[188], 0x47);
        assert!(!out[188..].contains(&2));
        // a block without the sync byte is dropped
        data[BLOCK] = 0x00;
        let out = a.feed(&data);
        assert_eq!(out.len(), TS_PACKET);
        assert_eq!(a.unsynced, 1);
    }

    #[test]
    fn aligner_drops_tei_null_filler() {
        let mut a = Aligner::new(true);
        let mut data = vec![0xFFu8; 2 * BLOCK];
        data[0] = 0x47;
        data[1] = 0x9F;
        data[2] = 0xFF;
        data[3] = 0x1F; // filler
        data[BLOCK] = 0x47;
        data[BLOCK + 1] = 0x01;
        data[BLOCK + 2] = 0x00; // real packet
        let out = a.feed(&data);
        assert_eq!(out.len(), TS_PACKET);
        assert_eq!(out[1], 0x01);
        let mut keep = Aligner::new(false);
        assert_eq!(keep.feed(&data).len(), 2 * TS_PACKET);
    }

    fn program(number: u16) -> Program {
        Program {
            number,
            pmt_pid: 0x400 + number % 16,
            pcr_pid: 0x1FFF,
            ecm_pids: vec![],
            es_pids: vec![0x1000 + number % 16],
        }
    }

    #[test]
    fn program_selection() {
        // CS110 ND16 as received: AT-X is service 333, the PAT starts with 290.
        let progs: Vec<Program> = [290, 311, 316, 321, 333, 353].into_iter().map(program).collect();
        let numbers = |v: &[&Program]| v.iter().map(|p| p.number).collect::<Vec<_>>();

        let (sel, note) = select_programs(&progs, DeviceProgram::Id(333));
        assert_eq!((numbers(&sel), note), (vec![333], None));

        let (sel, note) = select_programs(&progs, DeviceProgram::First);
        assert_eq!(numbers(&sel), vec![290]);
        let (level, msg) = note.expect("the programs left out are logged");
        assert_eq!(level, log::Level::Info);
        assert!(
            msg.contains(
                "first one, 290 (0x0122); programs 311, 316, 321, 333, 353 have no video/audio in this stream (name one"
            ),
            "{msg}"
        );

        let (sel, note) = select_programs(&progs, DeviceProgram::Id(0x333));
        assert_eq!(numbers(&sel), vec![290]);
        let (level, msg) = note.expect("a missing program is logged");
        assert_eq!(level, log::Level::Warn);
        assert!(
            msg.contains("program 819 (0x0333) is not in the PAT (programs 290, 311, 316, 321, 333, 353)"),
            "{msg}"
        );

        assert_eq!(numbers(&select_programs(&progs, DeviceProgram::All).0), vec![290, 311, 316, 321, 333, 353]);

        // One program: nothing is left out, nothing to say.
        let single = [program(101)];
        assert_eq!(select_programs(&single, DeviceProgram::First).1, None);
        // Nothing learned: nothing selected (push_program_lists warns about that).
        assert_eq!(select_programs(&[], DeviceProgram::Id(333)), (vec![], None));
        assert_eq!(select_programs(&[], DeviceProgram::First), (vec![], None));
    }

    #[test]
    fn lnb_overcurrent_reports() {
        use dth10_core::events::Event;
        let ev = |class, code, targets, payload: &[u8]| Event { targets, class, code, payload: payload.to_vec() };
        assert!(lnb_overcurrent_report(&ev(0, 0x1011, 0b10, &[0, 0, 0, 0, 1])));
        assert!(lnb_overcurrent_report(&ev(0, 0x1011, 0b11, &[0, 0, 0, 0, 0x81])));
        // for the terrestrial filter only, bit 0 clear, too short, or another event
        assert!(!lnb_overcurrent_report(&ev(0, 0x1011, 0b01, &[0, 0, 0, 0, 1])));
        assert!(!lnb_overcurrent_report(&ev(0, 0x1011, 0b10, &[0, 0, 0, 0, 2])));
        assert!(!lnb_overcurrent_report(&ev(0, 0x1011, 0b10, &[0, 0, 0, 0])));
        assert!(!lnb_overcurrent_report(&ev(0xCBDA, 0xCACD, 0b10, &[0, 0, 0, 0, 1])));
    }

    #[test]
    fn queue_wait_and_limit() {
        let q = TsQueue::new();
        assert!(!q.wait(0));
        q.push(vec![0x47; 188]);
        assert!(q.wait(0));
        assert_eq!(q.pop().map(|c| c.len()), Some(188));
        assert!(q.pop().is_none());
        for _ in 0..40 {
            q.push(vec![0; 1024 * 1024]);
        }
        assert!(q.len() <= 32);
    }
}
