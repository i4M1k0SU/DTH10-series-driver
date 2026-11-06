//! `dth10 serve`: hold the tuner(s) open for good and stream to clients over
//! a socket (`dth10_tuner::ipc`: a Unix domain socket on Linux and Windows
//! alike, or TCP on the loopback interface; on Windows the BonDriver starts
//! this as its server).
//!
//! The device's terrestrial and satellite streams are independent here
//! (docs/40): a recorder lists one unit as two tuners, one for GR and one for
//! BS/CS, and both can run at the same time; two TVTest windows do the same
//! through two copies of the BonDriver. A client only asks for a stream and
//! reads it. When the client goes away (a recorder kills `dth10 stream`,
//! TVTest closes), the connection closes and the daemon stops that stream
//! while the other one goes on; with both idle the kits go to standby. The
//! CEPM sessions, the application credentials and the kits' state stay in
//! this process between requests, so a request takes about a second instead
//! of the device start of `dth10 tune`. A request for a stream that another
//! client holds takes it over (the older client's connection ends) unless it
//! says `busy=fail`; the same client (by the `client=` id it gives) replaces
//! its own stream at once. A unit that goes away (USB error) is opened again
//! on the next request for it. Without any device the daemon still listens
//! and looks for one every few seconds (a box may start it before the USB
//! bus has enumerated the tuner), and a unit plugged in later is served too.
//! With `--idle-exit` the daemon ends when it has had no client for that long
//! (the server a BonDriver starts). The application key file (the
//! vendor's `CoBDACom.dll`) is read when it is there, or when a request first
//! needs it; without it the daemon still serves raw terrestrial streams and
//! refuses device-mode and BS/CS requests with the reason.

use std::error::Error;
use std::io::{self, BufRead, BufReader, Read, Write};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use clap::Args;
use dth10_core::LockExt;
use dth10_core::cepm::AppKeys;
use dth10_tuner::{DeviceProgram, StreamModeCfg, System, TsQueue, Tuner, TunerConfig};
use dth10_usb::{RecipientMode, UsbTransport};

use crate::{RecipientArg, app_keys_path, default_state_dir, lnb_of, state_file_for, stop};
use dth10_tuner::ipc::{Conn, Listener, Target};
use dth10_tuner::proto::{Holder, MAX_LINE, Reply, Request, Status, StreamRequest};

#[derive(Args)]
pub struct ServeArgs {
    /// Listen on this socket file (default: $DTH10_SOCKET, else /run/dth10/dth10.sock on Linux,
    /// dth10.sock next to this executable on Windows).
    #[arg(long, value_name = "PATH")]
    socket: Option<PathBuf>,
    /// Listen on this TCP address instead (e.g. 127.0.0.1:40780; $DTH10_LISTEN).
    #[arg(long, value_name = "ADDR", conflicts_with = "socket")]
    listen: Option<String>,
    /// Serve this unit, by USB serial number (repeatable). Default: every supported device found.
    #[arg(long, value_name = "SERIAL")]
    serial: Vec<String>,
    /// bmRequestType recipient of the vendor requests (default: device on Windows, interface elsewhere).
    #[arg(long, value_enum)]
    recipient: Option<RecipientArg>,
    /// Directory of the CEPM handshake cache (DTH10_<serial>.cepm). Default: $DTH10_STATE_DIR,
    /// else /var/lib/dth10 when it is writable, else the directory of this executable.
    #[arg(long, value_name = "DIR")]
    state_dir: Option<PathBuf>,
    /// The vendor application's CoBDACom.dll (PCastTV3), from which the application
    /// certificate, key and IV are read. Default: $DTH10_APP_KEYS, else CoBDACom.dll in
    /// the state directory, else next to this executable.
    #[arg(long, value_name = "FILE")]
    app_keys: Option<PathBuf>,
    /// Antenna (LNB) supply for BS/CS: 0 (off), 110 (11 V) or 150 (15 V). The DT-H70/U2
    /// has no such supply (the setting is for family models that have one).
    #[arg(long, value_name = "V", default_value_t = 0)]
    lnb: u32,
    /// ISDB-T: how long to wait for lock before streaming [ms].
    #[arg(long, value_name = "MS", default_value_t = 1200)]
    ter_lock_wait: u64,
    /// ISDB-S: how long to wait for lock and the TMCC TS-ID table [ms].
    #[arg(long, value_name = "MS", default_value_t = 1500)]
    sat_lock_wait: u64,
    /// Bulk IN transfers kept in flight per stream (39936 bytes each; default 16 on Linux, 4 on Windows).
    #[arg(long, value_name = "N")]
    read_depth: Option<usize>,
    /// End when no client has been connected for this many seconds (0 = run until stopped).
    #[arg(long, value_name = "SECONDS", default_value_t = 0)]
    idle_exit: u64,
}

/// The application credentials (the vendor application's `CoBDACom.dll`):
/// needed for device mode and BS/CS (the link decryption), not for raw
/// terrestrial streams. Read at start when the file is there, else on the
/// first request that needs it (the file may be put in place later), so a
/// daemon that only serves raw streams runs without it.
struct Keys {
    explicit: Option<PathBuf>,
    state_dir: PathBuf,
    loaded: Mutex<Option<Arc<AppKeys>>>,
}

impl Keys {
    /// Where the file is (or would be looked for) now.
    fn resolve(&self) -> Option<PathBuf> {
        app_keys_path(self.explicit.as_deref(), &self.state_dir)
    }

    fn loaded(&self) -> Option<Arc<AppKeys>> {
        self.loaded.locked().clone()
    }

    /// The credentials, read on first use.
    fn get(&self) -> Result<Arc<AppKeys>, String> {
        let mut l = self.loaded.locked();
        if let Some(k) = l.as_ref() {
            return Ok(k.clone());
        }
        let Some(p) = self.resolve() else {
            return Err(format!(
                "the application key file (the vendor application's CoBDACom.dll) is needed for device mode and BS/CS: put it in {} or give --app-keys",
                self.state_dir.display()
            ));
        };
        let t = Instant::now();
        let k = Arc::new(AppKeys::load(&p).map_err(|e| format!("application key file {}: {e}", p.display()))?);
        log::info!("application credentials read from {} in {:?}", p.display(), t.elapsed());
        *l = Some(k.clone());
        Ok(k)
    }
}

/// One served unit.
struct Unit {
    serial: String,
    cfg: TunerConfig,
    keys: Arc<Keys>,
    recipient: RecipientMode,
    /// The TS of each system: 0 = terrestrial, 1 = satellite.
    queues: [TsQueue; 2],
    /// `None` while the unit is not open (lost; opened again on the next request).
    tuner: Mutex<Option<Tuner>>,
    /// Per system: the connection streaming it and the flag that ends that connection's stream.
    slots: [Mutex<Slot>; 2],
    /// The last C/N read of each system, answered to a status request while
    /// the tuner is busy with a tune (a tune can take seconds; a client's
    /// signal-level poll must not wait for it).
    cnr: Mutex<Cnr>,
    /// Stream requests tuning the unit now (a status reply says `busy` then;
    /// the tuner is also locked briefly by the periodic status read).
    tuning: AtomicUsize,
}

/// The C/N of each system as last read, and when the status was last read.
#[derive(Default)]
struct Cnr {
    ter: Option<f32>,
    sat: Option<f32>,
    /// The last full status read (a client's status request or the periodic read).
    read: Option<Instant>,
}

impl Cnr {
    fn set(&mut self, system: System, cnr: Option<f32>) {
        match system {
            System::IsdbT => self.ter = cnr,
            System::IsdbS => self.sat = cnr,
        }
    }
}

/// How often the status of a unit that streams is read. Some kits follow the
/// carrier offset and acquire the signal again after a lost lock only inside
/// the status read (as in the vendor driver, whose application keeps reading
/// it during playback), so it is read with or without a client asking.
const STATUS_POLL: Duration = Duration::from_secs(1);
/// How often the periodic read looks at the units.
const STATUS_TICK: Duration = Duration::from_millis(250);

/// Whether the periodic read is due: a stream runs and no read (a client's
/// status request counts) happened within [`STATUS_POLL`].
fn status_due(streaming: bool, last_read: Option<Instant>, now: Instant) -> bool {
    streaming && last_read.is_none_or(|t| now.saturating_duration_since(t) >= STATUS_POLL)
}

#[derive(Default)]
struct Slot {
    /// Connection id, 0 = free.
    owner: u64,
    stop: Option<Arc<AtomicBool>>,
    /// The holder's `client=` id, when it gave one.
    client: Option<String>,
    /// The channel name of the holder's request (for status replies).
    channel: String,
}

/// What a stream request may do with a slot.
#[derive(Debug, PartialEq, Eq)]
enum Claim {
    /// Nobody streams it.
    Free,
    /// The same client (by its id) replaces its own stream: a channel change
    /// must not wait for the end of its old connection to be noticed.
    Replace,
    /// Another client's stream is taken over (its connection ends).
    TakeOver,
    /// Another client holds it and the request said `busy=fail`.
    Busy,
}

fn claim(slot: &Slot, busy_fail: bool, client: Option<&str>) -> Claim {
    if slot.owner == 0 {
        Claim::Free
    } else if client.is_some() && slot.client.as_deref() == client {
        Claim::Replace
    } else if busy_fail {
        Claim::Busy
    } else {
        Claim::TakeOver
    }
}

fn slot(system: System) -> usize {
    match system {
        System::IsdbT => 0,
        System::IsdbS => 1,
    }
}

fn system_name(system: System) -> &'static str {
    match system {
        System::IsdbT => "ISDB-T",
        System::IsdbS => "ISDB-S",
    }
}

impl Unit {
    /// Open the device unless it is open already.
    fn open(&self, tuner: &mut Option<Tuner>) -> Result<(), String> {
        if tuner.is_some() {
            return Ok(());
        }
        let t0 = Instant::now();
        let transport = UsbTransport::open_matching(self.recipient, Some(&self.serial))
            .map_err(|e| format!("unit {}: {e}", self.serial))?;
        let mut t = Tuner::open_with(transport, &self.cfg, self.queues[0].clone())
            .map_err(|e| format!("unit {}: {e}", self.serial))?;
        t.set_queue(System::IsdbS, self.queues[1].clone());
        if let Some(k) = self.keys.loaded() {
            t.with_app_keys(k);
        }
        log::info!("unit {}: opened in {:?}", self.serial, t0.elapsed());
        *tuner = Some(t);
        Ok(())
    }
}

/// The unit a request names, or the only one.
fn pick_unit<'a>(units: &'a [Arc<Unit>], serial: Option<&str>) -> Result<&'a Arc<Unit>, String> {
    let served = || units.iter().map(|u| u.serial.as_str()).collect::<Vec<_>>().join(", ");
    if units.is_empty() {
        return Err("no unit yet (no supported device found; waiting for one)".into());
    }
    match serial {
        Some(s) => units.iter().find(|u| u.serial == s).ok_or_else(|| format!("no unit {s} (served: {})", served())),
        None if units.len() == 1 => Ok(&units[0]),
        None => Err(format!("several units are served, give unit=<serial>: {}", served())),
    }
}

fn reply(conn: &Conn, r: &Reply) -> io::Result<()> {
    let mut c = conn;
    c.write_all(format!("{}\n", r.to_line()).as_bytes())
}

/// Shared between the accept loop and the connections: how many are being
/// served and when the last one ended (for `--idle-exit`).
struct Activity {
    active: AtomicUsize,
    last_end: Mutex<Instant>,
}

/// One client: read the request; a status request is answered at once, a
/// stream request takes the stream over, tunes, pumps the TS until the client
/// goes away (or a newer client takes the stream), then releases.
fn serve_connection(conn: Conn, id: u64, units: &Units) {
    let _ = conn.set_read_timeout(Some(Duration::from_secs(5)));
    let mut line = String::new();
    {
        let mut r = BufReader::new(&conn);
        if let Err(e) = (&mut r).take(MAX_LINE as u64).read_line(&mut line) {
            log::warn!("connection {id}: request: {e}");
            return;
        }
    }
    let req = match Request::parse(line.trim_end()) {
        Ok(r) => r,
        Err(e) => {
            log::warn!("connection {id}: bad request {:?}: {e}", line.trim_end());
            let _ = reply(&conn, &Reply::Error(e));
            return;
        }
    };
    let (unit_name, req) = match req {
        Request::Status { unit } => (unit, None),
        Request::Stream(r) => (r.unit.clone(), Some(r)),
    };
    let units: Vec<Arc<Unit>> = units.locked().clone();
    let unit = match pick_unit(&units, unit_name.as_deref()) {
        Ok(u) => u,
        Err(e) => {
            log::warn!("connection {id}: {e}");
            let _ = reply(&conn, &Reply::Error(e));
            return;
        }
    };
    match req {
        None => serve_status(&conn, id, unit),
        Some(r) => serve_stream(conn, id, unit, r),
    }
}

fn serve_status(conn: &Conn, id: u64, unit: &Unit) {
    let holders = std::array::from_fn(|i| {
        let s = unit.slots[i].locked();
        (s.owner != 0).then(|| Holder { conn: s.owner, client: s.client.clone(), channel: s.channel.clone() })
    });
    let r = match unit.tuner.try_lock() {
        Ok(mut guard) => match guard.as_mut() {
            // Not open (nothing streamed yet, or the device went away): nothing to
            // measure. A status request never opens the device: that can take
            // seconds with a device in trouble, and a client polls this often.
            None => Reply::Ok(Status { ter: None, sat: None, holders, busy: false }.to_info(&unit.serial)),
            Some(t) => {
                let (ter, sat) = t.poll_status();
                *unit.cnr.locked() = Cnr { ter, sat, read: Some(Instant::now()) };
                Reply::Ok(Status { ter, sat, holders, busy: false }.to_info(&unit.serial))
            }
        },
        Err(std::sync::TryLockError::WouldBlock) => {
            // A tune (or a stream's start or end, or the periodic status read)
            // is in progress: answer with the last values rather than wait.
            let (ter, sat) = {
                let c = unit.cnr.locked();
                (c.ter, c.sat)
            };
            let busy = unit.tuning.load(Ordering::Relaxed) > 0;
            Reply::Ok(Status { ter, sat, holders, busy }.to_info(&unit.serial))
        }
        Err(std::sync::TryLockError::Poisoned(p)) => {
            drop(p.into_inner());
            Reply::Error("tuner state poisoned".into())
        }
    };
    log::debug!("connection {id}: status -> {}", r.to_line());
    let _ = reply(conn, &r);
}

fn serve_stream(conn: Conn, id: u64, unit: &Unit, req: StreamRequest) {
    let Some(program) = DeviceProgram::parse(&req.program) else {
        let _ = reply(&conn, &Reply::Error(format!("bad program {:?}", req.program)));
        return;
    };
    let system = req.channel.system;
    let i = slot(system);
    let what = format!(
        "{} {} ({} Hz{})",
        system_name(system),
        req.channel.name,
        req.channel.frequency_hz,
        if req.raw { ", raw" } else { "" }
    );

    // Device mode and BS/CS decrypt the link, which takes the application
    // credentials; a raw terrestrial stream does not. Refused here, before the
    // stream is claimed, with a reason the client can act on.
    let keys = if !req.raw || system == System::IsdbS {
        match unit.keys.get() {
            Ok(k) => Some(k),
            Err(e) => {
                log::warn!("connection {id}: {what}: {e}");
                let _ = reply(&conn, &Reply::Error(e));
                return;
            }
        }
    } else {
        None
    };

    // Take the stream over from whoever holds it, unless asked not to; the
    // same client (by its id) replacing its own stream is never refused, so
    // that a channel change does not wait for the old connection's end to
    // be noticed.
    let stop = Arc::new(AtomicBool::new(false));
    {
        let mut s = unit.slots[i].locked();
        let how = claim(&s, req.busy_fail, req.client.as_deref());
        if how == Claim::Busy {
            let why =
                format!("{} stream of unit {} is held by connection {}", system_name(system), unit.serial, s.owner);
            log::info!("connection {id}: {what}: busy ({why})");
            let _ = reply(&conn, &Reply::Busy(why));
            return;
        }
        if let Some(prev) = s.stop.take() {
            prev.store(true, Ordering::Relaxed);
            log::info!(
                "unit {}: {} stream {} connection {} by connection {id}",
                unit.serial,
                system_name(system),
                if how == Claim::Replace { "replaced (same client) from" } else { "taken over from" },
                s.owner
            );
        }
        s.owner = id;
        s.stop = Some(stop.clone());
        s.client = req.client.clone();
        s.channel = req.channel.name.clone();
    }

    let t0 = Instant::now();
    unit.tuning.fetch_add(1, Ordering::Relaxed);
    let tuned: Result<f32, String> = {
        let mut guard = unit.tuner.locked();
        unit.open(&mut guard).and_then(|()| {
            let t = guard.as_mut().expect("opened");
            if let Some(k) = keys.as_ref() {
                t.with_app_keys(k.clone());
            }
            t.set_stream_mode(if req.raw { StreamModeCfg::Raw } else { StreamModeCfg::Device });
            t.set_device_program(program);
            // The LNB supply and the lock wait of this request (each client asks
            // with its own, e.g. two BonDriver DLLs of two units), else the
            // daemon's: set on every tune, so one client's values do not stay
            // behind for the next.
            let d = &unit.cfg;
            t.set_lnb(match req.lnb {
                Some(v) => lnb_of(v).0,
                None if d.lnb_power => d.lnb_voltage,
                None => 0,
            });
            let wait = match system {
                System::IsdbT => d.ter_lock_wait_ms,
                System::IsdbS => d.sat_lock_wait_ms,
            };
            t.set_lock_wait_ms(system, req.lock_wait_ms.unwrap_or(wait));
            match t.set_channel(&req.channel) {
                Ok(()) => Ok(t.signal_level_of(system)),
                Err(e) => {
                    t.release(system);
                    if matches!(e, dth10_core::Error::Usb(_)) {
                        log::warn!("unit {}: device lost ({e}); opened again on the next request", unit.serial);
                        *guard = None;
                    }
                    Err(e.to_string())
                }
            }
        })
    };
    unit.tuning.fetch_sub(1, Ordering::Relaxed);
    let cnr = match tuned {
        Ok(cnr) => {
            unit.cnr.locked().set(system, Some(cnr));
            cnr
        }
        Err(e) => {
            log::warn!("unit {}: connection {id} {what}: {e}", unit.serial);
            let _ = reply(&conn, &Reply::Error(e));
            let mut s = unit.slots[i].locked();
            if s.owner == id {
                *s = Slot::default();
            }
            return;
        }
    };
    log::info!("unit {}: connection {id} streams {what} after {:?}, C/N {cnr:.1} dB", unit.serial, t0.elapsed());
    let info = format!("{} {} {} Hz C/N {cnr:.1} dB", unit.serial, system_name(system), req.channel.frequency_hz);
    let mut why = "client closed";
    let mut lost_device = false;
    let mut dropped = 0u64;
    if reply(&conn, &Reply::Ok(info)).is_ok() {
        let queue = &unit.queues[i];
        let mut c = &conn;
        let mut total = 0u64;
        let mut checked = Instant::now();
        let dropped0 = queue.dropped_bytes();
        let mut reported = Instant::now();
        'pump: loop {
            if stop.load(Ordering::Relaxed) {
                why = "taken over";
                break;
            }
            if stop::stopping() {
                why = "daemon stopping";
                break;
            }
            if queue.wait(200) {
                while let Some(chunk) = queue.pop() {
                    if let Err(e) = c.write_all(&chunk) {
                        why = match e.kind() {
                            io::ErrorKind::BrokenPipe
                            | io::ErrorKind::ConnectionReset
                            | io::ErrorKind::ConnectionAborted => "client closed",
                            _ => {
                                log::warn!("connection {id}: write: {e}");
                                "write error"
                            }
                        };
                        break 'pump;
                    }
                    total += chunk.len() as u64;
                }
            } else if checked.elapsed() >= Duration::from_secs(1) {
                checked = Instant::now();
                if conn.peer_closed() {
                    break;
                }
                // The reader thread gave up on the device (unplugged): end the
                // stream so the client notices and asks again later.
                if let Ok(guard) = unit.tuner.try_lock()
                    && guard.as_ref().is_none_or(|t| t.stream_failed(system))
                {
                    why = "device lost";
                    lost_device = true;
                    break;
                }
            }
            if reported.elapsed() >= Duration::from_secs(60) {
                reported = Instant::now();
                let d = queue.dropped_bytes() - dropped0;
                if d > dropped {
                    log::warn!(
                        "unit {}: connection {id} {what}: the client reads too slowly, {} bytes dropped so far",
                        unit.serial,
                        d
                    );
                }
                dropped = d;
            }
        }
        dropped = queue.dropped_bytes() - dropped0;
        log::info!(
            "unit {}: connection {id} {what} ended ({why}): {total} bytes in {:.1} s{}",
            unit.serial,
            t0.elapsed().as_secs_f64(),
            if dropped > 0 { format!(", {dropped} bytes dropped (client too slow)") } else { String::new() }
        );
    }
    let _ = dropped;

    // Release the stream unless a newer connection holds it now.
    let mine = {
        let mut s = unit.slots[i].locked();
        if s.owner == id {
            *s = Slot::default();
            true
        } else {
            false
        }
    };
    if mine {
        let mut guard = unit.tuner.locked();
        if lost_device {
            log::warn!(
                "unit {}: the device went away during {what}; it is opened again on the next request",
                unit.serial
            );
            *guard = None;
        } else if let Some(t) = guard.as_mut() {
            t.release(system);
        }
        unit.cnr.locked().set(system, None);
    }
}

/// Read the status of every unit that streams about once a second (see
/// [`STATUS_POLL`]), until `quit` is set or the daemon stops. A unit whose
/// tuner is busy (a tune, a stream start or end, a client's status request)
/// is skipped this round; the lock is held only for the read itself, so a
/// tune never waits for this. A failed read is logged at debug level only.
fn poll_status(units: &Units, quit: &AtomicBool) {
    while !quit.load(Ordering::Relaxed) && !stop::stopping() {
        let list: Vec<Arc<Unit>> = units.locked().clone();
        for u in &list {
            let streaming = u.slots.iter().any(|s| s.locked().owner != 0);
            if !status_due(streaming, u.cnr.locked().read, Instant::now()) {
                continue;
            }
            let Ok(mut guard) = u.tuner.try_lock() else { continue };
            if let Some(t) = guard.as_mut() {
                let (ter, sat) = t.poll_status();
                // Stored under the tuner lock: a stream's end clears its value
                // under the same lock, after this.
                *u.cnr.locked() = Cnr { ter, sat, read: Some(Instant::now()) };
            }
        }
        std::thread::sleep(STATUS_TICK);
    }
}

/// The units served, in discovery order; grows when a device appears later.
type Units = Arc<Mutex<Vec<Arc<Unit>>>>;

/// How often the daemon looks for devices: while it has none, and otherwise.
const SCAN_WITHOUT_UNIT: Duration = Duration::from_secs(2);
const SCAN_WITH_UNITS: Duration = Duration::from_secs(10);

/// Add a unit for every supported device that is not served yet.
fn discover(units: &Units, make_unit: &dyn Fn(&str) -> Arc<Unit>) {
    let found = match UsbTransport::list_all() {
        Ok(v) => v,
        Err(e) => {
            log::debug!("device list: {e}");
            return;
        }
    };
    let mut u = units.locked();
    for d in found {
        let Some(serial) = d.serial else { continue };
        if u.iter().any(|x| x.serial == serial) {
            continue;
        }
        log::info!("dth10 serve: unit {serial} ({}) at {}", dth10_core::product_label(d.product_id), d.location);
        u.push(make_unit(&serial));
    }
}

pub fn run(a: &ServeArgs) -> Result<(), Box<dyn Error>> {
    let state_dir = a.state_dir.clone().unwrap_or_else(default_state_dir);
    let keys = Arc::new(Keys { explicit: a.app_keys.clone(), state_dir: state_dir.clone(), loaded: Mutex::new(None) });
    if let Err(e) = keys.get() {
        log::warn!("{e}; until then only raw terrestrial streams are served");
    }
    let recipient = match a.recipient {
        Some(RecipientArg::Device) => RecipientMode::Device,
        Some(RecipientArg::Interface) => RecipientMode::Interface,
        None => RecipientMode::platform_default(),
    };
    let (lnb_voltage, lnb_power) = lnb_of(a.lnb);
    let make_unit = |serial: &str| -> Arc<Unit> {
        let cfg = TunerConfig {
            recipient,
            serial: serial.to_owned(),
            stream_mode: StreamModeCfg::Device,
            device_program: DeviceProgram::First,
            device_decrypt: true,
            cepm_state: state_file_for(&state_dir, Some(serial)),
            app_keys: keys.resolve(),
            drop_filler: true,
            read_depth: a.read_depth.unwrap_or_else(dth10_tuner::config::default_read_depth).clamp(1, 64),
            exclusive: false,
            log_events: true,
            ter_lock_wait_ms: a.ter_lock_wait,
            sat_lock_wait_ms: a.sat_lock_wait,
            lnb_voltage,
            lnb_power,
            ..TunerConfig::default()
        };
        Arc::new(Unit {
            serial: serial.to_owned(),
            cfg,
            keys: keys.clone(),
            recipient,
            queues: [TsQueue::new(), TsQueue::new()],
            tuner: Mutex::new(None),
            slots: [Mutex::new(Slot::default()), Mutex::new(Slot::default())],
            cnr: Mutex::new(Cnr::default()),
            tuning: AtomicUsize::new(0),
        })
    };
    let units: Units = Arc::new(Mutex::new(Vec::new()));
    let auto = a.serial.is_empty();
    if auto {
        discover(&units, &make_unit);
    } else {
        let mut u = units.locked();
        for s in &a.serial {
            u.push(make_unit(s));
        }
    }
    // Listen first: a second daemon for the same target ends here, before it
    // touches the device.
    let target = Target::resolve(a.socket.as_deref(), a.listen.as_deref());
    let listener = Listener::bind(&target)?;
    let idle = if a.idle_exit > 0 { format!("; ends after {} s without a client", a.idle_exit) } else { String::new() };
    {
        let u = units.locked();
        if u.is_empty() {
            log::warn!(
                "dth10 serve: no supported device found (VID 0411); listening on {target}{idle} and looking for one every {} s",
                SCAN_WITHOUT_UNIT.as_secs()
            );
        } else {
            log::info!(
                "dth10 serve: unit(s) {}; listening on {target}{idle}",
                u.iter().map(|x| x.serial.as_str()).collect::<Vec<_>>().join(", ")
            );
        }
    }
    let activity = Arc::new(Activity { active: AtomicUsize::new(0), last_end: Mutex::new(Instant::now()) });
    let next_id = AtomicU64::new(1);
    let quit = Arc::new(AtomicBool::new(false));
    let poller = {
        let (units, quit) = (units.clone(), quit.clone());
        std::thread::spawn(move || poll_status(&units, &quit))
    };
    let mut threads: Vec<std::thread::JoinHandle<()>> = Vec::new();
    let mut last_scan = Instant::now();
    while !stop::stopping() {
        match listener.accept() {
            Ok(conn) => {
                let id = next_id.fetch_add(1, Ordering::Relaxed);
                let units = units.clone();
                let act = activity.clone();
                act.active.fetch_add(1, Ordering::Relaxed);
                threads.push(std::thread::spawn(move || {
                    serve_connection(conn, id, &units);
                    *act.last_end.locked() = Instant::now();
                    act.active.fetch_sub(1, Ordering::Relaxed);
                }));
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => std::thread::sleep(Duration::from_millis(100)),
            Err(e) => {
                log::warn!("accept: {e}");
                std::thread::sleep(Duration::from_millis(500));
            }
        }
        threads.retain(|t| !t.is_finished());
        if auto {
            let interval = if units.locked().is_empty() { SCAN_WITHOUT_UNIT } else { SCAN_WITH_UNITS };
            if last_scan.elapsed() >= interval {
                last_scan = Instant::now();
                discover(&units, &make_unit);
            }
        }
        if a.idle_exit > 0
            && activity.active.load(Ordering::Relaxed) == 0
            && activity.last_end.locked().elapsed() >= Duration::from_secs(a.idle_exit)
        {
            log::info!("dth10 serve: no client for {} s, ending", a.idle_exit);
            break;
        }
    }
    log::info!("dth10 serve: stopping");
    let units: Vec<Arc<Unit>> = units.locked().clone();
    for u in &units {
        for s in &u.slots {
            if let Some(f) = s.locked().stop.as_ref() {
                f.store(true, Ordering::Relaxed);
            }
        }
    }
    quit.store(true, Ordering::Relaxed);
    let _ = poller.join();
    for t in threads {
        let _ = t.join();
    }
    for u in &units {
        if let Some(t) = u.tuner.locked().as_mut() {
            t.close();
        }
    }
    listener.cleanup(&target);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keys_are_optional_until_needed() {
        let dir = std::env::temp_dir().join(format!("dth10-keys-test-{}", std::process::id()));
        let _ = std::fs::create_dir_all(&dir);
        // no file anywhere: a clear message naming the state directory
        let none = Keys { explicit: None, state_dir: dir.clone(), loaded: Mutex::new(None) };
        let e = none.get().unwrap_err();
        assert!(e.contains("CoBDACom.dll") && e.contains("--app-keys"), "{e}");
        assert!(none.loaded().is_none());
        // a named file that is not there: its path in the message
        let missing = dir.join("nothing.dll");
        let named = Keys { explicit: Some(missing.clone()), state_dir: dir.clone(), loaded: Mutex::new(None) };
        assert!(named.get().unwrap_err().contains("nothing.dll"));
        let _ = std::fs::remove_dir(&dir);
    }

    fn held(owner: u64, client: Option<&str>) -> Slot {
        Slot { owner, stop: None, client: client.map(str::to_owned), channel: "27".into() }
    }

    #[test]
    fn stream_claims() {
        assert_eq!(claim(&Slot::default(), true, Some("a")), Claim::Free);
        assert_eq!(claim(&Slot::default(), false, None), Claim::Free);
        // another client's stream: refused with busy=fail, taken over without
        assert_eq!(claim(&held(3, Some("a")), true, Some("b")), Claim::Busy);
        assert_eq!(claim(&held(3, Some("a")), false, Some("b")), Claim::TakeOver);
        assert_eq!(claim(&held(3, None), true, Some("b")), Claim::Busy);
        assert_eq!(claim(&held(3, None), true, None), Claim::Busy);
        assert_eq!(claim(&held(3, None), false, None), Claim::TakeOver);
        // the same client changing channel replaces its own stream even with busy=fail
        assert_eq!(claim(&held(3, Some("a")), true, Some("a")), Claim::Replace);
        assert_eq!(claim(&held(3, Some("a")), false, Some("a")), Claim::Replace);
    }

    #[test]
    fn status_read_schedule() {
        let now = Instant::now();
        let ago = |ms: u64| now.checked_sub(Duration::from_millis(ms));
        // nothing streams: never read, however old the last read
        assert!(!status_due(false, None, now));
        assert!(!status_due(false, ago(60_000), now));
        // a stream runs: read when nothing was read yet or the last read is a second old
        assert!(status_due(true, None, now));
        assert!(status_due(true, ago(1000), now));
        assert!(status_due(true, ago(5000), now));
        // a read (a client's status request) less than a second ago: not yet
        assert!(!status_due(true, ago(999), now));
        assert!(!status_due(true, Some(now), now));
        // a read stamped after `now` (taken by another thread meanwhile): not due
        assert!(!status_due(true, Some(now + Duration::from_millis(10)), now));
    }
}
