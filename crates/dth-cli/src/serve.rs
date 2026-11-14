//! `dth serve`: hold the tuner(s) open for good and stream to clients over
//! a socket (`dth_proto::ipc`: a Unix domain socket on Linux and Windows
//! alike, or TCP on the loopback interface; on Windows the BonDriver starts
//! this as its server).
//!
//! The device's terrestrial and satellite streams are independent here
//! (docs/40): a recorder lists one unit as two tuners, one for GR and one for
//! BS/CS, and both can run at the same time; two TVTest windows do the same
//! through two copies of the BonDriver. On a unit whose tuner #1 is a second
//! terrestrial tuner (the two-tuner models) the two streams are two GR
//! tuners instead, picked by the request's `tuner=` (`0` without it, `1`, or
//! `auto` = the first free one). A client only asks for a stream and
//! reads it. When the client goes away (a recorder kills `dth stream`,
//! TVTest closes), the connection closes and the daemon stops that stream
//! while the other one goes on; with both idle the kits go to standby. The
//! CEPM sessions, the application credentials and the kits' state stay in
//! this process between requests, so a request takes about a second instead
//! of the device start of `dth tune`. A request for a stream that another
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
//!
//! Units of the DT-H30/U2 family (`dth30_backend`) are served the same
//! way: one terrestrial tuner, `raw` = the scrambled TS as received,
//! `device` = one service descrambled by the unit's own B-CAS card. They
//! need no application key file but the firmware from the vendor's
//! `BufFsUSB.sys` (`--firmware`). A unit without a USB serial number is
//! named `DTH30-<product ID>`.

use std::error::Error;
use std::io::{self, BufRead, BufReader, Read, Write};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use clap::Args;
use dth_common::{
    Backend, DeviceProgram, LockExt, Request as UnitRequest, Session, StreamModeCfg, System, TsQueue, default_tuner,
};
use dth_proto::ipc::{Conn, Listener, Target};
use dth_proto::proto::{Holder, MAX_LINE, Reply, Request, Status, StreamRequest, TunerChoice};
use dth10_backend::{Dth10Backend, KeyStore, TunerConfig};
use dth10_usb::RecipientMode;
use dth30_backend::{Dth30Backend, UNIT_PREFIX};

use crate::{RecipientArg, default_state_dir, lnb_of, stop};

#[derive(Args)]
pub struct ServeArgs {
    /// Listen on this socket file (default: $DTH_SOCKET, else /run/dth/dth.sock on Linux,
    /// dth.sock next to this executable on Windows).
    #[arg(long, value_name = "PATH")]
    socket: Option<PathBuf>,
    /// Listen on this TCP address instead (e.g. 127.0.0.1:40780; $DTH_LISTEN).
    #[arg(long, value_name = "ADDR", conflicts_with = "socket")]
    listen: Option<String>,
    /// Serve this unit, by USB serial number (repeatable). Default: every supported device found.
    #[arg(long, value_name = "SERIAL")]
    serial: Vec<String>,
    /// bmRequestType recipient of the vendor requests (default: device on Windows, interface elsewhere).
    #[arg(long, value_enum)]
    recipient: Option<RecipientArg>,
    /// Directory of the CEPM handshake cache (DTH10_<serial>.cepm). Default: $DTH_STATE_DIR,
    /// else /var/lib/dth when it is writable, else the directory of this executable.
    #[arg(long, value_name = "DIR")]
    state_dir: Option<PathBuf>,
    /// The vendor application's CoBDACom.dll (PCastTV3), from which the application
    /// certificate, key and IV are read. Default: $DTH10_APP_KEYS, else CoBDACom.dll in
    /// the state directory, else next to this executable.
    #[arg(long, value_name = "FILE")]
    app_keys: Option<PathBuf>,
    /// DT-H30/U2 family: the vendor driver BufFsUSB.sys the firmware is taken from. Default:
    /// $DTH30_FIRMWARE, else BufFsUSB.sys in the state directory, else next to this executable.
    #[arg(long, value_name = "FILE")]
    firmware: Option<PathBuf>,
    /// Antenna (LNB) supply for BS/CS: 0 (off), 110 (11 V) or 150 (15 V). The DT-H70/U2
    /// has no such supply (the setting is for family models that have one).
    #[arg(long, value_name = "V", default_value_t = 0)]
    lnb: u32,
    /// ISDB-T: switch the attenuator on for requests that do not say (the vendor
    /// application's "use attenuator"; kits without one ignore it).
    #[arg(long)]
    attenuator: bool,
    /// ISDB-S: switch the booster on for requests that do not say (the vendor
    /// application's "use booster"; separate from --lnb).
    #[arg(long)]
    booster: bool,
    /// ISDB-T: how long to wait for lock before streaming [ms] (up to 2 s longer while there is a signal).
    #[arg(long, value_name = "MS", default_value_t = 1200)]
    ter_lock_wait: u64,
    /// ISDB-S: how long to wait for lock and the TMCC TS-ID table [ms] (up to 2 s longer while there is a signal).
    #[arg(long, value_name = "MS", default_value_t = 1500)]
    sat_lock_wait: u64,
    /// Bulk IN transfers kept in flight per stream (39936 bytes each; default 16 on Linux, 4 on Windows).
    #[arg(long, value_name = "N")]
    read_depth: Option<usize>,
    /// End when no client has been connected for this many seconds (0 = run until stopped).
    #[arg(long, value_name = "SECONDS", default_value_t = 0)]
    idle_exit: u64,
}

/// What a request that does not say gets: the daemon's options.
struct Defaults {
    lnb_voltage: u32,
    lnb_power: bool,
    attenuator: bool,
    booster: bool,
    ter_lock_wait_ms: u64,
    sat_lock_wait_ms: u64,
}

/// One served unit.
struct Unit {
    serial: String,
    /// The family it belongs to.
    backend: Arc<dyn Backend>,
    defaults: Arc<Defaults>,
    /// The TS of each tuner: 0 = terrestrial, 1 = satellite (or the second terrestrial tuner).
    queues: [TsQueue; 2],
    /// `None` while the unit is not open (lost; opened again on the next request).
    tuner: Mutex<Option<Box<dyn Session>>>,
    /// Whether tuner #1 is a second terrestrial tuner; `None` until the unit was first opened.
    second_ter: Mutex<Option<bool>>,
    /// Per tuner: the connection streaming it and the flag that ends that connection's stream.
    slots: [Mutex<Slot>; 2],
    /// The last C/N read of each tuner, answered to a status request while
    /// the tuner is busy with a tune (a tune can take seconds; a client's
    /// signal-level poll must not wait for it).
    cnr: Mutex<Cnr>,
    /// Stream requests tuning the unit now (a status reply says `busy` then;
    /// the tuner is also locked briefly by the periodic status read).
    tuning: AtomicUsize,
}

/// The C/N of each tuner as last read, and when the status was last read.
#[derive(Default)]
struct Cnr {
    tuners: [Option<f32>; 2],
    /// The last full status read (a client's status request or the periodic read).
    read: Option<Instant>,
}

impl Cnr {
    fn set(&mut self, tuner: usize, cnr: Option<f32>) {
        self.tuners[tuner] = cnr;
    }
}

/// A unit's status reply from the C/N of its tuners: tuner #1 is the
/// satellite one, or the second terrestrial one (`ter1=`).
fn unit_status(cnr: [Option<f32>; 2], second_ter: bool, holders: [Option<Holder>; 2], busy: bool) -> Status {
    let (sat, ter1) = if second_ter { (None, cnr[1]) } else { (cnr[1], None) };
    Status { ter: cnr[0], sat, second_ter, ter1, holders, busy }
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

/// The tuner a stream request goes to (key `tuner=`, see `dth_proto::proto`):
/// a satellite channel to the satellite tuner; a terrestrial one to tuner 0,
/// to tuner 1 when it names it on a unit whose tuner #1 is a second terrestrial
/// tuner, or with `auto` to the tuner this client streams already, else the
/// first free one, else tuner 0 (taken over, or `busy`, as the claim decides).
fn pick_tuner(
    system: System,
    choice: Option<TunerChoice>,
    second_ter: bool,
    slots: [&Slot; 2],
    client: Option<&str>,
) -> Result<usize, String> {
    match (system, choice) {
        (System::IsdbS, _) => Ok(default_tuner(System::IsdbS)),
        (System::IsdbT, None | Some(TunerChoice::Index(0))) => Ok(0),
        (System::IsdbT, Some(TunerChoice::Index(1))) if second_ter => Ok(1),
        (System::IsdbT, Some(TunerChoice::Index(n))) => Err(format!("no terrestrial tuner {n} on this unit")),
        (System::IsdbT, Some(TunerChoice::Auto)) if !second_ter => Ok(0),
        (System::IsdbT, Some(TunerChoice::Auto)) => {
            let mine =
                client.and_then(|c| (0..2).find(|&i| slots[i].owner != 0 && slots[i].client.as_deref() == Some(c)));
            Ok(mine.or_else(|| (0..2).find(|&i| slots[i].owner == 0)).unwrap_or(0))
        }
    }
}

fn system_name(system: System) -> &'static str {
    match system {
        System::IsdbT => "ISDB-T",
        System::IsdbS => "ISDB-S",
    }
}

/// The log name of a tuner's stream: its system, `ISDB-T #1` for the second
/// terrestrial tuner.
fn tuner_name(system: System, tuner: usize, second_ter: bool) -> &'static str {
    match system {
        System::IsdbT if second_ter && tuner == 1 => "ISDB-T #1",
        s => system_name(s),
    }
}

impl Unit {
    /// Open the device unless it is open already.
    fn open(&self, tuner: &mut Option<Box<dyn Session>>) -> Result<(), String> {
        if tuner.is_some() {
            return Ok(());
        }
        let t0 = Instant::now();
        let mut t = self
            .backend
            .open(&self.serial, self.queues[0].clone())
            .map_err(|e| format!("unit {}: {e}", self.serial))?;
        t.set_queue(1, self.queues[1].clone());
        *self.second_ter.locked() = Some(t.has_second_terrestrial());
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
    let second_ter = unit.second_ter.locked().unwrap_or(false);
    let r = match unit.tuner.try_lock() {
        Ok(mut guard) => match guard.as_mut() {
            // Not open (nothing streamed yet, or the device went away): nothing to
            // measure. A status request never opens the device: that can take
            // seconds with a device in trouble, and a client polls this often.
            None => Reply::Ok(unit_status([None, None], second_ter, holders, false).to_info(&unit.serial)),
            Some(t) => {
                let cnr = t.poll_status();
                *unit.cnr.locked() = Cnr { tuners: cnr, read: Some(Instant::now()) };
                Reply::Ok(unit_status(cnr, second_ter, holders, false).to_info(&unit.serial))
            }
        },
        Err(std::sync::TryLockError::WouldBlock) => {
            // A tune (or a stream's start or end, or the periodic status read)
            // is in progress: answer with the last values rather than wait.
            let cnr = unit.cnr.locked().tuners;
            let busy = unit.tuning.load(Ordering::Relaxed) > 0;
            Reply::Ok(unit_status(cnr, second_ter, holders, busy).to_info(&unit.serial))
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
    let describe = |name: &str| {
        format!("{name} {} ({} Hz{})", req.channel.name, req.channel.frequency_hz, if req.raw { ", raw" } else { "" })
    };
    let what = describe(system_name(system));

    // What the unit's family needs for this request (the DTH10 series' key
    // file for the link decryption, the DT-H30/U2 family's firmware file and
    // its one terrestrial tuner): refused here, before the stream is claimed,
    // with a reason the client can act on.
    let asked = match req.tuner {
        Some(TunerChoice::Index(n)) => usize::from(n),
        _ => default_tuner(system),
    };
    if let Err(e) = unit.backend.check(&unit.serial, UnitRequest { system, tuner: asked, raw: req.raw }) {
        log::warn!("connection {id}: {what}: {e}");
        let _ = reply(&conn, &Reply::Error(e));
        return;
    }

    // Which terrestrial tuner a request for tuner 1 or `auto` may use depends
    // on the unit's tuner #1, known once the unit was opened: open it first.
    if system == System::IsdbT
        && !matches!(req.tuner, None | Some(TunerChoice::Index(0)))
        && unit.second_ter.locked().is_none()
    {
        unit.tuning.fetch_add(1, Ordering::Relaxed);
        let opened = unit.open(&mut unit.tuner.locked());
        unit.tuning.fetch_sub(1, Ordering::Relaxed);
        if let Err(e) = opened {
            log::warn!("connection {id}: {what}: {e}");
            let _ = reply(&conn, &Reply::Error(e));
            return;
        }
    }
    let second_ter = unit.second_ter.locked().unwrap_or(false);

    // Take the stream over from whoever holds it, unless asked not to; the
    // same client (by its id) replacing its own stream is never refused, so
    // that a channel change does not wait for the old connection's end to
    // be noticed. Both slots are held while the tuner is picked, so that two
    // `auto` requests do not pick the same free tuner.
    let stop = Arc::new(AtomicBool::new(false));
    let i = {
        let mut slots = [unit.slots[0].locked(), unit.slots[1].locked()];
        let i = match pick_tuner(system, req.tuner, second_ter, [&*slots[0], &*slots[1]], req.client.as_deref()) {
            Ok(i) => i,
            Err(e) => {
                let e = format!("unit {}: {e}", unit.serial);
                log::warn!("connection {id}: {what}: {e}");
                let _ = reply(&conn, &Reply::Error(e));
                return;
            }
        };
        let name = tuner_name(system, i, second_ter);
        let s = &mut slots[i];
        let how = claim(s, req.busy_fail, req.client.as_deref());
        if how == Claim::Busy {
            let why = format!("{name} stream of unit {} is held by connection {}", unit.serial, s.owner);
            log::info!("connection {id}: {}: busy ({why})", describe(name));
            let _ = reply(&conn, &Reply::Busy(why));
            return;
        }
        if let Some(prev) = s.stop.take() {
            prev.store(true, Ordering::Relaxed);
            log::info!(
                "unit {}: {name} stream {} connection {} by connection {id}",
                unit.serial,
                if how == Claim::Replace { "replaced (same client) from" } else { "taken over from" },
                s.owner
            );
        }
        s.owner = id;
        s.stop = Some(stop.clone());
        s.client = req.client.clone();
        s.channel = req.channel.name.clone();
        i
    };
    let what = describe(tuner_name(system, i, second_ter));

    let t0 = Instant::now();
    unit.tuning.fetch_add(1, Ordering::Relaxed);
    let tuned: Result<f32, String> = {
        let mut guard = unit.tuner.locked();
        unit.open(&mut guard).and_then(|()| {
            let t = guard.as_mut().expect("opened");
            t.set_stream_mode(if req.raw { StreamModeCfg::Raw } else { StreamModeCfg::Device });
            t.set_device_program(program);
            // The LNB supply, the attenuator / booster and the lock wait of this
            // request (each client asks with its own, e.g. two BonDriver DLLs of
            // two units), else the daemon's: set on every tune, so one client's
            // values do not stay behind for the next.
            let d = &unit.defaults;
            t.set_lnb(match req.lnb {
                Some(v) => lnb_of(v).0,
                None if d.lnb_power => d.lnb_voltage,
                None => 0,
            });
            t.set_attenuator(req.attenuator.unwrap_or(d.attenuator));
            t.set_booster(req.booster.unwrap_or(d.booster));
            let wait = match system {
                System::IsdbT => d.ter_lock_wait_ms,
                System::IsdbS => d.sat_lock_wait_ms,
            };
            t.set_lock_wait_ms(system, req.lock_wait_ms.unwrap_or(wait));
            match t.set_channel_on(i, &req.channel) {
                Ok(()) => Ok(t.signal_level_of(i)),
                Err(e) => {
                    t.release(i);
                    if matches!(e, dth_common::Error::Usb(_)) {
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
            unit.cnr.locked().set(i, Some(cnr));
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
    let mut info = format!("{} {} {} Hz C/N {cnr:.1} dB", unit.serial, system_name(system), req.channel.frequency_hz);
    if second_ter && system == System::IsdbT {
        // which of the two terrestrial tuners (an `auto` request learns it here)
        info.push_str(&format!(" tuner={i}"));
    }
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
                    && guard.as_ref().is_none_or(|t| t.stream_failed(i))
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
            t.release(i);
        }
        unit.cnr.locked().set(i, None);
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
                let cnr = t.poll_status();
                // Stored under the tuner lock: a stream's end clears its value
                // under the same lock, after this.
                *u.cnr.locked() = Cnr { tuners: cnr, read: Some(Instant::now()) };
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
/// Builds a served unit from its name and family.
type MakeUnit<'a> = &'a dyn Fn(&str, Arc<dyn Backend>) -> Arc<Unit>;

fn discover(units: &Units, backends: &[Arc<dyn Backend>], make_unit: MakeUnit<'_>) {
    let mut found = Vec::new();
    for b in backends {
        match b.units() {
            Ok(v) => found.extend(v.into_iter().map(|u| (u, b.clone()))),
            Err(e) => log::debug!("{} device list: {e}", b.family()),
        }
    }
    let mut u = units.locked();
    for (unit, backend) in found {
        if u.iter().any(|x| x.serial == unit.name) {
            continue;
        }
        log::info!("dth serve: unit {} ({}) at {}", unit.name, unit.label, unit.location);
        u.push(make_unit(&unit.name, backend));
    }
}

/// The backend of a unit named on the command line: the family that has a
/// unit of that name now, a DT-H30/U2-family name, else the DTH10 series.
fn backend_of(name: &str, backends: &[Arc<dyn Backend>]) -> Arc<dyn Backend> {
    if let Some(b) = backends.iter().find(|b| b.units().is_ok_and(|v| v.iter().any(|u| u.name == name))) {
        return b.clone();
    }
    backends[usize::from(name.starts_with(UNIT_PREFIX))].clone()
}

pub fn run(a: &ServeArgs) -> Result<(), Box<dyn Error>> {
    let state_dir = a.state_dir.clone().unwrap_or_else(default_state_dir);
    let keys = Arc::new(KeyStore::new(a.app_keys.clone(), state_dir.clone()));
    if let Err(e) = keys.get() {
        log::warn!(
            "{e}; until then DTH10-series units serve only raw terrestrial streams (DT-H30/U2-family units need no key file)"
        );
    }
    let recipient = match a.recipient {
        Some(RecipientArg::Device) => RecipientMode::Device,
        Some(RecipientArg::Interface) => RecipientMode::Interface,
        None => RecipientMode::platform_default(),
    };
    let (lnb_voltage, lnb_power) = lnb_of(a.lnb);
    let read_depth = a.read_depth.unwrap_or_else(dth_common::stream::default_read_depth).clamp(1, 64);
    let template = TunerConfig {
        recipient,
        stream_mode: StreamModeCfg::Device,
        device_program: DeviceProgram::First,
        device_decrypt: true,
        drop_filler: true,
        read_depth,
        exclusive: false,
        log_events: true,
        ter_lock_wait_ms: a.ter_lock_wait,
        sat_lock_wait_ms: a.sat_lock_wait,
        lnb_voltage,
        lnb_power,
        attenuator: a.attenuator,
        booster: a.booster,
        ..TunerConfig::default()
    };
    let backends: Vec<Arc<dyn Backend>> = vec![
        Arc::new(Dth10Backend::new(template, Some(state_dir.clone()), keys)),
        Arc::new(Dth30Backend::new(a.firmware.clone(), state_dir.clone(), read_depth)),
    ];
    let defaults = Arc::new(Defaults {
        lnb_voltage,
        lnb_power,
        attenuator: a.attenuator,
        booster: a.booster,
        ter_lock_wait_ms: a.ter_lock_wait,
        sat_lock_wait_ms: a.sat_lock_wait,
    });
    let make_unit = |serial: &str, backend: Arc<dyn Backend>| -> Arc<Unit> {
        Arc::new(Unit {
            serial: serial.to_owned(),
            backend,
            defaults: defaults.clone(),
            queues: [TsQueue::new(), TsQueue::new()],
            tuner: Mutex::new(None),
            second_ter: Mutex::new(None),
            slots: [Mutex::new(Slot::default()), Mutex::new(Slot::default())],
            cnr: Mutex::new(Cnr::default()),
            tuning: AtomicUsize::new(0),
        })
    };
    let units: Units = Arc::new(Mutex::new(Vec::new()));
    let auto = a.serial.is_empty();
    if auto {
        discover(&units, &backends, &make_unit);
    } else {
        let mut u = units.locked();
        for s in &a.serial {
            u.push(make_unit(s, backend_of(s, &backends)));
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
                "dth serve: no supported device found (VID 0411); listening on {target}{idle} and looking for one every {} s",
                SCAN_WITHOUT_UNIT.as_secs()
            );
        } else {
            log::info!(
                "dth serve: unit(s) {}; listening on {target}{idle}",
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
                discover(&units, &backends, &make_unit);
            }
        }
        if a.idle_exit > 0
            && activity.active.load(Ordering::Relaxed) == 0
            && activity.last_end.locked().elapsed() >= Duration::from_secs(a.idle_exit)
        {
            log::info!("dth serve: no client for {} s, ending", a.idle_exit);
            break;
        }
    }
    log::info!("dth serve: stopping");
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
    fn tuner_picks() {
        use System::{IsdbS, IsdbT};
        use TunerChoice::{Auto, Index};
        let free = Slot::default();
        let (a, b) = (held(3, Some("a")), held(4, Some("b")));
        let pick = |system, choice, second_ter, slots: [&Slot; 2], client| {
            pick_tuner(system, choice, second_ter, slots, client)
        };
        // a unit with a satellite tuner (or none): terrestrial always on tuner 0,
        // satellite on tuner 1, whatever is held
        for second_ter in [false, true] {
            assert_eq!(pick(IsdbS, None, second_ter, [&a, &free], Some("b")), Ok(1));
        }
        assert_eq!(pick(IsdbT, None, false, [&a, &free], Some("b")), Ok(0));
        assert_eq!(pick(IsdbT, Some(Auto), false, [&a, &free], Some("b")), Ok(0), "auto is tuner 0 there");
        assert_eq!(pick(IsdbT, Some(Index(0)), false, [&free, &free], None), Ok(0));
        assert!(pick(IsdbT, Some(Index(1)), false, [&free, &free], None).is_err(), "tuner 1 is not terrestrial");
        // two terrestrial tuners: no key = tuner 0, an index is taken as named
        assert_eq!(pick(IsdbT, None, true, [&a, &free], Some("b")), Ok(0));
        assert_eq!(pick(IsdbT, Some(Index(1)), true, [&free, &a], Some("b")), Ok(1));
        // auto: the first free tuner ...
        assert_eq!(pick(IsdbT, Some(Auto), true, [&free, &free], Some("b")), Ok(0));
        assert_eq!(pick(IsdbT, Some(Auto), true, [&a, &free], Some("b")), Ok(1));
        assert_eq!(pick(IsdbT, Some(Auto), true, [&a, &free], None), Ok(1));
        // ... but the one the same client streams already (a channel change)
        assert_eq!(pick(IsdbT, Some(Auto), true, [&free, &b], Some("b")), Ok(1));
        assert_eq!(pick(IsdbT, Some(Auto), true, [&a, &b], Some("b")), Ok(1));
        // both held by others: tuner 0 (taken over, or busy with busy=fail)
        assert_eq!(pick(IsdbT, Some(Auto), true, [&a, &b], Some("c")), Ok(0));
        assert_eq!(pick(IsdbT, Some(Auto), true, [&a, &b], None), Ok(0));
    }

    #[test]
    fn status_by_unit_kind() {
        let h = Some(Holder { conn: 5, client: None, channel: "27".into() });
        // tuner #1 is the satellite one: the reply as before
        let st = unit_status([Some(30.1), Some(18.6)], false, [None, h.clone()], false);
        assert_eq!(st.to_info("X"), "X ter=30.1 sat=18.6 sat_ch=27 sat_by=5");
        assert_eq!(unit_status([None, None], false, [None, None], false).to_info("X"), "X ter=- sat=-");
        // tuner #1 is a second terrestrial tuner
        let st = unit_status([None, Some(28.0)], true, [None, h], true);
        assert_eq!(st.to_info("X"), "X ter=- sat=- ter1=28.0 ter1_ch=27 ter1_by=5 busy");
        assert_eq!((st.sat, st.ter1), (None, Some(28.0)));
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
