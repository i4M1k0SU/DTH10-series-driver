//! The DLL's client of the server: the tuner is held by `dth10 serve`, the
//! server process (`dth10.exe` in the DLL's folder) that this DLL starts when
//! none answers,
//! and every DLL instance (another TVTest window, a renamed copy of this DLL
//! with its own ini) is a client of the same server. The device's terrestrial
//! and satellite streams are independent in the server (docs/40), so one
//! window can watch GR while another watches BS. The relation never changes:
//! the DLL is the client, the server the server, whichever DLL started it;
//! the server ends by itself after `[Server] IdleExitSec` seconds without a
//! client.
//!
//! Where they meet is not configurable: the socket file `dth10.sock` in the
//! DLL's folder (AF_UNIX, Windows 10 1803 or later). Where that cannot be
//! used (an older Windows, a folder path too long for a socket address) the
//! DLL falls back by itself to a loopback TCP port derived from the folder,
//! and starts the server with `--listen` accordingly.
//!
//! A supervisor thread per DLL instance asks the server for the signal level
//! once a second (so `GetSignalLevel` never waits) and brings the stream
//! back when it is not running: a server that was killed or crashed is
//! started again and the last channel asked for once more; a stream the
//! server ended (the device was unplugged) or could not start yet (no device
//! at all) is asked for again with growing pauses, so the picture comes back
//! by itself once the tuner is there.
//!
//! Every stream request carries this instance's client id, so the server
//! replaces this DLL's own previous stream at once on a channel change and
//! answers `busy` at once for a stream another window holds (docs/30).
//!
//! Nothing here waits long on TVTest's thread: a server that ends right
//! after it was started (cannot bind its socket, lost the bind race) is
//! noticed by its exit, not by a timeout, and a server that is up but has no
//! device answers every request at once with an error.

use std::ffi::OsString;
use std::io::{BufRead, BufReader, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use dth10_core::LockExt;
use dth10_tuner::ipc::{self, Conn, Target};
use dth10_tuner::proto::{MAX_LINE, Reply, Request, Status, StreamRequest};
use dth10_tuner::{Channel, DeviceProgram, StreamModeCfg, System, TsQueue};

use crate::config::Config;

/// `[Server]` keys of the ini.
#[derive(Debug, Clone)]
pub struct ServerConfig {
    /// `IdleExitSec`: the server ends after this long without a client (0 = never).
    pub idle_exit_secs: u64,
    /// `StartTimeoutSec`: how long to wait for a server this DLL started to
    /// answer. It binds its socket well under a second after start (before it
    /// touches the device), so this is only a backstop; a server that ends at
    /// once is noticed by its exit.
    pub start_timeout_secs: u64,
}

impl Default for ServerConfig {
    fn default() -> Self {
        ServerConfig { idle_exit_secs: 5, start_timeout_secs: 5 }
    }
}

/// The server executable and its socket file, both in the DLL's folder.
const SERVER_EXE: &str = "dth10.exe";
const SERVER_SOCKET: &str = "dth10.sock";
/// The loopback ports of the TCP fallback: one of a hundred, picked by the
/// DLL's folder.
const FALLBACK_PORT_BASE: u16 = 40780;

/// How long a failed recovery waits before the next attempt, at first; it
/// doubles up to [`RETRY_MAX`] while the attempts keep failing.
const RETRY_FIRST: Duration = Duration::from_secs(2);
const RETRY_MAX: Duration = Duration::from_secs(30);

/// How long a status request may take. The server answers from what it has
/// (it never opens the device for a status request), so this only catches a
/// server that is stuck.
const STATUS_TIMEOUT: Duration = Duration::from_secs(3);

/// The DLL's side of a stream: the reader thread that copies the TS from the
/// server's connection into the queue TVTest reads from.
struct ClientStream {
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl ClientStream {
    fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// How the server is started (kept, so that it can be started again).
struct Launch {
    exe: PathBuf,
    args: Vec<OsString>,
    log: PathBuf,
}

/// Why a status request got no answer.
#[derive(Debug)]
enum NoAnswer {
    /// Nothing listens at the target (no socket file, connection refused):
    /// there is no server.
    Unreachable(String),
    /// A server accepted the connection but did not answer in time: it is
    /// there, so no second one is started.
    Failed(String),
}

impl std::fmt::Display for NoAnswer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NoAnswer::Unreachable(e) => write!(f, "cannot reach the server: {e}"),
            NoAnswer::Failed(e) => write!(f, "the server does not answer: {e}"),
        }
    }
}

/// What the DLL's thread and the supervisor thread share.
struct Inner {
    target: Target,
    /// This DLL instance's id in stream requests: the server replaces a
    /// stream held under the same id at once (a channel change), and refuses
    /// (`busy`) one held by another window.
    client: String,
    unit: Option<String>,
    program: String,
    raw: bool,
    /// This DLL's `[ISDB-S] LnbPower` / `LnbVoltage` (0 = off), `[ISDB-T]
    /// Attenuator`, `[ISDB-S] Booster` and lock waits, sent with every stream
    /// request: they apply to this DLL's unit only, whichever DLL started the server.
    lnb: u32,
    attenuator: bool,
    booster: bool,
    ter_lock_wait_ms: u64,
    sat_lock_wait_ms: u64,
    queue: TsQueue,
    launch: Launch,
    start_timeout: Duration,
    /// The channel TVTest asked for last: what a recovery brings back.
    want: Mutex<Option<Channel>>,
    stream: Mutex<Option<ClientStream>>,
    /// Serialises stop/start of the stream between TVTest's thread and the supervisor.
    ops: Mutex<()>,
    /// The C/N of both systems as last read by the supervisor, so that
    /// `signal_level` answers at once: TVTest calls it on the thread that also
    /// reads the TS, and a status request can wait for a tune in the server.
    cnr: Mutex<(Option<f32>, Option<f32>)>,
    /// The stream wanted is not running: the reader thread saw it end
    /// without being told to stop, or it could not be started yet.
    lost: AtomicBool,
    closing: AtomicBool,
}

pub struct Client {
    inner: Arc<Inner>,
    supervisor: Option<JoinHandle<()>>,
}

fn program_text(p: DeviceProgram) -> String {
    match p {
        DeviceProgram::First => "first".into(),
        DeviceProgram::All => "all".into(),
        DeviceProgram::Id(n) => n.to_string(),
    }
}

/// The loopback port of the TCP fallback for a DLL folder (FNV-1a of the
/// folder's path, case-folded): fixed per folder, so every DLL instance of
/// that folder meets the same server, and two installs seldom collide.
fn fallback_port(dll_dir: &Path) -> u16 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in dll_dir.to_string_lossy().to_lowercase().bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    FALLBACK_PORT_BASE + (h % 100) as u16
}

/// Where the server listens: the socket file in the DLL's folder, or, where
/// a Unix domain socket cannot be used there, the folder's loopback port.
fn server_target(dll_dir: &Path) -> Target {
    let path = dll_dir.join(SERVER_SOCKET);
    if ipc::unix_socket_usable(&path) {
        return Target::Unix(path);
    }
    let t = Target::Tcp(format!("127.0.0.1:{}", fallback_port(dll_dir)));
    log::warn!(
        "server: a Unix domain socket at {} cannot be used here (Windows before 1803, or the path is too long for one); using {t} instead",
        path.display()
    );
    t
}

/// The server's command line: `dth10.exe serve` in the DLL's folder, told
/// where to listen and what the ini says about the tuner.
fn launch_spec(cfg: &Config, dll_dir: &Path, target: &Target) -> Launch {
    let mut args: Vec<OsString> = vec!["serve".into(), "-v".into()];
    match target {
        Target::Unix(p) => {
            args.push("--socket".into());
            args.push(p.clone().into());
        }
        Target::Tcp(a) => {
            args.push("--listen".into());
            args.push(a.clone().into());
        }
    }
    args.push("--idle-exit".into());
    args.push(cfg.server.idle_exit_secs.to_string().into());
    // The handshake cache DTH10_<serial>.cepm lives next to the DLL too.
    args.push("--state-dir".into());
    args.push(dll_dir.into());
    if let Some(k) = cfg.tuner.app_keys.as_ref() {
        args.push("--app-keys".into());
        args.push(k.clone().into());
    }
    Launch { exe: dll_dir.join(SERVER_EXE), args, log: dll_dir.join("dth10-serve.log") }
}

/// One `status` request: `Ok(Some(status))` when a unit answered, `Ok(None)`
/// when the server answered but has no unit to report on (no device yet, or
/// an unknown serial), and [`NoAnswer`] when it did not answer.
fn status_of(target: &Target, unit: Option<&str>) -> Result<Option<Status>, NoAnswer> {
    let conn = Conn::connect(target).map_err(|e| NoAnswer::Unreachable(e.to_string()))?;
    let failed = |e: std::io::Error| NoAnswer::Failed(e.to_string());
    conn.set_read_timeout(Some(STATUS_TIMEOUT)).map_err(failed)?;
    let mut c = &conn;
    c.write_all(format!("{}\n", Request::Status { unit: unit.map(str::to_owned) }.to_line()).as_bytes())
        .map_err(failed)?;
    let mut line = String::new();
    (&mut BufReader::new(&conn)).take(MAX_LINE as u64).read_line(&mut line).map_err(failed)?;
    match Reply::parse(&line) {
        Reply::Ok(info) => Ok(Some(Status::parse(&info))),
        Reply::Busy(why) | Reply::Error(why) => {
            log::debug!("server: status: {why}");
            Ok(None)
        }
    }
}

/// Take the whole 188-byte packets off the front of `pending`, leaving the
/// tail for the rest of it to arrive (the connection delivers the server's
/// packets in any split).
fn whole_packets(pending: &mut Vec<u8>) -> Option<Vec<u8>> {
    let whole = pending.len() - pending.len() % 188;
    if whole == 0 {
        return None;
    }
    let rest = pending.split_off(whole);
    Some(std::mem::replace(pending, rest))
}

/// Whether the supervisor should try to bring the stream back now: a channel
/// is wanted, the stream is not running or the server does not answer, and
/// the wait after the previous attempt is over.
fn recovery_due(wanted: bool, lost: bool, alive: bool, last_try: Option<Instant>, delay: Duration) -> bool {
    wanted && (lost || !alive) && last_try.is_none_or(|t| t.elapsed() >= delay)
}

/// The wait before the next attempt after a failed one.
fn next_delay(after_failure: Duration) -> Duration {
    (after_failure * 2).min(RETRY_MAX)
}

/// A stream request failed because the server has no device (none at all,
/// or this unit is not there): worth asking again later by itself.
fn no_device(error: &str) -> bool {
    error.starts_with("no unit") || error.contains("no supported device") || error.contains("no device")
}

impl Inner {
    fn status(&self) -> Result<Option<Status>, NoAnswer> {
        status_of(&self.target, self.unit.as_deref())
    }

    fn spawn_server(&self) -> Result<Child, String> {
        let mut cmd = Command::new(&self.launch.exe);
        cmd.args(&self.launch.args);
        match std::fs::OpenOptions::new().create(true).append(true).open(&self.launch.log) {
            Ok(f) => {
                let f2 = f.try_clone().map_err(|e| e.to_string())?;
                cmd.stdout(Stdio::from(f)).stderr(Stdio::from(f2));
            }
            Err(e) => {
                log::warn!("server: cannot write its log {}: {e}", self.launch.log.display());
                cmd.stdout(Stdio::null()).stderr(Stdio::null());
            }
        }
        cmd.stdin(Stdio::null());
        #[cfg(windows)]
        {
            use std::os::windows::process::CommandExt;
            const CREATE_NO_WINDOW: u32 = 0x0800_0000;
            cmd.creation_flags(CREATE_NO_WINDOW);
        }
        log::info!(
            "server: starting {} serve at {} (log {})",
            self.launch.exe.display(),
            self.target,
            self.launch.log.display()
        );
        cmd.spawn().map_err(|e| format!("cannot start the server {}: {e}", self.launch.exe.display()))
    }

    /// The server answers, or is started here and waited for. A server that
    /// ends right after it was started (cannot bind the socket, lost the bind
    /// race against another DLL's server) is noticed by its exit, so TVTest
    /// does not wait out the start timeout for it.
    fn ensure_server(&self) -> Result<(), String> {
        match self.status() {
            Ok(_) => return Ok(()),
            Err(e @ NoAnswer::Failed(_)) => return Err(format!("{e} (at {})", self.target)),
            Err(NoAnswer::Unreachable(e)) => log::info!("server: none answers at {} ({e}); starting one", self.target),
        }
        let mut child = self.spawn_server()?;
        let t0 = Instant::now();
        loop {
            std::thread::sleep(Duration::from_millis(100));
            if self.closing.load(Ordering::Relaxed) {
                return Err("closing".into());
            }
            match self.status() {
                Ok(_) => {
                    log::info!("server: started and answering after {:?}", t0.elapsed());
                    return Ok(());
                }
                Err(e @ NoAnswer::Failed(_)) => return Err(format!("the server started but {e}")),
                Err(NoAnswer::Unreachable(e)) => {
                    if let Ok(Some(exit)) = child.try_wait() {
                        // Gone already. Another DLL's server may have won the
                        // bind race (this one then exits at once): ask once more.
                        return match self.status() {
                            Ok(_) => Ok(()),
                            Err(_) => {
                                Err(format!("the server ended at once ({exit}); see {}", self.launch.log.display()))
                            }
                        };
                    }
                    if t0.elapsed() >= self.start_timeout {
                        return Err(format!("the server did not come up within {:?}: {e}", self.start_timeout));
                    }
                }
            }
        }
    }

    fn stop_stream(&self) {
        if let Some(mut s) = self.stream.locked().take() {
            s.stop();
        }
        self.queue.purge();
    }

    /// Ask the server for the channel's stream and feed the queue from it.
    fn start_stream(self: &Arc<Self>, ch: &Channel) -> Result<(), String> {
        let t0 = Instant::now();
        let req = Request::Stream(StreamRequest {
            unit: self.unit.clone(),
            channel: ch.clone(),
            program: self.program.clone(),
            raw: self.raw && ch.system == System::IsdbT,
            // Another window holds this stream: fail rather than cut it off.
            busy_fail: true,
            client: Some(self.client.clone()),
            lnb: Some(self.lnb),
            attenuator: Some(self.attenuator),
            booster: Some(self.booster),
            lock_wait_ms: Some(match ch.system {
                System::IsdbT => self.ter_lock_wait_ms,
                System::IsdbS => self.sat_lock_wait_ms,
            }),
        });
        // Our own previous stream (same client id) is replaced at once by the
        // server; one held by another window is answered "busy" at once, and
        // TVTest gets the failure without any waiting.
        let conn =
            Conn::connect(&self.target).map_err(|e| format!("cannot reach the server at {}: {e}", self.target))?;
        conn.set_read_timeout(Some(Duration::from_secs(60))).map_err(|e| e.to_string())?;
        let mut c = &conn;
        c.write_all(format!("{}\n", req.to_line()).as_bytes()).map_err(|e| e.to_string())?;
        let mut reader = BufReader::new(conn);
        let mut line = String::new();
        (&mut reader)
            .take(MAX_LINE as u64)
            .read_line(&mut line)
            .map_err(|e| format!("no reply from the server: {e}"))?;
        match Reply::parse(&line) {
            Reply::Ok(info) => log::info!("server: streaming {} after {:?}: {info}", ch.name, t0.elapsed()),
            Reply::Busy(why) => return Err(format!("busy: {why}")),
            Reply::Error(e) => return Err(e),
        }
        reader.get_ref().set_read_timeout(Some(Duration::from_millis(500))).map_err(|e| e.to_string())?;
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = stop.clone();
        let inner = self.clone();
        let name = ch.name.clone();
        let handle = std::thread::Builder::new()
            .name("dth10-client-ts".into())
            .spawn(move || {
                let mut buf = vec![0u8; 64 * 1024];
                let mut pending: Vec<u8> = Vec::new();
                let mut total = 0u64;
                let mut ended = None;
                while !stop2.load(Ordering::Relaxed) {
                    match reader.read(&mut buf) {
                        Ok(0) => {
                            ended = Some("the server ended it");
                            break;
                        }
                        Ok(n) => {
                            total += n as u64;
                            pending.extend_from_slice(&buf[..n]);
                            if let Some(chunk) = whole_packets(&mut pending) {
                                inner.queue.push(chunk);
                            }
                        }
                        Err(e)
                            if matches!(
                                e.kind(),
                                std::io::ErrorKind::WouldBlock
                                    | std::io::ErrorKind::TimedOut
                                    | std::io::ErrorKind::Interrupted
                            ) => {}
                        Err(e) => {
                            log::warn!("server: read: {e}");
                            ended = Some("read error");
                            break;
                        }
                    }
                }
                if let Some(why) = ended
                    && !stop2.load(Ordering::Relaxed)
                {
                    log::warn!("server: the stream of {name} ended ({why}, {total} bytes); it will be asked for again");
                    inner.lost.store(true, Ordering::Relaxed);
                }
            })
            .map_err(|e| e.to_string())?;
        *self.stream.locked() = Some(ClientStream { stop, handle: Some(handle) });
        Ok(())
    }
}

/// Once a second: the signal level, and the stream brought back when it is
/// not running.
fn supervise(inner: Arc<Inner>) {
    let mut last_try: Option<Instant> = None;
    let mut delay = RETRY_FIRST;
    while !inner.closing.load(Ordering::Relaxed) {
        let alive = match inner.status() {
            Ok(st) => {
                *inner.cnr.locked() = st.map(|s| s.cnr()).unwrap_or((None, None));
                true
            }
            Err(e) => {
                *inner.cnr.locked() = (None, None);
                // A server that is there but slow is not replaced by a second one.
                matches!(e, NoAnswer::Failed(_))
            }
        };
        let want = inner.want.locked().clone();
        if let Some(ch) = want
            && recovery_due(true, inner.lost.load(Ordering::Relaxed), alive, last_try, delay)
        {
            last_try = Some(Instant::now());
            let _ops = inner.ops.locked();
            if inner.closing.load(Ordering::Relaxed) {
                break;
            }
            log::warn!(
                "server: {}; asking for the stream of {} again",
                if alive { "the stream is not running" } else { "the server is not answering" },
                ch.name
            );
            inner.stop_stream();
            inner.lost.store(false, Ordering::Relaxed);
            match inner.ensure_server().and_then(|()| inner.start_stream(&ch)) {
                Ok(()) => {
                    log::info!("server: the stream of {} is back", ch.name);
                    delay = RETRY_FIRST;
                }
                Err(e) => {
                    log::warn!("server: the stream of {} is not back yet ({e}); next try in {delay:?}", ch.name);
                    inner.lost.store(true, Ordering::Relaxed);
                    delay = next_delay(delay);
                }
            }
        }
        for _ in 0..10 {
            if inner.closing.load(Ordering::Relaxed) {
                break;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }
}

impl Client {
    /// Connect to the server in the DLL's folder, starting it when none answers.
    pub fn open(cfg: &Config, queue: TsQueue) -> Result<Client, String> {
        static INSTANCE: AtomicU64 = AtomicU64::new(0);
        let dll_dir =
            cfg.dll_dir.clone().ok_or("the DLL's own folder is unknown, so the server dth10.exe cannot be found")?;
        let target = server_target(&dll_dir);
        let launch = launch_spec(cfg, &dll_dir, &target);
        let inner = Arc::new(Inner {
            target,
            client: format!("{}-{}", std::process::id(), INSTANCE.fetch_add(1, Ordering::Relaxed) + 1),
            unit: (!cfg.tuner.serial.is_empty()).then(|| cfg.tuner.serial.clone()),
            program: program_text(cfg.tuner.device_program),
            raw: cfg.tuner.stream_mode == StreamModeCfg::Raw,
            lnb: if cfg.tuner.lnb_power { cfg.tuner.lnb_voltage } else { 0 },
            attenuator: cfg.tuner.attenuator,
            booster: cfg.tuner.booster,
            ter_lock_wait_ms: cfg.tuner.ter_lock_wait_ms,
            sat_lock_wait_ms: cfg.tuner.sat_lock_wait_ms,
            queue,
            launch,
            start_timeout: Duration::from_secs(cfg.server.start_timeout_secs.max(1)),
            want: Mutex::new(None),
            stream: Mutex::new(None),
            ops: Mutex::new(()),
            cnr: Mutex::new((None, None)),
            lost: AtomicBool::new(false),
            closing: AtomicBool::new(false),
        });
        inner.ensure_server()?;
        log::info!("server: using the dth10 serve at {} as client {}", inner.target, inner.client);
        let sup = inner.clone();
        let supervisor =
            std::thread::Builder::new().name("dth10-client-watch".into()).spawn(move || supervise(sup)).ok();
        Ok(Client { inner, supervisor })
    }

    /// Ask the server for the channel's stream (starting the server again
    /// first when it has gone away). Without a device the request fails at
    /// once, and the supervisor keeps asking for the channel until the tuner
    /// is plugged in.
    pub fn set_channel(&mut self, ch: &Channel) -> Result<(), String> {
        let _ops = self.inner.ops.locked();
        *self.inner.want.locked() = Some(ch.clone());
        self.inner.lost.store(false, Ordering::Relaxed);
        self.inner.stop_stream();
        self.inner.ensure_server()?;
        self.inner.start_stream(ch).inspect_err(|e| {
            if no_device(e) {
                self.inner.lost.store(true, Ordering::Relaxed);
            }
        })
    }

    /// C/N of the current channel's system, as last read by the supervisor (never waits).
    pub fn signal_level(&mut self) -> f32 {
        let Some(system) = self.inner.want.locked().as_ref().map(|c| c.system) else { return 0.0 };
        let (ter, sat) = *self.inner.cnr.locked();
        match system {
            System::IsdbT => ter.unwrap_or(0.0),
            System::IsdbS => sat.unwrap_or(0.0),
        }
    }

    /// End the stream and the supervisor; the server releases the stream (and
    /// ends itself when idle).
    pub fn close(&mut self) {
        self.inner.closing.store(true, Ordering::Relaxed);
        {
            let _ops = self.inner.ops.locked();
            *self.inner.want.locked() = None;
            self.inner.stop_stream();
        }
        if let Some(h) = self.supervisor.take() {
            let _ = h.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dth10_tuner::ipc::Listener;
    use std::sync::atomic::AtomicUsize;

    /// A folder standing in for the DLL's, emptied afterwards.
    fn dll_folder(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("dth10-client-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        dir
    }

    #[test]
    fn packets_are_cut_at_188() {
        let mut pending = vec![0x47u8; 188 * 2 + 10];
        let out = whole_packets(&mut pending).expect("two packets");
        assert_eq!((out.len(), pending.len()), (376, 10));
        assert!(whole_packets(&mut pending).is_none());
        pending.extend_from_slice(&[0u8; 178]);
        assert_eq!(whole_packets(&mut pending).map(|v| v.len()), Some(188));
        assert!(pending.is_empty());
    }

    #[test]
    fn recovery_timing() {
        let d = Duration::from_secs(2);
        assert!(!recovery_due(false, true, false, None, d), "nothing wanted");
        assert!(!recovery_due(true, false, true, None, d), "stream fine, server alive");
        assert!(recovery_due(true, true, true, None, d), "stream lost");
        assert!(recovery_due(true, false, false, None, d), "server gone");
        assert!(!recovery_due(true, true, true, Some(Instant::now()), d), "too soon after the last try");
        assert!(recovery_due(true, true, true, Some(Instant::now() - Duration::from_secs(3)), d));
        assert_eq!(next_delay(Duration::from_secs(2)), Duration::from_secs(4));
        assert_eq!(next_delay(Duration::from_secs(20)), RETRY_MAX);
        assert!(no_device("no unit yet (no supported device found; waiting for one)"));
        assert!(!no_device("ISDB-T: no lock (no signal on this channel)"));
    }

    #[test]
    fn target_is_the_socket_next_to_the_dll_or_a_fixed_port() {
        let dir = dll_folder("target");
        assert_eq!(server_target(&dir), Target::Unix(dir.join(SERVER_SOCKET)));
        // a folder path too long for a socket address falls back to TCP
        let long = dir.join("x".repeat(120));
        assert!(matches!(server_target(&long), Target::Tcp(_)));
        // the fallback port is fixed per folder and within the range
        let p = fallback_port(&dir);
        assert_eq!(p, fallback_port(&dir));
        assert!((FALLBACK_PORT_BASE..FALLBACK_PORT_BASE + 100).contains(&p));
        assert_eq!(fallback_port(Path::new("C:\\TVTest")), fallback_port(Path::new("c:\\tvtest")));
        // the server is the exe next to the DLL, told where to listen
        let mut cfg = Config::default();
        cfg.tuner.ter_lock_wait_ms = 2000;
        cfg.tuner.serial = "SN1".into();
        let l = launch_spec(&cfg, &dir, &Target::Unix(dir.join(SERVER_SOCKET)));
        assert_eq!(l.exe, dir.join(SERVER_EXE));
        let args: Vec<String> = l.args.iter().map(|a| a.to_string_lossy().into_owned()).collect();
        assert_eq!(&args[..2], &["serve", "-v"]);
        assert!(args.windows(2).any(|w| w[0] == "--socket" && w[1] == dir.join(SERVER_SOCKET).to_string_lossy()));
        // per-request settings are not on the server's command line
        assert!(!args.iter().any(|a| a == "--lnb" || a.ends_with("-lock-wait")), "{args:?}");
        // one server for every unit: the DLL's Serial goes into its requests, not
        // the server's command line; the read depth is the server's own default
        assert!(!args.iter().any(|a| a == "--serial" || a == "--read-depth"), "{args:?}");
        assert!(
            args.windows(2).any(|w| w[0] == "--state-dir" && w[1] == dir.to_string_lossy()),
            "the handshake cache lives next to the DLL"
        );
        let l = launch_spec(&cfg, &dir, &Target::Tcp("127.0.0.1:40790".into()));
        assert!(args_of(&l).windows(2).any(|w| w[0] == "--listen" && w[1] == "127.0.0.1:40790"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    fn args_of(l: &Launch) -> Vec<String> {
        l.args.iter().map(|a| a.to_string_lossy().into_owned()).collect()
    }

    #[test]
    fn a_server_that_ends_at_once_is_not_waited_for() {
        // A stand-in dth10.exe that exits at once (cmd.exe with no console
        // input): OpenTuner fails well within the timeout, by its exit.
        let dir = dll_folder("none");
        let system32 =
            PathBuf::from(std::env::var("SystemRoot").unwrap_or_else(|_| "C:\\Windows".into())).join("System32");
        std::fs::copy(system32.join("cmd.exe"), dir.join(SERVER_EXE)).expect("copy cmd.exe as the stand-in server");
        let mut cfg = Config { dll_dir: Some(dir.clone()), ..Config::default() };
        cfg.server.start_timeout_secs = 20;
        let t0 = Instant::now();
        let r = Client::open(&cfg, TsQueue::new());
        let took = t0.elapsed();
        let err = match r {
            Ok(_) => panic!("a server answered although the one started had ended"),
            Err(e) => e,
        };
        assert!(took < Duration::from_secs(5), "waited {took:?} for a server that had ended");
        assert!(err.contains("ended at once"), "{err}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn no_exe_fails_at_once() {
        let dir = dll_folder("noexe");
        let cfg = Config { dll_dir: Some(dir.clone()), ..Config::default() };
        let t0 = Instant::now();
        let err = Client::open(&cfg, TsQueue::new()).err().expect("no server");
        assert!(t0.elapsed() < Duration::from_secs(2));
        assert!(err.contains("cannot start the server"), "{err}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A stand-in for `dth10 serve`: answers status, serves every stream
    /// request with a few packets and ends the first stream at once (as a
    /// crash of the server would), counting the stream requests.
    fn fake_server(target: Target, streams: Arc<AtomicUsize>, stop: Arc<AtomicBool>) -> JoinHandle<()> {
        let listener = Listener::bind(&target).expect("bind the fake server");
        std::thread::spawn(move || {
            let mut conns = Vec::new();
            while !stop.load(Ordering::Relaxed) {
                let conn = match listener.accept() {
                    Ok(c) => c,
                    Err(_) => {
                        std::thread::sleep(Duration::from_millis(20));
                        continue;
                    }
                };
                let streams = streams.clone();
                let stop = stop.clone();
                conns.push(std::thread::spawn(move || {
                    let _ = conn.set_read_timeout(Some(Duration::from_secs(5)));
                    let mut line = String::new();
                    let _ = (&mut BufReader::new(&conn)).take(MAX_LINE as u64).read_line(&mut line);
                    let mut c = &conn;
                    match Request::parse(line.trim_end()) {
                        Ok(Request::Status { .. }) => {
                            let _ = c.write_all(b"ok X ter=30.0 sat=-\n");
                        }
                        Ok(Request::Stream(r)) => {
                            assert!(r.busy_fail && r.client.is_some(), "the DLL asks with busy=fail and its client id");
                            let n = streams.fetch_add(1, Ordering::Relaxed) + 1;
                            let _ = c.write_all(b"ok X ISDB-T 557142857 Hz C/N 30.0 dB\n");
                            let mut pkt = [0u8; 188];
                            pkt[0] = 0x47;
                            let _ = c.write_all(&pkt.repeat(10));
                            if n > 1 {
                                // a stream that goes on until the test ends
                                while !stop.load(Ordering::Relaxed) {
                                    std::thread::sleep(Duration::from_millis(50));
                                }
                            }
                        }
                        Err(e) => panic!("bad request {line:?}: {e}"),
                    }
                }));
            }
            for c in conns {
                let _ = c.join();
            }
            listener.cleanup(&target);
        })
    }

    /// Two tuners, one DLL each, told apart by `[Device] Serial`: both DLLs of
    /// the folder use the one server there, and every request of a DLL names
    /// its own unit.
    #[test]
    fn two_units_are_told_apart_by_serial() {
        let dir = dll_folder("two-units");
        let target = server_target(&dir);
        let listener = Listener::bind(&target).expect("bind the fake server");
        // (request kind, unit, client id) of every request the fake server got
        type Seen = Arc<Mutex<Vec<(&'static str, Option<String>, Option<String>)>>>;
        let seen: Seen = Arc::default();
        type Lnbs = Arc<Mutex<Vec<(Option<String>, Option<u32>, Option<u64>, Option<bool>, Option<bool>)>>>;
        let lnbs: Lnbs = Arc::default();
        let stop = Arc::new(AtomicBool::new(false));
        let server = {
            let (seen, lnbs, stop, target) = (seen.clone(), lnbs.clone(), stop.clone(), target.clone());
            std::thread::spawn(move || {
                while !stop.load(Ordering::Relaxed) {
                    let Ok(conn) = listener.accept() else {
                        std::thread::sleep(Duration::from_millis(20));
                        continue;
                    };
                    let _ = conn.set_read_timeout(Some(Duration::from_secs(5)));
                    let mut line = String::new();
                    let _ = (&mut BufReader::new(&conn)).take(MAX_LINE as u64).read_line(&mut line);
                    let mut c = &conn;
                    match Request::parse(line.trim_end()) {
                        Ok(Request::Status { unit }) => {
                            seen.locked().push(("status", unit.clone(), None));
                            let _ = c.write_all(
                                format!(
                                    "ok {} ter=- sat=-
",
                                    unit.unwrap_or_default()
                                )
                                .as_bytes(),
                            );
                        }
                        Ok(Request::Stream(r)) => {
                            seen.locked().push(("stream", r.unit.clone(), r.client.clone()));
                            lnbs.locked().push((r.unit.clone(), r.lnb, r.lock_wait_ms, r.attenuator, r.booster));
                            // the stream ends at once; this test only looks at the requests
                            let _ = c.write_all(
                                b"ok X ISDB-T 557142857 Hz C/N 30.0 dB
",
                            );
                        }
                        Err(e) => panic!("bad request {line:?}: {e}"),
                    }
                }
                listener.cleanup(&target);
            })
        };

        // unit A without, unit B with an antenna supply and booster; different lock waits
        let open = |serial: &str, lnb_power: bool, sat_wait: u64| {
            let mut cfg = Config { dll_dir: Some(dir.clone()), ..Config::default() };
            cfg.tuner.serial = serial.into();
            cfg.tuner.lnb_power = lnb_power;
            cfg.tuner.booster = lnb_power;
            cfg.tuner.sat_lock_wait_ms = sat_wait;
            Client::open(&cfg, TsQueue::new()).expect("open against the fake server")
        };
        let (mut a, mut b) = (open("SN-A", false, 1500), open("SN-B", true, 2500));
        let ch = Channel::parse("BS15_0").expect("BS channel");
        a.set_channel(&ch).expect("stream of unit A");
        b.set_channel(&ch).expect("stream of unit B");
        a.close();
        b.close();
        stop.store(true, Ordering::Relaxed);
        let _ = server.join();
        let _ = std::fs::remove_dir_all(&dir);

        let seen = seen.locked();
        let streams: Vec<_> = seen.iter().filter(|(k, ..)| *k == "stream").collect();
        assert_eq!(streams.len(), 2, "{seen:?}");
        assert_eq!(streams[0].1.as_deref(), Some("SN-A"));
        assert_eq!(streams[1].1.as_deref(), Some("SN-B"));
        assert_ne!(streams[0].2, streams[1].2, "each DLL instance has its own client id");
        assert!(
            seen.iter().filter(|(k, ..)| *k == "status").all(|(_, u, _)| u.is_some()),
            "every status request names the unit: {seen:?}"
        );
        // each DLL's own LNB, switches and lock wait went with its own requests
        let lnbs = lnbs.locked();
        assert_eq!(lnbs[0], (Some("SN-A".into()), Some(0), Some(1500), Some(false), Some(false)));
        assert_eq!(lnbs[1], (Some("SN-B".into()), Some(150), Some(2500), Some(false), Some(true)));
    }

    #[test]
    fn stream_comes_back_after_the_server_ends_it() {
        let dir = dll_folder("recover");
        let target = server_target(&dir);
        let streams = Arc::new(AtomicUsize::new(0));
        let stop = Arc::new(AtomicBool::new(false));
        let server = fake_server(target, streams.clone(), stop.clone());

        let cfg = Config { dll_dir: Some(dir.clone()), ..Config::default() };
        let queue = TsQueue::new();
        let mut h = Client::open(&cfg, queue.clone()).expect("open against the fake server");
        let mut ch = Channel::terrestrial_hz(557_142_857);
        ch.name = "27".into();
        h.set_channel(&ch).expect("first stream");

        // The fake server ends the first stream at once; the supervisor asks
        // for it again by itself.
        let t0 = Instant::now();
        while streams.load(Ordering::Relaxed) < 2 && t0.elapsed() < Duration::from_secs(10) {
            std::thread::sleep(Duration::from_millis(100));
        }
        assert!(streams.load(Ordering::Relaxed) >= 2, "the stream was not asked for again");
        assert!(queue.wait(3000), "no packets from the second stream");
        assert_eq!(queue.pop().map(|c| c.len() % 188), Some(0));
        // the signal level comes from the supervisor's status polls
        let t0 = Instant::now();
        while h.signal_level() == 0.0 && t0.elapsed() < Duration::from_secs(5) {
            std::thread::sleep(Duration::from_millis(100));
        }
        assert_eq!(h.signal_level(), 30.0);

        h.close();
        stop.store(true, Ordering::Relaxed);
        let _ = server.join();
        let _ = std::fs::remove_dir_all(&dir);
    }
}
