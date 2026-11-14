//! `dth`: stand-alone tuner command for the BUFFALO DTH10 series of USB TV
//! tuners (the models the vendor's Windows driver covers; verified on
//! a DT-H70/U2).
//!
//! `dth tune` opens the device, tunes, and writes the transport stream to
//! stdout (or a file) until it is stopped (SIGTERM / SIGINT / Ctrl-C, the
//! reader closing the pipe, or `--duration`): the tuner command a recorder
//! such as Mirakurun or mirakc runs. Every run opens the device itself. In the
//! device-side descrambling mode (the default) the result of the CEPM
//! handshake (docs/22) is cached in `<state dir>/DTH10_<serial>.cepm` so later runs
//! skip it while the device keeps the session. The application credentials
//! the handshake needs are read from the user's copy of the vendor
//! application's `CoBDACom.dll` (`--app-keys`); they are not built in.
//!
//! `dth serve` holds the device open for good instead and streams to
//! `dth stream` clients over a Unix domain socket (on Windows too, where
//! the BonDriver starts it as its server; TCP on the loopback interface is
//! the alternative): the terrestrial and the satellite stream of one unit then run
//! independently, so a recorder can list the unit as two tuners (GR and
//! BS/CS), or two TVTest windows can watch GR and BS at once.
//!
//! `list`, `info`, `tune` and `serve` also drive the DTH30 series
//! (`dth30_backend`): one terrestrial tuner, firmware from the vendor's
//! `BufFsUSB.sys` (`--firmware`), device mode descrambled by the unit's own
//! B-CAS card (no application keys). Such a unit is named by its unit name
//! (`DTH30-<product ID>` when it has no USB serial number).

mod serve;
mod stop;
mod stream;

use std::error::Error;
use std::fs::File;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use clap::{Args, Parser, Subcommand, ValueEnum};
use dth_common::{
    Backend, Channel, DeviceProgram, Request, Session, StreamModeCfg, System, TsQueue, Unit, default_tuner,
};
use dth10_backend::{Dth10Backend, KeyStore, TunerConfig};
use dth10_usb::{RecipientMode, UsbTransport};
use dth30_backend::Dth30Backend;

#[derive(Parser)]
#[command(name = "dth", version, about = "User-mode tuner command for the BUFFALO DTH10/DTH30 series of USB TV tuners")]
struct Cli {
    /// Log level: error, warn, info, debug or trace (RUST_LOG overrides it).
    #[arg(long, global = true, value_name = "LEVEL")]
    log_level: Option<String>,
    /// -v = info, -vv = debug (default: warn). Logs go to stderr.
    #[arg(short, long, global = true, action = clap::ArgAction::Count)]
    verbose: u8,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// List the connected supported devices.
    List,
    /// Show the device information (DTH10 series: tuner descriptors, firmware, endpoints; DT-H30/U2
    /// family: endpoints, board and front end, firmware source; this resets its chip).
    Info(DeviceArgs),
    /// Tune and write the TS to stdout (or --output) until stopped.
    Tune(Box<TuneArgs>),
    /// Hold the device open for good and stream to `dth stream` clients over a socket.
    Serve(serve::ServeArgs),
    /// Ask `dth serve` for a stream and write its TS to stdout (a recorder's tuner command).
    Stream(Box<stream::StreamArgs>),
    /// Ask `dth serve` for the C/N of both systems (a health check).
    Status(stream::StatusArgs),
}

#[derive(Args, Clone, Default)]
struct DeviceArgs {
    /// Select the device by USB serial number, or a DTH30 series unit by the unit name `list`
    /// shows (default: the first one found, the DTH10 series first; `info` shows every one).
    #[arg(long, value_name = "SERIAL")]
    serial: Option<String>,
    /// Open this usbfs node (/dev/bus/usb/BBB/DDD) instead of searching.
    #[cfg(any(target_os = "linux", target_os = "android"))]
    #[arg(long, value_name = "PATH")]
    device: Option<PathBuf>,
    /// bmRequestType recipient of the vendor requests (default: device on Windows, interface elsewhere).
    #[arg(long, value_enum)]
    recipient: Option<RecipientArg>,
    /// DTH30 series: the vendor driver BufFsUSB.sys the firmware is taken from. Default:
    /// $DTH30_FIRMWARE, else BufFsUSB.sys in the state directory, else next to this executable.
    #[arg(long, value_name = "FILE")]
    firmware: Option<PathBuf>,
}

#[derive(Clone, Copy, ValueEnum)]
pub(crate) enum RecipientArg {
    Interface,
    Device,
}

#[derive(Clone, Copy, PartialEq, Eq, ValueEnum)]
pub(crate) enum Mode {
    /// The built-in B-CAS card descrambles one program and the link encryption
    /// is removed here: clear TS of that program (PSI + its ES). BS/CS channels
    /// always use this mode (the device has no raw mode on the satellite stream).
    Device,
    /// Terrestrial only: the full TS as broadcast (PSI clear, ES MULTI2-scrambled).
    Raw,
}

/// The channel: a name, or a frequency with the ISDB-S TS selection.
#[derive(Args, Clone)]
pub(crate) struct ChannelArgs {
    /// Channel name: 13..62 (UHF physical channel), BS15 or BS15_1 (BS transponder
    /// and relative TS number, default 0), ND02 (CS110 transponder).
    #[arg(short, long, value_name = "NAME", conflicts_with_all = ["ter", "sat"])]
    channel: Option<String>,
    /// ISDB-T centre frequency in Hz (e.g. 557142857 for 27ch).
    #[arg(long, value_name = "HZ")]
    ter: Option<u32>,
    /// ISDB-S IF frequency in Hz (satellite RF - 10678 MHz, e.g. 1318000000 for BS15).
    #[arg(long, value_name = "HZ", conflicts_with = "ter")]
    sat: Option<u32>,
    /// ISDB-S: select this TSID (hex, e.g. 40f1) instead of a relative TS number.
    #[arg(long, value_name = "HEX", value_parser = parse_hex16)]
    tsid: Option<u16>,
    /// ISDB-S: relative TS number within the transponder (TMCC table index).
    #[arg(long, value_name = "N", conflicts_with = "tsid")]
    slot: Option<u8>,
}

#[derive(Args)]
struct TuneArgs {
    #[command(flatten)]
    dev: DeviceArgs,
    #[command(flatten)]
    chan: ChannelArgs,
    /// Stream mode.
    #[arg(long, value_enum, default_value_t = Mode::Device)]
    mode: Mode,
    /// ISDB-T: the terrestrial tuner to use on a unit with two (tuner 1 = the second
    /// terrestrial tuner of the two-tuner models; default 0).
    #[arg(long, value_name = "0|1", value_parser = clap::value_parser!(u8).range(0..=1))]
    tuner: Option<u8>,
    /// Device mode: the program the device descrambles: first, a service id
    /// (decimal or 0x hex), or all (= the last program of the PAT: the device
    /// descrambles exactly one program per tuning).
    #[arg(long, value_name = "PROG", default_value = "first")]
    program: String,
    /// Device mode: leave the link encryption in place (output stays encrypted; for analysis).
    #[arg(long)]
    no_decrypt: bool,
    /// Directory of the CEPM handshake cache (DTH10_<serial>.cepm). Default: $DTH_STATE_DIR,
    /// else /var/lib/dth when it is writable, else the directory of this executable.
    #[arg(long, value_name = "DIR")]
    state_dir: Option<PathBuf>,
    /// Neither read nor write the handshake cache (a new handshake every run).
    #[arg(long)]
    no_state: bool,
    /// Device mode: the vendor application's CoBDACom.dll (PCastTV3), from which the
    /// application certificate, key and IV are read. Default: $DTH10_APP_KEYS, else
    /// CoBDACom.dll in the state directory, else next to this executable.
    #[arg(long, value_name = "FILE")]
    app_keys: Option<PathBuf>,
    /// Stop after this many seconds (0 = until stopped).
    #[arg(short = 't', long, value_name = "SECONDS", default_value_t = 0.0)]
    duration: f64,
    /// Write the TS to this file instead of stdout.
    #[arg(short, long, value_name = "FILE")]
    output: Option<PathBuf>,
    /// Antenna (LNB) supply for BS/CS: 0 (off), 110 (11 V) or 150 (15 V). The DT-H70/U2
    /// has no such supply (the setting is for family models that have one).
    #[arg(long, value_name = "V", default_value_t = 0)]
    lnb: u32,
    /// ISDB-T: switch the attenuator on (the vendor application's "use attenuator";
    /// kits without one ignore it).
    #[arg(long)]
    attenuator: bool,
    /// ISDB-S: switch the booster on (the vendor application's "use booster"; separate from --lnb).
    #[arg(long)]
    booster: bool,
    /// ISDB-T: how long to wait for lock before streaming [ms] (up to 2 s longer while there is a signal).
    #[arg(long, value_name = "MS", default_value_t = 1200)]
    ter_lock_wait: u64,
    /// ISDB-S: how long to wait for lock and the TMCC TS-ID table [ms] (up to 2 s longer while there is a signal).
    #[arg(long, value_name = "MS", default_value_t = 1500)]
    sat_lock_wait: u64,
    /// ISDB-S: command path (bRequest bit7) of the satellite demodulator's I2C
    /// (default: as the vendor driver, from the device's descriptor flags).
    #[arg(long, value_name = "0|1")]
    sat_i2c_path: Option<u8>,
    /// ISDB-S: command path of the satellite stream-enable command.
    #[arg(long, value_name = "0|1", default_value_t = 1)]
    sat_stream_path: u8,
    /// ISDB-S: bulk IN endpoint of the satellite TS (hex; 0 = the second bulk IN endpoint).
    #[arg(long, value_name = "HEX", default_value = "0", value_parser = parse_hex8)]
    sat_endpoint: u8,
    /// Keep the bridge's TEI+null filler packets sent right after tuning.
    #[arg(long)]
    keep_filler: bool,
    /// Bulk IN transfers kept in flight (39936 bytes each); the stall the stream
    /// survives. Default 16 on Linux, 4 on Windows.
    #[arg(long, value_name = "N")]
    read_depth: Option<usize>,
    /// Print a line of statistics (packets/s, C/N) to stderr every second.
    #[arg(long)]
    stats: bool,
}

fn parse_hex16(s: &str) -> Result<u16, String> {
    let h = s.trim_start_matches("0x").trim_start_matches("0X");
    u16::from_str_radix(h, 16).map_err(|e| e.to_string())
}

fn parse_hex8(s: &str) -> Result<u8, String> {
    let h = s.trim_start_matches("0x").trim_start_matches("0X");
    u8::from_str_radix(h, 16).map_err(|e| e.to_string())
}

fn main() {
    let cli = Cli::parse();
    init_logging(&cli);
    let r = match cli.cmd {
        Cmd::List => list(),
        Cmd::Info(d) => info(&d),
        Cmd::Tune(t) => tune(&t),
        Cmd::Serve(a) => serve::run(&a),
        Cmd::Stream(a) => stream::run(&a),
        Cmd::Status(a) => stream::status(&a),
    };
    if let Err(e) = r {
        eprintln!("dth: {e}");
        std::process::exit(1);
    }
}

fn init_logging(cli: &Cli) {
    let level = cli.log_level.clone().unwrap_or_else(|| {
        match cli.verbose {
            0 => "warn",
            1 => "info",
            _ => "debug",
        }
        .into()
    });
    let mut b = env_logger::Builder::new();
    b.parse_filters(&level);
    if let Ok(env) = std::env::var("RUST_LOG") {
        b.parse_filters(&env);
    }
    b.format_timestamp_millis();
    let _ = b.try_init();
}

/// The backends of the families, DTH10 series first (the order the command
/// tries them in when no unit is named), with the options of `d`. `tune`
/// passes its own DTH10 series settings in `template`, the handshake cache
/// directory (none with `--no-state`) and the key file.
fn backends_with(
    d: &DeviceArgs,
    template: TunerConfig,
    state_dir: PathBuf,
    cepm_dir: Option<PathBuf>,
    keys: Option<PathBuf>,
) -> Vec<Arc<dyn Backend>> {
    let read_depth = template.read_depth;
    let keys = Arc::new(KeyStore::new(keys, state_dir.clone()));
    let mut template = template;
    template.recipient = recipient_mode(d);
    vec![
        Arc::new(Dth10Backend::new(template, cepm_dir, keys)),
        Arc::new(Dth30Backend::new(d.firmware.clone(), state_dir, read_depth)),
    ]
}

/// [`backends_with`] with the handshake cache in the state directory.
fn backends(d: &DeviceArgs, template: TunerConfig, state_dir: PathBuf, keys: Option<PathBuf>) -> Vec<Arc<dyn Backend>> {
    backends_with(d, template, state_dir.clone(), Some(state_dir), keys)
}

/// The units of every family, by family.
fn all_units(backends: &[Arc<dyn Backend>]) -> Vec<(Arc<dyn Backend>, Unit)> {
    let mut out = Vec::new();
    for b in backends {
        match b.units() {
            Ok(v) => out.extend(v.into_iter().map(|u| (b.clone(), u))),
            Err(e) => log::debug!("{} device list: {e}", b.family()),
        }
    }
    out
}

fn list() -> Result<(), Box<dyn Error>> {
    let units = all_units(&backends(&DeviceArgs::default(), TunerConfig::default(), default_state_dir(), None));
    if units.is_empty() {
        println!("no supported device found (VID 0411)");
    }
    for (_, u) in units {
        println!("{:04x}:{:04x} {} unit {} ({})", u.vendor_id, u.product_id, u.label, u.name, u.location);
    }
    Ok(())
}

fn recipient_mode(d: &DeviceArgs) -> RecipientMode {
    match d.recipient {
        Some(RecipientArg::Device) => RecipientMode::Device,
        Some(RecipientArg::Interface) => RecipientMode::Interface,
        None => RecipientMode::platform_default(),
    }
}

/// The unit the options name (`--serial`), or the first one found (the
/// DTH10 series first), with its family.
fn pick(backends: &[Arc<dyn Backend>], serial: Option<&str>) -> Result<(Arc<dyn Backend>, String), Box<dyn Error>> {
    all_units(backends)
        .into_iter()
        .find(|(_, u)| serial.is_none_or(|s| s == u.name))
        .map(|(b, u)| (b, u.name))
        .ok_or_else(|| match serial {
            Some(s) => format!("no supported device with serial or unit name {s}").into(),
            None => "no supported device found".into(),
        })
}

/// A device node given on the command line (Linux): always the DTH10 series.
#[cfg(any(target_os = "linux", target_os = "android"))]
fn device_node(d: &DeviceArgs) -> Option<&Path> {
    d.device.as_deref()
}

#[cfg(not(any(target_os = "linux", target_os = "android")))]
fn device_node(_d: &DeviceArgs) -> Option<&Path> {
    None
}

#[cfg(any(target_os = "linux", target_os = "android"))]
fn open_node(p: &Path, d: &DeviceArgs) -> Result<UsbTransport, Box<dyn Error>> {
    Ok(UsbTransport::open_path(p, recipient_mode(d))?)
}

#[cfg(not(any(target_os = "linux", target_os = "android")))]
fn open_node(_p: &Path, _d: &DeviceArgs) -> Result<UsbTransport, Box<dyn Error>> {
    Err("device nodes are a Linux option".into())
}

/// `info` of the unit the options name, or of every unit found (one block
/// each, separated by a blank line). A unit that cannot be read (another
/// program holds it) is reported and the others are still shown.
fn info(d: &DeviceArgs) -> Result<(), Box<dyn Error>> {
    if let Some(p) = device_node(d) {
        print!("{}", Dth10Backend::describe(open_node(p, d)?)?);
        return Ok(());
    }
    let backends = backends(d, TunerConfig::default(), default_state_dir(), None);
    if let Some(s) = d.serial.as_deref() {
        let (b, name) = pick(&backends, Some(s))?;
        print!("{}", b.info(&name)?);
        return Ok(());
    }
    let units = all_units(&backends);
    if units.is_empty() {
        return Err("no supported device found".into());
    }
    let mut failed = 0;
    for (i, (b, u)) in units.iter().enumerate() {
        if i > 0 {
            println!();
        }
        match b.info(&u.name) {
            Ok(text) => print!("{text}"),
            Err(e) => {
                println!("model: {} ({:04x})", u.label, u.product_id);
                println!("unit: {}", u.name);
                println!("error: {e}");
                failed += 1;
            }
        }
    }
    if failed > 0 {
        return Err(format!("{failed} of {} devices could not be read", units.len()).into());
    }
    Ok(())
}

/// The channel of the options: `--channel`, `--ter` or `--sat` (+ `--tsid` / `--slot`).
pub(crate) fn channel_of(a: &ChannelArgs) -> Result<Channel, String> {
    let mut ch = if let Some(name) = a.channel.as_deref() {
        Channel::parse(name).ok_or_else(|| format!("unknown channel {name:?} (13..62, BSxx[_n], NDxx[_n])"))?
    } else if let Some(hz) = a.ter {
        Channel::terrestrial_hz(hz)
    } else if let Some(hz) = a.sat {
        Channel::satellite_hz(hz, a.tsid)
    } else {
        return Err("--channel, --ter or --sat is required".into());
    };
    if ch.system == System::IsdbS {
        if let (Some(t), true) = (a.tsid, a.channel.is_some()) {
            ch = ch.with_tsid(t);
        }
        if let Some(s) = a.slot {
            ch.slot = Some(s);
        }
    } else if a.tsid.is_some() || a.slot.is_some() {
        return Err("--tsid / --slot apply to ISDB-S channels only".into());
    }
    Ok(ch)
}

#[cfg(unix)]
fn writable_dir(p: &Path) -> bool {
    use std::ffi::CString;
    use std::os::unix::ffi::OsStrExt;
    let Ok(c) = CString::new(p.as_os_str().as_bytes()) else { return false };
    // SAFETY: `c` is a valid NUL-terminated path for the duration of the call.
    p.is_dir() && unsafe { libc::access(c.as_ptr(), libc::W_OK) } == 0
}

/// The state directory: `$DTH_STATE_DIR` (or the former `$DTH10_STATE_DIR`),
/// else `/var/lib/dth` when it is writable (or the former `/var/lib/dth10`,
/// which keeps its handshake cache), else the directory of this executable.
pub(crate) fn default_state_dir() -> PathBuf {
    if let Some(d) = std::env::var_os("DTH_STATE_DIR").or_else(|| std::env::var_os("DTH10_STATE_DIR")) {
        return PathBuf::from(d);
    }
    #[cfg(unix)]
    for p in [Path::new("/var/lib/dth"), Path::new("/var/lib/dth10")] {
        if writable_dir(p) {
            return p.to_path_buf();
        }
    }
    std::env::current_exe().ok().and_then(|p| p.parent().map(Path::to_path_buf)).unwrap_or_else(|| PathBuf::from("."))
}

/// The tuner of `--tuner` for a channel of `system`: terrestrial channels only.
pub(crate) fn tuner_of(tuner: Option<u8>, system: System) -> Result<usize, String> {
    match (tuner, system) {
        (Some(_), System::IsdbS) => Err("--tuner applies to ISDB-T channels only".into()),
        (Some(n), System::IsdbT) => Ok(n as usize),
        (None, s) => Ok(default_tuner(s)),
    }
}

/// The LNB option: (voltage, on).
pub(crate) fn lnb_of(v: u32) -> (u32, bool) {
    let volt = match v {
        0 => 0,
        v if v <= 110 => 110,
        _ => 150,
    };
    (volt, v != 0)
}

fn tune(a: &TuneArgs) -> Result<(), Box<dyn Error>> {
    let ch = channel_of(&a.chan)?;
    let tuner_index = tuner_of(a.tuner, ch.system)?;
    let program = DeviceProgram::parse(&a.program).ok_or_else(|| format!("bad --program {:?}", a.program))?;
    let state_dir = a.state_dir.clone().unwrap_or_else(default_state_dir);
    let cepm_dir = if a.no_state { None } else { Some(state_dir.clone()) };
    let template = tune_config(a, program);
    let raw = a.mode == Mode::Raw;
    let req = Request { system: ch.system, tuner: tuner_index, raw };
    let queue = TsQueue::new();
    let t0;
    let mut tuner: Box<dyn Session> = if let Some(p) = device_node(&a.dev) {
        let keys = Arc::new(KeyStore::new(a.app_keys.clone(), state_dir.clone()));
        let b = Dth10Backend::new(template, cepm_dir, keys);
        b.check("", req)?;
        stop::install();
        t0 = Instant::now();
        b.open_transport(open_node(p, &a.dev)?, queue.clone())?
    } else {
        let backends = backends_with(&a.dev, template, state_dir.clone(), cepm_dir, a.app_keys.clone());
        let (b, name) = pick(&backends, a.dev.serial.as_deref())?;
        // What the unit's family needs (the key file, the firmware, a
        // tuner it has), before the device is touched.
        b.check(&name, req)?;
        stop::install();
        t0 = Instant::now();
        b.open(&name, queue.clone())?
    };
    tuner.set_stream_mode(if raw { StreamModeCfg::Raw } else { StreamModeCfg::Device });
    tuner.set_device_program(program);
    tuner.set_lock_wait_ms(System::IsdbT, a.ter_lock_wait);
    tuner.set_lock_wait_ms(System::IsdbS, a.sat_lock_wait);
    log::info!("device ready in {:?}", t0.elapsed());
    tuner.set_channel_on(tuner_index, &ch)?;
    log::info!("streaming {} after {:?}, C/N {:.1} dB", ch.name, t0.elapsed(), tuner.signal_level_of(tuner_index));
    let mut out: Box<dyn Write> = match a.output.as_ref() {
        Some(p) => Box::new(File::create(p)?),
        None => Box::new(io::stdout().lock()),
    };
    let limit = (a.duration > 0.0).then(|| Duration::from_secs_f64(a.duration));
    let start = Instant::now();
    let mut total = 0u64;
    let mut sec_bytes = 0u64;
    let mut last_stats = Instant::now();
    let mut why = "stopped";
    'run: while !stop::stopping() {
        if limit.is_some_and(|l| start.elapsed() >= l) {
            why = "duration reached";
            break;
        }
        if queue.wait(200) {
            while let Some(chunk) = queue.pop() {
                if let Err(e) = out.write_all(&chunk) {
                    why = if e.kind() == io::ErrorKind::BrokenPipe {
                        "output closed"
                    } else {
                        log::error!("write: {e}");
                        "write error"
                    };
                    break 'run;
                }
                total += chunk.len() as u64;
                sec_bytes += chunk.len() as u64;
            }
        }
        // The status is read once a second with or without --stats: some kits
        // follow the carrier offset and acquire the signal again after a lost
        // lock only inside the status read.
        if last_stats.elapsed() >= Duration::from_secs(1) {
            let cnr = tuner.poll_status()[tuner_index];
            if a.stats {
                let dt = last_stats.elapsed().as_secs_f64();
                eprintln!(
                    "[t+{:6.1}s] {:6.0} pkt/s {:5.2} MB/s  C/N {:5.1} dB  total {:.1} MB",
                    start.elapsed().as_secs_f64(),
                    sec_bytes as f64 / 188.0 / dt,
                    sec_bytes as f64 / 1e6 / dt,
                    cnr.unwrap_or(0.0),
                    total as f64 / 1e6
                );
            }
            last_stats = Instant::now();
            sec_bytes = 0;
        }
    }
    let _ = out.flush();
    drop(out);
    tuner.close();
    log::info!("{why}: {total} bytes in {:.1} s", start.elapsed().as_secs_f64());
    Ok(())
}

/// The DTH10 series settings of `tune`.
fn tune_config(a: &TuneArgs, program: DeviceProgram) -> TunerConfig {
    let (lnb_voltage, lnb_power) = lnb_of(a.lnb);
    TunerConfig {
        recipient: recipient_mode(&a.dev),
        serial: String::new(),
        stream_mode: if a.mode == Mode::Device { StreamModeCfg::Device } else { StreamModeCfg::Raw },
        device_program: program,
        device_decrypt: !a.no_decrypt,
        cepm_state: None,
        app_keys: None,
        drop_filler: !a.keep_filler,
        read_depth: a.read_depth.unwrap_or_else(dth_common::stream::default_read_depth).clamp(1, 64),
        exclusive: true,
        log_events: false,
        ter_lock_wait_ms: a.ter_lock_wait,
        attenuator: a.attenuator,
        ter_bandwidth_hz: 6_000_000,
        sat_i2c_path: a.sat_i2c_path.map(|p| p.min(1)),
        sat_stream_path: a.sat_stream_path.min(1),
        sat_endpoint: a.sat_endpoint,
        lnb_voltage,
        lnb_power,
        booster: a.booster,
        sat_lock_wait_ms: a.sat_lock_wait,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(v: &[&str]) -> TuneArgs {
        let cli = Cli::parse_from(std::iter::once("dth").chain(v.iter().copied()));
        match cli.cmd {
            Cmd::Tune(t) => *t,
            _ => panic!("not tune"),
        }
    }

    #[test]
    fn channels_from_options() {
        let c = channel_of(&args(&["tune", "-c", "27"]).chan).unwrap();
        assert_eq!((c.system, c.frequency_hz), (System::IsdbT, 557_142_857));
        let c = channel_of(&args(&["tune", "--ter", "557142857"]).chan).unwrap();
        assert_eq!(c.frequency_hz, 557_142_857);
        let c = channel_of(&args(&["tune", "-c", "BS15_1"]).chan).unwrap();
        assert_eq!((c.frequency_hz, c.slot), (1_318_000_000, Some(1)));
        let c = channel_of(&args(&["tune", "-c", "BS15", "--tsid", "40f2"]).chan).unwrap();
        assert_eq!((c.tsid, c.slot), (0x40F2, None));
        let c = channel_of(&args(&["tune", "--sat", "1613000000"]).chan).unwrap();
        assert_eq!((c.system, c.slot), (System::IsdbS, Some(0)));
        let c = channel_of(&args(&["tune", "--sat", "1613000000", "--tsid", "6020"]).chan).unwrap();
        assert_eq!((c.tsid, c.slot), (0x6020, None));
        assert!(channel_of(&args(&["tune"]).chan).is_err());
        assert!(channel_of(&args(&["tune", "-c", "27", "--slot", "1"]).chan).is_err());
        assert!(channel_of(&args(&["tune", "-c", "BS16"]).chan).is_err());
    }

    #[test]
    fn lnb_values() {
        assert_eq!(lnb_of(0), (0, false));
        assert_eq!(lnb_of(11), (110, true));
        assert_eq!(lnb_of(150), (150, true));
    }

    #[test]
    fn switches_default_off() {
        let a = args(&["tune", "-c", "27"]);
        assert_eq!((a.attenuator, a.booster, a.lnb), (false, false, 0));
        let a = args(&["tune", "-c", "27", "--attenuator", "--booster"]);
        assert_eq!((a.attenuator, a.booster, a.lnb), (true, true, 0), "the booster does not imply the LNB supply");
    }

    #[test]
    fn tuner_option() {
        assert_eq!(args(&["tune", "-c", "27"]).tuner, None);
        assert_eq!(args(&["tune", "-c", "27", "--tuner", "1"]).tuner, Some(1));
        assert!(Cli::try_parse_from(["dth", "tune", "-c", "27", "--tuner", "2"]).is_err());
        assert_eq!(tuner_of(None, System::IsdbT), Ok(0));
        assert_eq!(tuner_of(None, System::IsdbS), Ok(1));
        assert_eq!(tuner_of(Some(1), System::IsdbT), Ok(1));
        assert!(tuner_of(Some(0), System::IsdbS).is_err(), "BS/CS always use the satellite tuner");
    }
}
