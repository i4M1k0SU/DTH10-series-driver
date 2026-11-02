//! `dth10`: stand-alone tuner command for the BUFFALO DTH10 series of USB TV
//! tuners (the models the vendor's Windows driver covers; verified on
//! a DT-H70/U2).
//!
//! `dth10 tune` opens the device, tunes, and writes the transport stream to
//! stdout (or a file) until it is stopped (SIGTERM / SIGINT / Ctrl-C, the
//! reader closing the pipe, or `--duration`): the tuner command a recorder
//! such as Mirakurun or mirakc runs. Every run opens the device itself. In the
//! device-side descrambling mode (the default) the result of the CEPM
//! handshake (docs/22) is cached in `<state dir>/DTH10_<serial>.cepm` so later runs
//! skip it while the device keeps the session. The application credentials
//! the handshake needs are read from the user's copy of the vendor
//! application's `CoBDACom.dll` (`--app-keys`); they are not built in.
//!
//! `dth10 serve` holds the device open for good instead and streams to
//! `dth10 stream` clients over a Unix domain socket (on Windows too, where
//! the BonDriver starts it as its server; TCP on the loopback interface is
//! the alternative): the terrestrial and the satellite stream of one unit then run
//! independently, so a recorder can list the unit as two tuners (GR and
//! BS/CS), or two TVTest windows can watch GR and BS at once.

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
use dth10_core::Device;
use dth10_core::cepm::AppKeys;
use dth10_core::tlv::DeviceInformation;
use dth10_tuner::{Channel, DeviceProgram, StreamModeCfg, System, TsQueue, Tuner, TunerConfig};
use dth10_usb::{RecipientMode, UsbTransport};

#[derive(Parser)]
#[command(name = "dth10", version, about = "User-mode tuner command for the BUFFALO DTH10 series of USB TV tuners")]
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
    /// Show the device information (tuner descriptors, firmware, endpoints).
    Info(DeviceArgs),
    /// Tune and write the TS to stdout (or --output) until stopped.
    Tune(Box<TuneArgs>),
    /// Hold the device open for good and stream to `dth10 stream` clients over a socket.
    Serve(serve::ServeArgs),
    /// Ask `dth10 serve` for a stream and write its TS to stdout (a recorder's tuner command).
    Stream(Box<stream::StreamArgs>),
    /// Ask `dth10 serve` for the C/N of both systems (a health check).
    Status(stream::StatusArgs),
}

#[derive(Args, Clone)]
struct DeviceArgs {
    /// Select the device by USB serial number (default: the first one found).
    #[arg(long, value_name = "SERIAL")]
    serial: Option<String>,
    /// Open this usbfs node (/dev/bus/usb/BBB/DDD) instead of searching.
    #[cfg(any(target_os = "linux", target_os = "android"))]
    #[arg(long, value_name = "PATH")]
    device: Option<PathBuf>,
    /// bmRequestType recipient of the vendor requests (default: device on Windows, interface elsewhere).
    #[arg(long, value_enum)]
    recipient: Option<RecipientArg>,
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
    /// Device mode: the program the device descrambles: first, a service id
    /// (decimal or 0x hex), or all (= the last program of the PAT: the device
    /// descrambles exactly one program per tuning).
    #[arg(long, value_name = "PROG", default_value = "first")]
    program: String,
    /// Device mode: leave the link encryption in place (output stays encrypted; for analysis).
    #[arg(long)]
    no_decrypt: bool,
    /// Directory of the CEPM handshake cache (DTH10_<serial>.cepm). Default: $DTH10_STATE_DIR,
    /// else /var/lib/dth10 when it is writable, else the directory of this executable.
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
    /// ISDB-T: how long to wait for lock before streaming [ms].
    #[arg(long, value_name = "MS", default_value_t = 1200)]
    ter_lock_wait: u64,
    /// ISDB-S: how long to wait for lock and the TMCC TS-ID table [ms].
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
        eprintln!("dth10: {e}");
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

fn list() -> Result<(), Box<dyn Error>> {
    let found = UsbTransport::list_all()?;
    if found.is_empty() {
        println!("no supported device found (VID 0411)");
    }
    for d in found {
        println!(
            "{:04x}:{:04x} {} serial {} ({})",
            d.vendor_id,
            d.product_id,
            d.product_name(),
            d.serial.as_deref().unwrap_or("-"),
            d.location
        );
    }
    Ok(())
}

fn open_transport(d: &DeviceArgs) -> Result<UsbTransport, Box<dyn Error>> {
    let mode = match d.recipient {
        Some(RecipientArg::Device) => RecipientMode::Device,
        Some(RecipientArg::Interface) => RecipientMode::Interface,
        None => RecipientMode::platform_default(),
    };
    #[cfg(any(target_os = "linux", target_os = "android"))]
    if let Some(p) = d.device.as_ref() {
        return Ok(UsbTransport::open_path(p, mode)?);
    }
    Ok(UsbTransport::open_matching(mode, d.serial.as_deref())?)
}

fn print_desc(index: u8, di: &DeviceInformation) {
    let d = &di.desc;
    println!(
        "tuner #{index}: kit {:?} model {:?} hw 0x{:x} flags 0x{:x} demod 0x{:02x} tuner 0x{:02x} IF {} Hz",
        d.kit_name, d.model, d.hw_version, d.flags, d.demod_i2c_addr, d.tuner_i2c_addr, d.if_freq_hz
    );
}

fn info(d: &DeviceArgs) -> Result<(), Box<dyn Error>> {
    let t = open_transport(d)?;
    println!("serial: {}", t.serial().unwrap_or("-"));
    println!("endpoints: {:?}", t.pipes());
    let dev0 = Device::new(t.clone());
    let di = dev0.read_device_information()?;
    print_desc(0, &di);
    match dev0.firmware_version() {
        Ok((a, b, c)) => println!("firmware: {a}.{b}.{c}"),
        Err(e) => println!("firmware: {e}"),
    }
    if di.desc.flags & 1 != 0 {
        let dev1 = Device::new(t).with_tuner_index(1);
        match dev1.read_device_information() {
            Ok(di1) => print_desc(1, &di1),
            Err(e) => println!("tuner #1: {e}"),
        }
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

pub(crate) fn default_state_dir() -> PathBuf {
    if let Some(d) = std::env::var_os("DTH10_STATE_DIR") {
        return PathBuf::from(d);
    }
    #[cfg(unix)]
    {
        let p = Path::new("/var/lib/dth10");
        if writable_dir(p) {
            return p.to_path_buf();
        }
    }
    std::env::current_exe().ok().and_then(|p| p.parent().map(Path::to_path_buf)).unwrap_or_else(|| PathBuf::from("."))
}

/// The application key file: `--app-keys`, `$DTH10_APP_KEYS`, `CoBDACom.dll`
/// in the state directory, `CoBDACom.dll` next to the executable.
pub(crate) fn app_keys_path(explicit: Option<&Path>, state_dir: &Path) -> Option<PathBuf> {
    if let Some(p) = explicit {
        return Some(p.to_path_buf());
    }
    if let Some(p) = std::env::var_os("DTH10_APP_KEYS") {
        return Some(PathBuf::from(p));
    }
    let candidates = [
        Some(state_dir.join("CoBDACom.dll")),
        std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.join("CoBDACom.dll"))),
    ];
    candidates.into_iter().flatten().find(|p| p.is_file())
}

/// `<state dir>/DTH10_<serial>.cepm` (the prefix tells the file's product in a
/// folder shared with other tuners' files), with the directory created; `None`
/// when the directory cannot be used.
pub(crate) fn state_file_for(dir: &Path, serial: Option<&str>) -> Option<PathBuf> {
    if let Err(e) = std::fs::create_dir_all(dir) {
        log::warn!("state directory {}: {e}; running without the handshake cache", dir.display());
        return None;
    }
    let name: String = serial
        .unwrap_or("device")
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() || c == '-' || c == '_' { c } else { '_' })
        .collect();
    Some(dir.join(format!("DTH10_{name}.cepm")))
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
    let program = DeviceProgram::parse(&a.program).ok_or_else(|| format!("bad --program {:?}", a.program))?;
    let state_dir = a.state_dir.clone().unwrap_or_else(default_state_dir);
    let decrypt = !a.no_decrypt;
    // Read the application credentials before touching the device: a missing
    // file is the most likely setup problem and should fail at once. BS/CS
    // channels need them whatever the mode (the satellite stream only exists
    // in device mode).
    let need_keys = decrypt && (a.mode == Mode::Device || ch.system == System::IsdbS);
    let (keys, keys_path) = if need_keys {
        let path = app_keys_path(a.app_keys.as_deref(), &state_dir)
            .ok_or_else(|| format!("device mode (and every BS/CS channel) needs the vendor application's CoBDACom.dll (PCastTV3): put it in {} or give --app-keys", state_dir.display()))?;
        let t = Instant::now();
        let keys = AppKeys::load(&path)?;
        log::info!(
            "application credentials read from {} in {:?} ({} IV candidates)",
            path.display(),
            t.elapsed(),
            keys.iv_candidates()
        );
        (Some(Arc::new(keys)), Some(path))
    } else {
        (None, None)
    };
    stop::install();
    let t0 = Instant::now();
    let transport = open_transport(&a.dev)?;
    let serial = transport.serial().map(str::to_owned);
    log::info!("device {} opened in {:?}", serial.as_deref().unwrap_or("?"), t0.elapsed());
    let (lnb_voltage, lnb_power) = lnb_of(a.lnb);
    let cfg = TunerConfig {
        recipient: RecipientMode::platform_default(),
        serial: String::new(),
        stream_mode: if a.mode == Mode::Device { StreamModeCfg::Device } else { StreamModeCfg::Raw },
        device_program: program,
        device_decrypt: decrypt,
        cepm_state: if a.no_state { None } else { state_file_for(&state_dir, serial.as_deref()) },
        app_keys: keys_path,
        drop_filler: !a.keep_filler,
        read_depth: a.read_depth.unwrap_or_else(dth10_tuner::config::default_read_depth).clamp(1, 64),
        exclusive: true,
        log_events: false,
        ter_lock_wait_ms: a.ter_lock_wait,
        ter_bandwidth_hz: 6_000_000,
        sat_i2c_path: a.sat_i2c_path.map(|p| p.min(1)),
        sat_stream_path: a.sat_stream_path.min(1),
        sat_endpoint: a.sat_endpoint,
        lnb_voltage,
        lnb_power,
        sat_lock_wait_ms: a.sat_lock_wait,
    };
    let queue = TsQueue::new();
    let mut tuner = Tuner::open_with(transport, &cfg, queue.clone())?;
    if let Some(k) = keys {
        tuner.with_app_keys(k);
    }
    log::info!("device ready in {:?}", t0.elapsed());
    tuner.set_channel(&ch)?;
    log::info!("streaming {} after {:?}, C/N {:.1} dB", ch.name, t0.elapsed(), tuner.signal_level());

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
        if a.stats && last_stats.elapsed() >= Duration::from_secs(1) {
            let dt = last_stats.elapsed().as_secs_f64();
            eprintln!(
                "[t+{:6.1}s] {:6.0} pkt/s {:5.2} MB/s  C/N {:5.1} dB  total {:.1} MB",
                start.elapsed().as_secs_f64(),
                sec_bytes as f64 / 188.0 / dt,
                sec_bytes as f64 / 1e6 / dt,
                tuner.signal_level(),
                total as f64 / 1e6
            );
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

#[cfg(test)]
mod tests {
    use super::*;

    fn args(v: &[&str]) -> TuneArgs {
        let cli = Cli::parse_from(std::iter::once("dth10").chain(v.iter().copied()));
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
    fn state_file_name() {
        let dir = std::env::temp_dir().join("dth10-test-state");
        assert_eq!(
            state_file_for(&dir, Some("04110199-1108 22N0326")),
            Some(dir.join("DTH10_04110199-1108_22N0326.cepm"))
        );
        let _ = std::fs::remove_dir(&dir);
        let a = args(&["tune", "-c", "27", "--app-keys", "k.dll"]);
        assert_eq!(app_keys_path(a.app_keys.as_deref(), &dir), Some(PathBuf::from("k.dll")));
        assert_eq!(lnb_of(0), (0, false));
        assert_eq!(lnb_of(11), (110, true));
        assert_eq!(lnb_of(150), (150, true));
    }
}
