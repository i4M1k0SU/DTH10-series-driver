//! `dth10-diag`: hardware diagnostics for the BUFFALO DTH10 series of USB TV
//! tuners (Cobalt EVA01 bridge), for the developer debugging the hardware.
//! Watching and recording are the `dth10` command (`dth10 tune -c 27`,
//! `dth10 tune -c BS15_0`) and the BonDriver, not this tool.
//!
//! Commands:
//!
//! * `info [--raw]` – device information TLV of the `--tuner` path (hex dump with `--raw`), firmware version, endpoints
//! * `gpio-read` – the 16-byte GPIO/status block
//! * `raw` – an arbitrary vendor command (wValue/wIndex, OUT bytes, IN length)
//! * `events` – the interrupt-endpoint events for a number of seconds
//! * `i2c-read` / `i2c-write` – I2C registers through the bridge
//! * `tune` – ISDB-T probe: tune a UHF channel, wait for lock, print the signal
//!   status (lock bits, C/N, BER); optionally capture stream #0 in raw mode
//! * `tune-sat` – ISDB-S probe: tune a BS/CS110 transponder, wait for lock, print
//!   the signal status with the TMCC TS-ID table, select the TS
//! * `scan` – ISDB-T lock and C/N of a range of UHF channels
//! * `cepm` – run the CEPM handshake (docs/22) and print its values
//! * `reset` – USB port reset (the device re-enumerates; not on Windows)
//! * `hub-power` – switch the power of the tuner's hub port (Linux only)
//!
//! Raw vendor commands, CEPM handshakes (an interrupted one locks the device's
//! handshake, docs/22 §6b), a raw-mode capture on stream #0 (it stops the
//! satellite stream, docs/40) and hub-power can leave the device unusable until
//! it is power-cycled. That is why they are here and not in `dth10`.

use std::error::Error;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use clap::{Args, Parser, Subcommand, ValueEnum};
use dth10_core::cepm;
use dth10_core::kit::{BER_UNUSED, SignalStatus, TuneRequest, field, lock};
use dth10_core::satellite::SatelliteKit;
use dth10_core::terrestrial::TerrestrialKit;
use dth10_core::ts_slot::{self, SlotLayout};
use dth10_core::{Device, StreamMode, TunerDesc, unverified_kit_error};
use dth10_tuner::channels::uhf_center_hz;
use dth10_tuner::config::parse_num;
use dth10_tuner::{Channel, System};
use dth10_usb::{EventReader, RecipientMode, StreamReader, UsbTransport};

type Res<T = ()> = Result<T, Box<dyn Error>>;

#[derive(Parser)]
#[command(
    name = "dth10-diag",
    version,
    about = "Hardware diagnostics for the BUFFALO DTH10 series of USB TV tuners (developer tool; recording is `dth10 tune`)"
)]
struct Cli {
    /// Select the device by USB serial number (default: the first one found; `info` shows every one).
    #[arg(long, global = true, help_heading = "Global options", value_name = "SERIAL")]
    serial: Option<String>,
    /// bmRequestType recipient of the vendor requests (default: device on Windows, interface elsewhere).
    #[arg(long, global = true, help_heading = "Global options", value_enum)]
    recipient: Option<RecipientArg>,
    /// Command path (bRequest bit7) of info, gpio-read, raw, i2c-read, i2c-write and cepm:
    /// 0 = tuner #0 (terrestrial), 1 = tuner #1. tune and scan always use tuner #0, tune-sat has --i2c-tuner.
    #[arg(long, global = true, help_heading = "Global options", value_name = "0|1", default_value_t = 0, value_parser = clap::value_parser!(u8).range(0..=1))]
    tuner: u8,
    /// Print the interrupt-endpoint events from a background thread while the command runs.
    #[arg(long, global = true, help_heading = "Global options")]
    drain_events: bool,
    /// -v = info, -vv = debug (default: warn; RUST_LOG overrides). Logs go to stderr.
    #[arg(short, long, global = true, help_heading = "Global options", action = clap::ArgAction::Count)]
    verbose: u8,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Clone, Copy, ValueEnum)]
enum RecipientArg {
    Interface,
    Device,
}

#[derive(Clone, Copy, PartialEq, Eq, ValueEnum)]
enum HubAction {
    Status,
    Off,
    On,
    /// Off, 2 s, on.
    Cycle,
}

#[derive(Subcommand)]
enum Cmd {
    /// Read the device-information TLV of the --tuner path, the firmware version and the endpoint map.
    Info {
        /// Also hex-dump the raw TLV blob.
        #[arg(long)]
        raw: bool,
    },
    /// Read the 16-byte GPIO/status block (wIndex 0x1010).
    GpioRead,
    /// Send an arbitrary vendor command on the --tuner path: wValue/wIndex in hex, optional OUT
    /// bytes (hex) and IN length.
    ///
    /// Commands whose effect is not known can leave the device unusable until it is power-cycled.
    Raw {
        /// wValue (hex).
        #[arg(value_parser = parse_hex16)]
        value: u16,
        /// wIndex (hex).
        #[arg(value_parser = parse_hex16)]
        index: u16,
        /// OUT payload as hex digits, e.g. 00000000ff00000000000000.
        #[arg(long, value_name = "HEX", value_parser = parse_hex_bytes)]
        out: Option<HexBytes>,
        /// Number of bytes to read back.
        #[arg(long, value_name = "N", default_value_t = 0)]
        r#in: usize,
        /// Extra bRequest flag bits (kept & 0xFC), decimal or 0x hex (e.g. 0x40).
        #[arg(long, default_value = "0", value_parser = parse_u8_auto)]
        flags: u8,
    },
    /// Print the interrupt-endpoint events for a number of seconds.
    Events {
        #[arg(long, default_value_t = 10)]
        seconds: u64,
    },
    /// Read I2C registers through the bridge on the --tuner path: 7-bit address, start register,
    /// count (decimal, or hex with a 0x prefix: 0x19; a bare 19 is decimal).
    I2cRead {
        #[arg(value_parser = parse_u8_auto)]
        addr: u8,
        #[arg(value_parser = parse_u8_auto)]
        reg: u8,
        #[arg(default_value = "1", value_parser = parse_u8_auto)]
        count: u8,
    },
    /// Write I2C registers through the bridge on the --tuner path: 7-bit address and register
    /// (decimal, or hex with 0x), data bytes (hex digits).
    I2cWrite {
        #[arg(value_parser = parse_u8_auto)]
        addr: u8,
        #[arg(value_parser = parse_u8_auto)]
        reg: u8,
        #[arg(value_parser = parse_hex_bytes)]
        data: HexBytes,
    },
    /// ISDB-T probe: tune a UHF channel on tuner #0, wait for lock and print the signal status.
    ///
    /// Initialises the terrestrial kit, sends the device start, opens the kit, tunes, then prints
    /// the signal status (lock bits, sync state, C/N, quality, strength, BER of layers A/B/C)
    /// every 250 ms until it locks or --lock-wait passes. Exits with an error without lock.
    ///
    /// With --file, stream #0 (EP 0x81) is captured after lock, in raw mode (enable byte0 = 1,
    /// the full TS as broadcast) unless --enable-bytes gives another payload. Raw mode on stream
    /// #0 stops the satellite stream until the device is power-cycled (docs/40): BS/CS channels
    /// do not work again before that. Recording a channel is `dth10 tune -c 27` (device mode with
    /// link decryption), not this command.
    Tune(TuneArgs),
    /// ISDB-S probe: tune a BS/CS110 transponder, wait for lock and print the signal status with
    /// the TMCC TS-ID table, then select the TS.
    ///
    /// Same shape as `tune`: initialises the terrestrial kit and the satellite kit (the order of
    /// the original driver and of `dth10`), sends the device start, opens the satellite kit,
    /// tunes, then prints the signal status (lock, C/N, BER, TS-ID table) every 250 ms until it
    /// locks with a TS-ID table or --lock-wait passes. The TS is then selected: the table entry of
    /// the channel's relative TS number (BS15_1 = entry 1), else the channel's own TSID, or --tsid.
    /// Exits with an error without lock.
    ///
    /// No stream is started: the device outputs the satellite stream only in its own mode with
    /// link decryption (docs/40), which is what `dth10` does. Recording a satellite channel is
    /// `dth10 tune -c BS15_0` (device mode).
    TuneSat(TuneSatArgs),
    /// ISDB-T: tune a range of UHF channels on tuner #0 and print lock / C/N for each.
    Scan {
        /// First UHF channel (13..62).
        #[arg(long, default_value_t = 13, value_parser = clap::value_parser!(u32).range(13..=62))]
        first: u32,
        /// Last UHF channel (13..62).
        #[arg(long, default_value_t = 62, value_parser = clap::value_parser!(u32).range(13..=62))]
        last: u32,
        /// Settling time per channel in milliseconds.
        #[arg(long, default_value_t = 700)]
        wait_ms: u64,
    },
    /// Run the CEPM handshake (vendor command 0xDEAF/1..6, docs/22) and print the device
    /// certificate, nonces, shared secret and session identity.
    ///
    /// The six steps run in one go: a handshake interrupted half way locks the device's handshake
    /// until it is power-cycled (docs/22 §6b). `dth10` runs and caches its handshakes itself.
    Cepm {
        /// The vendor application's CoBDACom.dll (PCastTV3) holding the application credentials.
        /// Default: $DTH10_APP_KEYS, else ./CoBDACom.dll.
        #[arg(long, value_name = "FILE")]
        app_keys: Option<PathBuf>,
        /// Request flag bits of the handshake commands (decimal or 0x hex): 0x00 = tuner #0 path,
        /// 0x80 = tuner #1 path. Default: the path of --tuner.
        #[arg(long, value_name = "FLAGS", value_parser = parse_u8_auto)]
        flags: Option<u8>,
        /// Save the handshake to this file in the `DTH10_<serial>.cepm` format of `dth10` (one record per
        /// path; the record of this path is replaced, others in the file are kept).
        #[arg(long, value_name = "FILE")]
        save: Option<PathBuf>,
    },
    /// Reset the tuner's USB port (usbfs USBDEVFS_RESET): the device re-enumerates without a
    /// power cycle. Not supported on Windows (the USB library has no reset there).
    Reset,
    /// Switch the power of the hub port the tuner hangs off (the hub's PORT_POWER feature; only
    /// hubs with per-port power switching really cut the power). A remote stand-in for pulling
    /// the tuner's cable. Linux only.
    HubPower {
        #[arg(value_enum)]
        action: HubAction,
        /// The hub as BUS:ADDR (default: the tuner's parent hub).
        #[arg(long)]
        hub: Option<String>,
        /// The port on that hub (default: the tuner's port).
        #[arg(long)]
        port: Option<u8>,
    },
}

#[derive(Args)]
struct TuneArgs {
    /// UHF physical channel 13..62 (27 or 27ch), or the centre frequency in Hz (e.g. 557142857).
    #[arg(value_parser = ter_channel)]
    channel: Channel,
    /// How long to wait for lock [ms].
    #[arg(long, value_name = "MS", default_value_t = 1500)]
    lock_wait: u64,
    /// Skip the device start command (wValue 0 / wIndex 0x10) after the kit Init.
    #[arg(long)]
    no_start: bool,
    /// Switch the attenuator on before tuning (kits without one ignore it).
    #[arg(long)]
    attenuator: bool,
    /// After lock, capture stream #0 to this file (raw mode unless --enable-bytes; stops the
    /// satellite stream until the device is power-cycled, see the description above).
    #[arg(long, value_name = "FILE", help_heading = "Capture (raw mode)")]
    file: Option<PathBuf>,
    /// Capture duration in seconds.
    #[arg(long, value_name = "S", default_value_t = 10, requires = "file", help_heading = "Capture (raw mode)")]
    seconds: u64,
    /// Write the 256-byte bulk blocks as received instead of the 188-byte TS packets.
    #[arg(long, requires = "file", help_heading = "Capture (raw mode)")]
    blocks: bool,
    /// The 16-byte stream-enable payload (32 hex digits; byte0 = mode) instead of raw full-TS mode (byte0 = 1).
    #[arg(long, value_name = "HEX", value_parser = parse_enable_bytes, requires = "file", help_heading = "Capture (raw mode)")]
    enable_bytes: Option<[u8; 16]>,
}

#[derive(Args)]
struct TuneSatArgs {
    /// BS/CS110 channel as in `dth10`: BS15 or BS15_1 (BS transponder and relative TS number,
    /// default 0), ND02 (CS110 transponder), or the IF frequency in Hz (satellite RF - 10678 MHz,
    /// e.g. 1318000000 for BS15; selects relative TS 0).
    #[arg(value_parser = sat_channel)]
    channel: Channel,
    /// Select this TSID (hex, e.g. 40f1) instead of the relative TS number.
    #[arg(long, value_name = "HEX", value_parser = parse_hex16)]
    tsid: Option<u16>,
    /// How long to wait for lock and the TMCC TS-ID table [ms].
    #[arg(long, value_name = "MS", default_value_t = 1500)]
    lock_wait: u64,
    /// Skip the device start command (wValue 0 / wIndex 0x10) after the kit Init.
    #[arg(long)]
    no_start: bool,
    /// Antenna (LNB) supply: 0 (off), 110 (11 V) or 150 (15 V). The DT-H70/U2 has no such supply
    /// (the setting is for family models that have one).
    #[arg(long, value_name = "V", default_value_t = 0, value_parser = parse_lnb)]
    lnb: u32,
    /// Switch the booster on before tuning (separate from --lnb).
    #[arg(long)]
    booster: bool,
    /// Command path (bRequest bit7) of the satellite kit's I2C/GPIO: 0 = the tuner #0 path, which
    /// reaches the satellite demodulator on the DT-H70/U2; 1 = the tuner #1 path, whose I2C reaches
    /// nothing on the DT-H70/U2 and takes about 1 s per transfer (docs/40).
    #[arg(long, value_name = "0|1", default_value_t = 0, value_parser = clap::value_parser!(u8).range(0..=1))]
    i2c_tuner: u8,
}

/// A byte string given as hex digits (whitespace ignored), e.g. `00000000ff00`.
#[derive(Clone, Debug, PartialEq, Eq)]
struct HexBytes(Vec<u8>);

/// Always hexadecimal, with or without `0x` (`40f1`, `0xDEAF`).
fn parse_hex16(s: &str) -> Result<u16, String> {
    let h = s.trim_start_matches("0x").trim_start_matches("0X");
    u16::from_str_radix(h, 16).map_err(|e| format!("{s:?}: {e} (hex expected)"))
}

/// A byte as decimal, or hexadecimal with `0x`: `0x19` = 25, `19` = 19, `ff`
/// is an error. Unlike the hex options of `dth10` (and `parse_hex16` here),
/// bare digits are decimal (docs/40 relies on this).
fn parse_u8_auto(s: &str) -> Result<u8, String> {
    let n = parse_num(s).ok_or_else(|| format!("{s:?}: decimal, or hex with 0x (e.g. 0x19), expected"))?;
    u8::try_from(n).map_err(|_| format!("{s:?}: out of range 0..=255"))
}

fn parse_hex_bytes(s: &str) -> Result<HexBytes, String> {
    let s: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if !s.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err("hex digits expected".into());
    }
    if !s.len().is_multiple_of(2) {
        return Err("odd number of hex digits".into());
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|e| e.to_string()))
        .collect::<Result<Vec<u8>, String>>()
        .map(HexBytes)
}

fn parse_enable_bytes(s: &str) -> Result<[u8; 16], String> {
    let b = parse_hex_bytes(s)?.0;
    b.as_slice().try_into().map_err(|_| format!("{} bytes given, the stream-enable payload has 16", b.len()))
}

fn on_off(on: bool) -> &'static str {
    if on { "on" } else { "off" }
}

fn parse_lnb(s: &str) -> Result<u32, String> {
    match s.trim() {
        "0" => Ok(0),
        "110" => Ok(110),
        "150" => Ok(150),
        _ => Err("0, 110 or 150 expected".into()),
    }
}

/// `tune`'s channel: a UHF channel name (13..62, `27ch`) or a frequency in Hz.
fn ter_channel(s: &str) -> Result<Channel, String> {
    match Channel::parse(s) {
        Some(c) if c.system == System::IsdbT => Ok(c),
        Some(_) => Err(format!("{s}: a BS/CS channel; use tune-sat")),
        None => match s.trim().parse::<u32>() {
            Ok(hz) if hz > 1000 => Ok(Channel::terrestrial_hz(hz)),
            _ => Err(format!("unknown channel {s:?} (13..62, or a frequency in Hz)")),
        },
    }
}

/// `tune-sat`'s channel: a BS/CS110 channel name (`BS15`, `BS15_1`, `ND02`) or an IF frequency in Hz.
fn sat_channel(s: &str) -> Result<Channel, String> {
    match Channel::parse(s) {
        Some(c) if c.system == System::IsdbS => Ok(c),
        Some(_) => Err(format!("{s}: a terrestrial channel; use tune")),
        None => match s.trim().parse::<u32>() {
            Ok(hz) if hz > 1000 => Ok(Channel::satellite_hz(hz, None)),
            _ => Err(format!("unknown channel {s:?} (BSxx[_n], NDxx[_n], or an IF frequency in Hz)")),
        },
    }
}

fn hexdump(data: &[u8]) {
    for (i, chunk) in data.chunks(16).enumerate() {
        let hex: Vec<String> = chunk.iter().map(|b| format!("{b:02x}")).collect();
        let asc: String = chunk.iter().map(|&b| if (0x20..0x7f).contains(&b) { b as char } else { '.' }).collect();
        println!("{:04x}  {:<48} {}", i * 16, hex.join(" "), asc);
    }
}

fn hex(data: &[u8]) -> String {
    data.iter().map(|b| format!("{b:02x}")).collect()
}

fn init_logging(verbose: u8) {
    let level = match verbose {
        0 => "warn",
        1 => "info",
        _ => "debug",
    };
    let mut b = env_logger::Builder::new();
    b.parse_filters(level);
    if let Ok(env) = std::env::var("RUST_LOG") {
        b.parse_filters(&env);
    }
    b.format_timestamp_millis();
    let _ = b.try_init();
}

/// How the device is opened (the global options).
struct Opener {
    mode: RecipientMode,
    serial: Option<String>,
    drain_events: bool,
}

impl Opener {
    fn open(&self) -> Res<UsbTransport> {
        let t = UsbTransport::open_matching(self.mode, self.serial.as_deref())?;
        eprintln!("{}", self.header(t.product_id(), t.serial()));
        self.attach(t)
    }

    /// The line naming an opened device.
    fn header(&self, product_id: u16, serial: Option<&str>) -> String {
        format!(
            "device {} serial {} (bmRequestType recipient {:?})",
            dth10_core::product_label(product_id),
            serial.unwrap_or("-"),
            self.mode
        )
    }

    /// Start the --drain-events thread on an opened device.
    fn attach(&self, t: UsbTransport) -> Res<UsbTransport> {
        if self.drain_events
            && let Some(ep) = t.pipes().interrupt_in
        {
            let mut rd = EventReader::new(t.interrupt_in(ep)?);
            std::thread::spawn(move || {
                loop {
                    match rd.next(Duration::from_millis(500)) {
                        Ok(Some(ev)) => eprintln!(
                            "[event] targets {} class 0x{:04x} code 0x{:04x} payload {:02x?}",
                            ev.targets, ev.class, ev.code, ev.payload
                        ),
                        Ok(None) => {}
                        Err(e) => {
                            eprintln!("[event] error {e}");
                            break;
                        }
                    }
                }
            });
            eprintln!("draining interrupt endpoint 0x{ep:02x}");
        }
        Ok(t)
    }
}

fn main() {
    let cli = Cli::parse();
    init_logging(cli.verbose);
    if let Err(e) = run(cli) {
        eprintln!("dth10-diag: {e}");
        std::process::exit(1);
    }
}

fn run(cli: Cli) -> Res {
    let Cli { serial, recipient, tuner, drain_events, verbose: _, cmd } = cli;
    let mode = match recipient {
        Some(RecipientArg::Interface) => RecipientMode::Interface,
        Some(RecipientArg::Device) => RecipientMode::Device,
        None => RecipientMode::platform_default(),
    };
    let o = Opener { mode, serial, drain_events };
    // The kit commands pick their command paths themselves.
    let tuner0_only = |what: &str| -> Res {
        if tuner != 0 {
            return Err(format!("{what} does not take --tuner (it drives the kits on their own paths)").into());
        }
        Ok(())
    };
    match cmd {
        Cmd::Info { raw } => info(&o, tuner, raw),
        Cmd::GpioRead => {
            let dev = Device::new(o.open()?).with_tuner_index(tuner);
            hexdump(&dev.gpio_read()?);
            Ok(())
        }
        Cmd::Raw { value, index, out, r#in, flags } => {
            let t = o.open()?;
            let out = out.map_or_default(|h| h.0);
            let mut inbuf = vec![0u8; r#in];
            let flags = if tuner == 0 { flags } else { flags | 0x80 };
            let n = dth10_core::protocol::send(
                &t,
                flags,
                value,
                index,
                &out,
                if r#in > 0 { Some(&mut inbuf[..]) } else { None },
            )?;
            println!("ok, {n} bytes received");
            hexdump(&inbuf[..n]);
            Ok(())
        }
        Cmd::Events { seconds } => events(&o, seconds),
        Cmd::I2cRead { addr, reg, count } => {
            let mut dev = Device::new(o.open()?).with_tuner_index(tuner);
            let mut buf = vec![0u8; count as usize];
            dev.i2c_reg_read(addr, reg, &mut buf)?;
            println!(
                "i2c 0x{addr:02x} reg 0x{reg:02x}: {}",
                buf.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" ")
            );
            Ok(())
        }
        Cmd::I2cWrite { addr, reg, data } => {
            let mut dev = Device::new(o.open()?).with_tuner_index(tuner);
            dev.i2c_reg_write(addr, reg, &data.0)?;
            println!("wrote i2c 0x{addr:02x} reg 0x{reg:02x}: {:02x?}", data.0);
            Ok(())
        }
        Cmd::Tune(a) => {
            tuner0_only("tune")?;
            tune(&o, &a)
        }
        Cmd::TuneSat(a) => {
            tuner0_only("tune-sat (use --i2c-tuner)")?;
            tune_sat(&o, &a)
        }
        Cmd::Scan { first, last, wait_ms } => {
            tuner0_only("scan")?;
            scan(&o, first, last, wait_ms)
        }
        Cmd::Cepm { app_keys, flags, save } => {
            run_cepm(&o, flags.unwrap_or_else(|| cepm::path_flags(tuner)), app_keys.as_deref(), save.as_deref())
        }
        Cmd::Reset => {
            use dth10_usb::nusb::MaybeFuture;
            let t = o.open()?;
            t.device().reset().wait()?;
            println!("port reset done; the device re-enumerates (open it again)");
            Ok(())
        }
        Cmd::HubPower { action, hub, port } => hub_power(action, hub.as_deref(), port, o.serial.as_deref()),
    }
}

fn print_desc(index: u8, d: &TunerDesc) {
    println!(
        "tuner #{index}: kit {:?} model {:?} hw 0x{:x} flags 0x{:x} demod 0x{:02x} tuner 0x{:02x} IF {} Hz xtal {}/{}",
        d.kit_name,
        d.model,
        d.hw_version,
        d.flags,
        d.demod_i2c_addr,
        d.tuner_i2c_addr,
        d.if_freq_hz,
        d.demod_xtal_hz,
        d.tuner_xtal_hz
    );
}

/// `info` of the device --serial names, or of every supported device found
/// (each block starts with its `device ... serial ...` line on stdout). A
/// device that cannot be opened is reported and the others are still shown.
fn info(o: &Opener, tuner: u8, raw: bool) -> Res {
    if o.serial.is_some() {
        return info_of(o.open()?, tuner, raw);
    }
    let found = UsbTransport::list_all()?;
    if found.is_empty() {
        return Err("no supported device found".into());
    }
    let mut failed = 0;
    for (i, f) in found.iter().enumerate() {
        if i > 0 {
            println!();
        }
        println!("{}", o.header(f.product_id, f.serial.as_deref()));
        let r = f.open(o.mode).map_err(Into::into).and_then(|t| o.attach(t)).and_then(|t| info_of(t, tuner, raw));
        if let Err(e) = r {
            failed += 1;
            println!("error: {e}");
        }
    }
    if failed > 0 {
        return Err(format!("{failed} of {} devices could not be read", found.len()).into());
    }
    Ok(())
}

fn info_of(t: UsbTransport, tuner: u8, raw: bool) -> Res {
    println!("endpoints: {:?}", t.pipes());
    let dev = Device::new(t).with_tuner_index(tuner);
    let blob = dev.read_device_information_raw()?;
    println!("device information blob: {} bytes", blob.len());
    if raw {
        hexdump(&blob);
    }
    match dth10_core::tlv::parse_device_information(&blob) {
        Ok(di) => {
            println!("{:#?}", di.info);
            println!("{:#?}", di.desc);
            for (tag, data) in &di.raw_entries {
                let hex: Vec<String> = data.iter().map(|b| format!("{b:02x}")).collect();
                println!("  tag 0x{tag:02x} len {:3}: {}", data.len(), hex.join(" "));
            }
        }
        Err(e) => println!("TLV parse error: {e}"),
    }
    match dev.firmware_version() {
        Ok((a, b, c)) => println!("firmware version: {a}.{b}.{c}"),
        Err(e) => println!("firmware version read failed: {e}"),
    }
    Ok(())
}

fn events(o: &Opener, seconds: u64) -> Res {
    let t = o.open()?;
    let ep = t.pipes().interrupt_in.ok_or("no interrupt endpoint")?;
    let mut rd = EventReader::new(t.interrupt_in(ep)?);
    let start = Instant::now();
    let mut n = 0;
    while start.elapsed() < Duration::from_secs(seconds) {
        if let Some(ev) = rd.next(Duration::from_millis(500))? {
            n += 1;
            println!(
                "{:7.3}s event targets {} class 0x{:04x} code 0x{:04x} payload {:02x?}",
                start.elapsed().as_secs_f64(),
                ev.targets,
                ev.class,
                ev.code,
                ev.payload
            );
        }
    }
    println!("{n} events");
    Ok(())
}

/// One reading of the signal status: lock bits, the raw sync register (ISDB-T:
/// `0xB0 & 0x0F`, ISDB-S: register 0xC3), C/N, quality, strength and the BER
/// of each layer (`-` = not measured, `unused` = layer not in use).
fn status_line(st: &SignalStatus, sat: bool) -> String {
    let bits: Vec<&str> = [(lock::SYNC_STARTED, "sync"), (lock::SIGNAL, "signal"), (lock::LOCKED, "LOCKED")]
        .iter()
        .filter(|(b, _)| st.lock & b != 0)
        .map(|(_, n)| *n)
        .collect();
    let ber: Vec<String> = [("A", field::BER_A), ("B", field::BER_B), ("C", field::BER_C)]
        .iter()
        .zip(st.ber)
        .map(|(&(layer, f), v)| match v {
            _ if st.mask & f == 0 => format!("{layer} -"),
            BER_UNUSED => format!("{layer} unused"),
            0 => format!("{layer} 0"),
            v => format!("{layer} {:.1e}", v as f64 * 1e-9),
        })
        .collect();
    let sync = if sat { format!("reg C3 0x{:02x}", st.sync_state) } else { format!("sync {:2}", st.sync_state) };
    format!(
        "lock 0x{:03x} [{}] {sync} C/N {:.2} dB quality {} strength {} BER {}",
        st.lock,
        bits.join(" "),
        st.cnr_mdb as f64 / 1000.0,
        st.quality,
        st.strength,
        ber.join(" ")
    )
}

/// Read the signal status every 250 ms, printing each reading, until `done`
/// holds or `wait` has passed. Returns the last good reading and whether
/// `done` was reached.
fn wait_lock(
    wait: Duration,
    sat: bool,
    mut status: impl FnMut() -> dth10_core::Result<SignalStatus>,
    done: impl Fn(&SignalStatus) -> bool,
) -> (Option<SignalStatus>, bool) {
    let t0 = Instant::now();
    let mut last = None;
    loop {
        std::thread::sleep(Duration::from_millis(250));
        match status() {
            Ok(st) => {
                println!("{:5.2} s  {}", t0.elapsed().as_secs_f64(), status_line(&st, sat));
                let ok = done(&st);
                last = Some(st);
                if ok {
                    return (last, true);
                }
            }
            Err(e) => println!("{:5.2} s  status error: {e}", t0.elapsed().as_secs_f64()),
        }
        if t0.elapsed() >= wait {
            return (last, false);
        }
    }
}

fn tune(o: &Opener, a: &TuneArgs) -> Res {
    let t = o.open()?;
    let (ep, _) = t.pipes().ts0.ok_or("no TS endpoint 0x81 found")?;
    let pid = t.product_id();
    let dev = Device::new(t);
    let di = dev.read_device_information()?;
    print_desc(0, &di.desc);
    let mut kit = TerrestrialKit::new(dev, di.desc, pid).map_err(|e| unverified_kit_error(pid, 0, e))?;
    kit.init()?;
    if !a.no_start {
        kit.dev().start()?; // wValue 0 / wIndex 0x10, after the kit Init (the original driver's order)
    }
    kit.open()?;
    let r = ter_probe(&mut kit, ep, a);
    if let Err(e) = kit.close() {
        log::warn!("terrestrial kit close: {e}");
    }
    r
}

fn ter_probe(kit: &mut TerrestrialKit<UsbTransport>, ep: u8, a: &TuneArgs) -> Res {
    let ch = &a.channel;
    if kit.has_attenuator() {
        let applied = kit.set_attenuator(a.attenuator)?;
        println!("attenuator {}: applied={applied} readback {:?}", on_off(a.attenuator), kit.attenuator()?);
    } else if a.attenuator {
        println!("attenuator: the {} kit has none", kit.kind());
    }
    kit.tune(&TuneRequest::isdb_t(ch.frequency_hz as u64, 6_000_000))?;
    let tuned = Instant::now();
    println!("tuned to {} ({} Hz)", ch.name, ch.frequency_hz);
    let (_, locked) = wait_lock(
        Duration::from_millis(a.lock_wait),
        false,
        || kit.signal_status(field::ALL),
        |st| st.lock & lock::LOCKED != 0,
    );
    if !locked {
        let skipped = if a.file.is_some() { "; capture skipped" } else { "" };
        return Err(format!("ISDB-T: no lock within {} ms{skipped}", a.lock_wait).into());
    }
    println!("ISDB-T: locked");
    match a.file.as_deref() {
        Some(file) => capture(kit, ep, a, file, tuned),
        None => Ok(()),
    }
}

/// `tune --file`: enable stream #0, read it for `--seconds` and write it to `file`.
fn capture(kit: &mut TerrestrialKit<UsbTransport>, ep: u8, a: &TuneArgs, file: &Path, tuned: Instant) -> Res {
    let byte0 = match &a.enable_bytes {
        Some(b) => b[0],
        None => StreamMode::RawFullTs as u8,
    };
    if byte0 != StreamMode::EncryptedPartial as u8 {
        eprintln!(
            "warning: enabling stream #0 with byte0 = {byte0} (raw mode is byte0 = 1), not the device's own mode 0."
        );
        eprintln!(
            "warning: raw mode on stream #0 stops the satellite stream until the device is power-cycled (docs/40): BS/CS will not work before that."
        );
    }
    // docs/40: the bridge sends TEI+null filler for about 1.5 s after tuning;
    // enabling after that keeps it out of the capture.
    if let Some(rest) = Duration::from_millis(1500).checked_sub(tuned.elapsed()) {
        std::thread::sleep(rest);
    }
    let mut reader = StreamReader::new(
        kit.dev().transport().bulk_in(ep)?,
        StreamReader::DEFAULT_TRANSFER_LEN,
        StreamReader::DEFAULT_DEPTH,
    )?;
    let mut f = std::fs::File::create(file)?;
    reader.start(); // bulk reads first, then the enable command (docs/21)
    match &a.enable_bytes {
        Some(b) => {
            kit.dev().stream_enable_payload(b)?;
            println!("stream enable with custom payload {b:02x?}");
        }
        None => {
            let desc = kit.desc().clone();
            kit.dev().stream_enable_mode(&desc, StreamMode::RawFullTs)?
        }
    }
    let layout = SlotLayout::for_product(kit.dev().transport().product_id());
    let r = capture_loop(&mut reader, &mut f, a.seconds, a.blocks, layout);
    reader.stop();
    let disabled = kit.dev().stream_disable();
    let (total, transfers) = r?;
    disabled?;
    println!("\n{transfers} transfers, captured {total} bytes to {}", file.display());
    Ok(())
}

fn capture_loop(
    reader: &mut StreamReader,
    out: &mut impl Write,
    seconds: u64,
    blocks: bool,
    layout: SlotLayout,
) -> Res<(u64, u64)> {
    let start = Instant::now();
    let (mut total, mut transfers) = (0u64, 0u64);
    let mut sec = PacketStats::default();
    let mut last_report = Instant::now();
    while start.elapsed() < Duration::from_secs(seconds) {
        match reader.next(Duration::from_millis(500)) {
            Ok(Some(data)) => {
                transfers += 1;
                if blocks {
                    out.write_all(&data)?;
                    total += data.len() as u64;
                }
                // 256-byte blocks with one TS packet each, placed as the model's layout says.
                let mut scratch = [0u8; ts_slot::TS_PACKET];
                for blk in data.as_chunks::<{ ts_slot::SLOT }>().0 {
                    let pkt = layout.packet(blk, &mut scratch);
                    if !blocks {
                        out.write_all(pkt)?;
                        total += pkt.len() as u64;
                    }
                    sec.note(pkt);
                }
            }
            Ok(None) => eprint!("."),
            Err(e) => {
                eprintln!("[t+{:5.1}s] bulk read error: {e}; clearing halt", start.elapsed().as_secs_f64());
                match reader.clear_halt() {
                    Ok(()) => eprintln!("[t+{:5.1}s] halt cleared", start.elapsed().as_secs_f64()),
                    Err(e2) => eprintln!("[t+{:5.1}s] clear_halt failed: {e2}", start.elapsed().as_secs_f64()),
                }
                std::thread::sleep(Duration::from_millis(50));
            }
        }
        if last_report.elapsed() >= Duration::from_secs(1) {
            eprintln!(
                "[t+{:5.1}s] {} pkt/s, pusi {} pes-start {}, total {} B",
                start.elapsed().as_secs_f64(),
                sec.pkts,
                sec.pusi,
                sec.pes,
                total
            );
            last_report = Instant::now();
            sec = PacketStats::default();
        }
    }
    Ok((total, transfers))
}

/// Per-second stream statistics.
#[derive(Default, Debug, PartialEq, Eq)]
struct PacketStats {
    pkts: u64,
    /// Packets with payload_unit_start_indicator.
    pusi: u64,
    /// Of those, payloads beginning with a PES start code (00 00 01).
    pes: u64,
}

impl PacketStats {
    fn note(&mut self, p: &[u8]) {
        self.pkts += 1;
        if p.len() < 188 || p[1] & 0x40 == 0 {
            return;
        }
        self.pusi += 1;
        let mut off = 4;
        if p[3] & 0x20 != 0 {
            off += 1 + p[4] as usize;
        }
        if off + 3 <= 188 && p[off] == 0 && p[off + 1] == 0 && p[off + 2] == 1 {
            self.pes += 1;
        }
    }
}

/// The TSID to select: the TMCC table entry of the channel's relative TS
/// number when the table has it, else the channel's own TSID (the rule of
/// `dth10`). 0 = nothing to select.
fn pick_tsid(ch: &Channel, ids: &[u16]) -> u16 {
    if let Some(slot) = ch.slot
        && let Some(&id) = ids.get(slot as usize)
        && id != 0
        && id != 0xFFFF
    {
        return id;
    }
    ch.tsid
}

fn tune_sat(o: &Opener, a: &TuneSatArgs) -> Res {
    let ch = match a.tsid {
        Some(t) => a.channel.clone().with_tsid(t),
        None => a.channel.clone(),
    };
    let t = o.open()?;
    let dev0 = Device::new(t.clone());
    let info0 = dev0.read_device_information()?;
    print_desc(0, &info0.desc);
    if info0.desc.flags & 1 == 0 {
        eprintln!("warning: tuner #0 descriptor flags bit0 is clear (no second tuner advertised)");
    }
    let info1 = Device::new(t.clone()).with_tuner_index(1).read_device_information()?;
    print_desc(1, &info1.desc);
    // The original driver and `dth10` initialise both kits, then send the
    // device start; the terrestrial kit stays in standby here.
    match TerrestrialKit::new(dev0, info0.desc.clone(), t.product_id()) {
        Ok(mut ter) => {
            if let Err(e) = ter.init() {
                eprintln!("warning: terrestrial kit Init: {e}");
            }
        }
        Err(e) => eprintln!("warning: terrestrial kit: {}", unverified_kit_error(t.product_id(), 0, e)),
    }
    let mut kit =
        SatelliteKit::new(Device::new(t.clone()).with_tuner_index(a.i2c_tuner), info1.desc.clone(), t.product_id())
            .map_err(|e| unverified_kit_error(t.product_id(), 1, e))?;
    println!("satellite kit ({}): I2C/GPIO via the tuner #{} path", kit.kind(), a.i2c_tuner);
    kit.init()?;
    if !a.no_start {
        Device::new(t).start()?; // wValue 0 / wIndex 0x10 on the tuner #0 path, as `dth10` sends it
    }
    kit.open()?;
    let r = sat_probe(&mut kit, &ch, a);
    if a.lnb != 0
        && let Err(e) = kit.lnb_power(0)
    {
        log::warn!("LNB off: {e}");
    }
    if let Err(e) = kit.close() {
        log::warn!("satellite kit close: {e}");
    }
    r
}

fn sat_probe(kit: &mut SatelliteKit<UsbTransport>, ch: &Channel, a: &TuneSatArgs) -> Res {
    if a.lnb != 0 {
        let healthy = kit.lnb_power(a.lnb)?;
        let st = kit.dev_ref().gpio_read()?;
        println!(
            "LNB {}: healthy={healthy} gpio status {:02x?} readback {:?}",
            a.lnb,
            &st[8..13],
            dth10_core::device::LnbGpio::of(kit.desc()).map(|g| dth10_core::device::lnb_get(kit.dev_ref(), g))
        );
    }
    let applied = kit.set_booster(a.booster)?;
    println!("booster {}: applied={applied} readback {:?}", on_off(a.booster), kit.booster()?);
    kit.tune(ch.frequency_hz as u64)?;
    println!("tuned to {} (IF {} Hz)", ch.name, ch.frequency_hz);
    let (last, _) = wait_lock(
        Duration::from_millis(a.lock_wait),
        true,
        || kit.signal_status(field::ALL | field::TSID_TABLE),
        |st| st.lock & lock::LOCKED != 0 && st.tsid_count > 0,
    );
    let st = match last {
        Some(st) if st.lock & lock::LOCKED != 0 => st,
        _ => return Err(format!("ISDB-S: no lock within {} ms (no signal, or no antenna)", a.lock_wait).into()),
    };
    let ids = &st.tsids[..(st.tsid_count as usize).min(st.tsids.len())];
    let table: Vec<String> = ids.iter().map(|t| format!("{t:04x}")).collect();
    println!("ISDB-S: locked, TMCC TS-ID table [{}]", table.join(" "));
    let tsid = pick_tsid(ch, ids);
    if tsid == 0 {
        return Err("no TSID to select (empty TMCC table; give --tsid)".into());
    }
    if !ids.contains(&tsid) {
        eprintln!("warning: TSID 0x{tsid:04x} is not in the TMCC table");
    }
    kit.set_tsid(tsid)?;
    match ch.slot {
        Some(slot) => println!("TSID 0x{tsid:04x} selected (relative TS {slot})"),
        None => println!("TSID 0x{tsid:04x} selected"),
    }
    std::thread::sleep(Duration::from_millis(250));
    match kit.signal_status(field::ALL | field::TSID_TABLE) {
        Ok(st) => println!("after the TS selection: {}", status_line(&st, true)),
        Err(e) => println!("status error: {e}"),
    }
    Ok(())
}

fn scan(o: &Opener, first: u32, last: u32, wait_ms: u64) -> Res {
    if first > last {
        return Err(format!("--first {first} is above --last {last}").into());
    }
    let t = o.open()?;
    let pid = t.product_id();
    let dev = Device::new(t);
    let di = dev.read_device_information()?;
    print_desc(0, &di.desc);
    let mut kit = TerrestrialKit::new(dev, di.desc, pid).map_err(|e| unverified_kit_error(pid, 0, e))?;
    kit.init()?;
    kit.dev().start()?;
    kit.open()?;
    for ch in first..=last {
        let freq = uhf_center_hz(ch).ok_or("UHF channel out of range")?; // 13..62, checked by the option parser
        if let Err(e) = kit.tune(&TuneRequest::isdb_t(freq as u64, 6_000_000)) {
            println!("ch {ch}: tune error {e}");
            continue;
        }
        std::thread::sleep(Duration::from_millis(wait_ms));
        match kit.signal_status(field::ALL_BASIC) {
            Ok(st) => println!(
                "ch {ch:2} {freq} Hz: lock 0x{:03x} sync {:2} cnr {:6.2} dB quality {:3} strength {}",
                st.lock,
                st.sync_state,
                st.cnr_mdb as f64 / 1000.0,
                st.quality,
                st.strength
            ),
            Err(e) => println!("ch {ch}: status error {e}"),
        }
    }
    kit.close()?;
    Ok(())
}

/// The application credentials: `--app-keys`, `$DTH10_APP_KEYS`, `./CoBDACom.dll`.
fn load_app_keys(given: Option<&Path>) -> Res<cepm::AppKeys> {
    let path = match given {
        Some(p) => p.to_path_buf(),
        None => std::env::var_os("DTH10_APP_KEYS").map(PathBuf::from).unwrap_or_else(|| "CoBDACom.dll".into()),
    };
    let keys = cepm::AppKeys::load(&path)?;
    eprintln!("application credentials read from {} ({} IV candidates)", path.display(), keys.iv_candidates());
    Ok(keys)
}

fn run_cepm(o: &Opener, flags: u8, app_keys: Option<&Path>, save: Option<&Path>) -> Res {
    let keys = load_app_keys(app_keys)?;
    let t = o.open()?;
    let hs = cepm::Handshake::run_for_path(&t, flags, &keys)?;
    if let Some(p) = save {
        let mut set = cepm::load_set(p);
        set.retain(|(f, _)| *f != flags);
        set.push((flags, hs.clone()));
        cepm::save_set(p, &set)?;
        println!("saved to {} (path flags 0x{flags:02x}; {} record(s) in the file)", p.display(), set.len());
    }
    println!("device certificate:");
    hexdump(&hs.dev_cert);
    println!("device nonce      {}", hex(&hs.dev_nonce));
    println!("device ephemeral  {}", hex(&hs.dev_ephemeral));
    println!("shared secret     {}", hex(&hs.shared));
    println!("step 5 response:");
    hexdump(&hs.m5);
    println!("session id        {}", hex(&hs.session_id));
    Ok(())
}

/// The hub port the tuner hangs off: `status`, `off`, `on` or `cycle` through the
/// hub class requests (GetPortStatus, Clear/SetPortFeature PORT_POWER). Linux only
/// (the bus number and port chain of a device are read from sysfs).
#[cfg(any(target_os = "linux", target_os = "android"))]
fn hub_power(action: HubAction, hub: Option<&str>, port: Option<u8>, serial: Option<&str>) -> Res {
    use dth10_usb::nusb::MaybeFuture;
    use dth10_usb::nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};
    const PORT_POWER: u16 = 8;
    let all: Vec<dth10_usb::nusb::DeviceInfo> = dth10_usb::nusb::list_devices().wait()?.collect();
    let (hub_info, port) = if let Some(spec) = hub {
        let (b, a) = spec.split_once(':').ok_or("--hub takes BUS:ADDR")?;
        let (b, a): (u8, u8) = (b.parse()?, a.parse()?);
        let h = all.iter().find(|d| d.busnum() == b && d.device_address() == a).ok_or("no such hub")?;
        (h.clone(), port.ok_or("--port is needed with --hub")?)
    } else {
        let tuner = all
            .iter()
            .find(|d| {
                d.vendor_id() == dth10_core::VID_BUFFALO
                    && dth10_core::product_name(d.product_id()).is_some()
                    && serial.is_none_or(|s| d.serial_number().is_some_and(|n| n.eq_ignore_ascii_case(s)))
            })
            .ok_or("no supported tuner found (VID 0411)")?;
        let chain = tuner.port_chain();
        let (&last, parent) = chain.split_last().ok_or("the tuner is a root device")?;
        let h = all
            .iter()
            .find(|d| d.busnum() == tuner.busnum() && d.port_chain() == parent && d.class() == 9)
            .ok_or("the tuner's parent hub was not found")?;
        (h.clone(), port.unwrap_or(last))
    };
    println!(
        "hub {:04x}:{:04x} bus {} addr {} {:?}, port {port}",
        hub_info.vendor_id(),
        hub_info.product_id(),
        hub_info.busnum(),
        hub_info.device_address(),
        hub_info.product_string()
    );
    let dev = hub_info.open().wait()?;
    let timeout = Duration::from_secs(2);
    // Hub descriptor: wHubCharacteristics bits 0-1 = power switching (0 ganged, 1 per port, 2/3 none).
    let desc = dev
        .control_in(
            ControlIn {
                control_type: ControlType::Class,
                recipient: Recipient::Device,
                request: 6,
                value: 0x2900,
                index: 0,
                length: 9,
            },
            timeout,
        )
        .wait()?;
    if desc.len() >= 5 {
        let ch = u16::from_le_bytes([desc[3], desc[4]]);
        println!(
            "hub: {} ports, power switching: {}",
            desc[2],
            match ch & 3 {
                0 => "ganged",
                1 => "per port",
                _ => "none (power always on)",
            }
        );
    }
    let status = |what: &str| -> Res {
        let st = dev
            .control_in(
                ControlIn {
                    control_type: ControlType::Class,
                    recipient: Recipient::Other,
                    request: 0,
                    value: 0,
                    index: port as u16,
                    length: 4,
                },
                timeout,
            )
            .wait()?;
        if st.len() >= 4 {
            let ps = u16::from_le_bytes([st[0], st[1]]);
            println!(
                "port {port} {what}: status 0x{ps:04x} (connected {}, enabled {}, powered {})",
                ps & 1 != 0,
                ps & 2 != 0,
                ps & 0x100 != 0
            );
        }
        Ok(())
    };
    let feature = |set: bool| -> Res {
        dev.control_out(
            ControlOut {
                control_type: ControlType::Class,
                recipient: Recipient::Other,
                request: if set { 3 } else { 1 },
                value: PORT_POWER,
                index: port as u16,
                data: &[],
            },
            timeout,
        )
        .wait()?;
        Ok(())
    };
    status("before")?;
    match action {
        HubAction::Status => {}
        HubAction::Off => feature(false)?,
        HubAction::On => feature(true)?,
        HubAction::Cycle => {
            feature(false)?;
            std::thread::sleep(Duration::from_secs(2));
            status("while off")?;
            feature(true)?;
        }
    }
    if action != HubAction::Status {
        std::thread::sleep(Duration::from_millis(500));
        status("after")?;
    }
    Ok(())
}

#[cfg(not(any(target_os = "linux", target_os = "android")))]
fn hub_power(_action: HubAction, _hub: Option<&str>, _port: Option<u8>, _serial: Option<&str>) -> Res {
    Err("hub-power is available on Linux only".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;

    fn parse(v: &[&str]) -> Result<Cli, clap::Error> {
        Cli::try_parse_from(std::iter::once("dth10-diag").chain(v.iter().copied()))
    }

    #[test]
    fn cli_definition_is_consistent() {
        Cli::command().debug_assert();
    }

    #[test]
    fn number_parsers() {
        // Bare digits are decimal here (docs/40), hex needs 0x.
        assert_eq!(parse_u8_auto("25"), Ok(25));
        assert_eq!(parse_u8_auto("0x19"), Ok(0x19));
        assert_eq!(parse_u8_auto("0X80"), Ok(0x80));
        assert!(parse_u8_auto("256").is_err());
        assert!(parse_u8_auto("ff").is_err());
        assert_eq!(parse_hex16("40f1"), Ok(0x40F1));
        assert_eq!(parse_hex16("0xDEAF"), Ok(0xDEAF));
        assert_eq!(parse_hex_bytes("00 ff 12"), Ok(HexBytes(vec![0, 0xFF, 0x12])));
        assert!(parse_hex_bytes("abc").is_err());
        assert!(parse_hex_bytes("zz").is_err());
        assert!(parse_hex_bytes("éé").is_err());
        assert_eq!(parse_lnb("150"), Ok(150));
        assert!(parse_lnb("15").is_err());
    }

    #[test]
    fn channel_arguments() {
        let c = ter_channel("27").unwrap();
        assert_eq!((c.system, c.frequency_hz), (System::IsdbT, 557_142_857));
        assert_eq!(ter_channel("27ch").unwrap().frequency_hz, 557_142_857);
        assert_eq!(ter_channel("557142857").unwrap().frequency_hz, 557_142_857);
        assert!(ter_channel("12").is_err());
        assert!(ter_channel("BS15").is_err());
        let s = sat_channel("BS15_1").unwrap();
        assert_eq!((s.system, s.frequency_hz, s.slot), (System::IsdbS, 1_318_000_000, Some(1)));
        assert_eq!(sat_channel("ND02").unwrap().tsid, 0x6020);
        let f = sat_channel("1318000000").unwrap();
        assert_eq!((f.system, f.slot), (System::IsdbS, Some(0)));
        assert!(sat_channel("27").is_err());
        assert!(sat_channel("BS16").is_err());
    }

    #[test]
    fn capture_options_need_a_file() {
        assert!(parse(&["tune", "27"]).is_ok());
        assert!(parse(&["tune", "27", "--seconds", "5"]).is_err());
        assert!(parse(&["tune", "27", "--blocks"]).is_err());
        let payload = "0101000000000000 0000000000000000";
        assert!(parse(&["tune", "27", "--enable-bytes", payload]).is_err());
        let Cmd::Tune(a) =
            parse(&["tune", "27ch", "--file", "x.ts", "--seconds", "5", "--blocks", "--enable-bytes", payload])
                .unwrap()
                .cmd
        else {
            panic!("not tune")
        };
        let mut want = [0u8; 16];
        want[..2].copy_from_slice(&[1, 1]);
        assert_eq!((a.seconds, a.blocks, a.enable_bytes), (5, true, Some(want)));
        assert!(parse(&["tune", "27", "--file", "x.ts", "--enable-bytes", "0101"]).is_err());
        // The options removed from tune are gone for good.
        for gone in ["--decrypt", "--cepm-state=x", "--rehandshake-at=1", "--pids=1:100", "--raw"] {
            assert!(parse(&["tune", "27", "--file", "x.ts", gone]).is_err(), "{gone}");
        }
    }

    #[test]
    fn tune_sat_defaults_and_options() {
        let Cmd::TuneSat(a) = parse(&["tune-sat", "BS15"]).unwrap().cmd else { panic!("not tune-sat") };
        assert_eq!((a.i2c_tuner, a.lnb, a.tsid, a.lock_wait), (0, 0, None, 1500));
        let Cmd::TuneSat(a) = parse(&["tune-sat", "1613000000", "--tsid", "6020", "--i2c-tuner", "1"]).unwrap().cmd
        else {
            panic!("not tune-sat")
        };
        assert_eq!((a.tsid, a.i2c_tuner), (Some(0x6020), 1));
        for gone in ["--file=x.ts", "--seconds=5", "--stream-tuner=1", "--with-ter"] {
            assert!(parse(&["tune-sat", "BS15", gone]).is_err(), "{gone}");
        }
        assert!(parse(&["tune-sat", "BS15", "--lnb", "12"]).is_err());
        assert!(parse(&["tune-sat", "BS15", "--i2c-tuner", "2"]).is_err());
    }

    #[test]
    fn scan_range_is_validated() {
        assert!(parse(&["scan"]).is_ok());
        assert!(parse(&["scan", "--first", "1"]).is_err());
        assert!(parse(&["scan", "--last", "63"]).is_err());
        let Cmd::Scan { first, last, .. } = parse(&["scan", "--first", "20", "--last", "30"]).unwrap().cmd else {
            panic!("not scan")
        };
        assert_eq!((first, last), (20, 30));
    }

    #[test]
    fn global_options() {
        let cli = parse(&["--serial", "04110199-1108 22N0326", "-v", "info", "--raw"]).unwrap();
        assert_eq!((cli.serial.as_deref(), cli.verbose), (Some("04110199-1108 22N0326"), 1));
        let cli = parse(&["gpio-read", "--serial", "X", "-vv", "--tuner", "1"]).unwrap();
        assert_eq!((cli.serial.as_deref(), cli.verbose, cli.tuner), (Some("X"), 2, 1));
        assert!(parse(&["--tuner", "2", "info"]).is_err());
        let gone: [&[&str]; 4] =
            [&["list"], &["capture", "x.ts"], &["cepm-resume", "00", "00"], &["--app-keys", "k.dll", "cepm"]];
        for gone in gone {
            assert!(parse(gone).is_err(), "{gone:?}");
        }
        let Cmd::Cepm { flags, app_keys, .. } = parse(&["cepm", "--flags", "0x80", "--app-keys", "k.dll"]).unwrap().cmd
        else {
            panic!("not cepm")
        };
        assert_eq!((flags, app_keys), (Some(0x80), Some(PathBuf::from("k.dll"))));
        assert!(matches!(parse(&["hub-power", "cycle"]).unwrap().cmd, Cmd::HubPower { action: HubAction::Cycle, .. }));
        assert!(parse(&["hub-power", "toggle"]).is_err());
    }

    #[test]
    fn tsid_choice() {
        let ch = Channel::parse("BS15_1").unwrap();
        assert_eq!(pick_tsid(&ch, &[0x40F0, 0x40F2]), 0x40F2); // the table entry of relative TS 1
        assert_eq!(pick_tsid(&ch, &[0x40F0]), 0x40F1); // not in the table: the channel's own TSID
        assert_eq!(pick_tsid(&ch.clone().with_tsid(0x40F5), &[0x40F0, 0x40F2]), 0x40F5);
        assert_eq!(pick_tsid(&Channel::satellite_hz(1_318_000_000, None), &[0x40F0]), 0x40F0);
        assert_eq!(pick_tsid(&Channel::satellite_hz(1_318_000_000, None), &[]), 0);
    }

    #[test]
    fn status_text() {
        let st = SignalStatus {
            mask: field::ALL,
            lock: 0x013,
            cnr_mdb: 34_120,
            quality: 100,
            strength: 5,
            ber: [0, 1234, BER_UNUSED],
            sync_state: 9,
            ..Default::default()
        };
        let s = status_line(&st, false);
        assert!(s.starts_with("lock 0x013 [sync signal LOCKED] sync  9 C/N 34.12 dB"), "{s}");
        assert!(s.ends_with("BER A 0 B 1.2e-6 C unused"), "{s}");
        let st = SignalStatus { mask: field::ALL_BASIC, sync_state: 0x10, ..Default::default() };
        let s = status_line(&st, true);
        assert!(s.contains("[] reg C3 0x10"), "{s}");
        assert!(s.ends_with("BER A - B - C -"), "{s}");
    }

    #[test]
    fn packet_statistics() {
        let mut s = PacketStats::default();
        let mut p = [0u8; 188];
        p[0] = 0x47;
        s.note(&p); // no PUSI
        p[1] = 0x40;
        p[3] = 0x10;
        p[4..7].copy_from_slice(&[0, 0, 1]);
        s.note(&p); // PUSI + PES start
        p[3] = 0x30; // adaptation field of 1 + 2 bytes, then the PES start code
        p[4] = 2;
        p[7..10].copy_from_slice(&[0, 0, 1]);
        s.note(&p);
        p[7] = 0xFF;
        s.note(&p); // PUSI without a PES start code
        assert_eq!(s, PacketStats { pkts: 4, pusi: 3, pes: 2 });
    }
}
