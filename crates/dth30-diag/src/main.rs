//! Hardware diagnostics for the BUFFALO DT-H30/U2 family (DT-H30/U2,
//! DT-H50/U2, DT-H50/U2W): a ViXS XCode2 chip behind a PLX NET2280.
//!
//! A developer tool, checked on a DT-H50/U2 only. Every command that touches the
//! device opens it afresh: `probe` only resets the NET2280 side and reads
//! the chip's identity (no firmware), the others first bring the chip up
//! with the firmware from the user's own vendor driver file.
//!
//! On Windows the device must be bound to WinUSB.

use std::error::Error;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use clap::{Args, Parser, Subcommand};
use dth30_core::bus::RemotePci;
use dth30_core::device::{Config, Dth30, StreamInfo};
use dth30_core::frontend::StdSleep;
use dth30_core::frontend::regs::{read_regs, write_regs};
use dth30_core::model;
use dth30_core::psi::ProgramScanner;
use dth30_core::vixs::stream::{CasMode, card_state, parse_generic_block};
use dth30_core::vixs::{Firmware, Options, Vixs};
use dth30_usb::{LocalSpace, Net2280Usb, StreamReader};

type Res<T = ()> = Result<T, Box<dyn Error>>;

/// Environment variable naming the vendor driver file (or an extracted image).
const FW_ENV: &str = "DTH30_FIRMWARE";

#[derive(Parser)]
#[command(
    name = "dth30-diag",
    version,
    about = "Hardware diagnostics for the BUFFALO DT-H30/U2 family of USB TV tuners (developer tool, checked on a DT-H50/U2 only)"
)]
struct Cli {
    /// Select the device by USB serial number (default: the first one found).
    #[arg(long, global = true, help_heading = "Global options", value_name = "SERIAL")]
    serial: Option<String>,
    /// -v = info, -vv = debug, -vvv = trace (default: warn; RUST_LOG overrides). Logs go to stderr.
    #[arg(short, long, global = true, help_heading = "Global options", action = clap::ArgAction::Count)]
    verbose: u8,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Args, Clone)]
struct FwArgs {
    /// The vendor driver BufFsUSB.sys (or an image extracted with `fw --out`).
    /// Default: $DTH30_FIRMWARE.
    #[arg(long, value_name = "PATH")]
    firmware: Option<PathBuf>,
}

#[derive(Subcommand)]
enum Cmd {
    /// List the connected devices of the family.
    List,
    /// Show the USB endpoints and a few NET2280 registers (changes nothing on the device).
    Info,
    /// Initialise the NET2280 side, assign the ViXS BARs and read the chip's identity
    /// and board ID (no firmware; the state is gone after a power cycle).
    Probe,
    /// Check a firmware source offline: extract the MIPS image from BufFsUSB.sys (or load an
    /// extracted image), verify its hash and show its vector table.
    Fw {
        /// BufFsUSB.sys or an extracted image (default: $DTH30_FIRMWARE).
        path: Option<PathBuf>,
        /// Write the extracted image to this file (for later use with --firmware).
        #[arg(long, value_name = "FILE")]
        out: Option<PathBuf>,
    },
    /// Bring the chip up (memory init, firmware, MIPS start) and report the result.
    BringUp(FwArgs),
    /// After bring-up, read I2C registers: 8-bit address, start register, count
    /// (decimal, or hex with 0x).
    I2cRead {
        #[command(flatten)]
        fw: FwArgs,
        #[arg(value_parser = parse_u8_auto)]
        addr: u8,
        #[arg(value_parser = parse_u8_auto)]
        reg: u8,
        #[arg(default_value = "1", value_parser = parse_u8_auto)]
        count: u8,
        /// I2C channel (0 or 1).
        #[arg(long, default_value_t = 0)]
        channel: u8,
    },
    /// After bring-up, write I2C registers: 8-bit address, register, data bytes (hex digits).
    I2cWrite {
        #[command(flatten)]
        fw: FwArgs,
        #[arg(value_parser = parse_u8_auto)]
        addr: u8,
        #[arg(value_parser = parse_u8_auto)]
        reg: u8,
        #[arg(value_parser = parse_hex_bytes)]
        data: HexBytes,
        #[arg(long, default_value_t = 0)]
        channel: u8,
    },
    /// After bring-up, initialise the board's front end, tune and print the signal status
    /// every 250 ms until it locks or --lock-wait passes.
    Tune(TuneArgs),
    /// After tune and lock, open the CAS object and the Raw stream, find the service in its
    /// PAT/PMT, open the Program stream and write it to a file (with --raw, the Raw stream).
    /// Reports how many packets are still scrambled.
    Record {
        #[command(flatten)]
        tune: TuneArgs,
        /// Output file.
        #[arg(short, long, value_name = "FILE")]
        output: PathBuf,
        /// Seconds to record.
        #[arg(long, default_value_t = 10)]
        seconds: u64,
        /// Service ID (default: the first service in the PAT).
        #[arg(long)]
        sid: Option<u16>,
        /// Record the Raw stream (EP 0x83, the full TS as the chip passes it) instead.
        #[arg(long)]
        raw: bool,
        /// CAS set-up: "plain" (B-CAS, the TS comes out in the clear) or "drm" (the vendor's; the
        /// firmware encrypts the TS again for the vendor's viewer).
        #[arg(long, default_value = "plain", value_parser = parse_cas_mode)]
        cas: CasMode,
    },
}

#[derive(Args, Clone)]
struct TuneArgs {
    #[command(flatten)]
    fw: FwArgs,
    /// UHF channel (13-62), VHF 1-12, CATV "C13".."C63", or a frequency in Hz.
    channel: String,
    #[arg(long, default_value_t = 3000, value_name = "MS")]
    lock_wait: u32,
    /// Subsystem IDs for board 0xB2 (MJ102 when EEEE:EEEE), as "VVVV:DDDD" in hex.
    #[arg(long, value_name = "VVVV:DDDD", value_parser = parse_subsystem)]
    subsystem: Option<(u16, u16)>,
}

fn parse_cas_mode(s: &str) -> Result<CasMode, String> {
    match s {
        "drm" => Ok(CasMode::BcasDrm),
        "plain" => Ok(CasMode::Bcas),
        _ => Err("expected drm or plain".into()),
    }
}

#[derive(Clone)]
struct HexBytes(Vec<u8>);

fn parse_u8_auto(s: &str) -> Result<u8, String> {
    match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Some(h) => u8::from_str_radix(h, 16),
        None => s.parse(),
    }
    .map_err(|e| e.to_string())
}

fn parse_hex_bytes(s: &str) -> Result<HexBytes, String> {
    let s: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if s.is_empty() || !s.len().is_multiple_of(2) {
        return Err("expected an even number of hex digits".into());
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|e| e.to_string()))
        .collect::<Result<_, _>>()
        .map(HexBytes)
}

fn parse_subsystem(s: &str) -> Result<(u16, u16), String> {
    let (a, b) = s.split_once(':').ok_or("expected VVVV:DDDD")?;
    let p = |x: &str| u16::from_str_radix(x, 16).map_err(|e| e.to_string());
    Ok((p(a)?, p(b)?))
}

fn init_logging(verbose: u8) {
    let level = match verbose {
        0 => "warn",
        1 => "info",
        2 => "debug",
        _ => "trace",
    };
    let mut b = env_logger::Builder::new();
    b.parse_filters(level);
    if let Ok(env) = std::env::var("RUST_LOG") {
        b.parse_filters(&env);
    }
    b.format_timestamp_millis();
    let _ = b.try_init();
}

fn main() {
    let cli = Cli::parse();
    init_logging(cli.verbose);
    if let Err(e) = run(cli) {
        eprintln!("dth30-diag: {e}");
        std::process::exit(1);
    }
}

fn run(cli: Cli) -> Res {
    let serial = cli.serial.as_deref();
    match cli.cmd {
        Cmd::List => list(),
        Cmd::Info => info(serial),
        Cmd::Probe => probe(serial),
        Cmd::Fw { path, out } => fw(path, out.as_deref()),
        Cmd::BringUp(a) => {
            let v = bring_up(serial, &a)?;
            report_bring_up(&v);
            Ok(())
        }
        Cmd::I2cRead { fw, addr, reg, count, channel } => {
            let mut v = bring_up(serial, &fw)?;
            let mut bus = StdSleep(v.i2c(channel, 400));
            let mut buf = vec![0u8; count as usize];
            read_regs(&mut bus, addr, reg, &mut buf)?;
            println!("i2c 0x{addr:02x} reg 0x{reg:02x}: {}", hex(&buf));
            Ok(())
        }
        Cmd::I2cWrite { fw, addr, reg, data, channel } => {
            let mut v = bring_up(serial, &fw)?;
            let mut bus = StdSleep(v.i2c(channel, 400));
            write_regs(&mut bus, addr, reg, &data.0)?;
            println!("wrote i2c 0x{addr:02x} reg 0x{reg:02x}: {}", hex(&data.0));
            Ok(())
        }
        Cmd::Tune(a) => {
            tune(serial, &a, CasMode::default())?;
            Ok(())
        }
        Cmd::Record { tune: a, output, seconds, sid, raw, cas } => record(serial, &a, cas, &output, seconds, sid, raw),
    }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect::<Vec<_>>().join(" ")
}

fn list() -> Res {
    let found = Net2280Usb::list_all()?;
    if found.is_empty() {
        println!("no device of the family found (VID 0411, PID 0128/0129/012A/0144)");
    }
    for d in found {
        println!(
            "{:04x}:{:04x} {} serial {} ({})",
            d.vendor_id,
            d.product_id,
            d.product_label(),
            d.serial.as_deref().unwrap_or("-"),
            d.location
        );
    }
    Ok(())
}

fn open(serial: Option<&str>) -> Res<Net2280Usb> {
    let t = Net2280Usb::open_matching(serial)?;
    eprintln!("device {} serial {}", t.product_label(), t.serial().unwrap_or("-"));
    Ok(t)
}

fn pipes(t: &Net2280Usb) -> Vec<u8> {
    t.endpoints().iter().map(|e| e.address).collect()
}

fn info(serial: Option<&str>) -> Res {
    let t = open(serial)?;
    for e in t.endpoints() {
        println!(
            "interface {} endpoint 0x{:02x} {:?} max packet {}",
            e.interface, e.address, e.transfer_type, e.max_packet_size
        );
    }
    // The NET2280's own identity and a few registers: reads only.
    println!("NET2280 config 0x00 (vendor/device): 0x{:08x}", t.local_read(LocalSpace::Config, 0x00)?);
    println!("NET2280 config 0x04 (command/status): 0x{:08x}", t.local_read(LocalSpace::Config, 0x04)?);
    for (off, name) in [(0x00u16, "DEVINIT"), (0x24, "USBIRQENB1"), (0x2C, "IRQSTAT1"), (0x100, "PCIMSTCTL")] {
        println!("NET2280 0x{off:03x} {name}: 0x{:08x}", t.csr_read(off)?);
    }
    Ok(())
}

fn probe(serial: Option<&str>) -> Res {
    let t = open(serial)?;
    let pid = t.product_id();
    let p = pipes(&t);
    let v = Vixs::open(t, pid, &p, Options::default())?;
    let l = v.layout();
    println!("ViXS vendor/device: 0x{:08x}", l.vixs_id);
    for (off, sizing, addr) in &l.bars {
        println!("BAR at config 0x{off:02x}: sizing read-back 0x{sizing:08x}, assigned 0x{addr:08x}");
    }
    println!("FB (BAR0) 0x{:08x}, MMR (BAR1) 0x{:08x}", l.fb, l.mmr);
    println!("FIFO endpoints {:02x?} at {:08x?}, EP_CFG {:08x?}", l.endpoints, l.fifo, l.ep_cfg);
    println!("DMA channel offset 0x{:02x} (mapped: {})", l.dma_offset, l.dma_mapped);
    report_board(v.board_id());
    Ok(())
}

fn report_board(reg: u32) {
    let id = model::board_id(reg);
    println!(
        "board ID register 0x{reg:08x}: chip {}, board 0x{id:02x}",
        if model::is_camino(reg) { "Camino" } else { "unknown" }
    );
    match model::kit_for_board(id, (0, 0)) {
        Some(k) => println!(
            "front end: {} (demodulator 0x{:02x}, tuner 0x{:02x}{})",
            k.name(),
            k.demod_addr(),
            k.tuner_addr(),
            if id == 0xB2 { "; board 0xB2 is MJ102 with subsystem EEEE:EEEE, else MJ110" } else { "" }
        ),
        None => println!("front end: unknown board"),
    }
}

fn firmware_path(arg: Option<PathBuf>) -> Res<PathBuf> {
    arg.or_else(|| std::env::var_os(FW_ENV).map(PathBuf::from))
        .ok_or_else(|| format!("give the vendor driver file BufFsUSB.sys with --firmware or ${FW_ENV}").into())
}

fn fw(path: Option<PathBuf>, out: Option<&Path>) -> Res {
    let path = firmware_path(path)?;
    let f = Firmware::load(&path)?;
    let sha: String = f.sha256().iter().map(|b| format!("{b:02x}")).collect();
    println!("firmware image: {} bytes, sha256 {sha} (known)", f.len());
    for (i, e) in f.vectors()?.iter().enumerate() {
        println!("vector {i}: dst 0x{:08x} words {:08x?}", e.dst, e.words);
    }
    if let Some(out) = out {
        std::fs::write(out, f.image())?;
        println!("wrote {}", out.display());
    }
    Ok(())
}

fn bring_up(serial: Option<&str>, a: &FwArgs) -> Res<Vixs<Net2280Usb>> {
    let f = Firmware::load(firmware_path(a.firmware.clone())?)?;
    let t = open(serial)?;
    let pid = t.product_id();
    let p = pipes(&t);
    let mut v = Vixs::open(t, pid, &p, Options::default())?;
    v.bring_up(&f)?;
    Ok(v)
}

fn report_bring_up(v: &Vixs<Net2280Usb>) {
    report_board(v.board_id());
    if let Some(m) = v.mem() {
        println!("memory init: {m:?}");
    }
    if let Some(m) = v.mips() {
        println!(
            "MIPS up: pool 0x{:08x}+0x{:x}, command ring at FB 0x{:08x}",
            m.pool_offset, m.pool_size, m.cmd_ring_fb
        );
    }
    if let Some(otp) = v.otp_signature() {
        println!("OTP signature: {otp:08x?}");
    }
}

fn channel_hz(channel: &str) -> Res<u32> {
    if let Ok(n) = channel.parse::<u32>()
        && n > 1_000_000
    {
        return Ok(n);
    }
    dth30_core::frontend::parse_channel(channel)
        .and_then(|(band, ch)| dth30_core::frontend::channel_hz(band, ch))
        .ok_or_else(|| format!("unknown channel {channel}").into())
}

/// Bring the device up, start the front end, tune and wait for lock.
fn tune(serial: Option<&str>, a: &TuneArgs, cas: CasMode) -> Res<Dth30<Net2280Usb>> {
    let hz = channel_hz(&a.channel)?;
    let f = Firmware::load(firmware_path(a.fw.firmware.clone())?)?;
    let t = open(serial)?;
    let pid = t.product_id();
    let p = pipes(&t);
    let cfg = Config { subsystem: a.subsystem, cas_mode: cas, ..Config::default() };
    let mut d = Dth30::open(t, pid, &p, Options::default(), cfg)?;
    d.bring_up(&f)?;
    println!("front end {} on board 0x{:02x}, tuning {hz} Hz", d.kit().name(), d.board());
    d.tune(hz)?;
    let deadline = Instant::now() + Duration::from_millis(a.lock_wait as u64);
    loop {
        let st = d.status()?;
        println!(
            "locked {} C/N {:.2} dB (raw 0x{:x}) level {} BER {:?}",
            st.locked, st.cnr_db, st.cnr_raw, st.level, st.ber
        );
        if st.locked {
            return Ok(d);
        }
        if Instant::now() >= deadline {
            return Err("no lock".into());
        }
        std::thread::sleep(Duration::from_millis(250));
    }
}

/// A started reader on a stream's endpoint.
fn reader(d: &mut Dth30<Net2280Usb>, s: &StreamInfo) -> Res<StreamReader> {
    let mut r = d.vixs_mut().bus().stream_reader(s.endpoint, s.transfer_len, StreamReader::DEFAULT_DEPTH)?;
    r.start();
    Ok(r)
}

/// Drain the CAS channel without waiting, logging each block's kind.
/// Returns the card state of the last block, if any came.
fn drain_cas(r: &mut StreamReader) -> Res<Option<u32>> {
    let mut card = None;
    while let Some(b) = r.next(Duration::from_millis(1))? {
        match parse_generic_block(&b) {
            Some((h, body)) => {
                log::info!("CAS block kind 0x{:x} card 0x{:x} body {} bytes", h.kind, h.card, body.len());
                card = Some(h.card);
            }
            None => log::info!("CAS block of {} bytes", b.len()),
        }
    }
    Ok(card)
}

/// Counts TS packets and those whose transport_scrambling_control is set.
#[derive(Default)]
struct TsCount {
    packets: u64,
    scrambled: u64,
    carry: Vec<u8>,
}

impl TsCount {
    fn push(&mut self, data: &[u8]) {
        self.carry.extend_from_slice(data);
        let mut i = 0;
        while self.carry.len() - i >= 188 {
            if self.carry[i] != 0x47 {
                i += 1;
                continue;
            }
            self.packets += 1;
            if self.carry[i + 3] & 0xC0 != 0 {
                self.scrambled += 1;
            }
            i += 188;
        }
        self.carry.drain(..i);
    }
}

/// Open the CAS object (which resets the card) and wait up to 5 s for the
/// card to report itself activated; the firmware descrambles only then.
fn open_cas_and_wait(d: &mut Dth30<Net2280Usb>) -> Res<(StreamInfo, StreamReader)> {
    let info = d.open_cas()?;
    let mut r = reader(d, &info)?;
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut card = 0;
    while card & card_state::ACTIVATED == 0 && Instant::now() < deadline {
        card = drain_cas(&mut r)?.unwrap_or(card);
        std::thread::sleep(Duration::from_millis(20));
    }
    if card & card_state::ACTIVATED != 0 {
        println!("B-CAS card activated (state 0x{card:x})");
    } else {
        println!("B-CAS card not activated within 5 s (state 0x{card:x}); the TS will stay scrambled");
    }
    Ok((info, r))
}

fn record(
    serial: Option<&str>,
    a: &TuneArgs,
    cas: CasMode,
    output: &Path,
    seconds: u64,
    sid: Option<u16>,
    raw: bool,
) -> Res {
    let mut d = tune(serial, a, cas)?;
    // The Raw stream is the broadcast TS as received and needs no card: the
    // CAS object is only opened (and the card reset) for the Program stream.
    let mut cas_rd = if raw { None } else { Some(open_cas_and_wait(&mut d)?) };
    let raw_info = d.open_raw()?;
    let mut raw_rd = reader(&mut d, &raw_info)?;
    let mut out = std::fs::File::create(output)?;
    let mut count = TsCount::default();
    let result =
        record_loop(&mut d, cas_rd.as_mut().map(|(_, r)| r), &mut raw_rd, &mut out, &mut count, seconds, sid, raw);
    // Close in the vendor's order whatever happened: Program (inside the
    // loop), Raw, CAS; each reader is cancelled after its close, then its
    // FIFO is emptied.
    let _ = d.close_raw();
    raw_rd.stop();
    let _ = d.flush_endpoint_fifo(raw_info.endpoint);
    if let Some((cas_info, mut r)) = cas_rd {
        let _ = d.close_cas();
        r.stop();
        let _ = d.flush_endpoint_fifo(cas_info.endpoint);
    }
    result?;
    println!("wrote {}: {} packets, {} scrambled", output.display(), count.packets, count.scrambled);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn record_loop(
    d: &mut Dth30<Net2280Usb>,
    mut cas_rd: Option<&mut StreamReader>,
    raw_rd: &mut StreamReader,
    out: &mut std::fs::File,
    count: &mut TsCount,
    seconds: u64,
    sid: Option<u16>,
    raw: bool,
) -> Res {
    let mut prog_rd = None;
    if raw {
        println!("recording the Raw stream (EP 0x{:02x}) for {seconds} s", raw_rd.endpoint_address());
    } else {
        let mut scan = ProgramScanner::new(sid);
        let deadline = Instant::now() + Duration::from_secs(5);
        let spec = loop {
            if let Some(r) = cas_rd.as_deref_mut() {
                drain_cas(r)?;
            }
            if let Some(b) = raw_rd.next(Duration::from_millis(200))?
                && let Some(spec) = scan.push(&b)
            {
                break spec;
            }
            if Instant::now() >= deadline {
                return Err("no PAT/PMT in the Raw stream within 5 s".into());
            }
        };
        println!(
            "service {} (PMT 0x{:x}, PCR 0x{:x}, ECM {:?}), {} streams, packets of {:?} bytes",
            spec.sid,
            spec.pmt_pid,
            spec.pcr_pid,
            spec.ecm_pid,
            spec.es.len(),
            scan.packet_size()
        );
        let info = d.open_program(&spec)?;
        prog_rd = Some(reader(d, &info)?);
        println!("recording the Program stream (EP 0x{:02x}) for {seconds} s", info.endpoint);
    }
    let end = Instant::now() + Duration::from_secs(seconds);
    while Instant::now() < end {
        if let Some(r) = cas_rd.as_deref_mut() {
            drain_cas(r)?;
        }
        match prog_rd.as_mut() {
            Some(p) => {
                if let Some(b) = p.next(Duration::from_millis(200))? {
                    count.push(&b);
                    out.write_all(&b)?;
                }
                // The Raw stream feeds the Program stream: keep it drained.
                while raw_rd.next(Duration::from_millis(1))?.is_some() {}
            }
            None => {
                if let Some(b) = raw_rd.next(Duration::from_millis(200))? {
                    count.push(&b);
                    out.write_all(&b)?;
                }
            }
        }
    }
    if let Some(mut p) = prog_rd {
        d.close_program()?;
        p.stop();
        d.flush_endpoint_fifo(p.endpoint_address())?;
    }
    Ok(())
}
