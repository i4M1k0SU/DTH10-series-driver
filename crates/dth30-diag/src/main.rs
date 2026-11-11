//! Hardware diagnostics for the BUFFALO DT-H30/U2 family (DT-H30/U2,
//! DT-H50/U2, DT-H50/U2W): a ViXS XCode2 chip behind a PLX NET2280.
//!
//! A developer tool, unverified on hardware. Every command that touches the
//! device opens it afresh: `probe` only resets the NET2280 side and reads
//! the chip's identity (no firmware), the others first bring the chip up
//! with the firmware from the user's own vendor driver file.
//!
//! On Windows the device must be bound to WinUSB.

use std::error::Error;
use std::path::{Path, PathBuf};
use std::time::Duration;

use clap::{Args, Parser, Subcommand};
use dth30_core::bus::RemotePci;
use dth30_core::frontend::regs::{read_regs, write_regs};
use dth30_core::frontend::{Frontend, StdSleep};
use dth30_core::model;
use dth30_core::vixs::{Firmware, Options, Vixs};
use dth30_usb::{LocalSpace, Net2280Usb};

type Res<T = ()> = Result<T, Box<dyn Error>>;

/// Environment variable naming the vendor driver file (or an extracted image).
const FW_ENV: &str = "DTH30_FIRMWARE";

#[derive(Parser)]
#[command(
    name = "dth30-diag",
    version,
    about = "Hardware diagnostics for the BUFFALO DT-H30/U2 family of USB TV tuners (developer tool, unverified on hardware)"
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
    Tune {
        #[command(flatten)]
        fw: FwArgs,
        /// UHF channel (13-62), VHF 1-12, CATV "C13".."C63", or a frequency in Hz.
        channel: String,
        #[arg(long, default_value_t = 3000, value_name = "MS")]
        lock_wait: u32,
        /// Subsystem IDs for board 0xB2 (MJ102 when EEEE:EEEE), as "VVVV:DDDD" in hex.
        #[arg(long, value_name = "VVVV:DDDD", value_parser = parse_subsystem)]
        subsystem: Option<(u16, u16)>,
    },
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
        Cmd::Tune { fw, channel, lock_wait, subsystem } => tune(serial, &fw, &channel, lock_wait, subsystem),
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
        println!("endpoint 0x{:02x} {:?} max packet {}", e.address, e.transfer_type, e.max_packet_size);
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

fn tune(serial: Option<&str>, a: &FwArgs, channel: &str, lock_wait: u32, subsystem: Option<(u16, u16)>) -> Res {
    let hz = match channel.parse::<u32>() {
        Ok(n) if n > 1_000_000 => n,
        _ => dth30_core::frontend::parse_channel(channel)
            .and_then(|(band, ch)| dth30_core::frontend::channel_hz(band, ch))
            .ok_or_else(|| format!("unknown channel {channel}"))?,
    };
    let mut v = bring_up(serial, a)?;
    let board = v.board();
    let mut fe = Frontend::for_board(StdSleep(v.i2c(0, 400)), board, subsystem.unwrap_or((0, 0)))?;
    println!("front end {} on board 0x{board:02x}, tuning {hz} Hz", fe.kind().name());
    fe.init()?;
    fe.tune(hz)?;
    let deadline = std::time::Instant::now() + Duration::from_millis(lock_wait as u64);
    loop {
        let st = fe.status()?;
        println!(
            "locked {} C/N {:.2} dB (raw 0x{:x}) level {} BER {:?}",
            st.locked, st.cnr_db, st.cnr_raw, st.level, st.ber
        );
        if st.locked {
            return Ok(());
        }
        if std::time::Instant::now() >= deadline {
            return Err("no lock".into());
        }
        std::thread::sleep(Duration::from_millis(250));
    }
}
