//! `dth10 stream`: ask `dth10 serve` for a stream and copy its TS to stdout,
//! the tuner command a recorder runs when the device is held by the daemon.
//! It owns no device: when the recorder kills it (or closes its pipe) the
//! connection closes and the daemon stops the stream properly.

use std::error::Error;
use std::io::{self, BufRead, BufReader, Read, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use clap::Args;

use crate::{ChannelArgs, Mode, channel_of, stop};
use dth10_tuner::System;
use dth10_tuner::ipc::{Conn, Target};
use dth10_tuner::proto::{Holder, MAX_LINE, Reply, Request, Status, StreamRequest, TunerChoice};

#[derive(Args)]
pub struct StreamArgs {
    #[command(flatten)]
    chan: ChannelArgs,
    /// The unit, by USB serial number (needed when the daemon serves several).
    #[arg(long, value_name = "SERIAL")]
    serial: Option<String>,
    /// Device mode: the program the device descrambles: first, a service id
    /// (decimal or 0x hex), or all (= the last program of the PAT).
    #[arg(long, value_name = "PROG", default_value = "first")]
    program: String,
    /// Stream mode (BS/CS always run in device mode).
    #[arg(long, value_enum, default_value_t = Mode::Device)]
    mode: Mode,
    /// ISDB-T: the terrestrial tuner on a unit with two (the two-tuner models): 0 (default),
    /// 1, or auto (the first free one). One recorder tuner entry per tuner: 0 and 1.
    #[arg(long, value_name = "0|1|auto", value_parser = parse_tuner)]
    tuner: Option<TunerChoice>,
    /// ISDB-T attenuator for this stream: --attenuator (= 1) or --attenuator=0
    /// (default: the daemon's setting).
    #[arg(long, value_name = "0|1", num_args = 0..=1, require_equals = true, default_missing_value = "1", value_parser = clap::value_parser!(u8).range(0..=1))]
    attenuator: Option<u8>,
    /// ISDB-S booster for this stream: --booster (= 1) or --booster=0
    /// (default: the daemon's setting).
    #[arg(long, value_name = "0|1", num_args = 0..=1, require_equals = true, default_missing_value = "1", value_parser = clap::value_parser!(u8).range(0..=1))]
    booster: Option<u8>,
    /// Fail instead of taking the stream over from another client that holds it.
    #[arg(long)]
    if_free: bool,
    /// The socket file of `dth10 serve` (default: $DTH10_SOCKET, else /run/dth10/dth10.sock on Linux,
    /// dth10.sock next to this executable on Windows).
    #[arg(long, value_name = "PATH")]
    socket: Option<PathBuf>,
    /// The TCP address of `dth10 serve` instead (e.g. 127.0.0.1:40780; $DTH10_LISTEN).
    #[arg(long, value_name = "ADDR", conflicts_with = "socket")]
    connect: Option<String>,
}

fn parse_tuner(s: &str) -> Result<TunerChoice, String> {
    TunerChoice::parse(s).ok_or_else(|| format!("{s:?}: 0, 1 or auto"))
}

pub fn run(a: &StreamArgs) -> Result<(), Box<dyn Error>> {
    let channel = channel_of(&a.chan)?;
    if a.tuner.is_some() && channel.system == System::IsdbS {
        return Err("--tuner applies to ISDB-T channels only".into());
    }
    let req = Request::Stream(StreamRequest {
        unit: a.serial.clone(),
        channel,
        program: a.program.clone(),
        raw: a.mode == Mode::Raw,
        busy_fail: a.if_free,
        client: None,
        // the daemon's LNB and lock-wait settings
        lnb: None,
        attenuator: a.attenuator.map(|v| v != 0),
        booster: a.booster.map(|v| v != 0),
        lock_wait_ms: None,
        tuner: a.tuner,
    });
    let target = Target::resolve(a.socket.as_deref(), a.connect.as_deref());
    stop::install();
    let t0 = Instant::now();
    let conn = Conn::connect(&target).map_err(|e| format!("cannot reach dth10 serve at {target}: {e}"))?;
    (&conn).write_all(format!("{}\n", req.to_line()).as_bytes())?;
    // Tuning can take a few seconds (handshake, PSI learn) before the reply.
    conn.set_read_timeout(Some(Duration::from_secs(60)))?;
    let mut r = BufReader::new(&conn);
    let mut line = String::new();
    (&mut r).take(MAX_LINE as u64).read_line(&mut line).map_err(|e| format!("no reply from dth10 serve: {e}"))?;
    match Reply::parse(&line) {
        Reply::Ok(info) => log::info!("streaming after {:?}: {info}", t0.elapsed()),
        Reply::Busy(why) => return Err(format!("busy: {why}").into()),
        Reply::Error(e) => return Err(e.into()),
    }
    conn.set_read_timeout(Some(Duration::from_millis(500)))?;
    let mut out = io::stdout().lock();
    let mut buf = vec![0u8; 64 * 1024];
    let mut total = 0u64;
    let why = loop {
        if stop::stopping() {
            break "stopped";
        }
        match r.read(&mut buf) {
            Ok(0) => break "the daemon ended the stream",
            Ok(n) => {
                if let Err(e) = out.write_all(&buf[..n]) {
                    break if e.kind() == io::ErrorKind::BrokenPipe {
                        "output closed"
                    } else {
                        log::error!("write: {e}");
                        "write error"
                    };
                }
                total += n as u64;
            }
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut | io::ErrorKind::Interrupted
                ) => {}
            Err(e) => {
                log::error!("read: {e}");
                break "read error";
            }
        }
    };
    let _ = out.flush();
    log::info!("{why}: {total} bytes in {:.1} s", t0.elapsed().as_secs_f64());
    Ok(())
}

#[derive(Args)]
pub struct StatusArgs {
    /// The unit, by USB serial number (needed when the daemon serves several).
    #[arg(long, value_name = "SERIAL")]
    serial: Option<String>,
    /// The socket file of `dth10 serve` (default: $DTH10_SOCKET, else /run/dth10/dth10.sock on Linux,
    /// dth10.sock next to this executable on Windows).
    #[arg(long, value_name = "PATH")]
    socket: Option<PathBuf>,
    /// The TCP address of `dth10 serve` instead (e.g. 127.0.0.1:40780; $DTH10_LISTEN).
    #[arg(long, value_name = "ADDR", conflicts_with = "socket")]
    connect: Option<String>,
}

/// `dth10 status`: ask the daemon for the C/N of both systems (a health check
/// for scripts; the BonDriver sends the same request).
pub fn status(a: &StatusArgs) -> Result<(), Box<dyn Error>> {
    let target = Target::resolve(a.socket.as_deref(), a.connect.as_deref());
    let conn = Conn::connect(&target).map_err(|e| format!("cannot reach dth10 serve at {target}: {e}"))?;
    conn.set_read_timeout(Some(Duration::from_secs(15)))?;
    (&conn).write_all(
        format!(
            "{}
",
            Request::Status { unit: a.serial.clone() }.to_line()
        )
        .as_bytes(),
    )?;
    let mut line = String::new();
    (&mut BufReader::new(&conn))
        .take(MAX_LINE as u64)
        .read_line(&mut line)
        .map_err(|e| format!("no reply from dth10 serve: {e}"))?;
    match Reply::parse(&line) {
        Reply::Ok(info) => {
            let st = Status::parse(&info);
            let show = |v: Option<f32>| v.map_or("-".to_owned(), |x| format!("{x:.1} dB"));
            let held = |h: &Option<Holder>| match h {
                Some(h) => format!(
                    "streaming {} for connection {}{}",
                    h.channel,
                    h.conn,
                    h.client.as_ref().map(|c| format!(" (client {c})")).unwrap_or_default()
                ),
                None => "free".to_owned(),
            };
            println!("{info}");
            println!("ISDB-T C/N {}, {}", show(st.ter), held(&st.holders[0]));
            if st.second_ter {
                println!("ISDB-T #1 C/N {}, {}", show(st.ter1), held(&st.holders[1]));
            } else {
                println!("ISDB-S C/N {}, {}", show(st.sat), held(&st.holders[1]));
            }
            if st.busy {
                println!("(a tune is in progress; the C/N values are the last read)");
            }
            Ok(())
        }
        Reply::Busy(why) | Reply::Error(why) => Err(why.into()),
    }
}

#[cfg(test)]
mod tests {
    use clap::Parser;

    use crate::{Cli, Cmd};

    fn args(v: &[&str]) -> super::StreamArgs {
        match Cli::parse_from(std::iter::once("dth10").chain(v.iter().copied())).cmd {
            Cmd::Stream(s) => *s,
            _ => panic!("not stream"),
        }
    }

    #[test]
    fn switches_are_optional() {
        let a = args(&["stream", "-c", "27"]);
        assert_eq!((a.attenuator, a.booster), (None, None), "the daemon's settings");
        let a = args(&["stream", "-c", "27", "--attenuator", "--booster=0"]);
        assert_eq!((a.attenuator, a.booster), (Some(1), Some(0)));
        assert!(Cli::try_parse_from(["dth10", "stream", "-c", "27", "--booster=2"]).is_err());
    }

    #[test]
    fn tuner_is_optional() {
        use dth10_tuner::proto::TunerChoice;
        assert_eq!(args(&["stream", "-c", "27"]).tuner, None, "no key: tuner 0");
        assert_eq!(args(&["stream", "-c", "27", "--tuner", "1"]).tuner, Some(TunerChoice::Index(1)));
        assert_eq!(args(&["stream", "-c", "27", "--tuner", "auto"]).tuner, Some(TunerChoice::Auto));
        assert!(Cli::try_parse_from(["dth10", "stream", "-c", "27", "--tuner", "2"]).is_err());
    }
}
