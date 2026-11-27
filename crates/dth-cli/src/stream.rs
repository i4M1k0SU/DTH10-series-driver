//! `dth stream`: ask `dth serve` for a stream and copy its TS to stdout,
//! the tuner command a recorder runs when the device is held by the daemon.
//! It owns no device: when the recorder kills it (or closes its pipe) the
//! connection closes and the daemon stops the stream properly.

use std::error::Error;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Read, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use clap::Args;

use crate::{ChannelArgs, Mode, channel_of, stop};
use dth_common::System;
use dth_proto::ipc::{Conn, Target};
use dth_proto::proto::{
    Holder, MAX_LINE, Reply, Request, STREAM_READ_BUF, STREAM_READ_POLL, STREAM_REPLY_TIMEOUT, Status, StreamRequest,
    TunerChoice,
};

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
    /// The socket file of `dth serve` (default: $DTH_SOCKET, else /run/dth/dth.sock on Linux,
    /// %TEMP%\dth-<hash of this executable's folder>.sock on Windows).
    #[arg(long, value_name = "PATH")]
    socket: Option<PathBuf>,
    /// The TCP address of `dth serve` instead (e.g. 127.0.0.1:40780; $DTH_LISTEN).
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
    let conn = Conn::connect(&target).map_err(|e| format!("cannot reach dth serve at {target}: {e}"))?;
    (&conn).write_all(format!("{}\n", req.to_line()).as_bytes())?;
    conn.set_read_timeout(Some(STREAM_REPLY_TIMEOUT))?;
    let mut r = BufReader::new(&conn);
    let mut line = String::new();
    (&mut r).take(MAX_LINE as u64).read_line(&mut line).map_err(|e| format!("no reply from dth serve: {e}"))?;
    match Reply::parse(&line) {
        Reply::Ok(info) => log::info!("streaming after {:?}: {info}", t0.elapsed()),
        Reply::Busy(why) => return Err(format!("busy: {why}").into()),
        Reply::Error(e) => return Err(e.into()),
    }
    conn.set_read_timeout(Some(STREAM_READ_POLL))?;
    let out = stdout_file().map_err(|e| format!("cannot use stdout: {e}"))?;
    // The TS that came along with the reply line goes first.
    let (why, total) = pump(&conn, r.buffer(), out);
    log::info!("{why}: {total} bytes in {:.1} s", t0.elapsed().as_secs_f64());
    Ok(())
}

/// stdout as a file of its own: `io::stdout()` is line buffered (it would
/// flush at every 0x0A of the TS), and its handle is watched for the reader
/// going away.
fn stdout_file() -> io::Result<File> {
    cfg_select! {
        unix => {
            use std::os::fd::AsFd;
            Ok(File::from(io::stdout().as_fd().try_clone_to_owned()?))
        }
        windows => {
            use std::os::windows::io::AsHandle;
            Ok(File::from(io::stdout().as_handle().try_clone_to_owned()?))
        }
    }
}

/// Copy the TS from `conn` to `out` until the daemon ends the stream, the
/// reader of `out` goes away or a stop is requested; `early` is the TS read
/// along with the reply line. Returns why it ended and the bytes copied.
fn pump(conn: &Conn, early: &[u8], out: File) -> (&'static str, u64) {
    let mut w = &out;
    if let Err(e) = w.write_all(early) {
        return (write_failed(&e), 0);
    }
    let mut total = early.len() as u64;
    let watch = Watch::new(conn, &out);
    let mut r = conn;
    let mut buf = vec![0u8; STREAM_READ_BUF];
    let why = loop {
        if stop::stopping() {
            break "stopped";
        }
        // A recorder that only closes its pipe while no TS comes (no lock, no
        // broadcast, a program's PAT awaited) is noticed here, not by a write.
        match watch.wait() {
            Ok(Ready::Input) => {}
            Ok(Ready::Nothing) => continue,
            Ok(Ready::OutputClosed) => break "output closed",
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) => {
                log::error!("wait: {e}");
                break "wait error";
            }
        }
        match r.read(&mut buf) {
            Ok(0) => break "the daemon ended the stream",
            Ok(n) => {
                if let Err(e) = w.write_all(&buf[..n]) {
                    break write_failed(&e);
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
    (why, total)
}

fn write_failed(e: &io::Error) -> &'static str {
    if output_gone(e) {
        "output closed"
    } else {
        log::error!("write: {e}");
        "write error"
    }
}

/// Whether a write to stdout failed because its reader has gone.
fn output_gone(e: &io::Error) -> bool {
    cfg_select! {
        windows => {
            use windows_sys::Win32::Foundation::{ERROR_BROKEN_PIPE, ERROR_NO_DATA, ERROR_PIPE_NOT_CONNECTED};
            // ERROR_NO_DATA: the reader closed its end; ERROR_PIPE_NOT_CONNECTED:
            // the reader is the server of a named pipe and disconnected it.
            e.kind() == io::ErrorKind::BrokenPipe
                || e.raw_os_error().is_some_and(|c| {
                    matches!(c.cast_unsigned(), ERROR_NO_DATA | ERROR_BROKEN_PIPE | ERROR_PIPE_NOT_CONNECTED)
                })
        }
        _ => e.kind() == io::ErrorKind::BrokenPipe,
    }
}

/// What a wait for the socket found.
enum Ready {
    /// TS (or the end of the stream) to read.
    Input,
    /// Nothing within `STREAM_READ_POLL`.
    Nothing,
    /// The reader of stdout has gone.
    OutputClosed,
}

cfg_select! {
    unix => {
        use std::os::fd::{AsFd, AsRawFd, BorrowedFd};

        /// Waits for the socket and, at the same time, for stdout to report
        /// that its reader has gone.
        struct Watch<'a> {
            sock: BorrowedFd<'a>,
            out: BorrowedFd<'a>,
        }

        impl<'a> Watch<'a> {
            fn new(conn: &'a Conn, out: &'a File) -> Self {
                let sock = match conn {
                    Conn::Unix(s) => s.as_fd(),
                    Conn::Tcp(s) => s.as_fd(),
                };
                Watch { sock, out: out.as_fd() }
            }

            fn wait(&self) -> io::Result<Ready> {
                const TIMEOUT_MS: libc::c_int = STREAM_READ_POLL.as_millis() as libc::c_int;
                let mut fds = [
                    libc::pollfd { fd: self.sock.as_raw_fd(), events: libc::POLLIN, revents: 0 },
                    // No events: POLLERR (Linux) or POLLHUP comes anyway when the
                    // reader of a pipe or socket has closed it.
                    libc::pollfd { fd: self.out.as_raw_fd(), events: 0, revents: 0 },
                ];
                // macOS is not known to report a pipe's closed reader to poll:
                // there the next write tells, as it always did.
                let n: libc::nfds_t = if cfg!(target_os = "macos") { 1 } else { 2 };
                // SAFETY: `fds` holds `n` entries and outlives the call.
                if unsafe { libc::poll(fds.as_mut_ptr(), n, TIMEOUT_MS) } < 0 {
                    // EINTR when a stop signal came, which the caller looks at.
                    return Err(io::Error::last_os_error());
                }
                Ok(if fds[1].revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) != 0 {
                    Ready::OutputClosed
                } else if fds[0].revents != 0 {
                    Ready::Input
                } else {
                    Ready::Nothing
                })
            }
        }
    }
    windows => {
        use std::os::windows::io::{AsHandle, AsRawHandle, AsRawSocket, AsSocket, BorrowedHandle, BorrowedSocket};
        use windows_sys::Win32::Networking::WinSock::{FD_SET, SOCKET, SOCKET_ERROR, TIMEVAL, WSAGetLastError, select};
        use windows_sys::Win32::Storage::FileSystem::{FILE_TYPE_PIPE, GetFileType, WriteFile};
        use windows_sys::Win32::System::Pipes::{GetNamedPipeInfo, PIPE_TYPE_MESSAGE};

        /// Waits for the socket; when nothing came, asks stdout (a pipe) with an
        /// empty write whether its reader is still there.
        struct Watch<'a> {
            sock: BorrowedSocket<'a>,
            out: BorrowedHandle<'a>,
            probe: bool,
        }

        impl<'a> Watch<'a> {
            fn new(conn: &'a Conn, out: &'a File) -> Self {
                let sock = match conn {
                    Conn::Unix(s) => s.as_socket(),
                    Conn::Tcp(s) => s.as_socket(),
                };
                let h = out.as_raw_handle();
                // SAFETY: a plain query on a handle `out` keeps open.
                let pipe = unsafe { GetFileType(h) } == FILE_TYPE_PIPE;
                // An empty write reaches the reader of a message-mode pipe as an
                // empty message, which it may take for the end: there a closed
                // reader shows at the next TS written. A handle that cannot tell
                // (the server end of an outbound pipe has no read access) is
                // taken for byte mode.
                let mut flags = 0;
                // SAFETY: as above; the null pointers are optional outputs.
                let message = unsafe {
                    GetNamedPipeInfo(h, &mut flags, std::ptr::null_mut(), std::ptr::null_mut(), std::ptr::null_mut())
                } != 0
                    && flags & PIPE_TYPE_MESSAGE != 0;
                Watch { sock, out: out.as_handle(), probe: pipe && !message }
            }

            fn wait(&self) -> io::Result<Ready> {
                const TIMEOUT: TIMEVAL = TIMEVAL {
                    tv_sec: STREAM_READ_POLL.as_secs() as i32,
                    tv_usec: STREAM_READ_POLL.subsec_micros() as i32,
                };
                let mut set = FD_SET { fd_count: 1, fd_array: [0; 64] };
                set.fd_array[0] = self.sock.as_raw_socket() as SOCKET;
                // SAFETY: the set and the timeout outlive the call.
                match unsafe { select(0, &mut set, std::ptr::null_mut(), std::ptr::null_mut(), &TIMEOUT) } {
                    // SAFETY: reads the calling thread's last socket error.
                    SOCKET_ERROR => Err(io::Error::from_raw_os_error(unsafe { WSAGetLastError() })),
                    0 if self.reader_gone() => Ok(Ready::OutputClosed),
                    0 => Ok(Ready::Nothing),
                    _ => Ok(Ready::Input),
                }
            }

            /// Whether an empty write to stdout says that its reader has gone
            /// (it reaches no reader of an anonymous or byte-mode pipe).
            fn reader_gone(&self) -> bool {
                if !self.probe {
                    return false;
                }
                let none = [0u8; 1];
                let mut written = 0;
                // SAFETY: writes no bytes, synchronously, to a handle `out` keeps open.
                let ok = unsafe {
                    WriteFile(self.out.as_raw_handle(), none.as_ptr(), 0, &mut written, std::ptr::null_mut())
                } != 0;
                !ok && output_gone(&io::Error::last_os_error())
            }
        }
    }
}

#[derive(Args)]
pub struct StatusArgs {
    /// The unit, by USB serial number (needed when the daemon serves several).
    #[arg(long, value_name = "SERIAL")]
    serial: Option<String>,
    /// The socket file of `dth serve` (default: $DTH_SOCKET, else /run/dth/dth.sock on Linux,
    /// %TEMP%\dth-<hash of this executable's folder>.sock on Windows).
    #[arg(long, value_name = "PATH")]
    socket: Option<PathBuf>,
    /// The TCP address of `dth serve` instead (e.g. 127.0.0.1:40780; $DTH_LISTEN).
    #[arg(long, value_name = "ADDR", conflicts_with = "socket")]
    connect: Option<String>,
}

/// How long `dth status` waits for the daemon's answer.
const STATUS_TIMEOUT: Duration = Duration::from_secs(15);

/// `dth status`: ask the daemon for the C/N of both systems (a health check
/// for scripts; the BonDriver sends the same request).
pub fn status(a: &StatusArgs) -> Result<(), Box<dyn Error>> {
    let target = Target::resolve(a.socket.as_deref(), a.connect.as_deref());
    let conn = Conn::connect(&target).map_err(|e| format!("cannot reach dth serve at {target}: {e}"))?;
    conn.set_read_timeout(Some(STATUS_TIMEOUT))?;
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
        .map_err(|e| format!("no reply from dth serve: {e}"))?;
    match Reply::parse(&line) {
        Reply::Ok(info) => {
            let st = Status::parse(&info);
            let show = |v: Option<f32>| v.map_or("-".to_owned(), |x| format!("{x:.1} dB"));
            let held = |h: &Option<Holder>| match h {
                Some(h) => format!(
                    "streaming {} for connection {}{}",
                    h.channel,
                    h.conn,
                    h.client.as_ref().map_or_default(|c| format!(" (client {c})"))
                ),
                None => "free".to_owned(),
            };
            println!("{info}");
            println!("ISDB-T C/N {}, {}", show(st.ter), held(&st.holders[0]));
            if st.second_ter {
                println!("ISDB-T #1 C/N {}, {}", show(st.ter1), held(&st.holders[1]));
            } else if !st.no_satellite {
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
        match Cli::parse_from(std::iter::once("dth").chain(v.iter().copied())).cmd {
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
        assert!(Cli::try_parse_from(["dth", "stream", "-c", "27", "--booster=2"]).is_err());
    }

    #[test]
    fn tuner_is_optional() {
        use dth_proto::proto::TunerChoice;
        assert_eq!(args(&["stream", "-c", "27"]).tuner, None, "no key: tuner 0");
        assert_eq!(args(&["stream", "-c", "27", "--tuner", "1"]).tuner, Some(TunerChoice::Index(1)));
        assert_eq!(args(&["stream", "-c", "27", "--tuner", "auto"]).tuner, Some(TunerChoice::Auto));
        assert!(Cli::try_parse_from(["dth", "stream", "-c", "27", "--tuner", "2"]).is_err());
    }

    mod pump {
        use std::fs::File;
        use std::io::{self, PipeReader, Read, Write};
        use std::net::TcpListener;
        use std::sync::atomic::{AtomicUsize, Ordering};
        use std::sync::mpsc::{self, Receiver};
        use std::time::Duration;

        use dth_proto::ipc::{Conn, Listener, Target};

        /// How long a test waits for `pump` to end: a few of its idle waits.
        const END_WAIT: Duration = Duration::from_secs(5);
        /// Long enough for some idle waits (and on Windows, their empty writes).
        const IDLE: Duration = Duration::from_millis(1500);

        /// A connection as `dth stream` holds it, and the daemon's end of it.
        fn connection(unix: bool) -> (Conn, Conn) {
            if !unix {
                let l = TcpListener::bind("127.0.0.1:0").unwrap();
                let c = Conn::connect(&Target::Tcp(l.local_addr().unwrap().to_string())).unwrap();
                return (c, Conn::Tcp(l.accept().unwrap().0));
            }
            static N: AtomicUsize = AtomicUsize::new(0);
            let path = std::env::temp_dir().join(format!(
                "dth-stream-test-{}-{}.sock",
                std::process::id(),
                N.fetch_add(1, Ordering::Relaxed)
            ));
            let _ = std::fs::remove_file(&path);
            let target = Target::Unix(path.clone());
            let listener = Listener::bind(&target).unwrap();
            let c = Conn::connect(&target).unwrap();
            let daemon = loop {
                match listener.accept() {
                    Ok(d) => break d,
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => std::thread::sleep(Duration::from_millis(10)),
                    Err(e) => panic!("accept: {e}"),
                }
            };
            listener.cleanup(&target);
            drop(listener);
            // and the lock file `Listener::bind` leaves next to the socket file
            let mut lock = path.into_os_string();
            lock.push(".lock");
            let _ = std::fs::remove_file(lock);
            (c, daemon)
        }

        /// An anonymous pipe: its reader, and its writer as stdout is held.
        fn pipe() -> (PipeReader, File) {
            let (r, w) = io::pipe().unwrap();
            let w = cfg_select! {
                unix => File::from(std::os::fd::OwnedFd::from(w)),
                windows => File::from(std::os::windows::io::OwnedHandle::from(w)),
            };
            (r, w)
        }

        /// `pump` on a thread of its own; its result comes on the channel.
        fn spawn(conn: Conn, early: &'static [u8], out: File) -> Receiver<(&'static str, u64)> {
            let (tx, rx) = mpsc::channel();
            std::thread::spawn(move || {
                let _ = tx.send(super::super::pump(&conn, early, out));
            });
            rx
        }

        #[test]
        fn copies_what_came_with_the_reply_then_the_stream_until_its_end() {
            let (conn, daemon) = connection(false);
            let (mut r, w) = pipe();
            let rx = spawn(conn, b"ab", w);
            (&daemon).write_all(b"cd\n").unwrap();
            drop(daemon);
            let mut got = Vec::new();
            r.read_to_end(&mut got).unwrap();
            assert_eq!(got, b"abcd\n");
            assert_eq!(rx.recv_timeout(END_WAIT).unwrap(), ("the daemon ended the stream", 5));
        }

        /// A recorder that closes its pipe while no TS comes (no lock, no
        /// broadcast) ends the stream, over either kind of socket.
        #[test]
        #[cfg_attr(target_os = "macos", ignore = "macOS is not known to report a closed reader to poll")]
        fn ends_when_the_reader_closes_while_no_ts_comes() {
            for unix in [false, true] {
                let (conn, _daemon) = connection(unix);
                let (r, w) = pipe();
                let rx = spawn(conn, b"", w);
                std::thread::sleep(IDLE);
                assert!(rx.try_recv().is_err(), "a reader that is there keeps it going");
                drop(r);
                assert_eq!(rx.recv_timeout(END_WAIT).unwrap(), ("output closed", 0), "unix socket: {unix}");
            }
        }

        /// Whatever tells the reader's absence while idle does not show to a
        /// reader that is there: its first read is the first TS, not an end.
        #[test]
        fn a_reader_sees_nothing_while_idle() {
            let (conn, daemon) = connection(false);
            let (mut r, w) = pipe();
            let rx = spawn(conn, b"", w);
            let reader = std::thread::spawn(move || {
                let mut b = [0u8; 16];
                let n = r.read(&mut b).unwrap();
                (b[..n].to_vec(), r)
            });
            std::thread::sleep(IDLE);
            (&daemon).write_all(b"x").unwrap();
            let (first, _r) = reader.join().unwrap();
            assert_eq!(first, b"x");
            drop(daemon);
            assert_eq!(rx.recv_timeout(END_WAIT).unwrap(), ("the daemon ended the stream", 1));
        }

        #[cfg(windows)]
        mod named_pipe {
            use std::fs::{File, OpenOptions};
            use std::io::{self, Read, Write};
            use std::os::windows::fs::OpenOptionsExt;
            use std::os::windows::io::{AsRawHandle, FromRawHandle, OwnedHandle};
            use std::sync::atomic::{AtomicUsize, Ordering};

            use windows_sys::Win32::Foundation::{GENERIC_READ, INVALID_HANDLE_VALUE};
            use windows_sys::Win32::Storage::FileSystem::{
                FILE_WRITE_ATTRIBUTES, PIPE_ACCESS_DUPLEX, PIPE_ACCESS_OUTBOUND, WriteFile,
            };
            use windows_sys::Win32::System::Pipes::{
                CreateNamedPipeW, DisconnectNamedPipe, GetNamedPipeInfo, PIPE_READMODE_MESSAGE, PIPE_TYPE_BYTE,
                PIPE_TYPE_MESSAGE, SetNamedPipeHandleState,
            };

            use super::{END_WAIT, IDLE, connection, spawn};

            /// A named pipe's server end (stdout, as a recorder that creates the
            /// pipe hands it over) and its client end, open for reading.
            fn named_pipe(open_mode: u32, pipe_mode: u32) -> (File, File) {
                static N: AtomicUsize = AtomicUsize::new(0);
                let name =
                    format!(r"\\.\pipe\dth-stream-test-{}-{}", std::process::id(), N.fetch_add(1, Ordering::Relaxed));
                let wide: Vec<u16> = name.encode_utf16().chain([0]).collect();
                // SAFETY: a NUL-terminated name and no security attributes.
                let h = unsafe {
                    CreateNamedPipeW(wide.as_ptr(), open_mode, pipe_mode, 1, 65536, 65536, 0, std::ptr::null())
                };
                assert_ne!(h, INVALID_HANDLE_VALUE, "{}", io::Error::last_os_error());
                // SAFETY: a handle just created and owned by nothing else.
                let server = File::from(unsafe { OwnedHandle::from_raw_handle(h) });
                // FILE_WRITE_ATTRIBUTES: to switch the read mode to messages.
                let client = OpenOptions::new().access_mode(GENERIC_READ | FILE_WRITE_ATTRIBUTES).open(&name).unwrap();
                (server, client)
            }

            #[test]
            fn ends_when_the_reader_of_an_outbound_pipe_closes() {
                let (server, client) = named_pipe(PIPE_ACCESS_OUTBOUND, PIPE_TYPE_BYTE);
                let mut flags = 0;
                // SAFETY: a query on an open handle; the null pointers are optional outputs.
                let told = unsafe {
                    GetNamedPipeInfo(
                        server.as_raw_handle(),
                        &mut flags,
                        std::ptr::null_mut(),
                        std::ptr::null_mut(),
                        std::ptr::null_mut(),
                    )
                };
                assert_eq!(told, 0, "the server end of an outbound pipe cannot tell its mode");
                let (conn, _daemon) = connection(false);
                let rx = spawn(conn, b"", server);
                std::thread::sleep(IDLE);
                assert!(rx.try_recv().is_err(), "a reader that is there keeps it going");
                drop(client);
                assert_eq!(rx.recv_timeout(END_WAIT).unwrap(), ("output closed", 0));
            }

            #[test]
            fn ends_when_the_pipe_server_disconnects() {
                let (server, _client) = named_pipe(PIPE_ACCESS_OUTBOUND, PIPE_TYPE_BYTE);
                let same = server.try_clone().unwrap();
                let (conn, _daemon) = connection(false);
                let rx = spawn(conn, b"", server);
                std::thread::sleep(IDLE);
                assert!(rx.try_recv().is_err(), "a connected pipe keeps it going");
                // SAFETY: a handle of the pipe's server end, open.
                assert_ne!(unsafe { DisconnectNamedPipe(same.as_raw_handle()) }, 0);
                assert_eq!(rx.recv_timeout(END_WAIT).unwrap(), ("output closed", 0));
            }

            /// An empty write would reach the reader of a message-mode pipe as an
            /// empty message: none is written.
            #[test]
            fn writes_no_empty_message() {
                let (server, mut client) = named_pipe(PIPE_ACCESS_DUPLEX, PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE);
                // SAFETY: a mode on an open handle opened with FILE_WRITE_ATTRIBUTES.
                let set = unsafe {
                    SetNamedPipeHandleState(
                        client.as_raw_handle(),
                        &PIPE_READMODE_MESSAGE,
                        std::ptr::null(),
                        std::ptr::null(),
                    )
                };
                assert_ne!(set, 0, "{}", io::Error::last_os_error());
                // That an empty write would show: the reader reads an empty message.
                let mut written = 0;
                // SAFETY: writes no bytes to an open handle.
                let ok =
                    unsafe { WriteFile(server.as_raw_handle(), [0u8].as_ptr(), 0, &mut written, std::ptr::null_mut()) };
                assert_ne!(ok, 0);
                assert_eq!(client.read(&mut [0u8; 16]).unwrap(), 0);

                let (conn, daemon) = connection(false);
                let rx = spawn(conn, b"", server);
                let reader = std::thread::spawn(move || {
                    let mut b = [0u8; 16];
                    let n = client.read(&mut b).unwrap();
                    (b[..n].to_vec(), client)
                });
                std::thread::sleep(IDLE);
                (&daemon).write_all(b"x").unwrap();
                let (first, _client) = reader.join().unwrap();
                assert_eq!(first, b"x");
                drop(daemon);
                assert_eq!(rx.recv_timeout(END_WAIT).unwrap(), ("the daemon ended the stream", 1));
            }
        }
    }
}
