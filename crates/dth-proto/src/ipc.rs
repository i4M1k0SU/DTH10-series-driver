//! Where `dth serve` listens and its clients (`dth stream`, the BonDriver)
//! connect: a Unix domain socket, on Linux and on Windows
//! alike (Windows has had AF_UNIX since Windows 10 1803; `socket2` exposes
//! it on both), or, as an alternative, TCP on the loopback interface. Both
//! sides see the same byte stream, so the protocol ([`crate::proto`]) does
//! not care which one carries it.

use std::fmt;
use std::io::{self, Read, Write};
use std::mem::MaybeUninit;
use std::net::{SocketAddr, TcpListener, TcpStream, ToSocketAddrs};
use std::path::{Path, PathBuf};
use std::time::Duration;

use socket2::{Domain, SockAddr, Socket, Type};

/// The Unix socket used when nothing else is given on Linux.
#[cfg(unix)]
pub const DEFAULT_SOCKET: &str = "/run/dth/dth.sock";

/// The permissions of the socket file on Unix: connecting needs write
/// permission, so owner and group may connect and others may not. A client
/// running as another user (a recorder) connects as a member of the socket
/// file's group (the daemon's group, or the folder's with its setgid bit).
#[cfg(unix)]
pub const SOCKET_MODE: u32 = 0o660;

/// A number for the folder `dir`, the same for every spelling Windows takes
/// as that folder's (FNV-1a of the path, case-folded). It names the server
/// of the programs in that folder, so that they meet without being told
/// where, and the servers of two folders are apart.
pub fn folder_hash(dir: &Path) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in dir.to_string_lossy().to_lowercase().bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// The socket file of the server of the folder `dir` (on Windows, the folder
/// of `dth.exe` and the BonDriver DLL): `dth-<hash>.sock` in the user's
/// temporary folder (`%TEMP%`), not in `dir` itself, so that nothing is left
/// in that folder and a folder where a socket cannot be made (a network
/// drive) still works.
pub fn folder_socket(dir: &Path) -> PathBuf {
    std::env::temp_dir().join(format!("dth-{:016x}.sock", folder_hash(dir)))
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Target {
    /// A Unix domain socket (a path).
    Unix(PathBuf),
    /// TCP, `host:port`.
    Tcp(String),
}

impl fmt::Display for Target {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Target::Unix(p) => write!(f, "{}", p.display()),
            Target::Tcp(a) => write!(f, "tcp {a}"),
        }
    }
}

impl Target {
    /// `--socket PATH` / `--listen ADDR` (or `--connect ADDR`) of the command line,
    /// else `$DTH_SOCKET` / `$DTH_LISTEN` (or the former `$DTH10_SOCKET` /
    /// `$DTH10_LISTEN`), else the platform default: the socket
    /// `/run/dth/dth.sock` on Linux, on Windows the [`folder_socket`] of the
    /// executable's folder (where the BonDriver DLL's server listens when
    /// `dth.exe` is the one next to the DLL).
    pub fn resolve(socket: Option<&Path>, addr: Option<&str>) -> Target {
        if let Some(p) = socket {
            return Target::Unix(p.to_path_buf());
        }
        if let Some(a) = addr {
            return Target::Tcp(a.to_owned());
        }
        if let Some(p) = std::env::var_os("DTH_SOCKET").or_else(|| std::env::var_os("DTH10_SOCKET")) {
            return Target::Unix(PathBuf::from(p));
        }
        if let Ok(a) = std::env::var("DTH_LISTEN").or_else(|_| std::env::var("DTH10_LISTEN")) {
            return Target::Tcp(a);
        }
        cfg_select! {
            unix => Target::Unix(PathBuf::from(DEFAULT_SOCKET)),
            _ => match std::env::current_exe().ok().and_then(|p| p.parent().map(Path::to_path_buf)) {
                Some(dir) => Target::Unix(folder_socket(&dir)),
                None => Target::Unix(std::env::temp_dir().join("dth.sock")),
            },
        }
    }
}

fn unix_socket() -> io::Result<Socket> {
    Socket::new(Domain::UNIX, Type::STREAM, None)
}

/// Whether a Unix domain socket at `path` can be used here: the address
/// family exists (Windows 10 1803 or later) and the path fits an address.
pub fn unix_socket_usable(path: &std::path::Path) -> bool {
    SockAddr::unix(path).is_ok() && unix_socket().is_ok()
}

/// Whether the file at `path` (not followed) is a Unix domain socket's: on
/// Unix by its type, on Windows by its reparse tag (a symbolic link, a
/// junction or a cloud file is a reparse point too, and connecting to any
/// file that is not a live socket fails the same way).
fn is_socket_file(path: &Path) -> io::Result<bool> {
    cfg_select! {
        unix => {
            use std::os::unix::fs::FileTypeExt;
            Ok(std::fs::symlink_metadata(path)?.file_type().is_socket())
        }
        windows => {
            use std::os::windows::ffi::OsStrExt;
            use windows_sys::Win32::Foundation::INVALID_HANDLE_VALUE;
            use windows_sys::Win32::Storage::FileSystem::{
                FILE_ATTRIBUTE_REPARSE_POINT, FindClose, FindFirstFileW, WIN32_FIND_DATAW,
            };
            /// The reparse tag of an AF_UNIX socket's file.
            const IO_REPARSE_TAG_AF_UNIX: u32 = 0x8000_0023;

            let wide: Vec<u16> = path.as_os_str().encode_wide().chain([0]).collect();
            // SAFETY: plain data, for which all zeroes is a valid value.
            let mut found: WIN32_FIND_DATAW = unsafe { std::mem::zeroed() };
            // SAFETY: a NUL-terminated path and a writable WIN32_FIND_DATAW.
            let h = unsafe { FindFirstFileW(wide.as_ptr(), &mut found) };
            if h == INVALID_HANDLE_VALUE {
                return Err(io::Error::last_os_error());
            }
            // SAFETY: the search handle FindFirstFileW returned, closed once.
            unsafe { FindClose(h) };
            // For a reparse point, dwReserved0 holds its reparse tag.
            Ok(found.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT != 0
                && found.dwReserved0 == IO_REPARSE_TAG_AF_UNIX)
        }
    }
}

/// The daemon's listening socket.
pub enum Listener {
    Unix(Socket),
    Tcp(TcpListener),
}

impl Listener {
    /// Listen on `target`. A socket file left behind by a daemon that is gone
    /// is removed (any other file at the path is an error and stays), and on
    /// Unix the new one gets `SOCKET_MODE` (0660); a live daemon (one that
    /// answers, or a TCP port in use) is an error, so that only one daemon
    /// serves a target.
    pub fn bind(target: &Target) -> io::Result<Listener> {
        match target {
            Target::Unix(path) => {
                if let Some(dir) = path.parent()
                    && !dir.is_empty()
                {
                    std::fs::create_dir_all(dir)?;
                }
                let addr = SockAddr::unix(path)?;
                if std::fs::symlink_metadata(path).is_ok() {
                    // Only a socket file is ever replaced: a path given by mistake
                    // that names another file must not cost that file.
                    if !is_socket_file(path)? {
                        return Err(io::Error::new(
                            io::ErrorKind::AlreadyExists,
                            format!("{} exists and is not a socket", path.display()),
                        ));
                    }
                    let probe = unix_socket()?;
                    match probe.connect(&addr) {
                        Ok(()) => {
                            return Err(io::Error::new(
                                io::ErrorKind::AddrInUse,
                                format!("another dth serve answers at {}", path.display()),
                            ));
                        }
                        Err(_) => std::fs::remove_file(path)?,
                    }
                }
                let l = unix_socket()?;
                l.bind(&addr)?;
                // Not left to the umask (022 makes 0755: only the owner can connect).
                #[cfg(unix)]
                {
                    use std::os::unix::fs::PermissionsExt;
                    std::fs::set_permissions(path, std::fs::Permissions::from_mode(SOCKET_MODE))?;
                }
                l.listen(8)?;
                l.set_nonblocking(true)?;
                Ok(Listener::Unix(l))
            }
            Target::Tcp(addr) => {
                let sa: SocketAddr = addr
                    .to_socket_addrs()?
                    .next()
                    .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, format!("bad address {addr}")))?;
                let l = TcpListener::bind(sa).map_err(|e| {
                    if e.kind() == io::ErrorKind::AddrInUse {
                        io::Error::new(e.kind(), format!("another dth serve listens at {sa}"))
                    } else {
                        e
                    }
                })?;
                l.set_nonblocking(true)?;
                Ok(Listener::Tcp(l))
            }
        }
    }

    /// The next connection, or `WouldBlock`.
    pub fn accept(&self) -> io::Result<Conn> {
        match self {
            Listener::Unix(l) => {
                let (c, _) = l.accept()?;
                c.set_nonblocking(false)?;
                Ok(Conn::Unix(c))
            }
            Listener::Tcp(l) => {
                let (c, _) = l.accept()?;
                c.set_nonblocking(false)?;
                c.set_nodelay(true)?;
                Ok(Conn::Tcp(c))
            }
        }
    }

    /// Remove what the listener left in the file system.
    pub fn cleanup(&self, target: &Target) {
        if let Target::Unix(p) = target {
            let _ = std::fs::remove_file(p);
        }
    }
}

/// One connection, either kind.
pub enum Conn {
    Unix(Socket),
    Tcp(TcpStream),
}

impl Conn {
    pub fn connect(target: &Target) -> io::Result<Conn> {
        match target {
            Target::Unix(p) => {
                let s = unix_socket()?;
                s.connect(&SockAddr::unix(p)?)?;
                Ok(Conn::Unix(s))
            }
            Target::Tcp(addr) => {
                // A refused loopback connect can take a second or two on Windows
                // (the SYN is retried); a daemon that is up accepts within ms.
                let sa: SocketAddr = addr
                    .to_socket_addrs()?
                    .next()
                    .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, format!("bad address {addr}")))?;
                let c = TcpStream::connect_timeout(&sa, Duration::from_millis(500))?;
                c.set_nodelay(true)?;
                Ok(Conn::Tcp(c))
            }
        }
    }

    pub fn set_read_timeout(&self, t: Option<Duration>) -> io::Result<()> {
        match self {
            Conn::Unix(c) => c.set_read_timeout(t),
            Conn::Tcp(c) => c.set_read_timeout(t),
        }
    }

    /// Whether the other side has closed its end (a recorder kills its tuner
    /// command to stop a stream; with nothing to write the socket would not tell).
    pub fn peer_closed(&self) -> bool {
        fn probe(set_nb: &dyn Fn(bool) -> io::Result<()>, peek: &dyn Fn() -> io::Result<usize>) -> bool {
            if set_nb(true).is_err() {
                return false;
            }
            let r = peek();
            let _ = set_nb(false);
            match r {
                Ok(0) => true,
                Ok(_) => false,
                Err(e) => e.kind() != io::ErrorKind::WouldBlock,
            }
        }
        match self {
            Conn::Unix(c) => probe(&|nb| c.set_nonblocking(nb), &|| {
                let mut b = [MaybeUninit::<u8>::uninit(); 1];
                c.peek(&mut b)
            }),
            Conn::Tcp(c) => probe(&|nb| c.set_nonblocking(nb), &|| {
                let mut b = [0u8; 1];
                c.peek(&mut b)
            }),
        }
    }
}

impl Read for Conn {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self {
            Conn::Unix(c) => c.read(buf),
            Conn::Tcp(c) => c.read(buf),
        }
    }
}

impl Write for Conn {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        match self {
            Conn::Unix(c) => c.write(buf),
            Conn::Tcp(c) => c.write(buf),
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        match self {
            Conn::Unix(c) => c.flush(),
            Conn::Tcp(c) => c.flush(),
        }
    }
}

impl Read for &Conn {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let this: &Conn = self;
        match this {
            Conn::Unix(c) => {
                let mut r: &Socket = c;
                r.read(buf)
            }
            Conn::Tcp(c) => {
                let mut r: &TcpStream = c;
                r.read(buf)
            }
        }
    }
}

impl Write for &Conn {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let this: &Conn = self;
        match this {
            Conn::Unix(c) => {
                let mut w: &Socket = c;
                w.write(buf)
            }
            Conn::Tcp(c) => {
                let mut w: &TcpStream = c;
                w.write(buf)
            }
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        let this: &Conn = self;
        match this {
            Conn::Unix(c) => {
                let mut w: &Socket = c;
                w.flush()
            }
            Conn::Tcp(c) => {
                let mut w: &TcpStream = c;
                w.flush()
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{BufRead, BufReader};

    #[test]
    fn the_folder_socket_is_in_the_temp_folder_and_named_by_the_folder() {
        let a = folder_socket(Path::new("C:\\TVTest"));
        assert_eq!(a.parent(), Some(std::env::temp_dir().as_path()));
        let name = a.file_name().and_then(|n| n.to_str()).expect("a name");
        assert!(name.starts_with("dth-") && name.ends_with(".sock") && name.len() == 4 + 16 + 5, "{name}");
        // the same folder however it is spelt, another folder elsewhere
        assert_eq!(a, folder_socket(Path::new("c:\\tvtest")));
        assert_ne!(a, folder_socket(Path::new("C:\\TVTest2")));
        // short enough for a socket address in a usual temp folder
        assert!(unix_socket_usable(&a), "{}", a.display());
    }

    /// A socket file in the temp directory, served and connected to on this
    /// platform (Linux and Windows alike).
    #[test]
    fn unix_socket_round_trip() {
        let path = std::env::temp_dir().join(format!("dth-ipc-test-{}.sock", std::process::id()));
        let _ = std::fs::remove_file(&path);
        let target = Target::Unix(path.clone());
        let listener = Listener::bind(&target).expect("bind");
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            assert_eq!(std::fs::metadata(&path).unwrap().permissions().mode() & 0o777, SOCKET_MODE);
        }
        let client = Conn::connect(&target).expect("connect");
        (&client).write_all(b"hello\n").unwrap();
        let server = loop {
            match listener.accept() {
                Ok(c) => break c,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => std::thread::sleep(Duration::from_millis(10)),
                Err(e) => panic!("accept: {e}"),
            }
        };
        let mut line = String::new();
        BufReader::new(&server).read_line(&mut line).unwrap();
        assert_eq!(line, "hello\n");
        assert!(!server.peer_closed());
        (&server).write_all(b"ok\n").unwrap();
        let mut reply = String::new();
        BufReader::new(&client).read_line(&mut reply).unwrap();
        assert_eq!(reply, "ok\n");
        drop(client);
        std::thread::sleep(Duration::from_millis(50));
        assert!(server.peer_closed());
        assert!(Listener::bind(&target).is_err(), "a live daemon is not replaced");
        listener.cleanup(&target);
        assert!(!path.exists());
    }

    #[test]
    fn a_file_that_is_not_a_socket_is_kept() {
        let path = std::env::temp_dir().join(format!("dth-ipc-test-file-{}.sock", std::process::id()));
        std::fs::write(&path, b"keep me").unwrap();
        let err = Listener::bind(&Target::Unix(path.clone())).err().expect("bind over a plain file");
        assert_eq!(err.kind(), io::ErrorKind::AlreadyExists);
        assert_eq!(std::fs::read(&path).unwrap(), b"keep me");
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn a_socket_left_behind_is_replaced() {
        let path = std::env::temp_dir().join(format!("dth-ipc-test-stale-{}.sock", std::process::id()));
        let _ = std::fs::remove_file(&path);
        let target = Target::Unix(path.clone());
        // A daemon that is gone without cleaning up leaves its socket file.
        drop(Listener::bind(&target).expect("bind"));
        assert!(std::fs::symlink_metadata(&path).is_ok(), "the socket file stays");
        assert!(is_socket_file(&path).unwrap());
        let listener = Listener::bind(&target).expect("bind over the stale socket");
        listener.cleanup(&target);
        assert!(std::fs::symlink_metadata(&path).is_err());
    }
}
