use core::fmt;

/// Errors produced by the protocol layers.
#[derive(Debug)]
pub enum Error {
    /// Error reported by the USB transport (message is transport specific).
    Usb(String),
    /// A transfer did not complete within the timeout.
    Timeout,
    /// The device transferred fewer bytes than required.
    ShortTransfer { expected: usize, actual: usize },
    /// A buffer exceeds a protocol limit (e.g. 4096-byte read, 256-byte I2C batch).
    TooLarge { limit: usize, actual: usize },
    /// The device information blob could not be parsed.
    InvalidTlv(&'static str),
    /// Generic protocol violation with a short description.
    Protocol(&'static str),
    /// The device/kit is not supported by this implementation.
    Unsupported(String),
    /// I2C access failed (after the USB transfer itself succeeded).
    I2c { addr: u8, reg: Option<u8>, msg: &'static str },
    /// The application credentials (CEPM) could not be read from the user's file.
    AppKeys(String),
}

pub type Result<T> = core::result::Result<T, Error>;

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Usb(m) => write!(f, "USB transport error: {m}"),
            Error::Timeout => write!(f, "USB transfer timed out"),
            Error::ShortTransfer { expected, actual } => {
                write!(f, "short transfer: expected {expected} bytes, got {actual}")
            }
            Error::TooLarge { limit, actual } => {
                write!(f, "buffer of {actual} bytes exceeds protocol limit of {limit}")
            }
            Error::InvalidTlv(m) => write!(f, "invalid device information blob: {m}"),
            Error::Protocol(m) => write!(f, "protocol error: {m}"),
            Error::Unsupported(m) => write!(f, "unsupported: {m}"),
            Error::I2c { addr, reg: Some(r), msg } => {
                write!(f, "I2C error at 0x{addr:02x} reg 0x{r:02x}: {msg}")
            }
            Error::I2c { addr, reg: None, msg } => write!(f, "I2C error at 0x{addr:02x}: {msg}"),
            Error::AppKeys(m) => write!(f, "application key file: {m}"),
        }
    }
}

impl std::error::Error for Error {}
