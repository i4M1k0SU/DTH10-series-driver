//! Vendor control-transfer encoding used by the original driver.
//!
//! The driver builds an 8-byte request descriptor and hands it to one of two
//! primitives:
//!
//! * control **OUT**: `bmRequestType = 0x41` (vendor, interface,
//!   host-to-device), 10 s timeout.
//! * control **IN**: `bmRequestType = 0xC1` (vendor, interface,
//!   device-to-host), 7 s timeout, at most 0x1000 bytes.
//!
//! Descriptor layout:
//!
//! | byte | content |
//! |------|---------|
//! | 0    | 1 (internal tag, not transmitted) |
//! | 1    | flags → **bRequest**. bit1 = OUT data stage, bit0 = IN data stage, upper bits = caller flags & 0xFC |
//! | 2-3  | **wValue** ("cmd": command class in the high byte, I2C address in the low byte for I2C classes) |
//! | 4-5  | **wIndex** ("arg": sub-command / register / GPIO opcode) |
//! | 6-7  | wLength |
//!
//! A command that both writes and reads is executed as two control transfers
//! with identical wValue/wIndex (bRequest = 0x03 for both).

use core::time::Duration;

use crate::error::{Error, Result};

/// `bmRequestType` of OUT requests in the original driver.
pub const BMREQ_OUT: u8 = 0x41;
/// `bmRequestType` of IN requests in the original driver.
pub const BMREQ_IN: u8 = 0xC1;

/// Timeout used by the original driver for OUT transfers.
pub const TIMEOUT_OUT: Duration = Duration::from_secs(10);
/// Timeout used by the original driver for IN transfers.
pub const TIMEOUT_IN: Duration = Duration::from_secs(7);
/// Largest IN transfer the original driver issues (its IN primitive clamps to 0x1000).
pub const MAX_IN_LEN: usize = 0x1000;

/// bRequest bit: the request carries an OUT data stage.
pub const FLAG_OUT: u8 = 0x02;
/// bRequest bit: the request carries an IN data stage.
pub const FLAG_IN: u8 = 0x01;

/// Minimal vendor-control-transfer interface a transport must provide.
///
/// Implementations are expected to use the vendor request type with the
/// *interface* recipient (0x41 / 0xC1) like the original driver. Note that
/// WinUSB rejects interface-recipient requests whose `wIndex` low byte is not
/// the interface number, so a WinUSB transport may have to fall back to the
/// *device* recipient (0x40 / 0xC0); whether the firmware accepts that must be
/// verified on real hardware.
pub trait UsbControl {
    /// Vendor control OUT transfer. Returns the number of bytes transferred.
    fn control_out(&self, request: u8, value: u16, index: u16, data: &[u8], timeout: Duration) -> Result<usize>;

    /// Vendor control IN transfer into `buf`. Returns the number of bytes received.
    fn control_in(&self, request: u8, value: u16, index: u16, buf: &mut [u8], timeout: Duration) -> Result<usize>;

    /// Hold the device's command channel for a multi-step exchange (a CEPM
    /// handshake) so that commands another thread sends to the same device
    /// do not get between its steps. The hold ends when the returned value is
    /// dropped. The default holds nothing (single-threaded transports).
    fn hold(&self) -> Box<dyn std::any::Any> {
        Box::new(())
    }
}

/// Well-known `wValue` / `wIndex` values (see `docs/00-architecture-notes.md` §3.3).
pub mod cmd {
    /// wValue of "system" commands.
    pub const SYS: u16 = 0x0000;
    /// wIndex: read the device-information TLV blob (IN 4096).
    pub const SYS_DEVICE_INFO: u16 = 0xC0DE;
    /// wIndex: read firmware version (IN 16, version at bytes 8..12).
    pub const SYS_FW_VERSION: u16 = 0x0000;
    /// wIndex: device start (OUT, no data), sent once the tuner kits are initialised.
    pub const SYS_START: u16 = 0x0010;
    /// wIndex: GPIO enable/direction, payload `{0, mask, mask}` (u32 LE ×3).
    pub const SYS_GPIO_ENABLE: u16 = 0x1004;
    /// wIndex: GPIO output, payload `{0, mask, value}` (u32 LE ×3).
    pub const SYS_GPIO_SET: u16 = 0x1006;
    /// wIndex: GPIO/status read (IN 16). Not in every firmware (the DT-H10/U7 has none).
    pub const SYS_GPIO_READ: u16 = 0x1010;

    /// wValue: I2C batch write (wIndex 0), payload = concatenated batch entries.
    pub const I2C_BATCH: u16 = 0x2B00;
    /// wValue high byte: raw I2C transfer, low byte = 7-bit address.
    pub const I2C_RAW: u16 = 0x2C00;
    /// wIndex for raw I2C write.
    pub const I2C_RAW_WRITE: u16 = 0x0030;
    /// wIndex for raw I2C read.
    pub const I2C_RAW_READ: u16 = 0x0033;
    /// wValue high byte: register (sub-address) I2C transfer, wIndex = register.
    pub const I2C_REG: u16 = 0x2D00;

    /// wValue of stream / "Cobalt BDA" commands.
    pub const CBDA: u16 = 0xCBDA;
    /// wIndex: stream enable/disable (OUT 16).
    pub const CBDA_STREAM_CTRL: u16 = 0x0002;
    /// wIndex: PID filter table (OUT 12 + 4n).
    pub const CBDA_PID_FILTER: u16 = 0x1000;
    /// wIndex: capability/status query (IN 12). Not in every firmware (the DT-H10/U7 has none).
    pub const CBDA_QUERY_BCAC: u16 = 0xBCAC;
    /// wIndex: fallback for the capability/status query (IN 4, flags 0x40).
    pub const CBDA_QUERY_CACD: u16 = 0xCACD;
    /// wIndex: query.
    pub const CBDA_QUERY_91D1: u16 = 0x91D1;
    /// wIndex: query (IN 48).
    pub const CBDA_QUERY_BDD0: u16 = 0xBDD0;
    /// wIndex: query.
    pub const CBDA_QUERY_BDD7: u16 = 0xBDD7;
    /// wIndex: query.
    pub const CBDA_QUERY_BDE5: u16 = 0xBDE5;
    /// wIndex: query (IN 96, flags 0x40).
    pub const CBDA_QUERY_BCAD: u16 = 0xBCAD;
}

/// Equivalent of the original driver's command primitive: execute one logical command.
///
/// * `flags`  – caller flags; only bits 2..7 are kept (`& 0xFC`).
/// * `out`    – OUT payload (may be empty).
/// * `inbuf`  – IN buffer; `None` or empty means no IN stage.
///
/// Returns the number of bytes received in the IN stage (0 when there is none).
///
/// Mirrors the original decision logic exactly: an OUT stage is issued when
/// there is OUT data **or** when no IN stage is requested (so a bare command
/// becomes a zero-length OUT transfer with bRequest = 0x02).
pub fn send<T: UsbControl + ?Sized>(
    t: &T,
    flags: u8,
    value: u16,
    index: u16,
    out: &[u8],
    inbuf: Option<&mut [u8]>,
) -> Result<usize> {
    let want_in = inbuf.as_ref().map_or(0, |b| b.len());
    if want_in > MAX_IN_LEN {
        return Err(Error::TooLarge { limit: MAX_IN_LEN, actual: want_in });
    }
    if out.len() > u16::MAX as usize {
        return Err(Error::TooLarge { limit: u16::MAX as usize, actual: out.len() });
    }

    let mut request = flags & 0xFC;
    let do_out = !out.is_empty() || want_in == 0;
    let do_in = want_in != 0;
    if do_out {
        request |= FLAG_OUT;
    }
    if do_in {
        request |= FLAG_IN;
    }

    if do_out {
        let n = t.control_out(request, value, index, out, TIMEOUT_OUT)?;
        if n < out.len() {
            return Err(Error::ShortTransfer { expected: out.len(), actual: n });
        }
    }
    if do_in {
        let buf = inbuf.expect("checked above");
        return t.control_in(request, value, index, buf, TIMEOUT_IN);
    }
    Ok(0)
}

/// Convenience: command with OUT payload only.
pub fn write<T: UsbControl + ?Sized>(t: &T, value: u16, index: u16, out: &[u8]) -> Result<()> {
    send(t, 0, value, index, out, None).map(|_| ())
}

/// Convenience: command with IN payload only. Returns bytes received.
pub fn read<T: UsbControl + ?Sized>(t: &T, flags: u8, value: u16, index: u16, buf: &mut [u8]) -> Result<usize> {
    send(t, flags, value, index, &[], Some(buf))
}

/// Read that must return exactly `buf.len()` bytes (as the original driver's raw I2C read requires).
pub fn read_exact<T: UsbControl + ?Sized>(t: &T, flags: u8, value: u16, index: u16, buf: &mut [u8]) -> Result<()> {
    let n = read(t, flags, value, index, buf)?;
    if n != buf.len() {
        return Err(Error::ShortTransfer { expected: buf.len(), actual: n });
    }
    Ok(())
}
