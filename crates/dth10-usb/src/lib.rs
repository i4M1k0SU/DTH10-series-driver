//! `nusb`-based transport for [`dth10_core`].
//!
//! * Windows: the device must be bound to **WinUSB** (e.g. with Zadig / libwdi,
//!   VID 0x0411). WinUSB refuses interface-recipient control transfers whose
//!   `wIndex` low byte differs from the interface number, which this protocol
//!   violates constantly, so on Windows the *device* recipient (0x40 / 0xC0)
//!   is used by default (confirmed to work on a DT-H70/U2, docs/40).
//! * Linux: usbfs (`/dev/bus/usb/BBB/DDD`); the original recipient (interface,
//!   0x41 / 0xC1) is used. Devices are normally found through sysfs; when sysfs
//!   is not available (a chroot that only has `/dev/bus/usb` bind-mounted), the
//!   device nodes are scanned directly ([`UsbTransport::list_all`]) or a node is
//!   opened by path (`UsbTransport::open_path`).
//! * macOS: IOKit (nusb links CoreFoundation and IOKit). No driver binding is
//!   needed for this vendor-specific device; the interface recipient is used,
//!   as on Linux. Devices are found through nusb's own enumeration (the
//!   `/dev/bus/usb` scan and `UsbTransport::open_path` are Linux-only).

use core::time::Duration;

use dth10_core::LockExt;
use dth10_core::error::{Error, Result};
use dth10_core::protocol::UsbControl;
use nusb::MaybeFuture;
use nusb::descriptors::TransferType;
use nusb::transfer::{Buffer, Bulk, ControlIn, ControlOut, ControlType, In, Interrupt, Recipient, TransferError};
use nusb::{Endpoint, Interface};

pub use nusb;

/// Which recipient field to put in `bmRequestType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecipientMode {
    /// 0x41 / 0xC1 — what the original driver sends.
    Interface,
    /// 0x40 / 0xC0 — WinUSB-compatible alternative.
    Device,
}

impl RecipientMode {
    /// Platform default: `Device` on Windows (WinUSB limitation), `Interface` elsewhere.
    pub fn platform_default() -> Self {
        if cfg!(windows) { RecipientMode::Device } else { RecipientMode::Interface }
    }
}

fn map_err(e: TransferError) -> Error {
    match e {
        // nusb cancels a control transfer when its timeout elapses.
        TransferError::Cancelled => Error::Timeout,
        other => Error::Usb(other.to_string()),
    }
}

fn map_io(e: nusb::Error) -> Error {
    Error::Usb(e.to_string())
}

/// Endpoint assignment as the original driver makes it.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PipeMap {
    /// EP 0x81 (bulk or interrupt IN) → TS stream #0.
    pub ts0: Option<(u8, TransferType)>,
    /// EP 0x82 (or another bulk IN) → TS stream #1.
    pub ts1: Option<(u8, TransferType)>,
    /// Bulk OUT pipe (stored by the driver, use unknown).
    pub bulk_out: Option<u8>,
    /// Interrupt IN pipe other than 0x81/0x82 → event notifications.
    pub interrupt_in: Option<u8>,
}

impl PipeMap {
    pub fn from_interface(iface: &Interface) -> Self {
        let mut m = PipeMap::default();
        let Some(desc) = iface.descriptor() else { return m };
        for ep in desc.endpoints() {
            let addr = ep.address();
            let is_in = addr & 0x80 != 0;
            match ep.transfer_type() {
                TransferType::Bulk => {
                    if is_in {
                        if addr == 0x81 {
                            m.ts0 = Some((addr, TransferType::Bulk));
                        } else {
                            m.ts1 = Some((addr, TransferType::Bulk));
                        }
                    } else {
                        m.bulk_out = Some(addr);
                    }
                }
                TransferType::Interrupt if is_in => match addr {
                    0x81 => m.ts0 = Some((addr, TransferType::Interrupt)),
                    0x82 => m.ts1 = Some((addr, TransferType::Interrupt)),
                    _ => m.interrupt_in = Some(addr),
                },
                _ => {}
            }
        }
        m
    }
}

/// A supported device found by [`UsbTransport::list_all`]: through the OS
/// enumeration (`info`) or, on Linux without sysfs, by its usbfs node (`path`).
#[derive(Debug, Clone)]
pub struct Found {
    pub vendor_id: u16,
    pub product_id: u16,
    /// USB serial number string (`iSerialNumber`), if the device has one.
    pub serial: Option<String>,
    /// Where it is: `bus B addr D`, or the device node path.
    pub location: String,
    pub info: Option<nusb::DeviceInfo>,
    #[cfg(any(target_os = "linux", target_os = "android"))]
    pub path: Option<std::path::PathBuf>,
}

impl Found {
    pub fn product_name(&self) -> &'static str {
        dth10_core::product_name(self.product_id).unwrap_or("?")
    }

    /// Open this device and claim interface 0.
    pub fn open(&self, mode: RecipientMode) -> Result<UsbTransport> {
        if let Some(info) = self.info.as_ref() {
            return UsbTransport::open(info, mode);
        }
        #[cfg(any(target_os = "linux", target_os = "android"))]
        if let Some(p) = self.path.as_ref() {
            return UsbTransport::open_path(p, mode);
        }
        Err(Error::Usb("device entry has neither OS info nor a device node".into()))
    }
}

/// An opened tuner bridge (interface 0 claimed). Cloning shares the same
/// USB handle (nusb handles are reference counted), which is how the two
/// tuner/filter instances of a dual-tuner device share one bridge.
#[derive(Clone)]
pub struct UsbTransport {
    device: nusb::Device,
    iface: Interface,
    recipient: Recipient,
    index_fix: bool,
    serial: Option<String>,
    product_id: u16,
    /// Shared by every clone of the transport: see [`UsbControl::hold`].
    hold: std::sync::Arc<CommandHold>,
}

/// The command channel of one device, held across a multi-step exchange
/// (`UsbControl::hold`). A plain flag under a mutex rather than a `MutexGuard`,
/// so that the hold can be returned as an owned value.
#[derive(Default)]
pub struct CommandHold {
    held: std::sync::Mutex<bool>,
    freed: std::sync::Condvar,
}

/// Releases the hold when dropped.
pub struct Held(std::sync::Arc<CommandHold>);

impl CommandHold {
    fn hold(self: &std::sync::Arc<Self>) -> Held {
        let mut held = self.held.locked();
        while *held {
            held = self.freed.wait(held).unwrap_or_else(|e| e.into_inner());
        }
        *held = true;
        Held(self.clone())
    }
}

impl Drop for Held {
    fn drop(&mut self) {
        *self.0.held.locked() = false;
        self.0.freed.notify_one();
    }
}

fn is_supported(vid: u16, pid: u16) -> bool {
    vid == dth10_core::VID_BUFFALO && dth10_core::product_name(pid).is_some()
}

impl UsbTransport {
    /// Enumerate supported devices (VID 0x0411 and a PID of
    /// [`dth10_core::SUPPORTED_PIDS`], unverified models included) through
    /// the OS (sysfs on Linux).
    pub fn list() -> Result<Vec<nusb::DeviceInfo>> {
        let it = nusb::list_devices().wait().map_err(map_io)?;
        Ok(it.filter(|d| is_supported(d.vendor_id(), d.product_id())).collect())
    }

    /// Enumerate supported devices; on Linux, when the OS enumeration yields
    /// nothing (no sysfs, e.g. inside a chroot), scan `/dev/bus/usb` instead.
    pub fn list_all() -> Result<Vec<Found>> {
        let mut out = Vec::new();
        match Self::list() {
            Ok(list) => {
                for d in list {
                    out.push(Found {
                        vendor_id: d.vendor_id(),
                        product_id: d.product_id(),
                        serial: d.serial_number().map(str::to_owned),
                        location: format!("bus {} addr {}", d.bus_id(), d.device_address()),
                        info: Some(d),
                        #[cfg(any(target_os = "linux", target_os = "android"))]
                        path: None,
                    });
                }
            }
            Err(e) => log::debug!("OS device enumeration failed: {e}"),
        }
        #[cfg(any(target_os = "linux", target_os = "android"))]
        if out.is_empty() {
            out = linux::scan_dev_bus_usb();
        }
        Ok(out)
    }

    /// Open a device and claim interface 0.
    pub fn open(info: &nusb::DeviceInfo, mode: RecipientMode) -> Result<Self> {
        let device = info.open().wait().map_err(map_io)?;
        Self::from_device(device, mode, info.serial_number().map(str::to_owned))
    }

    /// Open the usbfs node `path` (`/dev/bus/usb/BBB/DDD`) directly, for hosts
    /// without sysfs. The node must be a supported device.
    #[cfg(any(target_os = "linux", target_os = "android"))]
    pub fn open_path(path: &std::path::Path, mode: RecipientMode) -> Result<Self> {
        let device = linux::device_from_path(path)?;
        let desc = device.device_descriptor();
        if !is_supported(desc.vendor_id(), desc.product_id()) {
            return Err(Error::Usb(format!(
                "{} is {:04x}:{:04x}, not a supported tuner",
                path.display(),
                desc.vendor_id(),
                desc.product_id()
            )));
        }
        let serial = linux::read_serial(&device);
        Self::from_device(device, mode, serial)
    }

    fn from_device(device: nusb::Device, mode: RecipientMode, serial: Option<String>) -> Result<Self> {
        // usbfs leaves the configuration as the kernel chose it; select
        // configuration 1 only when another one is active, as re-selecting the
        // active configuration resets the endpoints.
        if device.active_configuration().map(|c| c.configuration_value()).ok() != Some(1)
            && let Err(e) = device.set_configuration(1).wait()
        {
            log::debug!("set_configuration(1): {e}");
        }
        // Detaching is a no-op on Windows; on Linux it frees interface 0 from a
        // kernel driver that may have bound to it.
        let product_id = device.device_descriptor().product_id();
        let iface = device.detach_and_claim_interface(0).wait().map_err(map_io)?;
        let recipient = match mode {
            RecipientMode::Interface => Recipient::Interface,
            RecipientMode::Device => Recipient::Device,
        };
        Ok(UsbTransport { device, iface, recipient, index_fix: false, serial, product_id, hold: Default::default() })
    }

    /// Open the first supported device found.
    pub fn open_first(mode: RecipientMode) -> Result<Self> {
        Self::open_matching(mode, None)
    }

    /// Open the supported device whose USB serial number equals `serial`
    /// (the first device when `serial` is `None`).
    pub fn open_matching(mode: RecipientMode, serial: Option<&str>) -> Result<Self> {
        let list = Self::list_all()?;
        let found = match serial {
            Some(sn) => list
                .iter()
                .find(|d| d.serial.as_deref().is_some_and(|s| s.eq_ignore_ascii_case(sn)))
                .ok_or_else(|| Error::Usb(format!("no supported device with serial {sn}")))?,
            None => list.first().ok_or_else(|| Error::Usb("no supported device found".into()))?,
        };
        found.open(mode)
    }

    pub fn interface(&self) -> &Interface {
        &self.iface
    }

    pub fn device(&self) -> &nusb::Device {
        &self.device
    }

    /// The device's USB serial number string, when it has one.
    pub fn serial(&self) -> Option<&str> {
        self.serial.as_deref()
    }

    /// The device's USB product ID.
    pub fn product_id(&self) -> u16 {
        self.product_id
    }

    pub fn pipes(&self) -> PipeMap {
        PipeMap::from_interface(&self.iface)
    }

    /// Open a bulk IN endpoint for streaming.
    pub fn bulk_in(&self, addr: u8) -> Result<Endpoint<Bulk, In>> {
        self.iface.endpoint::<Bulk, In>(addr).map_err(map_io)
    }

    /// Open an interrupt IN endpoint.
    pub fn interrupt_in(&self, addr: u8) -> Result<Endpoint<Interrupt, In>> {
        self.iface.endpoint::<Interrupt, In>(addr).map_err(map_io)
    }

    /// Experimental: when using the interface recipient on WinUSB, force the
    /// low byte of `wIndex` to the interface number (this changes the command
    /// and is only useful for probing).
    pub fn set_index_fix(&mut self, on: bool) {
        self.index_fix = on;
    }

    fn fix_index(&self, index: u16) -> u16 {
        if self.index_fix && self.recipient == Recipient::Interface {
            (index & 0xFF00) | self.iface.interface_number() as u16
        } else {
            index
        }
    }
}

impl UsbControl for UsbTransport {
    fn hold(&self) -> Box<dyn std::any::Any> {
        Box::new(self.hold.hold())
    }

    fn control_out(&self, request: u8, value: u16, index: u16, data: &[u8], timeout: Duration) -> Result<usize> {
        self.iface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: self.recipient,
                    request,
                    value,
                    index: self.fix_index(index),
                    data,
                },
                timeout,
            )
            .wait()
            .map_err(map_err)?;
        Ok(data.len())
    }

    fn control_in(&self, request: u8, value: u16, index: u16, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let v = self
            .iface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: self.recipient,
                    request,
                    value,
                    index: self.fix_index(index),
                    length: buf.len() as u16,
                },
                timeout,
            )
            .wait()
            .map_err(map_err)?;
        let n = v.len().min(buf.len());
        buf[..n].copy_from_slice(&v[..n]);
        Ok(n)
    }
}

/// Linux: device nodes without sysfs.
#[cfg(any(target_os = "linux", target_os = "android"))]
mod linux {
    use std::path::{Path, PathBuf};

    use dth10_core::error::{Error, Result};
    use nusb::MaybeFuture;

    use super::{Found, is_supported, map_io};

    /// Wrap the usbfs node `path` in a nusb device.
    pub(super) fn device_from_path(path: &Path) -> Result<nusb::Device> {
        let f = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(path)
            .map_err(|e| Error::Usb(format!("open {}: {e}", path.display())))?;
        nusb::Device::from_fd(f.into()).wait().map_err(map_io)
    }

    /// The serial number string descriptor (English, else the first language).
    pub(super) fn read_serial(device: &nusb::Device) -> Option<String> {
        let idx = device.device_descriptor().serial_number_string_index()?;
        let timeout = std::time::Duration::from_millis(500);
        let langs: Vec<u16> =
            device.get_string_descriptor_supported_languages(timeout).wait().map_or_default(|it| it.collect());
        let lang = if langs.contains(&0x0409) { 0x0409 } else { langs.first().copied().unwrap_or(0x0409) };
        device.get_string_descriptor(idx, lang, timeout).wait().ok()
    }

    /// Every supported device under `/dev/bus/usb`, found by opening each node
    /// and reading its descriptors (needs read/write access to the nodes).
    pub(super) fn scan_dev_bus_usb() -> Vec<Found> {
        let mut out = Vec::new();
        let Ok(buses) = std::fs::read_dir("/dev/bus/usb") else { return out };
        let mut nodes: Vec<PathBuf> = Vec::new();
        for bus in buses.flatten() {
            if let Ok(devs) = std::fs::read_dir(bus.path()) {
                nodes.extend(devs.flatten().map(|d| d.path()));
            }
        }
        nodes.sort();
        for path in nodes {
            let dev = match device_from_path(&path) {
                Ok(d) => d,
                Err(e) => {
                    log::trace!("{}: {e}", path.display());
                    continue;
                }
            };
            let desc = dev.device_descriptor();
            if !is_supported(desc.vendor_id(), desc.product_id()) {
                continue;
            }
            out.push(Found {
                vendor_id: desc.vendor_id(),
                product_id: desc.product_id(),
                serial: read_serial(&dev),
                location: path.display().to_string(),
                info: None,
                path: Some(path),
            });
        }
        out
    }
}

/// Continuous reader over a bulk IN endpoint, modelled on the WDF continuous
/// reader the original driver configures: transfer length
/// 39936 bytes (78 × 512), 4 transfers in flight.
///
/// Transfer buffers are plain heap buffers ([`Buffer::new`]), not nusb's
/// zero-copy ones: on an older 32-bit ARM kernel (Linux 4.9) the usbfs
/// memory-mapped buffers did not show what the device had written, while heap
/// buffers of the same reads were intact. At a few MB/s the copy is cheap.
pub struct StreamReader {
    ep: Endpoint<Bulk, In>,
    transfer_len: usize,
    depth: usize,
}

impl StreamReader {
    pub const DEFAULT_TRANSFER_LEN: usize = 39936;
    pub const DEFAULT_DEPTH: usize = 4;

    pub fn new(ep: Endpoint<Bulk, In>, transfer_len: usize, depth: usize) -> Result<Self> {
        let mps = ep.max_packet_size();
        if mps == 0 || !transfer_len.is_multiple_of(mps) {
            return Err(Error::Protocol("transfer length must be a multiple of the max packet size"));
        }
        Ok(StreamReader { ep, transfer_len, depth })
    }

    /// Submit the initial transfers.
    pub fn start(&mut self) {
        while self.ep.pending() < self.depth {
            self.ep.submit(Buffer::new(self.transfer_len));
        }
    }

    /// Wait for the next completed transfer and resubmit a buffer in its place.
    /// Returns the received bytes, `Ok(None)` on timeout.
    pub fn next(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>> {
        if self.ep.pending() == 0 {
            self.start();
        }
        let Some(c) = self.ep.wait_next_complete(timeout) else { return Ok(None) };
        self.ep.submit(Buffer::new(self.transfer_len));
        match c.status {
            Ok(()) => {
                let mut v = c.buffer.into_vec();
                v.truncate(c.actual_len);
                Ok(Some(v))
            }
            Err(TransferError::Cancelled) => Ok(None),
            Err(e) => Err(map_err(e)),
        }
    }

    /// Cancel all in-flight transfers and drain them.
    pub fn stop(&mut self) {
        self.ep.cancel_all();
        while self.ep.pending() > 0 {
            if self.ep.wait_next_complete(Duration::from_millis(500)).is_none() {
                break;
            }
        }
    }

    /// Clear a halt condition on the endpoint (the original driver aborts and resets the pipe on failure).
    pub fn clear_halt(&mut self) -> Result<()> {
        self.ep.clear_halt().wait().map_err(map_io)
    }

    pub fn endpoint_address(&self) -> u8 {
        self.ep.endpoint_address()
    }
}

/// Reader over the interrupt IN endpoint (events, see `dth10_core::events`).
/// The original configures 512-byte reads with one transfer in flight.
pub struct EventReader {
    ep: Endpoint<Interrupt, In>,
    transfer_len: usize,
}

impl EventReader {
    pub fn new(ep: Endpoint<Interrupt, In>) -> Self {
        let mps = ep.max_packet_size().max(1);
        // 512 rounded up to a multiple of the max packet size.
        let transfer_len = 512usize.div_ceil(mps) * mps;
        EventReader { ep, transfer_len }
    }

    /// Wait for the next event transfer; `Ok(None)` on timeout.
    pub fn next(&mut self, timeout: Duration) -> Result<Option<dth10_core::events::Event>> {
        if self.ep.pending() == 0 {
            self.ep.submit(Buffer::new(self.transfer_len));
        }
        let Some(c) = self.ep.wait_next_complete(timeout) else { return Ok(None) };
        match c.status {
            Ok(()) => Ok(dth10_core::events::Event::parse(&c.buffer[..c.actual_len])),
            Err(TransferError::Cancelled) => Ok(None),
            Err(e) => Err(map_err(e)),
        }
    }

    pub fn stop(&mut self) {
        self.ep.cancel_all();
        while self.ep.pending() > 0 {
            if self.ep.wait_next_complete(Duration::from_millis(500)).is_none() {
                break;
            }
        }
    }
}

/// Allocate a plain buffer (helper for one-off transfers).
pub fn buffer(len: usize) -> Buffer {
    Buffer::new(len)
}

#[cfg(test)]
mod tests {
    use super::is_supported;

    #[test]
    fn supported_ids() {
        assert!(is_supported(0x0411, 0x0199));
        // The successor models are listed too, as unverified models.
        assert!(is_supported(0x0411, 0x01FF));
        assert!(is_supported(0x0411, 0x0200));
        assert!(is_supported(0x0411, 0x0202));
        assert!(is_supported(0x0411, 0x0218));
        assert!(!is_supported(0x0411, 0x01D3));
        assert!(!is_supported(0x0412, 0x0199));
    }
}
