//! `nusb`-based transport for [`dth30_core`]: the PLX NET2280 in PCI host mode.
//!
//! It reproduces, byte for byte, what the vendor's USB layer sends according
//! to the static analysis in docs/26 (§2-§6, §9 of the NET2280 report); the
//! packet layouts are unit tested ([`packet`]). On a DT-H50/U2 the register
//! accesses, the bulk OUT DMA and the interrupt endpoint work; the TS
//! endpoints have not carried data yet.
//!
//! What goes over the bus:
//!
//! * Enumeration: the first configuration is selected (only when another one
//!   is active) and every interface of it is claimed, alternate setting 0, as
//!   the vendor stack selects them all. The DT-H50/U2 has three: interface 0
//!   with bulk 0x01 / 0x02 / 0x03 / 0x81 (unused), interface 1 with bulk OUT
//!   0x04 and the TS endpoints 0x82 / 0x83 / 0x84, and interface 2 with the
//!   register endpoints 0x0D / 0x8D / 0x0E / 0x8E and the status endpoint
//!   0x8F. Each endpoint is opened on the interface that has it. No vendor
//!   control transfer, no reset, no `CLEAR_FEATURE`: the vendor stack sends
//!   none either. The register endpoints must exist, as the vendor stack
//!   requires.
//! * Register accesses ([`Net2280Usb::local_read`] and friends,
//!   [`dth30_core::bus::RemotePci`]): a 10-byte OUT packet for a write, a
//!   6-byte OUT packet followed by a 4-byte IN reply for a read. One mutex
//!   serialises every access on both endpoint pairs, so a read's OUT and IN
//!   stay together. Each access has a 1 s budget (a read: both halves
//!   together); on timeout the outstanding transfers are cancelled and
//!   [`Error::Timeout`] is returned. There are no retries. The vendor stack
//!   reports a failed read as 0xFFFFFFFF; here it is an `Err`.
//! * IN transfers ask for whole max-size packets (nusb requires it): the
//!   device ends them with its short 4-byte packet, so the bus traffic is the
//!   same as the vendor's 4-byte requests.
//! * Bulk OUT ([`RemotePci::bulk_out`]): sent in chunks of at most 0x40000
//!   bytes, one after the other, without a terminating zero-length packet.
//! * Interrupt status (EP 0x8F): one 4-byte read, through the endpoint's
//!   transfer type from the descriptor (interrupt or bulk).
//! * TS ([`StreamReader`]): bulk IN 0x82 / 0x83 / 0x84 with several transfers
//!   in flight.
//!
//! Platforms:
//!
//! * Windows: the device must be bound to **WinUSB** (e.g. with Zadig /
//!   libwdi, VID 0x0411). Leave WinUSB's auto-suspend off; the vendor stack
//!   never idles the device.
//! * Linux: usbfs (`/dev/bus/usb/BBB/DDD`). Devices are normally found through
//!   sysfs; without sysfs (a chroot with only `/dev/bus/usb`) the device nodes
//!   are scanned directly ([`Net2280Usb::list_all`]) or a node is opened by
//!   path (`Net2280Usb::open_path`).
//! * macOS: IOKit, through nusb's own enumeration.

use core::time::Duration;
use std::sync::Mutex;
use std::time::Instant;

use dth_common::LockExt;
use dth30_core::bus::RemotePci;
use dth30_core::{Error, Result, model};
use nusb::MaybeFuture;
use nusb::descriptors::TransferType;
use nusb::transfer::{Buffer, Bulk, Completion, EndpointDirection, In, Interrupt, Out, TransferError};
use nusb::{Endpoint, Interface};

pub mod packet;

pub use nusb;
pub use packet::{LocalSpace, PciSpace};

/// NET2280 local register writes and read requests (CFGOUT).
pub const EP_CFGOUT: u8 = 0x0D;
/// NET2280 local register read replies (CFGIN).
pub const EP_CFGIN: u8 = 0x8D;
/// Remote PCI writes and read requests (PCIOUT).
pub const EP_PCIOUT: u8 = 0x0E;
/// Remote PCI read replies (PCIIN).
pub const EP_PCIIN: u8 = 0x8E;
/// Interrupt status (STATIN): IRQSTAT1, 4 bytes LE.
pub const EP_STATIN: u8 = 0x8F;
/// Programmable endpoint A on the BUFFALO products: bulk OUT into the
/// NET2280 DMA engine (host to ViXS memory).
pub const EP_A_OUT: u8 = 0x04;
/// Programmable endpoints B / C / D on the BUFFALO products: bulk IN from
/// FIFO B / C / D (TS).
pub const EP_B_IN: u8 = 0x82;
pub const EP_C_IN: u8 = 0x83;
pub const EP_D_IN: u8 = 0x84;

/// The endpoints the vendor stack refuses to start without.
pub const REQUIRED_ENDPOINTS: [u8; 4] = [EP_CFGOUT, EP_CFGIN, EP_PCIOUT, EP_PCIIN];

/// Budget of one register access (a read: OUT and IN together).
pub const REGISTER_TIMEOUT: Duration = Duration::from_secs(1);
/// How long a cancelled transfer is waited for.
pub const CANCEL_WAIT: Duration = Duration::from_secs(2);
/// Budget of one bulk OUT chunk (0x40000 bytes). The vendor stack has no
/// timeout here; this one only keeps a dead device from blocking forever.
pub const BULK_CHUNK_TIMEOUT: Duration = Duration::from_secs(5);

fn map_err(e: TransferError) -> Error {
    match e {
        // A transfer is only cancelled here when its time is up.
        TransferError::Cancelled => Error::Timeout,
        other => Error::Usb(other.to_string()),
    }
}

fn map_io(e: nusb::Error) -> Error {
    Error::Usb(e.to_string())
}

/// One endpoint of the claimed interfaces as its descriptor gives it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EndpointInfo {
    /// The interface that has it.
    pub interface: u8,
    pub address: u8,
    pub transfer_type: TransferType,
    pub max_packet_size: usize,
}

fn endpoint_list(ifaces: &[Interface]) -> Vec<EndpointInfo> {
    ifaces
        .iter()
        .filter_map(|i| i.descriptor().map(|d| (i.interface_number(), d)))
        .flat_map(|(n, d)| {
            d.endpoints()
                .map(|e| EndpointInfo {
                    interface: n,
                    address: e.address(),
                    transfer_type: e.transfer_type(),
                    max_packet_size: e.max_packet_size(),
                })
                .collect::<Vec<_>>()
        })
        .collect()
}

/// The required endpoints (see [`REQUIRED_ENDPOINTS`]) missing from `present`.
pub fn missing_endpoints(present: &[u8]) -> Vec<u8> {
    REQUIRED_ENDPOINTS.iter().copied().filter(|a| !present.contains(a)).collect()
}

/// A transfer length must be a nonzero multiple of the endpoint's max packet size.
pub fn check_transfer_len(len: usize, max_packet_size: usize) -> Result<()> {
    if max_packet_size == 0 || len == 0 || !len.is_multiple_of(max_packet_size) {
        return Err(Error::Protocol("transfer length must be a nonzero multiple of the max packet size"));
    }
    Ok(())
}

/// A bulk or interrupt endpoint, whichever the descriptor says it is (the
/// vendor stack uses one URB type for both).
enum Pipe<D: EndpointDirection> {
    Bulk(Endpoint<Bulk, D>),
    Interrupt(Endpoint<Interrupt, D>),
}

macro_rules! each {
    ($pipe:expr, $ep:ident => $e:expr) => {
        match $pipe {
            Pipe::Bulk($ep) => $e,
            Pipe::Interrupt($ep) => $e,
        }
    };
}

impl<D: EndpointDirection> Pipe<D> {
    /// Open `addr` on whichever of `ifaces` has it.
    fn open(ifaces: &[Interface], addr: u8) -> Result<Self> {
        let found = ifaces.iter().find_map(|i| {
            i.descriptor().and_then(|d| d.endpoints().find(|e| e.address() == addr).map(|e| (i, e.transfer_type())))
        });
        let Some((iface, ty)) = found else {
            return Err(Error::Unsupported(format!("endpoint {addr:#04x} is on none of the claimed interfaces")));
        };
        match ty {
            TransferType::Bulk => iface.endpoint::<Bulk, D>(addr).map(Pipe::Bulk).map_err(map_io),
            TransferType::Interrupt => iface.endpoint::<Interrupt, D>(addr).map(Pipe::Interrupt).map_err(map_io),
            t => Err(Error::Unsupported(format!("endpoint {addr:#04x} is {t:?}, not bulk or interrupt"))),
        }
    }

    fn address(&self) -> u8 {
        each!(self, ep => ep.endpoint_address())
    }

    fn max_packet_size(&self) -> usize {
        each!(self, ep => ep.max_packet_size())
    }

    fn pending(&self) -> usize {
        each!(self, ep => ep.pending())
    }

    fn submit(&mut self, buf: Buffer) {
        each!(self, ep => ep.submit(buf))
    }

    fn wait_next_complete(&mut self, timeout: Duration) -> Option<Completion> {
        each!(self, ep => ep.wait_next_complete(timeout))
    }

    fn cancel_all(&mut self) {
        each!(self, ep => ep.cancel_all())
    }

    fn clear_halt(&mut self) -> Result<()> {
        each!(self, ep => ep.clear_halt().wait().map_err(map_io))
    }

    /// Cancel whatever is in flight and wait (up to [`CANCEL_WAIT`]) for it.
    fn cancel_and_drain(&mut self) {
        self.cancel_all();
        let end = Instant::now() + CANCEL_WAIT;
        while self.pending() > 0 {
            let left = end.saturating_duration_since(Instant::now());
            if left.is_zero() || self.wait_next_complete(left).is_none() {
                break;
            }
        }
    }

    /// Make sure no transfer from an earlier, failed access is still around.
    fn settle(&mut self) -> Result<()> {
        if self.pending() > 0 {
            self.cancel_and_drain();
            if self.pending() > 0 {
                return Err(Error::Usb(format!(
                    "endpoint {:#04x}: a cancelled transfer did not return",
                    self.address()
                )));
            }
        }
        Ok(())
    }

    /// Submit one transfer and wait for it until `deadline`; on timeout it is
    /// cancelled and [`Error::Timeout`] returned.
    fn run(&mut self, buf: Buffer, deadline: Instant) -> Result<Completion> {
        self.settle()?;
        self.submit(buf);
        match self.wait_next_complete(deadline.saturating_duration_since(Instant::now())) {
            Some(c) => match c.status {
                Ok(()) => Ok(c),
                Err(e) => Err(map_err(e)),
            },
            None => {
                self.cancel_and_drain();
                Err(Error::Timeout)
            }
        }
    }
}

/// The register endpoints, all behind the one register lock.
struct RegPipes {
    local_out: Pipe<Out>,
    local_in: Pipe<In>,
    remote_out: Pipe<Out>,
    remote_in: Pipe<In>,
}

fn write_packet(out: &mut Pipe<Out>, pkt: &[u8], timeout: Duration) -> Result<()> {
    let c = out.run(Buffer::from(pkt), Instant::now() + timeout)?;
    if c.actual_len != pkt.len() {
        return Err(Error::ShortTransfer { expected: pkt.len(), actual: c.actual_len });
    }
    Ok(())
}

/// The OUT half, then (only when it went through) the IN half, within one
/// `timeout` for both.
fn read_packet(out: &mut Pipe<Out>, inp: &mut Pipe<In>, pkt: &[u8], timeout: Duration) -> Result<u32> {
    let deadline = Instant::now() + timeout;
    inp.settle()?;
    let c = out.run(Buffer::from(pkt), deadline)?;
    if c.actual_len != pkt.len() {
        return Err(Error::ShortTransfer { expected: pkt.len(), actual: c.actual_len });
    }
    let len = packet::round_up_to_packets(packet::REPLY_LEN, inp.max_packet_size());
    let c = inp.run(Buffer::new(len), deadline)?;
    if c.actual_len > packet::REPLY_LEN {
        log::warn!("register reply of {} bytes on {:#04x}, using the first 4", c.actual_len, inp.address());
    }
    packet::decode_reply(&c.buffer[..c.actual_len.min(c.buffer.len())])
}

/// A supported device found by [`Net2280Usb::list_all`]: through the OS
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
    /// The product name (or the product ID for an unknown one).
    pub fn product_label(&self) -> String {
        model::product_label(self.product_id)
    }

    /// Open this device and claim interface 0.
    pub fn open(&self) -> Result<Net2280Usb> {
        if let Some(info) = self.info.as_ref() {
            return Net2280Usb::open(info);
        }
        #[cfg(any(target_os = "linux", target_os = "android"))]
        if let Some(p) = self.path.as_ref() {
            return Net2280Usb::open_path(p);
        }
        Err(Error::Usb("device entry has neither OS info nor a device node".into()))
    }
}

/// An opened DTH30 series device (every interface claimed, register
/// endpoints open). Implements [`RemotePci`]; methods take `&self`, so one
/// handle can be shared (e.g. in an `Arc`) by the control path and a reader.
pub struct Net2280Usb {
    device: nusb::Device,
    ifaces: Vec<Interface>,
    serial: Option<String>,
    product_id: u16,
    endpoints: Vec<EndpointInfo>,
    timeout: Duration,
    /// Serialises every register access (local and remote), like the vendor
    /// stack's single lock.
    regs: Mutex<RegPipes>,
    /// EP 0x8F, opened on first use.
    status: Mutex<Option<Pipe<In>>>,
    /// Bulk OUT endpoints, opened on first use.
    outs: Mutex<Vec<Pipe<Out>>>,
}

impl Net2280Usb {
    /// Enumerate supported devices through the OS (sysfs on Linux).
    pub fn list() -> Result<Vec<nusb::DeviceInfo>> {
        let it = nusb::list_devices().wait().map_err(map_io)?;
        Ok(it.filter(|d| model::is_supported(d.vendor_id(), d.product_id())).collect())
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
    pub fn open(info: &nusb::DeviceInfo) -> Result<Self> {
        let device = info.open().wait().map_err(map_io)?;
        Self::from_device(device, info.serial_number().map(str::to_owned))
    }

    /// Open the usbfs node `path` (`/dev/bus/usb/BBB/DDD`) directly, for hosts
    /// without sysfs. The node must be a supported device.
    #[cfg(any(target_os = "linux", target_os = "android"))]
    pub fn open_path(path: &std::path::Path) -> Result<Self> {
        let device = linux::device_from_path(path)?;
        let desc = device.device_descriptor();
        if !model::is_supported(desc.vendor_id(), desc.product_id()) {
            return Err(Error::Usb(format!(
                "{} is {:04x}:{:04x}, not a supported device",
                path.display(),
                desc.vendor_id(),
                desc.product_id()
            )));
        }
        let serial = linux::read_serial(&device);
        Self::from_device(device, serial)
    }

    /// Open the first supported device found.
    pub fn open_first() -> Result<Self> {
        Self::open_matching(None)
    }

    /// Open the supported device whose USB serial number equals `serial`
    /// (the first device when `serial` is `None`).
    pub fn open_matching(serial: Option<&str>) -> Result<Self> {
        let list = Self::list_all()?;
        let found = match serial {
            Some(sn) => list
                .iter()
                .find(|d| d.serial.as_deref().is_some_and(|s| s.eq_ignore_ascii_case(sn)))
                .ok_or_else(|| Error::Usb(format!("no supported device with serial {sn}")))?,
            None => list.first().ok_or_else(|| Error::Usb("no supported device found".into()))?,
        };
        found.open()
    }

    fn from_device(device: nusb::Device, serial: Option<String>) -> Result<Self> {
        // The vendor stack selects the first configuration descriptor's value.
        // Re-selecting the active configuration resets the endpoints, so only
        // do it when another one is active (usbfs leaves the kernel's choice).
        let wanted = device.configurations().next().map(|c| c.configuration_value()).unwrap_or(1);
        if device.active_configuration().map(|c| c.configuration_value()).ok() != Some(wanted)
            && let Err(e) = device.set_configuration(wanted).wait()
        {
            log::debug!("set_configuration({wanted}): {e}");
        }
        let product_id = device.device_descriptor().product_id();
        // Detaching is a no-op on Windows; on Linux it frees an interface from
        // a kernel driver that may have bound to it.
        let numbers: Vec<u8> = match device.active_configuration() {
            Ok(c) => c.interfaces().map(|g| g.interface_number()).collect(),
            Err(e) => {
                log::debug!("active configuration: {e}");
                vec![0]
            }
        };
        let mut ifaces = Vec::with_capacity(numbers.len());
        for n in numbers {
            ifaces.push(device.detach_and_claim_interface(n).wait().map_err(map_io)?);
        }
        let endpoints = endpoint_list(&ifaces);
        let present: Vec<u8> = endpoints.iter().map(|e| e.address).collect();
        let missing = missing_endpoints(&present);
        if !missing.is_empty() {
            return Err(Error::Unsupported(format!(
                "the device lacks the NET2280 register endpoints {} (has {})",
                hex_list(&missing),
                hex_list(&present)
            )));
        }
        for e in &endpoints {
            log::debug!(
                "interface {} endpoint {:#04x}: {:?}, max packet {}",
                e.interface,
                e.address,
                e.transfer_type,
                e.max_packet_size
            );
        }
        for addr in [EP_STATIN, EP_A_OUT, EP_B_IN, EP_C_IN, EP_D_IN] {
            if !present.contains(&addr) {
                log::info!("endpoint {addr:#04x} is not in the descriptor");
            }
        }
        let regs = RegPipes {
            local_out: Pipe::open(&ifaces, EP_CFGOUT)?,
            local_in: Pipe::open(&ifaces, EP_CFGIN)?,
            remote_out: Pipe::open(&ifaces, EP_PCIOUT)?,
            remote_in: Pipe::open(&ifaces, EP_PCIIN)?,
        };
        Ok(Net2280Usb {
            device,
            ifaces,
            serial,
            product_id,
            endpoints,
            timeout: REGISTER_TIMEOUT,
            regs: Mutex::new(regs),
            status: Mutex::new(None),
            outs: Mutex::new(Vec::new()),
        })
    }

    pub fn device(&self) -> &nusb::Device {
        &self.device
    }

    /// The claimed interfaces, in descriptor order.
    pub fn interfaces(&self) -> &[Interface] {
        &self.ifaces
    }

    /// The device's USB serial number string, when it has one.
    pub fn serial(&self) -> Option<&str> {
        self.serial.as_deref()
    }

    /// The device's USB product ID.
    pub fn product_id(&self) -> u16 {
        self.product_id
    }

    /// The product name (or the product ID for an unknown one).
    pub fn product_label(&self) -> String {
        model::product_label(self.product_id)
    }

    /// The endpoints of every claimed interface (alternate setting 0).
    pub fn endpoints(&self) -> &[EndpointInfo] {
        &self.endpoints
    }

    /// The endpoint with address `addr`, if a claimed interface has it.
    pub fn endpoint(&self, addr: u8) -> Option<EndpointInfo> {
        self.endpoints.iter().copied().find(|e| e.address == addr)
    }

    /// Read a NET2280 local register (OUT 6 bytes on 0x0D, IN 4 bytes on 0x8D).
    pub fn local_read(&self, space: LocalSpace, offset: u16) -> Result<u32> {
        let pkt = packet::encode_local_read(space, offset);
        let mut p = self.regs.locked();
        let p = &mut *p;
        let v = read_packet(&mut p.local_out, &mut p.local_in, &pkt, self.timeout);
        log::trace!("local {space:?} read {offset:#05x} -> {v:x?}");
        v
    }

    /// Write a NET2280 local register (OUT 10 bytes on 0x0D).
    pub fn local_write(&self, space: LocalSpace, offset: u16, value: u32) -> Result<()> {
        let pkt = packet::encode_local_write(space, offset, value);
        log::trace!("local {space:?} write {offset:#05x} = {value:#010x}");
        write_packet(&mut self.regs.locked().local_out, &pkt, self.timeout)
    }

    /// A remote PCI read with an explicit control word ([`packet::remote_ctl`];
    /// 0x4000 is added): OUT 6 bytes on 0x0E, IN 4 bytes on 0x8E. Returns the
    /// whole reply dword.
    pub fn remote_read_raw(&self, word0: u16, addr: u32) -> Result<u32> {
        let pkt = packet::encode_remote_read(word0, addr);
        let mut p = self.regs.locked();
        let p = &mut *p;
        let v = read_packet(&mut p.remote_out, &mut p.remote_in, &pkt, self.timeout);
        log::trace!("remote {word0:#06x} read {addr:#010x} -> {v:x?}");
        v
    }

    /// A remote PCI write with an explicit control word: OUT 10 bytes on 0x0E.
    pub fn remote_write_raw(&self, word0: u16, addr: u32, value: u32) -> Result<()> {
        let pkt = packet::encode_remote_write(word0, addr, value);
        log::trace!("remote {word0:#06x} write {addr:#010x} = {value:#010x}");
        write_packet(&mut self.regs.locked().remote_out, &pkt, self.timeout)
    }

    /// 32-bit remote read (all byte enables).
    pub fn remote_read(&self, space: PciSpace, addr: u32) -> Result<u32> {
        self.remote_read_raw(packet::remote_ctl(space, true, packet::BE_ALL), addr)
    }

    /// 32-bit remote write (all byte enables).
    pub fn remote_write(&self, space: PciSpace, addr: u32, value: u32) -> Result<()> {
        self.remote_write_raw(packet::remote_ctl(space, false, packet::BE_ALL), addr, value)
    }

    /// 16-bit remote read: byte enables 0x3 / 0xC by address bit 1, the
    /// address sent unmasked, the matching half of the reply returned.
    pub fn remote_read16(&self, space: PciSpace, addr: u32) -> Result<u16> {
        let (be, shift) = packet::word_lanes(addr);
        Ok((self.remote_read_raw(packet::remote_ctl(space, true, be), addr)? >> shift) as u16)
    }

    /// 16-bit remote write, the data placed in its byte lanes.
    pub fn remote_write16(&self, space: PciSpace, addr: u32, value: u16) -> Result<()> {
        let (be, shift) = packet::word_lanes(addr);
        self.remote_write_raw(packet::remote_ctl(space, false, be), addr, (value as u32) << shift)
    }

    /// 8-bit remote read: byte enable `1 << (addr & 3)`.
    pub fn remote_read8(&self, space: PciSpace, addr: u32) -> Result<u8> {
        let (be, shift) = packet::byte_lanes(addr);
        Ok((self.remote_read_raw(packet::remote_ctl(space, true, be), addr)? >> shift) as u8)
    }

    /// 8-bit remote write, the data placed in its byte lane.
    pub fn remote_write8(&self, space: PciSpace, addr: u32, value: u8) -> Result<()> {
        let (be, shift) = packet::byte_lanes(addr);
        self.remote_write_raw(packet::remote_ctl(space, false, be), addr, (value as u32) << shift)
    }

    /// I/O read cycle on the remote bus (control 0x407F).
    pub fn io_read(&self, addr: u32) -> Result<u32> {
        self.remote_read(PciSpace::Io, addr)
    }

    /// I/O write cycle on the remote bus (control 0x406F).
    pub fn io_write(&self, addr: u32, value: u32) -> Result<()> {
        self.remote_write(PciSpace::Io, addr, value)
    }

    /// Cancel a pending interrupt-status read (see [`RemotePci::interrupt_status`]).
    pub fn cancel_interrupt(&self) {
        if let Some(p) = self.status.locked().as_mut() {
            p.cancel_and_drain();
        }
    }

    /// A reader for a TS endpoint (0x82 / 0x83 / 0x84), `depth` transfers of
    /// `transfer_len` bytes in flight ([`StreamReader::DEFAULT_TRANSFER_LEN`] =
    /// 256 TS packets is what the vendor reads). The endpoint stays claimed
    /// until the reader is dropped.
    pub fn stream_reader(&self, addr: u8, transfer_len: usize, depth: usize) -> Result<StreamReader> {
        if addr & 0x80 == 0 {
            return Err(Error::Protocol("stream endpoint must be an IN endpoint"));
        }
        if addr == EP_CFGIN || addr == EP_PCIIN || addr == EP_STATIN {
            return Err(Error::Protocol("register and status endpoints do not carry a stream"));
        }
        StreamReader::new(Pipe::open(&self.ifaces, addr)?, transfer_len, depth)
    }
}

fn hex_list(addrs: &[u8]) -> String {
    addrs.iter().map(|a| format!("{a:#04x}")).collect::<Vec<_>>().join(" ")
}

impl RemotePci for Net2280Usb {
    fn csr_read(&self, offset: u16) -> Result<u32> {
        self.local_read(LocalSpace::Csr, offset)
    }

    fn csr_write(&self, offset: u16, value: u32) -> Result<()> {
        self.local_write(LocalSpace::Csr, offset, value)
    }

    fn local_cfg_read(&self, offset: u16) -> Result<u32> {
        self.local_read(LocalSpace::Config, offset)
    }

    fn local_cfg_write(&self, offset: u16, value: u32) -> Result<()> {
        self.local_write(LocalSpace::Config, offset, value)
    }

    fn cfg_read(&self, addr: u32) -> Result<u32> {
        self.remote_read(PciSpace::Config, addr)
    }

    fn cfg_write(&self, addr: u32, value: u32) -> Result<()> {
        self.remote_write(PciSpace::Config, addr, value)
    }

    fn mem_read(&self, addr: u32) -> Result<u32> {
        self.remote_read(PciSpace::Memory, addr)
    }

    fn mem_write(&self, addr: u32, value: u32) -> Result<()> {
        self.remote_write(PciSpace::Memory, addr, value)
    }

    /// Chunks of at most 0x40000 bytes, sent one after the other; a chunk
    /// that is not fully sent ends the transfer. Not under the register lock
    /// (the vendor stack does not take it for bulk data either).
    fn bulk_out(&self, endpoint: u8, data: &[u8]) -> Result<()> {
        if endpoint & 0x80 != 0 {
            return Err(Error::Protocol("bulk_out needs an OUT endpoint"));
        }
        if endpoint == EP_CFGOUT || endpoint == EP_PCIOUT {
            return Err(Error::Protocol("register endpoints do not take bulk data"));
        }
        let mut outs = self.outs.locked();
        let idx = match outs.iter().position(|p| p.address() == endpoint) {
            Some(i) => i,
            None => {
                outs.push(Pipe::open(&self.ifaces, endpoint)?);
                outs.len() - 1
            }
        };
        let pipe = &mut outs[idx];
        let mut sent = 0;
        for len in packet::bulk_chunks(data.len()) {
            let c = pipe.run(Buffer::from(&data[sent..sent + len]), Instant::now() + BULK_CHUNK_TIMEOUT)?;
            if c.actual_len != len {
                return Err(Error::ShortTransfer { expected: len, actual: c.actual_len });
            }
            sent += len;
        }
        log::trace!("bulk out {endpoint:#04x}: {sent} bytes");
        Ok(())
    }

    /// One read of the 4-byte status word on EP 0x8F. A read that times out
    /// stays pending and is picked up by the next call, so no status word is
    /// lost ([`Net2280Usb::cancel_interrupt`] drops it).
    fn interrupt_status(&self, timeout: Duration) -> Result<Option<u32>> {
        let mut st = self.status.locked();
        if st.is_none() {
            *st = Some(Pipe::open(&self.ifaces, EP_STATIN)?);
        }
        let Some(pipe) = st.as_mut() else { return Err(Error::Protocol("status endpoint not open")) };
        if pipe.pending() == 0 {
            pipe.submit(Buffer::new(packet::round_up_to_packets(packet::REPLY_LEN, pipe.max_packet_size())));
        }
        let Some(c) = pipe.wait_next_complete(timeout) else { return Ok(None) };
        match c.status {
            Ok(()) => packet::decode_reply(&c.buffer[..c.actual_len.min(c.buffer.len())]).map(Some),
            Err(TransferError::Cancelled) => Ok(None),
            Err(e) => Err(map_err(e)),
        }
    }
}

/// Continuous reader over a TS endpoint (bulk IN 0x82 / 0x83 / 0x84), with
/// several transfers in flight (the vendor stack keeps one; more is safer in
/// user mode). Ends of transfers are wherever the device sends a short packet.
///
/// Transfer buffers are plain heap buffers ([`Buffer::new`]), not nusb's
/// zero-copy ones (which misbehaved on an older 32-bit ARM kernel with the
/// DTH10 series).
pub struct StreamReader {
    pipe: Pipe<In>,
    transfer_len: usize,
    depth: usize,
}

impl StreamReader {
    /// 256 TS packets (256 × 188 = 94 × 512), the vendor's read size.
    pub const DEFAULT_TRANSFER_LEN: usize = 48128;
    pub const DEFAULT_DEPTH: usize = 4;

    fn new(pipe: Pipe<In>, transfer_len: usize, depth: usize) -> Result<Self> {
        check_transfer_len(transfer_len, pipe.max_packet_size())?;
        if depth == 0 {
            return Err(Error::Protocol("stream depth must be at least 1"));
        }
        Ok(StreamReader { pipe, transfer_len, depth })
    }

    /// Submit the initial transfers.
    pub fn start(&mut self) {
        while self.pipe.pending() < self.depth {
            self.pipe.submit(Buffer::new(self.transfer_len));
        }
    }

    /// Wait for the next completed transfer and resubmit a buffer in its place.
    /// Returns the received bytes, `Ok(None)` on timeout.
    pub fn next(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>> {
        if self.pipe.pending() == 0 {
            self.start();
        }
        let Some(c) = self.pipe.wait_next_complete(timeout) else { return Ok(None) };
        self.pipe.submit(Buffer::new(self.transfer_len));
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
        self.pipe.cancel_and_drain();
    }

    /// Clear a halt on the endpoint (`CLEAR_FEATURE`). The vendor stack never
    /// sends one; do so only after a stall, with the reader stopped.
    pub fn clear_halt(&mut self) -> Result<()> {
        self.pipe.clear_halt()
    }

    pub fn endpoint_address(&self) -> u8 {
        self.pipe.address()
    }

    pub fn transfer_len(&self) -> usize {
        self.transfer_len
    }

    pub fn depth(&self) -> usize {
        self.depth
    }
}

/// Linux: device nodes without sysfs.
#[cfg(any(target_os = "linux", target_os = "android"))]
mod linux {
    use std::path::{Path, PathBuf};

    use dth30_core::{Error, Result, model};
    use nusb::MaybeFuture;

    use super::{Found, map_io};

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
            if !model::is_supported(desc.vendor_id(), desc.product_id()) {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn supported_ids() {
        for pid in [0x0128, 0x0129, 0x012A, 0x0144] {
            assert!(model::is_supported(0x0411, pid));
        }
        assert!(!model::is_supported(0x0411, 0x0199));
        assert!(!model::is_supported(0x0525, 0x0128));
    }

    #[test]
    fn required_endpoints() {
        let all = [0x0D, 0x8D, 0x0E, 0x8E, 0x8F, 0x04, 0x82, 0x83, 0x84];
        assert!(missing_endpoints(&all).is_empty());
        assert!(missing_endpoints(&[0x0D, 0x8D, 0x0E, 0x8E]).is_empty());
        assert_eq!(missing_endpoints(&[0x0D, 0x0E, 0x8F]), [0x8D, 0x8E]);
        assert_eq!(missing_endpoints(&[]), REQUIRED_ENDPOINTS);
        assert_eq!(hex_list(&[0x0D, 0x8E]), "0x0d 0x8e");
    }

    #[test]
    fn transfer_lengths() {
        assert!(check_transfer_len(StreamReader::DEFAULT_TRANSFER_LEN, 512).is_ok());
        assert!(check_transfer_len(StreamReader::DEFAULT_TRANSFER_LEN, 64).is_ok());
        assert_eq!(StreamReader::DEFAULT_TRANSFER_LEN, 256 * 188);
        assert!(check_transfer_len(48128 + 188, 512).is_err());
        assert!(check_transfer_len(0, 512).is_err());
        assert!(check_transfer_len(512, 0).is_err());
    }

    #[test]
    fn errors() {
        std::assert_matches!(map_err(TransferError::Cancelled), Error::Timeout);
        std::assert_matches!(map_err(TransferError::Stall), Error::Usb(_));
    }

    #[test]
    fn shareable() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<Net2280Usb>();
        fn assert_send<T: Send>() {}
        assert_send::<StreamReader>();
        // Usable behind the core's trait object.
        fn as_dyn(u: &Net2280Usb) -> &dyn RemotePci {
            u
        }
        let _ = as_dyn;
    }
}
