//! Stream commands: TS input set-up, the three stream opens, flush and close
//! (docs/26 6-stream-payloads.md).
//!
//! The vendor's user-mode filter starts a channel in this order (§9,
//! 5-usermode.md §3.3):
//!
//! 1. TS input set-up ([`tsd_config_fb_writes`], function 24), while no
//!    stream is open;
//! 2. the CAS object ("generic" stream, function 25,
//!    [`build_open_generic`]); its channel, EP 0x82, is read continuously in
//!    32 KB blocks with a 0x1EC-byte header ([`parse_generic_block`]);
//! 3. the Raw stream (PSI / TS pass-through, function 8 kind 1,
//!    [`build_open_psi`]) on EP 0x83, from which PAT and PMT are read;
//! 4. the Program stream (secured TS, function 8 kind 5,
//!    [`build_open_program`]) on EP 0x84.
//!
//! It stops Program, then Raw (each: flush = function 10, close = function
//! 9 in the OPEN slot, then the IN transfer is cancelled and the NET2280
//! FIFO flushed, [`flush_endpoint_fifo`]), and finally closes the CAS object
//! (function 26, no flush).
//!
//! The payload builders are pure and start from zeroed buffers; the vendor
//! does not clear its slots (§0.3), so the only intended difference is that
//! fields it leaves unwritten are zero here. The firmware overwrites the
//! stream-handle field of an OPEN with its own handle; only that word is
//! read back (§1.2). Which [`CasMode`] yields a clear TS is not known
//! (§7.5).
//!
//! Nothing here is verified on hardware.

use super::Vixs;
use super::cmd::{self, func};
use crate::bus::RemotePci;
use crate::model::KitKind;
use crate::net2280::{Layout, reg};
use crate::{Error, Result};

/// PCI addresses of the NET2280 FIFOs the payloads point the firmware at.
pub const FIFO_B: u32 = 0x3004_0000;
pub const FIFO_C: u32 = 0x3008_0000;
pub const FIFO_D: u32 = 0x300C_0000;

/// Payload lengths.
pub const GENERIC_OPEN_LEN: usize = 0x530;
pub const OPEN_LEN: usize = 0x1198;
pub const FLUSH_LEN: usize = 0x32C;
pub const TSD_CONFIG_LEN: usize = 4;

/// Buffers per buffer table.
pub const BUFFERS: u32 = 32;
/// TS transfer and buffer size: 256 packets of 188 bytes.
pub const TS_CHUNK: u32 = 0xBC00;
/// Generic (CAS) channel block size.
pub const GENERIC_BLOCK: u32 = 0x8000;
/// Header length of a generic block.
pub const GENERIC_HEADER_LEN: usize = 0x1EC;
/// Size of the host-to-firmware input buffers in the generic open.
const GENERIC_VIDEO_IN: u32 = 0x4_0000;

/// Where the firmware leaves its handle in the OPEN payload.
pub const GENERIC_HANDLE_OFFSET: u32 = 0x00;
pub const STREAM_HANDLE_OFFSET: u32 = 0x18;

/// Stream kinds of function 8 (payload +0x58).
pub const STREAM_KIND_PSI: u32 = 1;
pub const STREAM_KIND_SECURED_TS: u32 = 5;

/// Most elementary streams a Program open carries.
pub const MAX_ES: usize = 24;
/// Most AAC streams a Program open carries.
pub const MAX_AAC: usize = 2;
/// The video PID the vendor sends when the service has no MPEG-1/2 video.
pub const NO_VIDEO_PID: u16 = 0xFFFF;

/// The CAS set-up, a pair of the generic open's mode (+0x08) and the Program
/// open's secured mode (+0xF2C). The vendor's filter always uses
/// [`CasMode::BcasDrm`]; the kernel and the filter's own builder also accept
/// [`CasMode::Bcas`] (§0.5, §7.3). Which one gives a clear TS has to be
/// tried on hardware (§7.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CasMode {
    /// B-CAS with the "DRM" hand-off: generic mode 32, secured mode 16 (the
    /// vendor default).
    #[default]
    BcasDrm,
    /// Plain B-CAS: generic mode 16, secured mode 8 (non-DRM fallback,
    /// never used by the vendor).
    Bcas,
}

impl CasMode {
    /// The generic open's mode (payload +0x08).
    pub const fn generic_mode(self) -> u32 {
        match self {
            CasMode::BcasDrm => 32,
            CasMode::Bcas => 16,
        }
    }

    /// The Program open's secured mode (payload +0xF2C).
    pub const fn secured_mode(self) -> u32 {
        match self {
            CasMode::BcasDrm => 16,
            CasMode::Bcas => 8,
        }
    }
}

/// The three streams of the vendor's filter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StreamKind {
    /// The CAS object (generic stream, function 25 / 26), FIFO B.
    Generic,
    /// PSI / TS pass-through (function 8 kind 1), FIFO C.
    Raw,
    /// Secured TS of one service (function 8 kind 5), FIFO D.
    Program,
}

impl StreamKind {
    /// Index of the stream's FIFO in [`Layout::endpoints`] / [`Layout::fifo`]
    /// (1 = B, 2 = C, 3 = D).
    pub const fn fifo_index(self) -> usize {
        match self {
            StreamKind::Generic => 1,
            StreamKind::Raw => 2,
            StreamKind::Program => 3,
        }
    }

    /// The bulk IN endpoint on the BUFFALO products (0x82 / 0x83 / 0x84).
    pub const fn endpoint(self) -> u8 {
        match self {
            StreamKind::Generic => 0x82,
            StreamKind::Raw => 0x83,
            StreamKind::Program => 0x84,
        }
    }

    /// The vendor's IN transfer length on the stream's endpoint.
    pub const fn transfer_len(self) -> usize {
        match self {
            StreamKind::Generic => GENERIC_BLOCK as usize,
            StreamKind::Raw | StreamKind::Program => TS_CHUNK as usize,
        }
    }

    /// Bytes of header at the start of each received block (only the
    /// generic channel has one; TS data comes without).
    pub const fn header_len(self) -> usize {
        match self {
            StreamKind::Generic => GENERIC_HEADER_LEN,
            StreamKind::Raw | StreamKind::Program => 0,
        }
    }

    /// Where the firmware leaves its handle in the OPEN payload.
    pub const fn handle_offset(self) -> u32 {
        match self {
            StreamKind::Generic => GENERIC_HANDLE_OFFSET,
            StreamKind::Raw | StreamKind::Program => STREAM_HANDLE_OFFSET,
        }
    }

    const fn name(self) -> &'static str {
        match self {
            StreamKind::Generic => "CAS",
            StreamKind::Raw => "Raw",
            StreamKind::Program => "Program",
        }
    }
}

// ---- payload builders ----

fn put(b: &mut [u8], off: usize, v: u32) {
    b[off..off + 4].copy_from_slice(&v.to_le_bytes());
}

/// A buffer table: count, size, then `BUFFERS` addresses.
fn put_table(b: &mut [u8], off: usize, size: u32, addr: impl Fn(u32) -> u32) {
    put(b, off, BUFFERS);
    put(b, off + 4, size);
    for i in 0..BUFFERS {
        put(b, off + 8 + 4 * i as usize, addr(i));
    }
}

/// Stand-in for the host DMA buffer addresses the PCI version passes
/// (meaningless over USB: the firmware cannot reach host memory). Non-zero
/// and distinct per table and index (§9).
pub const fn fake_host_phys(table: u8, i: u32) -> u32 {
    0x0100_0000 | (table as u32) << 16 | i << 12
}

/// The FB writes of the TS input set-up and the DVO set-up, done before
/// function 24 (§5.1, 2-vixs-bringup.md §5.2). Only the 204-byte packet mask
/// differs between boards; everything else is the vendor's constant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TsdConfig {
    /// `packet_204_size_mask`: 3 on TOSHIBA kits (204-byte packets), 0 on
    /// SONY kits.
    pub packet_204_mask: u32,
}

impl TsdConfig {
    /// The vendor filter's values for a kit.
    pub const fn for_kit(kit: KitKind) -> Self {
        Self { packet_204_mask: if kit.ts_packet_204() { 3 } else { 0 } }
    }
}

/// `(FB offset, value)` in the vendor's order.
pub fn tsd_config_fb_writes(c: &TsdConfig) -> [(u32, u32); 14] {
    [
        (0x50, 1),    // TS input set-up valid
        (0x54, 0),    // sync on channel mask
        (0x58, 3),    // enable on channel mask
        (0x5C, 3),    // active valid high mask
        (0x60, 3),    // negative edge mask
        (0x64, 0xFF), // serial input mask
        (0x68, c.packet_204_mask),
        (0x6C, 1), // DVO set-up valid
        (0x70, 1), // data mode
        (0x74, 0), // clock invert
        (0x78, 0), // increment disable
        (0x7C, 0), // valid high
        (0x80, 1), // clock select
        (0x84, 1), // null packet
    ]
}

/// Function 24's payload: four bytes the vendor does not write.
pub fn build_tsd_config() -> [u8; TSD_CONFIG_LEN] {
    [0; TSD_CONFIG_LEN]
}

/// Function 25: open the CAS object (§2.3). The firmware's handle comes
/// back at +0x00.
pub fn build_open_generic(cookie: u32, mode: CasMode) -> [u8; GENERIC_OPEN_LEN] {
    let mut b = [0u8; GENERIC_OPEN_LEN];
    put(&mut b, 0x000, cookie);
    put(&mut b, 0x008, mode.generic_mode());
    put_table(&mut b, 0x00C, GENERIC_BLOCK, |_| FIFO_B); // generic out
    put_table(&mut b, 0x114, GENERIC_VIDEO_IN, |i| fake_host_phys(0, i)); // input to the firmware
    put_table(&mut b, 0x21C, GENERIC_BLOCK, |_| FIFO_B); // data out
    // +0x328 stream descriptor, +0x428.. two blobs and their lengths
    // (+0x4A8, +0x52C, written as 0 in mode 32): all zero.
    b
}

/// Function 26: close the CAS object. The OPEN payload is sent again with
/// only the first three words changed, as the vendor does (§2.5).
pub fn build_close_generic(open: &[u8; GENERIC_OPEN_LEN], phys: u32) -> [u8; GENERIC_OPEN_LEN] {
    let mut b = *open;
    put(&mut b, 0x000, phys);
    put(&mut b, 0x004, 0);
    put(&mut b, 0x008, 0);
    b
}

/// Function 8, kind 1: open the Raw (PSI / TS pass-through) stream (§3.3).
/// The firmware's handle comes back at +0x18.
pub fn build_open_psi(cookie: u32, ts_port: u32) -> [u8; OPEN_LEN] {
    let mut b = [0u8; OPEN_LEN];
    put(&mut b, 0x008, ts_port);
    put(&mut b, 0x018, cookie);
    put(&mut b, 0x058, STREAM_KIND_PSI);
    put(&mut b, 0x060, 0x0010_0000); // output flags
    put(&mut b, 0x07C, 0xFFFF); // PID filter: all (inferred)
    put(&mut b, 0x080, 0x1010); // meaning unknown
    put_table(&mut b, 0x314, TS_CHUNK, |_| FIFO_C); // video out
    b
}

/// One elementary stream of a Program open.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EsEntry {
    pub pid: u16,
    pub stream_type: u8,
    /// The ECM PID sent with this stream (the vendor sends the
    /// program-level CA descriptor's PID for every stream; 0 when none).
    pub ecm_pid: u16,
}

/// The first MPEG-1/2 video stream's PID (stream type 1 or 2), else
/// [`NO_VIDEO_PID`], as the vendor's filter picks it.
pub fn video_pid(es: &[EsEntry]) -> u16 {
    es.iter().find(|e| matches!(e.stream_type, 1 | 2)).map_or(NO_VIDEO_PID, |e| e.pid)
}

/// The first [`MAX_AAC`] AAC streams' PIDs (stream type 0x0F), 0 where
/// there is none.
pub fn aac_pids(es: &[EsEntry]) -> [u16; MAX_AAC] {
    let mut a = [0; MAX_AAC];
    for (slot, e) in a.iter_mut().zip(es.iter().filter(|e| e.stream_type == 0x0F)) {
        *slot = e.pid;
    }
    a
}

/// The elementary streams that go into a Program open: up to the first
/// PID 0, at most [`MAX_ES`], as the vendor's filter cuts them.
pub fn limit_es(es: &[EsEntry]) -> &[EsEntry] {
    let n = es.iter().take(MAX_ES).take_while(|e| e.pid != 0).count();
    &es[..n]
}

/// What a Program open carries besides the driver's constants (§4.1, §9).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProgramParams<'a> {
    pub ts_port: u32,
    /// The CAS object's firmware handle (0: none, the field stays 0).
    pub cas_phys: u32,
    /// The Raw stream's firmware handle (0: none, the field stays 0).
    pub raw_phys: u32,
    pub sid: u16,
    pub pmt_pid: u16,
    pub pcr_pid: u16,
    /// See [`video_pid`].
    pub video_pid: u16,
    /// AAC PIDs, 0 = none (the count sent is the number of non-zero
    /// entries); see [`aac_pids`].
    pub aac_pids: [u16; MAX_AAC],
    /// Cut by [`limit_es`].
    pub es: &'a [EsEntry],
    /// Selects the secured mode (+0xF2C).
    pub mode: CasMode,
}

impl<'a> ProgramParams<'a> {
    /// Parameters with the video and AAC PIDs picked from `es` the way the
    /// vendor's filter does.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        ts_port: u32,
        cas_phys: u32,
        raw_phys: u32,
        sid: u16,
        pmt_pid: u16,
        pcr_pid: u16,
        es: &'a [EsEntry],
        mode: CasMode,
    ) -> Self {
        let es = limit_es(es);
        let (video_pid, aac_pids) = (video_pid(es), aac_pids(es));
        Self { ts_port, cas_phys, raw_phys, sid, pmt_pid, pcr_pid, video_pid, aac_pids, es, mode }
    }
}

/// Function 8, kind 5: open the Program (secured TS) stream with the flags
/// the vendor's filter uses for pass-through (§4.3). The firmware's handle
/// comes back at +0x18.
pub fn build_open_program(cookie: u32, p: &ProgramParams) -> [u8; OPEN_LEN] {
    let mut b = [0u8; OPEN_LEN];
    let es = limit_es(p.es);
    if es.len() < p.es.len() {
        log::warn!("Program open: {} of {} elementary streams sent", es.len(), p.es.len());
    }
    let n_aac = p.aac_pids.iter().filter(|&&a| a != 0).count() as u32;
    let [aac1, aac2] = p.aac_pids.map(u32::from);
    let video = u32::from(p.video_pid);
    let sid = u32::from(p.sid);
    let pcr = u32::from(p.pcr_pid);

    put(&mut b, 0x008, p.ts_port); // video input port
    put(&mut b, 0x010, p.ts_port); // audio input port
    put(&mut b, 0x018, cookie);
    put(&mut b, 0x020, p.cas_phys);
    put(&mut b, 0x028, p.raw_phys);
    put(&mut b, 0x058, STREAM_KIND_SECURED_TS);
    put(&mut b, 0x060, 0x0010_0060); // output flags: TS video, TS audio
    put(&mut b, 0x068, 0x8000_0002);
    put(&mut b, 0x07C, video);
    put(&mut b, 0x080, video);
    put(&mut b, 0x084, 0x8000_0002);
    put(&mut b, 0x088, 8_000_000); // bit rate
    put(&mut b, 0x090, 720); // width
    put(&mut b, 0x094, 480); // height
    put(&mut b, 0x0C0, 0xFFFF_FFFF);
    put(&mut b, 0x0C4, 0xFFFF_FFFF);
    put(&mut b, 0x0D0, video);
    put_table(&mut b, 0x314, TS_CHUNK, |_| FIFO_D); // video out
    put_table(&mut b, 0x41C, GENERIC_BLOCK, |_| FIFO_B); // audio out
    put_table(&mut b, 0x524, TS_CHUNK, |i| fake_host_phys(1, i)); // video in
    put_table(&mut b, 0x62C, 0, |i| fake_host_phys(2, i)); // audio in, size 0 as the vendor writes it
    put(&mut b, 0xAE4, 0x1_0078);
    put(&mut b, 0xC08, pcr);
    put(&mut b, 0xC28, pcr);
    put(&mut b, 0xC40, u32::from(p.pmt_pid));
    put(&mut b, 0xC68, 500);
    if sid != 0 {
        put(&mut b, 0xC70, sid);
    }
    put(&mut b, 0xEE4, n_aac);
    for base in [0xEE8, 0xEF8] {
        put(&mut b, base, aac1);
        put(&mut b, base + 4, aac2);
    }
    put(&mut b, 0xF0C, 0x1100);
    put(&mut b, 0xF14, 1);
    put(&mut b, 0xF24, sid);
    put(&mut b, 0xF2C, p.mode.secured_mode());
    put(&mut b, 0x1068, es.len() as u32);
    for (i, e) in es.iter().enumerate() {
        put(&mut b, 0x106C + 4 * i, u32::from(e.pid));
        put(&mut b, 0x10CC + 4 * i, u32::from(e.ecm_pid));
        put(&mut b, 0x112C + 4 * i, u32::from(e.stream_type));
    }
    b
}

/// Function 10: the internal flush sent before a stream's close (§6.4).
pub fn build_flush(phys: u32) -> [u8; FLUSH_LEN] {
    let mut b = [0u8; FLUSH_LEN];
    put(&mut b, 0x000, phys);
    put(&mut b, 0x010, 0x8000);
    put(&mut b, 0x2F4, 1);
    b
}

/// Function 9: close a Raw or Program stream. The OPEN payload is sent
/// again with the handle in the first two words (§6.4).
pub fn build_close(open: &[u8; OPEN_LEN], phys: u32) -> [u8; OPEN_LEN] {
    let mut b = *open;
    put(&mut b, 0x000, phys);
    put(&mut b, 0x004, 0);
    b
}

/// Kinds of generic blocks (header +0x30).
pub mod generic_kind {
    pub const GENERIC: u32 = 0x5_0000;
    pub const DEVICE_IO: u32 = 0x8_0000;
    pub const SECODE: u32 = 0x10_0000;
    pub const DRM: u32 = 0x20_0000;
    pub const VT: u32 = 0x40_0000;
}

/// Card state bits of a generic block (header +0x88).
pub mod card_state {
    pub const INSERTED: u32 = 0x2000;
    pub const REMOVED: u32 = 0x4000;
    pub const ACTIVATED: u32 = 0x8000;
    pub const EMM_IDV: u32 = 0x1_0000;
    pub const EMM_COM: u32 = 0x2_0000;
    pub const AUTODISPLAY: u32 = 0x4_0000;
    pub const EMM_COMMAND: u32 = 0x8_0000;
}

/// The header of a block from the generic channel (EP 0x82, §6.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GenericBlockHeader {
    /// +0x30, see [`generic_kind`].
    pub kind: u32,
    /// +0x44.
    pub buf_index: u32,
    /// +0x48: body length (used for device I/O blocks).
    pub size: u32,
    /// +0x60 non-zero: the last frame.
    pub last: bool,
    /// +0x88, see [`card_state`].
    pub card: u32,
}

/// Split a generic block into its header and body; `None` when it is
/// shorter than the header.
pub fn parse_generic_block(b: &[u8]) -> Option<(GenericBlockHeader, &[u8])> {
    if b.len() < GENERIC_HEADER_LEN {
        return None;
    }
    let w = |o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
    let h = GenericBlockHeader { kind: w(0x30), buf_index: w(0x44), size: w(0x48), last: w(0x60) != 0, card: w(0x88) };
    Some((h, &b[GENERIC_HEADER_LEN..]))
}

/// Whether a TS packet is one of the vendor's "CDXM" packets in the Program
/// stream (sync byte, then "CDXM" at bytes 7..11), which a TS consumer
/// should drop (§9).
pub fn is_cdxm_packet(p: &[u8]) -> bool {
    p.len() >= 11 && p[0] == 0x47 && &p[7..11] == b"CDXM"
}

// ---- NET2280 FIFO flush ----

/// EP_STAT bits (NET2280): FIFO flush (write 1) and FIFO empty.
pub const EP_STAT_FIFO_FLUSH: u32 = 1 << 9;
pub const EP_STAT_FIFO_EMPTY: u32 = 1 << 10;
/// EP_AVAIL, the byte count in the FIFO, after EP_STAT in an endpoint block.
pub const EP_AVAIL: u16 = 0x10;

/// The register block of the programmable endpoint serving `ep`.
fn ep_block(layout: &Layout, ep: u8) -> Result<u16> {
    let i = layout.endpoints.iter().position(|&e| e == ep).ok_or(Error::Protocol("not a NET2280 FIFO endpoint"))?;
    Ok(reg::EP_BASE + 0x20 * (i as u16 + 1))
}

/// Empty the NET2280 FIFO of endpoint `ep` after its IN transfer was
/// cancelled, as the vendor does after a close (§6.4, 2-vixs-bringup.md
/// §6.4: EP_STAT / EP_AVAIL, 0x34C / 0x350 for FIFO B, 0x38C / 0x390 for
/// FIFO D): read both, and when the FIFO is not empty write FIFO_FLUSH and
/// read EP_AVAIL again. Returns whether a flush was written. The exact
/// emptiness test of the vendor is inferred.
pub fn flush_endpoint_fifo<B: RemotePci + ?Sized>(bus: &B, layout: &Layout, ep: u8) -> Result<bool> {
    let blk = ep_block(layout, ep)?;
    let stat = bus.csr_read(blk + reg::EP_STAT)?;
    let avail = bus.csr_read(blk + EP_AVAIL)?;
    if avail == 0 && stat & EP_STAT_FIFO_EMPTY != 0 {
        return Ok(false);
    }
    bus.csr_write(blk + reg::EP_STAT, EP_STAT_FIFO_FLUSH)?;
    let left = bus.csr_read(blk + EP_AVAIL)?;
    if left != 0 {
        log::warn!("NET2280 FIFO of endpoint {ep:#04x} still holds {left} bytes after the flush");
    }
    Ok(true)
}

// ---- commands ----

/// An open stream: what its close needs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StreamHandle {
    kind: StreamKind,
    cookie: u32,
    phys: u32,
    slot: u32,
    open_payload: Vec<u8>,
}

impl StreamHandle {
    pub fn kind(&self) -> StreamKind {
        self.kind
    }

    /// The value sent in the stream-handle field of the OPEN (non-zero,
    /// unique per stream; the firmware's data notifications may carry it,
    /// 2-vixs-bringup.md §5.3).
    pub fn cookie(&self) -> u32 {
        self.cookie
    }

    /// The firmware's handle, read back after the OPEN.
    pub fn phys(&self) -> u32 {
        self.phys
    }

    /// The command slot of the OPEN, reserved until the close.
    pub fn slot(&self) -> u32 {
        self.slot
    }

    /// The OPEN payload as sent.
    pub fn open_payload(&self) -> &[u8] {
        &self.open_payload
    }
}

impl<B: RemotePci> Vixs<B> {
    fn new_cookie(&mut self) -> u32 {
        let c = self.next_cookie;
        self.next_cookie = c.wrapping_add(1).max(1);
        c
    }

    fn ring_mut(&mut self) -> Result<&mut cmd::CmdRing> {
        self.ring.as_mut().ok_or(Error::Protocol("ViXS firmware not started"))
    }

    /// The bulk IN endpoint of a stream in this device's layout.
    pub fn stream_endpoint(&self, kind: StreamKind) -> u8 {
        self.layout.endpoints[kind.fifo_index()]
    }

    /// The TS input set-up (§5.1): the FB writes of [`tsd_config_fb_writes`],
    /// then function 24. The firmware refuses it while a stream is open.
    pub fn tsd_config(&mut self, c: &TsdConfig) -> Result<()> {
        for (off, v) in tsd_config_fb_writes(c) {
            self.chip.fb_write_checked(off, v)?;
        }
        let r = self.send_cmd(func::TSD_CONFIG, &build_tsd_config())?;
        if !r.ok() {
            log::error!("TS input set-up failed with status {}", r.status);
            return Err(Error::Protocol("ViXS TS input set-up failed"));
        }
        Ok(())
    }

    /// Send an OPEN in a fresh slot, keep the slot reserved and read the
    /// firmware's handle back.
    fn open_stream(&mut self, kind: StreamKind, cookie: u32, f: u32, payload: Vec<u8>) -> Result<StreamHandle> {
        let ring = self.ring_mut()?;
        let slot = ring.next_slot();
        ring.reserve(slot)?;
        let r = match self.send_cmd_slot(slot, f, &payload) {
            Ok(r) if r.ok() => r,
            Ok(r) => {
                self.ring_mut()?.release(slot);
                log::error!("{} open failed with status {}", kind.name(), r.status);
                return Err(Error::Protocol("ViXS stream open refused"));
            }
            Err(e) => {
                self.ring_mut()?.release(slot);
                return Err(e);
            }
        };
        let phys = self.chip.fb_read(r.payload_fb() + kind.handle_offset())?;
        log::debug!("{} open: slot {slot}, cookie {cookie:#x}, firmware handle {phys:#x}", kind.name());
        Ok(StreamHandle { kind, cookie, phys, slot, open_payload: payload })
    }

    /// Open the CAS object (function 25). Read its endpoint
    /// ([`StreamKind::Generic`]) continuously afterwards; an unread FIFO B
    /// may stall the firmware (§6.3, inferred).
    pub fn open_generic(&mut self, mode: CasMode) -> Result<StreamHandle> {
        let cookie = self.new_cookie();
        let p = build_open_generic(cookie, mode).to_vec();
        self.open_stream(StreamKind::Generic, cookie, func::OPEN_GENERIC, p)
    }

    /// Open the Raw stream (function 8, kind 1) on TS port `ts_port`.
    pub fn open_psi(&mut self, ts_port: u32) -> Result<StreamHandle> {
        let cookie = self.new_cookie();
        let p = build_open_psi(cookie, ts_port).to_vec();
        self.open_stream(StreamKind::Raw, cookie, func::OPEN_STREAM, p)
    }

    /// Open the Program stream (function 8, kind 5).
    pub fn open_program(&mut self, params: &ProgramParams) -> Result<StreamHandle> {
        let cookie = self.new_cookie();
        let p = build_open_program(cookie, params).to_vec();
        self.open_stream(StreamKind::Program, cookie, func::OPEN_STREAM, p)
    }

    /// The internal flush (function 10, a slot of its own) that precedes
    /// the close of a Raw or Program stream. The vendor keeps one IN
    /// transfer pending on the stream's endpoint meanwhile.
    pub fn flush_stream(&mut self, h: &StreamHandle) -> Result<()> {
        if h.kind == StreamKind::Generic {
            return Err(Error::Protocol("the CAS object is closed without a flush"));
        }
        let r = self.send_cmd(func::CONTROL, &build_flush(h.phys))?;
        if !r.ok() {
            log::error!("{} flush failed with status {}", h.kind.name(), r.status);
            return Err(Error::Protocol("ViXS stream flush failed"));
        }
        Ok(())
    }

    /// Close a stream in its OPEN slot (function 9, or 26 for the CAS
    /// object) and release the slot. Then cancel the IN transfer on the
    /// stream's endpoint and call [`Vixs::flush_endpoint_fifo`].
    pub fn close_stream(&mut self, h: &StreamHandle) -> Result<()> {
        let (f, payload) = match h.kind {
            StreamKind::Generic => {
                let open: &[u8; GENERIC_OPEN_LEN] =
                    h.open_payload.as_slice().try_into().map_err(|_| Error::Protocol("bad CAS open payload"))?;
                (func::CLOSE_GENERIC, build_close_generic(open, h.phys).to_vec())
            }
            StreamKind::Raw | StreamKind::Program => {
                let open: &[u8; OPEN_LEN] =
                    h.open_payload.as_slice().try_into().map_err(|_| Error::Protocol("bad stream open payload"))?;
                (func::CLOSE_STREAM, build_close(open, h.phys).to_vec())
            }
        };
        let r = self.send_cmd_slot(h.slot, f, &payload)?;
        if !r.ok() {
            log::error!("{} close failed with status {}", h.kind.name(), r.status);
            return Err(Error::Protocol("ViXS stream close failed"));
        }
        self.ring_mut()?.release(h.slot);
        Ok(())
    }

    /// Flush (Raw / Program only) and close a stream: the firmware side of
    /// the vendor's stop (§6.4). Keep reading the endpoint until this
    /// returns, then cancel the IN transfer and call
    /// [`Vixs::flush_endpoint_fifo`].
    pub fn stop_stream(&mut self, h: &StreamHandle) -> Result<()> {
        if h.kind != StreamKind::Generic {
            self.flush_stream(h)?;
        }
        self.close_stream(h)
    }

    /// [`flush_endpoint_fifo`] on this device.
    pub fn flush_endpoint_fifo(&self, ep: u8) -> Result<bool> {
        flush_endpoint_fifo(self.chip.bus(), &self.layout, ep)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::super::cmd::Completion;
    use super::super::mmr::testutil::*;
    use super::super::tests::running;
    use super::*;
    use crate::bus::mock::{MockPci, Op, Space};

    fn w(b: &[u8], off: usize) -> u32 {
        u32::from_le_bytes(b[off..off + 4].try_into().unwrap())
    }

    /// All words of `b` that are not listed in `set` are zero.
    fn only(b: &[u8], set: &[usize]) {
        for off in (0..b.len()).step_by(4) {
            if !set.contains(&off) {
                assert_eq!(w(b, off), 0, "word at {off:#x}");
            }
        }
    }

    fn table(b: &[u8], off: usize, size: u32, addr: impl Fn(u32) -> u32) -> Vec<usize> {
        assert_eq!(w(b, off), 32, "count at {off:#x}");
        assert_eq!(w(b, off + 4), size, "size at {off:#x}");
        for i in 0..32 {
            assert_eq!(w(b, off + 8 + 4 * i), addr(i as u32), "address {i} at {off:#x}");
        }
        (0..34).map(|i| off + 4 * i).collect()
    }

    #[test]
    fn cas_modes() {
        assert_eq!(CasMode::default(), CasMode::BcasDrm);
        assert_eq!((CasMode::BcasDrm.generic_mode(), CasMode::BcasDrm.secured_mode()), (32, 16));
        assert_eq!((CasMode::Bcas.generic_mode(), CasMode::Bcas.secured_mode()), (16, 8));
    }

    #[test]
    fn stream_kinds() {
        assert_eq!(StreamKind::Generic.endpoint(), 0x82);
        assert_eq!(StreamKind::Raw.endpoint(), 0x83);
        assert_eq!(StreamKind::Program.endpoint(), 0x84);
        assert_eq!(StreamKind::Generic.transfer_len(), 0x8000);
        assert_eq!(StreamKind::Program.transfer_len(), 48128);
        assert_eq!(StreamKind::Generic.header_len(), 0x1EC);
        assert_eq!(StreamKind::Raw.header_len(), 0);
        assert_eq!(StreamKind::Raw.handle_offset(), 0x18);
        assert_eq!(StreamKind::Generic.handle_offset(), 0);
        let l = super::super::tests::layout();
        for k in [StreamKind::Generic, StreamKind::Raw, StreamKind::Program] {
            assert_eq!(l.endpoints[k.fifo_index()], k.endpoint());
        }
        assert_eq!(l.fifo[1..], [FIFO_B, FIFO_C, FIFO_D]);
    }

    #[test]
    fn tsd_config_values() {
        let sony = tsd_config_fb_writes(&TsdConfig::for_kit(KitKind::Mj110));
        let want = [
            (0x50, 1),
            (0x54, 0),
            (0x58, 3),
            (0x5C, 3),
            (0x60, 3),
            (0x64, 0xFF),
            (0x68, 0),
            (0x6C, 1),
            (0x70, 1),
            (0x74, 0),
            (0x78, 0),
            (0x7C, 0),
            (0x80, 1),
            (0x84, 1),
        ];
        assert_eq!(sony, want);
        let toshiba = tsd_config_fb_writes(&TsdConfig::for_kit(KitKind::Toshiba));
        assert_eq!(toshiba[6], (0x68, 3));
        assert_eq!(toshiba[..6], want[..6]);
        assert_eq!(toshiba[7..], want[7..]);
        for k in [KitKind::Mj100, KitKind::Mj102] {
            assert_eq!(TsdConfig::for_kit(k).packet_204_mask, 0);
        }
        assert_eq!(build_tsd_config(), [0; 4]);
    }

    #[test]
    fn generic_open_layout() {
        for mode in [CasMode::BcasDrm, CasMode::Bcas] {
            let b = build_open_generic(0xC0DE, mode);
            assert_eq!(b.len(), 0x530);
            assert_eq!(w(&b, 0x000), 0xC0DE);
            assert_eq!(w(&b, 0x004), 0);
            assert_eq!(w(&b, 0x008), mode.generic_mode());
            let mut set = vec![0x000, 0x008];
            set.extend(table(&b, 0x00C, 0x8000, |_| 0x3004_0000));
            set.extend(table(&b, 0x114, 0x4_0000, |i| fake_host_phys(0, i)));
            set.extend(table(&b, 0x21C, 0x8000, |_| 0x3004_0000));
            // Left-over areas, +0x324, the descriptor, blobs and their lengths.
            only(&b, &set);
            assert!(b[0x094..0x114].iter().all(|&x| x == 0));
            assert_eq!(w(&b, 0x4A8), 0);
            assert_eq!(w(&b, 0x52C), 0);
        }
        // Host-address stand-ins are distinct and non-zero.
        let b = build_open_generic(1, CasMode::BcasDrm);
        let mut v: Vec<u32> = (0..32).map(|i| w(&b, 0x11C + 4 * i)).collect();
        v.sort();
        v.dedup();
        assert_eq!(v.len(), 32);
        assert!(v.iter().all(|&x| x != 0));
    }

    #[test]
    fn generic_close_reuses_the_open() {
        let open = build_open_generic(0xC0DE, CasMode::BcasDrm);
        let c = build_close_generic(&open, 0x8765_4321);
        assert_eq!((w(&c, 0), w(&c, 4), w(&c, 8)), (0x8765_4321, 0, 0));
        assert_eq!(c[0xC..], open[0xC..]);
    }

    #[test]
    fn psi_open_layout() {
        let b = build_open_psi(0x77, 1);
        assert_eq!(b.len(), 0x1198);
        assert_eq!(w(&b, 0x008), 1);
        assert_eq!(w(&b, 0x018), 0x77);
        assert_eq!(w(&b, 0x058), 1);
        assert_eq!(w(&b, 0x060), 0x0010_0000);
        assert_eq!(w(&b, 0x07C), 0xFFFF);
        assert_eq!(w(&b, 0x080), 0x1010);
        let mut set = vec![0x008, 0x018, 0x058, 0x060, 0x07C, 0x080];
        set.extend(table(&b, 0x314, 0xBC00, |_| 0x3008_0000));
        // audio out / video in / audio in counts and sizes, +0xD90, +0xD94: 0
        only(&b, &set);
        // Port 0 leaves +0x008 zero.
        assert_eq!(w(&build_open_psi(0x77, 0), 0x008), 0);
    }

    fn es(list: &[(u16, u8)], ecm: u16) -> Vec<EsEntry> {
        list.iter().map(|&(pid, stream_type)| EsEntry { pid, stream_type, ecm_pid: ecm }).collect()
    }

    #[test]
    fn video_and_aac_selection() {
        let e = es(&[(0x111, 0x1B), (0x100, 0x02), (0x110, 0x0F), (0x112, 0x0F), (0x113, 0x0F), (0x101, 0x01)], 0);
        assert_eq!(video_pid(&e), 0x100);
        assert_eq!(aac_pids(&e), [0x110, 0x112]);
        assert_eq!(aac_pids(&e[3..]), [0x112, 0x113]);
        assert_eq!(aac_pids(&e[4..]), [0x113, 0]);
        assert_eq!(video_pid(&es(&[(0x181, 0x1B)], 0)), 0xFFFF);
        assert_eq!(aac_pids(&[]), [0, 0]);
        let mut long = es(&[(0x200, 6); 30], 0);
        assert_eq!(limit_es(&long).len(), 24);
        long[3].pid = 0;
        assert_eq!(limit_es(&long).len(), 3);
    }

    #[test]
    fn program_open_layout() {
        let e = es(&[(0x0100, 0x02), (0x0110, 0x0F), (0x0111, 0x0F), (0x0138, 0x06), (0x0130, 0x0D)], 0x01F0);
        let p = ProgramParams::new(0, 0xCA5, 0xAA7, 0x0408, 0x01F0 + 0x10, 0x01FF, &e, CasMode::BcasDrm);
        assert_eq!(p.video_pid, 0x100);
        assert_eq!(p.aac_pids, [0x110, 0x111]);
        let b = build_open_program(0x55, &p);
        assert_eq!(b.len(), 0x1198);
        let want: &[(usize, u32)] = &[
            (0x008, 0),
            (0x010, 0),
            (0x018, 0x55),
            (0x020, 0xCA5),
            (0x028, 0xAA7),
            (0x058, 5),
            (0x060, 0x0010_0060),
            (0x068, 0x8000_0002),
            (0x07C, 0x100),
            (0x080, 0x100),
            (0x084, 0x8000_0002),
            (0x088, 8_000_000),
            (0x090, 720),
            (0x094, 480),
            (0x0C0, 0xFFFF_FFFF),
            (0x0C4, 0xFFFF_FFFF),
            (0x0D0, 0x100),
            (0xAE4, 0x1_0078),
            (0xC08, 0x1FF),
            (0xC28, 0x1FF),
            (0xC40, 0x200),
            (0xC68, 500),
            (0xC70, 0x408),
            (0xEE4, 2),
            (0xEE8, 0x110),
            (0xEEC, 0x111),
            (0xEF8, 0x110),
            (0xEFC, 0x111),
            (0xF0C, 0x1100),
            (0xF14, 1),
            (0xF24, 0x408),
            (0xF2C, 16),
            (0x1068, 5),
        ];
        let mut set: Vec<usize> = want.iter().map(|&(o, _)| o).collect();
        for &(off, v) in want {
            assert_eq!(w(&b, off), v, "word at {off:#x}");
        }
        for (i, x) in e.iter().enumerate() {
            assert_eq!(w(&b, 0x106C + 4 * i), u32::from(x.pid));
            assert_eq!(w(&b, 0x10CC + 4 * i), 0x1F0);
            assert_eq!(w(&b, 0x112C + 4 * i), u32::from(x.stream_type));
            set.extend([0x106C + 4 * i, 0x10CC + 4 * i, 0x112C + 4 * i]);
        }
        set.extend(table(&b, 0x314, 0xBC00, |_| 0x300C_0000));
        set.extend(table(&b, 0x41C, 0x8000, |_| 0x3004_0000));
        set.extend(table(&b, 0x524, 0xBC00, |i| fake_host_phys(1, i)));
        set.extend(table(&b, 0x62C, 0, |i| fake_host_phys(2, i)));
        // Everything else, including +0x000, +0x05C, +0x064, +0x118C and
        // the generic / data / frpt tables, is zero.
        only(&b, &set);
    }

    #[test]
    fn program_open_variants() {
        // Non-DRM mode, port 1, no handles, SID 0, one AAC, no video.
        let e = es(&[(0x0181, 0x1B), (0x0183, 0x0F)], 0);
        let p = ProgramParams::new(1, 0, 0, 0, 0x1FC8, 0x0181, &e, CasMode::Bcas);
        let b = build_open_program(9, &p);
        assert_eq!((w(&b, 0x008), w(&b, 0x010)), (1, 1));
        assert_eq!((w(&b, 0x020), w(&b, 0x028)), (0, 0));
        assert_eq!(w(&b, 0xF2C), 8);
        assert_eq!((w(&b, 0x07C), w(&b, 0x080), w(&b, 0x0D0)), (0xFFFF, 0xFFFF, 0xFFFF));
        assert_eq!((w(&b, 0xEE4), w(&b, 0xEE8), w(&b, 0xEEC)), (1, 0x183, 0));
        assert_eq!((w(&b, 0xC70), w(&b, 0xF24)), (0, 0));
        assert_eq!(w(&b, 0x1068), 2);
        assert_eq!(w(&b, 0x10CC), 0);
        // More than 24 streams are cut.
        let many = es(&[(0x300, 6); 26], 1);
        let p = ProgramParams { es: &many, ..p };
        let b = build_open_program(9, &p);
        assert_eq!(w(&b, 0x1068), 24);
        assert_eq!(w(&b, 0x106C + 4 * 23), 0x300);
        assert_eq!(w(&b, 0x118C), 0);
    }

    #[test]
    fn flush_and_close_layout() {
        let f = build_flush(0xABCD);
        assert_eq!(f.len(), 0x32C);
        assert_eq!((w(&f, 0), w(&f, 0x10), w(&f, 0x2F4)), (0xABCD, 0x8000, 1));
        only(&f, &[0, 0x10, 0x2F4]);
        let open = build_open_psi(0x77, 0);
        let c = build_close(&open, 0x1234);
        assert_eq!((w(&c, 0), w(&c, 4)), (0x1234, 0));
        assert_eq!(c[8..], open[8..]);
    }

    #[test]
    fn generic_blocks() {
        let mut b = vec![0u8; 0x8000];
        b[0x30..0x34].copy_from_slice(&0x20_0000u32.to_le_bytes());
        b[0x44] = 7;
        b[0x48] = 0x10;
        b[0x60] = 1;
        b[0x88..0x8C].copy_from_slice(&0x2000u32.to_le_bytes());
        b[0x1EC] = 0xAB;
        let (h, body) = parse_generic_block(&b).unwrap();
        assert_eq!(
            h,
            GenericBlockHeader {
                kind: generic_kind::DRM,
                buf_index: 7,
                size: 0x10,
                last: true,
                card: card_state::INSERTED
            }
        );
        assert_eq!(body.len(), 0x8000 - 0x1EC);
        assert_eq!(body[0], 0xAB);
        assert!(parse_generic_block(&b[..0x1EB]).is_none());
    }

    #[test]
    fn cdxm_packets() {
        let mut p = [0u8; 188];
        p[0] = 0x47;
        p[7..11].copy_from_slice(b"CDXM");
        assert!(is_cdxm_packet(&p));
        p[0] = 0x46;
        assert!(!is_cdxm_packet(&p));
        assert!(!is_cdxm_packet(&[0x47; 10]));
    }

    #[test]
    fn fifo_flush() {
        let m = MockPci::new();
        let l = super::super::tests::layout();
        // FIFO D not empty: flush and read back.
        m.set(Space::Csr, 0x38C, 0);
        m.queue(Space::Csr, 0x390, &[0x200, 0]);
        assert!(flush_endpoint_fifo(&m, &l, 0x84).unwrap());
        assert_eq!(
            m.ops(),
            vec![Op::CsrRead(0x38C), Op::CsrRead(0x390), Op::CsrWrite(0x38C, 0x200), Op::CsrRead(0x390)]
        );
        // FIFO B empty: nothing written.
        m.clear();
        m.set(Space::Csr, 0x34C, EP_STAT_FIFO_EMPTY);
        assert!(!flush_endpoint_fifo(&m, &l, 0x82).unwrap());
        assert_eq!(m.ops(), vec![Op::CsrRead(0x34C), Op::CsrRead(0x350)]);
        assert!(flush_endpoint_fifo(&m, &l, 0x81).is_err());
    }

    /// Script the completion of `cmds` (function, slot, status) in order,
    /// and the firmware handle `phys` of OPENs at `handle_offset`.
    pub(crate) fn script_cmds(v: &Vixs<MockPci>, cmds: &[(u32, u32, u32)]) {
        let ring = v.ring().unwrap().clone();
        let m = v.bus();
        m.set(Space::Mem, MMR + 0x30E4, 3);
        for &(f, slot, status) in cmds {
            let fb = ring.slot_fb(slot);
            m.queue(Space::Mem, MMR + 0x30E0, &[2]);
            m.queue(Space::Mem, MMR + 0x302C, &[f]);
            m.queue(Space::Mem, MMR + 0x3048, &[fb]);
            m.set(Space::Mem, FB + fb + 0x10, status);
        }
    }

    /// The payload of the `n`-th slot written by DMA.
    pub(crate) fn sent_payload(m: &MockPci, n: usize) -> Vec<u8> {
        m.bulk()[n].1[0x24..].to_vec()
    }

    #[test]
    fn open_and_close_streams() {
        let mut v = running(Completion::Poll);
        let ring = v.ring().unwrap().clone();
        script_cmds(&v, &[(25, 0, 1), (8, 1, 1), (8, 2, 1), (10, 3, 1), (9, 2, 1), (10, 4, 1), (9, 1, 1), (26, 0, 1)]);
        v.bus().set(Space::Mem, FB + ring.slot_fb(0) + 0x24, 0xF000_0001);
        v.bus().set(Space::Mem, FB + ring.slot_fb(1) + 0x24 + 0x18, 0xF000_0002);
        v.bus().set(Space::Mem, FB + ring.slot_fb(2) + 0x24 + 0x18, 0xF000_0003);
        v.bus().clear();

        let cas = v.open_generic(CasMode::BcasDrm).unwrap();
        assert_eq!((cas.kind(), cas.slot(), cas.phys(), cas.cookie()), (StreamKind::Generic, 0, 0xF000_0001, 1));
        let raw = v.open_psi(0).unwrap();
        assert_eq!((raw.slot(), raw.phys(), raw.cookie()), (1, 0xF000_0002, 2));
        assert_eq!(v.stream_endpoint(StreamKind::Raw), 0x83);
        let e = es(&[(0x100, 2)], 0);
        let p = ProgramParams::new(0, cas.phys(), raw.phys(), 1, 0x1F0, 0x1FF, &e, CasMode::BcasDrm);
        let pn = v.open_program(&p).unwrap();
        assert_eq!((pn.slot(), pn.phys(), pn.cookie()), (2, 0xF000_0003, 3));
        assert!((0..3).all(|i| v.ring().unwrap().is_reserved(i)));

        // The handles are read back with one plain FB read each.
        let acc = accesses(&v.bus().ops());
        for (slot, off) in [(0, 0), (1, 0x18), (2, 0x18)] {
            assert!(acc.contains(&Acc::FbR(ring.slot_fb(slot) + 0x24 + off)));
        }
        let m = v.bus();
        assert_eq!(sent_payload(m, 0)[..0x530], build_open_generic(1, CasMode::BcasDrm));
        assert_eq!(sent_payload(m, 1)[..0x1198], build_open_psi(2, 0));
        let sent = sent_payload(m, 2);
        assert_eq!(sent[..0x1198], build_open_program(3, &p));
        assert_eq!(w(&sent, 0x20), 0xF000_0001);
        assert_eq!(w(&sent, 0x28), 0xF000_0002);
        assert!(sent[0x1198..].iter().all(|&x| x == 0));

        // Program: flush in a new slot, close in its OPEN slot.
        v.bus().clear();
        v.stop_stream(&pn).unwrap();
        let b = v.bus().bulk();
        assert_eq!(w(&b[0].1, 0x1C), 3);
        assert_eq!(b[0].1[0x24..0x24 + 0x32C], build_flush(0xF000_0003));
        assert_eq!(w(&b[1].1, 0x1C), 2);
        assert_eq!(w(&b[1].1, 0x0C), 9);
        assert_eq!(b[1].1[0x24..0x24 + 0x1198], build_close(pn.open_payload().try_into().unwrap(), 0xF000_0003));
        assert!(!v.ring().unwrap().is_reserved(2));

        // Raw, then the CAS object without a flush.
        v.stop_stream(&raw).unwrap();
        v.bus().clear();
        v.stop_stream(&cas).unwrap();
        let b = v.bus().bulk();
        assert_eq!(b.len(), 1);
        assert_eq!((w(&b[0].1, 0x0C), w(&b[0].1, 0x1C)), (26, 0));
        let c = &b[0].1[0x24..];
        assert_eq!((w(c, 0), w(c, 4), w(c, 8)), (0xF000_0001, 0, 0));
        assert!(v.flush_stream(&cas).is_err());
        assert!((0..41).all(|i| !v.ring().unwrap().is_reserved(i)));
    }

    #[test]
    fn refused_open_releases_the_slot() {
        let mut v = running(Completion::Poll);
        script_cmds(&v, &[(8, 0, 2), (24, 1, 30)]);
        std::assert_matches!(v.open_psi(0), Err(Error::Protocol(_)));
        assert!(!v.ring().unwrap().is_reserved(0));
        // The TS input set-up refused (status 30: a stream is open).
        v.bus().clear();
        assert!(v.tsd_config(&TsdConfig::for_kit(KitKind::Toshiba)).is_err());
        let w = writes(&v.bus().ops());
        assert_eq!(w[..14], tsd_config_fb_writes(&TsdConfig::for_kit(KitKind::Toshiba)).map(|(o, v)| Acc::FbW(o, v)));
        assert!(w.contains(&Acc::W(0x3020, 24)));
    }
}
