//! Byte layout of the register packets, as pure functions (docs/26 §3.3, §4.3).
//!
//! Every register access is a bulk OUT packet whose first six bytes are
//!
//! ```text
//! off size
//! 0   u16  control word (LE)
//! 2   u32  address (LE)
//! 6   u32  data (LE), writes only
//! ```
//!
//! A write sends all ten bytes; a read sends only the first six and then
//! reads a 4-byte little-endian reply on the paired IN endpoint. Reads and
//! writes are told apart by the packet length alone.
//!
//! * Local packets (EP 0x0D / 0x8D) reach the NET2280 itself: the control
//!   word is `ctl` with a zero high byte, and the address is a 16-bit register
//!   offset (the high half is sent as zero).
//! * Remote packets (EP 0x0E / 0x8E) are master cycles on the PCI bus: the
//!   control word is `0x4000 | command | start | read | byte enables`, the
//!   address is the full 32-bit PCI address, sent unmasked even for sub-word
//!   accesses.

use dth30_core::{Error, Result};

/// Length of a write packet.
pub const WRITE_LEN: usize = 10;
/// Length of the OUT half of a read.
pub const READ_LEN: usize = 6;
/// Length of a read reply.
pub const REPLY_LEN: usize = 4;

/// The NET2280's own register spaces (local packets, EP 0x0D / 0x8D).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LocalSpace {
    /// Memory-mapped registers (BAR0 space, offsets 0x000-0x3FF): `ctl` 0x1F.
    Csr,
    /// The NET2280's own PCI configuration space: `ctl` 0x0F.
    Config,
    /// 8051 program memory, one byte (byte enable 1): `ctl` 0x21. The vendor
    /// stack has the call but never uses it.
    Mem8051,
}

impl LocalSpace {
    /// The control byte the vendor stack sends for this space.
    pub const fn ctl(self) -> u8 {
        match self {
            LocalSpace::Csr => 0x1F,
            LocalSpace::Config => 0x0F,
            LocalSpace::Mem8051 => 0x21,
        }
    }
}

/// Remote PCI cycle type (command bits 7:6 of the control word).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PciSpace {
    Memory,
    Io,
    Config,
}

impl PciSpace {
    pub const fn command_bits(self) -> u16 {
        match self {
            PciSpace::Memory => 0x00,
            PciSpace::Io => 0x40,
            PciSpace::Config => 0x80,
        }
    }
}

/// Always ORed into the remote control word (the vendor's default
/// PCIMSTCTL value; meaning of bit 14 unknown, docs/26 §11).
pub const PCI_MST_CTL_DEFAULT: u16 = 0x4000;
/// Start bit of the remote control word.
pub const PCI_MST_START: u16 = 0x20;
/// Read bit of the remote control word.
pub const PCI_MST_READ: u16 = 0x10;
/// All four byte enables.
pub const BE_ALL: u8 = 0x0F;

/// The remote control word: `0x4000 | command | start | read? | byte enables`.
pub const fn remote_ctl(space: PciSpace, read: bool, byte_enable: u8) -> u16 {
    PCI_MST_CTL_DEFAULT
        | space.command_bits()
        | PCI_MST_START
        | if read { PCI_MST_READ } else { 0 }
        | (byte_enable & 0x0F) as u16
}

fn header(word0: u16, addr: u32) -> [u8; READ_LEN] {
    let w = word0.to_le_bytes();
    let a = addr.to_le_bytes();
    [w[0], w[1], a[0], a[1], a[2], a[3]]
}

fn with_data(h: [u8; READ_LEN], value: u32) -> [u8; WRITE_LEN] {
    let d = value.to_le_bytes();
    [h[0], h[1], h[2], h[3], h[4], h[5], d[0], d[1], d[2], d[3]]
}

/// OUT half of a local read: `[ctl, 0x00, off_lo, off_hi, 0x00, 0x00]`.
pub fn encode_local_read(space: LocalSpace, offset: u16) -> [u8; READ_LEN] {
    header(space.ctl() as u16, offset as u32)
}

/// A local write: `[ctl, 0x00, off_lo, off_hi, 0x00, 0x00, value (LE)]`.
pub fn encode_local_write(space: LocalSpace, offset: u16, value: u32) -> [u8; WRITE_LEN] {
    with_data(encode_local_read(space, offset), value)
}

/// OUT half of a remote read with an explicit control word (see [`remote_ctl`]).
pub fn encode_remote_read(word0: u16, addr: u32) -> [u8; READ_LEN] {
    header(word0 | PCI_MST_CTL_DEFAULT, addr)
}

/// A remote write with an explicit control word (see [`remote_ctl`]).
pub fn encode_remote_write(word0: u16, addr: u32, value: u32) -> [u8; WRITE_LEN] {
    with_data(header(word0 | PCI_MST_CTL_DEFAULT, addr), value)
}

/// Byte enables and data shift of a 16-bit access at `addr`: the upper half
/// of the dword when bit 1 is set, else the lower half.
pub const fn word_lanes(addr: u32) -> (u8, u32) {
    if addr & 2 != 0 { (0x0C, 16) } else { (0x03, 0) }
}

/// Byte enables and data shift of an 8-bit access at `addr`.
pub const fn byte_lanes(addr: u32) -> (u8, u32) {
    let lane = addr & 3;
    (1 << lane, 8 * lane)
}

/// The 4-byte little-endian reply of a read. Bytes beyond the fourth are
/// ignored (the vendor asks for exactly four).
pub fn decode_reply(reply: &[u8]) -> Result<u32> {
    match reply.first_chunk::<REPLY_LEN>() {
        Some(b) => Ok(u32::from_le_bytes(*b)),
        None => Err(Error::ShortTransfer { expected: REPLY_LEN, actual: reply.len() }),
    }
}

/// Largest bulk transfer the vendor stack submits at once (its maximum
/// transfer size); longer buffers go out as consecutive chunks of this size.
pub const BULK_CHUNK: usize = 0x40000;

/// The chunk lengths a bulk OUT of `len` bytes is split into. An empty buffer
/// is one zero-length transfer, as the vendor stack submits it.
pub fn bulk_chunks(len: usize) -> impl Iterator<Item = usize> {
    let n = len.div_ceil(BULK_CHUNK).max(1);
    (0..n).map(move |i| (len - i * BULK_CHUNK).min(BULK_CHUNK))
}

/// `len` rounded up to a whole number of `max_packet_size` packets (IN
/// transfers must ask for whole packets; a device answering with a shorter
/// packet ends the transfer, so the bus traffic is the same).
pub fn round_up_to_packets(len: usize, max_packet_size: usize) -> usize {
    let mps = max_packet_size.max(1);
    len.max(1).div_ceil(mps) * mps
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn local_write_matches_spec_example() {
        // USBIRQENB1 (0x24) = 0x81000000.
        assert_eq!(
            encode_local_write(LocalSpace::Csr, 0x24, 0x8100_0000),
            [0x1F, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81]
        );
    }

    #[test]
    fn local_read_matches_spec_example() {
        // DEVINIT (0x00).
        assert_eq!(encode_local_read(LocalSpace::Csr, 0x00), [0x1F, 0x00, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(encode_local_read(LocalSpace::Csr, 0x0380), [0x1F, 0x00, 0x80, 0x03, 0x00, 0x00]);
    }

    #[test]
    fn local_config_and_8051() {
        // NET2280 PCI command = 0x16; BAR2 = 0x30000000.
        assert_eq!(
            encode_local_write(LocalSpace::Config, 0x04, 0x16),
            [0x0F, 0x00, 0x04, 0x00, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00]
        );
        assert_eq!(
            encode_local_write(LocalSpace::Config, 0x18, 0x3000_0000),
            [0x0F, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30]
        );
        assert_eq!(encode_local_read(LocalSpace::Config, 0x00), [0x0F, 0x00, 0x00, 0x00, 0x00, 0x00]);
        // 8051: "!" plus its terminating NUL, the byte zero-extended.
        assert_eq!(
            encode_local_write(LocalSpace::Mem8051, 0x1234, 0xAB),
            [0x21, 0x00, 0x34, 0x12, 0x00, 0x00, 0xAB, 0x00, 0x00, 0x00]
        );
    }

    #[test]
    fn remote_control_words_match_table() {
        assert_eq!(remote_ctl(PciSpace::Memory, false, BE_ALL), 0x402F);
        assert_eq!(remote_ctl(PciSpace::Memory, true, BE_ALL), 0x403F);
        assert_eq!(remote_ctl(PciSpace::Config, false, BE_ALL), 0x40AF);
        assert_eq!(remote_ctl(PciSpace::Config, true, BE_ALL), 0x40BF);
        assert_eq!(remote_ctl(PciSpace::Io, false, BE_ALL), 0x406F);
        assert_eq!(remote_ctl(PciSpace::Io, true, BE_ALL), 0x407F);
    }

    #[test]
    fn remote_packets_match_spec_examples() {
        // ViXS VID/DID read.
        assert_eq!(
            encode_remote_read(remote_ctl(PciSpace::Config, true, BE_ALL), 0x1_0000),
            [0xBF, 0x40, 0x00, 0x00, 0x01, 0x00]
        );
        // ViXS Command = 7.
        assert_eq!(
            encode_remote_write(remote_ctl(PciSpace::Config, false, BE_ALL), 0x1_0004, 7),
            [0xAF, 0x40, 0x04, 0x00, 0x01, 0x00, 0x07, 0x00, 0x00, 0x00]
        );
        // Memory read of the board-ID register.
        assert_eq!(
            encode_remote_read(remote_ctl(PciSpace::Memory, true, BE_ALL), 0x2000_0478),
            [0x3F, 0x40, 0x78, 0x04, 0x00, 0x20]
        );
        assert_eq!(
            encode_remote_write(remote_ctl(PciSpace::Memory, false, BE_ALL), 0x1000_0000, 0xDEAD_BEEF),
            [0x2F, 0x40, 0x00, 0x00, 0x00, 0x10, 0xEF, 0xBE, 0xAD, 0xDE]
        );
        // The 0x4000 default is ORed in even when the caller leaves it out.
        assert_eq!(encode_remote_read(0x7F, 0xCF8), [0x7F, 0x40, 0xF8, 0x0C, 0x00, 0x00]);
    }

    #[test]
    fn sub_word_lanes() {
        assert_eq!(word_lanes(0x1000_0000), (0x3, 0));
        assert_eq!(word_lanes(0x1000_0002), (0xC, 16));
        assert_eq!(word_lanes(0x1000_0003), (0xC, 16));
        assert_eq!(byte_lanes(0x10), (0x1, 0));
        assert_eq!(byte_lanes(0x11), (0x2, 8));
        assert_eq!(byte_lanes(0x12), (0x4, 16));
        assert_eq!(byte_lanes(0x13), (0x8, 24));
        // 16-bit memory write at an odd half: BE 0xC, data in the upper lanes,
        // address sent unmasked.
        let (be, shift) = word_lanes(0x1000_0006);
        assert_eq!(
            encode_remote_write(remote_ctl(PciSpace::Memory, false, be), 0x1000_0006, 0xABCD << shift),
            [0x2C, 0x40, 0x06, 0x00, 0x00, 0x10, 0x00, 0x00, 0xCD, 0xAB]
        );
        // 8-bit I/O read at lane 1.
        let (be, _) = byte_lanes(0x61);
        assert_eq!(encode_remote_read(remote_ctl(PciSpace::Io, true, be), 0x61), [0x72, 0x40, 0x61, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn reply_decoding() {
        assert_eq!(decode_reply(&[0x45, 0x17, 0x00, 0x00]).unwrap(), 0x1745);
        assert_eq!(decode_reply(&[1, 2, 3, 4, 5]).unwrap(), 0x0403_0201);
        assert!(matches!(decode_reply(&[1, 2, 3]), Err(Error::ShortTransfer { expected: 4, actual: 3 })));
        assert!(matches!(decode_reply(&[]), Err(Error::ShortTransfer { expected: 4, actual: 0 })));
    }

    #[test]
    fn bulk_chunking() {
        assert_eq!(bulk_chunks(0x10_0000).collect::<Vec<_>>(), [BULK_CHUNK; 4]);
        assert_eq!(bulk_chunks(0x23C0).collect::<Vec<_>>(), [0x23C0]);
        assert_eq!(bulk_chunks(0x40001).collect::<Vec<_>>(), [BULK_CHUNK, 1]);
        assert_eq!(bulk_chunks(BULK_CHUNK).collect::<Vec<_>>(), [BULK_CHUNK]);
        assert_eq!(bulk_chunks(0).collect::<Vec<_>>(), [0]);
    }

    #[test]
    fn packet_rounding() {
        assert_eq!(round_up_to_packets(4, 64), 64);
        assert_eq!(round_up_to_packets(4, 512), 512);
        assert_eq!(round_up_to_packets(4, 4), 4);
        assert_eq!(round_up_to_packets(48128, 512), 48128);
        assert_eq!(round_up_to_packets(4, 0), 4);
    }
}
