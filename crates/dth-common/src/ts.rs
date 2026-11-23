//! MPEG-2 transport stream values (ISO/IEC 13818-1) that the front ends and
//! both families' stream code share.

/// Size of a TS packet, in bytes.
pub const TS_PACKET: usize = 188;

/// The sync byte every TS packet starts with.
pub const SYNC_BYTE: u8 = 0x47;

/// The PID of null (stuffing) packets; in a PMT's `PCR_PID` it means "no PCR".
pub const NULL_PID: u16 = 0x1FFF;

/// The `transport_scrambling_control` bits of a packet's fourth byte.
pub const SCRAMBLING_CONTROL: u8 = 0xC0;

/// The PID of the PAT.
pub const PAT_PID: u16 = 0x0000;

/// Table IDs of the PAT and the PMT.
pub const PAT_TABLE_ID: u8 = 0x00;
pub const PMT_TABLE_ID: u8 = 0x02;

/// Descriptor tag of the CA descriptor, which names an ECM PID.
pub const CA_DESCRIPTOR_TAG: u8 = 0x09;
