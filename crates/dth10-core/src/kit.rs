//! Types every tuner kit uses: the tune request the vendor driver's filter
//! layer hands to a kit, and the signal status block a kit fills in, with
//! its lock and field bits. The bit meanings are the ones the vendor driver
//! uses for all its kits; the TC90517 register conditions quoted in the
//! lock bits are how that demodulator sets them, the other demodulators map
//! their own state onto the same bits.

/// Lock flag bits of [`SignalStatus::lock`].
pub mod lock {
    /// `0xB0 & 0x0F != 0` — synchronisation started.
    pub const SYNC_STARTED: u32 = 0x001;
    /// `0xB0 & 0x0F > 2` — signal detected (used as "signal present").
    pub const SIGNAL: u32 = 0x002;
    /// `0xB0 & 0x0F >= 8 && 0x80 & 0x28 == 0` — fully locked (TS sync).
    pub const LOCKED: u32 = 0x010;
    /// `0x81 bit7` — unknown.
    pub const REG81_BIT7: u32 = 0x100;
}

/// Request/valid mask bits of [`SignalStatus::mask`].
pub mod field {
    pub const LOCK: u32 = 0x01;
    pub const STRENGTH: u32 = 0x02;
    pub const QUALITY: u32 = 0x04;
    pub const CNR: u32 = 0x08;
    pub const BER_A: u32 = 0x10;
    pub const BER_B: u32 = 0x20;
    pub const BER_C: u32 = 0x40;
    /// ISDB-S only: TMCC TS-ID table (`SignalStatus::tsids`), read when locked.
    pub const TSID_TABLE: u32 = 0x10000;
    pub const ALL_BASIC: u32 = LOCK | STRENGTH | QUALITY | CNR;
    pub const ALL: u32 = ALL_BASIC | BER_A | BER_B | BER_C;
}

/// The signal status block the original driver fills in, as named fields.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SignalStatus {
    /// Input: requested fields; output: valid fields (see [`field`]).
    pub mask: u32,
    /// Lock flags (see [`lock`]).
    pub lock: u32,
    /// Signal strength. The demodulator writes `18000 - cnr_mdb`; the
    /// tuner then overwrites it with its own reading.
    pub strength: i32,
    /// Quality 0..100.
    pub quality: u32,
    /// C/N in 0.001 dB.
    pub cnr_mdb: i32,
    /// BER of layers A/B/C ×1e-9; `0xFFFF_FFFF` = layer unused.
    pub ber: [u32; 3],
    /// Raw `0xB0 & 0x0F` TS-sync state (ISDB-T) or register 0xC3 (ISDB-S), for diagnostics.
    pub sync_state: u8,
    /// ISDB-S: number of valid entries in `tsids`.
    pub tsid_count: u8,
    /// ISDB-S: TS-IDs of the transponder, packed.
    pub tsids: [u16; 8],
    /// ISDB-S: raw registers 0xC8..0xCF (TMCC modulation modes / slot counts / `TS-ID[0]`).
    pub tmcc_raw: [u8; 8],
}

/// Value reported for a layer that is not in use.
pub const BER_UNUSED: u32 = 0xFFFF_FFFF;

/// The 24-byte tune request the original driver's filter layer builds from
/// the BDA tuning request and hands to the kit.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TuneRequest {
    /// Bytes 0..8: frequency in Hz (u64). For the satellite kit the filter layer has
    /// already subtracted the 10.678 GHz LNB LO.
    pub frequency_hz: u64,
    /// Bytes 8..12: bandwidth in Hz (6000000 / 8000000 for ISDB-T, 34500000 for ISDB-S).
    pub bandwidth_hz: u32,
    /// Bytes 12..16: modulation code (0x22→1, 0x23→2, 0x17→3, 0x03→4, 0x0B→5, else 0).
    pub modulation: u32,
    /// Bytes 16..20: copied from the BDA request (TSID candidate, unconfirmed).
    pub extra: u32,
    /// Bytes 20..24: always 0.
    pub reserved: u32,
}

impl TuneRequest {
    pub fn isdb_t(frequency_hz: u64, bandwidth_hz: u32) -> Self {
        TuneRequest { frequency_hz, bandwidth_hz, ..Default::default() }
    }

    /// Low 32 bits of the frequency, as consumed by the ISDB-T kit.
    pub fn frequency_u32(&self) -> u32 {
        self.frequency_hz as u32
    }

    pub fn as_bytes(&self) -> [u8; 24] {
        let mut b = [0u8; 24];
        b[0..8].copy_from_slice(&self.frequency_hz.to_le_bytes());
        b[8..12].copy_from_slice(&self.bandwidth_hz.to_le_bytes());
        b[12..16].copy_from_slice(&self.modulation.to_le_bytes());
        b[16..20].copy_from_slice(&self.extra.to_le_bytes());
        b[20..24].copy_from_slice(&self.reserved.to_le_bytes());
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tune_request_layout() {
        let r =
            TuneRequest { frequency_hz: 473_142_857, bandwidth_hz: 6_000_000, modulation: 0, extra: 7, reserved: 0 };
        let b = r.as_bytes();
        assert_eq!(&b[0..4], &473_142_857u32.to_le_bytes());
        assert_eq!(&b[8..12], &6_000_000u32.to_le_bytes());
        assert_eq!(b[16], 7);
    }
}
