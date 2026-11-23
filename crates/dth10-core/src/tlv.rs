//! Device information blob (wValue 0x0000 / wIndex 0xC0DE, 4096 bytes IN).
//!
//! Format:
//!
//! ```text
//! entry := tag len data
//!   tag : 1 byte, bit7 must be set; tag = byte & 0x7F
//!   len : 0x80|n           (1 byte,  n < 128)
//!       | 0x40|hi, lo      (2 bytes, n = ((hi & 0x3F) << 8) | lo)
//! ```
//!
//! The blob starts with a single entry of tag 1 (a container) whose data is a
//! sequence of entries. Integers are big-endian with a variable number of
//! bytes (1..4). Strings are copied into fixed buffers and NUL terminated.

use crate::error::{Error, Result};

/// The tags of the device-information blob, named after the field each one
/// fills.
pub mod tag {
    /// The container entry the blob starts with.
    pub const CONTAINER: u8 = 0x01;
    /// [`DeviceInfo::str_14`](super::DeviceInfo::str_14).
    pub const STR_14: u8 = 0x14;
    /// [`DeviceInfo::str_19`](super::DeviceInfo::str_19).
    pub const STR_19: u8 = 0x19;
    /// [`DeviceInfo::str_1a`](super::DeviceInfo::str_1a).
    pub const STR_1A: u8 = 0x1A;
    /// [`DeviceInfo::str_1c`](super::DeviceInfo::str_1c).
    pub const STR_1C: u8 = 0x1C;
    /// [`TunerDesc::model`](super::TunerDesc::model).
    pub const MODEL: u8 = 0x1D;
    /// [`TunerDesc::byte_40`](super::TunerDesc::byte_40).
    pub const BYTE_40: u8 = 0x40;
    /// [`TunerDesc::byte_46`](super::TunerDesc::byte_46).
    pub const BYTE_46: u8 = 0x46;
    /// [`TunerDesc::byte_47`](super::TunerDesc::byte_47).
    pub const BYTE_47: u8 = 0x47;
    /// [`TunerDesc::kit_name`](super::TunerDesc::kit_name).
    pub const KIT_NAME: u8 = 0x50;
    /// [`TunerDesc::id`](super::TunerDesc::id).
    pub const ID: u8 = 0x51;
    /// [`TunerDesc::flags`](super::TunerDesc::flags).
    pub const FLAGS: u8 = 0x52;
    /// [`TunerDesc::gpio_mask_a`](super::TunerDesc::gpio_mask_a).
    pub const GPIO_MASK_A: u8 = 0x54;
    /// [`TunerDesc::gpio_mask_b`](super::TunerDesc::gpio_mask_b).
    pub const GPIO_MASK_B: u8 = 0x55;
    /// [`TunerDesc::demod_i2c_addr`](super::TunerDesc::demod_i2c_addr).
    pub const DEMOD_I2C_ADDR: u8 = 0x56;
    /// [`TunerDesc::tuner_i2c_addr`](super::TunerDesc::tuner_i2c_addr).
    pub const TUNER_I2C_ADDR: u8 = 0x57;
    /// [`TunerDesc::stream_param`](super::TunerDesc::stream_param).
    pub const STREAM_PARAM: u8 = 0x58;
    /// [`TunerDesc::byte_5c`](super::TunerDesc::byte_5c).
    pub const BYTE_5C: u8 = 0x5C;
    /// [`TunerDesc::demod_xtal_hz`](super::TunerDesc::demod_xtal_hz).
    pub const DEMOD_XTAL: u8 = 0x5D;
    /// [`TunerDesc::tuner_xtal_hz`](super::TunerDesc::tuner_xtal_hz).
    pub const TUNER_XTAL: u8 = 0x5E;
    /// [`TunerDesc::if_freq_hz`](super::TunerDesc::if_freq_hz).
    pub const IF_FREQ: u8 = 0x5F;
    /// [`TunerDesc::hw_version`](super::TunerDesc::hw_version).
    pub const HW_VERSION: u8 = 0x60;
    /// [`TunerDesc::tuner_opts`](super::TunerDesc::tuner_opts).
    pub const TUNER_OPTS: u8 = 0x61;
    /// [`TunerDesc::val_62`](super::TunerDesc::val_62).
    pub const VAL_62: u8 = 0x62;
    /// [`TunerDesc::gpio_mask_c`](super::TunerDesc::gpio_mask_c).
    pub const GPIO_MASK_C: u8 = 0x63;
    /// [`TunerDesc::gpio_c_open`](super::TunerDesc::gpio_c_open).
    pub const GPIO_C_OPEN: u8 = 0x64;
    /// [`TunerDesc::gpio_c_close`](super::TunerDesc::gpio_c_close).
    pub const GPIO_C_CLOSE: u8 = 0x66;
    /// [`TunerDesc::val_67`](super::TunerDesc::val_67).
    pub const VAL_67: u8 = 0x67;
}

/// Bits of [`DeviceInfo::present`]: which fields the blob carried, as the
/// original parser sets them.
pub mod info_present {
    pub const STR_1C: u32 = 0x01;
    pub const STR_19: u32 = 0x02;
    pub const STR_1A: u32 = 0x04;
    pub const STR_14: u32 = 0x10;
}

/// Bits of [`TunerDesc::present`]: which fields the blob carried, as the
/// original parser sets them (the fields of the other tags have none). The
/// kits test some of them.
pub mod present {
    pub const HW_VERSION: u32 = 0x01;
    pub const ID: u32 = 0x02;
    pub const MODEL: u32 = 0x04;
    pub const KIT_NAME: u32 = 0x08;
    pub const FLAGS: u32 = 0x10;
    pub const GPIO_MASK_A: u32 = 0x20;
    pub const GPIO_MASK_B: u32 = 0x80;
    /// Also set when the parser applies the default of `byte_5c`.
    pub const BYTE_5C: u32 = 0x800;
    pub const STREAM_PARAM: u32 = 0x1000;
    pub const TUNER_XTAL: u32 = 0x2000;
    pub const IF_FREQ: u32 = 0x4000;
    pub const TUNER_OPTS: u32 = 0x8000;
    pub const DEMOD_XTAL: u32 = 0x10000;
    /// `val_62`, the demodulator IF override.
    pub const DEMOD_IF: u32 = 0x20000;
    pub const GPIO_MASK_C: u32 = 0x40000;
    pub const GPIO_C_CLOSE: u32 = 0x80000;
    pub const GPIO_C_OPEN: u32 = 0x100000;
    pub const VAL_67: u32 = 0x200000;
}

/// Bits of [`TunerDesc::flags`] that the kits test.
pub mod flag {
    /// Bit0: read by the MxL13xRF tuner set-up together with its
    /// `tuner_opts` bit1 (tuner register 0x04 bit0).
    pub const MXL_REG04: u32 = 0x01;
    /// Bit3: the tuner sits directly on the bridge's I2C bus.
    pub const TUNER_DIRECT_I2C: u32 = 0x08;
    /// Bit5: Init does not power up or configure the chips.
    pub const SKIP_INIT_CONFIG: u32 = 0x20;
    /// Bit7: diversity-capable (BQ200X kits).
    pub const DIVERSITY: u32 = 0x80;
}

/// [`TunerDesc::hw_version`] of the ISDB-T tuner (tuner #0) of most kits.
pub const HW_ISDB_T: u32 = 0x102;
/// [`TunerDesc::hw_version`] of the ISDB-S tuner (tuner #1).
pub const HW_ISDB_S: u32 = 0x402;
/// [`TunerDesc::hw_version`] of an ATSC unit (the LGDT3305 kit).
pub const HW_ATSC: u32 = 0x101;

/// One TLV entry borrowed from the blob.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Entry<'a> {
    pub tag: u8,
    pub data: &'a [u8],
}

/// Iterator over the entries of a TLV region.
#[derive(Debug, Clone)]
pub struct Entries<'a> {
    rest: &'a [u8],
    failed: bool,
}

impl<'a> Entries<'a> {
    pub fn new(region: &'a [u8]) -> Self {
        Entries { rest: region, failed: false }
    }

    /// `true` once a malformed entry was hit (iteration stops there).
    pub fn failed(&self) -> bool {
        self.failed
    }
}

impl<'a> Iterator for Entries<'a> {
    type Item = Entry<'a>;

    fn next(&mut self) -> Option<Entry<'a>> {
        if self.failed || self.rest.len() < 2 {
            return None;
        }
        let b0 = self.rest[0];
        if b0 & 0x80 == 0 {
            self.failed = true;
            return None;
        }
        let tag = b0 & 0x7F;
        let b1 = self.rest[1];
        let (len, hdr) = if b1 & 0x80 != 0 {
            ((b1 & 0x7F) as usize, 2)
        } else if b1 & 0x40 != 0 {
            if self.rest.len() < 3 {
                return None;
            }
            ((((b1 & 0x3F) as usize) << 8) | self.rest[2] as usize, 3)
        } else {
            self.failed = true;
            return None;
        };
        if self.rest.len() < hdr + len {
            return None;
        }
        let data = &self.rest[hdr..hdr + len];
        self.rest = &self.rest[hdr + len..];
        // The original parser stops at a zero-length entry.
        if len == 0 {
            self.rest = &[];
            return None;
        }
        Some(Entry { tag, data })
    }
}

fn be_int(data: &[u8]) -> u32 {
    data.iter().fold(0u32, |acc, &b| (acc << 8) | b as u32)
}

fn str_field(data: &[u8], max: usize) -> String {
    let take = data.len().min(max - 1);
    let bytes = &data[..take];
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).into_owned()
}

/// Device-level information.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DeviceInfo {
    /// Presence bits as set by the original parser.
    pub present: u32,
    /// tag 0x1C (flag 0x01) — 128-byte string (manufacturer?).
    pub str_1c: String,
    /// tag 0x19 (flag 0x02) — 128-byte string (model name?).
    pub str_19: String,
    /// tag 0x1A (flag 0x04) — 64-byte string (model number?).
    pub str_1a: String,
    /// tag 0x14 (flag 0x10) — 64-byte string (serial number?).
    pub str_14: String,
}

/// Tuner descriptor.
///
/// Field names follow the TLV tag letters where the meaning is not yet
/// confirmed; see `docs/00-architecture-notes.md` §4.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct TunerDesc {
    /// Presence bits as set by the original parser.
    pub present: u32,
    /// 'P' 0x50 — tuner kit name, e.g. `TC90517_MxL135RF` (32 bytes).
    pub kit_name: String,
    /// 'Q' 0x51 — tuner/device id.
    pub id: u32,
    /// 'R' 0x52 — flags: bit2 = the satellite kit's I2C over the tuner #0 path, bit3 = tuner I2C directly on
    /// the bus, bit5 = skip power-up/config in Init, bit7 = diversity-capable (BQ200X kits: two demodulators
    /// and tuners on one channel, built only when diversity is requested).
    pub flags: u32,
    /// '`' 0x60 — hardware version: 0x101 / 0x102 / 0x104 / 0x402.
    pub hw_version: u32,
    /// 'V' 0x56 — demodulator I2C 7-bit address.
    pub demod_i2c_addr: u8,
    /// 'F' 0x46 — I2C address of the second (slave) demodulator in BQ200X diversity.
    pub byte_46: u8,
    /// 'W' 0x57 — tuner I2C 7-bit address.
    pub tuner_i2c_addr: u8,
    /// 'G' 0x47 — I2C address of the second tuner in BQ200X diversity.
    pub byte_47: u8,
    /// '^' 0x5E — tuner crystal frequency in Hz.
    pub tuner_xtal_hz: u32,
    /// '_' 0x5F — IF frequency in Hz, negative = spectrum inverted.
    pub if_freq_hz: i32,
    /// 'a' 0x61 — tuner option bits.
    pub tuner_opts: u32,
    /// ']' 0x5D — demodulator crystal frequency in Hz.
    pub demod_xtal_hz: u32,
    /// 'b' 0x62 — demodulator IF override (BQ200X, LGDT3305; used when present).
    pub val_62: u32,
    /// 0x40 — unknown byte.
    pub byte_40: u8,
    /// '\\' 0x5C — unknown byte, defaults to 1.
    pub byte_5c: u8,
    /// 'X' 0x58 — stream parameter copied into the stream-enable command.
    pub stream_param: u32,
    /// 'T' 0x54 — GPIO mask A, asserted first during power-up.
    pub gpio_mask_a: u32,
    /// 'U' 0x55 — GPIO mask B, asserted second during power-up.
    pub gpio_mask_b: u32,
    /// 'c' 0x63 — GPIO mask C.
    pub gpio_mask_c: u32,
    /// 'f' 0x66 — GPIO value written with mask C at Close.
    pub gpio_c_close: u32,
    /// 'd' 0x64 — GPIO value written with mask C at Open.
    pub gpio_c_open: u32,
    /// 'g' 0x67 — GPIO value written with mask C at open in BQ200X diversity.
    pub val_67: u32,
    /// 0x1D — model string, e.g. `h70`, `h55`, `f200` (32 bytes).
    pub model: String,
}

impl TunerDesc {
    /// Tuner I2C traffic goes directly over the bridge (not through the
    /// demodulator's I2C gate).
    pub fn tuner_direct_i2c(&self) -> bool {
        self.flags & flag::TUNER_DIRECT_I2C != 0
    }
    /// Init must not power up / configure the chips (`flags` bit5).
    pub fn skip_init_config(&self) -> bool {
        self.flags & flag::SKIP_INIT_CONFIG != 0
    }
    /// Model string starts with the given prefix (the driver uses `strncmp`).
    pub fn model_is(&self, prefix: &str) -> bool {
        self.model.starts_with(prefix)
    }
}

/// Parsed result of the device-information command.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DeviceInformation {
    pub info: DeviceInfo,
    pub desc: TunerDesc,
    /// All entries of the container, for diagnostics (tag, data).
    pub raw_entries: Vec<(u8, Vec<u8>)>,
}

/// Parse the 4096-byte blob returned by wValue 0 / wIndex 0xC0DE.
pub fn parse_device_information(blob: &[u8]) -> Result<DeviceInformation> {
    let mut top = Entries::new(blob);
    let container = top.next().ok_or(Error::InvalidTlv("empty blob"))?;
    if container.tag != tag::CONTAINER {
        return Err(Error::InvalidTlv("first entry is not the tag-1 container"));
    }

    let mut out = DeviceInformation::default();
    let info = &mut out.info;
    let desc = &mut out.desc;
    desc.byte_5c = 0; // set explicitly below if present

    for e in Entries::new(container.data) {
        out.raw_entries.push((e.tag, e.data.to_vec()));
        let d = e.data;
        match e.tag {
            tag::STR_1C => {
                info.str_1c = str_field(d, 128);
                info.present |= info_present::STR_1C;
            }
            tag::STR_19 => {
                info.str_19 = str_field(d, 128);
                info.present |= info_present::STR_19;
            }
            tag::STR_1A => {
                info.str_1a = str_field(d, 64);
                info.present |= info_present::STR_1A;
            }
            tag::STR_14 => {
                info.str_14 = str_field(d, 64);
                info.present |= info_present::STR_14;
            }
            tag::MODEL => {
                desc.model = str_field(d, 32);
                desc.present |= present::MODEL;
            }
            tag::BYTE_40 => desc.byte_40 = d[0],
            tag::BYTE_46 => desc.byte_46 = d[0],
            tag::BYTE_47 => desc.byte_47 = d[0],
            tag::KIT_NAME => {
                desc.kit_name = str_field(d, 32);
                desc.present |= present::KIT_NAME;
            }
            tag::ID => {
                desc.id = be_int(d);
                desc.present |= present::ID;
            }
            tag::FLAGS => {
                desc.flags = be_int(d);
                desc.present |= present::FLAGS;
            }
            tag::GPIO_MASK_A => {
                desc.gpio_mask_a = be_int(d);
                desc.present |= present::GPIO_MASK_A;
            }
            tag::GPIO_MASK_B => {
                desc.gpio_mask_b = be_int(d);
                desc.present |= present::GPIO_MASK_B;
            }
            tag::DEMOD_I2C_ADDR => desc.demod_i2c_addr = d[0],
            tag::TUNER_I2C_ADDR => desc.tuner_i2c_addr = d[0],
            tag::STREAM_PARAM => {
                desc.stream_param = be_int(d);
                desc.present |= present::STREAM_PARAM;
            }
            tag::BYTE_5C => {
                desc.byte_5c = d[0];
                desc.present |= present::BYTE_5C;
            }
            tag::DEMOD_XTAL => {
                desc.demod_xtal_hz = be_int(d);
                desc.present |= present::DEMOD_XTAL;
            }
            tag::TUNER_XTAL => {
                desc.tuner_xtal_hz = be_int(d);
                desc.present |= present::TUNER_XTAL;
            }
            tag::IF_FREQ => {
                desc.if_freq_hz = be_int(d) as i32;
                desc.present |= present::IF_FREQ;
            }
            tag::HW_VERSION => {
                desc.hw_version = be_int(d);
                desc.present |= present::HW_VERSION;
            }
            tag::TUNER_OPTS => {
                desc.tuner_opts = be_int(d);
                desc.present |= present::TUNER_OPTS;
            }
            tag::VAL_62 => {
                desc.val_62 = be_int(d);
                desc.present |= present::DEMOD_IF;
            }
            tag::GPIO_MASK_C => {
                desc.gpio_mask_c = be_int(d);
                desc.present |= present::GPIO_MASK_C;
            }
            tag::GPIO_C_OPEN => {
                desc.gpio_c_open = be_int(d);
                desc.present |= present::GPIO_C_OPEN;
            }
            tag::GPIO_C_CLOSE => {
                desc.gpio_c_close = be_int(d);
                desc.present |= present::GPIO_C_CLOSE;
            }
            tag::VAL_67 => {
                desc.val_67 = be_int(d);
                desc.present |= present::VAL_67;
            }
            _ => {}
        }
    }

    // Defaults applied by the original parser after the loop.
    if desc.present & present::BYTE_5C == 0 {
        desc.byte_5c = 1;
        desc.present |= present::BYTE_5C;
    }
    Ok(out)
}

/// Decode the 16-byte firmware-version response (wValue 0 / wIndex 0):
/// bytes 8..12 hold a little-endian u32 printed as `"%lu.%lu.%lu"` with
/// `(v >> 24, (v >> 16) & 0xFF, v & 0xFFFF)`.
pub fn firmware_version(resp: &[u8]) -> Option<(u8, u8, u16)> {
    if resp.len() < 12 {
        return None;
    }
    let v = u32::from_le_bytes([resp[8], resp[9], resp[10], resp[11]]);
    Some(((v >> 24) as u8, ((v >> 16) & 0xFF) as u8, (v & 0xFFFF) as u16))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(tag: u8, data: &[u8]) -> Vec<u8> {
        let mut v = vec![0x80 | tag];
        if data.len() < 128 {
            v.push(0x80 | data.len() as u8);
        } else {
            v.push(0x40 | (data.len() >> 8) as u8);
            v.push((data.len() & 0xFF) as u8);
        }
        v.extend_from_slice(data);
        v
    }

    #[test]
    fn parses_container() {
        let mut inner = Vec::new();
        inner.extend(entry(0x50, b"TC90517_MxL135RF"));
        inner.extend(entry(0x51, &[0x00]));
        inner.extend(entry(0x52, &[0x00, 0x80]));
        inner.extend(entry(0x56, &[0x10]));
        inner.extend(entry(0x57, &[0x60]));
        inner.extend(entry(0x5D, &[0x00, 0xF4, 0x24, 0x00])); // 16000000
        inner.extend(entry(0x5F, &(-44_000_000i32 as u32).to_be_bytes()));
        inner.extend(entry(0x1D, b"h70"));
        inner.extend(entry(0x1C, &[b'A'; 200]));
        let blob = entry(0x01, &inner);
        let r = parse_device_information(&blob).unwrap();
        assert_eq!(r.desc.kit_name, "TC90517_MxL135RF");
        assert_eq!(r.desc.flags, 0x80);
        assert_eq!(r.desc.demod_i2c_addr, 0x10);
        assert_eq!(r.desc.tuner_i2c_addr, 0x60);
        assert_eq!(r.desc.demod_xtal_hz, 16_000_000);
        assert_eq!(r.desc.if_freq_hz, -44_000_000);
        assert_eq!(r.desc.model, "h70");
        assert_eq!(r.desc.byte_5c, 1);
        assert_eq!(r.info.str_1c.len(), 127);
        assert!(r.desc.model_is("h70"));
    }

    #[test]
    fn rejects_bad_header() {
        assert!(parse_device_information(&[0x01, 0x81, 0x00]).is_err());
    }

    #[test]
    fn fw_version() {
        let mut r = [0u8; 16];
        r[8..12].copy_from_slice(&0x0102_0003u32.to_le_bytes());
        assert_eq!(firmware_version(&r), Some((1, 2, 3)));
    }
}
