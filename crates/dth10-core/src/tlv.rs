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
    /// 'R' 0x52 — flags: bit3 = tuner I2C directly on the bus,
    /// bit5 = skip power-up/config in Init, bit7 = also tested by the original driver (purpose unknown).
    pub flags: u32,
    /// '`' 0x60 — hardware version: 0x101 / 0x102 / 0x104 / 0x402.
    pub hw_version: u32,
    /// 'V' 0x56 — demodulator I2C 7-bit address.
    pub demod_i2c_addr: u8,
    /// 'F' 0x46 — unknown byte.
    pub byte_46: u8,
    /// 'W' 0x57 — tuner I2C 7-bit address.
    pub tuner_i2c_addr: u8,
    /// 'G' 0x47 — unknown byte.
    pub byte_47: u8,
    /// '^' 0x5E — tuner crystal frequency in Hz.
    pub tuner_xtal_hz: u32,
    /// '_' 0x5F — IF frequency in Hz, negative = spectrum inverted.
    pub if_freq_hz: i32,
    /// 'a' 0x61 — tuner option bits.
    pub tuner_opts: u32,
    /// ']' 0x5D — demodulator crystal frequency in Hz.
    pub demod_xtal_hz: u32,
    /// 'b' 0x62 — unknown.
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
    /// 'g' 0x67 — unknown.
    pub val_67: u32,
    /// 0x1D — model string, e.g. `h70`, `h55`, `f200` (32 bytes).
    pub model: String,
}

impl TunerDesc {
    /// Tuner I2C traffic goes directly over the bridge (not through the
    /// demodulator's I2C gate).
    pub fn tuner_direct_i2c(&self) -> bool {
        self.flags & 0x8 != 0
    }
    /// Init must not power up / configure the chips (`flags` bit5).
    pub fn skip_init_config(&self) -> bool {
        self.flags & 0x20 != 0
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
    if container.tag != 1 {
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
            0x1C => {
                info.str_1c = str_field(d, 128);
                info.present |= 0x01;
            }
            0x19 => {
                info.str_19 = str_field(d, 128);
                info.present |= 0x02;
            }
            0x1A => {
                info.str_1a = str_field(d, 64);
                info.present |= 0x04;
            }
            0x14 => {
                info.str_14 = str_field(d, 64);
                info.present |= 0x10;
            }
            0x1D => {
                desc.model = str_field(d, 32);
                desc.present |= 0x04;
            }
            0x40 => desc.byte_40 = d[0],
            0x46 => desc.byte_46 = d[0],
            0x47 => desc.byte_47 = d[0],
            0x50 => {
                desc.kit_name = str_field(d, 32);
                desc.present |= 0x08;
            }
            0x51 => {
                desc.id = be_int(d);
                desc.present |= 0x02;
            }
            0x52 => {
                desc.flags = be_int(d);
                desc.present |= 0x10;
            }
            0x54 => {
                desc.gpio_mask_a = be_int(d);
                desc.present |= 0x20;
            }
            0x55 => {
                desc.gpio_mask_b = be_int(d);
                desc.present |= 0x80;
            }
            0x56 => desc.demod_i2c_addr = d[0],
            0x57 => desc.tuner_i2c_addr = d[0],
            0x58 => {
                desc.stream_param = be_int(d);
                desc.present |= 0x1000;
            }
            0x5C => {
                desc.byte_5c = d[0];
                desc.present |= 0x800;
            }
            0x5D => {
                desc.demod_xtal_hz = be_int(d);
                desc.present |= 0x10000;
            }
            0x5E => {
                desc.tuner_xtal_hz = be_int(d);
                desc.present |= 0x2000;
            }
            0x5F => {
                desc.if_freq_hz = be_int(d) as i32;
                desc.present |= 0x4000;
            }
            0x60 => {
                desc.hw_version = be_int(d);
                desc.present |= 0x01;
            }
            0x61 => {
                desc.tuner_opts = be_int(d);
                desc.present |= 0x8000;
            }
            0x62 => {
                desc.val_62 = be_int(d);
                desc.present |= 0x20000;
            }
            0x63 => {
                desc.gpio_mask_c = be_int(d);
                desc.present |= 0x40000;
            }
            0x64 => {
                desc.gpio_c_open = be_int(d);
                desc.present |= 0x100000;
            }
            0x66 => {
                desc.gpio_c_close = be_int(d);
                desc.present |= 0x80000;
            }
            0x67 => {
                desc.val_67 = be_int(d);
                desc.present |= 0x200000;
            }
            _ => {}
        }
    }

    // Defaults applied by the original parser after the loop.
    if desc.present & 0x800 == 0 {
        desc.byte_5c = 1;
        desc.present |= 0x800;
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
