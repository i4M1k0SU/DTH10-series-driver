//! The MIPS firmware of the ViXS chip, taken from the user's copy of the
//! vendor driver.
//!
//! The firmware is not shipped with this crate. The vendor driver
//! (`BufFsUSB.sys`, x86) carries it in its `.data` section as a raw,
//! headerless little-endian MIPS memory image (docs/26 2-vixs-bringup.md
//! §4.1, §8.2): the image starts at a fixed virtual address and its length is
//! a dword at another one. [`Firmware::from_driver`] parses the PE headers,
//! maps both addresses to file offsets through the section table and checks
//! the SHA-256 of the result. A pre-extracted raw image is accepted as well
//! ([`Firmware::from_image`]) when its hash matches. Only the hashes are
//! kept here.
//!
//! The image's vector table (count at +0x110, 16-byte entries from +0x120)
//! is used by the loader as the vendor does: read back from the chip's SDRAM
//! after the download ([`super::mips`]). [`Firmware::vectors`] parses the
//! same table from the image, for diagnostics.

use std::path::Path;

use sha2::{Digest, Sha256};

use crate::{Error, Result};

/// SHA-256 of the firmware image (1,208,208 bytes) in BufFsUSB 1.4.1.9.
pub const FW_SHA256: [u8; 32] = hex32("4ed8c3f765c04fe8f90b63b32008059ec5384a91173a9843b6ac931a808d1d5d");
/// Length of that image.
pub const FW_LEN: usize = 0x12_6F90;

/// A vendor driver build the firmware can be extracted from.
#[derive(Debug, Clone, Copy)]
pub struct KnownDriver {
    pub name: &'static str,
    /// SHA-256 of the driver file.
    pub sha256: [u8; 32],
    /// Virtual address of the image.
    pub image_va: u32,
    /// Virtual address of the dword holding the image length.
    pub len_va: u32,
    /// SHA-256 of the image.
    pub image_sha256: [u8; 32],
}

/// The driver builds this crate knows.
pub const KNOWN_DRIVERS: &[KnownDriver] = &[KnownDriver {
    name: "BufFsUSB.sys 1.4.1.9 (x86)",
    sha256: hex32("60872bd9225a8fd7ef402b83ccbcbae58edee40d05c73da9eba8bba90dc4e286"),
    image_va: 0x0005_1058,
    len_va: 0x0017_7FE8,
    image_sha256: FW_SHA256,
}];

/// Images accepted by [`Firmware::from_image`].
pub const KNOWN_IMAGES: &[[u8; 32]] = &[FW_SHA256];

const fn hex32(s: &str) -> [u8; 32] {
    const fn nib(c: u8) -> u8 {
        match c {
            b'0'..=b'9' => c - b'0',
            b'a'..=b'f' => c - b'a' + 10,
            _ => panic!("bad hex digit"),
        }
    }
    let b = s.as_bytes();
    assert!(b.len() == 64);
    let mut out = [0; 32];
    let mut i = 0;
    while i < 32 {
        out[i] = nib(b[2 * i]) << 4 | nib(b[2 * i + 1]);
        i += 1;
    }
    out
}

fn sha256(data: &[u8]) -> [u8; 32] {
    Sha256::digest(data).into()
}

fn hex(h: &[u8; 32]) -> String {
    h.iter().map(|b| format!("{b:02x}")).collect()
}

/// One entry of the firmware's vector table: four words written at `dst`
/// in SDRAM window 0 (`dst + 0` is cleared last, after w1..w3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VectorEntry {
    pub dst: u32,
    pub words: [u32; 3],
}

/// Offset of the vector-table entry count in the image.
pub const VECTOR_COUNT_OFFSET: u32 = 0x110;
/// Offset of the first vector-table entry.
pub const VECTOR_TABLE_OFFSET: u32 = 0x120;

/// A verified firmware image.
#[derive(Clone)]
pub struct Firmware {
    image: Vec<u8>,
}

impl core::fmt::Debug for Firmware {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Firmware({} bytes)", self.image.len())
    }
}

impl Firmware {
    /// Read a file: a vendor driver (PE, starts with `MZ`) or a raw image.
    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let data = std::fs::read(path).map_err(|e| Error::Unsupported(format!("{}: {e}", path.display())))?;
        Self::from_bytes(&data)
    }

    /// A vendor driver (PE) or a raw image, told apart by the `MZ` signature.
    pub fn from_bytes(data: &[u8]) -> Result<Self> {
        if data.starts_with(b"MZ") { Self::from_driver(data) } else { Self::from_image(data) }
    }

    /// Extract the image from a vendor driver file. Known builds are
    /// recognised by their hash; for an unknown file each known location is
    /// tried and the result accepted only if its hash matches.
    pub fn from_driver(sys: &[u8]) -> Result<Self> {
        let pe = Pe::parse(sys)?;
        let file_hash = sha256(sys);
        let ordered = KNOWN_DRIVERS
            .iter()
            .filter(|d| d.sha256 == file_hash)
            .chain(KNOWN_DRIVERS.iter().filter(|d| d.sha256 != file_hash));
        let mut last = None;
        for d in ordered {
            match extract(&pe, sys, d.image_va, d.len_va).and_then(|img| verify(img, &[d.image_sha256])) {
                Ok(image) => {
                    log::info!("firmware taken from {} ({} bytes)", d.name, image.len());
                    return Ok(Self { image });
                }
                Err(e) => last = Some(e),
            }
        }
        Err(last.unwrap_or_else(|| Error::Unsupported("no known driver layout".into())))
    }

    /// Accept a raw image whose hash is known.
    pub fn from_image(image: &[u8]) -> Result<Self> {
        verify(image.to_vec(), KNOWN_IMAGES).map(|image| Self { image })
    }

    /// A firmware image that skips verification (tests only).
    #[cfg(test)]
    pub(crate) fn unverified(image: Vec<u8>) -> Self {
        Self { image }
    }

    pub fn image(&self) -> &[u8] {
        &self.image
    }

    pub fn len(&self) -> usize {
        self.image.len()
    }

    pub fn is_empty(&self) -> bool {
        self.image.is_empty()
    }

    pub fn sha256(&self) -> [u8; 32] {
        sha256(&self.image)
    }

    fn word(&self, off: u32) -> Option<u32> {
        let o = off as usize;
        self.image.get(o..o + 4).map(|b| u32::from_le_bytes(b.try_into().unwrap()))
    }

    /// The vector table as stored in the image (count at +0x110).
    pub fn vectors(&self) -> Result<Vec<VectorEntry>> {
        let count = self.word(VECTOR_COUNT_OFFSET).ok_or(Error::Protocol("firmware too short"))?;
        if count == 0xFFFF_FFFF || count > 0x1000 {
            return Err(Error::Protocol("firmware vector table count invalid"));
        }
        (0..count)
            .map(|k| {
                let e = VECTOR_TABLE_OFFSET + 16 * k;
                let w = |i: u32| self.word(e + 4 * i).ok_or(Error::Protocol("firmware vector table truncated"));
                Ok(VectorEntry { dst: w(0)?, words: [w(1)?, w(2)?, w(3)?] })
            })
            .collect()
    }
}

fn verify(image: Vec<u8>, known: &[[u8; 32]]) -> Result<Vec<u8>> {
    let h = sha256(&image);
    if known.contains(&h) {
        Ok(image)
    } else {
        Err(Error::Unsupported(format!("unknown firmware image ({} bytes, sha256 {})", image.len(), hex(&h))))
    }
}

/// The parts of a PE file the extraction needs.
#[derive(Debug)]
struct Pe {
    image_base: u64,
    /// (virtual address, virtual size, raw offset, raw size)
    sections: Vec<(u32, u32, u32, u32)>,
}

fn rd16(d: &[u8], o: usize) -> Result<u16> {
    d.get(o..o + 2).map(|b| u16::from_le_bytes([b[0], b[1]])).ok_or(Error::Protocol("PE file truncated"))
}

fn rd32(d: &[u8], o: usize) -> Result<u32> {
    d.get(o..o + 4).map(|b| u32::from_le_bytes(b.try_into().unwrap())).ok_or(Error::Protocol("PE file truncated"))
}

impl Pe {
    fn parse(d: &[u8]) -> Result<Self> {
        if !d.starts_with(b"MZ") {
            return Err(Error::Protocol("not a PE file (no MZ header)"));
        }
        let pe = rd32(d, 0x3C)? as usize;
        if d.get(pe..pe + 4) != Some(b"PE\0\0") {
            return Err(Error::Protocol("not a PE file (no PE signature)"));
        }
        let nsec = rd16(d, pe + 6)? as usize;
        let opt_size = rd16(d, pe + 20)? as usize;
        let opt = pe + 24;
        let image_base = match rd16(d, opt)? {
            0x10B => u64::from(rd32(d, opt + 28)?),
            0x20B => u64::from(rd32(d, opt + 24)?) | u64::from(rd32(d, opt + 28)?) << 32,
            _ => return Err(Error::Protocol("PE optional header of unknown kind")),
        };
        let mut sections = Vec::with_capacity(nsec);
        for i in 0..nsec {
            let s = opt + opt_size + 40 * i;
            sections.push((rd32(d, s + 12)?, rd32(d, s + 8)?, rd32(d, s + 20)?, rd32(d, s + 16)?));
        }
        Ok(Self { image_base, sections })
    }

    /// File offset of `len` bytes at virtual address `va`, if they lie in
    /// one section's raw data.
    fn offset(&self, va: u32, len: usize) -> Result<usize> {
        let rva =
            u64::from(va).checked_sub(self.image_base).ok_or(Error::Protocol("address below the PE image base"))?;
        for &(sva, vsize, raw, rsize) in &self.sections {
            let (sva, end) = (u64::from(sva), u64::from(sva) + u64::from(vsize.max(rsize)));
            if (sva..end).contains(&rva) {
                let within = rva - sva;
                if within + len as u64 > u64::from(rsize) {
                    return Err(Error::Protocol("firmware not inside the section's file data"));
                }
                return Ok(raw as usize + within as usize);
            }
        }
        Err(Error::Protocol("address not in any PE section"))
    }
}

fn extract(pe: &Pe, sys: &[u8], image_va: u32, len_va: u32) -> Result<Vec<u8>> {
    let len = rd32(sys, pe.offset(len_va, 4)?)? as usize;
    if len == 0 || len > 0x100_0000 {
        return Err(Error::Protocol("implausible firmware length"));
    }
    let off = pe.offset(image_va, len)?;
    sys.get(off..off + len).map(<[u8]>::to_vec).ok_or(Error::Protocol("PE file truncated"))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A synthetic x86 PE: image base 0x10000, `.text` at RVA 0x1000 and a
    /// `.data` section at RVA 0x41000 whose file data starts at 0x600 (so
    /// file offsets differ from RVAs), with `image` at VA 0x51058 and its
    /// length at VA 0x177FE8.
    fn synthetic_pe(image: &[u8]) -> Vec<u8> {
        let data_raw = 0x600usize;
        let data_rva = 0x41000u32;
        let data_len = (0x177FE8 - 0x10000 - data_rva + 4) as usize;
        let mut f = vec![0u8; data_raw + data_len];
        f[0..2].copy_from_slice(b"MZ");
        f[0x3C..0x40].copy_from_slice(&0x80u32.to_le_bytes());
        f[0x80..0x84].copy_from_slice(b"PE\0\0");
        f[0x84..0x86].copy_from_slice(&0x14Cu16.to_le_bytes());
        f[0x86..0x88].copy_from_slice(&2u16.to_le_bytes());
        f[0x94..0x96].copy_from_slice(&0xE0u16.to_le_bytes());
        let opt = 0x98;
        f[opt..opt + 2].copy_from_slice(&0x10Bu16.to_le_bytes());
        f[opt + 28..opt + 32].copy_from_slice(&0x10000u32.to_le_bytes());
        let mut sec = |i: usize, name: &[u8], va: u32, vs: u32, raw: u32, rs: u32| {
            let s = opt + 0xE0 + 40 * i;
            f[s..s + name.len()].copy_from_slice(name);
            f[s + 8..s + 12].copy_from_slice(&vs.to_le_bytes());
            f[s + 12..s + 16].copy_from_slice(&va.to_le_bytes());
            f[s + 16..s + 20].copy_from_slice(&rs.to_le_bytes());
            f[s + 20..s + 24].copy_from_slice(&raw.to_le_bytes());
        };
        sec(0, b".text", 0x1000, 0x100, 0x400, 0x200);
        sec(1, b".data", data_rva, data_len as u32, data_raw as u32, data_len as u32);
        let img_off = data_raw + (0x51058 - 0x10000 - data_rva) as usize;
        f[img_off..img_off + image.len()].copy_from_slice(image);
        let len_off = data_raw + (0x177FE8 - 0x10000 - data_rva) as usize;
        f[len_off..len_off + 4].copy_from_slice(&(image.len() as u32).to_le_bytes());
        f
    }

    fn test_image() -> Vec<u8> {
        let mut img: Vec<u8> = (0..0x400u32).flat_map(|i| i.wrapping_mul(0x0101_0101).to_le_bytes()).collect();
        img[0x110..0x114].copy_from_slice(&2u32.to_le_bytes());
        for (k, e) in [[0x180u32, 1, 2, 3], [0x200, 4, 5, 6]].iter().enumerate() {
            for (i, w) in e.iter().enumerate() {
                let o = 0x120 + 16 * k + 4 * i;
                img[o..o + 4].copy_from_slice(&w.to_le_bytes());
            }
        }
        img
    }

    #[test]
    fn hex_constants() {
        assert_eq!(FW_SHA256[0], 0x4E);
        assert_eq!(FW_SHA256[31], 0x5D);
        assert_eq!(hex(&KNOWN_DRIVERS[0].sha256), "60872bd9225a8fd7ef402b83ccbcbae58edee40d05c73da9eba8bba90dc4e286");
    }

    #[test]
    fn extracts_through_the_section_table() {
        let img = test_image();
        let sys = synthetic_pe(&img);
        let pe = Pe::parse(&sys).unwrap();
        assert_eq!(pe.image_base, 0x10000);
        assert_eq!(pe.offset(0x51058, 4).unwrap(), 0x600 + 0x58);
        let got = extract(&pe, &sys, 0x51058, 0x177FE8).unwrap();
        assert_eq!(got, img);
        // The same bytes pass when their hash is the expected one.
        assert_eq!(verify(got, &[sha256(&img)]).unwrap(), img);
    }

    #[test]
    fn unknown_images_are_refused() {
        let img = test_image();
        let sys = synthetic_pe(&img);
        let e = Firmware::from_driver(&sys).unwrap_err();
        assert!(matches!(e, Error::Unsupported(ref m) if m.contains(&hex(&sha256(&img)))), "{e:?}");
        assert!(Firmware::from_image(&img).is_err());
        assert!(Firmware::from_bytes(&img).is_err());
    }

    #[test]
    fn malformed_files() {
        assert!(Pe::parse(b"MZ").is_err());
        let mut sys = synthetic_pe(&test_image());
        // Length pointing past the section's file data.
        let len_off = 0x600 + (0x177FE8 - 0x10000 - 0x41000) as usize;
        sys[len_off..len_off + 4].copy_from_slice(&0x00F0_0000u32.to_le_bytes());
        let pe = Pe::parse(&sys).unwrap();
        assert!(extract(&pe, &sys, 0x51058, 0x177FE8).is_err());
        // Address outside every section.
        assert!(pe.offset(0x0100_0000, 4).is_err());
        assert!(pe.offset(0x100, 4).is_err());
    }

    #[test]
    fn vector_table_from_image() {
        let fw = Firmware::unverified(test_image());
        assert_eq!(
            fw.vectors().unwrap(),
            vec![VectorEntry { dst: 0x180, words: [1, 2, 3] }, VectorEntry { dst: 0x200, words: [4, 5, 6] }]
        );
        let mut img = test_image();
        img[0x110..0x114].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(Firmware::unverified(img).vectors().is_err());
    }

    /// Checks against the user's own copy of the vendor driver when it is
    /// present (`DTH30_DRIVER` = path of BufFsUSB.sys); skipped otherwise.
    #[test]
    fn real_driver_if_available() {
        let Ok(path) = std::env::var("DTH30_DRIVER") else { return };
        let fw = Firmware::load(path).unwrap();
        assert_eq!(fw.len(), FW_LEN);
        assert_eq!(fw.sha256(), FW_SHA256);
        assert_eq!(fw.vectors().unwrap().len(), 6);
    }
}
