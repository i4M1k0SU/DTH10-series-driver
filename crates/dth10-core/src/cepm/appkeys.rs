//! Application-side CEPM credentials, read at run time from the user's own
//! copy of the vendor application's `CoBDACom.dll` (PCastTV3). None of that
//! data is part of this crate: the loader interprets the DLL's x86 code just
//! far enough to recover the values the application builds at run time, and
//! proves each value right either algebraically or against the device's own
//! key packets.
//!
//! What is recovered and how it is verified:
//!
//! * the application certificate (104 bytes: 24-byte header, secp160r1
//!   public key `Q`, ECDSA self-signature) and the matching private key `d`
//!   (20 bytes). The application writes both into its data segment byte by
//!   byte with immediate-store instructions (`mov byte ptr [addr], imm8`,
//!   `mov al, imm8; mov [addr], al`, ...). The loader replays every such store
//!   it can decode into a sparse image, finds the certificate by its
//!   self-signature and the key by `d·G == Q`;
//! * the CBC IV of the content-key unwrap (16 bytes), which the application
//!   assembles on the stack with immediate stores and then reverses. Every
//!   16-byte stack buffer the code assembles is a candidate (as stored and
//!   reversed); the right one is the one under which the first content blob's
//!   CMAC verifies ([`AppKeys::find_content_iv`]).
//!
//! The decoder is a pattern scanner, not a disassembler: it recognises only
//! the store forms above and skips everything else byte by byte, so it also
//! picks up garbage. The verifications make that harmless.

use std::collections::{BTreeMap, HashSet};
use std::path::Path;
use std::sync::Mutex;

use crate::LockExt;
use aes::Aes128;
use aes::cipher::{BlockCipherDecrypt, KeyInit};
use num_bigint::BigUint;
use num_traits::Zero;

use super::aes_cmac;
use super::ec::{Curve, Point};
use crate::error::{Error, Result};

/// Length of a certificate (header 24, public key 40, signature 40).
pub const CERT_LEN: usize = 104;
/// Length of a private key.
pub const PRIV_LEN: usize = 20;

/// The application credentials and the material needed to settle the IV.
pub struct AppKeys {
    /// The application certificate sent in handshake step 1.
    pub cert: [u8; CERT_LEN],
    /// The application private key (big-endian scalar) that signs step 4.
    pub priv_key: [u8; PRIV_LEN],
    /// Where the data came from (for messages).
    pub source: String,
    /// 16-byte stack buffers assembled by the code, as stored and reversed.
    iv_candidates: Vec<[u8; 16]>,
    /// Raw bytes of the non-code sections, the last resort for the IV.
    data: Vec<u8>,
    /// The content IV once settled.
    iv_content: Mutex<Option<[u8; 16]>>,
}

impl std::fmt::Debug for AppKeys {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AppKeys")
            .field("source", &self.source)
            .field("iv_candidates", &self.iv_candidates.len())
            .field("iv_settled", &self.content_iv().is_some())
            .finish()
    }
}

impl AppKeys {
    /// Read `path` (the application's `CoBDACom.dll`) and recover the credentials.
    pub fn load(path: &Path) -> Result<AppKeys> {
        let bytes = std::fs::read(path).map_err(|e| Error::AppKeys(format!("cannot read {}: {e}", path.display())))?;
        Self::from_image(&bytes, &path.display().to_string())
    }

    /// Recover the credentials from the bytes of a PE image.
    pub fn from_image(bytes: &[u8], source: &str) -> Result<AppKeys> {
        let pe = parse_pe(bytes).map_err(|m| Error::AppKeys(format!("{source}: {m}")))?;
        let mut code = Vec::new();
        let mut data = Vec::new();
        let mut code_va = None;
        for s in &pe.sections {
            let end = (s.raw as usize).saturating_add(s.rsize as usize).min(bytes.len());
            let raw = bytes.get(s.raw as usize..end).unwrap_or(&[]);
            if s.is_code() {
                if code_va.is_none() {
                    code_va = Some(pe.image_base + s.va as u64);
                }
                code.extend_from_slice(raw);
            } else {
                data.extend_from_slice(raw);
            }
        }
        if code.is_empty() {
            return Err(Error::AppKeys(format!("{source}: no code section")));
        }
        Self::from_parts(&code, pe.image_base, pe.image_size(), data, source)
    }

    /// The core: `code` is the executable section(s), stores to absolute
    /// addresses inside `[image_base, image_base + image_size)` build the data image.
    fn from_parts(code: &[u8], image_base: u64, image_size: u64, data: Vec<u8>, source: &str) -> Result<AppKeys> {
        let stores = scan_stores(code);
        let image_end = image_base.saturating_add(image_size);
        let mut first: BTreeMap<u64, u8> = BTreeMap::new();
        let mut last: BTreeMap<u64, u8> = BTreeMap::new();
        for s in &stores {
            if let Target::Abs(a) = s.target {
                if a < image_base || a.saturating_add(s.len as u64) > image_end {
                    continue;
                }
                for k in 0..s.len {
                    first.entry(a + k as u64).or_insert(s.bytes[k]);
                    last.insert(a + k as u64, s.bytes[k]);
                }
            }
        }
        let curve = Curve::secp160r1();
        let (cert_addr, cert, q) = find_certificate(&curve, &first).ok_or_else(|| {
            Error::AppKeys(format!("{source}: no application certificate found (is this CoBDACom.dll of PCastTV3?)"))
        })?;
        let priv_key = find_private_key(&curve, &q, cert_addr, &first, &last).ok_or_else(|| {
            Error::AppKeys(format!("{source}: application private key not found next to the certificate"))
        })?;
        let iv_candidates = stack_buffers(&stores);
        log::debug!(
            "appkeys: {source}: certificate at 0x{cert_addr:x}, {} IV candidates, {} data bytes",
            iv_candidates.len(),
            data.len()
        );
        Ok(AppKeys { cert, priv_key, source: source.to_owned(), iv_candidates, data, iv_content: Mutex::new(None) })
    }

    /// The private key as a scalar.
    pub fn app_priv(&self) -> BigUint {
        BigUint::from_bytes_be(&self.priv_key)
    }

    /// Number of IV candidates taken from the code.
    pub fn iv_candidates(&self) -> usize {
        self.iv_candidates.len()
    }

    /// The content IV, once [`AppKeys::find_content_iv`] settled it.
    pub fn content_iv(&self) -> Option<[u8; 16]> {
        *self.iv_content.locked()
    }

    /// Settle the content IV with a 64-byte content blob (`{id 16, wrapped 32,
    /// cmac 16}`) and the session's K3: the IV under which the unwrapped CMAC
    /// key authenticates the blob. Cached once found.
    pub fn find_content_iv(&self, k3: &[u8; 16], blob: &[u8]) -> Option<[u8; 16]> {
        if let Some(iv) = self.content_iv() {
            return Some(iv);
        }
        if blob.len() < 64 {
            return None;
        }
        // CBC: D[0..16] = AES_dec(c0) ^ IV, and D[0..16] is the CMAC key, so
        // the IV test costs one CMAC per candidate.
        let cipher = Aes128::new(k3.into());
        let mut d0 = aes::Block::try_from(&blob[16..32]).expect("16-byte block");
        cipher.decrypt_block(&mut d0);
        let test = |iv: &[u8; 16]| {
            let mut k = [0u8; 16];
            for i in 0..16 {
                k[i] = d0[i] ^ iv[i];
            }
            aes_cmac(&k, &blob[..48]) == blob[48..64]
        };
        let found = self.iv_candidates.iter().find(|c| test(c)).copied().or_else(|| {
            log::debug!("appkeys: content IV not among the stack buffers, scanning the data sections");
            self.data.windows(16).find_map(|w| {
                let mut iv = [0u8; 16];
                iv.copy_from_slice(w);
                if test(&iv) {
                    return Some(iv);
                }
                iv.reverse();
                test(&iv).then_some(iv)
            })
        });
        if let Some(iv) = found {
            *self.iv_content.locked() = Some(iv);
            log::info!("appkeys: content IV settled from {}", self.source);
        }
        found
    }
}

// ---------------------------------------------------------------- PE image

struct Section {
    va: u32,
    vsize: u32,
    raw: u32,
    rsize: u32,
    characteristics: u32,
    name: [u8; 8],
}

impl Section {
    fn is_code(&self) -> bool {
        const CNT_CODE: u32 = 0x20;
        const MEM_EXECUTE: u32 = 0x2000_0000;
        self.characteristics & (CNT_CODE | MEM_EXECUTE) != 0 || self.name.starts_with(b".text")
    }
}

struct Pe {
    image_base: u64,
    sections: Vec<Section>,
}

impl Pe {
    fn image_size(&self) -> u64 {
        self.sections.iter().map(|s| s.va as u64 + s.vsize.max(s.rsize) as u64).max().unwrap_or(0)
    }
}

fn rd16(b: &[u8], o: usize) -> Option<u16> {
    Some(u16::from_le_bytes(b.get(o..o + 2)?.try_into().ok()?))
}

fn rd32(b: &[u8], o: usize) -> Option<u32> {
    Some(u32::from_le_bytes(b.get(o..o + 4)?.try_into().ok()?))
}

fn rd64(b: &[u8], o: usize) -> Option<u64> {
    Some(u64::from_le_bytes(b.get(o..o + 8)?.try_into().ok()?))
}

fn parse_pe(b: &[u8]) -> std::result::Result<Pe, &'static str> {
    if b.get(..2) != Some(b"MZ") {
        return Err("not a PE file (no MZ header)");
    }
    let pe = rd32(b, 0x3C).ok_or("truncated DOS header")? as usize;
    if b.get(pe..pe + 4) != Some(b"PE\0\0") {
        return Err("not a PE file (no PE signature)");
    }
    let nsec = rd16(b, pe + 6).ok_or("truncated COFF header")? as usize;
    let opt_size = rd16(b, pe + 20).ok_or("truncated COFF header")? as usize;
    let opt = pe + 24;
    let magic = rd16(b, opt).ok_or("truncated optional header")?;
    let image_base = match magic {
        0x10B => rd32(b, opt + 28).ok_or("truncated optional header")? as u64,
        0x20B => rd64(b, opt + 24).ok_or("truncated optional header")?,
        _ => return Err("unknown optional header magic"),
    };
    let mut sections = Vec::with_capacity(nsec);
    for i in 0..nsec {
        let o = opt + opt_size + i * 40;
        let hdr = b.get(o..o + 40).ok_or("truncated section table")?;
        let mut name = [0u8; 8];
        name.copy_from_slice(&hdr[..8]);
        sections.push(Section {
            vsize: rd32(hdr, 8).unwrap(),
            va: rd32(hdr, 12).unwrap(),
            rsize: rd32(hdr, 16).unwrap(),
            raw: rd32(hdr, 20).unwrap(),
            characteristics: rd32(hdr, 36).unwrap(),
            name,
        });
    }
    if sections.is_empty() {
        return Err("no sections");
    }
    Ok(Pe { image_base, sections })
}

// ---------------------------------------------------------------- x86 stores

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Target {
    /// `[imm32]`
    Abs(u64),
    /// `[base + disp]` with `base` = 4 (esp) or 5 (ebp): a stack slot.
    Stack(u8, i32),
}

#[derive(Clone, Copy, Debug)]
struct Store {
    /// Offset of the instruction in the code.
    pos: usize,
    target: Target,
    bytes: [u8; 4],
    len: usize,
}

/// How far back (in code bytes) a `mov reg, imm` is trusted to still hold:
/// the scanner does not decode the instructions in between. The application
/// keeps repeated byte values in registers across its whole store sequence
/// (over a thousand bytes of code), so the window spans a function.
const REG_FRESH: usize = 8192;

#[derive(Default, Clone, Copy)]
struct Regs {
    r8: [Option<(u8, usize)>; 8],
    r16: [Option<(u16, usize)>; 8],
    r32: [Option<(u32, usize)>; 8],
}

impl Regs {
    fn set8(&mut self, r: usize, v: u8, pos: usize) {
        self.r8[r] = Some((v, pos));
        self.r16[r & 3] = None;
        self.r32[r & 3] = None;
    }

    fn set16(&mut self, r: usize, v: u16, pos: usize) {
        self.r16[r] = Some((v, pos));
        self.r32[r] = None;
        if r < 4 {
            self.r8[r] = Some((v as u8, pos));
            self.r8[r + 4] = Some(((v >> 8) as u8, pos));
        }
    }

    fn set32(&mut self, r: usize, v: u32, pos: usize) {
        self.r32[r] = Some((v, pos));
        self.r16[r] = Some((v as u16, pos));
        if r < 4 {
            self.r8[r] = Some((v as u8, pos));
            self.r8[r + 4] = Some(((v >> 8) as u8, pos));
        }
    }

    fn fresh<T: Copy>(slot: Option<(T, usize)>, pos: usize) -> Option<T> {
        slot.filter(|(_, p)| pos.saturating_sub(*p) <= REG_FRESH).map(|(v, _)| v)
    }

    fn get8(&self, r: usize, pos: usize) -> Option<u8> {
        Self::fresh(self.r8[r], pos)
    }

    fn get16(&self, r: usize, pos: usize) -> Option<u16> {
        Self::fresh(self.r16[r], pos)
    }

    fn get32(&self, r: usize, pos: usize) -> Option<u32> {
        Self::fresh(self.r32[r], pos)
    }
}

/// ModRM (+ SIB) at `code[i]` for the memory forms we care about:
/// `[disp32]`, `[ebp+disp8/32]`, and through a SIB byte without index
/// `[esp+disp8/32]`, `[ebp+disp8/32]`, `[disp32]`. Returns the target and
/// the bytes consumed.
fn modrm_target(code: &[u8], i: usize) -> Option<(Target, usize)> {
    let b = *code.get(i)?;
    let md = b >> 6;
    match (md, b & 7) {
        (0, 5) => Some((Target::Abs(rd32(code, i + 1)? as u64), 5)),
        (1, 5) => Some((Target::Stack(5, *code.get(i + 1)? as i8 as i32), 2)),
        (2, 5) => Some((Target::Stack(5, rd32(code, i + 1)? as i32), 5)),
        (_, 4) => {
            let sib = *code.get(i + 1)?;
            let (index, base) = ((sib >> 3) & 7, sib & 7);
            if index != 4 {
                return None; // indexed addressing: not a plain slot
            }
            match (md, base) {
                (0, 5) => Some((Target::Abs(rd32(code, i + 2)? as u64), 6)),
                (0, 4) => Some((Target::Stack(4, 0), 2)),
                (1, 4 | 5) => Some((Target::Stack(base, *code.get(i + 2)? as i8 as i32), 3)),
                (2, 4 | 5) => Some((Target::Stack(base, rd32(code, i + 2)? as i32), 6)),
                _ => None,
            }
        }
        _ => None,
    }
}

fn modrm_reg(code: &[u8], i: usize) -> usize {
    ((code[i] >> 3) & 7) as usize
}

fn plausible(t: Target) -> bool {
    match t {
        Target::Abs(_) => true,
        Target::Stack(_, d) => (-0x10000..0x10000).contains(&d),
    }
}

/// Every immediate-to-memory store the scanner recognises, in code order.
fn scan_stores(code: &[u8]) -> Vec<Store> {
    let mut out = Vec::new();
    let mut regs = Regs::default();
    let mut i = 0;
    let mut push = |pos: usize, target: Target, v: u32, len: usize| {
        if plausible(target) {
            out.push(Store { pos, target, bytes: v.to_le_bytes(), len });
        }
    };
    while i < code.len() {
        let op = code[i];
        let mut step = 1;
        match op {
            0xB0..=0xB7 => {
                if let Some(&v) = code.get(i + 1) {
                    regs.set8((op - 0xB0) as usize, v, i);
                    step = 2;
                }
            }
            0xB8..=0xBF => {
                if let Some(v) = rd32(code, i + 1) {
                    regs.set32((op - 0xB8) as usize, v, i);
                    step = 5;
                }
            }
            0x66 => match code.get(i + 1).copied() {
                Some(o2 @ 0xB8..=0xBF) => {
                    if let Some(v) = rd16(code, i + 2) {
                        regs.set16((o2 - 0xB8) as usize, v, i);
                        step = 4;
                    }
                }
                Some(0xC7) => {
                    if let Some((t, n)) = modrm_target(code, i + 2)
                        && modrm_reg(code, i + 2) == 0
                        && let Some(v) = rd16(code, i + 2 + n)
                    {
                        push(i, t, v as u32, 2);
                        step = 2 + n + 2;
                    }
                }
                Some(0x89) => {
                    if let Some((t, n)) = modrm_target(code, i + 2)
                        && let Some(v) = regs.get16(modrm_reg(code, i + 2), i)
                    {
                        push(i, t, v as u32, 2);
                        step = 2 + n;
                    }
                }
                Some(0xA3) => {
                    if let (Some(a), Some(v)) = (rd32(code, i + 2), regs.get16(0, i)) {
                        push(i, Target::Abs(a as u64), v as u32, 2);
                        step = 6;
                    }
                }
                _ => {}
            },
            0xC6 => {
                if let Some((t, n)) = modrm_target(code, i + 1)
                    && modrm_reg(code, i + 1) == 0
                    && let Some(&v) = code.get(i + 1 + n)
                {
                    push(i, t, v as u32, 1);
                    step = 1 + n + 1;
                }
            }
            0xC7 => {
                if let Some((t, n)) = modrm_target(code, i + 1)
                    && modrm_reg(code, i + 1) == 0
                    && let Some(v) = rd32(code, i + 1 + n)
                {
                    push(i, t, v, 4);
                    step = 1 + n + 4;
                }
            }
            0x88 => {
                if let Some((t, n)) = modrm_target(code, i + 1)
                    && let Some(v) = regs.get8(modrm_reg(code, i + 1), i)
                {
                    push(i, t, v as u32, 1);
                    step = 1 + n;
                }
            }
            0x89 => {
                if let Some((t, n)) = modrm_target(code, i + 1)
                    && let Some(v) = regs.get32(modrm_reg(code, i + 1), i)
                {
                    push(i, t, v, 4);
                    step = 1 + n;
                }
            }
            0xA2 => {
                if let (Some(a), Some(v)) = (rd32(code, i + 1), regs.get8(0, i)) {
                    push(i, Target::Abs(a as u64), v as u32, 1);
                    step = 5;
                }
            }
            0xA3 => {
                if let (Some(a), Some(v)) = (rd32(code, i + 1), regs.get32(0, i)) {
                    push(i, Target::Abs(a as u64), v, 4);
                    step = 5;
                }
            }
            _ => {}
        }
        i += step;
    }
    out
}

/// Contiguous runs of written addresses.
fn runs(image: &BTreeMap<u64, u8>) -> Vec<(u64, Vec<u8>)> {
    let mut out: Vec<(u64, Vec<u8>)> = Vec::new();
    for (&a, &b) in image {
        match out.last_mut() {
            Some((start, bytes)) if *start + bytes.len() as u64 == a => bytes.push(b),
            _ => out.push((a, vec![b])),
        }
    }
    out
}

/// The first 104-byte window whose bytes 24..64 are a point on the curve that
/// verifies the signature in bytes 64..104 over bytes 0..64.
fn find_certificate(curve: &Curve, image: &BTreeMap<u64, u8>) -> Option<(u64, [u8; CERT_LEN], Point)> {
    for (start, bytes) in runs(image) {
        if bytes.len() < CERT_LEN {
            continue;
        }
        for o in 0..=bytes.len() - CERT_LEN {
            let c = &bytes[o..o + CERT_LEN];
            let Some(q) = Point::from_bytes(curve, &c[24..64]) else { continue };
            if curve.verify(&q, &c[..64], &c[64..CERT_LEN]) {
                let mut cert = [0u8; CERT_LEN];
                cert.copy_from_slice(c);
                return Some((start + o as u64, cert, q));
            }
        }
    }
    None
}

fn key_matches(curve: &Curve, d: &[u8], q: &Point) -> bool {
    let d = BigUint::from_bytes_be(d);
    if d.is_zero() || d >= curve.n {
        return false;
    }
    curve.public_key(&d) == *q
}

/// The private key: right after the certificate (first or last value
/// written there), else any 20-byte window of the written image near it.
fn find_private_key(
    curve: &Curve,
    q: &Point,
    cert_addr: u64,
    first: &BTreeMap<u64, u8>,
    last: &BTreeMap<u64, u8>,
) -> Option<[u8; PRIV_LEN]> {
    let at = |image: &BTreeMap<u64, u8>, a: u64| -> Option<[u8; PRIV_LEN]> {
        let mut k = [0u8; PRIV_LEN];
        for (i, b) in k.iter_mut().enumerate() {
            *b = *image.get(&(a + i as u64))?;
        }
        Some(k)
    };
    let after = cert_addr + CERT_LEN as u64;
    for image in [first, last] {
        if let Some(k) = at(image, after)
            && key_matches(curve, &k, q)
        {
            return Some(k);
        }
    }
    for image in [first, last] {
        for (start, bytes) in runs(image) {
            if start.abs_diff(cert_addr) > 0x10000 || bytes.len() < PRIV_LEN {
                continue;
            }
            for w in bytes.windows(PRIV_LEN) {
                if key_matches(curve, w, q) {
                    let mut k = [0u8; PRIV_LEN];
                    k.copy_from_slice(w);
                    return Some(k);
                }
            }
        }
    }
    None
}

/// Every complete 16-byte stack buffer assembled by nearby stores (first
/// write wins per byte), as stored and reversed.
fn stack_buffers(stores: &[Store]) -> Vec<[u8; 16]> {
    const WINDOW_CODE_BYTES: usize = 400;
    const LIMIT: usize = 50_000;
    let stack: Vec<&Store> = stores.iter().filter(|s| matches!(s.target, Target::Stack(..))).collect();
    let mut seen = HashSet::new();
    let mut out = Vec::new();
    for (i, head) in stack.iter().enumerate() {
        // slots keyed by (base register, displacement)
        let mut map: BTreeMap<(u8, i32), u8> = BTreeMap::new();
        for s in &stack[i..] {
            if s.pos - head.pos > WINDOW_CODE_BYTES {
                break;
            }
            if let Target::Stack(base, d) = s.target {
                for k in 0..s.len {
                    map.entry((base, d + k as i32)).or_insert(s.bytes[k]);
                }
            }
        }
        let starts: Vec<(u8, i32)> = map.keys().copied().collect();
        for (base, s0) in starts {
            let mut buf = [0u8; 16];
            let complete = (0..16).all(|k| match map.get(&(base, s0 + k)) {
                Some(&b) => {
                    buf[k as usize] = b;
                    true
                }
                None => false,
            });
            if !complete {
                continue;
            }
            if seen.insert(buf) {
                out.push(buf);
            }
            let mut rev = buf;
            rev.reverse();
            if seen.insert(rev) {
                out.push(rev);
            }
            if out.len() >= LIMIT {
                return out;
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use aes::cipher::BlockCipherEncrypt;

    const BASE: u64 = 0x1000_0000;

    /// A fake data segment: the certificate at `cert_addr`, the key after it.
    fn cert_and_key(curve: &Curve) -> ([u8; CERT_LEN], [u8; PRIV_LEN]) {
        let d = curve.random_scalar();
        let q = curve.public_key(&d).to_bytes().unwrap();
        let mut cert = [0u8; CERT_LEN];
        cert[..4].copy_from_slice(&[0x11, 0x10, 0xFF, 0xFF]);
        cert[4..24].copy_from_slice(&[0xAB; 20]);
        cert[24..64].copy_from_slice(&q);
        let sig = curve.sign(&d, &cert[..64]);
        cert[64..].copy_from_slice(&sig);
        (cert, super::super::ec::to_fixed(&d))
    }

    /// Emit the application's style of stores for `bytes` at `addr`: a mix of
    /// `mov byte [abs], imm8`, `mov al, imm8; mov [abs], al` and `mov bl, imm8;
    /// mov [abs], bl`, plus dword stores for the last 20 bytes.
    fn emit_data_stores(code: &mut Vec<u8>, addr: u32, bytes: &[u8]) {
        let mut i = 0;
        while i < bytes.len() {
            let a = (addr + i as u32).to_le_bytes();
            match i % 4 {
                0 if bytes.len() - i >= 4 && i >= bytes.len() - 20 => {
                    code.push(0xC7);
                    code.push(0x05);
                    code.extend_from_slice(&a);
                    code.extend_from_slice(&bytes[i..i + 4]);
                    code.extend_from_slice(&[0x90, 0x8B, 0xFF]); // junk between stores
                    i += 4;
                    continue;
                }
                0 => {
                    code.extend_from_slice(&[0xC6, 0x05]);
                    code.extend_from_slice(&a);
                    code.push(bytes[i]);
                }
                1 => {
                    code.extend_from_slice(&[0xB0, bytes[i], 0xA2]);
                    code.extend_from_slice(&a);
                }
                2 => {
                    code.extend_from_slice(&[0xB3, bytes[i], 0x88, 0x1D]);
                    code.extend_from_slice(&a);
                }
                _ => {
                    code.extend_from_slice(&[0xB8, bytes[i], 0x00, 0x00, 0x00, 0xA2]);
                    code.extend_from_slice(&a);
                }
            }
            i += 1;
        }
    }

    /// Sixteen `mov byte [esp+disp32], imm8` (the application's form, the IV
    /// stored reversed), then a decoy ebp-relative buffer built from a dword
    /// store and twelve byte stores, with junk around.
    fn emit_stack_iv(code: &mut Vec<u8>, stored: &[u8; 16]) {
        code.extend_from_slice(&[0x56, 0x57, 0x8B, 0xBC, 0x24, 0xD0, 0x03, 0x00, 0x00, 0x8B, 0xF1]);
        for (k, b) in stored.iter().enumerate() {
            code.extend_from_slice(&[0xC6, 0x84, 0x24]);
            code.extend_from_slice(&(0x3A4u32 + k as u32).to_le_bytes());
            code.push(*b);
        }
        code.extend_from_slice(&[0xC7, 0x44, 0x24, 0x14, 0x05, 0x40, 0x00, 0x80]);
        code.extend_from_slice(&[0xC7, 0x45, 0xDC, 1, 2, 3, 4]);
        for k in 0..12u8 {
            code.extend_from_slice(&[0xC6, 0x45, 0xE0 + k, 0x10 + k]);
            if k % 3 == 0 {
                code.extend_from_slice(&[0x33, 0xC0]);
            }
        }
    }

    fn cbc_encrypt(key: &[u8; 16], iv: &[u8; 16], data: &mut [u8]) {
        let cipher = Aes128::new(key.into());
        let mut prev = *iv;
        for block in data.as_chunks_mut::<16>().0 {
            for i in 0..16 {
                block[i] ^= prev[i];
            }
            let mut b = aes::Block::from(*block);
            cipher.encrypt_block(&mut b);
            block.copy_from_slice(&b);
            prev.copy_from_slice(&b);
        }
    }

    #[test]
    fn recovers_certificate_key_and_iv() {
        let curve = Curve::secp160r1();
        let (cert, priv_key) = cert_and_key(&curve);
        let iv: [u8; 16] = rand::random();
        let mut stored = iv;
        stored.reverse();
        let mut code = vec![0xCC; 37];
        emit_stack_iv(&mut code, &stored);
        code.extend_from_slice(&[0xE8, 0x12, 0x34, 0x56, 0x78]);
        let mut blob = Vec::new();
        blob.extend_from_slice(&cert);
        blob.extend_from_slice(&priv_key);
        emit_data_stores(&mut code, 0x1005_2630, &blob);
        // the application zeroes the key later: last write must not win
        code.extend_from_slice(&[0xC7, 0x05, 0x98, 0x26, 0x05, 0x10, 0, 0, 0, 0]);
        code.extend_from_slice(&[0xC3; 9]);
        let keys = AppKeys::from_parts(&code, BASE, 0x6_0000, vec![0x5A; 300], "test").expect("recovered");
        assert_eq!(keys.cert, cert);
        assert_eq!(keys.priv_key, priv_key);
        assert!(keys.iv_candidates() > 0);
        assert!(keys.content_iv().is_none());

        // a content blob under that IV: D = {cmac key, ctr key}
        let secret = [0x33u8; 20];
        let k3: [u8; 16] = super::super::kdf(&secret, 3)[..16].try_into().unwrap();
        let d: [u8; 32] = rand::random();
        let mut wrapped = d;
        cbc_encrypt(&k3, &iv, &mut wrapped);
        let mut content = [0u8; 64];
        content[..16].copy_from_slice(&[0x77; 16]);
        content[16..48].copy_from_slice(&wrapped);
        let mac = aes_cmac(&d[..16].try_into().unwrap(), &content[..48]);
        content[48..].copy_from_slice(&mac);
        assert_eq!(keys.find_content_iv(&k3, &content), Some(iv));
        assert_eq!(keys.content_iv(), Some(iv));
        // and the session can unwrap with it
        let s = super::super::Session { id: [0; 16], secret };
        assert_eq!(s.unwrap_content(&iv, &content).unwrap(), d);
    }

    #[test]
    fn iv_falls_back_to_data_sections() {
        let curve = Curve::secp160r1();
        let (cert, priv_key) = cert_and_key(&curve);
        let mut code = Vec::new();
        let mut blob = Vec::new();
        blob.extend_from_slice(&cert);
        blob.extend_from_slice(&priv_key);
        emit_data_stores(&mut code, 0x1005_0100, &blob);
        let iv: [u8; 16] = rand::random();
        let mut data = vec![0u8; 100];
        data.extend_from_slice(&iv);
        data.extend_from_slice(&[1u8; 50]);
        let keys = AppKeys::from_parts(&code, BASE, 0x6_0000, data, "test").unwrap();
        let k3 = [9u8; 16];
        let d = [4u8; 32];
        let mut wrapped = d;
        cbc_encrypt(&k3, &iv, &mut wrapped);
        let mut content = [0u8; 64];
        content[16..48].copy_from_slice(&wrapped);
        let mac = aes_cmac(&d[..16].try_into().unwrap(), &content[..48]);
        content[48..].copy_from_slice(&mac);
        assert_eq!(keys.find_content_iv(&k3, &content), Some(iv));
        assert_eq!(keys.find_content_iv(&[0; 16], &content), Some(iv), "cached");
    }

    #[test]
    fn rejects_images_without_credentials() {
        let code = vec![0x90u8; 1000];
        assert!(AppKeys::from_parts(&code, BASE, 0x1000, Vec::new(), "test").is_err());
        assert!(AppKeys::from_image(b"not a dll", "test").is_err());
    }

    /// With `DTH10_APP_KEYS` pointing at the application's DLL, the real
    /// credentials are recovered and verified (the loader checks the
    /// self-signature and the key; this only exercises the PE path).
    #[test]
    fn real_dll_when_available() {
        let Ok(p) = std::env::var("DTH10_APP_KEYS") else { return };
        let keys = AppKeys::load(Path::new(&p)).expect("credentials recovered from the DLL");
        assert_eq!(&keys.cert[..2], &[0x11, 0x10], "certificate format tag");
        assert!(keys.iv_candidates() > 0);
    }
}
