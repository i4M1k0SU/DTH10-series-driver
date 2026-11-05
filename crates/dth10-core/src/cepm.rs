//! CEPM: the link encryption between the EVA01 bridge and the host application
//! used when the stream is enabled in the original driver's mode (stream-enable
//! byte0 = 0, [`crate::StreamMode::EncryptedPartial`]), as the vendor application
//! implements it (docs/22-link-encryption.md).
//!
//! Summary:
//! * every TS packet with `transport_scrambling_control != 0` has its 184
//!   bytes after the header encrypted with **AES-128-CTR** (per-PID counter,
//!   continuous key stream);
//! * keys arrive in-band as adaptation-field private data ("key packets"):
//!   type 0xEC carries a 96-byte session blob that only the *device* can
//!   unwrap (vendor command 0xDEAF/7), type 0xC0..0xCF a 64-byte content blob
//!   unwrapped host-side with AES-CBC + AES-CMAC;
//! * the stream is bound to a mutual-authentication handshake (ECDSA + ECDH
//!   on secp160r1, vendor commands 0xDEAF/1..6) that needs the application's
//!   certificate and private key. Those, and the content IV, are the vendor
//!   application's data and are not part of this crate: [`AppKeys`] reads
//!   them at run time from the user's own copy of `CoBDACom.dll` (see
//!   [`appkeys`]). The session secret is the ECDH shared secret; the device
//!   side unwrap (0xDEAF/7) is never used (docs/22 §6c).
//!
//! All vendor commands of this module go through the tuner/filter #1 request
//! path (bRequest bit 7, [`FLAGS`]); the device STALLs them on the #0 path
//! (verified on a DT-H70/U2).

pub mod appkeys;
pub mod ec;

pub use appkeys::AppKeys;

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use aes::Aes128;
use aes::cipher::{BlockCipherDecrypt, BlockCipherEncrypt, KeyInit};
use cmac::{Cmac, Mac};
use sha1::{Digest, Sha1};

use crate::error::{Error, Result};
use crate::protocol::{self, UsbControl};
use ec::{Curve, Point};

/// wValue of every CEPM vendor command; wIndex selects the step.
pub const CMD_CEPM: u16 = 0xDEAF;
/// Request flag bits of the tuner/filter #1 path (bRequest bit 7).
///
/// Each tuner path has its own security context (own device certificate,
/// header byte 12 = tuner index) and the TS of a tuner is bound to the
/// handshake done on that tuner's path. On a freshly powered device only the
/// #1 path answers 0xDEAF; the #0 path starts answering once a #1 handshake
/// has completed, so [`Handshake::run_for_path`] does #1 first when needed.
pub const FLAGS: u8 = 0x80;
/// Request flag bits of the tuner/filter #0 path.
pub const FLAGS_TUNER0: u8 = 0x00;

/// Request flags of the CEPM path serving `tuner_index`.
pub fn path_flags(tuner_index: u8) -> u8 {
    if tuner_index == 0 { FLAGS_TUNER0 } else { FLAGS }
}

/// Persist several handshakes (one per path): records of `flags` + [`Handshake::to_bytes`].
pub fn save_set(path: &std::path::Path, set: &[(u8, Handshake)]) -> std::io::Result<()> {
    let mut v = Vec::new();
    for (flags, hs) in set {
        v.push(*flags);
        v.extend_from_slice(&hs.to_bytes());
    }
    std::fs::write(path, v)
}

/// Load what [`save_set`] wrote (an empty list when the file is missing or malformed).
pub fn load_set(path: &std::path::Path) -> Vec<(u8, Handshake)> {
    let Ok(b) = std::fs::read(path) else { return Vec::new() };
    let rec = 1 + Handshake::SERIALIZED_LEN;
    if b.len() % rec != 0 {
        return Vec::new();
    }
    b.chunks_exact(rec).filter_map(|c| Handshake::from_bytes(&c[1..]).map(|hs| (c[0], hs))).collect()
}

/// Key packet types.
pub const KEYPKT_SESSION: u8 = 0xEC;

/// `KDF(secret, c) = SHA-1(secret ‖ 00×12 ‖ 00 00 00 (c+1))`.
pub fn kdf(secret: &[u8; 20], counter: u8) -> [u8; 20] {
    let mut buf = [0u8; 36];
    buf[..20].copy_from_slice(secret);
    buf[35] = counter.wrapping_add(1);
    Sha1::digest(buf).into()
}

pub(crate) fn aes_cmac(key: &[u8; 16], data: &[u8]) -> [u8; 16] {
    let mut m = <Cmac<Aes128> as KeyInit>::new_from_slice(key).expect("16-byte key");
    m.update(data);
    m.finalize().into_bytes().into()
}

/// AES-128-CBC decryption in place (`data.len()` must be a multiple of 16).
fn aes_cbc_decrypt(key: &[u8; 16], iv: &[u8; 16], data: &mut [u8]) {
    let cipher = Aes128::new(key.into());
    let mut prev = *iv;
    for block in data.as_chunks_mut::<16>().0 {
        let c: [u8; 16] = *block;
        let mut b = aes::Block::from(*block);
        cipher.decrypt_block(&mut b);
        for i in 0..16 {
            block[i] = b[i] ^ prev[i];
        }
        prev = c;
    }
}

/// AES-128-CTR key stream with a big-endian 128-bit counter and a persistent
/// position inside the current block.
pub struct Ctr {
    cipher: Aes128,
    counter: [u8; 16],
    ks: [u8; 16],
    pos: usize,
}

/// Key-stream blocks made per AES call: one TS payload (184 bytes) is 12
/// blocks, and `encrypt_blocks` lets the cipher work on several at once (the
/// fixsliced software AES of hosts without AES instructions does two per
/// pass, AES-NI and the like eight), which is what keeps the daemon's
/// decryption cheap on a small ARM box.
const CTR_PAR: usize = 12;

impl Ctr {
    pub fn new(key: &[u8; 16], iv: [u8; 16]) -> Self {
        let cipher = Aes128::new(key.into());
        let mut ks = aes::Block::from(iv);
        cipher.encrypt_block(&mut ks);
        Ctr { cipher, counter: iv, ks: ks.into(), pos: 0 }
    }

    fn increment(&mut self) {
        for i in (0..16).rev() {
            self.counter[i] = self.counter[i].wrapping_add(1);
            if self.counter[i] != 0 {
                break;
            }
        }
    }

    /// XOR `buf` with the key stream, continuing from the current position.
    pub fn apply(&mut self, mut buf: &mut [u8]) {
        // The rest of the current block first.
        if self.pos < 16 && !buf.is_empty() {
            let n = (16 - self.pos).min(buf.len());
            for (x, k) in buf[..n].iter_mut().zip(&self.ks[self.pos..self.pos + n]) {
                *x ^= k;
            }
            self.pos += n;
            buf = &mut std::mem::take(&mut buf)[n..];
        }
        // Then whole blocks, several per AES call; the last one becomes the
        // current block (used up when `pos` reaches 16).
        while !buf.is_empty() {
            let n = buf.len().div_ceil(16).min(CTR_PAR);
            let mut ks = [aes::Block::default(); CTR_PAR];
            for b in ks.iter_mut().take(n) {
                self.increment();
                *b = aes::Block::from(self.counter);
            }
            self.cipher.encrypt_blocks(&mut ks[..n]);
            let take = (n * 16).min(buf.len());
            let (head, rest) = std::mem::take(&mut buf).split_at_mut(take);
            let mut used = 16;
            for (chunk, k) in head.chunks_mut(16).zip(&ks[..n]) {
                for (x, kk) in chunk.iter_mut().zip(k.iter()) {
                    *x ^= kk;
                }
                used = chunk.len();
            }
            self.ks = ks[n - 1].into();
            self.pos = used;
            buf = rest;
        }
    }
}

/// Initial counter block for a PID (the PID merged into the second word, both words stored big-endian):
/// `BE32((dw2 & 0xF0000FFF) | pid << 12) ‖ BE32(dw1) ‖ 00×8`.
pub fn ctr_iv(pid: u16, dw1: u32, dw2: u32) -> [u8; 16] {
    let w = (dw2 & 0xF000_0FFF) | ((pid as u32) << 12);
    let mut iv = [0u8; 16];
    iv[..4].copy_from_slice(&w.to_be_bytes());
    iv[4..8].copy_from_slice(&dw1.to_be_bytes());
    iv
}

/// PIDs 0x1FC8..=0x1FCF share the common (PID 0) context.
fn context_pid(pid: u16) -> u16 {
    if (0x1FC8..=0x1FCF).contains(&pid) { 0 } else { pid }
}

fn vendor<T: UsbControl + ?Sized>(t: &T, flags: u8, step: u16, out: &[u8], inbuf: Option<&mut [u8]>) -> Result<usize> {
    protocol::send(t, flags, CMD_CEPM, step, out, inbuf)
}

/// How long the device may take to compute a handshake response (ECDSA on
/// the bridge MCU); the IN stage STALLs until the answer is ready.
const RESPONSE_WAIT: std::time::Duration = std::time::Duration::from_secs(4);
const RETRY_INTERVAL: std::time::Duration = std::time::Duration::from_millis(50);
const TIMEOUT_IN: std::time::Duration = std::time::Duration::from_secs(7);

/// IN stage with retry while the device answers STALL (`request` already
/// carries the stage bits).
fn control_in_retry<T: UsbControl + ?Sized>(t: &T, request: u8, step: u16, buf: &mut [u8]) -> Result<usize> {
    let deadline = std::time::Instant::now() + RESPONSE_WAIT;
    loop {
        match t.control_in(request, CMD_CEPM, step, buf, TIMEOUT_IN) {
            Err(Error::Usb(msg)) if std::time::Instant::now() < deadline => {
                log::trace!("cepm/{step}: {msg}, retrying");
                std::thread::sleep(RETRY_INTERVAL);
            }
            r => return r,
        }
    }
}

/// Delay before the first IN attempt of a step: the bridge drops the
/// handshake when it is polled before the previous step was processed
/// (ECDSA verification on the MCU). Overridable with `DTH10_CEPM_DELAY_MS`.
fn step_delay() -> std::time::Duration {
    let ms = std::env::var("DTH10_CEPM_DELAY_MS").ok().and_then(|v| v.parse().ok()).unwrap_or(400u64);
    std::time::Duration::from_millis(ms)
}

/// IN-only step (bRequest = flags | 0x01) after [`step_delay`], with retry.
fn vendor_in<T: UsbControl + ?Sized>(t: &T, flags: u8, step: u16, buf: &mut [u8]) -> Result<usize> {
    std::thread::sleep(step_delay());
    control_in_retry(t, (flags & 0xFC) | protocol::FLAG_IN, step, buf)
}

/// Result of the 0xDEAF/1..6 handshake.
#[derive(Clone, Debug)]
pub struct Handshake {
    /// Device certificate from step 2 (header 24, public key 40, self-signature 40).
    pub dev_cert: [u8; 104],
    /// Device nonce from step 2.
    pub dev_nonce: [u8; 16],
    /// Our nonce sent in step 1 (echoed by the device in step 3).
    pub nonce: [u8; 16],
    /// Device ephemeral public key from step 3.
    pub dev_ephemeral: [u8; 40],
    /// ECDH shared secret (x coordinate).
    pub shared: [u8; 20],
    /// Raw 52-byte step-5 response; bytes 4..20 are the session identity.
    pub m5: [u8; 52],
    /// Session identity from step 5 (compared with 0xEC key packets by the app).
    pub session_id: [u8; 16],
}

impl Handshake {
    /// Serialised size of [`Handshake::to_bytes`].
    pub const SERIALIZED_LEN: usize = 104 + 16 + 16 + 40 + 20 + 52 + 16;

    /// Flat serialisation (the handshake result can be reused across process
    /// restarts as long as the device was not power-cycled; the application
    /// keeps it in a shared memory section for the same reason).
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut v = Vec::with_capacity(Self::SERIALIZED_LEN);
        v.extend_from_slice(&self.dev_cert);
        v.extend_from_slice(&self.dev_nonce);
        v.extend_from_slice(&self.nonce);
        v.extend_from_slice(&self.dev_ephemeral);
        v.extend_from_slice(&self.shared);
        v.extend_from_slice(&self.m5);
        v.extend_from_slice(&self.session_id);
        v
    }

    pub fn from_bytes(b: &[u8]) -> Option<Handshake> {
        if b.len() != Self::SERIALIZED_LEN {
            return None;
        }
        let mut o = 0;
        let mut take = |n: usize| {
            let s = &b[o..o + n];
            o += n;
            s
        };
        Some(Handshake {
            dev_cert: take(104).try_into().unwrap(),
            dev_nonce: take(16).try_into().unwrap(),
            nonce: take(16).try_into().unwrap(),
            dev_ephemeral: take(40).try_into().unwrap(),
            shared: take(20).try_into().unwrap(),
            m5: take(52).try_into().unwrap(),
            session_id: take(16).try_into().unwrap(),
        })
    }

    /// The 20-byte device identity the application compares with bytes
    /// 40..60 of 0xEC key packets: certificate bytes 4..24.
    pub fn device_identity(&self) -> [u8; 20] {
        self.dev_cert[4..24].try_into().unwrap()
    }

    /// 0xDEAF/0: liveness probe, succeeds with no data.
    pub fn probe<T: UsbControl + ?Sized>(t: &T, flags: u8) -> Result<()> {
        vendor(t, flags, 0, &[], None).map(|_| ())
    }

    /// Handshake on the path given by `flags`, doing the #1-path handshake
    /// first when `flags` selects path #0 (see [`FLAGS`]).
    pub fn run_for_path<T: UsbControl + ?Sized>(t: &T, flags: u8, keys: &AppKeys) -> Result<Handshake> {
        // One handshake at a time on the device (the other stream's reader
        // thread may renew its own session while this one runs).
        let _hold = t.hold();
        if flags & FLAGS == 0 {
            match Handshake::run(t, FLAGS, keys) {
                Ok(_) => log::debug!("cepm: path-1 handshake done before the path-0 one"),
                Err(e) => log::debug!("cepm: path-1 handshake before path-0 failed: {e}"),
            }
        }
        Handshake::run(t, flags, keys)
    }

    /// Run steps 1..6 on the path selected by `flags` ([`FLAGS`] or [`FLAGS_TUNER0`]).
    pub fn run<T: UsbControl + ?Sized>(t: &T, flags: u8, keys: &AppKeys) -> Result<Handshake> {
        let curve = Curve::secp160r1();
        let app_priv = keys.app_priv();

        // 1: nonce ‖ application certificate
        let nonce: [u8; 16] = rand::random();
        let mut m1 = [0u8; 120];
        m1[..16].copy_from_slice(&nonce);
        m1[16..].copy_from_slice(&keys.cert);
        vendor(t, flags, 1, &m1, None)?;
        log::debug!("cepm/1 sent nonce {}", hex(&nonce));

        // 2: device nonce ‖ device certificate
        let mut m2 = [0u8; 120];
        let n = vendor_in(t, flags, 2, &mut m2)?;
        if n < 120 {
            return Err(Error::ShortTransfer { expected: 120, actual: n });
        }
        let dev_nonce: [u8; 16] = m2[..16].try_into().unwrap();
        let dev_cert: [u8; 104] = m2[16..].try_into().unwrap();
        let dev_pub = Point::from_bytes(&curve, &dev_cert[24..64])
            .ok_or(Error::Protocol("cepm: device public key not on curve"))?;
        if !curve.verify(&dev_pub, &dev_cert[..64], &dev_cert[64..104]) {
            return Err(Error::Protocol("cepm: device certificate signature invalid"));
        }
        log::debug!("cepm/2 device nonce {} cert header {}", hex(&dev_nonce), hex(&dev_cert[..24]));

        // 3: device ephemeral key ‖ FF×8 ‖ our nonce ‖ signature
        let mut m3 = [0u8; 104];
        let n = vendor_in(t, flags, 3, &mut m3)?;
        if n < 104 {
            return Err(Error::ShortTransfer { expected: 104, actual: n });
        }
        if !curve.verify(&dev_pub, &m3[..64], &m3[64..104]) {
            return Err(Error::Protocol("cepm: step-3 signature invalid"));
        }
        if m3[48..64] != nonce {
            return Err(Error::Protocol("cepm: step-3 nonce mismatch"));
        }
        let dev_ephemeral: [u8; 40] = m3[..40].try_into().unwrap();
        let dev_eph_pt = Point::from_bytes(&curve, &dev_ephemeral)
            .ok_or(Error::Protocol("cepm: device ephemeral key not on curve"))?;
        log::debug!("cepm/3 device ephemeral {}", hex(&dev_ephemeral));

        // 4: our ephemeral key ‖ FF×8 ‖ device nonce ‖ signature(app key)
        let d_e = curve.random_scalar();
        let q_e = curve.public_key(&d_e).to_bytes().ok_or(Error::Protocol("cepm: ephemeral key generation failed"))?;
        let mut m4 = [0u8; 104];
        m4[..40].copy_from_slice(&q_e);
        m4[40..48].copy_from_slice(&[0xFF; 8]);
        m4[48..64].copy_from_slice(&dev_nonce);
        let sig = curve.sign(&app_priv, &m4[..64]);
        m4[64..].copy_from_slice(&sig);
        vendor(t, flags, 4, &m4, None)?;
        let shared = curve.ecdh(&d_e, &dev_eph_pt).ok_or(Error::Protocol("cepm: ECDH failed"))?;
        log::debug!("cepm/4 sent; shared secret {}", hex(&shared));

        // 5/6: session identity, echoed back
        let mut m5 = [0u8; 52];
        let n = vendor_in(t, flags, 5, &mut m5)?;
        if n < 52 {
            return Err(Error::ShortTransfer { expected: 52, actual: n });
        }
        let session_id: [u8; 16] = m5[4..20].try_into().unwrap();
        vendor(t, flags, 6, &m5[..n], None)?;
        log::debug!("cepm/5 {} ; /6 acknowledged", hex(&m5));

        Ok(Handshake { dev_cert, dev_nonce, nonce, dev_ephemeral, shared, m5, session_id })
    }

    /// Diagnostic: continue a handshake whose steps 2 and 3 were already read
    /// (device nonce and ephemeral key known) by sending step 4 and running
    /// steps 5/6. The device certificate is left blank.
    pub fn resume_from_step4<T: UsbControl + ?Sized>(
        t: &T,
        flags: u8,
        keys: &AppKeys,
        dev_nonce: [u8; 16],
        dev_ephemeral: [u8; 40],
    ) -> Result<Handshake> {
        let curve = Curve::secp160r1();
        let app_priv = keys.app_priv();
        let dev_eph_pt = Point::from_bytes(&curve, &dev_ephemeral)
            .ok_or(Error::Protocol("cepm: device ephemeral key not on curve"))?;
        let d_e = curve.random_scalar();
        let q_e = curve.public_key(&d_e).to_bytes().ok_or(Error::Protocol("cepm: ephemeral key generation failed"))?;
        let mut m4 = [0u8; 104];
        m4[..40].copy_from_slice(&q_e);
        m4[40..48].copy_from_slice(&[0xFF; 8]);
        m4[48..64].copy_from_slice(&dev_nonce);
        let sig = curve.sign(&app_priv, &m4[..64]);
        m4[64..].copy_from_slice(&sig);
        vendor(t, flags, 4, &m4, None)?;
        let shared = curve.ecdh(&d_e, &dev_eph_pt).ok_or(Error::Protocol("cepm: ECDH failed"))?;
        log::debug!("cepm/4 sent; shared secret {}", hex(&shared));
        let mut m5 = [0u8; 52];
        let n = vendor_in(t, flags, 5, &mut m5)?;
        if n < 52 {
            return Err(Error::ShortTransfer { expected: 52, actual: n });
        }
        let session_id: [u8; 16] = m5[4..20].try_into().unwrap();
        vendor(t, flags, 6, &m5[..n], None)?;
        Ok(Handshake { dev_cert: [0; 104], dev_nonce, nonce: [0; 16], dev_ephemeral, shared, m5, session_id })
    }
}

/// A session secret obtained from the device.
#[derive(Clone, Debug)]
pub struct Session {
    pub id: [u8; 16],
    pub secret: [u8; 20],
}

impl Session {
    /// `KDF(secret, 3)[0..16]`: CBC key for content blobs.
    pub fn k3(&self) -> [u8; 16] {
        kdf(&self.secret, 3)[..16].try_into().unwrap()
    }

    /// Unwrap a 64-byte content blob from a 0xC0..0xCF key packet
    /// with the application's content IV (`iv`, settled by
    /// [`AppKeys::find_content_iv`]). Returns the 32 decrypted bytes: `[0..16]`
    /// is the CMAC key used to authenticate the blob, `[16..32]` the AES-CTR
    /// stream key.
    pub fn unwrap_content(&self, iv: &[u8; 16], blob: &[u8]) -> Result<[u8; 32]> {
        if blob.len() < 64 {
            return Err(Error::Protocol("cepm: content blob too short"));
        }
        // The application compares blob[0..16] with the id the caller took from
        // the same packet, so the content id is independent of the session id;
        // the blob is bound to the session only through K3 and the CMAC.
        let mut d = [0u8; 32];
        d.copy_from_slice(&blob[16..48]);
        aes_cbc_decrypt(&self.k3(), iv, &mut d);
        let k: [u8; 16] = d[..16].try_into().unwrap();
        if aes_cmac(&k, &blob[..48]) != blob[48..64] {
            return Err(Error::Protocol("cepm: content blob CMAC mismatch"));
        }
        Ok(d)
    }
}

/// Counters kept by [`Decryptor`].
#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    pub packets: u64,
    pub decrypted: u64,
    pub clear: u64,
    pub key_packets: u64,
    pub session_unwraps: u64,
    pub content_unwraps: u64,
    pub dropped_no_key: u64,
    pub errors: u64,
}

/// Stream decryptor following the vendor application's processing (188-byte packets).
///
/// When the device announces sessions whose identity field does not match
/// our handshake (it does that when it has no valid handshake with us: after a
/// power cycle while we reuse a saved [`Handshake`], and after the B-CAS card
/// was pulled and reinserted, which drops the device's session), the decryptor
/// runs a new handshake (at most once per [`REHANDSHAKE_MIN_INTERVAL`]) and
/// reports it through [`Decryptor::take_new_handshake`].
pub struct Decryptor {
    /// Names the stream in log lines (empty = none).
    pub label: String,
    hs: Handshake,
    keys: Arc<AppKeys>,
    session: Option<Session>,
    content_id: [u8; 16],
    content_key: Option<[u8; 16]>,
    ctx: HashMap<u16, Ctr>,
    foreign_announcements: u32,
    last_rehandshake: Option<Instant>,
    new_handshake: bool,
    last_announce: ([u8; 16], [u8; 20]),
    pub stats: Stats,
}

/// Foreign session announcements tolerated before a new handshake is attempted.
const FOREIGN_ANNOUNCEMENTS_BEFORE_REHANDSHAKE: u32 = 2;
/// Minimum interval between stream-triggered handshakes.
pub const REHANDSHAKE_MIN_INTERVAL: Duration = Duration::from_secs(10);

impl Decryptor {
    pub fn new(hs: Handshake, keys: Arc<AppKeys>) -> Self {
        Decryptor {
            label: String::new(),
            hs,
            keys,
            session: None,
            content_id: [0; 16],
            content_key: None,
            ctx: HashMap::new(),
            foreign_announcements: 0,
            last_rehandshake: None,
            new_handshake: false,
            last_announce: ([0; 16], [0; 20]),
            stats: Stats::default(),
        }
    }

    pub fn handshake(&self) -> &Handshake {
        &self.hs
    }

    pub fn app_keys(&self) -> &Arc<AppKeys> {
        &self.keys
    }

    /// `Some(handshake)` once after the decryptor replaced a stale handshake
    /// (the caller should persist it).
    pub fn take_new_handshake(&mut self) -> Option<Handshake> {
        if self.new_handshake {
            self.new_handshake = false;
            Some(self.hs.clone())
        } else {
            None
        }
    }

    pub fn session(&self) -> Option<&Session> {
        self.session.as_ref()
    }

    /// Does `p` (188 bytes) carry key material? An adaptation-field-only
    /// packet whose 181-byte private data starts with a key type byte
    /// (0xEC session, 0xC0..0xCF content). Bytes 8..24 hold a constant tag the
    /// application compares as well; the structure alone is specific enough
    /// on this device's output, so the tag is not needed.
    pub fn is_key_packet(p: &[u8]) -> bool {
        p.len() >= 120
            && p[0] == 0x47
            && (p[3] & 0x30) == 0x20
            && p[4] == 0xB7
            && p[5] == 0x02
            && p[6] == 0xB5
            && (p[7] == KEYPKT_SESSION || (0xC0..=0xCF).contains(&p[7]))
    }

    /// Process one 188-byte packet in place. `Ok(true)` means the packet is
    /// part of the output (clear or decrypted), `Ok(false)` that it was
    /// consumed (key packet) or has to be dropped (no key yet).
    pub fn process<T: UsbControl + ?Sized>(&mut self, t: &T, flags: u8, p: &mut [u8]) -> Result<bool> {
        self.stats.packets += 1;
        if p.len() < 188 || p[0] != 0x47 {
            return Ok(true);
        }
        let pid = ((p[1] as u16 & 0x1F) << 8) | p[2] as u16;
        if pid == 0x1FFF {
            self.stats.clear += 1;
            return Ok(true);
        }
        if Self::is_key_packet(p) {
            self.stats.key_packets += 1;
            let id: [u8; 16] = p[24..40].try_into().unwrap();
            if p[7] == KEYPKT_SESSION {
                // As in the application, the blob is only processed when its id differs
                // from the current session's id ("no session" compares as all
                // zeros, so the device's initial null announcement is skipped).
                let current = self.session.as_ref().map_or([0u8; 16], |s| s.id);
                let ident: [u8; 20] = p[40..60].try_into().unwrap();
                if self.last_announce != (id, ident) {
                    self.last_announce = (id, ident);
                    self.foreign_announcements = 0;
                    log::debug!(
                        "cepm: 0xEC id {} ident {} dw1 {} dw2 0x{:08x} tail {}",
                        hex(&id),
                        hex(&ident),
                        u32::from_le_bytes(p[88..92].try_into().unwrap()),
                        u32::from_le_bytes(p[92..96].try_into().unwrap()),
                        hex(&p[96..120])
                    );
                }
                if id == current {
                    return Ok(false);
                }
                if id == self.hs.session_id {
                    // The handshake session is served by the device object itself
                    // (as in the application); its secret is the ECDH shared secret.
                    log::debug!("cepm: 0xEC announces the handshake session {}", hex(&id));
                    self.session = Some(Session { id, secret: self.hs.shared });
                    self.stats.session_unwraps += 1;
                    self.foreign_announcements = 0;
                    return Ok(false);
                }
                // Any other session: one with a foreign identity (device without a
                // valid handshake, e.g. after a power cycle or when the B-CAS card was
                // reinserted) or one with our identity but bound to another handshake
                // (a stale saved state). The application would unwrap the latter with
                // wIndex 7; on this device a blob the current session
                // does not own makes the request time out, the device drops the session
                // and the path refuses 0xDEAF until a replug. A new handshake rebinds
                // the stream to us instead, so that is all we ever do here.
                self.foreign_announcements += 1;
                let allowed = self.last_rehandshake.is_none_or(|t0| t0.elapsed() >= REHANDSHAKE_MIN_INTERVAL);
                let why = if ident == self.hs.device_identity() {
                    "bound to another handshake on this path (stale saved state?)"
                } else if self.session.is_some() {
                    "session dropped (B-CAS card reinserted?)"
                } else {
                    "foreign identity, no session yet"
                };
                if allowed && self.foreign_announcements >= FOREIGN_ANNOUNCEMENTS_BEFORE_REHANDSHAKE {
                    self.last_rehandshake = Some(Instant::now());
                    self.foreign_announcements = 0;
                    log::info!(
                        "cepm{}: 0xEC announces session {} ({}), running a new handshake",
                        self.label,
                        hex(&id),
                        why
                    );
                    match Handshake::run_for_path(t, flags, &self.keys) {
                        Ok(hs) => {
                            log::info!("cepm{}: new handshake done, session id {}", self.label, hex(&hs.session_id));
                            self.hs = hs;
                            self.new_handshake = true;
                            self.session = None;
                            self.content_key = None;
                            self.content_id = [0; 16];
                            self.ctx.clear();
                        }
                        Err(e) => {
                            self.stats.errors += 1;
                            log::warn!("cepm{}: new handshake failed: {e}", self.label);
                        }
                    }
                } else if self.foreign_announcements == 1 {
                    log::debug!("cepm: 0xEC announces session {} ({}), ignored for now", hex(&id), why);
                }
                return Ok(false);
            }
            let Some(sess) = self.session.as_ref() else {
                self.stats.dropped_no_key += 1;
                return Ok(false);
            };
            if self.content_key.is_none() || self.content_id != id {
                let Some(iv) = self.keys.find_content_iv(&sess.k3(), &p[24..88]) else {
                    self.stats.errors += 1;
                    if self.stats.errors <= 3 || self.stats.errors.is_multiple_of(1000) {
                        log::warn!("cepm{}: content IV not found in {} (wrong file?)", self.label, self.keys.source);
                    }
                    return Ok(false);
                };
                match sess.unwrap_content(&iv, &p[24..88]) {
                    Ok(d) => {
                        self.stats.content_unwraps += 1;
                        self.content_key = Some(d[16..32].try_into().unwrap());
                        self.content_id = id;
                        self.ctx.clear();
                        log::debug!("cepm: content key for session {} installed", hex(&id));
                    }
                    Err(e) => {
                        self.stats.errors += 1;
                        log::warn!(
                            "cepm{}: content unwrap failed: {e} (key packet pid 0x{pid:04x} type 0x{:02x}, content id {})",
                            self.label,
                            p[7],
                            hex(&id)
                        );
                        return Ok(false);
                    }
                }
            }
            let key = self.content_key.unwrap();
            let dw1 = u32::from_le_bytes(p[88..92].try_into().unwrap());
            let dw2 = u32::from_le_bytes(p[92..96].try_into().unwrap());
            let cpid = context_pid(pid);
            self.ctx.insert(cpid, Ctr::new(&key, ctr_iv(cpid, dw1, dw2)));
            log::trace!("cepm: key packet pid 0x{pid:04x} type 0x{:02x} dw1 {dw1} dw2 0x{dw2:08x}", p[7]);
            return Ok(false);
        }
        if p[3] & 0xC0 == 0 {
            self.stats.clear += 1;
            return Ok(true);
        }
        let key = if self.ctx.contains_key(&pid) { pid } else { 0 };
        match self.ctx.get_mut(&key) {
            Some(c) => {
                c.apply(&mut p[4..188]);
                p[3] &= 0x3F;
                self.stats.decrypted += 1;
                Ok(true)
            }
            None => {
                self.stats.dropped_no_key += 1;
                Ok(false)
            }
        }
    }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kdf_layout() {
        // SHA-1 over 36 bytes: secret, 12 zero bytes, 00 00 00 02 for counter 1
        let secret = [0x11u8; 20];
        let mut buf = [0u8; 36];
        buf[..20].copy_from_slice(&secret);
        buf[35] = 2;
        assert_eq!(kdf(&secret, 1), <[u8; 20]>::from(Sha1::digest(buf)));
    }

    #[test]
    fn ctr_iv_layout() {
        let iv = ctr_iv(0x0110, 1, 0x2000_0000);
        assert_eq!(&iv[..4], &[0x20, 0x11, 0x00, 0x00]);
        assert_eq!(&iv[4..8], &[0, 0, 0, 1]);
        assert_eq!(&iv[8..], &[0; 8]);
        assert_eq!(context_pid(0x1FC8), 0);
        assert_eq!(context_pid(0x1FD0), 0x1FD0);
    }

    #[test]
    fn ctr_is_standard_big_endian() {
        let key = [0u8; 16];
        let mut c = Ctr::new(&key, [0xFF; 16]);
        let mut buf = [0u8; 32];
        c.apply(&mut buf);
        let cipher = Aes128::new(&key.into());
        let mut b0 = aes::Block::from([0xFF; 16]);
        cipher.encrypt_block(&mut b0);
        let mut b1 = aes::Block::from([0; 16]); // 0xFF..FF + 1 wraps to 0
        cipher.encrypt_block(&mut b1);
        assert_eq!(&buf[..16], b0.as_slice());
        assert_eq!(&buf[16..], b1.as_slice());
        // position persists across calls
        let mut c2 = Ctr::new(&key, [0xFF; 16]);
        let mut a = [0u8; 5];
        let mut b = [0u8; 27];
        c2.apply(&mut a);
        c2.apply(&mut b);
        assert_eq!(&buf[..5], &a);
        assert_eq!(&buf[5..], &b);
    }

    #[test]
    fn ctr_batches_match_single_blocks() {
        let key = [3u8; 16];
        let iv = [0xFEu8; 16];
        // reference: one AES call per block
        let mut want = vec![0u8; 1000];
        let cipher = Aes128::new(&key.into());
        let mut ctr = iv;
        for chunk in want.chunks_mut(16) {
            let mut b = aes::Block::from(ctr);
            cipher.encrypt_block(&mut b);
            for (x, k) in chunk.iter_mut().zip(b.iter()) {
                *x ^= k;
            }
            for j in (0..16).rev() {
                ctr[j] = ctr[j].wrapping_add(1);
                if ctr[j] != 0 {
                    break;
                }
            }
        }
        let mut one = vec![0u8; 1000];
        Ctr::new(&key, iv).apply(&mut one);
        assert_eq!(one, want);
        // in uneven pieces, as TS payloads of 184 bytes and odd tails come
        let mut pieces = vec![0u8; 1000];
        let mut c = Ctr::new(&key, iv);
        let mut off = 0;
        for n in [184, 7, 184, 1, 16, 32, 184, 184, 184, 24] {
            c.apply(&mut pieces[off..off + n]);
            off += n;
        }
        assert_eq!(off, 1000);
        assert_eq!(pieces, want);
    }

    #[test]
    fn cbc_roundtrip() {
        use aes::cipher::BlockCipherEncrypt;
        let key = [7u8; 16];
        let iv = [9u8; 16];
        let plain = [0x42u8; 32];
        // encrypt manually
        let cipher = Aes128::new(&key.into());
        let mut ct = [0u8; 32];
        let mut prev = iv;
        for (i, blk) in plain.chunks(16).enumerate() {
            let mut b = aes::Block::default();
            for j in 0..16 {
                b[j] = blk[j] ^ prev[j];
            }
            cipher.encrypt_block(&mut b);
            ct[i * 16..i * 16 + 16].copy_from_slice(&b);
            prev = b.into();
        }
        let mut d = ct;
        aes_cbc_decrypt(&key, &iv, &mut d);
        assert_eq!(d, plain);
    }

    #[test]
    fn key_packet_detection() {
        let mut p = [0u8; 188];
        p[0] = 0x47;
        p[3] = 0x20;
        p[4] = 0xB7;
        p[5] = 0x02;
        p[6] = 0xB5;
        p[7] = 0xC1;
        assert!(Decryptor::is_key_packet(&p));
        p[7] = 0xEC;
        assert!(Decryptor::is_key_packet(&p));
        p[5] = 0x10;
        assert!(!Decryptor::is_key_packet(&p));
        p[5] = 0x02;
        p[7] = 0x12;
        assert!(!Decryptor::is_key_packet(&p));
    }
}
