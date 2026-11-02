//! Minimal secp160r1 arithmetic (affine coordinates, `num-bigint`), ECDSA
//! with SHA-1 digests and ECDH as used by Cobalt's CEPM handshake (signing,
//! verification and the key agreement on the secp160r1 curve).
//!
//! Performance and side-channel resistance are irrelevant here: the handshake
//! runs once per device open and the application key is read from the vendor's
//! own DLL (`appkeys`), where it sits as plain constants.

use num_bigint::BigUint;
use num_integer::Integer;
use num_traits::{One, Zero};
use sha1::{Digest, Sha1};

/// Size of a field element / scalar in bytes.
pub const FIELD_BYTES: usize = 20;

fn hex(s: &str) -> BigUint {
    BigUint::parse_bytes(s.as_bytes(), 16).expect("valid hex")
}

/// Curve parameters (secp160r1, SEC 2).
pub struct Curve {
    pub p: BigUint,
    pub a: BigUint,
    pub b: BigUint,
    pub gx: BigUint,
    pub gy: BigUint,
    pub n: BigUint,
}

impl Curve {
    pub fn secp160r1() -> Curve {
        Curve {
            p: hex("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF7FFFFFFF"),
            a: hex("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF7FFFFFFC"),
            b: hex("1C97BEFC54BD7A8B65ACF89F81D4D4ADC565FA45"),
            gx: hex("4A96B5688EF573284664698968C38BB913CBFC82"),
            gy: hex("23A628553168947D59DCC912042351377AC5FB32"),
            n: hex("0100000000000000000001F4C8F927AED3CA752257"),
        }
    }

    fn g(&self) -> Point {
        Point::Affine(self.gx.clone(), self.gy.clone())
    }

    fn inv_mod_p(&self, x: &BigUint) -> BigUint {
        // p is prime: x^(p-2)
        x.modpow(&(&self.p - 2u32), &self.p)
    }

    fn inv_mod_n(&self, x: &BigUint) -> BigUint {
        x.modpow(&(&self.n - 2u32), &self.n)
    }

    fn sub_p(&self, a: &BigUint, b: &BigUint) -> BigUint {
        if a >= b { a - b } else { &self.p - (b - a) }
    }

    /// Is `(x, y)` on the curve?
    pub fn contains(&self, x: &BigUint, y: &BigUint) -> bool {
        if x >= &self.p || y >= &self.p {
            return false;
        }
        let lhs = (y * y) % &self.p;
        let rhs = (x * x * x + &self.a * x + &self.b) % &self.p;
        lhs == rhs
    }

    pub fn add(&self, p1: &Point, p2: &Point) -> Point {
        match (p1, p2) {
            (Point::Infinity, _) => p2.clone(),
            (_, Point::Infinity) => p1.clone(),
            (Point::Affine(x1, y1), Point::Affine(x2, y2)) => {
                let lambda = if x1 == x2 {
                    if (y1 + y2) % &self.p == BigUint::zero() {
                        return Point::Infinity;
                    }
                    // tangent: (3x^2 + a) / 2y
                    let num = (3u32 * x1 * x1 + &self.a) % &self.p;
                    let den = self.inv_mod_p(&((2u32 * y1) % &self.p));
                    (num * den) % &self.p
                } else {
                    let num = self.sub_p(y2, y1);
                    let den = self.inv_mod_p(&self.sub_p(x2, x1));
                    (num * den) % &self.p
                };
                let x3 = self.sub_p(&self.sub_p(&((&lambda * &lambda) % &self.p), x1), x2);
                let y3 = self.sub_p(&((&lambda * self.sub_p(x1, &x3)) % &self.p), y1);
                Point::Affine(x3, y3)
            }
        }
    }

    pub fn mul(&self, k: &BigUint, pt: &Point) -> Point {
        let mut result = Point::Infinity;
        let mut addend = pt.clone();
        for i in 0..k.bits() {
            if k.bit(i) {
                result = self.add(&result, &addend);
            }
            addend = self.add(&addend, &addend);
        }
        result
    }

    /// Public key for scalar `d`.
    pub fn public_key(&self, d: &BigUint) -> Point {
        self.mul(d, &self.g())
    }

    /// Random scalar in `[1, n-1]`.
    pub fn random_scalar(&self) -> BigUint {
        // 32 random bytes reduced modulo n-1, plus one: uniform enough for
        // a one-shot handshake (bias ~2^-95).
        let bytes: [u8; 32] = rand::random();
        BigUint::from_bytes_be(&bytes) % (&self.n - BigUint::one()) + BigUint::one()
    }

    /// ECDSA signature `r || s` (20 + 20 bytes, big-endian) over the SHA-1 of `msg`.
    pub fn sign(&self, d: &BigUint, msg: &[u8]) -> [u8; 2 * FIELD_BYTES] {
        let e = BigUint::from_bytes_be(&Sha1::digest(msg));
        loop {
            let k = self.random_scalar();
            let Point::Affine(rx, _) = self.mul(&k, &self.g()) else { continue };
            let r = rx % &self.n;
            if r.is_zero() {
                continue;
            }
            let s = (self.inv_mod_n(&k) * ((&e + d * &r) % &self.n)) % &self.n;
            if s.is_zero() || s.bits() > 160 {
                continue;
            }
            let mut out = [0u8; 2 * FIELD_BYTES];
            out[..FIELD_BYTES].copy_from_slice(&to_fixed(&r));
            out[FIELD_BYTES..].copy_from_slice(&to_fixed(&s));
            return out;
        }
    }

    /// Verify an ECDSA signature `r || s` over the SHA-1 of `msg` with public key `q`.
    pub fn verify(&self, q: &Point, msg: &[u8], sig: &[u8]) -> bool {
        if sig.len() != 2 * FIELD_BYTES {
            return false;
        }
        let r = BigUint::from_bytes_be(&sig[..FIELD_BYTES]);
        let s = BigUint::from_bytes_be(&sig[FIELD_BYTES..]);
        if r.is_zero() || s.is_zero() || r >= self.n || s >= self.n {
            return false;
        }
        let e = BigUint::from_bytes_be(&Sha1::digest(msg));
        let w = self.inv_mod_n(&s);
        let u1 = (&e * &w) % &self.n;
        let u2 = (&r * &w) % &self.n;
        match self.add(&self.mul(&u1, &self.g()), &self.mul(&u2, q)) {
            Point::Infinity => false,
            Point::Affine(x, _) => x % &self.n == r,
        }
    }

    /// ECDH: x-coordinate of `d * q`, 20 bytes big-endian.
    pub fn ecdh(&self, d: &BigUint, q: &Point) -> Option<[u8; FIELD_BYTES]> {
        match self.mul(d, q) {
            Point::Infinity => None,
            Point::Affine(x, _) => Some(to_fixed(&x)),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Point {
    Infinity,
    Affine(BigUint, BigUint),
}

impl Point {
    /// Parse `x || y` (40 bytes) and check that it lies on the curve.
    pub fn from_bytes(curve: &Curve, xy: &[u8]) -> Option<Point> {
        if xy.len() != 2 * FIELD_BYTES {
            return None;
        }
        let x = BigUint::from_bytes_be(&xy[..FIELD_BYTES]);
        let y = BigUint::from_bytes_be(&xy[FIELD_BYTES..]);
        curve.contains(&x, &y).then_some(Point::Affine(x, y))
    }

    /// `x || y`, 40 bytes big-endian.
    pub fn to_bytes(&self) -> Option<[u8; 2 * FIELD_BYTES]> {
        match self {
            Point::Infinity => None,
            Point::Affine(x, y) => {
                let mut out = [0u8; 2 * FIELD_BYTES];
                out[..FIELD_BYTES].copy_from_slice(&to_fixed(x));
                out[FIELD_BYTES..].copy_from_slice(&to_fixed(y));
                Some(out)
            }
        }
    }
}

/// Big-endian, left-padded to 20 bytes (`BN_bn2bin` into a zeroed buffer).
pub fn to_fixed(v: &BigUint) -> [u8; FIELD_BYTES] {
    let b = v.to_bytes_be();
    let mut out = [0u8; FIELD_BYTES];
    let n = b.len().min(FIELD_BYTES);
    out[FIELD_BYTES - n..].copy_from_slice(&b[b.len() - n..]);
    out
}

pub fn scalar_from_bytes(b: &[u8]) -> BigUint {
    BigUint::from_bytes_be(b)
}

#[allow(dead_code)]
fn gcd_is_one(a: &BigUint, b: &BigUint) -> bool {
    a.gcd(b).is_one()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn h(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    /// Device certificate returned by a DT-H70/U2 for 0xDEAF/2 (docs/22):
    /// bytes 24..64 = public key, 64..104 = self-signature over bytes 0..64.
    const DEV_CERT: &str = "1110ffff0411019904110199010000000b08164e00000146c8ac51a0fa7db12b11eb806c1872f9216af049619a12105a3d2584e5fd58defee5b5edf4a8eec9238f3e6bcd22cf822154f59f2d1e17db7049b17bf9107b2d704246055ee1cc54626b045a87fd7a7f42";
    /// 0xDEAF/3 message from the same device (ephemeral key, FF*8, our nonce 00..0F, signature).
    const DEV_M3: &str = "4ce73df13fdf54dd634f48045924cf518ab3a53a1d00fd85b784e8f6ed7ab623cc6849f0a9e5283bffffffffffffffff000102030405060708090a0b0c0d0e0f74c54aa357535def92e64afb327110b1722122dd64a2fe85cf62a49cc3f76ae7469fa01d00354bf8";
    #[test]
    fn generator_is_on_curve() {
        let c = Curve::secp160r1();
        assert!(c.contains(&c.gx, &c.gy));
        assert_eq!(c.mul(&c.n, &c.g()), Point::Infinity);
    }

    #[test]
    fn device_certificate_is_self_signed() {
        let c = Curve::secp160r1();
        let cert = h(DEV_CERT);
        let q = Point::from_bytes(&c, &cert[24..64]).expect("public key on curve");
        assert!(c.verify(&q, &cert[..64], &cert[64..104]));
        assert!(!c.verify(&q, &cert[1..65], &cert[64..104]));
    }

    #[test]
    fn device_m3_is_signed_by_certificate_key() {
        let c = Curve::secp160r1();
        let cert = h(DEV_CERT);
        let m3 = h(DEV_M3);
        let q = Point::from_bytes(&c, &cert[24..64]).unwrap();
        assert!(Point::from_bytes(&c, &m3[..40]).is_some(), "ephemeral key on curve");
        assert!(c.verify(&q, &m3[..64], &m3[64..104]));
    }

    #[test]
    fn sign_verify_roundtrip() {
        let c = Curve::secp160r1();
        let d = c.random_scalar();
        let q = c.public_key(&d);
        let sig = c.sign(&d, b"hello");
        assert!(c.verify(&q, b"hello", &sig));
        assert!(!c.verify(&q, b"hellp", &sig));
        let back = Point::from_bytes(&c, &q.to_bytes().unwrap()).unwrap();
        assert_eq!(back, q);
    }

    #[test]
    fn ecdh_is_symmetric() {
        let c = Curve::secp160r1();
        let a = c.random_scalar();
        let b = c.random_scalar();
        let qa = c.public_key(&a);
        let qb = c.public_key(&b);
        assert_eq!(c.ecdh(&a, &qb), c.ecdh(&b, &qa));
    }
}
