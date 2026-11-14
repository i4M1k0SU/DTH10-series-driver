//! Register access the way the vendor's user-mode filter does it on top of
//! the raw I2C transfers (docs/26 4-tuning §1):
//!
//! * every transfer is retried on failure, up to [`XFER_ATTEMPTS`] attempts
//!   [`XFER_RETRY_MS`] apart;
//! * a register write is `[reg, data...]` with at most [`WRITE_CHUNK`] data
//!   bytes per transfer, the register advanced by the chunk size for the
//!   next one; an empty write sends `[reg]` alone;
//! * a register read writes `[reg]` (no stop) and reads with a repeated
//!   start, at most [`READ_CHUNK`] bytes per transfer;
//! * the tuner is reached directly, directly with a trailing 0xFE byte
//!   (MJ100), or through the demodulator's 0xFE repeater (MJ102/MJ110).
//!   Reads always go directly to the device.
//!
//! The vendor runs its transfers at 400 kHz on I2C channel 0; the clock and
//! channel are the [`I2c`] implementation's business.

use core::time::Duration;

use crate::i2c::I2c;
use crate::{Error, Result};

/// Millisecond delays (the vendor's `Sleep` calls). Kept apart from the
/// I2C trait so tests can record them instead of sleeping.
pub trait Delay {
    fn delay_ms(&mut self, ms: u32);
}

impl<T: Delay + ?Sized> Delay for &mut T {
    fn delay_ms(&mut self, ms: u32) {
        (**self).delay_ms(ms)
    }
}

/// What a front end needs: an I2C bus and a way to wait.
pub trait Bus: I2c + Delay {}

impl<T: I2c + Delay + ?Sized> Bus for T {}

/// An [`I2c`] bus that waits with [`std::thread::sleep`].
#[derive(Debug, Default)]
pub struct StdSleep<I>(pub I);

impl<I: I2c> I2c for StdSleep<I> {
    fn write(&mut self, addr8: u8, data: &[u8]) -> Result<()> {
        self.0.write(addr8, data)
    }
    fn write_read(&mut self, addr8: u8, wr: &[u8], rd: &mut [u8]) -> Result<()> {
        self.0.write_read(addr8, wr, rd)
    }
}

impl<I> Delay for StdSleep<I> {
    fn delay_ms(&mut self, ms: u32) {
        std::thread::sleep(Duration::from_millis(ms.into()))
    }
}

/// Attempts per transfer. The vendor resends up to 10 times; read here as
/// 10 attempts in all.
pub const XFER_ATTEMPTS: u32 = 10;
/// Wait between attempts (ms).
pub const XFER_RETRY_MS: u32 = 5;
/// Data bytes per register-write transfer.
pub const WRITE_CHUNK: usize = 8;
/// Bytes per register-read transfer.
pub const READ_CHUNK: usize = 16;

/// Whether a failed transfer is worth repeating. The vendor gives up at
/// once when the driver reports the device removed or in standby; the
/// closest thing here is a transport error, so only I2C-level errors and
/// timeouts are retried.
fn retryable(e: &Error) -> bool {
    matches!(e, Error::I2c { .. } | Error::Timeout)
}

/// Run one transfer with the vendor's retry policy.
fn retry<B: Bus + ?Sized, T>(bus: &mut B, mut op: impl FnMut(&mut B) -> Result<T>) -> Result<T> {
    let mut attempt = 1;
    loop {
        match op(bus) {
            Ok(v) => return Ok(v),
            Err(e) if retryable(&e) && attempt < XFER_ATTEMPTS => {
                attempt += 1;
                bus.delay_ms(XFER_RETRY_MS);
            }
            Err(e) => return Err(e),
        }
    }
}

/// One raw write transfer, retried.
pub fn write_raw<B: Bus + ?Sized>(bus: &mut B, addr8: u8, data: &[u8]) -> Result<()> {
    retry(bus, |b| b.write(addr8, data))
}

/// Write `data` to consecutive registers from `reg`.
pub fn write_regs<B: Bus + ?Sized>(bus: &mut B, addr8: u8, reg: u8, data: &[u8]) -> Result<()> {
    if data.is_empty() {
        return write_raw(bus, addr8, &[reg]);
    }
    let mut buf = [0u8; 1 + WRITE_CHUNK];
    for (i, chunk) in data.chunks(WRITE_CHUNK).enumerate() {
        buf[0] = reg.wrapping_add((i * WRITE_CHUNK) as u8);
        buf[1..=chunk.len()].copy_from_slice(chunk);
        write_raw(bus, addr8, &buf[..=chunk.len()])?;
    }
    Ok(())
}

/// Write one register.
pub fn write_reg<B: Bus + ?Sized>(bus: &mut B, addr8: u8, reg: u8, value: u8) -> Result<()> {
    write_regs(bus, addr8, reg, &[value])
}

/// Read consecutive registers from `reg` into `buf`.
pub fn read_regs<B: Bus + ?Sized>(bus: &mut B, addr8: u8, reg: u8, buf: &mut [u8]) -> Result<()> {
    for (i, chunk) in buf.chunks_mut(READ_CHUNK).enumerate() {
        let r = reg.wrapping_add((i * READ_CHUNK) as u8);
        retry(bus, |b| b.write_read(addr8, &[r], chunk))?;
    }
    Ok(())
}

/// Read one register.
pub fn read_reg<B: Bus + ?Sized>(bus: &mut B, addr8: u8, reg: u8) -> Result<u8> {
    let mut v = [0u8];
    read_regs(bus, addr8, reg, &mut v)?;
    Ok(v[0])
}

/// How tuner writes reach the tuner.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TunerPath {
    /// `tuner: [reg, data...]`.
    Direct,
    /// `tuner: [reg, data..., 0xFE]` (MJ100; the kernel's standby writes
    /// have the same shape).
    DirectTrailingFe,
    /// `demod: [0xFE, tuner, reg, data...]` (the demodulator's repeater).
    ViaDemod,
}

/// Tuner register bytes per repeater transfer: the repeater write is a
/// demodulator register write to 0xFE, which must stay one transfer, so the
/// tuner address and register leave room for 6 data bytes. The vendor only
/// ever writes one tuner register at a time this way; longer writes are
/// split here with the tuner register advanced (the vendor's generic
/// chunking would advance the demodulator register instead).
const REPEATER_CHUNK: usize = WRITE_CHUNK - 2;

/// Write tuner registers along `path`.
pub fn write_tuner<B: Bus + ?Sized>(
    bus: &mut B,
    path: TunerPath,
    demod: u8,
    tuner: u8,
    reg: u8,
    data: &[u8],
) -> Result<()> {
    match path {
        TunerPath::Direct => write_regs(bus, tuner, reg, data),
        TunerPath::DirectTrailingFe => {
            let mut buf = Vec::with_capacity(data.len() + 1);
            buf.extend_from_slice(data);
            buf.push(0xFE);
            write_regs(bus, tuner, reg, &buf)
        }
        TunerPath::ViaDemod => {
            let mut buf = [0u8; WRITE_CHUNK];
            buf[0] = tuner;
            for (i, chunk) in data.chunks(REPEATER_CHUNK).enumerate() {
                buf[1] = reg.wrapping_add((i * REPEATER_CHUNK) as u8);
                buf[2..2 + chunk.len()].copy_from_slice(chunk);
                write_regs(bus, demod, 0xFE, &buf[..2 + chunk.len()])?;
            }
            if data.is_empty() {
                write_regs(bus, demod, 0xFE, &[tuner, reg])?;
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::testutil::{Ev, Rig};
    use super::*;

    #[test]
    fn writes_are_split_into_eight_byte_chunks() {
        let mut rig = Rig::new();
        let data: Vec<u8> = (1..=20).collect();
        write_regs(&mut rig, 0x20, 0x10, &data).unwrap();
        assert_eq!(
            rig.ev,
            vec![
                Ev::W(0x20, vec![0x10, 1, 2, 3, 4, 5, 6, 7, 8]),
                Ev::W(0x20, vec![0x18, 9, 10, 11, 12, 13, 14, 15, 16]),
                Ev::W(0x20, vec![0x20, 17, 18, 19, 20]),
            ]
        );
        rig.ev.clear();
        write_regs(&mut rig, 0x20, 0x33, &[]).unwrap();
        assert_eq!(rig.ev, vec![Ev::W(0x20, vec![0x33])]);
    }

    #[test]
    fn reads_are_split_into_sixteen_byte_chunks() {
        let mut rig = Rig::new();
        for r in 0..40u8 {
            rig.mock.set(0x20, 0x90 + r, r);
        }
        let mut buf = [0u8; 40];
        read_regs(&mut rig, 0x20, 0x90, &mut buf).unwrap();
        assert_eq!(rig.ev, vec![Ev::R(0x20, vec![0x90], 16), Ev::R(0x20, vec![0xA0], 16), Ev::R(0x20, vec![0xB0], 8)]);
        assert!(buf.iter().enumerate().all(|(i, &v)| v == i as u8));
    }

    #[test]
    fn transfers_are_retried_ten_times_5_ms_apart() {
        let mut rig = Rig::new();
        rig.fail_times(0x20, 3);
        write_reg(&mut rig, 0x20, 0x01, 0x40).unwrap();
        let w = Ev::W(0x20, vec![0x01, 0x40]);
        assert_eq!(rig.ev, vec![w.clone(), Ev::D(5), w.clone(), Ev::D(5), w.clone(), Ev::D(5), w.clone()]);

        let mut rig = Rig::new();
        rig.fail_always(0x20);
        std::assert_matches!(read_reg(&mut rig, 0x20, 0x80), Err(Error::I2c { .. }));
        assert_eq!(rig.ev.iter().filter(|e| matches!(e, Ev::R(..))).count(), 10);
        assert_eq!(rig.ev.iter().filter(|e| **e == Ev::D(5)).count(), 9);
    }

    #[test]
    fn transport_errors_are_not_retried() {
        let mut rig = Rig::new();
        rig.fail_with_usb(0x20);
        std::assert_matches!(write_reg(&mut rig, 0x20, 0x01, 0x40), Err(Error::Usb(_)));
        assert_eq!(rig.ev, vec![Ev::W(0x20, vec![0x01, 0x40])]);
    }

    #[test]
    fn tuner_paths() {
        let mut rig = Rig::new();
        write_tuner(&mut rig, TunerPath::Direct, 0x20, 0xC0, 0x01, &[1, 2, 3, 4, 5]).unwrap();
        write_tuner(&mut rig, TunerPath::DirectTrailingFe, 0x20, 0xC0, 0x95, &[0x03]).unwrap();
        write_tuner(&mut rig, TunerPath::ViaDemod, 0x30, 0xC6, 0x14, &[0x49]).unwrap();
        assert_eq!(
            rig.ev,
            vec![
                Ev::W(0xC0, vec![0x01, 1, 2, 3, 4, 5]),
                Ev::W(0xC0, vec![0x95, 0x03, 0xFE]),
                Ev::W(0x30, vec![0xFE, 0xC6, 0x14, 0x49]),
            ]
        );
        rig.ev.clear();
        write_tuner(&mut rig, TunerPath::ViaDemod, 0x20, 0xC0, 0x10, &[1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        assert_eq!(
            rig.ev,
            vec![Ev::W(0x20, vec![0xFE, 0xC0, 0x10, 1, 2, 3, 4, 5, 6]), Ev::W(0x20, vec![0xFE, 0xC0, 0x16, 7, 8])]
        );
    }
}
