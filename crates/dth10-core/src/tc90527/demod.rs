//! The TC90527 demodulator of the `TC90527_*` kits. The vendor drivers drive
//! it with the TC90517 code for the pre- / post-tune and the signal status
//! ([`crate::tc90517::Tc90517`] with
//! [`TerVariant::Tc90527`](crate::tc90517::TerVariant::Tc90527): the
//! post-tune steps of the TC90517_MxL13xRF kit); its configuration, TS pins,
//! wake, standby and the preamble of its status are its own.

use core::time::Duration;
use std::thread::sleep;

use crate::device::Device;
use crate::error::{Error, Result};
use crate::kit::VendorDriver;
use crate::protocol::UsbControl;
use crate::tlv::TunerDesc;

/// Presence bit of the descriptor's demodulator IF (`val_62`).
const PRESENT_DEMOD_IF: u32 = 0x20000;

/// Registers 0x11 / 0x12 by demodulator crystal (Hz).
const XTAL_REGS: [(u32, u8, u8); 6] = [
    (16_000_000, 0x2D, 0x02),
    (18_000_000, 0x28, 0x02),
    (24_000_000, 0x1E, 0x02),
    (25_400_000, 0x18, 0x02),
    (32_000_000, 0x2D, 0x04),
    (36_000_000, 0x28, 0x04),
];

/// The block a 0 Hz IF adds (single registers); 0x30 is set apart.
const ZERO_IF_BLOCK_1: [(u8, u8); 7] =
    [(0x16, 0x04), (0x1C, 0x20), (0x1D, 0x00), (0x1E, 0x00), (0x1F, 0x00), (0x22, 0x80), (0x23, 0x4C)];
const ZERO_IF_BLOCK_2: [(u8, u8); 8] =
    [(0x31, 0x00), (0x32, 0x00), (0x3D, 0x00), (0x47, 0x50), (0xD0, 0x1E), (0xD1, 0x0B), (0xF8, 0x42), (0xFA, 0x04)];

/// The configuration of the demodulator at `addr` (Init, or Open with flags
/// bit5 set), one batch: `01=40 04=02 0F=00 10=00`, 0x11 / 0x12 by the
/// descriptor's demodulator crystal (16 / 18 / 24 / 25.4 / 32 / 36 MHz), for
/// a 0 Hz IF a block of IF registers (3 and 4 MHz have none), then
/// `75={(~(X >> 3)) & 2 | 4, 32, 01} EF=01 01=40` (X = `stream_param`).
/// Another crystal or IF fails with the batch so far left queued, as in the
/// vendor drivers (the next transfer sends it). In the IF block the
/// DT-H30/PCIEM-TS driver sets 0x30 bit3 for a negative demodulator IF
/// (`val_62`, when the descriptor has one).
pub fn configure<T: UsbControl>(dev: &mut Device<T>, addr: u8, desc: &TunerDesc, driver: VendorDriver) -> Result<()> {
    for (r, v) in [(0x01, 0x40), (0x04, 0x02), (0x0F, 0x00), (0x10, 0x00)] {
        dev.i2c_batch_append(addr, r, &[v])?;
    }
    let Some(&(_, r11, r12)) = XTAL_REGS.iter().find(|x| x.0 == desc.demod_xtal_hz) else {
        return Err(Error::Unsupported(format!("TC90527 demodulator crystal {} Hz", desc.demod_xtal_hz)));
    };
    dev.i2c_batch_append(addr, 0x11, &[r11])?;
    dev.i2c_batch_append(addr, 0x12, &[r12])?;
    match desc.if_freq_hz.unsigned_abs() {
        3_000_000 | 4_000_000 => {}
        0 => {
            let inverted =
                driver == VendorDriver::Dth30 && desc.present & PRESENT_DEMOD_IF != 0 && (desc.val_62 as i32) < 0;
            for (r, v) in ZERO_IF_BLOCK_1 {
                dev.i2c_batch_append(addr, r, &[v])?;
            }
            dev.i2c_batch_append(addr, 0x30, &[if inverted { 0x29 } else { 0x21 }])?;
            for (r, v) in ZERO_IF_BLOCK_2 {
                dev.i2c_batch_append(addr, r, &[v])?;
            }
            dev.i2c_batch_append(addr, 0x31, &[0x00, 0x00])?;
            dev.i2c_batch_append(addr, 0x38, &[0x01, 0x15])?;
        }
        _ => return Err(Error::Unsupported(format!("TC90527 IF {} Hz", desc.if_freq_hz))),
    }
    let b0 = (!(desc.stream_param >> 3) & 0x02) as u8 | 0x04;
    dev.i2c_batch_append(addr, 0x75, &[b0, 0x32, 0x01])?;
    dev.i2c_batch_append(addr, 0xEF, &[0x01])?;
    dev.i2c_batch_append(addr, 0x01, &[0x40])?;
    dev.i2c_batch_flush()
}

/// The TS pin control (modes 0 / 1 / 2 = output / stopped / third state):
/// `1D / 1E` = `01 / 15`, `A9 / 95`, `55 / 55` in one batch; another mode
/// only sends the batch.
pub fn set_ts_pins<T: UsbControl>(dev: &mut Device<T>, addr: u8, mode: u8) -> Result<()> {
    let (r1d, r1e) = match mode {
        0 => (0x01, 0x15),
        1 => (0xA9, 0x95),
        2 => (0x55, 0x55),
        _ => return dev.i2c_batch_flush(),
    };
    dev.i2c_batch_append(addr, 0x1D, &[r1d])?;
    dev.i2c_batch_append(addr, 0x1E, &[r1e])?;
    dev.i2c_batch_flush()
}

fn read1<T: UsbControl>(dev: &mut Device<T>, addr: u8, reg: u8) -> Result<u8> {
    let mut b = [0u8; 1];
    dev.i2c_reg_read(addr, reg, &mut b)?;
    Ok(b[0])
}

/// Whether the wake and standby switch register 0x1F bits 3:2: always with
/// the series driver, for `TC90527_FC5500` only with the DT-H30/PCIEM-TS one.
fn switches_1f(driver: VendorDriver, kit_name: &str) -> bool {
    driver == VendorDriver::Series || kit_name == "TC90527_FC5500"
}

/// Wake (Open): `1F |= 0C` (see [`switches_1f`]; else 0x1F is written back
/// as read), `04 &= D7`, 2 ms, `03 <- 00`, immediate writes; the first
/// failure ends it.
pub fn wake<T: UsbControl>(dev: &mut Device<T>, addr: u8, driver: VendorDriver, kit_name: &str) -> Result<()> {
    let v = read1(dev, addr, 0x1F)?;
    dev.i2c_reg_write(addr, 0x1F, &[if switches_1f(driver, kit_name) { v | 0x0C } else { v }])?;
    let v = read1(dev, addr, 0x04)?;
    dev.i2c_reg_write(addr, 0x04, &[v & 0xD7])?;
    sleep(Duration::from_millis(2));
    dev.i2c_reg_write(addr, 0x03, &[0x00])
}

/// Standby (end of Init, Close): TS stopped, `03 <- 10`, `04 |= 28`,
/// `1F &= F3` (see [`switches_1f`]; else 0x1F is written back as read),
/// immediate writes; the first failure ends it.
pub fn standby<T: UsbControl>(dev: &mut Device<T>, addr: u8, driver: VendorDriver, kit_name: &str) -> Result<()> {
    set_ts_pins(dev, addr, 1)?;
    dev.i2c_reg_write(addr, 0x03, &[0x10])?;
    let v = read1(dev, addr, 0x04)?;
    dev.i2c_reg_write(addr, 0x04, &[v | 0x28])?;
    let v = read1(dev, addr, 0x1F)?;
    dev.i2c_reg_write(addr, 0x1F, &[if switches_1f(driver, kit_name) { v & 0xF3 } else { v }])
}

/// The preamble of the signal status (both drivers, once a configuration
/// has passed its checks): `FA <- 06` when the TMCC of layer A reads
/// modulation 3, coding rate 0 and 13 segments (`B3[7:5] = 3`, `B3[4:2] = 0`,
/// `B4[6:3] = 13`), else `FA <- 04`. The rest of the status is the TC90517's.
pub fn status_preamble<T: UsbControl>(dev: &mut Device<T>, addr: u8) -> Result<()> {
    let b3 = read1(dev, addr, 0xB3)?;
    let b4 = read1(dev, addr, 0xB4)?;
    let fa = if b4 & 0x78 == 0x68 && b3 >> 5 == 3 && (b3 >> 2) & 7 == 0 { 0x06 } else { 0x04 };
    dev.i2c_reg_write(addr, 0xFA, &[fa])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regmock::{Op, RegMock, Write};

    fn desc(xtal: u32, if_hz: i32, x: u32) -> TunerDesc {
        TunerDesc {
            kit_name: "TC90527_FC5500".into(),
            demod_xtal_hz: xtal,
            if_freq_hz: if_hz,
            stream_param: x,
            ..Default::default()
        }
    }

    fn one(r: u8, v: u8) -> (u8, u8, Vec<u8>) {
        (0x10, r, vec![v])
    }

    fn tail(x: u32) -> Vec<(u8, u8, Vec<u8>)> {
        let b0 = if x & 0x10 != 0 { 0x04 } else { 0x06 };
        vec![(0x10, 0x75, vec![b0, 0x32, 0x01]), one(0xEF, 0x01), one(0x01, 0x40)]
    }

    #[test]
    fn configuration_by_crystal() {
        for (xtal, r11, r12) in [
            (16_000_000, 0x2D, 0x02),
            (18_000_000, 0x28, 0x02),
            (24_000_000, 0x1E, 0x02),
            (25_400_000, 0x18, 0x02),
            (32_000_000, 0x2D, 0x04),
            (36_000_000, 0x28, 0x04),
        ] {
            for (if_hz, x) in [(4_000_000, 0u32), (-3_000_000, 0x10)] {
                for driver in [VendorDriver::Series, VendorDriver::Dth30] {
                    let mut dev = Device::new(RegMock::default());
                    configure(&mut dev, 0x10, &desc(xtal, if_hz, x), driver).unwrap();
                    let mut b = vec![one(0x01, 0x40), one(0x04, 0x02), one(0x0F, 0x00), one(0x10, 0x00)];
                    b.extend([one(0x11, r11), one(0x12, r12)]);
                    b.extend(tail(x));
                    assert_eq!(dev.transport().take_writes(), vec![Write::Batch(b)], "{xtal} Hz, IF {if_hz}");
                }
            }
        }
    }

    #[test]
    fn zero_if_adds_a_block() {
        for (driver, val_62, present, r30) in [
            (VendorDriver::Series, -4_000_000i32, PRESENT_DEMOD_IF, 0x21u8),
            (VendorDriver::Dth30, -4_000_000, PRESENT_DEMOD_IF, 0x29),
            (VendorDriver::Dth30, -4_000_000, 0, 0x21),
            (VendorDriver::Dth30, 4_000_000, PRESENT_DEMOD_IF, 0x21),
        ] {
            let d = TunerDesc { val_62: val_62 as u32, present, ..desc(25_400_000, 0, 0) };
            let mut dev = Device::new(RegMock::default());
            configure(&mut dev, 0x10, &d, driver).unwrap();
            let mut b = vec![one(0x01, 0x40), one(0x04, 0x02), one(0x0F, 0x00), one(0x10, 0x00)];
            b.extend([one(0x11, 0x18), one(0x12, 0x02)]);
            for (r, v) in [(0x16, 0x04), (0x1C, 0x20), (0x1D, 0x00), (0x1E, 0x00), (0x1F, 0x00), (0x22, 0x80)] {
                b.push(one(r, v));
            }
            b.extend([one(0x23, 0x4C), one(0x30, r30), one(0x31, 0x00), one(0x32, 0x00), one(0x3D, 0x00)]);
            b.extend([one(0x47, 0x50), one(0xD0, 0x1E), one(0xD1, 0x0B), one(0xF8, 0x42), one(0xFA, 0x04)]);
            b.extend([(0x10, 0x31, vec![0x00, 0x00]), (0x10, 0x38, vec![0x01, 0x15])]);
            b.extend(tail(0));
            assert_eq!(dev.transport().take_writes(), vec![Write::Batch(b)], "{driver:?} {val_62} {present:#x}");
        }
    }

    #[test]
    fn other_crystal_or_if_is_refused() {
        let mut dev = Device::new(RegMock::default());
        let e = configure(&mut dev, 0x10, &desc(20_000_000, 4_000_000, 0), VendorDriver::Series).unwrap_err();
        std::assert_matches!(e, Error::Unsupported(_) if e.to_string().contains("crystal 20000000 Hz"), "{e}");
        assert_eq!(dev.transport().take_writes(), vec![]);
        assert_eq!(dev.i2c_batch_len(), 4 * 6, "left queued");
        let mut dev = Device::new(RegMock::default());
        let e = configure(&mut dev, 0x10, &desc(16_000_000, 44_000_000, 0), VendorDriver::Dth30).unwrap_err();
        std::assert_matches!(e, Error::Unsupported(_) if e.to_string().contains("IF 44000000 Hz"), "{e}");
        assert_eq!(dev.i2c_batch_len(), 6 * 6);
    }

    #[test]
    fn ts_pins() {
        for (mode, r1d, r1e) in [(0u8, 0x01u8, 0x15u8), (1, 0xA9, 0x95), (2, 0x55, 0x55)] {
            let mut dev = Device::new(RegMock::default());
            set_ts_pins(&mut dev, 0x10, mode).unwrap();
            assert_eq!(dev.transport().take_writes(), vec![Write::Batch(vec![one(0x1D, r1d), one(0x1E, r1e)])]);
        }
        let mut dev = Device::new(RegMock::default());
        set_ts_pins(&mut dev, 0x10, 3).unwrap();
        assert_eq!(dev.transport().take(), vec![]);
    }

    #[test]
    fn wake_and_standby_switch_0x1f_by_driver_and_kit() {
        for (driver, name, switched) in [
            (VendorDriver::Series, "TC90527_FC5500", true),
            (VendorDriver::Dth30, "TC90527_FC5500", true),
            (VendorDriver::Dth30, "TC90527_MTV808", false),
        ] {
            let mut dev = Device::new(RegMock::with_regs(&[(0x10, 0x1F, 0x51), (0x10, 0x04, 0xFF)]));
            wake(&mut dev, 0x10, driver, name).unwrap();
            let r1f = if switched { 0x5D } else { 0x51 };
            assert_eq!(
                dev.transport().take(),
                vec![
                    Op::R(0x10, 0x1F),
                    Op::W(0x10, 0x1F, r1f),
                    Op::R(0x10, 0x04),
                    Op::W(0x10, 0x04, 0xD7),
                    Op::W(0x10, 0x03, 0)
                ],
                "{driver:?} {name}"
            );
            standby(&mut dev, 0x10, driver, name).unwrap();
            // 0x1F back to 0x51 either way: bits 3:2 cleared again, or never set
            let r1f = 0x51;
            assert_eq!(
                dev.transport().take(),
                vec![
                    Op::Other,
                    Op::W(0x10, 0x03, 0x10),
                    Op::R(0x10, 0x04),
                    Op::W(0x10, 0x04, 0xFF),
                    Op::R(0x10, 0x1F),
                    Op::W(0x10, 0x1F, r1f),
                ],
                "{driver:?} {name}"
            );
            let mut dev = Device::new(RegMock::with_regs(&[(0x10, 0x1F, 0xFF)]));
            standby(&mut dev, 0x10, driver, name).unwrap();
            assert_eq!(dev.transport().reg(0x10, 0x1F), if switched { 0xF3 } else { 0xFF }, "{driver:?} {name}");
        }
    }

    #[test]
    fn status_preamble_by_layer_a() {
        for (b3, b4, fa) in [
            (0x60u8, 0x68u8, 0x06u8),
            (0x63, 0xEF, 0x06),
            (0x64, 0x68, 0x04),
            (0x40, 0x68, 0x04),
            (0x60, 0x60, 0x04),
            (0x00, 0x00, 0x04),
        ] {
            let mut dev = Device::new(RegMock::with_regs(&[(0x10, 0xB3, b3), (0x10, 0xB4, b4)]));
            status_preamble(&mut dev, 0x10).unwrap();
            assert_eq!(dev.transport().take(), vec![Op::R(0x10, 0xB3), Op::R(0x10, 0xB4), Op::W(0x10, 0xFA, fa)]);
        }
    }
}
