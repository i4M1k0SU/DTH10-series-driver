//! The demodulator of the TC90522 / TC90532 terrestrial kits (hw 0x102). The
//! vendor driver drives it with the TC90517 code (wake, standby, pre- /
//! post-tune, status: [`crate::tc90517::Tc90517`] with
//! [`TerVariant::Tc9052x`](crate::tc90517::TerVariant::Tc9052x)); only its
//! configuration, TS pins and the series driver's attenuator are its own.

use crate::device::Device;
use crate::error::{Error, Result};
use crate::kit::VendorDriver;
use crate::lj101::demod::write_block;
use crate::protocol::UsbControl;
use crate::tlv::TunerDesc;

/// The series driver's configuration, first block.
const BLOCK_1_SERIES: [(u8, u8); 23] = [
    (0x01, 0x00),
    (0x03, 0x00),
    (0x04, 0x02),
    (0x05, 0x00),
    (0x06, 0x80),
    (0x07, 0x00),
    (0x08, 0x00),
    (0x0C, 0x00),
    (0x0E, 0x77),
    (0x0F, 0x10),
    (0x11, 0x27),
    (0x12, 0x08),
    (0x13, 0x33),
    (0x14, 0x20),
    (0x15, 0x40),
    (0x16, 0x0C),
    (0x17, 0x70),
    (0x18, 0xB0),
    (0x1A, 0x30),
    (0x1C, 0x20),
    (0x1D, 0x02),
    (0x1E, 0x28),
    (0x1F, 0xAA),
];
/// The series driver's configuration, second block.
const BLOCK_2_SERIES: [(u8, u8); 24] = [
    (0x20, 0x00),
    (0x21, 0xFF),
    (0x22, 0x80),
    (0x23, 0x4C),
    (0x24, 0x4C),
    (0x25, 0x80),
    (0x26, 0x00),
    (0x28, 0x60),
    (0x2B, 0x40),
    (0x2C, 0x5A),
    (0x2D, 0x05),
    (0x2E, 0xFF),
    (0x2F, 0x00),
    (0x30, 0x20),
    (0x31, 0x00),
    (0x32, 0x00),
    (0x34, 0x0F),
    (0x38, 0x00),
    (0x39, 0xAA),
    (0x3A, 0x2D),
    (0x3B, 0x21),
    (0x3C, 0x3F),
    (0x3D, 0x24),
    (0x3F, 0x0C),
];
/// The series driver's configuration, third block.
const BLOCK_3_SERIES: [(u8, u8); 13] = [
    (0x40, 0x0C),
    (0x47, 0x00),
    (0x49, 0xE4),
    (0x4B, 0x80),
    (0x4C, 0x00),
    (0x4F, 0x05),
    (0x50, 0x00),
    (0x51, 0x68),
    (0x52, 0x20),
    (0x58, 0x10),
    (0x5C, 0x50),
    (0x5D, 0x00),
    (0x6F, 0x00),
];
/// The series driver's configuration, fourth block.
const BLOCK_4_SERIES: [(u8, u8); 17] = [
    (0x70, 0x38),
    (0x71, 0x00),
    (0x72, 0x00),
    (0x73, 0x33),
    (0x74, 0x13),
    (0x75, 0x02),
    (0x76, 0x00),
    (0x77, 0x01),
    (0x7D, 0xD2),
    (0x7F, 0x00),
    (0xBA, 0x00),
    (0xBB, 0x00),
    (0xBC, 0x00),
    (0xE4, 0xF4),
    (0xE7, 0x02),
    (0xEF, 0x01),
    (0xF4, 0x80),
];

/// The DT-H30/PCIEM-TS driver's configuration, first block.
const BLOCK_1_DTH30: [(u8, u8); 16] = [
    (0x01, 0x00),
    (0x04, 0x02),
    (0x06, 0x80),
    (0x0E, 0x77),
    (0x0F, 0x17),
    (0x11, 0x27),
    (0x12, 0x08),
    (0x13, 0x33),
    (0x14, 0x20),
    (0x18, 0xB0),
    (0x1A, 0x30),
    (0x1C, 0x20),
    (0x1D, 0x02),
    (0x1E, 0x28),
    (0x1F, 0xAA),
    (0x25, 0x80),
];
/// The DT-H30/PCIEM-TS driver's configuration, start of the second block
/// (the IF-dependent registers follow it in the same batch).
const BLOCK_2_HEAD_DTH30: [(u8, u8); 4] = [(0x28, 0x60), (0x2B, 0x40), (0x2C, 0x5A), (0x2D, 0x05)];
/// The DT-H30/PCIEM-TS driver's configuration, end of the second block.
const BLOCK_2_TAIL_DTH30: [(u8, u8); 5] = [(0x38, 0x00), (0x3A, 0x2D), (0x3B, 0x21), (0x3C, 0x3F), (0x3D, 0x24)];
/// The DT-H30/PCIEM-TS driver's configuration, fourth block.
const BLOCK_4_DTH30: [(u8, u8); 13] = [
    (0x70, 0x38),
    (0x73, 0x01),
    (0x74, 0x12),
    (0x76, 0x00),
    (0x77, 0x01),
    (0x7D, 0xD2),
    (0xBA, 0x00),
    (0xBB, 0x00),
    (0xBC, 0x00),
    (0xE4, 0xF4),
    (0xE7, 0x02),
    (0xEF, 0x01),
    (0xF4, 0x80),
];

/// The value both drivers write to 0x75 last: `(~(X >> 3)) & 2` (X = `stream_param`).
fn reg75(desc: &TunerDesc) -> u8 {
    (!(desc.stream_param >> 3) & 0x02) as u8
}

/// The configuration of the demodulator at `addr`, as the kit's Init (or
/// Open with flags bit5 set) writes it: four batches, each sent before the
/// next is queued (a failed batch ends it), then `0x75 <- (~(X >> 3)) & 2`.
///
/// * Series driver: fixed values; the descriptor's IF and kit name are not looked at.
/// * DT-H30/PCIEM-TS driver: the second batch depends on the descriptor's
///   demodulator IF (`val_62`): 0 or 0xFFFFFFFF (none given) or ±4 MHz; any
///   other value fails with the start of that batch left queued, as in the
///   vendor driver (the next transfer sends it). A kit name starting with
///   "TC90532" changes 0x16 and adds `0x58 <- 10`.
pub fn configure<T: UsbControl>(dev: &mut Device<T>, addr: u8, desc: &TunerDesc, driver: VendorDriver) -> Result<()> {
    match driver {
        VendorDriver::Series => {
            write_block(dev, addr, &BLOCK_1_SERIES)?;
            write_block(dev, addr, &BLOCK_2_SERIES)?;
            write_block(dev, addr, &BLOCK_3_SERIES)?;
            write_block(dev, addr, &BLOCK_4_SERIES)?;
        }
        VendorDriver::Dth30 => {
            write_block(dev, addr, &BLOCK_1_DTH30)?;
            for &(r, v) in &BLOCK_2_HEAD_DTH30 {
                dev.i2c_batch_append(addr, r, &[v])?;
            }
            let block_2 = if_registers(desc)?;
            for (r, v) in block_2.into_iter().chain(BLOCK_2_TAIL_DTH30) {
                dev.i2c_batch_append(addr, r, &[v])?;
            }
            dev.i2c_batch_flush()?;
            dev.i2c_batch_append(addr, 0x47, &[0x00])?;
            dev.i2c_batch_append(addr, 0x49, &[0xE4])?;
            if desc.kit_name.starts_with("TC90532") {
                dev.i2c_batch_append(addr, 0x58, &[0x10])?;
            }
            dev.i2c_batch_append(addr, 0x6F, &[0x00])?;
            dev.i2c_batch_flush()?;
            write_block(dev, addr, &BLOCK_4_DTH30)?;
        }
    }
    dev.i2c_reg_write(addr, 0x75, &[reg75(desc)])
}

/// The DT-H30/PCIEM-TS driver's IF-dependent registers of the second batch,
/// in order: 0x31, 0x32, 0x34, 0x39, 0x16, 0x30. The IF is the descriptor's
/// demodulator IF (`val_62`), negative = spectrum inverted (0x30 bit3).
fn if_registers(desc: &TunerDesc) -> Result<[(u8, u8); 6]> {
    let v = desc.val_62 as i32;
    let tc90532 = desc.kit_name.starts_with("TC90532");
    let inverted = if v < 0 { 1u8 } else { 0 };
    let (r31, r32, r16, r30_bit1) = if v == -1 || v == 0 {
        (0x00, 0x00, if tc90532 { 0x0C } else { 0x1C }, 0)
    } else if v.unsigned_abs() == 4_000_000 {
        (0x0D, 0x56, if tc90532 { 0x00 } else { 0x10 }, 1)
    } else {
        return Err(Error::Unsupported(format!("TC90522 / TC90532 demodulator IF {v} Hz")));
    };
    let r30 = (((inverted | 4) << 2) | r30_bit1) << 1;
    Ok([(0x31, r31), (0x32, r32), (0x34, 0x0F), (0x39, 0xAA), (0x16, r16), (0x30, r30)])
}

/// The TS pin control of both drivers (modes 0 / 1 / 2 = output / stopped /
/// third state): `0x1D[7:2]` = 0 / 0x2A / 0x15 (bits 1:0 kept),
/// `0x1E = (v & EF) | 20` and `0x1F = (v & 3) | A8` whatever the mode, each a
/// read and a queued write (the next read sends it), then the batch is sent.
/// Another mode only sends the batch.
pub fn set_ts_pins<T: UsbControl>(dev: &mut Device<T>, addr: u8, mode: u8) -> Result<()> {
    let r1d = match mode {
        0 => 0x00,
        1 => 0xA8,
        2 => 0x54,
        _ => return dev.i2c_batch_flush(),
    };
    let mut v = [0u8; 1];
    dev.i2c_reg_read(addr, 0x1D, &mut v)?;
    dev.i2c_batch_append(addr, 0x1D, &[(v[0] & 0x03) | r1d])?;
    dev.i2c_reg_read(addr, 0x1E, &mut v)?;
    dev.i2c_batch_append(addr, 0x1E, &[(v[0] & 0xEF) | 0x20])?;
    dev.i2c_reg_read(addr, 0x1F, &mut v)?;
    dev.i2c_batch_append(addr, 0x1F, &[(v[0] & 0x03) | 0xA8])?;
    dev.i2c_batch_flush()
}

/// Fixed I2C addresses of the two chips that carry the series kit's attenuator switch.
const ATT_CHIP_A: u8 = 0x10;
const ATT_CHIP_B: u8 = 0x11;

fn read1<T: UsbControl>(dev: &mut Device<T>, addr: u8, reg: u8) -> Result<u8> {
    let mut b = [0u8; 1];
    dev.i2c_reg_read(addr, reg, &mut b)?;
    Ok(b[0])
}

/// The series driver's attenuator switch (the vendor application's "use
/// attenuator"): the demodulator's `0x0E[2:0]` selects the chip, 1 → 0x10
/// with `0x1E[5:4]` = 3 / 2 (on / off), 7 → 0x11 with `0x1C[3:2]` = 3 / 2,
/// read-modify-write, immediate. `Ok(false)`: neither chip is selected,
/// nothing is written. The configuration sets `0x0E = 77`, which selects 0x11.
pub fn set_attenuator<T: UsbControl>(dev: &mut Device<T>, addr: u8, on: bool) -> Result<bool> {
    let (chip, reg) = match read1(dev, addr, 0x0E)? & 7 {
        1 => (ATT_CHIP_A, 0x1E),
        7 => (ATT_CHIP_B, 0x1C),
        _ => return Ok(false),
    };
    let v = read1(dev, chip, reg)?;
    let v = if chip == ATT_CHIP_B {
        (v & 0xF3) | if on { 0x0C } else { 0x08 }
    } else {
        (v & 0x3F) | if on { 0x30 } else { 0x20 }
    };
    dev.i2c_reg_write(chip, reg, &[v])?;
    Ok(true)
}

/// The series driver's attenuator read-back, which does not mirror the
/// switch: the demodulator's `0x0F[6:4]` selects the chip, 1 → `0x10:0x1D[1:0] == 3`,
/// 7 → `0x11:0x1C[3:2] == 3`; `None` when neither is selected. With the
/// configuration (`0x0E = 77`, `0x0F = 10`) the switch is on 0x11 and the
/// read-back on 0x10, as in the vendor driver.
pub fn attenuator<T: UsbControl>(dev: &mut Device<T>, addr: u8) -> Result<Option<bool>> {
    match (read1(dev, addr, 0x0F)? >> 4) & 7 {
        1 => Ok(Some(read1(dev, ATT_CHIP_A, 0x1D)? & 0x03 == 0x03)),
        7 => Ok(Some(read1(dev, ATT_CHIP_B, 0x1C)? & 0x0C == 0x0C)),
        _ => Ok(None),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regmock::{Op, RegMock, Write};

    fn batch(block: &[(u8, u8)], addr: u8) -> Write {
        Write::Batch(block.iter().map(|&(r, v)| (addr, r, vec![v])).collect())
    }

    fn desc(name: &str, val_62: u32, x: u32) -> TunerDesc {
        TunerDesc { kit_name: name.into(), hw_version: 0x102, val_62, stream_param: x, ..Default::default() }
    }

    #[test]
    fn block_sizes_fit_batch() {
        for n in [BLOCK_1_SERIES.len(), BLOCK_2_SERIES.len(), BLOCK_3_SERIES.len(), BLOCK_4_SERIES.len()] {
            assert!(n * 6 <= crate::device::I2C_BATCH_MAX);
        }
        for n in [BLOCK_1_DTH30.len(), BLOCK_2_HEAD_DTH30.len() + 6 + BLOCK_2_TAIL_DTH30.len(), BLOCK_4_DTH30.len()] {
            assert!(n * 6 <= crate::device::I2C_BATCH_MAX);
        }
    }

    #[test]
    fn series_configuration() {
        for (x, r75) in [(0u32, 0x02u8), (0x10, 0x00)] {
            // the kit name and the IF are not looked at
            for d in [desc("TC90532_MTV808", 0, x), desc("TC90532_MTV600", 4_000_000, x)] {
                let mut dev = Device::new(RegMock::default());
                configure(&mut dev, 0x10, &d, VendorDriver::Series).unwrap();
                assert_eq!(
                    dev.transport().take_writes(),
                    vec![
                        batch(&BLOCK_1_SERIES, 0x10),
                        batch(&BLOCK_2_SERIES, 0x10),
                        batch(&BLOCK_3_SERIES, 0x10),
                        batch(&BLOCK_4_SERIES, 0x10),
                        Write::Reg(0x10, 0x75, vec![r75]),
                    ]
                );
            }
        }
    }

    /// The second and third batches of the DT-H30/PCIEM-TS configuration,
    /// written out by hand for one kit name and IF.
    fn dth30_middle(r31: u8, r32: u8, r16: u8, r30: u8, tc90532: bool) -> [Write; 2] {
        let mut b2 = vec![(0x28, 0x60), (0x2B, 0x40), (0x2C, 0x5A), (0x2D, 0x05)];
        b2.extend([(0x31, r31), (0x32, r32), (0x34, 0x0F), (0x39, 0xAA), (0x16, r16), (0x30, r30)]);
        b2.extend([(0x38, 0x00), (0x3A, 0x2D), (0x3B, 0x21), (0x3C, 0x3F), (0x3D, 0x24)]);
        let mut b3 = vec![(0x47, 0x00), (0x49, 0xE4)];
        if tc90532 {
            b3.push((0x58, 0x10));
        }
        b3.push((0x6F, 0x00));
        [batch(&b2, 0x10), batch(&b3, 0x10)]
    }

    #[test]
    fn dth30_configuration_by_if_and_kit_name() {
        for (name, if_hz, r31, r32, r16, r30) in [
            ("TC90522_MTV808", 0u32, 0x00, 0x00, 0x1C, 0x20),
            ("TC90522_MTV808", 0xFFFF_FFFF, 0x00, 0x00, 0x1C, 0x28),
            ("TC90522_FC5500", 4_000_000, 0x0D, 0x56, 0x10, 0x22),
            ("TC90522_FC5500", (-4_000_000i32) as u32, 0x0D, 0x56, 0x10, 0x2A),
            ("TC90532_MTV808", 0, 0x00, 0x00, 0x0C, 0x20),
            ("TC90532_MTV808", 0xFFFF_FFFF, 0x00, 0x00, 0x0C, 0x28),
            ("TC90532_FC5500", 4_000_000, 0x0D, 0x56, 0x00, 0x22),
            ("TC90532_FC5500", (-4_000_000i32) as u32, 0x0D, 0x56, 0x00, 0x2A),
        ] {
            let tc90532 = name.starts_with("TC90532");
            let mut dev = Device::new(RegMock::default());
            configure(&mut dev, 0x10, &desc(name, if_hz, 0x10), VendorDriver::Dth30).unwrap();
            let [b2, b3] = dth30_middle(r31, r32, r16, r30, tc90532);
            assert_eq!(
                dev.transport().take_writes(),
                vec![batch(&BLOCK_1_DTH30, 0x10), b2, b3, batch(&BLOCK_4_DTH30, 0x10), Write::Reg(0x10, 0x75, vec![0])],
                "{name} IF {if_hz:#x}"
            );
        }
    }

    #[test]
    fn dth30_configuration_refuses_another_if_after_the_first_batch() {
        let mut dev = Device::new(RegMock::default());
        let e = configure(&mut dev, 0x10, &desc("TC90522_FC5500", 44_000_000, 0), VendorDriver::Dth30).unwrap_err();
        assert!(matches!(e, Error::Unsupported(_)) && e.to_string().contains("44000000"), "{e}");
        assert_eq!(dev.transport().take_writes(), vec![batch(&BLOCK_1_DTH30, 0x10)]);
        // the start of the second batch is still queued, as in the vendor driver
        assert_eq!(dev.i2c_batch_len(), 4 * 6);
    }

    #[test]
    fn ts_pins_rewrite_0x1d_by_mode() {
        for (mode, r1d) in [(0u8, 0x02u8), (1, 0xAA), (2, 0x56)] {
            let mut dev =
                Device::new(RegMock::with_regs(&[(0x10, 0x1D, 0xFE), (0x10, 0x1E, 0x1C), (0x10, 0x1F, 0x55)]));
            set_ts_pins(&mut dev, 0x10, mode).unwrap();
            assert_eq!(
                dev.transport().take(),
                vec![Op::R(0x10, 0x1D), Op::Other, Op::R(0x10, 0x1E), Op::Other, Op::R(0x10, 0x1F), Op::Other],
                "mode {mode}"
            );
            let t = dev.transport();
            assert_eq!((t.reg(0x10, 0x1D), t.reg(0x10, 0x1E), t.reg(0x10, 0x1F)), (r1d, 0x2C, 0xA9), "mode {mode}");
        }
        let mut dev = Device::new(RegMock::default());
        set_ts_pins(&mut dev, 0x10, 3).unwrap();
        assert_eq!(dev.transport().take(), vec![]);
    }

    #[test]
    fn attenuator_switch_and_read_back_use_different_chips() {
        // the configuration: 0x0E = 77 (switch on 0x11), 0x0F = 10 (read-back on 0x10)
        let mut dev = Device::new(RegMock::with_regs(&[(0x10, 0x0E, 0x77), (0x10, 0x0F, 0x10), (0x11, 0x1C, 0xF3)]));
        assert!(set_attenuator(&mut dev, 0x10, true).unwrap());
        assert_eq!(dev.transport().take(), vec![Op::R(0x10, 0x0E), Op::R(0x11, 0x1C), Op::W(0x11, 0x1C, 0xFF)]);
        assert_eq!(attenuator(&mut dev, 0x10).unwrap(), Some(false));
        assert_eq!(dev.transport().take(), vec![Op::R(0x10, 0x0F), Op::R(0x10, 0x1D)]);
        assert!(set_attenuator(&mut dev, 0x10, false).unwrap());
        assert_eq!(dev.transport().reg(0x11, 0x1C), 0xFB);

        // 0x0E[2:0] = 1: 0x10:0x1E bits 5:4; 0x0F[6:4] = 7: 0x11:0x1C bits 3:2
        let mut dev = Device::new(RegMock::with_regs(&[(0x10, 0x0E, 0x01), (0x10, 0x0F, 0x70), (0x10, 0x1E, 0xCF)]));
        assert!(set_attenuator(&mut dev, 0x10, false).unwrap());
        assert_eq!(dev.transport().take(), vec![Op::R(0x10, 0x0E), Op::R(0x10, 0x1E), Op::W(0x10, 0x1E, 0x2F)]);
        assert!(set_attenuator(&mut dev, 0x10, true).unwrap());
        assert_eq!(dev.transport().reg(0x10, 0x1E), 0x3F);
        assert_eq!(attenuator(&mut dev, 0x10).unwrap(), Some(false));
        dev.transport().take();
        let mut dev = Device::new(RegMock::with_regs(&[(0x10, 0x0F, 0x70), (0x11, 0x1C, 0x0C), (0x10, 0x1D, 0x03)]));
        assert_eq!(attenuator(&mut dev, 0x10).unwrap(), Some(true));
        let mut dev = Device::new(RegMock::with_regs(&[(0x10, 0x0F, 0x10), (0x10, 0x1D, 0x03)]));
        assert_eq!(attenuator(&mut dev, 0x10).unwrap(), Some(true));
    }

    #[test]
    fn attenuator_is_left_alone_when_no_chip_is_selected() {
        for sel in [0x00u8, 0x02, 0x06] {
            let mut dev = Device::new(RegMock::with_regs(&[(0x10, 0x0E, sel), (0x10, 0x0F, sel << 4)]));
            assert!(!set_attenuator(&mut dev, 0x10, true).unwrap());
            assert_eq!(attenuator(&mut dev, 0x10).unwrap(), None);
            assert_eq!(dev.transport().take(), vec![Op::R(0x10, 0x0E), Op::R(0x10, 0x0F)]);
        }
    }
}
