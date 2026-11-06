//! The demodulator configuration of the SHARP_8011 module. The module carries
//! the same TC90517 as the SMT_LJ101 module and the vendor driver drives it
//! with the same code (wake / standby, pre- / post-tune, TS pins, status,
//! TSID, attenuator, booster: [`crate::tc90517::Tc90517`] and
//! [`crate::lj101::Tc90517Sat`]); only the register values written by the two
//! kits' Init / Open configuration are its own.

use crate::device::Device;
use crate::error::Result;
use crate::lj101::demod::write_block;
use crate::protocol::UsbControl;
use crate::tlv::TunerDesc;

/// Terrestrial side (hw 0x102), first block.
const BLOCK_T1: [(u8, u8); 24] = [
    (0x01, 0x00),
    (0x02, 0x00),
    (0x03, 0x00),
    (0x04, 0x00),
    (0x05, 0x40),
    (0x06, 0x00),
    (0x07, 0x40),
    (0x08, 0x00),
    (0x0C, 0x00),
    (0x0F, 0x5F),
    (0x11, 0x21),
    (0x12, 0x0B),
    (0x13, 0x2C),
    (0x14, 0x0A),
    (0x15, 0x40),
    (0x17, 0x70),
    (0x18, 0x31),
    (0x19, 0x13),
    (0x1A, 0x31),
    (0x1B, 0x13),
    (0x1C, 0x00),
    (0x1D, 0x00),
    (0x1E, 0x00),
    (0x1F, 0x00),
];
/// Terrestrial side, second block.
const BLOCK_T2: [(u8, u8); 28] = [
    (0x20, 0x00),
    (0x21, 0xFF),
    (0x22, 0x83),
    (0x23, 0x78),
    (0x24, 0x4C),
    (0x25, 0x80),
    (0x26, 0x00),
    (0x27, 0x0C),
    (0x28, 0x60),
    (0x29, 0x6B),
    (0x2A, 0x40),
    (0x2B, 0x40),
    (0x2C, 0xFF),
    (0x2D, 0x00),
    (0x2E, 0xFF),
    (0x2F, 0x00),
    (0x30, 0x28),
    (0x31, 0x0F),
    (0x32, 0xA0),
    (0x34, 0x3F),
    (0x38, 0x01),
    (0x39, 0x70),
    (0x3A, 0x10),
    (0x3B, 0x10),
    (0x3C, 0x00),
    (0x3D, 0x10),
    (0x3E, 0x08),
    (0x3F, 0x0C),
];
/// Terrestrial side, third block (the same values as the SMT_LJ101 module's).
const BLOCK_T3: [(u8, u8); 23] = [
    (0x40, 0x0C),
    (0x41, 0x00),
    (0x42, 0x00),
    (0x43, 0x4F),
    (0x44, 0xFF),
    (0x46, 0x20),
    (0x47, 0x00),
    (0x48, 0x90),
    (0x49, 0xE6),
    (0x4A, 0x02),
    (0x4B, 0x80),
    (0x4C, 0x00),
    (0x4F, 0x05),
    (0x50, 0x00),
    (0x51, 0x68),
    (0x52, 0x20),
    (0x54, 0x57),
    (0x55, 0xF1),
    (0x56, 0x20),
    (0x57, 0x70),
    (0x5C, 0x50),
    (0x5D, 0x00),
    (0x5F, 0x80),
];
/// Terrestrial side, fourth block.
const BLOCK_T4: [(u8, u8); 25] = [
    (0x70, 0x18),
    (0x71, 0x00),
    (0x72, 0x00),
    (0x75, 0x00),
    (0x76, 0x02),
    (0x77, 0x00),
    (0x7C, 0x00),
    (0x7D, 0x52),
    (0x7F, 0x00),
    (0xB0, 0xA0),
    (0xB1, 0x02),
    (0xB2, 0x3D),
    (0xB3, 0x25),
    (0xB4, 0x8B),
    (0xB5, 0x4B),
    (0xB6, 0x3F),
    (0xB7, 0xFF),
    (0xB8, 0xFF),
    (0xB9, 0xFC),
    (0xBA, 0x00),
    (0xBB, 0x00),
    (0xBC, 0x00),
    (0xE4, 0x84),
    (0xEC, 0x00),
    (0xEF, 0x00),
];
/// Satellite side (hw 0x402), first block.
const BLOCK_S1: [(u8, u8); 23] = [
    (0x03, 0x00),
    (0x04, 0x00),
    (0x06, 0x00),
    (0x07, 0x00),
    (0x09, 0x00),
    (0x0A, 0xFF),
    (0x0C, 0x51),
    (0x0D, 0xF6),
    (0x0E, 0xF0),
    (0x0F, 0x52),
    (0x10, 0xB2),
    (0x11, 0x40),
    (0x12, 0x30),
    (0x13, 0x00),
    (0x14, 0x00),
    (0x15, 0x10),
    (0x17, 0x20),
    (0x1A, 0x04),
    (0x1B, 0x00),
    (0x1C, 0x00),
    (0x1D, 0x00),
    (0x1E, 0x00),
    (0x1F, 0x00),
];
/// Satellite side, second block.
const BLOCK_S2: [(u8, u8); 20] = [
    (0x20, 0x00),
    (0x38, 0x40),
    (0x39, 0x10),
    (0x3B, 0x90),
    (0x51, 0xB0),
    (0x52, 0x89),
    (0x53, 0xB3),
    (0x57, 0x00),
    (0x5A, 0x2D),
    (0x5B, 0xD3),
    (0x85, 0x69),
    (0x87, 0x04),
    (0x8D, 0x00),
    (0x8E, 0x00),
    (0x8F, 0x00),
    (0x90, 0x00),
    (0xA3, 0x77),
    (0xA4, 0x00),
    (0xA5, 0x00),
    (0xA6, 0x04),
];

/// The terrestrial kit's (hw 0x102) configuration of the demodulator at
/// `addr`: four batches, then `0x75 <- 02` (fixed, whatever the descriptor
/// says). Unlike the SMT_LJ101 kit it never touches the satellite side.
pub fn configure_ter<T: UsbControl>(dev: &mut Device<T>, addr: u8) -> Result<()> {
    write_block(dev, addr, &BLOCK_T1)?;
    write_block(dev, addr, &BLOCK_T2)?;
    write_block(dev, addr, &BLOCK_T3)?;
    write_block(dev, addr, &BLOCK_T4)?;
    dev.i2c_reg_write(addr, 0x75, &[0x02])
}

/// The satellite kit's (hw 0x402) configuration of the demodulator at
/// `addr`: `0x01 <- 90`, two batches, then `0x04 <- (~(X >> 3)) & 2`
/// (X = `stream_param`). Unlike the SMT_LJ101 kit it keeps the descriptor's
/// address whatever `flags` bit5 says and never touches the terrestrial side.
pub fn configure_sat<T: UsbControl>(dev: &mut Device<T>, addr: u8, desc: &TunerDesc) -> Result<()> {
    dev.i2c_reg_write(addr, 0x01, &[0x90])?;
    write_block(dev, addr, &BLOCK_S1)?;
    write_block(dev, addr, &BLOCK_S2)?;
    let x_bit = (!(desc.stream_param >> 3) & 0x02) as u8;
    dev.i2c_reg_write(addr, 0x04, &[x_bit])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regmock::{RegMock, Write};

    fn batch(block: &[(u8, u8)], addr: u8) -> Write {
        Write::Batch(block.iter().map(|&(r, v)| (addr, r, vec![v])).collect())
    }

    #[test]
    fn block_sizes_fit_batch() {
        for b in [BLOCK_T1.len(), BLOCK_T2.len(), BLOCK_T3.len(), BLOCK_T4.len(), BLOCK_S1.len(), BLOCK_S2.len()] {
            assert!(b * 6 <= crate::device::I2C_BATCH_MAX);
        }
    }

    #[test]
    fn terrestrial_configuration() {
        let mut dev = Device::new(RegMock::default());
        configure_ter(&mut dev, 0x18).unwrap();
        let w = dev.transport().take_writes();
        assert_eq!(
            w,
            vec![
                batch(&BLOCK_T1, 0x18),
                batch(&BLOCK_T2, 0x18),
                batch(&BLOCK_T3, 0x18),
                batch(&BLOCK_T4, 0x18),
                Write::Reg(0x18, 0x75, vec![0x02]),
            ]
        );
        // a few values that differ from the SMT_LJ101 module's
        let t = dev.transport();
        assert_eq!((t.reg(0x18, 0x0F), t.reg(0x18, 0x23), t.reg(0x18, 0x7D), t.reg(0x18, 0xEF)), (0x5F, 0x78, 0x52, 0));
        assert_eq!((t.reg(0x18, 0xB0), t.reg(0x18, 0xB9)), (0xA0, 0xFC));
        // the third block is the SMT_LJ101 module's
        assert_eq!(BLOCK_T3, crate::lj101::demod::BLOCK_A3);
    }

    #[test]
    fn satellite_configuration() {
        for (x, bit) in [(0u32, 0x02u8), (0x10, 0x00), (0x08, 0x02)] {
            let desc = TunerDesc { stream_param: x, flags: 0x20, ..Default::default() };
            let mut dev = Device::new(RegMock::default());
            configure_sat(&mut dev, 0x19, &desc).unwrap();
            assert_eq!(
                dev.transport().take_writes(),
                vec![
                    Write::Reg(0x19, 0x01, vec![0x90]),
                    batch(&BLOCK_S1, 0x19),
                    batch(&BLOCK_S2, 0x19),
                    Write::Reg(0x19, 0x04, vec![bit]),
                ],
                "stream_param 0x{x:x}"
            );
        }
    }
}
