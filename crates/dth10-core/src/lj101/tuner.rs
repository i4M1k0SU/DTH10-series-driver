//! The satellite tuner of the SMT_LJ101 module: write-only, behind the
//! demodulator's 0xFE I2C repeater; the PLL word is the IF in MHz (doubled
//! below 1075 MHz) plus a VCO band.

use core::time::Duration;
use std::thread::sleep;

use crate::device::Device;
use crate::error::Result;
use crate::protocol::UsbControl;
use crate::tuner_bus::{TunerBus, TunerPath};

/// Write-only satellite tuner behind the demodulator's I2C repeater.
#[derive(Debug, Clone, Copy)]
pub struct Lj101Tuner {
    pub addr: u8,
    pub direct: bool,
}

/// PLL programming derived from the BS/CS IF in Hz.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PllWords {
    pub n: u16,
    pub band: u8,
    pub div: u8,
}

pub fn pll_words(if_hz: u64) -> PllWords {
    let fm = (if_hz + 500_000) / 1_000_000;
    let (mul, div) = if fm < 1075 { (2u64, 0x08u8) } else { (1, 0x00) };
    let n = ((if_hz * mul + 500_000) / 1_000_000) as u16;
    let band = if fm < 1100 {
        0
    } else if fm < 1200 {
        2
    } else if fm < 1400 {
        4
    } else if fm < 1500 {
        5
    } else if fm < 1600 {
        6
    } else {
        7
    };
    PllWords { n, band, div }
}

impl Lj101Tuner {
    fn bus(&self, demod_addr: u8) -> TunerBus {
        TunerBus {
            tuner_addr: self.addr,
            path: if self.direct { TunerPath::Direct } else { TunerPath::ViaDemod { demod_addr } },
        }
    }

    fn write<T: UsbControl>(&self, dev: &mut Device<T>, demod_addr: u8, data: &[u8]) -> Result<()> {
        self.bus(demod_addr).write(dev, data)
    }

    /// Initialisation: six writes, then 5 ms.
    pub fn init<T: UsbControl>(&self, dev: &mut Device<T>, demod_addr: u8) -> Result<()> {
        let r = (|| {
            self.write(dev, demod_addr, &[0x09, 0x03])?;
            self.write(dev, demod_addr, &[0x06, 0x2A, 0xE8, 0x84])?;
            self.write(dev, demod_addr, &[0x0A, 0x20, 0x78, 0x1E, 0x0A])?;
            self.write(dev, demod_addr, &[0x0F, 0x08, 0x0F])?;
            self.write(dev, demod_addr, &[0x18, 0x40])?;
            self.write(dev, demod_addr, &[0x04, 0x40])
        })();
        sleep(Duration::from_millis(5));
        r
    }

    /// Wake.
    pub fn wake<T: UsbControl>(&self, dev: &mut Device<T>, demod_addr: u8) -> Result<()> {
        self.write(dev, demod_addr, &[0x19, 0x60])?;
        self.write(dev, demod_addr, &[0x20, 0x00])?;
        self.write(dev, demod_addr, &[0x0E, 0x6F])
    }

    /// Standby (tuner part of the kit's sleep sequence).
    pub fn standby<T: UsbControl>(&self, dev: &mut Device<T>, demod_addr: u8) -> Result<()> {
        self.write(dev, demod_addr, &[0x0E, 0x2F])?;
        self.write(dev, demod_addr, &[0x16, 0x00])?;
        self.write(dev, demod_addr, &[0x02, 0x0C, 0xF6])?;
        self.write(dev, demod_addr, &[0x19, 0x40])?;
        self.write(dev, demod_addr, &[0x20, 0x08])
    }

    /// PLL programming for the BS/CS IF.
    pub fn set_frequency<T: UsbControl>(&self, dev: &mut Device<T>, demod_addr: u8, if_hz: u64) -> Result<()> {
        let p = pll_words(if_hz);
        self.write(dev, demod_addr, &[0x00, (p.n >> 8) as u8, (p.n & 0xFF) as u8, p.band << 4, p.div])?;
        self.write(dev, demod_addr, &[0x02, (p.band << 4) | 0x02])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pll_examples_from_doc() {
        assert_eq!(pll_words(1_049_480_000), PllWords { n: 0x0833, band: 0, div: 0x08 }); // BS-1
        assert_eq!(pll_words(1_087_840_000), PllWords { n: 0x0440, band: 0, div: 0 }); // BS-3
        assert_eq!(pll_words(1_126_200_000), PllWords { n: 0x0466, band: 2, div: 0 }); // BS-5
        assert_eq!(pll_words(1_279_640_000), PllWords { n: 0x0500, band: 4, div: 0 }); // BS-13
        assert_eq!(pll_words(1_613_000_000), PllWords { n: 0x064D, band: 7, div: 0 }); // ND-2
        assert_eq!(pll_words(2_053_000_000), PllWords { n: 0x0805, band: 7, div: 0 }); // ND-24
    }
}
