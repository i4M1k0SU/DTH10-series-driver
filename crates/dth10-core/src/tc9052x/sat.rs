//! The satellite kit (ISDB-S, hw 0x402) of the TC90522 / TC90532 modules:
//! `TC90522_MTV600`, `TC90532_MTV600` and (as the series driver maps it)
//! `TC90532_MTV808`, with the MTV600 tuner ([`Mtv600`]).
//!
//! The demodulator's satellite side is driven with the SMT_LJ101 hw 0x402
//! kit's code for the restart after the PLL, the TSID selection and the
//! signal status ([`Tc90517Sat`]); the configuration, the TS pins of the
//! series driver, the sleep and the booster are the kit's own. The two vendor
//! drivers ([`VendorDriver`]) differ:
//!
//! | | series driver | DT-H30/PCIEM-TS driver |
//! |---|---|---|
//! | configuration | `01=90`, 26 + 15 registers, `04` from X | `01=90`, 12 + 4 registers, `04` from X |
//! | TS pins 0x1F (output / stopped / mode 2) | `08` / `2A` / `19` | `88` / `AA` / `99` (the SMT_LJ101 values) |
//! | Init end, Close | `17=01` | tuner `0x23 \|= 1`, TS stopped, `17=01` |
//! | Open | `17=00` | TS output, `17=00`, tuner `0x23 &= ~1` (each step only after the previous one succeeded) |
//! | booster | `0x10:0x0E[2:0]` selects `0x10:0x1E[5:4]` or `0x11:0x1C[3:2]` | none |
//!
//! Neither driver puts the descriptor's tuner crystal, IF or demodulator
//! crystal to use; the DT-H30/PCIEM-TS driver reads the tuner option bit1.

use core::time::Duration;
use std::thread::sleep;

use super::mtv600::Mtv600;
use crate::device::{self, Device};
use crate::error::{Error, Result};
use crate::kit::{SignalStatus, VendorDriver};
use crate::lj101::Tc90517Sat;
use crate::lj101::demod::write_block;
use crate::protocol::UsbControl;
use crate::tlv::TunerDesc;
use crate::tuner_bus::{TunerBus, TunerPath};

/// The series driver's configuration, first block.
pub const BLOCK_S1_SERIES: [(u8, u8); 26] = [
    (0x03, 0x00),
    (0x04, 0x00),
    (0x06, 0x00),
    (0x07, 0x01),
    (0x08, 0x10),
    (0x09, 0x00),
    (0x0A, 0xFF),
    (0x0C, 0x59),
    (0x0D, 0xF2),
    (0x0E, 0xF0),
    (0x0F, 0x50),
    (0x10, 0xB2),
    (0x11, 0x00),
    (0x12, 0x30),
    (0x13, 0x80),
    (0x14, 0x00),
    (0x15, 0x00),
    (0x17, 0x00),
    (0x18, 0x00),
    (0x19, 0x00),
    (0x1A, 0x00),
    (0x1B, 0x00),
    (0x1C, 0x0A),
    (0x1D, 0xAA),
    (0x1E, 0x20),
    (0x1F, 0x08),
];
/// The series driver's configuration, second block.
pub const BLOCK_S2_SERIES: [(u8, u8); 15] = [
    (0x20, 0x00),
    (0x38, 0x40),
    (0x39, 0x10),
    (0x3B, 0x90),
    (0x85, 0x6A),
    (0x87, 0x00),
    (0x8D, 0x00),
    (0x8E, 0x00),
    (0x8F, 0x00),
    (0x90, 0x00),
    (0xA3, 0x11),
    (0xA4, 0x00),
    (0xA5, 0x40),
    (0xA6, 0x04),
    (0xA9, 0x17),
];
/// The DT-H30/PCIEM-TS driver's configuration, first block.
pub const BLOCK_S1_DTH30: [(u8, u8); 12] = [
    (0x07, 0x71),
    (0x08, 0x77),
    (0x0C, 0x59),
    (0x0D, 0xF2),
    (0x0F, 0x50),
    (0x13, 0x80),
    (0x18, 0x04),
    (0x19, 0x00),
    (0x1C, 0x0A),
    (0x1D, 0xAA),
    (0x1E, 0x20),
    (0x1F, 0x08),
];
/// The DT-H30/PCIEM-TS driver's configuration, second block.
pub const BLOCK_S2_DTH30: [(u8, u8); 4] = [(0x85, 0x6A), (0xA3, 0x11), (0xA5, 0x40), (0xA9, 0x17)];

/// A register block: (register, value) pairs written in one batch.
type Block = &'static [(u8, u8)];

/// The fixed chips of the series driver's booster.
const BOOSTER_SEL: u8 = 0x10;
const BOOSTER_ALT: u8 = 0x11;

/// The kit names (with hw 0x402).
pub const KIT_NAMES: [&str; 3] = ["TC90522_MTV600", "TC90532_MTV600", "TC90532_MTV808"];

/// The satellite kit.
pub struct Tc9052xSatKit<T: UsbControl> {
    pub dev: Device<T>,
    pub desc: TunerDesc,
    pub driver: VendorDriver,
    pub demod: Tc90517Sat,
    pub tuner: Mtv600,
    open_count: u32,
    last_if_hz: Option<u64>,
}

impl<T: UsbControl> Tc9052xSatKit<T> {
    /// Whether `desc` names this kit: one of [`KIT_NAMES`] with hw 0x402.
    pub fn supports(desc: &TunerDesc) -> bool {
        desc.hw_version == 0x402 && super::driver_for(desc, 0).is_some()
    }

    /// The kit for `desc`, following `driver` (see [`super::driver_for`]).
    pub fn new(dev: Device<T>, desc: TunerDesc, driver: VendorDriver) -> Result<Self> {
        if !Self::supports(&desc) {
            return Err(Error::Unsupported(format!(
                "satellite tuner kit {} hw 0x{:x} is not a TC90522 / TC90532 hw 0x402 kit",
                desc.kit_name, desc.hw_version
            )));
        }
        let path = if desc.tuner_direct_i2c() {
            TunerPath::Direct
        } else {
            TunerPath::ViaDemod { demod_addr: desc.demod_i2c_addr }
        };
        Ok(Tc9052xSatKit {
            demod: Tc90517Sat::new(desc.demod_i2c_addr),
            tuner: Mtv600::new(TunerBus { tuner_addr: desc.tuner_i2c_addr, path }, driver),
            dev,
            desc,
            driver,
            open_count: 0,
            last_if_hz: None,
        })
    }

    /// The demodulator configuration: `01=90` (immediate), two batches, then
    /// `04 = (~(X >> 3)) & 2` (X = `stream_param`), at the descriptor's address.
    fn configure(&mut self) -> Result<()> {
        let (b1, b2): (Block, Block) = match self.driver {
            VendorDriver::Series => (&BLOCK_S1_SERIES, &BLOCK_S2_SERIES),
            VendorDriver::Dth30 => (&BLOCK_S1_DTH30, &BLOCK_S2_DTH30),
        };
        let a = self.demod.addr;
        self.dev.i2c_reg_write(a, 0x01, &[0x90])?;
        write_block(&mut self.dev, a, b1)?;
        write_block(&mut self.dev, a, b2)?;
        let x_bit = (!(self.desc.stream_param >> 3) & 0x02) as u8;
        self.dev.i2c_reg_write(a, 0x04, &[x_bit])
    }

    /// TS pin control: 0 = output, 1 = stopped, 2 = the third setting.
    /// `0x1C = (0x1C & 3F) | 00 / 80 / 40` and 0x1F, in one batch.
    pub fn set_ts_pins(&mut self, mode: u8) -> Result<()> {
        if self.driver == VendorDriver::Dth30 {
            return self.demod.set_ts_pins(&mut self.dev, mode);
        }
        let a = self.demod.addr;
        let mut b = [0u8; 1];
        self.dev.i2c_reg_read(a, 0x1C, &mut b)?;
        let v = b[0] & 0x3F;
        let (r1c, r1f) = match mode {
            0 => (v, 0x08),
            1 => (v | 0x80, 0x2A),
            2 => (v | 0x40, 0x19),
            _ => return Ok(()),
        };
        self.dev.i2c_batch_append(a, 0x1C, &[r1c])?;
        self.dev.i2c_batch_append(a, 0x1F, &[r1f])?;
        self.dev.i2c_batch_flush()
    }

    /// Sleep (Init end, Close). Series: `0x17 <- 1`. DT-H30/PCIEM-TS: tuner
    /// `0x23 |= 1`, TS stopped, `0x17 <- 1`, each after the previous one
    /// succeeded.
    fn sleep(&mut self) -> Result<()> {
        if self.driver == VendorDriver::Dth30 {
            self.tuner.sleep(&mut self.dev)?;
            self.set_ts_pins(1)?;
        }
        self.demod.sleep(&mut self.dev)
    }

    /// Wake (Open). Series: `0x17 <- 0`. DT-H30/PCIEM-TS: TS output,
    /// `0x17 <- 0`, tuner `0x23 &= ~1`, each after the previous one succeeded.
    fn wake(&mut self) -> Result<()> {
        if self.driver == VendorDriver::Series {
            return self.demod.wake(&mut self.dev);
        }
        self.set_ts_pins(0)?;
        self.demod.wake(&mut self.dev)?;
        self.tuner.wake(&mut self.dev)
    }

    fn configure_all(&mut self) -> Result<()> {
        self.configure()?;
        self.tuner.init(&mut self.dev, &self.desc)
    }

    /// Init: power-up, demodulator configuration, tuner init, sleep. With
    /// flags bit5 set only the power-down, as in the vendor drivers.
    pub fn init(&mut self) -> Result<()> {
        if self.desc.skip_init_config() {
            return self.dev.power_down(&self.desc);
        }
        self.dev.power_up(&self.desc)?;
        self.configure_all()?;
        self.last_if_hz = None;
        self.sleep()
    }

    /// Open (reference counted): wake (with flags bit5 set: power-up,
    /// configuration and tuner init), then GPIO mask C <- open value, which
    /// the vendor drivers write whether or not the steps before succeeded.
    pub fn open(&mut self) -> Result<()> {
        self.open_count += 1;
        if self.open_count != 1 {
            return Ok(());
        }
        let r = if self.desc.skip_init_config() {
            self.dev.power_up(&self.desc).and_then(|()| {
                self.last_if_hz = None;
                self.configure_all()
            })
        } else {
            self.wake()
        };
        let g = self.dev.gpio_write(self.desc.gpio_mask_c, self.desc.gpio_c_open);
        let r = r.and(g);
        if r.is_err() {
            self.open_count = 0;
        }
        r
    }

    /// Close: sleep (with flags bit5 set: power-down). The GPIO of mask C is
    /// left as Open set it.
    pub fn close(&mut self) -> Result<()> {
        if self.open_count == 0 {
            return Ok(());
        }
        self.open_count -= 1;
        if self.open_count != 0 {
            return Ok(());
        }
        if self.desc.skip_init_config() { self.dev.power_down(&self.desc) } else { self.sleep() }
    }

    pub fn is_open(&self) -> bool {
        self.open_count > 0
    }

    /// Tune to a BS/CS IF (RF − 10.678 GHz) in Hz: TS stopped, the tuner at
    /// the IF in kHz, 100 ms, demodulator restart (`03=01`, `A5 |= 1`), 150 ms,
    /// TS output.
    pub fn tune(&mut self, if_hz: u64) -> Result<()> {
        if !self.is_open() {
            return Err(Error::Protocol("kit is not open"));
        }
        self.set_ts_pins(1)?;
        self.tuner.tune(&mut self.dev, (if_hz / 1000) as u32)?;
        sleep(Duration::from_millis(100));
        let r = self.demod.restart(&mut self.dev);
        sleep(Duration::from_millis(150));
        r?;
        self.set_ts_pins(0)?;
        self.last_if_hz = Some(if_hz);
        Ok(())
    }

    /// The IF of the last successful tune since the last configuration.
    pub fn last_if_hz(&self) -> Option<u64> {
        self.last_if_hz
    }

    /// Select the transport stream by TSID.
    pub fn set_tsid(&mut self, tsid: u16) -> Result<()> {
        self.demod.set_tsid(&mut self.dev, tsid)
    }

    /// Signal status (the SMT_LJ101 satellite status: lock from 0xC3, C/N from 0xBC).
    pub fn signal_status(&mut self, mask: u32) -> Result<SignalStatus> {
        let mut st = SignalStatus { mask, ..Default::default() };
        self.demod.status(&mut self.dev, &mut st)?;
        Ok(st)
    }

    /// Antenna (LNB) power: the bridge GPIO supply of the models that switch
    /// it there, no effect on others (both drivers hand it to the bridge).
    pub fn lnb_power(&mut self, volt: u32) -> Result<bool> {
        if let Some(gpio) = device::LnbGpio::of(&self.desc) {
            return device::lnb_set(&self.dev, gpio, volt);
        }
        Ok(true)
    }

    /// The booster (the vendor application's "use booster"). Series driver:
    /// `0x10:0x0E[2:0]` = 1 selects `0x10:0x1E`, set to `(v & 3F) | 30` (on) /
    /// `| 20` (off, so bit4 stays as it was), = 7 selects `0x11:0x1C`, bits
    /// 3:2 = 3 / 2; any other value: not switched. The chips are fixed,
    /// whatever the descriptor's demodulator address. DT-H30/PCIEM-TS driver:
    /// no booster. `Ok(false)`: not switched.
    pub fn set_booster(&mut self, on: bool) -> Result<bool> {
        if self.driver == VendorDriver::Dth30 {
            return Ok(false);
        }
        let mut b = [0u8; 1];
        self.dev.i2c_reg_read(BOOSTER_SEL, 0x0E, &mut b)?;
        let (chip, reg) = match b[0] & 7 {
            1 => (BOOSTER_SEL, 0x1E),
            7 => (BOOSTER_ALT, 0x1C),
            _ => return Ok(false),
        };
        self.dev.i2c_reg_read(chip, reg, &mut b)?;
        let v = if chip == BOOSTER_ALT {
            (b[0] & 0xF3) | if on { 0x0C } else { 0x08 }
        } else {
            (b[0] & 0x3F) | if on { 0x30 } else { 0x20 }
        };
        self.dev.i2c_reg_write(chip, reg, &[v])?;
        Ok(true)
    }

    /// Whether the booster is on, as the vendor drivers report it. Series:
    /// `(0x1C & 3) == 3` of this demodulator (the SMT_LJ101 kit's read-back,
    /// which does not look at the bits the switch sets). DT-H30/PCIEM-TS:
    /// always off.
    pub fn booster(&mut self) -> Result<bool> {
        match self.driver {
            VendorDriver::Series => self.demod.booster(&mut self.dev),
            VendorDriver::Dth30 => Ok(false),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regmock::{Op, RegMock, Write};

    fn desc(name: &str, hw: u32, flags: u32) -> TunerDesc {
        TunerDesc {
            kit_name: name.into(),
            hw_version: hw,
            flags,
            demod_i2c_addr: 0x11,
            tuner_i2c_addr: 0x60,
            gpio_mask_c: 0x10,
            gpio_c_open: 0x10,
            ..Default::default()
        }
    }

    fn kit(driver: VendorDriver, flags: u32) -> Tc9052xSatKit<RegMock> {
        Tc9052xSatKit::new(Device::new(RegMock::default()), desc("TC90532_MTV600", 0x402, flags), driver).unwrap()
    }

    fn batch(block: &[(u8, u8)], addr: u8) -> Write {
        Write::Batch(block.iter().map(|&(r, v)| (addr, r, vec![v])).collect())
    }

    /// A tuner write through the repeater of 0x11.
    fn tw(reg: u8, v: u8) -> Write {
        Write::Raw(0x11, vec![0xFE, 0xC0, reg, v])
    }

    /// A tuner read through the repeater of 0x11 (the mock reads zeros).
    fn tr(reg: u8) -> [Write; 2] {
        [Write::Raw(0x11, vec![0xFE, 0xC0, reg]), Write::Raw(0x11, vec![0xFE, 0xC1])]
    }

    #[test]
    fn kit_names_and_hw_version() {
        for (name, hw, ok) in [
            ("TC90522_MTV600", 0x402, true),
            ("TC90532_MTV600", 0x402, true),
            ("TC90532_MTV808", 0x402, true),
            ("TC90522_MTV808", 0x402, false),
            ("TC90532_MTV600", 0x102, false),
            ("TC90522_FC5500", 0x402, false),
            ("SMT_LJ101", 0x402, false),
        ] {
            let d = desc(name, hw, 0);
            assert_eq!(Tc9052xSatKit::<RegMock>::supports(&d), ok, "{name} 0x{hw:x}");
            let k = Tc9052xSatKit::new(Device::new(RegMock::default()), d, VendorDriver::Series);
            assert_eq!(k.is_ok(), ok, "{name} 0x{hw:x}");
        }
        assert_eq!(KIT_NAMES.map(|n| Tc9052xSatKit::<RegMock>::supports(&desc(n, 0x402, 0))), [true; 3]);
    }

    #[test]
    fn block_sizes_fit_batch() {
        for b in [BLOCK_S1_SERIES.len(), BLOCK_S2_SERIES.len(), BLOCK_S1_DTH30.len(), BLOCK_S2_DTH30.len()] {
            assert!(b * 6 <= crate::device::I2C_BATCH_MAX);
        }
    }

    #[test]
    fn configurations() {
        for (driver, b1, b2) in [
            (VendorDriver::Series, &BLOCK_S1_SERIES[..], &BLOCK_S2_SERIES[..]),
            (VendorDriver::Dth30, &BLOCK_S1_DTH30[..], &BLOCK_S2_DTH30[..]),
        ] {
            for (x, bit) in [(0u32, 0x02u8), (0x10, 0x00)] {
                let mut k = kit(driver, 0);
                k.desc.stream_param = x;
                k.configure().unwrap();
                assert_eq!(
                    k.dev.transport().take_writes(),
                    vec![
                        Write::Reg(0x11, 0x01, vec![0x90]),
                        batch(b1, 0x11),
                        batch(b2, 0x11),
                        Write::Reg(0x11, 0x04, vec![bit])
                    ],
                    "{driver:?} X 0x{x:x}"
                );
            }
        }
    }

    #[test]
    fn series_init_open_tune_close() {
        let mut k = kit(VendorDriver::Series, 0);
        k.init().unwrap();
        let w = k.dev.transport().take_writes();
        assert_eq!(w.len(), 4 + 36 + 9 + 1, "{w:?}");
        assert_eq!(w[0], Write::Reg(0x11, 0x01, vec![0x90]));
        assert_eq!(w[4], tw(0x20, 0x0B));
        assert_eq!(w[39], tw(0x43, 0x04));
        assert_eq!(w[40..42], tr(0x26));
        assert_eq!(w[42], tw(0x26, 0x00));
        // the sleep: 0x17 <- 1 only
        assert_eq!(w.last(), Some(&Write::Reg(0x11, 0x17, vec![0x01])));
        assert!(k.tune(1_318_000_000).is_err(), "not open");

        k.open().unwrap();
        assert_eq!(k.dev.transport().take_writes(), vec![Write::Reg(0x11, 0x17, vec![0x00])]);
        k.tune(1_318_000_000).unwrap();
        let w = k.dev.transport().take_writes();
        // TS stopped (0x1C = 0x0A from the configuration), then the PLL
        assert_eq!(w[0], Write::Batch(vec![(0x11, 0x1C, vec![0x8A]), (0x11, 0x1F, vec![0x2A])]));
        assert_eq!(w[1..3], tr(0x23));
        assert_eq!(w[3], tw(0x23, 0x40));
        assert_eq!(w[4..7], [tw(0x24, 0x00), tw(0x25, 0x00), tr(0x31)[0].clone()]);
        // restart and TS output
        let n = w.len();
        assert_eq!(
            w[n - 3..],
            [
                Write::Reg(0x11, 0x03, vec![0x01]),
                Write::Reg(0x11, 0xA5, vec![0x41]),
                Write::Batch(vec![(0x11, 0x1C, vec![0x0A]), (0x11, 0x1F, vec![0x08])]),
            ]
        );
        assert_eq!(k.last_if_hz(), Some(1_318_000_000));
        k.set_tsid(0x40F1).unwrap();
        assert_eq!(
            k.dev.transport().take_writes(),
            vec![Write::Batch(vec![(0x11, 0x8F, vec![0x40]), (0x11, 0x90, vec![0xF1])])]
        );
        k.close().unwrap();
        assert_eq!(k.dev.transport().take_writes(), vec![Write::Reg(0x11, 0x17, vec![0x01])]);
    }

    #[test]
    fn dth30_init_open_tune_close() {
        let mut k = kit(VendorDriver::Dth30, 0);
        k.init().unwrap();
        let w = k.dev.transport().take_writes();
        // configuration, table, 0x26 / 0x23 / 0x22, 0x26 bit0, then the sleep
        assert_eq!(w.len(), 4 + 36 + 9 + 3 + 5, "{w:?}");
        assert_eq!(w[1], batch(&BLOCK_S1_DTH30, 0x11));
        assert_eq!(w[4 + 2], tw(0x22, 0x09));
        let n = w.len();
        assert_eq!(w[n - 6..n - 5], [tw(0x26, 0x01)]);
        assert_eq!(
            w[n - 5..],
            [
                tr(0x23)[0].clone(),
                tr(0x23)[1].clone(),
                tw(0x23, 0x01),
                Write::Batch(vec![(0x11, 0x1C, vec![0x8A]), (0x11, 0x1F, vec![0xAA])]),
                Write::Reg(0x11, 0x17, vec![0x01]),
            ]
        );

        k.open().unwrap();
        let w = k.dev.transport().take_writes();
        assert_eq!(
            w,
            [
                Write::Batch(vec![(0x11, 0x1C, vec![0x0A]), (0x11, 0x1F, vec![0x88])]),
                Write::Reg(0x11, 0x17, vec![0x00]),
                tr(0x23)[0].clone(),
                tr(0x23)[1].clone(),
                tw(0x23, 0x00),
            ]
        );
        k.tune(1_318_000_000).unwrap();
        let w = k.dev.transport().take_writes();
        assert_eq!(w[0], Write::Batch(vec![(0x11, 0x1C, vec![0x8A]), (0x11, 0x1F, vec![0xAA])]));
        // the first tune after the init: 0x26 <- 0x80, then 0x23 / 0x22
        assert_eq!(w[1..3], tr(0x26));
        assert_eq!(w[3], tw(0x26, 0x80));
        assert!(!w.contains(&tw(0x24, 0x00)), "no 0x24 write");
        assert!(w.contains(&tw(0x21, 0x93)));
        let n = w.len();
        assert_eq!(w[n - 1], Write::Batch(vec![(0x11, 0x1C, vec![0x0A]), (0x11, 0x1F, vec![0x88])]));
        k.close().unwrap();
        let w = k.dev.transport().take_writes();
        assert_eq!(w.len(), 3 + 2, "{w:?}");
        assert_eq!(w[4], Write::Reg(0x11, 0x17, vec![0x01]));
    }

    #[test]
    fn flags_bit5_configures_at_open_and_powers_down_at_close() {
        for driver in [VendorDriver::Series, VendorDriver::Dth30] {
            let mut k = kit(driver, 0x20);
            k.init().unwrap();
            assert_eq!(k.dev.transport().take_writes(), vec![], "Init only powers down");
            k.open().unwrap();
            let w = k.dev.transport().take_writes();
            assert_eq!(w[0], Write::Reg(0x11, 0x01, vec![0x90]));
            assert!(!w.contains(&Write::Reg(0x11, 0x17, vec![0x00])), "no wake");
            k.close().unwrap();
            assert_eq!(k.dev.transport().take_writes(), vec![], "Close only powers down");
        }
    }

    #[test]
    fn series_booster_on_the_selected_fixed_chip() {
        let regs = |sel: u8| [(0x10, 0x0E, sel), (0x10, 0x1E, 0xDF), (0x11, 0x1C, 0x0A)];
        let mk = |sel| {
            Tc9052xSatKit::new(
                Device::new(RegMock::with_regs(&regs(sel))),
                desc("TC90532_MTV600", 0x402, 0),
                VendorDriver::Series,
            )
            .unwrap()
        };
        let mut k = mk(0xF1);
        assert!(k.set_booster(true).unwrap());
        assert_eq!(k.dev.transport().take(), vec![Op::R(0x10, 0x0E), Op::R(0x10, 0x1E), Op::W(0x10, 0x1E, 0x3F)]);
        assert!(k.set_booster(false).unwrap());
        assert_eq!(k.dev.transport().reg(0x10, 0x1E), 0x3F, "bit4 is not cleared");
        let mut k = mk(0x07);
        assert!(k.set_booster(true).unwrap());
        assert_eq!(k.dev.transport().take(), vec![Op::R(0x10, 0x0E), Op::R(0x11, 0x1C), Op::W(0x11, 0x1C, 0x0E)]);
        assert!(k.set_booster(false).unwrap());
        assert_eq!(k.dev.transport().reg(0x11, 0x1C), 0x0A);
        // the read-back looks at this demodulator's 0x1C bits 1:0
        assert!(!k.booster().unwrap());
        for sel in [0x00, 0x02, 0x06] {
            let mut k = mk(sel);
            assert!(!k.set_booster(true).unwrap());
            assert_eq!(k.dev.transport().take(), vec![Op::R(0x10, 0x0E)]);
        }
    }

    #[test]
    fn dth30_has_no_booster() {
        let mut k = kit(VendorDriver::Dth30, 0);
        assert!(!k.set_booster(true).unwrap());
        assert!(!k.booster().unwrap());
        assert_eq!(k.dev.transport().take(), vec![]);
    }

    #[test]
    fn lnb_power_follows_the_model() {
        let mut k = kit(VendorDriver::Series, 0);
        assert!(k.lnb_power(150).unwrap());
        assert_eq!(k.dev.transport().take(), vec![]);
        let mut d = desc("TC90522_MTV600", 0x402, 0);
        d.model = "h70b".into();
        let mut k = Tc9052xSatKit::new(Device::new(RegMock::default()), d, VendorDriver::Dth30).unwrap();
        let _ = k.lnb_power(110).unwrap();
        let ops = k.dev.transport().take();
        assert!(!ops.is_empty() && ops.iter().all(|op| matches!(op, Op::Other)), "{ops:?}");
    }

    #[test]
    fn out_of_range_if_fails_after_the_ts_stop() {
        let mut k = kit(VendorDriver::Series, 0);
        k.init().unwrap();
        k.open().unwrap();
        k.dev.transport().take_writes();
        assert!(k.tune(2_300_000_000).is_err());
        let w = k.dev.transport().take_writes();
        assert_eq!(w, [Write::Batch(vec![(0x11, 0x1C, vec![0x8A]), (0x11, 0x1F, vec![0x2A])])]);
        assert_eq!(k.last_if_hz(), None);
    }
}
