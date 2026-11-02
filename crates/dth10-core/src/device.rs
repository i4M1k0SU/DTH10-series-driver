//! Bridge-level operations: device information, GPIO, I2C, stream control.
//!
//! This corresponds to the I2C-bus and GPIO methods of the original driver's
//! bridge objects plus the commands it sends at start-up (device information,
//! firmware version, device start).

use core::time::Duration;
use std::thread::sleep;

use crate::error::{Error, Result};
use crate::protocol::{self, UsbControl, cmd};
use crate::tlv::{self, DeviceInformation, TunerDesc};

/// Byte 0 of the stream-enable payload, see [`Device::stream_enable`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StreamMode {
    /// What the original driver sends: PSI + 1seg only, device-encrypted (PBDA style).
    EncryptedPartial = 0,
    /// Complete transport stream, unencrypted by the device.
    RawFullTs = 1,
}

/// Maximum size of the I2C batch buffer (the original driver's limit, checked when queueing).
pub const I2C_BATCH_MAX: usize = 256;

/// A tuner bridge reachable through a [`UsbControl`] transport.
pub struct Device<T: UsbControl> {
    transport: T,
    batch: Vec<u8>,
    /// Flag bits OR-ed into every bRequest (0x80 selects tuner #1, as the original
    /// driver's per-tuner bus proxy does); not applied when the caller passes flag 0x40.
    base_flags: u8,
}

/// Flag bit: command addresses tuner/filter #1 instead of #0.
pub const FLAG_TUNER1: u8 = 0x80;
/// Flag bit: device-wide command; the tuner-select bit is not added.
pub const FLAG_GLOBAL: u8 = 0x40;

impl<T: UsbControl> Device<T> {
    pub fn new(transport: T) -> Self {
        Device { transport, batch: Vec::with_capacity(I2C_BATCH_MAX), base_flags: 0 }
    }

    /// Address tuner `index` (0 or 1). Tuner #1 commands carry bRequest bit7.
    pub fn with_tuner_index(mut self, index: u8) -> Self {
        self.base_flags = if index == 0 { 0 } else { FLAG_TUNER1 };
        self
    }

    pub fn tuner_index(&self) -> u8 {
        (self.base_flags & FLAG_TUNER1 != 0) as u8
    }

    /// Effective flags for a command (the original per-tuner proxy's semantics).
    pub fn flags(&self, f: u8) -> u8 {
        if f & FLAG_GLOBAL != 0 { f } else { f | self.base_flags }
    }

    fn write(&self, value: u16, index: u16, out: &[u8]) -> Result<()> {
        protocol::send(&self.transport, self.flags(0), value, index, out, None).map(|_| ())
    }

    fn read(&self, flags: u8, value: u16, index: u16, buf: &mut [u8]) -> Result<usize> {
        protocol::send(&self.transport, self.flags(flags), value, index, &[], Some(buf))
    }

    fn read_exact(&self, value: u16, index: u16, buf: &mut [u8]) -> Result<()> {
        let n = self.read(0, value, index, buf)?;
        if n != buf.len() {
            return Err(Error::ShortTransfer { expected: buf.len(), actual: n });
        }
        Ok(())
    }

    pub fn transport(&self) -> &T {
        &self.transport
    }

    pub fn into_transport(self) -> T {
        self.transport
    }

    // ----------------------------------------------------------------- system

    /// Read and parse the device-information blob (first start-up step).
    pub fn read_device_information(&self) -> Result<DeviceInformation> {
        let mut blob = vec![0u8; 0x1000];
        let n = self.read(0, cmd::SYS, cmd::SYS_DEVICE_INFO, &mut blob)?;
        tlv::parse_device_information(&blob[..n])
    }

    /// Raw device-information blob, for diagnostics.
    pub fn read_device_information_raw(&self) -> Result<Vec<u8>> {
        let mut blob = vec![0u8; 0x1000];
        let n = self.read(0, cmd::SYS, cmd::SYS_DEVICE_INFO, &mut blob)?;
        blob.truncate(n);
        Ok(blob)
    }

    /// Firmware version `(major, minor, build)` (second start-up step).
    pub fn firmware_version(&self) -> Result<(u8, u8, u16)> {
        let mut buf = [0u8; 16];
        let n = self.read(0, cmd::SYS, cmd::SYS_FW_VERSION, &mut buf)?;
        tlv::firmware_version(&buf[..n]).ok_or(Error::Protocol("firmware version response too short"))
    }

    /// Device start command: zero-length OUT, wIndex 0x10.
    pub fn start(&self) -> Result<()> {
        self.write(cmd::SYS, cmd::SYS_START, &[])
    }

    // ------------------------------------------------------------------- GPIO

    fn gpio_payload(mask: u32, value: u32) -> [u8; 12] {
        let mut p = [0u8; 12];
        p[4..8].copy_from_slice(&mask.to_le_bytes());
        p[8..12].copy_from_slice(&value.to_le_bytes());
        p
    }

    /// GPIO enable/direction: wIndex 0x1004 `{0, mask, mask}`.
    pub fn gpio_enable(&self, mask: u32) -> Result<()> {
        self.write(cmd::SYS, cmd::SYS_GPIO_ENABLE, &Self::gpio_payload(mask, mask))
    }

    /// GPIO output: wIndex 0x1006 `{0, mask, value}`.
    /// A zero mask is a no-op, as in the original.
    pub fn gpio_write(&self, mask: u32, value: u32) -> Result<()> {
        if mask == 0 {
            return Ok(());
        }
        self.write(cmd::SYS, cmd::SYS_GPIO_SET, &Self::gpio_payload(mask, value))
    }

    /// Set the GPIO output bits in `mask`.
    pub fn gpio_set_bits(&self, mask: u32) -> Result<()> {
        self.write(cmd::SYS, cmd::SYS_GPIO_SET, &Self::gpio_payload(mask, mask))
    }

    /// Clear the GPIO output bits in `mask`.
    pub fn gpio_clear_bits(&self, mask: u32) -> Result<()> {
        self.write(cmd::SYS, cmd::SYS_GPIO_SET, &Self::gpio_payload(mask, 0))
    }

    /// wIndex 0x1010: 16-byte status/GPIO input block.
    pub fn gpio_read(&self) -> Result<[u8; 16]> {
        let mut buf = [0u8; 16];
        self.read(0, cmd::SYS, cmd::SYS_GPIO_READ, &mut buf)?;
        Ok(buf)
    }

    /// Power-up sequence of the TC90517 kit.
    pub fn power_up(&self, desc: &TunerDesc) -> Result<()> {
        let a = desc.gpio_mask_a;
        let b = desc.gpio_mask_b;
        let c = desc.gpio_mask_c;
        self.gpio_enable(a | b | c)?;
        self.gpio_clear_bits(a | b)?;
        sleep(Duration::from_millis(2));
        self.gpio_set_bits(a)?;
        sleep(Duration::from_millis(15));
        self.gpio_set_bits(b)?;
        sleep(Duration::from_millis(10));
        Ok(())
    }

    /// Power-down sequence of the TC90517 kit.
    pub fn power_down(&self, desc: &TunerDesc) -> Result<()> {
        let a = desc.gpio_mask_a;
        let b = desc.gpio_mask_b;
        let c = desc.gpio_mask_c;
        self.gpio_enable(a | b | c)?;
        self.gpio_clear_bits(a | b)
    }

    // -------------------------------------------------------------------- I2C

    /// Raw write to a 7-bit I2C address
    /// (wValue 0x2C00|addr, wIndex 0x30). Flushes a pending batch first.
    pub fn i2c_write(&mut self, addr: u8, data: &[u8]) -> Result<()> {
        self.i2c_batch_flush()?;
        self.write(cmd::I2C_RAW | addr as u16, cmd::I2C_RAW_WRITE, data)
    }

    /// Raw read from a 7-bit I2C address
    /// (wValue 0x2C00|addr, wIndex 0x33). Fails unless exactly `buf.len()` bytes arrive.
    pub fn i2c_read(&mut self, addr: u8, buf: &mut [u8]) -> Result<()> {
        self.i2c_batch_flush()?;
        self.read_exact(cmd::I2C_RAW | addr as u16, cmd::I2C_RAW_READ, buf).map_err(|e| match e {
            Error::ShortTransfer { .. } => Error::I2c { addr, reg: None, msg: "short read" },
            e => e,
        })
    }

    /// Register write (wValue 0x2D00|addr, wIndex reg).
    pub fn i2c_reg_write(&mut self, addr: u8, reg: u8, data: &[u8]) -> Result<()> {
        self.i2c_batch_flush()?;
        self.write(cmd::I2C_REG | addr as u16, reg as u16, data)
    }

    /// Register read (wValue 0x2D00|addr, wIndex reg).
    pub fn i2c_reg_read(&mut self, addr: u8, reg: u8, buf: &mut [u8]) -> Result<()> {
        self.i2c_batch_flush()?;
        self.read_exact(cmd::I2C_REG | addr as u16, reg as u16, buf).map_err(|e| match e {
            Error::ShortTransfer { .. } => Error::I2c { addr, reg: Some(reg), msg: "short read" },
            e => e,
        })
    }

    /// Queue a register write into the batch buffer.
    ///
    /// Entry layout: `[0, len+1, 0, addr<<1, reg, data...]`.
    pub fn i2c_batch_append(&mut self, addr: u8, reg: u8, data: &[u8]) -> Result<()> {
        let need = data.len() + 5;
        if self.batch.len() + need > I2C_BATCH_MAX || data.len() > 0xFE {
            return Err(Error::TooLarge { limit: I2C_BATCH_MAX, actual: self.batch.len() + need });
        }
        self.batch.push(0);
        self.batch.push((data.len() + 1) as u8);
        self.batch.push(0);
        self.batch.push(addr << 1);
        self.batch.push(reg);
        self.batch.extend_from_slice(data);
        Ok(())
    }

    /// Send the queued batch (wValue 0x2B00, wIndex 0).
    pub fn i2c_batch_flush(&mut self) -> Result<()> {
        if self.batch.is_empty() {
            return Ok(());
        }
        let r = self.write(cmd::I2C_BATCH, 0, &self.batch);
        self.batch.clear();
        r
    }

    /// Number of bytes currently queued in the batch buffer.
    pub fn i2c_batch_len(&self) -> usize {
        self.batch.len()
    }

    // ----------------------------------------------------------------- stream

    /// Enable the TS stream (wValue 0xCBDA, wIndex 2, 16 bytes).
    ///
    /// Payload: `{mode, 1, 1, 0xBC, stream_param (u32 LE), 0, 0, stream_id, 0, 0, 0, 0, 0}`.
    /// The original sets `stream_id` to 0 when `desc.id == 0`, otherwise 2, and
    /// `mode` = 0.
    ///
    /// Verified on a DT-H70/U2 (firmware 1.0.18): with `mode` = 0 the device
    /// delivers only PSI + the partial-reception (1seg) layer, re-encrypted
    /// with a device key (scrambling-control bits set to the reserved value 01,
    /// key-info packets inserted); with `mode` = 1 it delivers the complete
    /// transport stream as received from the demodulator, in the clear
    /// (PSI readable, elementary streams still MULTI2-scrambled as broadcast).
    pub fn stream_enable(&self, desc: &TunerDesc) -> Result<()> {
        self.stream_enable_mode(desc, StreamMode::RawFullTs)
    }

    /// Stream enable with an explicit [`StreamMode`].
    pub fn stream_enable_mode(&self, desc: &TunerDesc, mode: StreamMode) -> Result<()> {
        self.stream_enable_byte0(desc, mode as u8)
    }

    /// Stream enable with an arbitrary first payload byte (experiments).
    pub fn stream_enable_byte0(&self, desc: &TunerDesc, byte0: u8) -> Result<()> {
        let mut p = [0u8; 16];
        p[0] = byte0;
        p[1] = 1;
        p[2] = 1;
        p[3] = 0xBC;
        p[4..8].copy_from_slice(&desc.stream_param.to_le_bytes());
        p[10] = if desc.id == 0 { 0 } else { 2 };
        self.write(cmd::CBDA, cmd::CBDA_STREAM_CTRL, &p)
    }

    /// Stream enable with an arbitrary 16-byte payload (experiments).
    pub fn stream_enable_payload(&self, payload: &[u8; 16]) -> Result<()> {
        self.write(cmd::CBDA, cmd::CBDA_STREAM_CTRL, payload)
    }

    /// Disable the TS stream (`{0, 0, 0, 0xBC, 0...}`).
    pub fn stream_disable(&self) -> Result<()> {
        self.stream_disable_byte0(0)
    }

    /// Disable with an arbitrary first payload byte (experiments: whether a
    /// raw-mode stream wants a matching stop).
    pub fn stream_disable_byte0(&self, byte0: u8) -> Result<()> {
        let mut p = [0u8; 16];
        p[0] = byte0;
        p[3] = 0xBC;
        self.write(cmd::CBDA, cmd::CBDA_STREAM_CTRL, &p)
    }

    /// The original driver forwards the `KSPROPERTY_BDA_CA_SET_PROGRAM_PIDS` payload
    /// `BDA_PROGRAM_PID_LIST { ulProgramNumber, ulcPIDs, ulPID[n] }` sent as-is
    /// (wValue 0xCBDA, wIndex 0x1000). Whether the device passes all PIDs when
    /// no table was sent is not known (docs/20 §6).
    pub fn set_program_pids(&self, program_number: u32, pids: &[u16]) -> Result<()> {
        let mut t = Vec::with_capacity(12 + 4 * pids.len());
        t.extend_from_slice(&program_number.to_le_bytes());
        t.extend_from_slice(&(pids.len() as u32).to_le_bytes());
        t.extend_from_slice(&0u32.to_le_bytes());
        for &p in pids {
            t.extend_from_slice(&(p as u32).to_le_bytes());
        }
        self.set_pid_filter_raw(&t)
    }

    /// Send a raw PID-filter table (12 + 4n bytes, `n` = u32 LE at byte 4).
    pub fn set_pid_filter_raw(&self, table: &[u8]) -> Result<()> {
        if table.len() < 12 {
            return Err(Error::Protocol("PID filter table shorter than 12 bytes"));
        }
        let n = u32::from_le_bytes([table[4], table[5], table[6], table[7]]) as usize;
        let expect = 12 + 4 * n;
        if table.len() < expect {
            return Err(Error::Protocol("PID filter table shorter than its count implies"));
        }
        self.write(cmd::CBDA, cmd::CBDA_PID_FILTER, &table[..expect])
    }

    /// Generic CBDA query: IN transfer with `flags`.
    pub fn cbda_query(&self, index: u16, flags: u8, buf: &mut [u8]) -> Result<usize> {
        self.read(flags, cmd::CBDA, index, buf)
    }
}

/// LNB (BS/CS antenna) power control, model "h70" only.
///
/// `volt` is 150 (15 V), 110 (11 V) or 0 (off). Returns `Ok(true)` when the
/// bridge reported the supply as healthy (status bit set after 150 ms) and
/// `Ok(false)` when it was switched off again because the status bit was clear.
pub fn lnb_set<T: UsbControl>(dev: &Device<T>, volt: u32) -> Result<bool> {
    let bits: u32 = match volt {
        150 => 0x2020,
        110 => 0x0020,
        _ => 0,
    };
    dev.gpio_write(0x2020 & !bits, 0)?;
    dev.gpio_write(bits, bits)?;
    if volt == 0 {
        return Ok(true);
    }
    sleep(Duration::from_millis(150));
    let st = dev.gpio_read()?;
    if st[12] & 1 == 0 {
        dev.gpio_write(0x2020, 0)?;
        return Ok(false);
    }
    Ok(true)
}

/// Read back the LNB state, model "h70" only: 0, 110 or 150.
pub fn lnb_get<T: UsbControl>(dev: &Device<T>) -> Result<u32> {
    let st = dev.gpio_read()?;
    let v = u32::from_le_bytes([st[8], st[9], st[10], st[11]]);
    if v & 0x20 == 0 {
        Ok(0)
    } else if v & 0x2000 != 0 {
        Ok(150)
    } else {
        Ok(110)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;

    /// One control transfer as the mock saw it: request, wValue, wIndex, data.
    type Call = (u8, u16, u16, Vec<u8>);

    #[derive(Default)]
    struct Mock {
        log: RefCell<Vec<Call>>,
    }

    impl UsbControl for Mock {
        fn control_out(&self, r: u8, v: u16, i: u16, d: &[u8], _: Duration) -> Result<usize> {
            self.log.borrow_mut().push((r, v, i, d.to_vec()));
            Ok(d.len())
        }
        fn control_in(&self, r: u8, v: u16, i: u16, b: &mut [u8], _: Duration) -> Result<usize> {
            self.log.borrow_mut().push((r, v, i, vec![]));
            Ok(b.len())
        }
    }

    #[test]
    fn batch_then_raw_write_flushes() {
        let mut dev = Device::new(Mock::default());
        dev.i2c_batch_append(0x10, 0x04, &[0x01]).unwrap();
        dev.i2c_batch_append(0x10, 0x1C, &[0x28, 0x83, 0x80, 0x08]).unwrap();
        dev.i2c_write(0x10, &[0xFE, 0xC0]).unwrap();
        let log = dev.transport().log.borrow();
        assert_eq!(log.len(), 2);
        assert_eq!(log[0].0, 0x02);
        assert_eq!(log[0].1, 0x2B00);
        assert_eq!(log[0].2, 0);
        assert_eq!(log[0].3, vec![0, 2, 0, 0x20, 0x04, 0x01, 0, 5, 0, 0x20, 0x1C, 0x28, 0x83, 0x80, 0x08]);
        assert_eq!(log[1].1, 0x2C10);
        assert_eq!(log[1].2, 0x30);
    }

    #[test]
    fn stream_enable_payload() {
        let dev = Device::new(Mock::default());
        let desc = TunerDesc { stream_param: 0x11223344, ..Default::default() };
        dev.stream_enable(&desc).unwrap();
        let log = dev.transport().log.borrow();
        assert_eq!(log[0].1, 0xCBDA);
        assert_eq!(log[0].2, 2);
        assert_eq!(log[0].3, vec![1, 1, 1, 0xBC, 0x44, 0x33, 0x22, 0x11, 0, 0, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn tuner1_sets_bit7_unless_global() {
        let dev = Device::new(Mock::default()).with_tuner_index(1);
        dev.start().unwrap();
        let mut b = [0u8; 4];
        dev.cbda_query(0xCACD, 0x40, &mut b).unwrap();
        let log = dev.transport().log.borrow();
        assert_eq!(log[0].0, 0x82);
        assert_eq!(log[1].0, 0x41);
    }

    #[test]
    fn bare_command_is_zero_length_out() {
        let dev = Device::new(Mock::default());
        dev.start().unwrap();
        let log = dev.transport().log.borrow();
        assert_eq!(log[0], (0x02, 0, 0x10, vec![]));
    }
}
