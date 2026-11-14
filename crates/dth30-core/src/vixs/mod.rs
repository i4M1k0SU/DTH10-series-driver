//! The ViXS XCode2 ("Camino") chip: registers, memory init, firmware, MIPS commands, I2C master.
//!
//! [`Vixs::open`] brings up the NET2280 and the remote PCI bus
//! ([`crate::net2280::hw_init`]) and reads the board ID; this does not need
//! the firmware. [`Vixs::bring_up`] then repeats the rest of the vendor
//! driver's start-up in order (docs/26 2-vixs-bringup.md §4.2, §8.1):
//!
//! 1. memory init ([`meminit`]), firmware download and MIPS start with the
//!    start-up handshake ([`mips`]), retried as the vendor does;
//! 2. the chip's DMA engine ([`dma`]), the I2C master ([`i2c`]) and the GPIOs
//!    ([`gpio`]);
//! 3. the OTP signature (command 27) and the LED state "not in use".
//!
//! Afterwards [`Vixs::send_cmd`] talks to the firmware ([`cmd`]) and
//! [`Vixs::i2c`] gives the host-driven I2C bus to the front end. The stream
//! commands (TS input set-up, open / flush / close) are in [`stream`].
//!
//! None of this is verified on hardware.

pub mod cmd;
pub mod dma;
pub mod firmware;
pub mod gpio;
pub mod i2c;
pub mod meminit;
pub mod mips;
pub mod mmr;
pub mod stream;

use core::time::Duration;

pub use cmd::{CmdResult, Completion, Heartbeat, M2hHandler, M2hMessage};
pub use firmware::Firmware;
pub use meminit::MemInit;
pub use mips::MipsInfo;
pub use mmr::Chip;
pub use stream::{CasMode, StreamHandle, StreamKind};

use crate::bus::RemotePci;
use crate::net2280::{self, IrqState, Layout};
use crate::{Error, Result};

/// Bring-up options. The defaults are the vendor's.
#[derive(Debug, Clone)]
pub struct Options {
    /// Power_Mode handed to the memory init (vendor default 6).
    pub power_mode: u32,
    /// How many times memory init + MIPS start are tried (vendor: 10).
    pub mips_attempts: u32,
    /// Read the OTP signature at the end of the bring-up, as the vendor does.
    pub read_otp: bool,
    /// How command completion is waited for.
    pub completion: Completion,
    /// How long a command may take.
    pub cmd_timeout: Duration,
    /// The sleep used for all waits (replaceable in tests).
    pub sleep: fn(Duration),
}

impl Default for Options {
    fn default() -> Self {
        Self {
            power_mode: meminit::DEFAULT_POWER_MODE,
            mips_attempts: 10,
            read_otp: true,
            completion: Completion::Interrupt,
            cmd_timeout: Duration::from_secs(3),
            sleep: std::thread::sleep,
        }
    }
}

/// The ViXS chip behind a NET2280.
pub struct Vixs<B: RemotePci> {
    pub(crate) chip: Chip<B>,
    pub(crate) layout: Layout,
    pub(crate) opts: Options,
    mem: Option<MemInit>,
    mips: Option<MipsInfo>,
    pub(crate) ring: Option<cmd::CmdRing>,
    pub(crate) irq: IrqState,
    pub(crate) alive: bool,
    pub(crate) hb: cmd::HeartbeatState,
    pub(crate) pending: Option<cmd::Pending>,
    pub(crate) m2h_handler: Option<M2hHandler<B>>,
    otp: Option<[u32; cmd::OTP_WORDS]>,
    /// The next stream cookie ([`stream`]).
    pub(crate) next_cookie: u32,
    /// Open streams, for the firmware's stream messages.
    pub(crate) streams: Vec<stream::OpenStream>,
}

impl<B: RemotePci> Vixs<B> {
    /// Bring up the NET2280 and the PCI bus and read the board ID.
    /// `pipes` are the endpoint addresses of the device's configuration.
    pub fn open(bus: B, pid: u16, pipes: &[u8], opts: Options) -> Result<Self> {
        let layout = net2280::hw_init(&bus, pid, pipes, opts.sleep)?;
        let board_id = bus.mem_read(layout.mmr + mmr::BOARD_ID)?;
        if board_id == 0xFFFF_FFFF {
            return Err(Error::Protocol("ViXS board-ID register reads all ones"));
        }
        log::info!("ViXS board ID {board_id:08x}");
        let chip = Chip::new(bus, layout.mmr, layout.fb, board_id, opts.sleep);
        Ok(Self::with_chip(chip, layout, opts))
    }

    pub(crate) fn with_chip(chip: Chip<B>, layout: Layout, opts: Options) -> Self {
        Self {
            chip,
            layout,
            opts,
            mem: None,
            mips: None,
            ring: None,
            irq: IrqState::new(),
            alive: false,
            hb: Default::default(),
            pending: None,
            m2h_handler: None,
            otp: None,
            next_cookie: 1,
            streams: Vec::new(),
        }
    }

    /// Initialise the chip and start the firmware (see the module docs).
    pub fn bring_up(&mut self, fw: &Firmware) -> Result<()> {
        // The vendor reads the board ID again here and uses that value from
        // now on.
        self.chip.mmr_read(mmr::BOARD_ID)?;
        let board_id = self.chip.mmr_read(mmr::BOARD_ID)?;
        self.chip.board_id = board_id;
        if board_id & mmr::CHIP_MASK != mmr::CAMINO {
            return Err(Error::Unsupported(format!("ViXS chip {board_id:08x} is not a Camino")));
        }
        self.chip.wait_ticks(3);

        let mut attempt = 0;
        let (mem, info) = loop {
            let mem = meminit::mem_init(&self.chip, board_id, self.opts.power_mode)?;
            let id = self.chip.mmr_read(mmr::BOARD_ID)?;
            log::debug!("board ID after memory init: {id:08x}");
            match mips::start(&mut self.chip, &self.layout, fw, &mem) {
                Ok(info) => break (mem, info),
                Err(e) if self.chip.is_removed() || matches!(e, Error::Usb(_)) => return Err(e),
                Err(e) => {
                    attempt += 1;
                    log::warn!("MIPS start attempt {attempt} failed: {e}");
                    if attempt >= self.opts.mips_attempts {
                        self.stop_chip()?;
                        return Err(e);
                    }
                }
            }
        };
        self.ring = Some(cmd::CmdRing::new(info.cmd_ring_fb, mips::COM_LIST_SIZE)?);
        self.mem = Some(mem);
        self.mips = Some(info);

        dma::init(&mut self.chip)?;
        i2c::init(&mut self.chip)?;
        gpio::init(&mut self.chip)?;
        if self.opts.read_otp {
            self.otp = Some(self.read_otp_signature()?);
        }
        gpio::update_leds(&self.chip, gpio::app_status::NOT_USING)?;
        log::info!("ViXS up: {} MB SDRAM, {} kHz, Power_Mode {}", mem.mem_size >> 20, mem.clock_khz, mem.power_mode);
        Ok(())
    }

    /// What the vendor does after the MIPS could not be started: stop the
    /// chip's clocks.
    fn stop_chip(&mut self) -> Result<()> {
        self.chip.mmr_write_checked(0x000, 0)?;
        self.chip.wait_ticks(1);
        self.chip.mmr_write_checked(0x060, 0x0077_1DF7)?;
        self.chip.wait_ticks(1);
        self.chip.mmr_write_checked(0x05C, 0xF8)?;
        self.chip.wait_ticks(1);
        Ok(())
    }

    /// The full board-ID register value.
    pub fn board_id(&self) -> u32 {
        self.chip.board_id
    }

    /// The board type (low byte of the board ID).
    pub fn board(&self) -> u8 {
        self.chip.board()
    }

    pub fn layout(&self) -> &Layout {
        &self.layout
    }

    pub fn options(&self) -> &Options {
        &self.opts
    }

    pub fn bus(&self) -> &B {
        self.chip.bus()
    }

    pub fn chip(&self) -> &Chip<B> {
        &self.chip
    }

    pub fn chip_mut(&mut self) -> &mut Chip<B> {
        &mut self.chip
    }

    pub fn into_bus(self) -> B {
        self.chip.into_bus()
    }

    /// The memory init's result, once brought up.
    pub fn mem(&self) -> Option<&MemInit> {
        self.mem.as_ref()
    }

    /// What the firmware reported at start-up, once brought up.
    pub fn mips(&self) -> Option<&MipsInfo> {
        self.mips.as_ref()
    }

    /// The command ring, once brought up.
    pub fn ring(&self) -> Option<&cmd::CmdRing> {
        self.ring.as_ref()
    }

    /// The OTP signature read at bring-up.
    pub fn otp_signature(&self) -> Option<&[u32; cmd::OTP_WORDS]> {
        self.otp.as_ref()
    }

    /// I2C channel `channel` (0 or 1) at `khz` (the vendor's user-mode code
    /// uses channel 0 at 400 kHz, its kernel code 100 kHz).
    pub fn i2c(&mut self, channel: u8, khz: u32) -> i2c::I2cMaster<'_, B> {
        i2c::I2cMaster::new(&mut self.chip, channel, khz)
    }

    /// Update the LED / "in use" bits for an application state
    /// ([`gpio::app_status`] bits).
    pub fn set_app_status(&mut self, status: u32) -> Result<()> {
        gpio::update_leds(&self.chip, status)
    }

    /// Write `data` to SDRAM at FB offset `fb_off` (window 0) by DMA,
    /// at most 1 MB.
    pub fn dma_to_fb(&self, fb_off: u32, data: &[u8]) -> Result<()> {
        net2280::dma_write(self.chip.bus(), &self.layout, self.chip.fb + fb_off, data)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::mmr::testutil::*;
    use super::*;
    use crate::bus::mock::{MockPci, Op, Space};

    pub(crate) fn opts(completion: Completion) -> Options {
        Options { completion, cmd_timeout: Duration::from_millis(50), sleep: no_sleep, ..Options::default() }
    }

    pub(crate) fn layout() -> Layout {
        Layout {
            vixs_id: 0,
            bars: vec![],
            fb: FB,
            mmr: MMR,
            endpoints: [0x04, 0x82, 0x83, 0x84],
            fifo: [0x3000_0000, 0x3004_0000, 0x3008_0000, 0x300C_0000],
            ep_cfg: [0; 4],
            dma_offset: 0,
            dma_mapped: true,
        }
    }

    /// A chip whose firmware runs, with the ring of a pool at 1 MB.
    pub(crate) fn running(completion: Completion) -> Vixs<MockPci> {
        let mut v = Vixs::with_chip(chip(0x0300_00B2), layout(), opts(completion));
        v.ring = Some(cmd::CmdRing::new(0x10_0840, mips::COM_LIST_SIZE).unwrap());
        v.alive = true;
        v
    }

    const PIPES: [u8; 9] = [0x0D, 0x8D, 0x0E, 0x8E, 0x8F, 0x04, 0x82, 0x83, 0x84];

    fn device(board_id: u32) -> MockPci {
        let m = MockPci::new();
        m.queue(Space::Cfg, 0x1_0010, &[0xFFF0_0000]);
        m.queue(Space::Cfg, 0x1_0014, &[0xFFFF_0000]);
        m.set(Space::Csr, 0x320, 0x04);
        m.set(Space::Mem, MMR + 0x478, board_id);
        m
    }

    #[test]
    fn open_reads_board_id() {
        let v = Vixs::open(device(0x0300_00B3), 0x0128, &PIPES, opts(Completion::Poll)).unwrap();
        assert_eq!(v.board_id(), 0x0300_00B3);
        assert_eq!(v.board(), 0xB3);
        assert_eq!(v.layout().mmr, MMR);
        assert_eq!(v.bus().ops().last(), Some(&Op::MemRead(MMR + 0x478)));
        assert!(Vixs::open(device(0xFFFF_FFFF), 0x0128, &PIPES, opts(Completion::Poll)).is_err());
    }

    /// Script everything the bring-up reads.
    fn script_bring_up(m: &MockPci) {
        m.set(Space::Mem, MMR + 0x828, 1);
        m.queue(Space::Mem, MMR + 0x874, &[0x1234_5678, 0x8765_4321]);
        super::mips::tests::script_firmware(m);
        m.queue(Space::Mem, MMR + 0x1A2C, &vec![0x80; 180 + 127 * 30]);
        m.queue(Space::Mem, MMR + 0x3030, &[0]);
    }

    #[test]
    fn full_bring_up() {
        let mut v = Vixs::open(device(0x0300_00B4), 0x0129, &PIPES, opts(Completion::Poll)).unwrap();
        script_bring_up(v.bus());
        v.bus().clear();
        v.bring_up(&super::mips::tests::fw(0x200)).unwrap();
        assert_eq!(v.mem().unwrap().mem_size, 0x0400_0000);
        assert_eq!(v.mips().unwrap().cmd_ring_fb, 0x10_0840);
        assert_eq!(v.ring().unwrap().slots, 41);
        assert!(v.otp_signature().is_some());
        let acc = accesses(&v.bus().ops());
        // Phases in order: memory init, MIPS start, DMA, I2C, GPIO, OTP, LED.
        let pos = |a: Acc| acc.iter().position(|x| *x == a).unwrap_or_else(|| panic!("{a:?} missing"));
        let order = [
            pos(Acc::W(0x000, 0xFFFF_FDFF)),
            pos(Acc::W(0x3000, 0x0FFF_FFFF)),
            pos(Acc::W(0x1A2C, 127)),
            pos(Acc::W(0x10A8, 0x67)),
            pos(Acc::W(0x108C, 0x3_0005)),
            pos(Acc::W(0x3020, 27)),
        ];
        assert!(order.array_windows().all(|[a, b]| a < b), "{order:?}");
        // The LED of board 0xB4 ends in "not in use".
        assert_eq!(*writes(&v.bus().ops()).last().unwrap(), Acc::W(0x1088, 0x2_0005));
        // The first two accesses re-read the board ID.
        assert_eq!(&v.bus().ops()[..2], &[Op::MemRead(MMR + 0x478), Op::MemRead(MMR + 0x478)]);
    }

    #[test]
    fn mips_failures_are_retried_then_the_chip_is_stopped() {
        let mut v = Vixs::open(device(0x0300_00B2), 0x0128, &PIPES, opts(Completion::Poll)).unwrap();
        v.opts.mips_attempts = 2;
        v.bus().set(Space::Mem, MMR + 0x828, 1);
        v.bus().queue(Space::Mem, MMR + 0x874, &[0x1234_5678, 0x8765_4321, 0x1234_5678, 0x8765_4321]);
        // The SDRAM write test fails every time.
        v.bus().queue(Space::Mem, FB + 0x44, &[0, 0]);
        v.bus().clear();
        assert!(v.bring_up(&super::mips::tests::fw(0x200)).is_err());
        let w = writes(&v.bus().ops());
        assert_eq!(w.iter().filter(|a| **a == Acc::W(0x000, 0xFFFF_FDFF)).count(), 2);
        assert_eq!(&w[w.len() - 3..], &[Acc::W(0x000, 0), Acc::W(0x060, 0x0077_1DF7), Acc::W(0x05C, 0xF8)]);
        assert!(v.ring().is_none());
    }

    #[test]
    fn not_a_camino() {
        let mut v = Vixs::open(device(0x0100_00B2), 0x0128, &PIPES, opts(Completion::Poll)).unwrap();
        std::assert_matches!(v.bring_up(&super::mips::tests::fw(0x200)), Err(Error::Unsupported(_)));
    }
}
