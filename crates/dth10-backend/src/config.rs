//! Settings of a [`crate::Tuner`]: the DTH10 series' own ones, which the
//! command and the daemon fill from their options (the stream settings shared
//! with the DT-H30/U2 family are [`dth_common::stream`]).

use std::path::PathBuf;

use dth_common::stream::{DeviceProgram, StreamModeCfg, default_read_depth};
use dth10_usb::RecipientMode;

#[derive(Debug, Clone)]
pub struct TunerConfig {
    /// bmRequestType recipient of the vendor requests (`dth --recipient`; the
    /// DLL always uses the platform default, device on Windows, docs/30).
    pub recipient: RecipientMode,
    /// `[Device] Serial`: pick the device whose USB serial number matches (empty = first).
    pub serial: String,
    /// `[Stream] Mode = raw | device`
    pub stream_mode: StreamModeCfg,
    /// `[Stream] Program = first | all | <service id>` (device mode only).
    pub device_program: DeviceProgram,
    /// In device mode, run the CEPM handshake and decrypt the link encryption
    /// (docs/22) so the host receives clear TS. Always on in the DLL and the
    /// daemon; `dth tune --no-decrypt` turns it off for analysis.
    pub device_decrypt: bool,
    /// File keeping the handshake results between runs (`DTH10_<serial>.cepm`
    /// in the state directory, see `dth --state-dir`);
    /// the device keeps a handshake until it loses the session (docs/22 §6c).
    pub cepm_state: Option<PathBuf>,
    /// `[Stream] AppKeys`: the vendor application's `CoBDACom.dll` (supplied by
    /// the user), from which the application certificate, private key and
    /// content IV of the link encryption are read at run time (docs/22 §6).
    /// Required for `device_decrypt`.
    pub app_keys: Option<PathBuf>,
    /// Drop the bridge's TEI+null filler packets (the DLL and the daemon always
    /// do; `dth tune --keep-filler` keeps them for analysis).
    pub drop_filler: bool,
    /// `[Stream] ReadDepth`: bulk IN transfers kept in flight (39936 bytes
    /// each, 19 ms of stream). The host controller fills them without the
    /// reader thread, so the depth is the stall the stream survives: 4 (the
    /// original driver) on Windows, 16 elsewhere, where hubs and older
    /// kernels have shown completion gaps of over 100 ms.
    pub read_depth: usize,
    /// One stream at a time (TVTest, `dth tune`): tuning a channel on one
    /// tuner stops the other tuner's stream and puts its kit to standby.
    /// `false` (`dth serve`): the streams of the two tuners (terrestrial and
    /// satellite, or two terrestrial ones) run independently, each in the
    /// device's own mode (docs/40).
    pub exclusive: bool,
    /// Log the device's interrupt events (card events) while open
    /// (`dth serve`; `DTH10_EVENTS=1` does the same for every front end).
    pub log_events: bool,
    /// `[ISDB-T] LockWaitMs`
    pub ter_lock_wait_ms: u64,
    /// `[ISDB-T] Attenuator`: the terrestrial attenuator (the vendor
    /// application's "use attenuator"), set on every terrestrial tune. Only
    /// the `SMT_LJ101` hw 0x102 kit (the DT-H70/U2) and the `SHARP_8011` hw
    /// 0x102 kit have one; other kits ignore it.
    pub attenuator: bool,
    /// ISDB-T channel bandwidth in Hz: 6 MHz on the Japanese UHF raster (fixed in the DLL).
    pub ter_bandwidth_hz: u32,
    /// Command path (bRequest bit7) for the satellite demodulator/tuner I2C and
    /// GPIO. `None` = as the vendor driver: the tuner #0 path when TLV #1 flags
    /// bit2 is set (the DT-H70/U2, whose tuner #1 path reaches no I2C device,
    /// docs/40), else the proxy path #1 (docs/20 §5.2). `dth tune
    /// --sat-i2c-path` overrides it for diagnostics.
    pub sat_i2c_path: Option<u8>,
    /// Command path (bRequest bit7) of the satellite stream-enable command:
    /// filter #1, which the satellite CEPM handshake is bound to (fixed in the DLL).
    pub sat_stream_path: u8,
    /// Bulk IN endpoint of the satellite TS (0 = auto: the second bulk IN, 0x82; fixed in the DLL).
    pub sat_endpoint: u8,
    /// `[ISDB-S] LnbVoltage = 0 | 110 | 150`: the voltage asked for when the
    /// supply is switched on.
    pub lnb_voltage: u32,
    /// `[ISDB-S] LnbPower`: switch the antenna (LNB) supply on for BS/CS
    /// channels. Off by default: the DT-H70/U2 has no antenna power supply
    /// (the vendor driver's GPIO sequence for model "h70" exists and is sent
    /// when this is on, but powers nothing on this product); the setting is
    /// for family models that have one.
    pub lnb_power: bool,
    /// `[ISDB-S] Booster`: the satellite booster (the vendor application's
    /// "use booster"), set on every satellite tune; separate from the LNB supply.
    pub booster: bool,
    /// `[ISDB-S] LockWaitMs`
    pub sat_lock_wait_ms: u64,
}

impl Default for TunerConfig {
    fn default() -> Self {
        TunerConfig {
            recipient: RecipientMode::platform_default(),
            serial: String::new(),
            stream_mode: StreamModeCfg::Raw,
            device_program: DeviceProgram::First,
            device_decrypt: true,
            cepm_state: None,
            app_keys: None,
            drop_filler: true,
            read_depth: default_read_depth(),
            exclusive: true,
            log_events: false,
            ter_lock_wait_ms: 1200,
            attenuator: false,
            ter_bandwidth_hz: 6_000_000,
            sat_i2c_path: None,
            sat_stream_path: 1,
            sat_endpoint: 0,
            lnb_voltage: 150,
            lnb_power: false,
            booster: false,
            sat_lock_wait_ms: 1500,
        }
    }
}
