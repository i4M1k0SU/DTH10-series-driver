//! The DTH10 series behind [`dth_common`]'s [`Backend`] and [`Session`]:
//! [`Dth10Backend`] finds and opens units, [`KeyStore`] holds the
//! application credentials the link decryption needs.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use dth_common::{
    Backend, Channel, DeviceProgram, Error, LockExt, Request, Result, Session, StreamModeCfg, System, TsQueue, Unit,
};
use dth10_core::Device;
use dth10_core::cepm::AppKeys;
use dth10_core::satellite::SatelliteKit;
use dth10_core::terrestrial::TerrestrialKit;
use dth10_core::tlv::DeviceInformation;
use dth10_usb::UsbTransport;

use crate::{Tuner, TunerConfig};

/// The application key file: `explicit`, `$DTH10_APP_KEYS`, `CoBDACom.dll`
/// in the state directory, `CoBDACom.dll` next to the executable.
pub fn app_keys_path(explicit: Option<&Path>, state_dir: &Path) -> Option<PathBuf> {
    if let Some(p) = explicit {
        return Some(p.to_path_buf());
    }
    if let Some(p) = std::env::var_os("DTH10_APP_KEYS") {
        return Some(PathBuf::from(p));
    }
    let candidates = [
        Some(state_dir.join("CoBDACom.dll")),
        std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.join("CoBDACom.dll"))),
    ];
    candidates.into_iter().flatten().find(|p| p.is_file())
}

/// `<state dir>/DTH10_<serial>.cepm` (the prefix tells the file's product in a
/// folder shared with other tuners' files), with the directory created; `None`
/// when the directory cannot be used.
pub fn state_file_for(dir: &Path, serial: Option<&str>) -> Option<PathBuf> {
    if let Err(e) = std::fs::create_dir_all(dir) {
        log::warn!("state directory {}: {e}; running without the handshake cache", dir.display());
        return None;
    }
    let name: String = serial
        .unwrap_or("device")
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() || c == '-' || c == '_' { c } else { '_' })
        .collect();
    Some(dir.join(format!("DTH10_{name}.cepm")))
}

/// The application credentials (the vendor application's `CoBDACom.dll`):
/// needed for device mode and BS/CS (the link decryption), not for raw
/// terrestrial streams. Read on first use, so the file may be put in place
/// while the daemon runs.
pub struct KeyStore {
    explicit: Option<PathBuf>,
    state_dir: PathBuf,
    loaded: Mutex<Option<Arc<AppKeys>>>,
}

impl KeyStore {
    pub fn new(explicit: Option<PathBuf>, state_dir: PathBuf) -> Self {
        Self { explicit, state_dir, loaded: Mutex::new(None) }
    }

    /// Where the file is (or would be looked for) now.
    pub fn resolve(&self) -> Option<PathBuf> {
        app_keys_path(self.explicit.as_deref(), &self.state_dir)
    }

    /// The credentials, read on first use.
    pub fn get(&self) -> std::result::Result<Arc<AppKeys>, String> {
        let mut l = self.loaded.locked();
        if let Some(k) = l.as_ref() {
            return Ok(k.clone());
        }
        let Some(p) = self.resolve() else {
            return Err(format!(
                "the application key file (the vendor application's CoBDACom.dll) is needed for device mode and BS/CS: put it in {} or give --app-keys",
                self.state_dir.display()
            ));
        };
        let t = Instant::now();
        let k = Arc::new(AppKeys::load(&p).map_err(|e| format!("application key file {}: {e}", p.display()))?);
        log::info!(
            "application credentials read from {} in {:?} ({} IV candidates)",
            p.display(),
            t.elapsed(),
            k.iv_candidates()
        );
        *l = Some(k.clone());
        Ok(k)
    }
}

/// Whether a stream needs the link decryption, hence the credentials.
fn needs_keys(decrypt: bool, raw: bool, system: System) -> bool {
    decrypt && (!raw || system == System::IsdbS)
}

/// The DTH10 series. `template` carries the settings every unit gets (the
/// serial number and the handshake cache are filled in per unit).
pub struct Dth10Backend {
    template: TunerConfig,
    /// Where the handshake cache lives; `None`: no cache.
    cepm_dir: Option<PathBuf>,
    keys: Arc<KeyStore>,
}

impl Dth10Backend {
    pub fn new(template: TunerConfig, cepm_dir: Option<PathBuf>, keys: Arc<KeyStore>) -> Self {
        Self { template, cepm_dir, keys }
    }

    pub fn keys(&self) -> &Arc<KeyStore> {
        &self.keys
    }

    /// Open a device whose transport is already open (a device node given
    /// on the command line).
    pub fn open_transport(&self, t: UsbTransport, queue: TsQueue) -> Result<Box<dyn Session>> {
        let serial = t.serial().map(str::to_owned);
        let mut cfg = self.template.clone();
        cfg.serial = serial.clone().unwrap_or_default();
        cfg.cepm_state = self.cepm_dir.as_deref().and_then(|d| state_file_for(d, serial.as_deref()));
        cfg.app_keys = self.keys.resolve();
        let decrypt = cfg.device_decrypt;
        let mode = cfg.stream_mode;
        let tuner = Tuner::open_with(t, &cfg, queue)?;
        Ok(Box::new(Dth10Session { tuner, keys: self.keys.clone(), decrypt, mode }))
    }

    /// The device information of an open transport, as `info` shows it.
    pub fn describe(t: UsbTransport) -> Result<String> {
        let mut s = String::new();
        let _ = writeln!(s, "model: {} ({:04x})", dth10_core::product_label(t.product_id()), t.product_id());
        let _ = writeln!(s, "serial: {}", t.serial().unwrap_or("-"));
        let _ = writeln!(s, "endpoints: {:?}", t.pipes());
        let dev0 = Device::new(t.clone());
        let di = dev0.read_device_information()?;
        describe_tuner(&mut s, 0, &di);
        match dev0.firmware_version() {
            Ok((a, b, c)) => {
                let _ = writeln!(s, "firmware: {a}.{b}.{c}");
            }
            Err(e) => {
                let _ = writeln!(s, "firmware: {e}");
            }
        }
        if di.desc.flags & 1 != 0 {
            let dev1 = Device::new(t).with_tuner_index(1);
            match dev1.read_device_information() {
                Ok(di1) => describe_tuner(&mut s, 1, &di1),
                Err(e) => {
                    let _ = writeln!(s, "tuner #1: {e}");
                }
            }
        }
        Ok(s)
    }
}

fn describe_tuner(s: &mut String, index: u8, di: &DeviceInformation) {
    let d = &di.desc;
    // what this crate makes of the kit: tuner #1 is a satellite tuner, or a
    // second terrestrial one on the two-tuner models
    let role = if TerrestrialKit::<UsbTransport>::supports(d) {
        if index == 0 { "ISDB-T" } else { "ISDB-T (second terrestrial tuner)" }
    } else if index == 1 && SatelliteKit::<UsbTransport>::supports(d) {
        "ISDB-S"
    } else {
        "not supported"
    };
    let _ = writeln!(
        s,
        "tuner #{index}: kit {:?} model {:?} hw 0x{:x} flags 0x{:x} demod 0x{:02x} tuner 0x{:02x} IF {} Hz: {role}",
        d.kit_name, d.model, d.hw_version, d.flags, d.demod_i2c_addr, d.tuner_i2c_addr, d.if_freq_hz
    );
}

impl Backend for Dth10Backend {
    fn family(&self) -> &'static str {
        "DTH10 series"
    }

    /// Units with a USB serial number (every unit of the series has one).
    fn units(&self) -> Result<Vec<Unit>> {
        Ok(UsbTransport::list_all()?
            .into_iter()
            .filter_map(|d| {
                Some(Unit {
                    name: d.serial?,
                    vendor_id: d.vendor_id,
                    product_id: d.product_id,
                    label: dth10_core::product_label(d.product_id),
                    location: d.location,
                })
            })
            .collect())
    }

    /// Device mode and BS/CS decrypt the link, which takes the application
    /// credentials (read here, so a missing file is reported before the
    /// device is touched); a raw terrestrial stream does not.
    fn check(&self, _name: &str, req: Request) -> std::result::Result<(), String> {
        if needs_keys(self.template.device_decrypt, req.raw, req.system) {
            self.keys.get()?;
        }
        Ok(())
    }

    fn open(&self, name: &str, queue: TsQueue) -> Result<Box<dyn Session>> {
        let t = UsbTransport::open_matching(self.template.recipient, Some(name))?;
        self.open_transport(t, queue)
    }

    fn info(&self, name: &str) -> Result<String> {
        Self::describe(UsbTransport::open_matching(self.template.recipient, Some(name))?)
    }
}

/// An open DTH10 series unit: the [`Tuner`] and the credentials it is given
/// before a tune that decrypts the link.
struct Dth10Session {
    tuner: Tuner,
    keys: Arc<KeyStore>,
    decrypt: bool,
    mode: StreamModeCfg,
}

impl Session for Dth10Session {
    fn set_queue(&mut self, tuner: usize, queue: TsQueue) {
        self.tuner.set_queue(tuner, queue)
    }
    fn has_second_terrestrial(&self) -> bool {
        self.tuner.has_second_terrestrial()
    }
    fn set_stream_mode(&mut self, mode: StreamModeCfg) {
        self.mode = mode;
        self.tuner.set_stream_mode(mode)
    }
    fn set_device_program(&mut self, program: DeviceProgram) {
        self.tuner.set_device_program(program)
    }
    fn set_lnb(&mut self, volt: u32) {
        self.tuner.set_lnb(volt)
    }
    fn set_attenuator(&mut self, on: bool) {
        self.tuner.set_attenuator(on)
    }
    fn set_booster(&mut self, on: bool) {
        self.tuner.set_booster(on)
    }
    fn set_lock_wait_ms(&mut self, system: System, ms: u64) {
        self.tuner.set_lock_wait_ms(system, ms)
    }
    fn set_channel_on(&mut self, tuner: usize, ch: &Channel) -> Result<()> {
        if needs_keys(self.decrypt, self.mode == StreamModeCfg::Raw, ch.system) {
            let k = self.keys.get().map_err(Error::AppKeys)?;
            self.tuner.with_app_keys(k);
        }
        self.tuner.set_channel_on(tuner, ch)
    }
    fn signal_level_of(&mut self, tuner: usize) -> f32 {
        self.tuner.signal_level_of(tuner)
    }
    fn release(&mut self, tuner: usize) {
        self.tuner.release(tuner)
    }
    fn stream_failed(&self, tuner: usize) -> bool {
        self.tuner.stream_failed(tuner)
    }
    fn poll_status(&mut self) -> [Option<f32>; 2] {
        self.tuner.poll_status()
    }
    fn close(&mut self) {
        self.tuner.close()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keys_only_for_link_decryption() {
        assert!(needs_keys(true, false, System::IsdbT));
        assert!(needs_keys(true, true, System::IsdbS));
        assert!(!needs_keys(true, true, System::IsdbT));
        assert!(!needs_keys(false, false, System::IsdbS));
    }

    #[test]
    fn state_file_name() {
        let dir = std::env::temp_dir().join(format!("dth10-backend-test-{}", std::process::id()));
        let p = state_file_for(&dir, Some("SN/1 2")).unwrap();
        assert_eq!(p.file_name().unwrap(), "DTH10_SN_1_2.cepm");
        assert_eq!(state_file_for(&dir, None).unwrap().file_name().unwrap(), "DTH10_device.cepm");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
