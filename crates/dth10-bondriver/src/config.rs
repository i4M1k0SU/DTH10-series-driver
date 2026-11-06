//! Settings read from `<DLL name>.ini` next to the DLL (TVTest convention),
//! via `GetPrivateProfileStringW` so the file format matches other BonDrivers.
//! The tuner-related keys map onto [`TunerConfig`] (docs/30).
//!
//! Some [`TunerConfig`] fields are deliberately not read from the ini: each has
//! one working value on every unit of the series, so the DLL always uses the
//! default (the `dth10` command keeps options for most of them, for diagnostics):
//! * `recipient`: the device recipient, the only one WinUSB lets through for
//!   this protocol's `wIndex` arguments (docs/30); the DLL is Windows-only.
//! * `device_decrypt`: on. Without the link decryption the device-mode stream
//!   is AES-CTR ciphertext no player can use, the terrestrial PSI learn falls
//!   back to the raw enable that stops BS/CS until a power cycle, and
//!   satellite channels are refused (docs/22, docs/40).
//! * `drop_filler`: on. The bridge's `47 9F FF 1F` packets (TEI set, PID
//!   0x1FFF) carry nothing and only mark the demodulator output as not valid
//!   yet, about 1.5 s after a tune (docs/40); a host counts them as errors.
//! * `sat_i2c_path`: from the device's own descriptor, as the vendor driver
//!   does (TLV #1 flags bit2), so the DLL needs no `I2CPath` key.
//! * `ter_bandwidth_hz`: 6 MHz, the channel width of the UHF 13-62 table.
//! * `sat_stream_path` / `sat_endpoint`: path #1 and the second bulk IN
//!   endpoint (0x82): the firmware's filter #1, which carries the satellite
//!   TS on every model and to which its CEPM handshake is bound (docs/20 §5.2,
//!   docs/21 §5.7, docs/40).
//!
//! * `read_depth`: the server's own default (4 on Windows, the vendor
//!   driver's value). It was a server-wide setting anyway, decided by the DLL
//!   that started the server; `dth10 serve --read-depth` remains for tests.
//!
//! The DLL always works through the server `dth10.exe` in its own folder, at
//! the socket file `dth10.sock` there (loopback TCP when a Unix socket cannot
//! be used), so the former `[Helper] Use` / `Exe` / `Socket` / `Address` keys
//! are gone too (the section is now `[Server]`), and so are `[Stream] CepmState` (the server keeps its
//! handshake cache `DTH10_<serial>.cepm` in the DLL's folder) and
//! `[Stream] AppKeys` (the key file is always `CoBDACom.dll` in the DLL's folder).
//!
//! The ini template shipped with the DLL (`BonDriver_DTH10.ini`) holds every
//! key read here, set to its default; a test keeps the two in step.

use std::path::{Path, PathBuf};

use dth10_tuner::config::{DeviceProgram, StreamModeCfg, TunerConfig, parse_num};
use dth10_tuner::proto::TunerChoice;

use crate::client::ServerConfig;
use windows_sys::Win32::System::LibraryLoader::{
    GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS, GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, GetModuleFileNameW,
    GetModuleHandleExW,
};
use windows_sys::Win32::System::WindowsProgramming::GetPrivateProfileStringW;

#[derive(Debug, Clone)]
pub struct Config {
    pub ini_path: Option<PathBuf>,
    /// The folder of the DLL (the server executable and its log live there).
    pub dll_dir: Option<PathBuf>,
    /// `[Device]`, `[Stream]`, `[ISDB-T]`, `[ISDB-S]` keys.
    pub tuner: TunerConfig,
    /// `[Device] Spaces`: the tuning spaces this DLL shows, as indices of
    /// `channels::space_name` (0 = terrestrial, 1 = BS, 2 = CS110); all by default.
    pub spaces: Vec<u32>,
    /// `[Device] Tuner`: the terrestrial tuner this DLL asks for on a unit with
    /// two (`0`, `1`; `auto` or empty = the first free one). Units with one
    /// terrestrial tuner always use tuner 0.
    pub ter_tuner: TunerChoice,
    /// `[Server]` keys.
    pub server: ServerConfig,
    /// `[Log] Level` (`error|warn|info|debug|trace`), `[Log] File`
    pub log_level: String,
    pub log_file: String,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            ini_path: None,
            dll_dir: None,
            // Device mode: the DLL shows BS/CS by default, and one raw tune of the
            // terrestrial stream would stop them until a power cycle (docs/40).
            tuner: TunerConfig { stream_mode: StreamModeCfg::Device, ..TunerConfig::default() },
            spaces: vec![0, 1, 2],
            ter_tuner: TunerChoice::Auto,
            server: ServerConfig::default(),
            log_level: "warn".into(),
            log_file: String::new(),
        }
    }
}

/// `GR,BS,CS` and the like (also 地デジ / T / CS110) to space indices; empty = all.
pub fn parse_spaces(s: &str) -> Vec<u32> {
    let mut v: Vec<u32> = s
        .split(|c: char| c == ',' || c == ';' || c.is_whitespace())
        .filter(|w| !w.is_empty())
        .filter_map(|w| match w.to_ascii_uppercase().as_str() {
            "GR" | "T" | "UHF" | "ISDB-T" | "地デジ" => Some(0),
            "BS" => Some(1),
            "CS" | "CS110" => Some(2),
            _ => None,
        })
        .collect();
    v.sort_unstable();
    v.dedup();
    if v.is_empty() { vec![0, 1, 2] } else { v }
}

fn wide(s: &str) -> Vec<u16> {
    s.encode_utf16().chain(std::iter::once(0)).collect()
}

/// Path of the DLL that contains `addr`.
pub fn module_path(addr: *const ()) -> Option<PathBuf> {
    let mut hmod = std::ptr::null_mut();
    // SAFETY: plain Win32 calls with valid out-pointers.
    unsafe {
        if GetModuleHandleExW(
            GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            addr as *const u16,
            &mut hmod,
        ) == 0
        {
            return None;
        }
        let mut buf = vec![0u16; 1024];
        let n = GetModuleFileNameW(hmod, buf.as_mut_ptr(), buf.len() as u32) as usize;
        if n == 0 {
            return None;
        }
        Some(PathBuf::from(String::from_utf16_lossy(&buf[..n])))
    }
}

struct Ini {
    path: Vec<u16>,
}

impl Ini {
    fn get(&self, section: &str, key: &str) -> Option<String> {
        let sec = wide(section);
        let k = wide(key);
        let def = wide("\u{1}");
        let mut buf = vec![0u16; 512];
        // SAFETY: plain Win32 call with valid buffers.
        let n = unsafe {
            GetPrivateProfileStringW(
                sec.as_ptr(),
                k.as_ptr(),
                def.as_ptr(),
                buf.as_mut_ptr(),
                buf.len() as u32,
                self.path.as_ptr(),
            )
        } as usize;
        let s = String::from_utf16_lossy(&buf[..n]);
        if s == "\u{1}" { None } else { Some(s.trim().to_owned()) }
    }

    fn u64(&self, section: &str, key: &str, default: u64) -> u64 {
        self.get(section, key).and_then(|s| parse_num(&s)).unwrap_or(default)
    }

    fn bool(&self, section: &str, key: &str, default: bool) -> bool {
        match self.get(section, key).as_deref().map(|s| s.to_ascii_lowercase()) {
            Some(s) if s == "1" || s == "true" || s == "yes" || s == "on" => true,
            Some(s) if s == "0" || s == "false" || s == "no" || s == "off" => false,
            _ => default,
        }
    }
}

impl Config {
    /// Load `<dll>.ini` of the DLL that contains `module_addr`; missing file
    /// or keys keep the defaults.
    pub fn load(module_addr: *const ()) -> Config {
        module_path(module_addr).map_or_else(Config::default, |dll| Config::for_dll(&dll))
    }

    /// The settings of the DLL at `dll`: `<dll>.ini` next to it, the
    /// defaults for whatever it does not set.
    fn for_dll(dll: &Path) -> Config {
        let mut c = Config { dll_dir: dll.parent().map(|d| d.to_path_buf()), ..Config::default() };
        // The application key file is always the vendor application's
        // CoBDACom.dll, put next to the DLL by the user.
        c.tuner.app_keys = dll.parent().map(|d| d.join("CoBDACom.dll"));
        let ini = dll.with_extension("ini");
        if !ini.exists() {
            return c;
        }
        let f = Ini { path: wide(&ini.to_string_lossy()) };
        c.ini_path = Some(ini);
        if let Some(sp) = f.get("Device", "Spaces") {
            c.spaces = parse_spaces(&sp);
        }
        if let Some(t) = f.get("Device", "Tuner") {
            c.ter_tuner = TunerChoice::parse(&t).unwrap_or(TunerChoice::Auto);
        }
        let t = &mut c.tuner;
        t.serial = f.get("Device", "Serial").unwrap_or_default();
        if let Some(m) = f.get("Stream", "Mode") {
            t.stream_mode = match m.to_ascii_lowercase().as_str() {
                "device" | "tuner" | "descramble" => StreamModeCfg::Device,
                _ => StreamModeCfg::Raw,
            };
        }
        if let Some(p) = f.get("Stream", "Program") {
            t.device_program = DeviceProgram::parse(&p).unwrap_or(DeviceProgram::First);
        }
        t.ter_lock_wait_ms = f.u64("ISDB-T", "LockWaitMs", t.ter_lock_wait_ms);
        t.attenuator = f.bool("ISDB-T", "Attenuator", t.attenuator);
        t.lnb_voltage = match f.u64("ISDB-S", "LnbVoltage", t.lnb_voltage as u64) {
            0 => 0,
            11 | 110 => 110,
            15 | 150 => 150,
            _ => 150,
        };
        t.lnb_power = f.bool("ISDB-S", "LnbPower", t.lnb_power);
        t.booster = f.bool("ISDB-S", "Booster", t.booster);
        t.sat_lock_wait_ms = f.u64("ISDB-S", "LockWaitMs", t.sat_lock_wait_ms);
        c.server.idle_exit_secs = f.u64("Server", "IdleExitSec", c.server.idle_exit_secs);
        c.server.start_timeout_secs = f.u64("Server", "StartTimeoutSec", c.server.start_timeout_secs);
        c.log_level = f.get("Log", "Level").unwrap_or(c.log_level);
        c.log_file = f.get("Log", "File").unwrap_or_default();
        c
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dth10_usb::RecipientMode;

    #[test]
    fn spaces_parse() {
        assert_eq!(parse_spaces("GR"), vec![0]);
        assert_eq!(parse_spaces("BS, CS"), vec![1, 2]);
        assert_eq!(parse_spaces("CS110;BS;BS"), vec![1, 2]);
        assert_eq!(parse_spaces(""), vec![0, 1, 2]);
        assert_eq!(parse_spaces("x"), vec![0, 1, 2]);
    }

    /// The DLL `BonDriver_Test.dll` in a folder of its own, with `ini` (if
    /// any) as its ini: the settings it loads and that folder.
    fn load_with(name: &str, ini: Option<&str>) -> (Config, PathBuf) {
        let dir = std::env::temp_dir().join(format!("dth10-bondriver-config-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        let dll = dir.join("BonDriver_Test.dll");
        if let Some(text) = ini {
            std::fs::write(dll.with_extension("ini"), text).expect("write ini");
        }
        let c = Config::for_dll(&dll);
        let _ = std::fs::remove_dir_all(&dir);
        (c, dir)
    }

    /// The settings every unit of the series shares: not in the ini at all.
    fn assert_fixed(t: &TunerConfig) {
        // the device recipient (the DLL is Windows-only), link decryption,
        // filler dropped, 6 MHz, satellite TS on path #1 / auto EP, the
        // satellite I2C path from the device's descriptor, no own cache file
        assert_eq!(t.recipient, RecipientMode::Device);
        assert!(t.device_decrypt && t.drop_filler);
        assert_eq!((t.ter_bandwidth_hz, t.sat_stream_path, t.sat_endpoint), (6_000_000, 1, 0));
        assert_eq!(t.sat_i2c_path, None);
        assert_eq!(t.cepm_state, None);
    }

    /// Without an ini: the defaults, with the key file next to the DLL.
    #[test]
    fn no_ini_gives_the_defaults() {
        let (c, dir) = load_with("none", None);
        let t = &c.tuner;
        assert_eq!(c.ini_path, None);
        assert_eq!(c.spaces, vec![0, 1, 2]);
        assert_eq!(c.ter_tuner, TunerChoice::Auto);
        assert_eq!(
            (t.serial.as_str(), t.stream_mode, t.device_program),
            ("", StreamModeCfg::Device, DeviceProgram::First)
        );
        assert_eq!(t.app_keys, Some(dir.join("CoBDACom.dll")));
        assert_eq!((t.ter_lock_wait_ms, t.sat_lock_wait_ms), (1200, 1500));
        assert_eq!((t.lnb_voltage, t.lnb_power), (150, false));
        assert_eq!((t.attenuator, t.booster), (false, false));
        assert_eq!((c.server.idle_exit_secs, c.server.start_timeout_secs), (5, 5));
        assert_eq!((c.log_level.as_str(), c.log_file.as_str()), ("warn", ""));
        assert_fixed(t);
    }

    /// The ini template shipped with the DLL sets every key to its default:
    /// loading it gives the same settings as no ini at all.
    #[test]
    fn shipped_template_is_the_defaults() {
        let template = include_str!("../BonDriver_DTH10.ini");
        let (c, dir) = load_with("template", Some(template));
        let (d, _) = load_with("template-none", None);
        let (t, u) = (&c.tuner, &d.tuner);
        assert_eq!(c.ini_path, Some(dir.join("BonDriver_Test.ini")));
        assert_eq!(c.spaces, d.spaces);
        assert_eq!(c.ter_tuner, d.ter_tuner);
        assert_eq!(
            (t.serial.as_str(), t.stream_mode, t.device_program),
            (u.serial.as_str(), u.stream_mode, u.device_program)
        );
        assert_eq!(t.app_keys, Some(dir.join("CoBDACom.dll")), "the key file next to the DLL");
        assert_eq!((t.ter_lock_wait_ms, t.sat_lock_wait_ms), (u.ter_lock_wait_ms, u.sat_lock_wait_ms));
        assert_eq!((t.lnb_voltage, t.lnb_power), (u.lnb_voltage, u.lnb_power));
        assert_eq!((t.attenuator, t.booster), (u.attenuator, u.booster));
        assert_eq!(
            (c.server.idle_exit_secs, c.server.start_timeout_secs),
            (d.server.idle_exit_secs, d.server.start_timeout_secs)
        );
        assert_eq!((c.log_level.as_str(), c.log_file.as_str()), (d.log_level.as_str(), d.log_file.as_str()));
        assert_fixed(t);
    }

    /// Every key the DLL reads, by section: the shipped template must list
    /// exactly these (no retired key left in it, none missing).
    const KEYS: &[(&str, &str)] = &[
        ("Device", "Serial"),
        ("Device", "Spaces"),
        ("Device", "Tuner"),
        ("Stream", "Mode"),
        ("Stream", "Program"),
        ("ISDB-T", "LockWaitMs"),
        ("ISDB-T", "Attenuator"),
        ("ISDB-S", "LnbPower"),
        ("ISDB-S", "LnbVoltage"),
        ("ISDB-S", "Booster"),
        ("ISDB-S", "LockWaitMs"),
        ("Log", "Level"),
        ("Log", "File"),
        ("Server", "IdleExitSec"),
        ("Server", "StartTimeoutSec"),
    ];

    #[test]
    fn shipped_template_lists_exactly_the_keys_read() {
        let mut section = String::new();
        let mut found: Vec<(String, String)> = Vec::new();
        for line in include_str!("../BonDriver_DTH10.ini").lines().map(str::trim) {
            if let Some(name) = line.strip_prefix('[').and_then(|l| l.strip_suffix(']')) {
                section = name.to_owned();
            } else if !line.starts_with(';')
                && let Some((key, _)) = line.split_once('=')
            {
                found.push((section.clone(), key.trim().to_owned()));
            }
        }
        let mut want: Vec<(String, String)> = KEYS.iter().map(|(s, k)| (s.to_string(), k.to_string())).collect();
        found.sort();
        want.sort();
        assert_eq!(found, want, "the template and the keys the DLL reads differ");
    }

    /// Every key of the current ini, set to something other than its default.
    #[test]
    fn reads_every_current_key() {
        let ini = "[Device]\r\nSerial=SN1\r\nSpaces=BS,CS\r\nTuner=1\r\n\
                   [Stream]\r\nMode=raw\r\nProgram=0x400\r\n\
                   [ISDB-T]\r\nLockWaitMs=2000\r\nAttenuator=1\r\n\
                   [ISDB-S]\r\nLnbVoltage=110\r\nLnbPower=1\r\nBooster=yes\r\nLockWaitMs=2500\r\n\
                   [Log]\r\nLevel=debug\r\nFile=C:\\logs\\bon.log\r\n\
                   [Server]\r\nIdleExitSec=9\r\nStartTimeoutSec=7\r\n";
        let (c, dir) = load_with("keys", Some(ini));
        let t = &c.tuner;
        assert_eq!((t.serial.as_str(), c.spaces.as_slice()), ("SN1", &[1, 2][..]));
        assert_eq!(c.ter_tuner, TunerChoice::Index(1));
        assert_eq!((t.stream_mode, t.device_program), (StreamModeCfg::Raw, DeviceProgram::Id(0x400)));
        assert_eq!(t.app_keys, Some(dir.join("CoBDACom.dll")));
        assert_eq!((t.ter_lock_wait_ms, t.sat_lock_wait_ms), (2000, 2500));
        assert_eq!((t.lnb_voltage, t.lnb_power), (110, true));
        assert_eq!((t.attenuator, t.booster), (true, true));
        assert_eq!((c.log_level.as_str(), c.log_file.as_str()), ("debug", "C:\\logs\\bon.log"));
        assert_eq!((c.server.idle_exit_secs, c.server.start_timeout_secs), (9, 7));
        assert_fixed(t);
    }

    #[test]
    fn tuner_values() {
        for (index, (value, expected)) in [
            ("0", TunerChoice::Index(0)),
            ("1", TunerChoice::Index(1)),
            ("auto", TunerChoice::Auto),
            ("AUTO", TunerChoice::Auto),
            ("", TunerChoice::Auto),
            ("2", TunerChoice::Auto),
            ("x", TunerChoice::Auto),
        ]
        .into_iter()
        .enumerate()
        {
            let ini = format!("[Device]\r\nTuner={value}\r\n");
            let (c, _) = load_with(&format!("tuner-{index}"), Some(&ini));
            assert_eq!(c.ter_tuner, expected, "Tuner={value}");
        }
    }

    #[test]
    fn lnb_voltage_uses_default_for_invalid_values() {
        for (index, (value, expected)) in [
            ("0", 0),
            ("11", 110),
            ("15", 150),
            ("110", 110),
            ("150", 150),
            ("1", 150),
            ("50", 150),
            ("111", 150),
            ("151", 150),
            ("-1", 150),
            ("invalid", 150),
            ("", 150),
            ("18446744073709551616", 150),
        ]
        .into_iter()
        .enumerate()
        {
            let ini = format!("[ISDB-S]\r\nLnbVoltage={value}\r\n");
            let (c, _) = load_with(&format!("lnb-voltage-{index}"), Some(&ini));
            assert_eq!(c.tuner.lnb_voltage, expected, "LnbVoltage={value}");
        }
        let (c, _) = load_with("lnb-voltage-missing", Some("[ISDB-S]\r\nLnbPower=1\r\n"));
        assert_eq!(c.tuner.lnb_voltage, 150);
    }

    /// An ini of an older version that still has retired keys (with values
    /// that would matter) loads with the fixed settings, and its current keys
    /// are still read.
    #[test]
    fn retired_keys_are_ignored() {
        let ini = "[Device]\r\nRecipient=interface\r\nSerial=SN1\r\n\
                   [Stream]\r\nDecrypt=0\r\nDropFiller=0\r\nReadDepth=8\r\nCepmState=C:\\x\\y.cepm\r\nAppKeys=C:\\keys\\other.dll\r\n\
                   [ISDB-T]\r\nBandwidthHz=8000000\r\n\
                   [ISDB-S]\r\nI2CPath=1\r\nStreamPath=0\r\nEndpoint=0x81\r\n\
                   [Helper]\r\nUse=0\r\nExe=other.exe\r\nSocket=elsewhere.sock\r\nAddress=127.0.0.1:1\r\nIdleExitSec=9\r\n";
        let (c, dir) = load_with("retired", Some(ini));
        assert_eq!(c.tuner.serial, "SN1");
        assert_eq!(c.tuner.app_keys, Some(dir.join("CoBDACom.dll")));
        // the old [Helper] section and ReadDepth are not read
        assert_eq!(c.server.idle_exit_secs, 5);
        assert_eq!(c.tuner.read_depth, TunerConfig::default().read_depth);
        assert_fixed(&c.tuner);
    }
}
