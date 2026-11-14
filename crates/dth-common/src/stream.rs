//! Stream settings shared by both families and their front ends (the keys
//! named in the comments are the BonDriver ini's, docs/30).

/// How the TS is requested from the bridge (`[Stream] Mode`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamModeCfg {
    /// `raw`: full transport stream as received (PSI clear, ES MULTI2-scrambled
    /// as broadcast). The host descrambles with its own B-CAS reader.
    Raw,
    /// `device`: the bridge's own mode (what the original driver used): the
    /// selected program's PIDs are pushed to the device, which descrambles
    /// with the built-in B-CAS card and wraps the output in the link
    /// encryption of docs/22, undone here when `device_decrypt` is set.
    Device,
}

/// Which program's PID list is pushed to the device in `device` mode
/// (`[Stream] Program`). The bridge keeps one list at a time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceProgram {
    /// `first`: the first program of the PAT (lowest program number).
    First,
    /// `all`: push every program's list in turn. The device descrambles one
    /// program per tuning and reads only the program number of the list
    /// (docs/20 §6), so this ends up with the last program of the PAT.
    All,
    /// A program (service) number.
    Id(u16),
}

impl DeviceProgram {
    /// `first` / `all` / a decimal or `0x` hex service id.
    pub fn parse(s: &str) -> Option<DeviceProgram> {
        let s = s.trim();
        match s.to_ascii_lowercase().as_str() {
            "" | "first" | "auto" => Some(DeviceProgram::First),
            "all" => Some(DeviceProgram::All),
            _ => parse_num(s).map(|n| DeviceProgram::Id(n as u16)),
        }
    }
}

/// Platform default of [`TunerConfig::read_depth`].
pub fn default_read_depth() -> usize {
    if cfg!(windows) { 4 } else { 16 }
}

/// Decimal or `0x`-prefixed hexadecimal.
pub fn parse_num(s: &str) -> Option<u64> {
    let s = s.trim();
    if let Some(h) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(h, 16).ok()
    } else {
        s.parse().ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn program_parses() {
        assert_eq!(DeviceProgram::parse("first"), Some(DeviceProgram::First));
        assert_eq!(DeviceProgram::parse("ALL"), Some(DeviceProgram::All));
        assert_eq!(DeviceProgram::parse("0x400"), Some(DeviceProgram::Id(0x400)));
        assert_eq!(DeviceProgram::parse("1024"), Some(DeviceProgram::Id(1024)));
        assert_eq!(DeviceProgram::parse("x"), None);
    }
}
