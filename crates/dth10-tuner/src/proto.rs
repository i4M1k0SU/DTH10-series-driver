//! What a client (`dth10 stream`, or the BonDriver) and
//! `dth10 serve` (the daemon that holds the device) say to each other over
//! the daemon's socket ([`crate::ipc`]): the client sends one request line,
//! the daemon answers one reply line and, after `ok` to a stream request, the
//! transport stream follows until either side closes the connection.
//!
//! ```text
//! > stream ter=557142857 name=27 program=1024
//! > stream unit=04110199-110822N0326 sat=1318000000 tsid=40f1 program=101 busy=fail client=4120-1 lnb=0 lock_wait=1500
//! > stream sat=1318000000 slot=1 program=first
//! < ok 04110199-110822N0326 ISDB-S 1318000000 Hz C/N 18.6 dB
//! < busy ISDB-S stream of unit 04110199-110822N0326 is held by connection 3
//! < error no unit 04110199-110822N0326
//! > status
//! < ok 04110199-110822N0326 ter=30.1 sat=18.6 ter_ch=27 ter_by=5/4120-1 sat_ch=BS15_0 sat_by=7
//! ```
//!
//! `ter` / `sat` are frequencies in Hz (ISDB-T centre, ISDB-S IF), `tsid` the
//! transport_stream_id in hex, `slot` the relative TS number within the
//! transponder, `name` the channel name for the log, `program` what the
//! device descrambles (`first`, `all` or a service id), `mode` `device`
//! (default) or `raw`. A stream request takes a stream that another client
//! holds over, unless it says `busy=fail`, in which case the answer is `busy`
//! at once; a request whose `client=` id is the one the holder gave replaces
//! that stream even then (it is the same client changing channel). `lnb`
//! (antenna supply: `0` off, `110`, `150` volts; satellite only) and
//! `lock_wait` (ms to wait for lock on this tune) apply to this request's
//! unit and tune only; without them the daemon's own settings apply, so two
//! clients of two units each get their own values.
//! `status` (with an optional `unit=`) answers with the C/N of both systems
//! in dB (`-` when not tuned) and, for each system that is streaming, the
//! channel (`ter_ch=`, `sat_ch=`) and the holder (`ter_by=`, `sat_by=`: the
//! connection id, then `/` and the client id when it gave one); `busy` at
//! the end means a tune is in progress and the C/N values are the last read.
//! The connection ends after the reply. The text form does not depend on the
//! platform, so its tests run everywhere.

use crate::config::parse_num;
use crate::{Channel, System};

/// The longest line either side reads.
pub const MAX_LINE: usize = 512;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Request {
    Stream(StreamRequest),
    Status { unit: Option<String> },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StreamRequest {
    /// The unit, by USB serial number (the only served unit when absent).
    pub unit: Option<String>,
    pub channel: Channel,
    /// `first`, `all` or a service id, as given.
    pub program: String,
    /// Terrestrial only: the full TS as broadcast instead of the device's own mode.
    pub raw: bool,
    /// Fail with `busy` instead of taking the stream over from another client.
    pub busy_fail: bool,
    /// Who asks (the BonDriver gives every DLL instance its own id): a stream
    /// held under the same id is replaced at once even with `busy_fail`.
    pub client: Option<String>,
    /// Antenna (LNB) supply for this tune: 0 = off, else the voltage (110 /
    /// 150); `None` = the daemon's setting. Used by satellite tunes only.
    pub lnb: Option<u32>,
    /// How long this tune waits for lock [ms]; `None` = the daemon's setting.
    pub lock_wait_ms: Option<u64>,
}

impl Request {
    /// The request line, without its newline.
    pub fn to_line(&self) -> String {
        match self {
            Request::Status { unit } => match unit {
                Some(u) => format!("status unit={u}"),
                None => "status".into(),
            },
            Request::Stream(r) => {
                let mut s = String::from("stream");
                if let Some(u) = r.unit.as_ref() {
                    s.push_str(&format!(" unit={u}"));
                }
                let ch = &r.channel;
                match ch.system {
                    System::IsdbT => s.push_str(&format!(" ter={}", ch.frequency_hz)),
                    System::IsdbS => {
                        s.push_str(&format!(" sat={}", ch.frequency_hz));
                        if ch.tsid != 0 {
                            s.push_str(&format!(" tsid={:04x}", ch.tsid));
                        }
                        if let Some(n) = ch.slot {
                            s.push_str(&format!(" slot={n}"));
                        }
                    }
                }
                if !ch.name.is_empty() && !ch.name.contains(char::is_whitespace) {
                    s.push_str(&format!(" name={}", ch.name));
                }
                s.push_str(&format!(" program={}", r.program));
                if r.raw {
                    s.push_str(" mode=raw");
                }
                if r.busy_fail {
                    s.push_str(" busy=fail");
                }
                if let Some(c) = r.client.as_ref() {
                    s.push_str(&format!(" client={c}"));
                }
                if let Some(v) = r.lnb {
                    s.push_str(&format!(" lnb={v}"));
                }
                if let Some(ms) = r.lock_wait_ms {
                    s.push_str(&format!(" lock_wait={ms}"));
                }
                s
            }
        }
    }

    pub fn parse(line: &str) -> Result<Request, String> {
        let mut words = line.split_whitespace();
        let kind = words.next().ok_or("empty request")?;
        if kind == "status" {
            let mut unit = None;
            for word in words {
                match word.split_once('=') {
                    Some(("unit", v)) if !v.is_empty() => unit = Some(v.to_owned()),
                    _ => return Err(format!("{word:?}: unknown")),
                }
            }
            return Ok(Request::Status { unit });
        }
        if kind != "stream" {
            return Err("expected a stream or status request".into());
        }
        let (mut unit, mut ter, mut sat, mut tsid, mut slot, mut name) = (None, None, None, None, None, None);
        let mut program = String::from("first");
        let (mut raw, mut busy_fail) = (false, false);
        let mut client = None;
        let (mut lnb, mut lock_wait_ms) = (None, None);
        for word in words {
            let (key, value) = word.split_once('=').ok_or_else(|| format!("{word:?}: expected key=value"))?;
            match key {
                "unit" if !value.is_empty() => unit = Some(value.to_owned()),
                "ter" => ter = Some(parse_u32(value)?),
                "sat" => sat = Some(parse_u32(value)?),
                "tsid" => {
                    tsid = Some(
                        u16::from_str_radix(value.trim_start_matches("0x"), 16)
                            .map_err(|e| format!("tsid {value:?}: {e}"))?,
                    )
                }
                "slot" => slot = Some(value.parse::<u8>().map_err(|e| format!("slot {value:?}: {e}"))?),
                "name" => name = Some(value.to_owned()),
                "program" if !value.is_empty() => program = value.to_owned(),
                "mode" => {
                    raw = match value {
                        "raw" => true,
                        "device" => false,
                        _ => return Err(format!("mode {value:?}: device or raw")),
                    }
                }
                "busy" => {
                    busy_fail = match value {
                        "fail" => true,
                        "takeover" => false,
                        _ => return Err(format!("busy {value:?}: fail or takeover")),
                    }
                }
                "client" if !value.is_empty() => client = Some(value.to_owned()),
                "lnb" => {
                    lnb = Some(match parse_u32(value)? {
                        v @ (0 | 110 | 150) => v,
                        v => return Err(format!("lnb {v}: 0, 110 or 150")),
                    })
                }
                "lock_wait" => {
                    lock_wait_ms = Some(parse_num(value).ok_or_else(|| format!("lock_wait {value:?}: not a number"))?)
                }
                _ => return Err(format!("{word:?}: unknown")),
            }
        }
        let mut channel = match (ter, sat) {
            (Some(hz), None) => {
                if tsid.is_some() || slot.is_some() {
                    return Err("tsid= / slot= go with sat=".into());
                }
                Channel::terrestrial_hz(hz)
            }
            (None, Some(hz)) => Channel::satellite_hz(hz, tsid),
            _ => return Err("expected ter=HZ or sat=HZ".into()),
        };
        if let Some(n) = slot {
            channel.slot = Some(n);
        }
        if let Some(n) = name {
            channel.name = n;
        }
        Ok(Request::Stream(StreamRequest { unit, channel, program, raw, busy_fail, client, lnb, lock_wait_ms }))
    }
}

fn parse_u32(s: &str) -> Result<u32, String> {
    parse_num(s).and_then(|n| u32::try_from(n).ok()).ok_or_else(|| format!("{s:?}: not a number"))
}

/// The daemon's answer to a request.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Reply {
    /// The request was served; for a stream the TS follows. The text says what.
    Ok(String),
    /// A `busy=fail` stream request for a stream another client holds.
    Busy(String),
    Error(String),
}

impl Reply {
    /// The reply line, without its newline.
    pub fn to_line(&self) -> String {
        let one_line = |s: &str| s.replace(['\r', '\n'], " ");
        match self {
            Reply::Ok(info) => format!("ok {}", one_line(info)),
            Reply::Busy(why) => format!("busy {}", one_line(why)),
            Reply::Error(why) => format!("error {}", one_line(why)),
        }
    }

    pub fn parse(line: &str) -> Reply {
        let line = line.trim();
        if let Some(info) = line.strip_prefix("ok")
            && (info.is_empty() || info.starts_with(' '))
        {
            return Reply::Ok(info.trim().to_owned());
        }
        if let Some(why) = line.strip_prefix("busy ") {
            return Reply::Busy(why.to_owned());
        }
        Reply::Error(line.strip_prefix("error ").unwrap_or(line).to_owned())
    }
}

/// Who streams one system, as a status reply tells it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Holder {
    /// The daemon's connection id of the stream.
    pub conn: u64,
    /// The `client=` id that connection gave, if any.
    pub client: Option<String>,
    /// The channel name of the stream request.
    pub channel: String,
}

/// What a status reply says: `<serial> ter=30.1 sat=- [ter_ch=27 ter_by=5/4120-1]
/// [sat_ch=BS15_0 sat_by=7] [busy]`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Status {
    /// C/N in dB of the terrestrial demodulator (`None` = `-`, not tuned).
    pub ter: Option<f32>,
    /// C/N in dB of the satellite demodulator.
    pub sat: Option<f32>,
    /// Per system (0 = terrestrial, 1 = satellite): the client streaming it.
    pub holders: [Option<Holder>; 2],
    /// A tune was in progress; the C/N values are the last ones read.
    pub busy: bool,
}

impl Status {
    /// The reply's text after `ok`.
    pub fn to_info(&self, serial: &str) -> String {
        let show = |v: Option<f32>| v.map_or("-".to_owned(), |x| format!("{x:.1}"));
        let mut s = format!("{serial} ter={} sat={}", show(self.ter), show(self.sat));
        for (i, h) in self.holders.iter().enumerate() {
            let Some(h) = h else { continue };
            let sys = if i == 0 { "ter" } else { "sat" };
            let ch: String = h.channel.split_whitespace().collect::<Vec<_>>().join("_");
            s.push_str(&format!(" {sys}_ch={} {sys}_by={}", if ch.is_empty() { "?" } else { &ch }, h.conn));
            if let Some(c) = h.client.as_ref() {
                s.push('/');
                s.push_str(c);
            }
        }
        if self.busy {
            s.push_str(" busy");
        }
        s
    }

    /// The status in a reply's text (words it does not know are ignored).
    pub fn parse(info: &str) -> Status {
        let mut st = Status::default();
        let (mut ch, mut by): ([Option<String>; 2], [Option<String>; 2]) = Default::default();
        for word in info.split_whitespace() {
            match word.split_once('=') {
                Some(("ter", v)) => st.ter = v.parse().ok(),
                Some(("sat", v)) => st.sat = v.parse().ok(),
                Some(("ter_ch", v)) => ch[0] = Some(v.to_owned()),
                Some(("sat_ch", v)) => ch[1] = Some(v.to_owned()),
                Some(("ter_by", v)) => by[0] = Some(v.to_owned()),
                Some(("sat_by", v)) => by[1] = Some(v.to_owned()),
                None if word == "busy" => st.busy = true,
                _ => {}
            }
        }
        for i in 0..2 {
            let Some(b) = by[i].take() else { continue };
            let (conn, client) = match b.split_once('/') {
                Some((c, id)) => (c, Some(id.to_owned())),
                None => (b.as_str(), None),
            };
            let Ok(conn) = conn.parse::<u64>() else { continue };
            st.holders[i] = Some(Holder { conn, client, channel: ch[i].take().unwrap_or_default() });
        }
        st
    }

    /// The C/N values alone.
    pub fn cnr(&self) -> (Option<f32>, Option<f32>) {
        (self.ter, self.sat)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stream(unit: Option<&str>, channel: Channel, program: &str, raw: bool, busy_fail: bool) -> Request {
        Request::Stream(StreamRequest {
            unit: unit.map(str::to_owned),
            channel,
            program: program.into(),
            raw,
            busy_fail,
            client: None,
            lnb: None,
            lock_wait_ms: None,
        })
    }

    #[test]
    fn lnb_and_lock_wait_round_trip() {
        let Request::Stream(r) = Request::parse("stream sat=1318000000 client=1-1 lnb=150 lock_wait=2500").unwrap()
        else {
            panic!()
        };
        assert_eq!((r.lnb, r.lock_wait_ms), (Some(150), Some(2500)));
        let line = Request::Stream(r.clone()).to_line();
        assert!(line.ends_with(" client=1-1 lnb=150 lock_wait=2500"), "{line}");
        assert_eq!(Request::parse(&line).unwrap(), Request::Stream(r));
        // absent: the daemon's settings
        let Request::Stream(r) = Request::parse("stream ter=557142857").unwrap() else { panic!() };
        assert_eq!((r.lnb, r.lock_wait_ms), (None, None));
        assert!(Request::parse("stream sat=1318000000 lnb=12").is_err());
        assert!(Request::parse("stream sat=1318000000 lock_wait=x").is_err());
    }

    #[test]
    fn client_id_round_trips() {
        let Request::Stream(r) = Request::parse("stream ter=557142857 busy=fail client=4120-1").unwrap() else {
            panic!()
        };
        assert_eq!((r.busy_fail, r.client.as_deref()), (true, Some("4120-1")));
        let line = Request::Stream(r.clone()).to_line();
        assert!(line.ends_with(" busy=fail client=4120-1"), "{line}");
        assert_eq!(Request::parse(&line).unwrap(), Request::Stream(r));
    }

    #[test]
    fn requests_round_trip() {
        let bs = Channel::parse("BS15_1").unwrap();
        for r in [
            stream(None, Channel::terrestrial(27).unwrap(), "first", false, false),
            stream(Some("04110199-110822N0326"), bs.clone(), "101", false, true),
            stream(None, Channel::satellite_hz(1_613_000_000, Some(0x6020)), "0x12a", false, false),
            stream(None, Channel::terrestrial_hz(557_142_857), "all", true, false),
            Request::Status { unit: None },
            Request::Status { unit: Some("X".into()) },
        ] {
            let line = r.to_line();
            assert_eq!(Request::parse(&line), Ok(r), "{line}");
        }
        assert_eq!(bs.system, System::IsdbS);
    }

    #[test]
    fn request_lines() {
        let Request::Stream(r) =
            Request::parse("stream unit=A sat=1318000000 tsid=0x40F1 program=101 busy=fail").unwrap()
        else {
            panic!()
        };
        assert_eq!(r.unit.as_deref(), Some("A"));
        assert_eq!(
            (r.channel.system, r.channel.frequency_hz, r.channel.tsid, r.channel.slot),
            (System::IsdbS, 1_318_000_000, 0x40F1, None)
        );
        assert_eq!((r.program.as_str(), r.raw, r.busy_fail), ("101", false, true));
        let Request::Stream(r) = Request::parse("stream sat=1318000000").unwrap() else { panic!() };
        assert_eq!((r.channel.tsid, r.channel.slot, r.program.as_str(), r.busy_fail), (0, Some(0), "first", false));
        let Request::Stream(r) = Request::parse("stream ter=557142857 name=27 mode=raw busy=takeover").unwrap() else {
            panic!()
        };
        assert_eq!((r.channel.system, r.channel.name.as_str(), r.raw), (System::IsdbT, "27", true));
        assert_eq!(Request::parse("status"), Ok(Request::Status { unit: None }));
        for bad in [
            "",
            "tune ter=1",
            "stream",
            "stream ter=1 sat=2",
            "stream ter=1 tsid=40f1",
            "stream ter=x",
            "stream sat=1 slot=300",
            "stream ter=1 mode=fast",
            "stream ter=1 busy=maybe",
            "stream ter=1 lnb",
            "stream ter=1 unit=",
            "status ter=1",
        ] {
            assert!(Request::parse(bad).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn replies() {
        assert_eq!(Reply::parse("ok\n"), Reply::Ok(String::new()));
        assert_eq!(
            Reply::parse("ok X ISDB-T 557142857 Hz C/N 30.1 dB\n"),
            Reply::Ok("X ISDB-T 557142857 Hz C/N 30.1 dB".into())
        );
        let e = Reply::Error("no lock\n(C/N 0.0 dB)".into());
        assert_eq!(e.to_line(), "error no lock (C/N 0.0 dB)");
        assert_eq!(Reply::parse(&e.to_line()), Reply::Error("no lock (C/N 0.0 dB)".into()));
        let b = Reply::Busy("held by 3".into());
        assert_eq!(Reply::parse(&b.to_line()), b);
        assert_eq!(Reply::parse("garbage"), Reply::Error("garbage".into()));
        assert_eq!(Reply::parse("okay"), Reply::Error("okay".into()));
        assert_eq!(Status::parse("X ter=30.1 sat=-").cnr(), (Some(30.1), None));
        assert_eq!(Status::parse("X ter=- sat=18.6").cnr(), (None, Some(18.6)));
    }

    #[test]
    fn status_round_trips() {
        let st = Status {
            ter: Some(30.1),
            sat: None,
            holders: [
                Some(Holder { conn: 5, client: Some("4120-1".into()), channel: "27".into() }),
                Some(Holder { conn: 7, client: None, channel: "BS15 0".into() }),
            ],
            busy: true,
        };
        let info = st.to_info("X");
        assert_eq!(info, "X ter=30.1 sat=- ter_ch=27 ter_by=5/4120-1 sat_ch=BS15_0 sat_by=7 busy");
        let back = Status::parse(&info);
        assert_eq!(back.holders[0], st.holders[0]);
        assert_eq!(
            back.holders[1].as_ref().map(|h| (h.conn, h.client.clone(), h.channel.as_str())),
            Some((7, None, "BS15_0"))
        );
        assert!(back.busy);
        assert_eq!(back.cnr(), (Some(30.1), None));
        assert_eq!(Status::parse("X ter=- sat=-"), Status::default());
        assert_eq!(Status::parse("X ter=- sat=- ter_by=nonsense"), Status::default());
    }
}
