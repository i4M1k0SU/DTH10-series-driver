//! The log line format of the `dth` command and server, the BonDriver DLL
//! and the diagnostic tools: env_logger's own layout, but stamped with the
//! local time and its offset from UTC instead of UTC, to the millisecond:
//!
//! ```text
//! [2026-11-15T21:03:12.345+09:00 INFO  dth::serve] ...
//! ```
//!
//! The offset is the operating system's for the moment of the line
//! (GetTimeZoneInformation on Windows, localtime_r(3), and so `TZ` and
//! `/etc/localtime`, elsewhere), so a change to or from summer time shows in
//! the lines that follow it. Where no time zone is set (a chroot without
//! `/etc/localtime`) the lines carry `+00:00`. The date is worked out here
//! from the offset, so no time zone database is needed.

use std::io::Write;
use std::time::{SystemTime, UNIX_EPOCH};

use env_logger::Builder;

/// Set the format of `builder` to the one described above.
pub fn local_time(builder: &mut Builder) -> &mut Builder {
    builder.format(|buf, record| {
        let style = buf.default_level_style(record.level());
        write!(buf, "[{} {style}{:<5}{style:#}", now(), record.level())?;
        match record.target() {
            "" => {}
            target => write!(buf, " {target}")?,
        }
        writeln!(buf, "] {}", record.args())
    })
}

/// The local time now, as the log lines carry it.
pub fn now() -> String {
    let unix_ms = match SystemTime::now().duration_since(UNIX_EPOCH) {
        Ok(d) => d.as_millis() as i64,
        Err(e) => -(e.duration().as_millis() as i64),
    };
    stamp(unix_ms, local_offset_secs())
}

/// The Unix time `unix_ms` (milliseconds) at `offset_secs` east of UTC, to
/// the millisecond, with the offset: `2026-11-15T21:03:12.345+09:00`
/// (RFC 3339).
fn stamp(unix_ms: i64, offset_secs: i32) -> String {
    let local_ms = unix_ms + i64::from(offset_secs) * 1000;
    let (y, mo, d) = civil_from_days(local_ms.div_euclid(86_400_000));
    let ms = local_ms.rem_euclid(86_400_000);
    let (h, mi, s, ms) = (ms / 3_600_000, ms / 60_000 % 60, ms / 1000 % 60, ms % 1000);
    let sign = if offset_secs < 0 { '-' } else { '+' };
    let off_min = offset_secs.unsigned_abs() / 60;
    format!("{y:04}-{mo:02}-{d:02}T{h:02}:{mi:02}:{s:02}.{ms:03}{sign}{:02}:{:02}", off_min / 60, off_min % 60)
}

/// The proleptic Gregorian date (year, month, day) of a count of days since
/// 1970-01-01 (Howard Hinnant's `civil_from_days`).
fn civil_from_days(days: i64) -> (i64, u32, u32) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097); // [0, 146096]
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365; // [0, 399]
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // [0, 365]
    let mp = (5 * doy + 2) / 153; // [0, 11], March first
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = yoe + era * 400 + i64::from(m <= 2);
    (y, m, d)
}

/// The local offset from UTC in seconds (east positive) now, the moment of
/// a log line.
fn local_offset_secs() -> i32 {
    cfg_select! {
        unix => {
            // SAFETY: time(NULL) only returns the time; localtime_r only
            // writes the `tm` it is given, and an all-zero `tm` is a valid
            // value of the plain C struct.
            let t = unsafe { libc::time(std::ptr::null_mut()) };
            let mut tm: libc::tm = unsafe { std::mem::zeroed() };
            if unsafe { libc::localtime_r(&t, &mut tm) }.is_null() {
                return 0;
            }
            // Less than a day either way, so it fits.
            tm.tm_gmtoff as i32
        }
        windows => {
            use windows_sys::Win32::System::Time::{GetTimeZoneInformation, TIME_ZONE_INFORMATION};
            // What GetTimeZoneInformation returns: no summer time in this
            // zone, standard time now, summer time now.
            const UNKNOWN: u32 = 0;
            const STANDARD: u32 = 1;
            const DAYLIGHT: u32 = 2;
            // SAFETY: the call only fills the struct it is given; an all-zero
            // TIME_ZONE_INFORMATION is a valid value of the plain C struct.
            let mut tzi: TIME_ZONE_INFORMATION = unsafe { std::mem::zeroed() };
            // The bias is in minutes, UTC = local + bias.
            let bias = match unsafe { GetTimeZoneInformation(&mut tzi) } {
                UNKNOWN => tzi.Bias,
                STANDARD => tzi.Bias + tzi.StandardBias,
                DAYLIGHT => tzi.Bias + tzi.DaylightBias,
                _ => return 0,
            };
            -bias * 60
        }
        _ => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_stamp_is_local_time_with_its_offset() {
        // 2026-11-15T12:03:12.345Z
        let ts = 1_794_744_192_345;
        assert_eq!(stamp(ts, 9 * 3600), "2026-11-15T21:03:12.345+09:00");
        assert_eq!(stamp(ts, 0), "2026-11-15T12:03:12.345+00:00");
        assert_eq!(stamp(ts, -(5 * 3600 + 30 * 60)), "2026-11-15T06:33:12.345-05:30");
        // a day boundary is crossed in local time, not in UTC
        assert_eq!(stamp(1_794_772_800_000, 9 * 3600), "2026-11-16T05:00:00.000+09:00");
        assert_eq!(stamp(0, -3600), "1969-12-31T23:00:00.000-01:00");
        assert_eq!(stamp(951_868_799_999, 0), "2000-02-29T23:59:59.999+00:00");
        assert_eq!(stamp(951_868_799_999, 1), "2000-03-01T00:00:00.999+00:00");
    }

    #[test]
    fn every_day_of_four_centuries_matches_jiff() {
        let epoch = jiff::civil::date(1970, 1, 1);
        for days in -146_097..146_097 {
            let d = epoch.checked_add(jiff::Span::new().days(days)).expect("in range");
            let want = (i64::from(d.year()), d.month() as u32, d.day() as u32);
            assert_eq!(civil_from_days(days), want, "{days}");
        }
    }

    #[test]
    fn now_reads_back_as_the_moment_it_was_taken() {
        use jiff::Timestamp;
        let before = Timestamp::now();
        let s = now();
        let after = Timestamp::now();
        let back: Timestamp = s.parse().expect("RFC 3339 with an offset");
        // the stamp is cut to the millisecond
        let floor = Timestamp::from_millisecond(before.as_millisecond()).expect("in range");
        assert!(floor <= back && back <= after, "{s}");
        // a real offset: within a day of UTC, whole minutes
        let off = local_offset_secs();
        assert!(off.abs() < 24 * 3600 && off % 60 == 0, "{off}");
    }
}
