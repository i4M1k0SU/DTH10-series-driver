//! Stop requests: SIGINT / SIGTERM / SIGHUP on unix (what a recorder sends to
//! end a tuner command), Ctrl-C / Ctrl-Break / console close on Windows.

use std::sync::atomic::{AtomicBool, Ordering};

static STOP: AtomicBool = AtomicBool::new(false);

/// Whether a stop was requested.
pub fn stopping() -> bool {
    STOP.load(Ordering::Relaxed)
}

#[cfg(unix)]
pub fn install() {
    extern "C" fn on_signal(_: libc::c_int) {
        STOP.store(true, Ordering::Relaxed);
    }
    for sig in [libc::SIGINT, libc::SIGTERM, libc::SIGHUP] {
        // SAFETY: the handler only stores an atomic, which is async-signal-safe,
        // and lives for the whole process.
        if unsafe { libc::signal(sig, on_signal as *const () as libc::sighandler_t) } == libc::SIG_ERR {
            log::warn!("cannot install the handler of signal {sig}");
        }
    }
}

#[cfg(windows)]
pub fn install() {
    use windows_sys::Win32::System::Console::{
        CTRL_BREAK_EVENT, CTRL_C_EVENT, CTRL_CLOSE_EVENT, SetConsoleCtrlHandler,
    };

    unsafe extern "system" fn on_ctrl(ctrl: u32) -> windows_sys::core::BOOL {
        if matches!(ctrl, CTRL_C_EVENT | CTRL_BREAK_EVENT | CTRL_CLOSE_EVENT) {
            STOP.store(true, Ordering::Relaxed);
            1
        } else {
            0
        }
    }
    // SAFETY: registers a handler of the documented signature for the process lifetime.
    if unsafe { SetConsoleCtrlHandler(Some(on_ctrl), 1) } == 0 {
        log::warn!("cannot install the Ctrl-C handler");
    }
}
